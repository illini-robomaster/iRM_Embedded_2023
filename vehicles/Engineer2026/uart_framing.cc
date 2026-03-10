/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2025 RoboMaster.                                          *
 *  Illini RoboMaster @ University of Illinois at Urbana-Champaign          *
 *                                                                          *
 *  This program is free software: you can redistribute it and/or modify    *
 *  it under the terms of the GNU General Public License as published by    *
 *  the Free Software Foundation, either version 3 of the License, or       *
 *  (at your option) any later version.                                     *
 *                                                                          *
 *  This program is distributed in the hope that it will be useful,         *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 *  GNU General Public License for more details.                            *
 *                                                                          *
 *  You should have received a copy of the GNU General Public License       *
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.    *
 *                                                                          *
 ****************************************************************************/

#include "uart_framing.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "bsp_print.h"

// ── CRC16-Modbus ─────────────────────────────────────────────────────────────

uint16_t crc16_modbus(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFFu;
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i];
    for (int b = 0; b < 8; ++b) {
      if (crc & 0x0001u)
        crc = (crc >> 1) ^ 0xA001u;  // reflected 0x8005
      else
        crc >>= 1;
    }
  }
  return crc;
}

// ── RX parser ────────────────────────────────────────────────────────────────

bool UartRxParseLine(char* line, float out_targets[6]) {
  // Expect: $payload*CCCC
  char* star = nullptr;
  if (line[0] != '$' || (star = strchr(line + 1, '*')) == nullptr) {
    print("ARM UART FRAME ERR: missing $ or *\r\n");
    return false;
  }

  // CRC field must be valid hex.
  char* end_ptr = nullptr;
  unsigned long rx_crc = strtoul(star + 1, &end_ptr, 16);
  if (end_ptr == star + 1) {
    print("ARM UART BAD CRC FIELD\r\n");
    return false;
  }

  // Recompute CRC over the payload bytes (between '$' and '*').
  size_t   payload_len = (size_t)(star - line - 1);
  uint16_t calc_crc =
      crc16_modbus(reinterpret_cast<const uint8_t*>(line + 1), payload_len);
  if (calc_crc != (uint16_t)rx_crc) {
    print("ARM UART CRC MISMATCH: got %04lX calc %04X\r\n",
          rx_crc, (unsigned)calc_crc);
    return false;
  }

  // Null-terminate payload at '*' so sscanf sees only the CSV.
  *star = '\0';
  int n = sscanf(line + 1, "%f,%f,%f,%f,%f,%f",
                 &out_targets[0], &out_targets[1], &out_targets[2],
                 &out_targets[3], &out_targets[4], &out_targets[5]);
  return n == 6;
}

// ── TX encoder ───────────────────────────────────────────────────────────────

void UartTxSendFeedback(bsp::UART* uart, const float enc[6]) {
  char payload_buf[56];
  int  payload_len = snprintf(payload_buf, sizeof(payload_buf),
                               "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
                               enc[0], enc[1], enc[2], enc[3], enc[4], enc[5]);
  if (payload_len <= 0) return;

  uint16_t tx_crc =
      crc16_modbus(reinterpret_cast<const uint8_t*>(payload_buf), (size_t)payload_len);
  char tx_buf[72];
  int  tx_len = snprintf(tx_buf, sizeof(tx_buf), "$%s*%04X\n",
                          payload_buf, (unsigned)tx_crc);
  if (tx_len > 0)
    uart->Write(reinterpret_cast<const uint8_t*>(tx_buf), (uint32_t)tx_len);
}


