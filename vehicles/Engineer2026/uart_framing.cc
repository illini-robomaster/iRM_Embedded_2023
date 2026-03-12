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

bool UartRxParseFrame(const uint8_t* buf, float out_targets[6]) {
  if (buf[0] != 0xA5u) {
    print("ARM UART FRAME ERR: bad SOF 0x%02X\r\n", (unsigned)buf[0]);
    return false;
  }
  if (buf[1] != 0x0Cu) {
    print("ARM UART FRAME ERR: bad LEN 0x%02X\r\n", (unsigned)buf[1]);
    return false;
  }

  uint16_t rx_crc   = (uint16_t)buf[14] | ((uint16_t)buf[15] << 8);
  uint16_t calc_crc = crc16_modbus(buf, 14);
  if (rx_crc != calc_crc) {
    print("ARM UART CRC MISMATCH: got %04X calc %04X\r\n",
          (unsigned)rx_crc, (unsigned)calc_crc);
    return false;
  }

  for (int i = 0; i < 6; ++i) {
    int16_t raw = (int16_t)((uint16_t)buf[2 + i * 2] | ((uint16_t)buf[3 + i * 2] << 8));
    out_targets[i] = (float)raw / 100.0f;
  }
  return true;
}

// ── TX encoder ───────────────────────────────────────────────────────────────

void UartTxSendFeedback(bsp::UART* uart, const float enc[6]) {
  uint8_t frame[UART_FRAME_LEN];
  frame[0] = 0xA5u;
  frame[1] = 0x0Cu;
  for (int i = 0; i < 6; ++i) {
    int16_t val       = (int16_t)(enc[i] * 100.0f);
    frame[2 + i * 2] = (uint8_t)(val & 0xFF);
    frame[3 + i * 2] = (uint8_t)((val >> 8) & 0xFF);
  }
  uint16_t crc  = crc16_modbus(frame, 14);
  frame[14]     = (uint8_t)(crc & 0xFF);
  frame[15]     = (uint8_t)((crc >> 8) & 0xFF);
  uart->Write(frame, UART_FRAME_LEN);
}


