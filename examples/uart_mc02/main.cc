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

/**
 * @brief UART10 loopback / echo diagnostic for DM_MC_02 (Engineer 2026 arm link).
 *
 * Hardware
 * --------
 *   huart10  (USART10, PE2=RX / PE3=TX)  ←→ OrangePi /dev/ttyS4  @ 115200 8N1
 *   print()  → USB CDC ACM (/dev/ttyACM0 or /dev/ttyUSB0)
 *
 * What this program does
 * ----------------------
 *   1. Every 500 ms sends a numbered heartbeat line on huart10 so you can
 *      verify TX is reaching the OrangePi (read it with `cat /dev/ttyS4`).
 *   2. Any bytes received on huart10 are:
 *        a. Hex-dumped via print() (visible on USB serial / OpenOCD).
 *        b. Echoed verbatim back on huart10 so the OrangePi can verify RX.
 *   3. A byte counter and per-read statistics are printed every second so
 *      you can tell at a glance whether DMA is delivering data.
 *
 * Quick OrangePi test
 * -------------------
 *   # Terminal 1 – watch MCU heartbeats arriving:
 *   cat /dev/ttyS4
 *
 *   # Terminal 2 – send a test frame and watch the echo:
 *   echo -n '$-18.600,0.000,0.000,0.000,0.000,0.000*3D4A\n' > /dev/ttyS4
 *
 *   # Or use minicom / picocom:
 *   picocom -b 115200 /dev/ttyS4
 */

#include "main.h"

#include <cstdio>
#include <cstring>

#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "usart.h"

// ── CRC16-Modbus (matches uart_framing.cc) ───────────────────────────────────
static uint16_t crc16_modbus(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFFu;
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i];
    for (int b = 0; b < 8; ++b)
      crc = (crc & 1u) ? ((crc >> 1) ^ 0xA001u) : (crc >> 1);
  }
  return crc;
}

// Send a properly-framed "$p0,p1,p2,p3,p4,p5*XXXX\n" feedback frame.
// All positions are 0.0 in this debug stub — keeps the ROS watchdog alive.
static void SendFeedbackFrame(bsp::UART* uart, const float pos[6]) {
  char payload[56];
  int  plen = snprintf(payload, sizeof(payload),
                       "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
                       pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
  if (plen <= 0) return;
  uint16_t crc = crc16_modbus(reinterpret_cast<const uint8_t*>(payload), (size_t)plen);
  char tx[72];
  int  tlen = snprintf(tx, sizeof(tx), "$%s*%04X\n", payload, (unsigned)crc);
  if (tlen > 0)
    uart->Write(reinterpret_cast<const uint8_t*>(tx), (uint32_t)tlen);
}

// ── FreeRTOS signal ──────────────────────────────────────────────────────────
#define RX_SIGNAL (1u << 0)

extern osThreadId_t defaultTaskHandle;

// ── CustomUART: wake the default task on idle-line interrupt ─────────────────
class CustomUART : public bsp::UART {
 public:
  using bsp::UART::UART;

 protected:
  void RxCompleteCallback() override final {
    osThreadFlagsSet(defaultTaskHandle, RX_SIGNAL);
  }
};

// ── Default task ─────────────────────────────────────────────────────────────
void RM_RTOS_Default_Task(const void* argument) {
  UNUSED(argument);

  // Debug output goes to USB CDC so huart10 is exclusively the OrangePi link.
  print_use_usb();

  CustomUART uart10(&huart10);
  uart10.SetupRx(256);
  uart10.SetupTx(256);

  print("UART10 echo test started. huart10 = %p\r\n", (void*)&huart10);

  uint32_t heartbeat_no   = 0;
  uint32_t total_rx_bytes = 0;
  uint32_t last_stats_tick = HAL_GetTick();
  uint32_t last_hb_tick    = HAL_GetTick();
  uint32_t last_fb_tick    = HAL_GetTick();

  // Pseudo positions sent in every feedback frame (all zeros for the debug stub).
  float pseudo_pos[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

  while (true) {
    // ── Feedback frame TX every 100 ms (10 Hz) — keeps ROS watchdog alive ──
    uint32_t now = HAL_GetTick();
    if (now - last_fb_tick >= 100) {
      last_fb_tick = now;
      SendFeedbackFrame(&uart10, pseudo_pos);
    }

    // ── Heartbeat TX every 500 ms ────────────────────────────────────────
    now = HAL_GetTick();
    if (now - last_hb_tick >= 500) {
      last_hb_tick = now;
      char hb[64];
      int  hb_len = snprintf(hb, sizeof(hb), "HB%lu\r\n", (unsigned long)heartbeat_no++);
      uart10.Write(reinterpret_cast<const uint8_t*>(hb), (uint32_t)hb_len);
    }

    // ── Wait up to 50 ms for an RX event, then loop for heartbeat ────────
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, 50);

    if (!(flags & RX_SIGNAL)) continue;  // timeout — just do heartbeat

    // ── Data arrived ─────────────────────────────────────────────────────
    uint8_t* rx_data = nullptr;
    int32_t  rx_len  = uart10.Read(&rx_data);

    if (rx_len <= 0) {
      print("UART10: RX signal but Read() returned %ld\r\n", (long)rx_len);
      continue;
    }

    total_rx_bytes += (uint32_t)rx_len;

    // Hex dump every received chunk.
    print("UART10 RX %ld bytes:", (long)rx_len);
    for (int32_t i = 0; i < rx_len && i < 32; ++i)
      print(" %02X", rx_data[i]);
    if (rx_len > 32) print(" ...");
    print("\r\n");

    // Also print as ASCII (replace non-printable with '.').
    print("         ASCII: \"");
    for (int32_t i = 0; i < rx_len && i < 64; ++i) {
      char c = (char)rx_data[i];
      print("%c", (c >= 0x20 && c < 0x7F) ? c : '.');
    }
    print("\"\r\n");

    // Echo back verbatim so the OrangePi can verify the link end-to-end.
    uart10.Write(rx_data, (uint32_t)rx_len);

    // ── Per-second stats ─────────────────────────────────────────────────
    now = HAL_GetTick();
    if (now - last_stats_tick >= 1000) {
      last_stats_tick = now;
      print("UART10 stats: total_rx=%lu bytes  uptime=%lu ms\r\n",
            (unsigned long)total_rx_bytes, (unsigned long)now);
    }
  }
}
