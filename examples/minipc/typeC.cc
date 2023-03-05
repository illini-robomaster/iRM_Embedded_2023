/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2022 RoboMaster.                                          *
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

#include "main.h"

#include <cstring>
#include <memory>

#include "bsp_gpio.h"
#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "minipc.h"
#include "rgb.h"

#define RX_SIGNAL (1 << 0)

extern osThreadId_t defaultTaskHandle;

static display::RGB* led = nullptr;

class CustomUART : public bsp::UART {
 public:
  using bsp::UART::UART;

 protected:
  /* notify application when rx data is pending read */
  void RxCompleteCallback() override final { osThreadFlagsSet(defaultTaskHandle, RX_SIGNAL); }
};

void RM_RTOS_Init(void) {
  led = new display::RGB(&htim5, 3, 2, 1, 1000000);
}

void RM_RTOS_Default_Task(const void* argument) {
  UNUSED(argument);

  uint32_t length;
  uint8_t* data;

  auto uart = std::make_unique<CustomUART>(&huart1);  // see cmake for which uart
  uart->SetupRx(50);
  uart->SetupTx(50);

  auto miniPCreceiver = communication::MiniPCProtocol();
  int total_processed_bytes = 0;

  while (true) {
    /* wait until rx data is available */
    led->Display(0xFF0000FF);
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & RX_SIGNAL) {  // unnecessary check
      /* time the non-blocking rx / tx calls (should be <= 1 osTick) */

      // max length of the UART buffer at 150Hz is ~50 bytes
      length = uart->Read(&data);
      total_processed_bytes += length;

      // if read anything, flash red
      led->Display(0xFFFF0000);

      miniPCreceiver.Receive(data, length);
      uint32_t valid_packet_cnt = miniPCreceiver.get_valid_packet_cnt();
      if (valid_packet_cnt > 998) {
        // If at least 99.9% packets are valid, pass
        led->Display(0xFF00FF00);
        osDelay(10000);
      }
      // blue when nothing is received
      led->Display(0xFF0000FF);
    }
  }
}
