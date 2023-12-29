/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2023 RoboMaster.                                          *
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
#include "minipc_protocol.h"
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

  auto uart = std::make_unique<CustomUART>(&huart1);  
  uart->SetupRx(50);
  uart->SetupTx(50);

  auto minipc_session = communication::MinipcPort();
  int total_processed_bytes = 0;

  /* To run this test, use Communication/communicator.py in the iRM_Vision_2023 repo
   * Need to set testing=Test.CRC
  **/

  while (true) {
    /* wait until rx data is available */
    led->Display(0xFF0000FF);

    int32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & RX_SIGNAL) {
      /* time the non-blocking rx / tx calls (should be <= 1 osTick) */

      // max length of the UART buffer at 150Hz is ~50 bytes
      length = uart->Read(&data);
      total_processed_bytes += length;
      minipc_session.ParseUartBuffer(data, length);
      uint32_t valid_packet_cnt = minipc_session.GetValidPacketCnt();
      uint8_t packet_to_send[minipc_session.MAX_PACKET_LENGTH];

      // Jetson / PC sends 200Hz valid packets for stress testing
      // For testing script, please see iRM_Vision_2023/Communication/communicator.py
      // For comm protocol details, please see iRM_Vision_2023/docs/comm_protocol.md
      if (valid_packet_cnt == 1000) {
        // Jetson test cases write 1000 packets. Pass
        led->Display(0xFF00FF00);
        osDelay(4000);
        led->Display(0xFFFF0000);
        // after 10 seconds, write 1000 alternating packets to Jetson
        communication::color_data_t color_data;
        uint8_t my_color = 1; // blue
        for (int i = 0; i < 1000; ++i) {
          if (i % 2 == 0) {
            my_color = 1; // blue
          } else {
            my_color = 0; // red
          }
          color_data.my_color = my_color;
          minipc_session.Pack(packet_to_send, (void*)&color_data, communication::COLOR_CMD_ID);
          uart->Write(packet_to_send, minipc_session.GetPacketLen(communication::COLOR_CMD_ID));
          // NOTE: THIS BREAKS WHEN WORKING AT 1000HZ!
          osDelay(2);
        }
      }
      // blue when nothing is received
      led->Display(0xFF0000FF);
    }
    osDelay(2);
  }
}
