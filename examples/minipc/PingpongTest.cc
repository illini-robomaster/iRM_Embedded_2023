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

  auto uart = std::make_unique<CustomUART>(&huart1);  // see cmake for which uart
  uart->SetupRx(50);
  uart->SetupTx(50);

  auto minipc_session = communication::MinipcPort();

  communication::gimbal_data_t gimbal_data;
  communication::color_data_t color_data;
  communication::chassis_data_t chassis_data;

  const communication::status_data_t* status_data;

  gimbal_data.rel_yaw = 100;
  gimbal_data.rel_pitch = 200;
  gimbal_data.debug_int = 50;
  gimbal_data.mode = 1;

  color_data.my_color = 1;

  chassis_data.vx = 20;
  chassis_data.vy = 30;
  chassis_data.vw = 40;

  uint8_t packet_to_send[minipc_session.MAX_PACKET_LENGTH];
  uint8_t *data;
  int32_t length;

  UNUSED(color_data);
  UNUSED(chassis_data);

  while (true) {
    /* wait until rx data is available */
    //led->Display(0xFF0000FF);

    // TX RX test. Use with communication/communicator.py in iRM_Vision_2023 repo
    // In the communicator.py, need to set TESTING_TX_RX = True for this test
    // The value for TESTING_CRC doesn't matter

    // Wait until first packet from minipc.
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & RX_SIGNAL) {
      // When packet received from miniPC, increase rel_pitch by 1 and send back
      length = uart->Read(&data);
      minipc_session.ParseUartBuffer(data, length);
      status_data = minipc_session.GetStatus();
      gimbal_data.rel_pitch = status_data->rel_pitch + 1;
      minipc_session.Pack(packet_to_send, (void*)&gimbal_data, communication::GIMBAL_CMD_ID);
      uart->Write(packet_to_send, minipc_session.GetPacketLen(communication::GIMBAL_CMD_ID));
    }
    osDelay(10);


    // example code for prepare other packets and send
    /*
    uart->Write(packet_to_send, minipc_session.GetPacketLen(communication::GIMBAL_CMD_ID));
    osDelay(1);

    minipc_session.Pack(packet_to_send, (void*)&color_data, communication::COLOR_CMD_ID);
    uart->Write(packet_to_send, minipc_session.GetPacketLen(communication::COLOR_CMD_ID));
    osDelay(1);

    minipc_session.Pack(packet_to_send, (void*)&chassis_data, communication::CHASSIS_CMD_ID);
    uart->Write(packet_to_send, minipc_session.GetPacketLen(communication::CHASSIS_CMD_ID));
    osDelay(1);
    */
  }
}
