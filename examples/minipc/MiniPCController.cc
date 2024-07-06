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

// Latency test. Use with communication/communicator.py in iRM_Vision_2023 repo
// In the communicator.py, need to set testing = Test.LATENCY for this test
void RM_RTOS_Default_Task(const void* argument) {
  UNUSED(argument);

  auto uart = std::make_unique<CustomUART>(&huart1); 
  uart->SetupRx(50);
  uart->SetupTx(50);

  auto minipc_session = communication::MinipcPort();

  //communication::gimbal_data_t gimbal_data;
  //communication::color_data_t color_data;
  //communication::chassis_data_t chassis_data;
  //communication::selfcheck_data_t selfcheck_data;
  //communication::arm_data_t arm_data;
  communication::selfcheck_data_t selfcheck_data;
  communication::arm_data_t arm_data;

  const communication::status_data_t* status_data;

  uint8_t packet_to_send[minipc_session.MAX_PACKET_LENGTH];
  uint8_t *data;
  uint8_t recv_cmd_id;
  int32_t length;

  while (true) {
    /* wait until rx data is available */
    //led->Display(0xFF0000FF);
    
    // Wait until first packet from minipc.
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & RX_SIGNAL) {
      length = uart->Read(&data);
      minipc_session.ParseUartBuffer(data, length);

      recv_cmd_id = minipc_session.GetCmdId();
      status_data = minipc_session.GetStatus();

      switch (recv_cmd_id) {
        case communication::GIMBAL_CMD_ID:
            // Forward gimbal data
            break;
        case communication::COLOR_CMD_ID:
            // Forward color data
            break;
        case communication::CHASSIS_CMD_ID:
            // Forward gimbal data
            break;
        case communication::SELFCHECK_CMD_ID:
            if (status_data->mode == 1){
                selfcheck_data.mode = 1;
                selfcheck_data.debug_int = status_data->debug_int;

                minipc_session.Pack(packet_to_send, (void*)&selfcheck_data, communication::SELFCHECK_CMD_ID);
                uart->Write(packet_to_send, minipc_session.GetPacketLen(communication::SELFCHECK_CMD_ID));
            } else if (status_data->mode == 2){
                // Respond with ID:
                //  BRD: 129 (129 - 127 = 2)
                //  See repo irm_tele_arm `config.py` for details
                selfcheck_data.mode = 2;
                selfcheck_data.debug_int = 129;

                minipc_session.Pack(packet_to_send, (void*)&selfcheck_data, communication::SELFCHECK_CMD_ID);
                uart->Write(packet_to_send, minipc_session.GetPacketLen(communication::SELFCHECK_CMD_ID));
            }
            break;
        case communication::ARM_CMD_ID:
            arm_data.floats[0] = status_data->floats[0];
            arm_data.floats[1] = status_data->floats[1];
            arm_data.floats[2] = status_data->floats[2];
            arm_data.floats[3] = status_data->floats[3];
            arm_data.floats[4] = status_data->floats[4];
            arm_data.floats[5] = status_data->floats[5];

            minipc_session.Pack(packet_to_send, (void*)&arm_data, communication::ARM_CMD_ID);
            uart->Write(packet_to_send, minipc_session.GetPacketLen(communication::ARM_CMD_ID));
            break;
        default:
                selfcheck_data.mode = status_data->mode;
                selfcheck_data.debug_int = 129;

                minipc_session.Pack(packet_to_send, (void*)&selfcheck_data, communication::SELFCHECK_CMD_ID);
                uart->Write(packet_to_send, minipc_session.GetPacketLen(communication::SELFCHECK_CMD_ID));
            break;
      }

    }
    osDelay(10);
  }
}
