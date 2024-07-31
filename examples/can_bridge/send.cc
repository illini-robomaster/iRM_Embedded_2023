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

#include "bsp_can_bridge.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"

static bsp::CAN* can = nullptr;
static bsp::CanBridge* send1 = nullptr;
static bsp::CanBridge* send2 = nullptr;

void RM_RTOS_Init(void) {
  print_use_uart(&huart4);
  can = new bsp::CAN(&hcan1, true);
  send1 = new bsp::CanBridge(can, 0x20A, 0x20B);
  send2 = new bsp::CanBridge(can, 0x20A, 0x20C);
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);

  while (true) {
    send1->cmd.id = bsp::VX;
    send1->cmd.data_float = 8980.1;
    send1->TransmitOutput();
    send2->cmd.id = bsp::VX;
    send2->cmd.data_float = 8980.1;
    send2->TransmitOutput();
    osDelay(1000);
    send1->cmd.id = bsp::VY;
    send1->cmd.data_float = -9.2;
    send1->TransmitOutput();
    send2->cmd.id = bsp::VY;
    send2->cmd.data_float = -9.2;
    send2->TransmitOutput();
    send1->cmd.id = bsp::VX;
    send1->cmd.data_float = 999;
    send1->TransmitOutput();
    send2->cmd.id = bsp::VX;
    send2->cmd.data_float = 999;
    send2->TransmitOutput();
    osDelay(1000);
    print("running\r\n");
  }
}
