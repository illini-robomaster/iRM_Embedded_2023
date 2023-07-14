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
static bsp::CanBridge* send = nullptr;

void RM_RTOS_Init(void) {
  print_use_uart(&huart1);
  can = new bsp::CAN(&hcan2, false);
  send = new bsp::CanBridge(can, 0x20A, 0x20B);
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);

  while (true) {
    send->cmd.id = bsp::VX;
    send->cmd.data_float = 8980.1;
    send->TransmitOutput();
    osDelay(1000);
    send->cmd.id = bsp::VY;
    send->cmd.data_float = -9.2;
    send->TransmitOutput();
    send->cmd.id = bsp::VX;
    send->cmd.data_float = 999;
    send->TransmitOutput();
    osDelay(1000);
  }
}
