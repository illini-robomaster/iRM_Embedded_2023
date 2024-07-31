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
static bsp::CanBridge* with_chassis = nullptr;
static bsp::CanBridge* with_shooter = nullptr;

void RM_RTOS_Init(void) {
  print_use_uart(&huart1);
  can = new bsp::CAN(&hcan2, false);
  with_chassis = new bsp::CanBridge(can, 0x20A, 0x20B);
  with_shooter = new bsp::CanBridge(can, 0x20A, 0x20C);
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);
  print("sending\n");
  while (true) {
    with_chassis->cmd.id = bsp::VX;
    with_chassis->cmd.data_float = 8980.1;
    with_chassis->TransmitOutput();
    with_shooter->cmd.id = bsp::VX;
    with_shooter->cmd.data_float = 8980.1;
    with_shooter->TransmitOutput();
    osDelay(1000);
    with_chassis->cmd.id = bsp::VY;
    with_chassis->cmd.data_float = -9.2;
    with_chassis->TransmitOutput();
    with_shooter->cmd.id = bsp::VY;
    with_shooter->cmd.data_float = -9.2;
    with_shooter->TransmitOutput();
    with_chassis->cmd.id = bsp::VX;
    with_chassis->cmd.data_float = 999;
    with_chassis->TransmitOutput();
    with_shooter->cmd.id = bsp::VX;
    with_shooter->cmd.data_float = 999;
    with_shooter->TransmitOutput();
    osDelay(1000);
    print("running\r\n");
  }
}
