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

// Declare pointers to CAN and CanBridge objects
static bsp::CAN* can = nullptr;
static bsp::CanBridge* send = nullptr;

/**
 * @brief Initialize the RTOS for RoboMaster
 *
 * This function initializes the UART for printing,
 * creates a new CAN object with the specified parameters,
 * and creates a new CanBridge object with the specified parameters.
 */
void RM_RTOS_Init(void) {
  print_use_uart(&huart1);
  can = new bsp::CAN(&hcan2, 0x201, false);
  send = new bsp::CanBridge(can, 0x20A, 0x20B);
}

/**
 * @brief Default task for the RTOS
 *
 * This function is the default task for the RTOS.
 * It continuously sends commands to the CanBridge object.
 * The task is delayed by 1000ms after each command.
 *
 * @param arguments The arguments passed to the task. Not used in this function.
 */
void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);

  while (true) {
    // Set the command id to VX and the data to 8980.1, then transmit the command
    send->cmd.id = bsp::VX;
    send->cmd.data_float = 8980.1;
    send->TransmitOutput();
    osDelay(1000);

    // Set the command id to VY and the data to -9.2, then transmit the command
    send->cmd.id = bsp::VY;
    send->cmd.data_float = -9.2;
    send->TransmitOutput();

    // Set the command id to VX and the data to 999, then transmit the command
    send->cmd.id = bsp::VX;
    send->cmd.data_float = 999;
    send->TransmitOutput();
    osDelay(1000);
  }
}