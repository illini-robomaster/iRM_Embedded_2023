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

#include "bsp_print.h"
#include "cmsis_os.h"
#include "motor.h"

static bsp::CAN* can = nullptr;
static control::BRTEncoder* encoder = nullptr;

void RM_RTOS_Init(void) {
  print_use_uart(&huart8);
  can = new bsp::CAN(&hcan1);
  encoder = new control::BRTEncoder(can, 0x01);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  while (true) {
    encoder->PrintData();
    osDelay(100);
  }
}
