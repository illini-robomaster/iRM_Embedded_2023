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


#include <cstdio>

#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"

const osThreadAttr_t imuTaskAttribute = {.name = "imuTask",
                                         .attr_bits = osThreadDetached,
                                         .cb_mem = nullptr,
                                         .cb_size = 0,
                                         .stack_mem = nullptr,
                                         .stack_size = 512 * 4,
                                         .priority = (osPriority_t)osPriorityRealtime,
                                         .tz_module = 0,
                                         .reserved = 0};
osThreadId_t imuTaskHandle;

void imuTask(void* arg) {
  UNUSED(arg);
}

//==================================================================================================
// RM Init
//==================================================================================================

void RM_RTOS_Init(void) {
  print_use_usb();
}

//==================================================================================================
// RM Thread Init
//==================================================================================================

void RM_RTOS_Threads_Init(void) {
  imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
}

//==================================================================================================
// RM Default Task
//==================================================================================================

void RM_RTOS_Default_Task(const void* arg) {
  UNUSED(arg);

  while (true) {
    osDelay(1);
  }
}

//==================================================================================================
// END
//==================================================================================================
