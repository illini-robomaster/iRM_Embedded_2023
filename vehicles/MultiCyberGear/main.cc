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


#include "bsp_print.h"
#include "cmsis_os2.h"
#include "main.h"

const osTimerAttr_t controlTimerAttribute = {
  .name = "controlTimer",
  .attr_bits = 0,
  .cb_mem = nullptr,
  .cb_size = 0,
};

osTimerId_t controlTimer;

void controlTask(void* arg) {
  UNUSED(arg);

  print("%lu\r\n", osKernelGetTickCount());
}

//==================================================================================================
// RM Init
//==================================================================================================

void RM_RTOS_Init(void) {
  print_use_usb();
}

void RM_RTOS_Timers_Init(void) {
  controlTimer = osTimerNew(controlTask, osTimerPeriodic, nullptr, &controlTimerAttribute);
}

//==================================================================================================
// RM Default Task
//==================================================================================================

void RM_RTOS_Default_Task(const void* arg) {
  UNUSED(arg);

  osTimerStart(controlTimer, 2U);
}

//==================================================================================================
// END
//==================================================================================================
