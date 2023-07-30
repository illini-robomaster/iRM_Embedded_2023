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

/*
 * A4988 stepper
 * 1B Green
 * 1A Black
 * 2A Red
 * 2B Blue
 * STEP PWM
 * MISO MOSI
 * PB14 PB15
 * DIR  ENABLE
 */

#include "main.h"

#include "bsp_print.h"
#include "cmsis_os.h"
#include "stepper.h"

static control::Stepper* stepper = nullptr;

void RM_RTOS_Init(void) {
  print_use_uart(&huart1);
  stepper = new control::Stepper(&htim1, 1, 1000000, DIR_GPIO_Port, DIR_Pin, ENABLE_GPIO_Port,
                                 ENABLE_Pin);
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);
  unsigned length = 700;
  unsigned speed = 1000;
  for (int i = 0; i < 2; ++i) {
    stepper->Move(control::FORWARD, speed);
    osDelay(length);
    stepper->Move(control::BACKWARD, speed);
    osDelay(length);
  }
  bool direction = true;
  while (true) {
    if (direction) {
      stepper->Move(control::FORWARD, speed);
      osDelay(length);
      stepper->Stop();
    } else {
      stepper->Move(control::BACKWARD, speed);
      osDelay(length);
      stepper->Stop();
    }
    direction = !direction;
    osDelay(500);
  }
}
