/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2024 RoboMaster.                                          *
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


#include "bsp_uart.h"
#include "main.h"
#include "sen_0366_dist.h"
#include "cmsis_os.h"
#include "bsp_print.h"
#include "bsp_gpio.h"

static distance::SEN_0366_DIST* sensor = nullptr;



void RM_RTOS_Init(void) {
  print_use_uart(&huart1);
  sensor = distance::SEN_0366_DIST::init(&huart6,0x80);
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);

  print("Begin\r\n");
  while (!sensor->begin()) osDelay(50);


  while (true) {
    osDelay(100);
    sensor->readValue();
      print("distance is: %2f m. \r\n", sensor->distance);
  }
}

