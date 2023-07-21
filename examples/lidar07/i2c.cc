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
#include "cmsis_os.h"
#include "lidar07.h"
#include "main.h"

static distance::LIDAR07_IIC* sensor = nullptr;

void RM_RTOS_Init(void) {
  print_use_uart(&huart1);
  sensor = new distance::LIDAR07_IIC(&hi2c2, 0x70, [](uint32_t milli) { osDelay(milli); });
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);

  while (!sensor->IsReady()) osDelay(50);
  print("Ready\r\n");
  while (!sensor->begin()) osDelay(50);
  print("Begin\r\n");
  while (!sensor->startFilter()) osDelay(50);
  print("Start Filter\r\n");

  while (true) {
    set_cursor(0, 0);
    clear_screen();
    while (!sensor->startMeasure()) osDelay(50);
    print("Distance: %.2f m\r\n", sensor->distance / 1000.0);
    osDelay(1000);
  }
}
