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

#include "bsp_laser.h"
#include "bsp_print.h"
#include "cmsis_os.h"

static bsp::Laser* laser = nullptr;

void RM_RTOS_Init(void) {
  print_use_uart(&huart1);
  laser = new bsp::Laser(LASER_GPIO_Port, LASER_Pin);
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);

  while (true) {
    set_cursor(0, 0);
    clear_screen();
    laser->On();
    print("laser on\r\n");
    osDelay(1000);
    set_cursor(0, 0);
    clear_screen();
    laser->Off();
    print("laser off\r\n");
    osDelay(1000);
  }
}
