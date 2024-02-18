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

#include "main.h"

#include "bsp_gpio.h"
#include "cmsis_os.h"
static bsp::GPIO* led = nullptr;

void RM_RTOS_Init(void) {
 led = new bsp::GPIO(LED_GPIO_Port, LED_Pin);
 HAL_Delay(3000);
}

void RM_RTOS_Default_Task(const void* args) {
 UNUSED(args);
 while (true) {
   led->High();
   osDelay(1000);
   led->Low();
   osDelay(1000);
 }
}