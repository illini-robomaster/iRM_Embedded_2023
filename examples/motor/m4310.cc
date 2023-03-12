/****************************************************************************
*                                                                          *
*  Copyright (C) 2022 RoboMaster.                                          *
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
//#define WITH_CONTROLLER

#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "motor.h"
#ifdef WITH_CONTROLLER
#include "dbus.h"
#endif


static bsp::CAN* can = nullptr;
static control::Motor4310* motor = nullptr;

#ifdef WITH_CONTROLLER
remote::DBUS* dbus = nullptr;
#endif

void RM_RTOS_Init() {
  print_use_uart(&huart1);
  can = new bsp::CAN(&hcan1, 0x201, true);
  motor = new control::Motor4310(can, 0x01);
}

void RM_RTOS_Default_Task(const void* args) {
  // need to press reset to start
  UNUSED(args);
  control::Motor4310::Initialize4310(motor);

  while (true) {
    motor->SetOutput4310(0, 0, 0.4, 0.05, 0);
    control::Motor4310::TransmitOutput4310(motor);
    osDelay(10);
  }
}