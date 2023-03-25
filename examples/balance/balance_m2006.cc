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

#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "main.h"
#include "motor.h"
#include "utils.h"
#include "dbus.h"

static bsp::CAN* can = nullptr;
static control::MotorCANBase* motorL = nullptr;
static control::MotorCANBase* motorR = nullptr;

void RM_RTOS_Init() {
  print_use_uart(&huart8);
  can = new bsp::CAN(&hcan1, 0x201, true);
  motorL = new control::Motor2006(can, 0x201);
  motorR = new control::Motor2006(can, 0x202);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  control::MotorCANBase* motors[] = {motorL, motorR};

  while (true) {
    motorL->SetOutput(0);
    motorR->SetOutput(0);
    control::MotorCANBase::TransmitOutput(motors, 2);
    clear_screen();
    set_cursor(0, 0);
    print("left omega: %f   left theta: %f\r\n", motorL->GetOmega(), motorL->GetTheta());
    print("right omega: %f   right theta: %f\r\n", motorR->GetOmega(), motorR->GetTheta());
    osDelay(100);
  }

}

