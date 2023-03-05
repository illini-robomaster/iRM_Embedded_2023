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

#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "main.h"
#include "motor.h"
#include "dbus.h"

#define TARGET_SPEED 30

bsp::CAN* can = nullptr;
control::MotorCANBase* motor1 = nullptr;
control::MotorCANBase* motor2 = nullptr;
static remote::DBUS* dbus;

void RM_RTOS_Init() {
//  print_use_uart(&huart1);
  dbus = new remote::DBUS(&huart1);
  can = new bsp::CAN(&hcan1, 0x201, true);
  motor1 = new control::Motor3508(can, 0x201);
  motor2 = new control::Motor3508(can, 0x202);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  control::MotorCANBase* motors[] = {motor1, motor2};
  control::PIDController pid1(20, 15, 30);
  control::PIDController pid2(20, 15, 30);

  while (true) {
    float diff1 = motor1->GetOmegaDelta(dbus->ch1 / 30);
    float diff2 = motor2->GetOmegaDelta(dbus->ch3 / 30);
    int16_t out1 = pid1.ComputeConstrainedOutput(diff1);
    int16_t out2 = pid2.ComputeConstrainedOutput(diff2);
    motor1->SetOutput(out1);
    motor2->SetOutput(out2);
    control::MotorCANBase::TransmitOutput(motors, 2);

    osDelay(10);
  }
}
