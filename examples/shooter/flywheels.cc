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
#include "controller.h"
#include "main.h"
#include "motor.h"
#include "dbus.h"

#define TARGET_SPEED 30

bsp::CAN* can = nullptr;
control::MotorCANBase* left = nullptr;
control::MotorCANBase* right = nullptr;
static remote::DBUS* dbus;

void RM_RTOS_Init() {
//  print_use_uart(&huart1);
  dbus = new remote::DBUS(&huart1);
  can = new bsp::CAN(&hcan1, 0x201, true);
  right = new control::Motor3508(can, 0x201);
  left = new control::Motor3508(can, 0x202);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  control::MotorCANBase* motors[] = {right, left};
  control::PIDController pid1(75, 15, 30);
  control::PIDController pid2(75, 15, 30);

  while (true) {
    if (dbus->swr == remote::UP){
      float diff1 = right->GetOmegaDelta(500);
      float diff2 = left->GetOmegaDelta(-500);
      int16_t out1 = pid1.ComputeConstrainedOutput(diff1);
      int16_t out2 = pid2.ComputeConstrainedOutput(diff2);
      right->SetOutput(out1);
      left->SetOutput(out2);
      control::MotorCANBase::TransmitOutput(motors, 2);
      osDelay(10);
    }
    else if (dbus->swr == remote::DOWN){
      float diff1 = right->GetOmegaDelta(0);
      float diff2 = left->GetOmegaDelta(0);
      int16_t out1 = pid1.ComputeConstrainedOutput(diff1);
      int16_t out2 = pid2.ComputeConstrainedOutput(diff2);
      right->SetOutput(out1);
      left->SetOutput(out2);
      control::MotorCANBase::TransmitOutput(motors, 2);
      osDelay(10);
    }

  }
}
