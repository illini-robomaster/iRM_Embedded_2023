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
control::MotorCANBase* left_flywheel_motor = nullptr;
control::MotorCANBase* right_flywheel_motor = nullptr;
remote::DBUS* dbus = nullptr;

void RM_RTOS_Init() {
  // print_use_uart(&huart1);
  can = new bsp::CAN(&hcan1, 0x201, true);
  right_flywheel_motor = new control::Motor3508(can, 0x201);
  left_flywheel_motor = new control::Motor3508(can, 0x202);

  dbus = new remote::DBUS(&huart1);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  osDelay(500);  // DBUS initialization needs time
  control::MotorCANBase* motors[] = {right_flywheel_motor, left_flywheel_motor};
  control::PIDController pid_right(20, 30, 25);
  control::PIDController pid_left(20, 30, 25);
  float target = 20;
  float diff_right = 0;
  int16_t out_right = 0;
  float diff_left = 0;
  int16_t out_left = 0;


  while (true) {
    
    if (dbus->swr == remote::UP) {
      target = 20;
      diff_right = motors[0]->GetOmegaDelta(target);
      out_right = pid_right.ComputeConstrainedOutput(diff_right);
      motors[0]->SetOutput(out_right);

      diff_left = motors[1]->GetOmegaDelta(-target);
      out_left = pid_right.ComputeConstrainedOutput(diff_left);
      motors[1]->SetOutput(out_left);

      control::MotorCANBase::TransmitOutput(motors, 2);
      // motor->PrintData();
    } else {
      target = 0;
      diff_right = motors[0]->GetOmegaDelta(target);
      out_right = pid_right.ComputeConstrainedOutput(diff_right);
      motors[0]->SetOutput(out_right);

      diff_left = motors[1]->GetOmegaDelta(-target);
      out_left = pid_right.ComputeConstrainedOutput(diff_left);
      motors[1]->SetOutput(out_left);

      control::MotorCANBase::TransmitOutput(motors, 2);
    }
    
    osDelay(10);
  }
}
