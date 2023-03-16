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
#include "chassis.h"

#define TARGET_SPEED 30

bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;
// initial flywheels
control::MotorCANBase* right = nullptr;
control::MotorCANBase* left = nullptr;
// initial chassis
control::MotorCANBase* fl_motor = nullptr;
control::MotorCANBase* fr_motor = nullptr;
control::MotorCANBase* bl_motor = nullptr;
control::MotorCANBase* br_motor = nullptr;

control::Chassis* chassis = nullptr;
static remote::DBUS* dbus;

void RM_RTOS_Init() {
//  print_use_uart(&huart1);
  dbus = new remote::DBUS(&huart1);
  can1 = new bsp::CAN(&hcan1, 0x201, true);
  can2 = new bsp::CAN(&hcan2, 0x201, false);
  right = new control::Motor3508(can1, 0x201);
  left = new control::Motor3508(can1, 0x202);

  fl_motor = new control::Motor3508(can2, 0x201);
  fr_motor = new control::Motor3508(can2, 0x202);
  bl_motor = new control::Motor3508(can2, 0x203);
  br_motor = new control::Motor3508(can2, 0x204);

  control::MotorCANBase* motors[control::FourWheel::motor_num];
  motors[control::FourWheel::front_left] = fl_motor;
  motors[control::FourWheel::front_right] = fr_motor;
  motors[control::FourWheel::back_left] = bl_motor;
  motors[control::FourWheel::back_right] = br_motor;

  control::chassis_t chassis_data;
  chassis_data.motors = motors;
  chassis_data.model = control::CHASSIS_MECANUM_WHEEL;
  chassis = new control::Chassis(chassis_data);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  osDelay(500);  // DBUS initialization needs time

  control::MotorCANBase* motors_flywheels[] = {right, left};
  control::MotorCANBase* motors[] = {fl_motor, fr_motor, bl_motor, br_motor};

  control::PIDController pid_left(75, 0, 50);
  control::PIDController pid_right(75, 0, 50);

  while (true) {
    chassis->SetSpeed(dbus->ch0, dbus->ch1, dbus->ch2);

    // Kill switch
    // if (dbus->swr == remote::UP || dbus->swr == remote::DOWN) {
    //   RM_ASSERT_TRUE(false, "Operation killed");
    // }

    chassis->Update(false, 30, 20, 60);

    if (dbus->swr == remote::UP){
      float diff1 = right->GetOmegaDelta(3000);
      float diff2 = left->GetOmegaDelta(-3000);
      int16_t out1 = pid_right.ComputeConstrainedOutput(diff1);
      int16_t out2 = pid_left.ComputeConstrainedOutput(diff2);
      right->SetOutput(out1);
      left->SetOutput(out2);
      control::MotorCANBase::TransmitOutput(motors_flywheels, 2);
    }
    else if (dbus->swr == remote::DOWN){
      float diff1 = right->GetOmegaDelta(0);
      float diff2 = left->GetOmegaDelta(0);
      int16_t out1 = pid_right.ComputeConstrainedOutput(diff1);
      int16_t out2 = pid_left.ComputeConstrainedOutput(diff2);
      right->SetOutput(out1);
      left->SetOutput(out2);
      control::MotorCANBase::TransmitOutput(motors_flywheels, 2);
    }
    control::MotorCANBase::TransmitOutput(motors, 4);
    osDelay(10);
  }
}
