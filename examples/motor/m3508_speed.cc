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
#include "bsp_os.h"

#define TARGET_Position_positive 8*PI

bsp::CAN* can = nullptr;
control::MotorCANBase* motor = nullptr;
remote::DBUS* dbus = nullptr;
control::ServoMotor* servo = nullptr;

void RM_RTOS_Init() {
  print_use_uart(&huart1);
  bsp::SetHighresClockTimer(&htim5);

  can = new bsp::CAN(&hcan1, true);
  motor = new control::Motor3508(can, 0x202);
  dbus = new remote::DBUS(&huart3);


  control::servo_t servo_data;
  servo_data.motor = motor;
  servo_data.max_speed = 30*PI;
  servo_data.max_acceleration = 100*PI;
  servo_data.transmission_ratio = M3508P19_RATIO;
  servo_data.omega_pid_param = new float[3]{60, 0.5, 10};
  servo_data.max_iout = 1000;
  servo_data.max_out = 15000;
  servo = new control::ServoMotor(servo_data);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  control::MotorCANBase* motors[] = {motor};
  bool change_flag = false;
  while (true) {
    if (motor->GetOmega() < 0.01 && change_flag == false) {
      servo->SetTarget(-TARGET_Position_positive);
      change_flag = true;

    } else if (motor->GetOmega() < 0.01 && change_flag == true) {
      servo->SetTarget(0);
      change_flag = false;
    }
    servo->CalcOutput();
    control::MotorCANBase::TransmitOutput(motors, 1);
    osDelay(10);
  }
}
