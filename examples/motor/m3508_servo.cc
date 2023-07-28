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
#include "dbus.h"
#include "main.h"
#include "motor.h"
#include "utils.h"

#define KEY_GPIO_GROUP GPIOB
#define KEY_GPIO_PIN GPIO_PIN_2

constexpr float RUN_SPEED = (10 * PI);
constexpr float ACCELERATION = (100 * PI);

bsp::CAN* can1 = nullptr;
control::MotorCANBase* motor1 = nullptr;
control::ServoMotor* servo1 = nullptr;

BoolEdgeDetector key_detector(false);

void jam_callback(control::ServoMotor* servo, const control::servo_jam_t data) {
  UNUSED(data);
  servo->SetTarget(servo->GetTheta() - (2 * PI/6), true);
}

void RM_RTOS_Init() {
  print_use_uart(&huart8);
  bsp::SetHighresClockTimer(&htim5);

  can1 = new bsp::CAN(&hcan1, true);
  motor1 = new control::Motor3508(can1, 0x201);
  control::servo_t servo_data;

  servo_data.motor = motor1;
  servo_data.max_speed = 2 * PI;
  servo_data.max_acceleration = 20 * PI;
  servo_data.transmission_ratio = M3508P19_RATIO;
  servo_data.omega_pid_param = new float[3]{15, 1, 5};
  servo_data.max_iout = 1000;
  servo_data.max_out = 10000;

  servo1 = new control::ServoMotor(servo_data);
  servo1->RegisterJamCallback(jam_callback, 0.15);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  control::MotorCANBase* motors[] = {motor1};

  // Wait for key to release
  while (true) {
    servo1->SetTarget(servo1->GetTarget() + (PI / 4), false);
    servo1->CalcOutput();
    control::MotorCANBase::TransmitOutput(motors, 1);
    osDelay(10);
  }
}
