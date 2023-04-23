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

// #define WITH_CONTROLLER

#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "main.h"
#include "motor.h"
#include "utils.h"
#ifdef WITH_CONTROLLER
#include "dbus.h"
#endif

#ifndef WITH_CONTROLLER
#define KEY_GPIO_GROUP GPIOB
#define KEY_GPIO_PIN GPIO_PIN_2
#endif

#define NOTCH (2 * PI / 4)
#define SPEED 200
#define ACCELERATION (80 * PI)

bsp::CAN* can1 = nullptr;
control::MotorCANBase* motor = nullptr;
control::ServoMotor* servo = nullptr;

bsp::GPIO* key = nullptr;

#ifdef WITH_CONTROLLER
remote::DBUS* dbus = nullptr;
#else
BoolEdgeDetector key_detector(false);
#endif

// #define USING_M3508
// #define USING_M2006
#define USING_M3508_STEERING

void RM_RTOS_Init() {
  print_use_uart(&huart1);
  bsp::SetHighresClockTimer(&htim5);

  can1 = new bsp::CAN(&hcan1, 0x201);
  key = new bsp::GPIO(IN2_GPIO_Port, IN2_Pin);

#ifdef USING_M3508
  motor = new control::Motor3508(can1, 0x201);
  control::servo_t servo_data;
  servo_data.motor = motor;
  servo_data.max_speed = SPEED;
  servo_data.max_acceleration = ACCELERATION;
  servo_data.transmission_ratio = M3508P19_RATIO;
  servo_data.omega_pid_param = new float[3]{150, 1.2, 5};
  servo_data.max_iout = 1000;
  servo_data.max_out = 13000;
  servo = new control::ServoMotor(servo_data);
#endif

#ifdef USING_M2006
  motor = new control::Motor2006(can1, 0x202);
  control::servo_t servo_data;
  servo_data.motor = motor;
  servo_data.max_speed = SPEED;
  servo_data.max_acceleration = ACCELERATION;
  servo_data.transmission_ratio = M2006P36_RATIO;
  servo_data.omega_pid_param = new float[3]{150, 1.2, 5};
  servo_data.max_iout = 1000;
  servo_data.max_out = 13000;
  servo = new control::ServoMotor(servo_data);
#endif

#ifdef USING_M3508_STEERING
  motor = new control::Motor3508(can1, 0x201);
  control::servo_t servo_data;
  servo_data.motor = motor;
  servo_data.max_speed = SPEED;
  servo_data.max_acceleration = ACCELERATION;
  servo_data.transmission_ratio = 8;
  servo_data.omega_pid_param = new float[3]{140, 1.2, 25};
  servo_data.max_iout = 1000;
  servo_data.max_out = 13000;
  servo = new control::ServoMotor(servo_data);
#endif

#ifdef WITH_CONTROLLER
  dbus = new remote::DBUS(&huart1);
#endif
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
#ifdef WITH_CONTROLLER
  osDelay(500);  // DBUS initialization needs time
#else

#endif
  // control::MotorCANBase* motors[] = {motor};

  float target = 0;

  while (true) {
#ifdef WITH_CONTROLLER
    target = float(dbus->ch1) / remote::DBUS::ROCKER_MAX * 6 * PI;
    servo->SetTarget(target, true);
#else
    key_detector.input(key->Read());
    constexpr float desired_target = 0.5 * 2 * PI;
    if (key_detector.posEdge() && servo->SetTarget(desired_target - target) != 0) {
      target = desired_target - target;
    }
#endif
    servo->CalcOutput();
    // control::MotorCANBase::TransmitOutput(motors, 1);

    static int i = 0;
    if (i > 10) {
      // print("%10.2f %10.2f %10.2f %10.2f ", dbus->ch0, dbus->ch0, dbus->ch0, dbus->ch0);
      print("%d ", !key->Read());
      servo->PrintData();
      i = 0;
    } else {
      i++;
    }

    osDelay(2);
  }
}
