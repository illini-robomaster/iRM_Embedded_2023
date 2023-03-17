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

#include "main.h"

#include <cmath>

#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "steering.h"
#include "motor.h"

bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;

constexpr float RUN_SPEED = (4 * PI);
constexpr float ALIGN_SPEED = (PI);
constexpr float ACCELERATION = (100 * PI);
constexpr float WHEEL_SPEED = 200;

constexpr float OFFSET_MOTOR1 = 0;
constexpr float OFFSET_MOTOR2 = 0;
constexpr float OFFSET_MOTOR3 = 0;
constexpr float OFFSET_MOTOR4 = 0;

control::MotorCANBase* motor1 = nullptr;
control::MotorCANBase* motor2 = nullptr;
control::MotorCANBase* motor3 = nullptr;
control::MotorCANBase* motor4 = nullptr;

control::SteeringMotor* steering1 = nullptr;
control::SteeringMotor* steering2 = nullptr;
control::SteeringMotor* steering3 = nullptr;
control::SteeringMotor* steering4 = nullptr;

control::MotorCANBase* motor5 = nullptr;
control::MotorCANBase* motor6 = nullptr;
control::MotorCANBase* motor7 = nullptr;
control::MotorCANBase* motor8 = nullptr;

bsp::GPIO* pe1 = nullptr;
bsp::GPIO* pe2 = nullptr;
bsp::GPIO* pe3 = nullptr;
bsp::GPIO* pe4 = nullptr;

bool steering_align_detect1() {
  return pe1->Read() == 0;
}

bool steering_align_detect2() {
  return pe2->Read() == 0;
}

bool steering_align_detect3() {
  return pe3->Read() == 0;
}

bool steering_align_detect4() {
  return pe4->Read() == 0;
}

control::steering_chassis_t* steering_chassis;

control::SteeringChassis* chassis;
remote::DBUS* dbus = nullptr;

void RM_RTOS_Init() {
  print_use_uart(&huart1);
  bsp::SetHighresClockTimer(&htim5);

  pe1 = new bsp::GPIO(IN1_GPIO_Port, IN1_Pin);
  pe2 = new bsp::GPIO(IN2_GPIO_Port, IN2_Pin);
  pe3 = new bsp::GPIO(IN3_GPIO_Port, IN3_Pin);
  pe4 = new bsp::GPIO(IN4_GPIO_Port, IN4_Pin);

  can1 = new bsp::CAN(&hcan1, 0x201, true);
  can2 = new bsp::CAN(&hcan2, 0x201, false);

  // init steerings
  motor1 = new control::Motor3508(can1, 0x201);
  motor2 = new control::Motor3508(can1, 0x202);
  motor3 = new control::Motor3508(can1, 0x203);
  motor4 = new control::Motor3508(can1, 0x204);

  control::steering_t steering_data;
  steering_data.motor = motor1;
  steering_data.max_speed = RUN_SPEED;
  steering_data.max_acceleration = ACCELERATION;
  steering_data.transmission_ratio = 8;
  steering_data.omega_pid_param = new float[3]{140, 1.2, 25};
  steering_data.max_iout = 1000;
  steering_data.max_out = 13000;
  steering_data.calibrate_offset = OFFSET_MOTOR1;
  steering_data.align_detect_func = steering_align_detect1;
  steering1 = new control::SteeringMotor(steering_data);

  steering_data.calibrate_offset = OFFSET_MOTOR2;
  steering_data.motor = motor2;
  steering_data.align_detect_func = steering_align_detect2;
  steering2 = new control::SteeringMotor(steering_data);

  steering_data.calibrate_offset = OFFSET_MOTOR3;
  steering_data.motor = motor3;
  steering_data.align_detect_func = steering_align_detect3;
  steering3 = new control::SteeringMotor(steering_data);

  steering_data.motor = motor4;
  steering_data.calibrate_offset = OFFSET_MOTOR4;
  steering_data.align_detect_func = steering_align_detect4;
  steering4 = new control::SteeringMotor(steering_data);

  steering_chassis = new control::steering_chassis_t();

  steering_chassis->fl_steer_motor = steering4;
  steering_chassis->fr_steer_motor = steering3;
  steering_chassis->bl_steer_motor = steering1;
  steering_chassis->br_steer_motor = steering2;

  // init wheels
  motor5 = new control::Motor3508(can2, 0x205);
  motor6 = new control::Motor3508(can2, 0x206);
  motor7 = new control::Motor3508(can2, 0x207);
  motor8 = new control::Motor3508(can2, 0x208);

  steering_chassis->fl_wheel_motor = motor8;
  steering_chassis->fr_wheel_motor = motor7;
  steering_chassis->bl_wheel_motor = motor5;
  steering_chassis->br_wheel_motor = motor6;

  chassis = new control::SteeringChassis(steering_chassis);

  dbus = new remote::DBUS(&huart3);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  control::MotorCANBase* steer_motors[] = {motor1, motor2, motor3, motor4};

  control::MotorCANBase* wheel_motors[] = {motor5, motor6, motor7, motor8};
  control::PIDController pid5(120, 15, 30);
  control::PIDController pid6(120, 15, 30);
  control::PIDController pid7(120, 15, 30);
  control::PIDController pid8(120, 15, 30);

  osDelay(500);  // DBUS initialization needs time

  double theta1 = 0.0;
  double theta2 = 0.0;
  double theta3 = 0.0;
  double theta4 = 0.0;

  double _theta1 = 0.0;
  double _theta2 = 0.0;
  double _theta3 = 0.0;
  double _theta4 = 0.0;

  double vx = 0.0;
  double vy = 0.0;
  double vw = 0.0;

  int cnt = 0;

  int ret1 = 0;
  int ret2 = 0;
  int ret3 = 0;
  int ret4 = 0;

  float v5 = 0;
  float v6 = 0;
  float v7 = 0;
  float v8 = 0;

  float s1 = 1.0;
  float s2 = 1.0;
  float s3 = 1.0;
  float s4 = 1.0;

  double _theta4_alt = 0.0;
  double _theta3_alt = 0.0;
  double _theta1_alt = 0.0;
  double _theta2_alt = 0.0;

  bool alignment = false;

  while (true) {
    vx = -static_cast<double>(dbus->ch3) / 660;
    vy = static_cast<double>(dbus->ch2) / 660;
    vw = static_cast<double>(dbus->ch0) / 660;

    // Kill switch
    if (dbus->swl == remote::UP || dbus->swl == remote::DOWN) {
      RM_ASSERT_TRUE(false, "Operation killed");
    }

    // Only Calibrate once per Switch change
    if (dbus->swr != remote::UP) {
      alignment = false;
    }
    // Right Switch Up to start a calibration
    if (dbus->swr == remote::UP && alignment == false) {
      chassis->SteerSetMaxSpeed(ALIGN_SPEED);
      bool alignment_complete = false;
      while (!alignment_complete) {
        chassis->SteerCalcOutput();
        control::MotorCANBase::TransmitOutput(steer_motors, 4);
        alignment_complete = chassis->Calibrate();
        osDelay(1);
      }
      chassis->ReAlign();
      alignment = true;
      chassis->SteerSetMaxSpeed(RUN_SPEED);

      v5 = 0;
      v6 = 0;
      v7 = 0;
      v8 = 0;
      theta1 = 0.0;
      theta2 = 0.0;
      theta3 = 0.0;
      theta4 = 0.0;

      _theta1 = 0.0;
      _theta2 = 0.0;
      _theta3 = 0.0;
      _theta4 = 0.0;

      _theta4_alt = 0.0;
      _theta3_alt = 0.0;
      _theta1_alt = 0.0;
      _theta2_alt = 0.0;
    } else {
      // Stay at current position when no command is given
      if (vx == 0 && vy == 0 && vw == 0) {
        ret1 = steering1->TurnRelative(0);
        ret2 = steering2->TurnRelative(0);
        ret3 = steering3->TurnRelative(0);
        ret4 = steering4->TurnRelative(0);

        v5 = 0;
        v6 = 0;
        v7 = 0;
        v8 = 0;

      } else {
        // Compute 2 position proposals, theta and theta + PI.
        _theta4 = atan2(vy + vw * cos(PI / 4), vx - vw * sin(PI / 4));
        _theta3 = atan2(vy + vw * cos(PI / 4), vx + vw * sin(PI / 4));
        _theta1 = atan2(vy - vw * cos(PI / 4), vx - vw * sin(PI / 4));
        _theta2 = atan2(vy - vw * cos(PI / 4), vx + vw * sin(PI / 4));

        _theta4_alt = wrap<double>(_theta4 + PI, -PI, PI);
        _theta3_alt = wrap<double>(_theta3 + PI, -PI, PI);
        _theta1_alt = wrap<double>(_theta1 + PI, -PI, PI);
        _theta2_alt = wrap<double>(_theta2 + PI, -PI, PI);

        // Go to the closer proposal and change wheel direction accordingly
        if (abs(wrap<double>(_theta1 - theta1, -PI, PI)) <
            abs(wrap<double>(_theta1_alt - theta1, -PI, PI))) {
          s1 = 1.0;
          ret1 = steering1->TurnRelative(wrap<double>(_theta1 - theta1, -PI, PI));
        } else {
          s1 = -1.0;
          ret1 = steering1->TurnRelative(wrap<double>(_theta1_alt - theta1, -PI, PI));
        }
        if (abs(wrap<double>(_theta2 - theta2, -PI, PI)) <
            abs(wrap<double>(_theta2_alt - theta2, -PI, PI))) {
          s2 = 1.0;
          ret2 = steering2->TurnRelative(wrap<double>(_theta2 - theta2, -PI, PI));
        } else {
          s2 = -1.0;
          ret2 = steering2->TurnRelative(wrap<double>(_theta2_alt - theta2, -PI, PI));
        }
        if (abs(wrap<double>(_theta3 - theta3, -PI, PI)) <
            abs(wrap<double>(_theta3_alt - theta3, -PI, PI))) {
          s3 = 1.0;
          ret3 = steering3->TurnRelative(wrap<double>(_theta3 - theta3, -PI, PI));
        } else {
          s3 = -1.0;
          ret3 = steering3->TurnRelative(wrap<double>(_theta3_alt - theta3, -PI, PI));
        }
        if (abs(wrap<double>(_theta4 - theta4, -PI, PI)) <
            abs(wrap<double>(_theta4_alt - theta4, -PI, PI))) {
          s4 = 1.0;
          ret4 = steering4->TurnRelative(wrap<double>(_theta4 - theta4, -PI, PI));
        } else {
          s4 = -1.0;
          ret4 = steering4->TurnRelative(wrap<double>(_theta4_alt - theta4, -PI, PI));
        }

        if (cnt % 500 == 0) {
          print("%10.4f, %10.4f, %10.4f, %10.4f\r\n", _theta1, _theta2, _theta3, _theta4);
          cnt = 0;
        }
        cnt += 1;

        // Update theta when TurnRelative complete
        if (ret1 == 0) {
          theta1 = s1 == 1.0 ? _theta1 : _theta1_alt;
        }
        if (ret2 == 0) {
          theta2 = s2 == 1.0 ? _theta2 : _theta2_alt;
        }
        if (ret3 == 0) {
          theta3 = s3 == 1.0 ? _theta3 : _theta3_alt;
        }
        if (ret4 == 0) {
          theta4 = s4 == 1.0 ? _theta4 : _theta4_alt;
        }

        // Wheels move only when all SteeringMotors are in position
        if (ret1 == 0 && ret2 == 0 && ret3 == 0 && ret4 == 0) {

          v5 = s1 * sqrt(pow(vy - vw * cos(PI / 4), 2.0) + pow(vx - vw * sin(PI / 4), 2.0));
          v6 = s2 * sqrt(pow(vy - vw * cos(PI / 4), 2.0) + pow(vx + vw * sin(PI / 4), 2.0));
          v7 = s3 * sqrt(pow(vy + vw * cos(PI / 4), 2.0) + pow(vx + vw * sin(PI / 4), 2.0));
          v8 = s4 * sqrt(pow(vy + vw * cos(PI / 4), 2.0) + pow(vx - vw * sin(PI / 4), 2.0));

        }
      }
    }

    chassis->SteerCalcOutput();

    motor5->SetOutput(pid5.ComputeConstrainedOutput(motor5->GetOmegaDelta(v5 * WHEEL_SPEED)));
    motor6->SetOutput(pid6.ComputeConstrainedOutput(motor6->GetOmegaDelta(v6 * WHEEL_SPEED)));
    motor7->SetOutput(pid7.ComputeConstrainedOutput(motor7->GetOmegaDelta(v7 * WHEEL_SPEED)));
    motor8->SetOutput(pid8.ComputeConstrainedOutput(motor8->GetOmegaDelta(v8 * WHEEL_SPEED)));

    control::MotorCANBase::TransmitOutput(steer_motors, 4);
    control::MotorCANBase::TransmitOutput(wheel_motors, 4);
    osDelay(2);
  }
}
