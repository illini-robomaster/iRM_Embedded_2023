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

#pragma once

#include "controller.h"
#include "motor.h"
#include "power_limit.h"

constexpr uint16_t MOTOR_NUM = 4;
constexpr float WHEEL_SPEED_FACTOR = 16;

namespace control {

typedef struct {
  control::SteeringMotor* fl_steer_motor = nullptr;
  control::SteeringMotor* fr_steer_motor = nullptr;
  control::SteeringMotor* bl_steer_motor = nullptr;
  control::SteeringMotor* br_steer_motor = nullptr;

  control::MotorCANBase* fl_wheel_motor = nullptr;
  control::MotorCANBase* fr_wheel_motor = nullptr;
  control::MotorCANBase* bl_wheel_motor = nullptr;
  control::MotorCANBase* br_wheel_motor = nullptr;
} steering_chassis_t;

class SteeringChassis {
 public:
  SteeringChassis(steering_chassis_t* chassis);

  ~SteeringChassis();

  // front -> positive, back -> negative
  void SetXSpeed(float _vx);

  // left -> positive, right -> negative
  void SetYSpeed(float _vy);

  // counterclockwise -> positive
  void SetWSpeed(float _vw);

  void Update(float power_limit, float chassis_power, float chassis_power_buffer);

  /**
   * @brief find the aligned position
   *        If the motors don't have an aligned position, the motors rotate until their detectors return True.
   *        If the motors have one, this function does nothing and return True.
   * @note  Calibrate() only find the aligned position, it doesn't guarantee the motor ends in the aligned position or stops
   *
   *
   * @return True only when all four motors have a aligned position
   */
  bool Calibrate();

  /**
   * @brief Call ReAlign() for all four SteeringMotor
   *        Turn the motor to the aligned position
   *        Do nothing if the motor doesn't have an aligned position
   * @return sum of all returned values
   *         0 if success
   */
  int ReAlign();

  /**
   * @brief Call SteeringMotor::CalcOutput() for all 4 Steering Motors
   */
  void SteerCalcOutput();

  /**
   * @brief Call SetMaxSpeed() for all 4 Steering Motor
   */
  void SteerSetMaxSpeed(const float max_speed);

  /**
   * @brief Call PrintData() for all 4 Steering Motor
   * The order is FrontLeft, FrontRight, BackLeft, BackRight
   * It prints current target, current position, and align position
   */
  void PrintData();

 private:
  control::SteeringMotor* fl_steer_motor;
  control::SteeringMotor* fr_steer_motor;
  control::SteeringMotor* bl_steer_motor;
  control::SteeringMotor* br_steer_motor;

  control::MotorCANBase* fl_wheel_motor;
  control::MotorCANBase* fr_wheel_motor;
  control::MotorCANBase* bl_wheel_motor;
  control::MotorCANBase* br_wheel_motor;

  // current velocity
  // right -> positive, left -> negative
  // front -> positive, back -> negative
  // counterclockwise -> positive
  float vx;
  float vy;
  float vw;

  // current steering pos of the 4 wheels
  float theta0;
  float theta1;
  float theta2;
  float theta3;

  // same as class Chassis
  ConstrainedPID pids[4];
  PowerLimit* power_limit;
  power_limit_t power_limit_info;

};  // class SteeringChassis ends

}  // namespace control
