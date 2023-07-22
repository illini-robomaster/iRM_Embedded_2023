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

namespace control {

/**
 * @brief gimbal models
 */
typedef enum {
  SHOOTER_SENTRY,
  SHOOTER_STANDARD,
} shooter_model_t;

/**
 * @brief structure used when shooter instance is initialized
 * @note Because shooter ZERO uses snail M2305 motors that can only be driven using PWM,
 *       interfaces are reserved since the old shooter could be used for demonstratio
 *       purposes in the future.
 */
typedef struct {
  MotorBase* left_flywheel_motor;  /* motor instance of left flywheel motor  */
  MotorBase* right_flywheel_motor; /* motor instance of right flywheel motor */
  MotorCANBase* load_motor;        /* CAN motor instance of load motor       */
  shooter_model_t model;
  int dial_direction = 1; /* CCW is 1(default), CW is -1                     */
} shooter_t;

/**
 * @brief wrapper class for shooter
 */
class Shooter {
 public:
  /**
   * @brief constructor for Shooter instance
   *
   * @param shooter structure that used to initialize gimbal, refer to type shooter_t
   */
  Shooter(shooter_t shooter);

  /**
   * @brief destructor for shooter
   *
   */
  ~Shooter();

  /**
   * @brief set the speed of accelerating motors
   *
   * @param speed
   */
  void SetFlywheelSpeed(float speed);

  /**
   * @brief load the next bullet
   *
   * @return int servomotor status, refer to type servo_status_t
   */
  int LoadNext();

  /**
   * @brief update the output of the motors under current configuration
   * @note does not command the motor immediately
   */
  void Update();

  void FastContinueShoot();

  void SlowContinueShoot();

  void DoubleShoot();

  void DialStop();

  void SetViolentShoot(bool status);

 private:
  // acquired from user
  MotorBase* left_flywheel_motor_;
  MotorBase* right_flywheel_motor_;
  MotorCANBase* load_motor_;
  ServoMotor* load_servo_;
  shooter_model_t model_;

  PIDController* left_pid_;  /* pid for left flywheel  */
  PIDController* right_pid_; /* pid for right flywheel */
  ConstrainedPID* load_moter_pid_;

  BoolEdgeDetector* flywheel_turning_detector_; /* flywheel turning state detector */
  float load_step_angle_;                       /* angle rotated for every bullet loaded */
  float load_double_angle_;                     /* angle rotated for every double bullet loaded */
  float speed_;                                 /* current turning speed of flywheels */
  float dial_speed_;                            /* current turning speed of dial */
  float dial_continue_fast_acceleration;        /* acceleration for fast continue shooting mode */
  float dial_continue_slowly_acceleration;      /* acceleration for slowly continue shooting mode */
  float dial_double_acceleration;               /* acceleration for double shooting mode */
  int dial_direction_;
  bool violent_shooting_;
  float violent_shooting_speed_;
};

}  // namespace control
