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

#pragma once

#include "controller.h"
#include "motor.h"
#include "power_limit.h"

#define MAX_WHEEL_NUM 4

namespace control {

/**
 * @brief chassis models
 */
typedef enum { CHASSIS_MECANUM_WHEEL, CHASSIS_ONE_WHEEL, CHASSIS_OMNI_WHEEL } chassis_model_t;

/**
 * @brief structure used when chassis instance is initialized
 */
typedef struct {
  MotorCANBase** motors; /* motor instances of all chassis motors */
  chassis_model_t model; /* chassis model                         */
} chassis_t;

/**
 * @brief motor configs for wheel vehicles
 */
struct FourWheel {
  enum { front_left,
         front_right,
         back_left,
         back_right,
         motor_num };
};

struct OneWheel {
  enum { center,
         motor_num };
};

/**
 * @brief wrapper class for chassis
 */
class Chassis {
 public:
  /**
   * @brief constructor for chassis
   *
   * @param chassis structure that used to initialize chassis, refer to type chassis_t
   */
  Chassis(const chassis_t chassis);

  /**
   * @brief destructor for chassis
   */
  ~Chassis();

  /**
   * @brief set the speed for chassis motors
   *
   * @param x_speed chassis speed on x-direction
   * @param y_speed chassis speed on y-direction
   * @param turn_speed chassis clockwise turning speed
   */
  void SetSpeed(const float x_speed, const float y_speed = 0, const float turn_speed = 0);

  /**
   * @brief calculate the output of the motors under current configuration
   * @note does not command the motor immediately
   */
  void Update(bool power_limit_on, float power_limit, float chassis_power,
              float chassis_power_buffer);

 private:
  // acquired from user
  MotorCANBase** motors_ = nullptr;
  chassis_model_t model_;

  // pids and current speeds for each motor on the chassis
  ConstrainedPID pids_[MAX_WHEEL_NUM];
  PowerLimit* power_limit_ = nullptr;
  float* speeds_ = nullptr;

  power_limit_t power_limit_info_;
};

}  // namespace control
