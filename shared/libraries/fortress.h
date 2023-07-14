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

#include "bsp_gpio.h"
#include "chassis.h"
#include "controller.h"
#include "motor.h"

namespace control {

typedef enum { ELEVATOR, SPINNER } fortress_component_t;

typedef struct {
  bsp::GPIO* leftSwitch;
  bsp::GPIO* rightSwitch;
  MotorCANBase* leftElevatorMotor;
  MotorCANBase* rightElevatorMotor;
  MotorCANBase* fortressMotor;
} fortress_t;

class Fortress {
 public:
  Fortress(const fortress_t fortress);
  bool Calibrate();
  void Transform(const bool fortress_mode);
  void Spin(bool power_limit_on, float power_limit, float chassis_power,
            float chassis_power_buffer);
  bool Error();
  void Stop(const fortress_component_t component);
  bool Finished();

 private:
  bool fortress_mode_ = false;

  bsp::GPIO* leftSwitch_ = nullptr;
  bsp::GPIO* rightSwitch_ = nullptr;
  MotorCANBase* leftElevatorMotor_ = nullptr;
  MotorCANBase* rightElevatortMotor_ = nullptr;
  ServoMotor* servo_left_ = nullptr;
  ServoMotor* servo_right_ = nullptr;

  float target_left_ = 0;
  float target_right_ = 0;

  bool left_reach_ = false;
  bool right_reach_ = false;

  MotorCANBase* fortressMotor_ = nullptr;

  BoolEdgeDetector* left_edge_;
  BoolEdgeDetector* right_edge_;

  Chassis* spinner_;
};

}  // namespace control
