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

namespace control {

typedef struct {
  float power_limit;
  float WARNING_power;
  float WARNING_power_buff;
  float buffer_total_current_limit;
  float power_total_current_limit;
} power_limit_t;

class PowerLimit {
 public:
  PowerLimit(int motor_num);
  void Output(bool turn_on, power_limit_t power_limit_info, float chassis_power,
              float chassis_power_buffer, float* PID_output, float* output);

 private:
  int motor_num_;
};

}  // namespace control
