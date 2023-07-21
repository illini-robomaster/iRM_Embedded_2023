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

#include "bsp_pwm.h"
#include "controller.h"

namespace bsp {

typedef struct {
  TIM_HandleTypeDef* htim;
  uint8_t channel;
  uint32_t clock_freq;
  float temp;
} heater_init_t;

class Heater {
 public:
  Heater(heater_init_t init);
  Heater(TIM_HandleTypeDef* htim, uint8_t channel, uint32_t clock_freq, float temp);
  float Update(float real_temp);

 private:
  PWM pwm_;
  float temp_;
  control::ConstrainedPID pid_;
};

}  // namespace bsp
