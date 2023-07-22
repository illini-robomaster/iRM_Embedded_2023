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

#include "bsp_gpio.h"
#include "bsp_pwm.h"

namespace control {

enum dir { FORWARD,
           BACKWARD };

class Stepper {
 public:
  Stepper(TIM_HandleTypeDef* htim, uint32_t channel, uint32_t clock_freq, GPIO_TypeDef* dir_group,
          uint16_t dir_pin, GPIO_TypeDef* enable_group, uint16_t enable_pin);
  void Move(dir direction, unsigned speed);
  void Stop();
  void Enable();
  void Disable();

 private:
  bsp::PWM stepper_;
  bsp::GPIO dir_;
  bsp::GPIO enable_;
};
}  // namespace control