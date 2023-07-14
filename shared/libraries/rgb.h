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

namespace display {

const uint32_t color_red = 0xFFFF0000;
const uint32_t color_green = 0xFF00FF00;
const uint32_t color_blue = 0xFF0000FF;
const uint32_t color_yellow = 0xFFFFFF00;
const uint32_t color_cyan = 0xFF00FFFF;
const uint32_t color_magenta = 0xFFFF00FF;

class RGB {
 public:
  RGB(TIM_HandleTypeDef* htim, uint8_t channelR, uint8_t channelG, uint8_t channelB,
      uint32_t clock_freq);
  void Display(uint32_t aRGB);
  void Stop();

 private:
  bsp::PWM R_, G_, B_;
};

}  // namespace display
