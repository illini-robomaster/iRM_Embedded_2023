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

#include "rgb.h"

namespace display {

RGB::RGB(TIM_HandleTypeDef* htim, uint8_t channelR, uint8_t channelG, uint8_t channelB,
         uint32_t clock_freq)
    : R_(htim, channelR, clock_freq, 0, 0),
      G_(htim, channelG, clock_freq, 0, 0),
      B_(htim, channelB, clock_freq, 0, 0) {
  R_.Start();
  G_.Start();
  B_.Start();
  R_.SetFrequency(3921);
  G_.SetFrequency(3921);
  B_.SetFrequency(3921);
}

void RGB::Display(uint32_t aRGB) {
  volatile uint8_t alpha;
  volatile uint16_t red, green, blue;

  alpha = (aRGB & 0xFF000000) >> 24;
  red = ((aRGB & 0x00FF0000) >> 16) * alpha / 255.0;
  green = ((aRGB & 0x0000FF00) >> 8) * alpha / 255.0;
  blue = ((aRGB & 0x000000FF) >> 0) * alpha / 255.0;

  R_.SetPulseWidth(red);
  G_.SetPulseWidth(green);
  B_.SetPulseWidth(blue);
}

void RGB::Stop() {
  R_.Stop();
  G_.Stop();
  B_.Stop();
}

}  // namespace display
