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

#include "bsp_ultrasonic.h"

namespace bsp {

Ultrasonic::Ultrasonic(GPIO_TypeDef* trig_group, uint16_t trig_pin, GPIO_TypeDef* echo_group,
                       uint16_t echo_pin, TIM_TypeDef* timer)
    : trig_(trig_group, trig_pin), echo_(echo_group, echo_pin), timer_(timer) {}

float Ultrasonic::GetDistance() {
  constexpr int TIME_OUT = 12000;
  constexpr float SOUND_SPEED_CM_PER_US = 0.0343;
  uint32_t base = timer_->CNT;
  uint32_t curr = base;
  trig_.High();
  while (curr - base < 20) curr = timer_->CNT;
  trig_.Low();
  base = timer_->CNT;
  // when the echo is emitted, echo turned to 1.
  while (!echo_.Read()) {
    if ((timer_->CNT - base) > TIME_OUT) {
      return -1;
    }
  }
  base = timer_->CNT;
  // when the echo is received, echo turned to 0.
  while (echo_.Read()) {
    if ((timer_->CNT - base) > TIME_OUT) {
      return -1;
    }
  }
  curr = timer_->CNT;
  return (curr - base) / 2.0 * SOUND_SPEED_CM_PER_US;
}

}  // namespace bsp