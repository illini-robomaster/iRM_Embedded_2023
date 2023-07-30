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

#include "stepper.h"

namespace control {

Stepper::Stepper(TIM_HandleTypeDef* htim, uint32_t channel, uint32_t clock_freq,
                 GPIO_TypeDef* dir_group, uint16_t dir_pin, GPIO_TypeDef* enable_group,
                 uint16_t enable_pin)
    : stepper_(htim, channel, clock_freq, 0, 0),
      dir_(dir_group, dir_pin),
      enable_(enable_group, enable_pin) {
  enable_.Low();
  stepper_.Start();
}

void Stepper::Move(dir direction, unsigned int speed) {
  switch (direction) {
    case FORWARD:
      dir_.High();
      break;
    case BACKWARD:
      dir_.Low();
      break;
    default:;
  }
  stepper_.SetFrequency(speed);
  stepper_.SetPulseWidth(1000000 / speed / 2);
}

void Stepper::Stop() {
  stepper_.SetFrequency(0);
  stepper_.SetPulseWidth(0);
}

void Stepper::Enable() { enable_.High(); }

void Stepper::Disable() { enable_.Low(); }

}  // namespace control
