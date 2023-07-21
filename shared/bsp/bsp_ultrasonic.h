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
#include "main.h"

namespace bsp {

class Ultrasonic {
 public:
  /**
   * @brief constructor for ultrasonic sensor class
   *
   * @param trig_group trig GPIO group
   * @param trig_pin   trig pin number
   * @param echo_group echo GPIO group
   * @param echo_pin   echo pin number
   * @param timer      1MHz timer used to calculate distance
   */
  Ultrasonic(GPIO_TypeDef* trig_group, uint16_t trig_pin, GPIO_TypeDef* echo_group,
             uint16_t echo_pin, TIM_TypeDef* timer);
  /**
   * @brief Get the current distance from ultrasonic sensor (Neo Notice: May take a few ms)
   *
   * @return current distance in cm
   */
  float GetDistance();

 private:
  GPIO trig_;
  GPIO echo_;
  TIM_TypeDef* timer_;
};

} /* namespace bsp */
