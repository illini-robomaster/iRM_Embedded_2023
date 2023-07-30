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

#include "bsp_error_handler.h"
#include "bsp_gpio.h"
#include "main.h"

namespace bsp {

class Relay {
 public:
  /**
   * @brief Constructor for relay
   *
   * @param relay_group relay GPIO group
   * @param relay_pin   relay GPIO pin number
   */
  Relay(GPIO_TypeDef* relay_group, uint16_t relay_pin);

  /**
   * @brief Turn on relay
   */
  void On();

  /**
   * @brief Turn off relay
   */
  void Off();

 private:
  GPIO relay_;
};

} /* namespace bsp */
