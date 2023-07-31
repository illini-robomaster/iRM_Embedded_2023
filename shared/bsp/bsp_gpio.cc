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

#include "bsp_gpio.h"

#include "FreeRTOS.h"

namespace bsp {

GPIO::GPIO(GPIO_TypeDef* group, uint16_t pin) : group_(group), pin_(pin), state_(0) {}

void GPIO::High() {
  HAL_GPIO_WritePin(group_, pin_, GPIO_PIN_SET);
  state_ = 1;
}

void GPIO::Low() {
  HAL_GPIO_WritePin(group_, pin_, GPIO_PIN_RESET);
  state_ = 0;
}

void GPIO::Toggle() {
  HAL_GPIO_TogglePin(group_, pin_);
  state_ ^= 1;
}

uint8_t GPIO::Read() {
  state_ = (HAL_GPIO_ReadPin(group_, pin_) == GPIO_PIN_SET);
  return state_;
}

GPIT* GPIT::gpits[NUM_GPITS] = {NULL};

GPIT::GPIT(uint16_t pin) : pin_(pin) {
  int gpio_idx = GetGPIOIndex(pin);
  if (gpio_idx >= 0 && gpio_idx < NUM_GPITS && !gpits[gpio_idx]) gpits[gpio_idx] = this;
}

void GPIT::IntCallback(uint16_t pin) {
  int gpio_idx = GetGPIOIndex(pin);
  if (gpio_idx >= 0 && gpio_idx < NUM_GPITS && gpits[gpio_idx]) gpits[gpio_idx]->IntCallback();
}

int GPIT::GetGPIOIndex(uint16_t pin) {
  for (int i = 0; i < NUM_GPITS; ++i)
    if (pin == (1 << i)) return i;
  return -1;
}

} /* namespace bsp */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) { bsp::GPIT::IntCallback(GPIO_Pin); }
