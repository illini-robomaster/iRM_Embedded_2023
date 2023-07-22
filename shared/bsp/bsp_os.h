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

#include "tim.h"

namespace bsp {

/**
 * @brief set the timer to be used to generate high resolution ticks in [us]
 *
 * @param htim  HAL timer handle pointer
 *
 * @note this has to be called before starting rtos scheduler
 */
void SetHighresClockTimer(TIM_HandleTypeDef* htim);

/**
 * @brief get the current counter value of the highres timer in [us]
 *
 * @return high res tick in [us] (0 if highres clock not set)
 */
uint32_t GetHighresTickMicroSec(void);

} /* namespace bsp */
