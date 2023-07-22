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

#include "bsp_os.h"

#include "cmsis_os.h"
#include "task.h"

static TIM_HandleTypeDef* htim_os = nullptr;

/**
 * @brief override FreeRTOS weak function to configure the timer used for generating run-time stats
 */
extern "C" void configureTimerForRunTimeStats(void) {
  if (!htim_os) return;
  __HAL_TIM_SET_AUTORELOAD(htim_os, 0xffffffff);
  __HAL_TIM_SET_COUNTER(htim_os, 0);
  __HAL_TIM_ENABLE(htim_os);
}

/**
 * @brief triggered when a stack overflow is detected
 *
 * @param xTask       task handle
 * @param pcTaskName  task name
 */
extern "C" void vApplicationStackOverflowHook(xTaskHandle xTask, signed char* pcTaskName) {
  (void)xTask;
  (void)pcTaskName;

  while (true) {
  }
}

extern "C" unsigned long getRunTimeCounterValue(void) {
  if (!htim_os) return 0;
  return htim_os->Instance->CNT;
}

namespace bsp {

void SetHighresClockTimer(TIM_HandleTypeDef* htim) { htim_os = htim; }

uint32_t GetHighresTickMicroSec(void) { return getRunTimeCounterValue(); }

} /* namespace bsp */
