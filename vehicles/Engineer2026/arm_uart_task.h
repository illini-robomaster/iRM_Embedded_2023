/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2025 RoboMaster.                                          *
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

/**
 * @brief Dedicated RTOS thread for OrangePi UART communication.
 *
 * Runs independently of the motor-control loop (ArmUpdate) so that:
 *  - Encoder feedback TX is sent at a steady 50 Hz regardless of motor state
 *  - Command RX is parsed promptly even when motors are disabled
 *  - Pre-enable encoder polling keeps GetTheta() fresh before ArmEnable()
 *
 * Register this thread in RM_RTOS_Threads_Init() using armUartTaskAttr.
 */

#pragma once

#include "cmsis_os.h"

/** RTOS thread attributes — use with osThreadNew(ArmUartTask, ...). */
extern const osThreadAttr_t armUartTaskAttr;

/** Thread entry point — do not call directly; pass to osThreadNew(). */
void ArmUartTask(void* arg);
