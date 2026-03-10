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
 * @brief Engineer 2026 arm controller — public interface.
 *
 * Call ArmInit() from RM_RTOS_Init() after all board peripherals are ready.
 * Call checkAllMotorsConnected() from RM_RTOS_Default_Task() before enabling
 * any motors.
 * Call ArmUpdate() from the main control loop every 5 ms.
 *
 * Motor assignments (all on hfdcan2):
 *   J1 — Motor4310     (DM4310),   POS_VEL mode
 *   J2 — MotorDMJ10010 (10010L),   FORCE_POS mode
 *   J3 — MotorDMJ10010 (10010L),   FORCE_POS mode
 *   J4 — Motor4310     (DM4310),   POS_VEL mode
 *   J5 — Motor4310     (DM4310),   POS_VEL mode
 *   J6 — MotorDMJ3507  (DM3507),   POS_VEL mode
 *   Gripper — Motor2006 (2006),    open-loop → hold-on-stall
 *
 * OrangePi UART: huart10 @ 115200 8N1
 *   RX: "j1,j2,j3,j4,j5,j6\n"  — target angles in degrees, 50 Hz
 *   TX: "j1,j2,j3,j4,j5,j6\n"  — measured angles in degrees, 200 Hz
 */

#pragma once

#include "motor.h"

/**
 * @brief Initialise arm CAN bus, UART, and all arm motor objects.
 *        Must be called once from RM_RTOS_Init().
 */
void ArmInit();

/**
 * @brief Block until every arm motor AND the three chassis motors passed as
 *        arguments have sent at least one CAN feedback frame.
 *        Call once from RM_RTOS_Default_Task(), BEFORE any MotorEnable() call.
 *
 * @param rl   Rear-left  DM3519 chassis motor pointer
 * @param rr   Rear-right DM3519 chassis motor pointer
 * @param lift Lift       DMJ10010 chassis motor pointer
 */
void checkAllMotorsConnected(control::MotorDM3519* rl,
                              control::MotorDM3519* rr,
                              control::MotorDMJ10010* lift);

/**
 * @brief Enable all arm DM motors (blocking — waits for CAN enable-ack per motor).
 *        Call once from RM_RTOS_Default_Task(), AFTER checkAllMotorsConnected().
 */
void ArmEnable();

/**
 * @brief Run one arm control tick.  Call every 5 ms from RM_RTOS_Default_Task().
 *        Handles UART RX parsing, watchdog, setpoint ramping, motor commands,
 *        gripper state machine, CAN TX, and UART TX feedback.
 *
 * @param test_mode  When true, OrangePi UART is ignored.  All six joint
 *                   targets are forced to 0 °  (home / zero position), the
 *                   watchdog is suppressed, and the arm is auto-enabled if it
 *                   was not already.  Useful for bench-testing arm motors
 *                   without an OrangePi present.  Activate by holding the
 *                   remote-controller left switch (swl) in the UP position.
 */
void ArmUpdate(bool test_mode = false);
