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
 *   RX: binary 16-byte frames — target angles in centidegrees, 50 Hz
 *   TX: binary 16-byte frames — measured angles in centidegrees, 50 Hz
 *
 * UART communication runs in a dedicated RTOS thread (ArmUartTask) so that
 * encoder feedback is always sent at a consistent rate regardless of motor state.
 * See arm_uart_task.h / arm_uart_task.cc.
 */

#pragma once

#include <cstdint>

#include "bsp_buzzer.h"
#include "bsp_uart.h"
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
 * @brief Safe-park the arm before disabling: move J2→-0.8 rad, J3→-0.3 rad,
 *        then disable all motors.  Blocking — uses osDelay internally.
 *
 * Call from the kill-switch handler in RM_RTOS_Default_Task() or anywhere
 * a controlled shutdown is needed.  No-op if the arm is not currently enabled.
 */
void ArmSafePark();

/**
 * @brief Run one arm motor-control tick.  Call every 5 ms from RM_RTOS_Default_Task().
 *        Handles watchdog, motor commands, gripper state machine, and CAN TX.
 *        UART RX/TX now runs in ArmUartTask (see arm_uart_task.h).
 *
 * @param test_mode  When true, OrangePi UART is ignored.  All six joint
 *                   targets are forced to 0 °  (home / zero position), the
 *                   watchdog is suppressed, and the arm is auto-enabled if it
 *                   was not already.  Useful for bench-testing arm motors
 *                   without an OrangePi present.  Activate by holding the
 *                   remote-controller left switch (swl) in the UP position.
 */
void ArmUpdate(bool test_mode = false);

/**
 * @brief Move all arm joints to position 0 in sequence: J3→J2→J4→J5→J6→J1.
 *
 * Each joint is commanded to 0 rad and held there until its output-shaft
 * angle is within @p thresh_rad.  Only then does the routine move on to the
 * next joint.  A per-joint @p timeout_ms guards against a stalled motor.
 *
 * Must be called from a FreeRTOS task (uses osDelay internally).
 * ArmEnable() is called automatically if the arm is not yet enabled.
 *
 * @param thresh_rad  Settle threshold [rad] — default 0.05 rad (~2.9°).
 * @param timeout_ms  Per-joint timeout [ms]  — default 10 000 ms.
 */
void ArmHomeSequence(float thresh_rad = 0.05f, uint32_t timeout_ms = 10000);

// Set to true by ArmHomeSequence(), reset to false whenever motors are disabled.
// Checked by ArmUpdate() to decide whether to enforce per-joint position limits.
extern bool arm_homed;

// Onboard buzzer (TIM12 CH2). Initialized in ArmInit().
// Available for boot jingles and status tones throughout the firmware.
extern bsp::Buzzer* arm_buzzer;

// ── Per-joint position limits — motor-angle space [degrees] ──────────────────
// The ROS bridge (uart_bridge_node.py) applies sign-flip and gear-ratio before
// transmitting, so the MCU receives MOTOR-shaft angles.  These limits are in
// that same space and must be consistent with the URDF / joint_limits.yaml on
// the ROS side.
//
//   J1 (gear 2:1, sign -1):  ±180° joint  →  ±360° motor
//   J2 (gear 1:1, sign -1):  ±90°  joint  →   ±90° motor
//   J3 (gear 1:1, sign +1):  -1.20…+4.0 rad  →  -68.75°…+229.18° motor
//   J4 (gear 1:1, sign -1):  ±180° joint  →  ±180° motor
//   J5 (gear 1:1, sign -1):  ±90°  joint  →   ±90° motor
//   J6 (gear 1:1, sign +1):  ±180° joint  →  ±180° motor
//
// Used in arm_mc02.cc (ArmUpdate SetOutput clamping) and
// arm_uart_task.cc (RX sanity clamp).
static constexpr float ARM_CMD_MIN_DEG[6] = {-360.0f, -90.0f, -68.75f, -180.0f, -90.0f, -180.0f};
static constexpr float ARM_CMD_MAX_DEG[6] = { 360.0f,  90.0f, 229.18f,  180.0f,  90.0f,  180.0f};
