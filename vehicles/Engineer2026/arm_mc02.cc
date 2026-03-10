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
 * @brief Engineer 2026 arm controller implementation.
 *
 * See arm_mc02.h for the full hardware / protocol description.
 */

#include "arm_mc02.h"

#include <cmath>
#include <cstdio>
#include <cstring>

#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "controller.h"
#include "fdcan.h"
#include "motor.h"
#include "uart_framing.h"
#include "usart.h"

#define TEST_UART_TRANSMISSION  // uncomment this line to test the transmission from mcu to ros2
// #define TEST_UART_RECEIVE       // uncomment this line to test the reception of UART Joint Variables from ros2 to mcu

#ifdef TEST_UART_TRANSMISSION
bool test_ros_tx = true;
#else
bool test_ros_tx = false;
#endif

#ifdef TEST_UART_RECEIVE
bool test_ros_rx = true;
#else
bool test_ros_rx = false;
#endif

// ── CAN IDs ─────────────────────────────────────────────────────────────────
// master_id = feedback frame ID configured in DAMIAO tool ("Master ID")
// can_id    = command frame ID configured in DAMIAO tool ("CAN ID")
// TODO: replace placeholder values with your DAMIAO-tool-configured IDs.
static constexpr uint16_t J1_MASTER_ID = 0x10, J1_CAN_ID = 0x11;  // Motor4310
static constexpr uint16_t J2_MASTER_ID = 0x12, J2_CAN_ID = 0x13;  // MotorDMJ10010
static constexpr uint16_t J3_MASTER_ID = 0x20, J3_CAN_ID = 0x21;  // MotorDMJ10010
static constexpr uint16_t J4_MASTER_ID = 0x16, J4_CAN_ID = 0x17;  // Motor4310
static constexpr uint16_t J5_MASTER_ID = 0x18, J5_CAN_ID = 0x19;  // Motor4310
static constexpr uint16_t J6_MASTER_ID = 0x14, J6_CAN_ID = 0x15;  // MotorDMJ3507
static constexpr uint16_t GRIP_RX_ID   = 0x206;                    // Motor2006

// ── Velocity & current limits ────────────────────────────────────────────────
// Setpoint ramp rate [deg/s] — matches MoveIt joint_limits.yaml.
// The ramp runs at 200 Hz (5 ms tick), so max_step = MAX_VEL_DEG[i] * 0.005f.
static constexpr float MAX_VEL_DEG[6] = {68.8f, 68.8f, 68.8f, 85.9f, 85.9f, 103.1f};

// Velocity limit forwarded to motor controllers [rad/s].
static constexpr float ARM_VEL_LIM[6] = {1.2f, 1.2f, 1.2f, 1.5f, 1.5f, 1.8f};

// J2/J3 FORCE_POS: peak current as a fraction of 99.74 A motor max [0, 1.0].
// Start at 50 % and reduce if motors run warm.
static constexpr float J23_CURRENT_LIM = 0.5f;

// ── UART watchdog ────────────────────────────────────────────────────────────
// If no valid RX frame arrives within this window, arm motors are disabled.
static constexpr uint32_t WATCHDOG_MS = 1000;

// ── Gripper (Motor2006) ───────────────────────────────────────────────────────
// Open-loop close current [−16384 … +16384 raw units]. Positive = close.
// TODO: tune direction and magnitude on the bench.
static constexpr int16_t GRIP_CLOSE_CURRENT = 2000;
// Current reading above which we consider the gripper stalled (closed).
// TODO: tune — start high and lower until reliable.
static constexpr int16_t GRIP_STALL_THRESH = 4000;
// Hold-position PID gains (drives Motor2006 in current mode to hold theta).
static constexpr float GRIP_KP     = 3000.0f;
static constexpr float GRIP_KI     = 0.0f;
static constexpr float GRIP_KD     = 100.0f;
static constexpr float GRIP_MAXOUT = 8000.0f;

// ── Helpers ───────────────────────────────────────────────────────────────────
static constexpr float DEG2RAD = (float)M_PI / 180.0f;
static constexpr float RAD2DEG = 180.0f / (float)M_PI;

template <typename T>
static inline T clamp(T v, T lo, T hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// ── Module-level state ────────────────────────────────────────────────────────
static bsp::CAN*  arm_can  = nullptr;  // hfdcan2 — arm motors
static bsp::UART* arm_uart = nullptr;  // huart10 — OrangePi link

static control::Motor4310* arm_j1 = nullptr;
static control::MotorDMJ10010* arm_j2 = nullptr;
static control::MotorDMJ10010* arm_j3 = nullptr;
static control::Motor4310* arm_j4 = nullptr;
static control::Motor4310* arm_j5 = nullptr;
static control::MotorDMJ3507* arm_j6 = nullptr;
static control::Motor2006*     gripper = nullptr;

// OrangePi-commanded targets [degrees], updated on valid UART RX.
static float cmd_target_deg[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
// Ramped setpoints sent to motors [degrees].
static float ramp_target_deg[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// Watchdog
static uint32_t last_valid_rx_tick = 0;
static bool     arm_enabled        = false;

// UART line accumulation buffer.
static char rx_line_buf[80];
static int  rx_line_len = 0;

// Gripper state machine.
enum class GripState { CLOSING, HOLDING };
static GripState grip_state    = GripState::CLOSING;
static float     grip_hold_pos = 0.0f;
static control::ConstrainedPID* grip_pid = nullptr;

// One-shot print flag for test-mode entry message.
static bool test_mode_printed = false;

// ── ArmInit ───────────────────────────────────────────────────────────────────

void ArmInit() {
  // Arm motors on FDCAN2, separate from the chassis FDCAN1 bus.
  arm_can = new bsp::CAN(&hfdcan2, 0);

  // OrangePi UART on huart10 @ 115200 8N1 (wired to /dev/ttyS4 on the OrangePi).
  arm_uart = new bsp::UART(&huart10);
  arm_uart->SetupRx(256);
  arm_uart->SetupTx(128);

  // Instantiate arm joints — all on the arm CAN bus.
  arm_j1 = new control::Motor4310(arm_can, J1_MASTER_ID, J1_CAN_ID, control::POS_VEL);
  arm_j2 = new control::MotorDMJ10010(arm_can, J2_MASTER_ID, J2_CAN_ID, control::FORCE_POS);
  arm_j3 = new control::MotorDMJ10010(arm_can, J3_MASTER_ID, J3_CAN_ID, control::FORCE_POS);
  arm_j4 = new control::Motor4310(arm_can, J4_MASTER_ID, J4_CAN_ID, control::POS_VEL);
  arm_j5 = new control::Motor4310(arm_can, J5_MASTER_ID, J5_CAN_ID, control::POS_VEL);
  arm_j6 = new control::MotorDMJ3507(arm_can, J6_MASTER_ID, J6_CAN_ID, control::POS_VEL);
  gripper = new control::Motor2006(arm_can, GRIP_RX_ID);

  grip_pid = new control::ConstrainedPID(GRIP_KP, GRIP_KI, GRIP_KD, GRIP_MAXOUT, 16384.0f);

  last_valid_rx_tick = HAL_GetTick();
}

// ── checkAllMotorsConnected ───────────────────────────────────────────────────

void checkAllMotorsConnected(control::MotorDM3519*   rl,
                              control::MotorDM3519*   rr,
                              control::MotorDMJ10010* lift) {
  print("=== Waiting for all motors to connect ===\r\n");
  while (true) {
    bool ok = true;

    // ── Chassis motors (hfdcan1) ────────────────────────────────────────────
    if (!rl->connection_flag_)   { print("  RL motor (DM3519) not connected\r\n");   ok = false; }
    if (!rr->connection_flag_)   { print("  RR motor (DM3519) not connected\r\n");   ok = false; }
    if (!lift->connection_flag_) { print("  Lift motor (10010L) not connected\r\n"); ok = false; }

    // Arm joints are NOT checked here — DM motors only emit CAN feedback
    // after receiving a command frame, so their connection_flag_ is always
    // false at this point. Connection for each arm joint is verified inside
    // ArmEnable() → MotorEnable(), which sends the enable frame and blocks
    // until the motor replies.

    if (ok) {
      print("=== All motors connected ===\r\n");
      return;
    }
    osDelay(100);
  }
}

// ── ArmEnable ─────────────────────────────────────────────────────────────────

void ArmEnable() {
  print("Enabling J1 (DM4310)...\r\n");
  arm_j1->MotorEnable();
  print("Enabling J2 (10010L)...\r\n");
  arm_j2->MotorEnable();
  print("Enabling J3 (10010L)...\r\n");
  arm_j3->MotorEnable();
  print("Enabling J4 (DM4310)...\r\n");
  arm_j4->MotorEnable();
  print("Enabling J5 (DM4310)...\r\n");
  arm_j5->MotorEnable();
  print("Enabling J6 (DM3507)...\r\n");
  arm_j6->MotorEnable();
  // Motor2006 does not need an explicit enable — it responds as soon as
  // CAN output commands are transmitted.
  print("Arm enabled.\r\n");
  arm_enabled = true;
  last_valid_rx_tick = HAL_GetTick();  // reset watchdog so we have 1 s to start sending
}

// ── ArmUpdate ─────────────────────────────────────────────────────────────────

void ArmUpdate(bool only_angle_read) {
  // ── 0. Test mode: poll joint positions without driving any motor ──────────
  // Sends the DM enable frame (0xFC) non-blocking on each tick to solicit a
  // CAN feedback packet from every joint — no position setpoint is issued.
  if (only_angle_read) {
    if (!test_mode_printed) {
      // print("ARM TEST MODE: polling joint positions (swl UP)\r\n");
      test_mode_printed = true;
    }
    if (!arm_enabled) ArmEnable();

    // The enable frame is the same for Motor4310, DMJ10010, and DMJ3507.
    // tx_id_actual_ = can_id + mode_offset:
    //   Motor4310  POS_VEL  → can_id + 0x100
    //   DMJ10010   FORCE_POS → can_id + 0x300
    //   DMJ3507    POS_VEL  → can_id + 0x100
    // constantly sending the disable frame can obtain the position feedback without enabling the motor, which is useful for testing and debugging without moving the arm. The feedback frame is sent at a low rate (~10 Hz) to avoid flooding the CAN bus.

    arm_j1->MotorDisable();
    arm_j2->MotorDisable();
    arm_j3->MotorDisable();
    arm_j4->MotorDisable();
    arm_j5->MotorDisable();
    arm_j6->MotorDisable();

    // Print all joint angles at ~1 Hz.
    if (HAL_GetTick() % 1000 < 5) {
      print("ARM TEST | J1=%6.2f J2=%6.2f J3=%6.2f J4=%6.2f J5=%6.2f J6=%6.2f [rad]\r\n",
            arm_j1->GetTheta(), arm_j2->GetTheta(),
            arm_j3->GetTheta(), arm_j4->GetTheta(),
            arm_j5->GetTheta(), arm_j6->GetTheta());
    }
  }
  // ── 1. UART RX: accumulate bytes into a line buffer, parse on '\n' ────────
  // Skipped in test mode — targets are already set above.
  if (!only_angle_read) {
    uint8_t* rx_buf = nullptr;
    int32_t rx_len = arm_uart->Read(&rx_buf);
    for (int32_t i = 0; i < rx_len; ++i) {
      char c = (char)rx_buf[i];
      if (c == '\r') continue;  // strip CR in case of CRLF line endings
      if (c == '\n') {
        rx_line_buf[rx_line_len] = '\0';
        if (UartRxParseLine(rx_line_buf, cmd_target_deg)) {
          last_valid_rx_tick = HAL_GetTick();
          if (!arm_enabled) ArmEnable();  // first valid frame after power-up
        }
        rx_line_len = 0;
      } else if (rx_line_len < (int)sizeof(rx_line_buf) - 1) {
        rx_line_buf[rx_line_len++] = c;
      } else {
        // Overlong line — discard and restart.
        rx_line_len = 0;
      }
    }
  }  // end if (!only_angle_read)

  // ── 2. Watchdog check ─────────────────────────────────────────────────────
  // Suppressed in test mode — OrangePi is intentionally absent.
  if (!only_angle_read && arm_enabled && (HAL_GetTick() - last_valid_rx_tick > WATCHDOG_MS)) {
    print("ARM WATCHDOG: no UART for >1 s — disabling arm motors\r\n");
    arm_j1->MotorDisable();
    arm_j2->MotorDisable();
    arm_j3->MotorDisable();
    arm_j4->MotorDisable();
    arm_j5->MotorDisable();
    arm_j6->MotorDisable();
    // Motor2006: send zero current.
    gripper->SetOutput(0);
    control::MotorCANBase* grip_arr[] = {gripper};
    control::MotorCANBase::TransmitOutput(grip_arr, 1);
    arm_enabled = false;
    return;
  }

  if (!arm_enabled) return;

  // ── 3. Setpoint ramp (per joint, 200 Hz → dt = 0.005 s) ─────────────────
  for (int i = 0; i < 6; ++i) {
    float max_step = MAX_VEL_DEG[i] * 0.005f;
    float delta = cmd_target_deg[i] - ramp_target_deg[i];
    ramp_target_deg[i] += clamp(delta, -max_step, max_step);
  }

  // ── 4. Motor commands ────────────────────────────────────────────────────
  // J1, J4, J5 — Motor4310, POS_VEL mode: SetOutput(position_rad, vel_limit_rad_s)
  arm_j1->SetOutput(ramp_target_deg[0] * DEG2RAD, ARM_VEL_LIM[0]);
  arm_j4->SetOutput(ramp_target_deg[3] * DEG2RAD, ARM_VEL_LIM[3]);
  arm_j5->SetOutput(ramp_target_deg[4] * DEG2RAD, ARM_VEL_LIM[4]);

  // J2, J3 — MotorDMJ10010, FORCE_POS mode: SetOutput(pos_rad, vel_limit_rad_s, current_frac)
  arm_j2->SetOutput(ramp_target_deg[1] * DEG2RAD, ARM_VEL_LIM[1], J23_CURRENT_LIM);
  arm_j3->SetOutput(ramp_target_deg[2] * DEG2RAD, ARM_VEL_LIM[2], J23_CURRENT_LIM);

  // J6 — MotorDMJ3507, POS_VEL mode: SetOutput(position_rad, vel_limit_rad_s)
  arm_j6->SetOutput(ramp_target_deg[5] * DEG2RAD, ARM_VEL_LIM[5]);

  // ── 5. Gripper state machine ─────────────────────────────────────────────
  switch (grip_state) {
    case GripState::CLOSING:
      gripper->SetOutput(GRIP_CLOSE_CURRENT);
      if (gripper->GetCurr() > GRIP_STALL_THRESH) {
        grip_hold_pos = gripper->GetTheta();
        grip_state = GripState::HOLDING;
      }
      break;
    case GripState::HOLDING: {
      int16_t pid_out = grip_pid->ComputeConstrainedOutput(
          gripper->GetThetaDelta(grip_hold_pos));
      gripper->SetOutput(pid_out);
      break;
    }
  }

  // ── 6. CAN transmit ──────────────────────────────────────────────────────
  control::Motor4310* j145[] = {arm_j1, arm_j4, arm_j5};
  control::MotorDMJ10010* j23[] = {arm_j2, arm_j3};
  control::MotorDMJ3507* j6arr[] = {arm_j6};
  control::MotorCANBase* grip[] = {gripper};
  if (test_ros_rx) {
    print("ARM UART RX | J1=%6.2f J2=%6.2f J3=%6.2f J4=%6.2f J5=%6.2f J6=%6.2f [deg]\r\n",
          cmd_target_deg[0], cmd_target_deg[1], cmd_target_deg[2],
          cmd_target_deg[3], cmd_target_deg[4], cmd_target_deg[5]);
  }
  if (!only_angle_read) {
    control::Motor4310::TransmitOutput(j145, 3);
    control::MotorDMJ10010::TransmitOutput(j23, 2);
    control::MotorDMJ3507::TransmitOutput(j6arr, 1);
    control::MotorCANBase::TransmitOutput(grip, 1);
  }

  // ── 7. UART TX: send encoder feedback to OrangePi ───────────────────────
  // Convert motor angles from rad → deg.  GetTheta() returns radians.
  const float enc[6] = {
      arm_j1->GetTheta() * RAD2DEG,
      arm_j2->GetTheta() * RAD2DEG,
      arm_j3->GetTheta() * RAD2DEG,
      arm_j4->GetTheta() * RAD2DEG,
      arm_j5->GetTheta() * RAD2DEG,
      arm_j6->GetTheta() * RAD2DEG,
  };
  if (HAL_GetTick() % 1000 < 5)
    print("ARM UART TX | J1=%6.2f J2=%6.2f J3=%6.2f J4=%6.2f J5=%6.2f J6=%6.2f [deg]\r\n",
          enc[0], enc[1], enc[2], enc[3], enc[4], enc[5]);
  UartTxSendFeedback(arm_uart, enc);
}
