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

#include "bsp_buzzer.h"
#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "controller.h"
#include "dbus.h"
#include "fdcan.h"
#include "motor.h"
#include "tim.h"
#include "uart_framing.h"
#include "usart.h"
#include "utils.h"

// #define TEST_UART_TRANSMISSION  // uncomment this line to test the transmission from mcu to ros2
// #define TEST_UART_RECEIVE  // uncomment this line to test the reception of UART Joint Variables from ros2 to mcu

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
// Velocity limit forwarded to motor controllers [rad/s].
static constexpr float ARM_VEL_LIM[6] = {1.2f, 1.2f, 1.2f, 1.5f, 1.5f, 1.8f};

// J2/J3 FORCE_POS: peak current as a fraction of 99.74 A motor max [0, 1.0].
// Start at 50 % and reduce if motors run warm.
static constexpr float J23_CURRENT_LIM = 0.5f;

// J6 FORCE_POS current limit (MotorDMJ3507).
static constexpr float J6_CURRENT_LIM = 0.5f;

// ── Command sanity limits ────────────────────────────────────────────────────
// Per-joint motor-angle limits (ARM_CMD_MIN_DEG / ARM_CMD_MAX_DEG) are defined
// in arm_mc02.h and shared with arm_uart_task.cc.

// ── UART watchdog ────────────────────────────────────────────────────────────
// If no valid RX frame arrives within this window, arm motors are disabled.
static constexpr uint32_t WATCHDOG_MS = 2000;

// ── Safe-park positions [rad] ───────────────────────────────────────────────
// Before fully disabling the arm, command J2 and J3 to these positions so the
// arm doesn't drop under gravity when power is cut.
static constexpr float PARK_J1_RAD = 0.0f;
static constexpr float PARK_J2_RAD = -0.8f;
static constexpr float PARK_J3_RAD = -0.3f;
static constexpr float PARK_THRESH_RAD = 0.08f;    // settle threshold [rad]
static constexpr uint32_t PARK_TIMEOUT_MS = 5000;  // give up after 5 s

// ── Gripper (Motor2006) ───────────────────────────────────────────────────────
// Open-loop close current [−16384 … +16384 raw units]. Positive = close.
// TODO: tune direction and magnitude on the bench.
static constexpr int16_t GRIP_CLOSE_CURRENT = 16384;  // C610 full-scale
static constexpr int16_t GRIP_OPEN_CURRENT  = -16384;
// Current magnitude above which we consider the gripper stalled.
// Closing: current is positive → stall when GetCurr() > +GRIP_STALL_THRESH.
// Opening: current is negative → stall when GetCurr() < -GRIP_STALL_THRESH.
// TODO: tune — start high and lower until reliable.
static constexpr int16_t GRIP_STALL_THRESH = 4000;
// Number of consecutive 5 ms ticks above threshold before declaring a stall.
// Filters single-sample current spikes without adding meaningful latency.
static constexpr uint8_t GRIP_STALL_DEBOUNCE = 3;
// Hold-position PID gains (drives Motor2006 in current mode to hold theta).
static constexpr float GRIP_KP     = 3000.0f;
static constexpr float GRIP_KI     = 0.0f;
static constexpr float GRIP_KD     = 100.0f;
static constexpr float GRIP_MAXOUT = 8000.0f;

// ── Helpers ───────────────────────────────────────────────────────────────────
static constexpr float DEG2RAD = (float)M_PI / 180.0f;
static constexpr float RAD2DEG = 180.0f / (float)M_PI;

extern remote::DBUS* dbus;

template <typename T>
static inline T clamp(T v, T lo, T hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// ── Module-level state ────────────────────────────────────────────────────────
// Motor pointers and shared variables are non-static so arm_uart_task.cc
// (the dedicated UART RTOS thread) can access them via extern declarations.

bsp::CAN*    arm_can    = nullptr;  // hfdcan2 — arm motors
bsp::UART*   arm_uart   = nullptr;  // huart10 — OrangePi link
bsp::Buzzer* arm_buzzer = nullptr;  // TIM12 CH2 (PB15) — onboard buzzer

control::Motor4310* arm_j1 = nullptr;
control::MotorDMJ10010* arm_j2 = nullptr;
control::MotorDMJ10010* arm_j3 = nullptr;
control::Motor4310* arm_j4 = nullptr;
control::Motor4310* arm_j5 = nullptr;
control::MotorDMJ3507* arm_j6 = nullptr;
control::Motor2006* gripper = nullptr;

// OrangePi-commanded targets [degrees], updated on valid UART RX.
// Written by ArmUartTask, read by ArmUpdate (single-core STM32 — no mutex needed
// for aligned 32-bit floats; worst case is a one-tick-old value).
volatile float cmd_target_deg[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// Watchdog — last tick at which a valid UART RX frame was received.
// Written by ArmUartTask, read by ArmUpdate.
volatile uint32_t last_valid_rx_tick = 0;
volatile bool arm_enabled = false;

// Safe-park state machine — non-blocking, runs inside ArmUpdate's 200 Hz loop.
// arm_parking is non-static so arm_uart_task.cc can check it via extern.
bool arm_parking = false;  // true while parking sequence is active
// true after ArmHomeSequence() completes; reset whenever motors are disabled.
// Position-limit clamping in ArmUpdate() is only applied when this is true,
// because before homing the arm may be physically outside the operational range.
bool arm_homed = false;
static uint32_t park_deadline = 0;  // HAL tick at which we give up and force-disable
static float park_hold_j1 = 0.0f;   // freeze other joints at their position when parking started
static float park_hold_j2 = 0.0f;
static float park_hold_j3 = 0.0f;
static float park_hold_j4 = 0.0f;
static float park_hold_j5 = 0.0f;
static float park_hold_j6 = 0.0f;

// Gripper state machine.
enum class GripState { IDLE, OPENING, CLOSING, HOLDING };
static GripState grip_state         = GripState::IDLE;
static float     grip_hold_pos      = 0.0f;
static uint8_t   grip_open_stall_count = 0;  // consecutive ticks above open stall threshold
static control::ConstrainedPID* grip_pid = nullptr;

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
  arm_j6 = new control::MotorDMJ3507(arm_can, J6_MASTER_ID, J6_CAN_ID, control::FORCE_POS);
  gripper = new control::Motor2006(arm_can, GRIP_RX_ID);

  grip_pid = new control::ConstrainedPID(GRIP_KP, GRIP_KI, GRIP_KD, GRIP_MAXOUT, 16384.0f);

  // Buzzer: TIM12 CH2 (PB15), APB1 timer clock 80 MHz, prescaler 24
  arm_buzzer = new bsp::Buzzer(&htim12, 2, 80000000 / 24);

  // Do NOT seed last_valid_rx_tick here — leave it at 0 so the bumpless-enable
  // in ArmUpdate() won't fire until the UART task has actually received a frame.
}

// ── checkAllMotorsConnected ───────────────────────────────────────────────────

void checkAllMotorsConnected(control::MotorDM3519* rl,
                             control::MotorDM3519* rr,
                             control::MotorDMJ10010* lift) {
  print("=== Waiting for all motors to connect ===\r\n");
  while (true) {
    bool ok = true;

    // ── Chassis motors (hfdcan1) ────────────────────────────────────────────
    if (!rl->connection_flag_) {
      print("  RL motor (DM3519) not connected\r\n");
      ok = false;
    }
    if (!rr->connection_flag_) {
      print("  RR motor (DM3519) not connected\r\n");
      ok = false;
    }
    if (!lift->connection_flag_) {
      print("  Lift motor (10010L) not connected\r\n");
      ok = false;
    }

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
  // Set arm_enabled FIRST to prevent the UART task's pre-enable polling from
  // sending MotorDisable frames that race with our MotorEnable calls below.
  // The UART task checks (!arm_enabled) before sending MotorDisable — setting
  // this early suppresses that immediately.
  arm_enabled = true;
  last_valid_rx_tick = HAL_GetTick();  // reset watchdog so we have 2 s grace

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
}

// ── ArmSafePark ───────────────────────────────────────────────────────────────

void ArmSafePark() {
  if (!arm_enabled) return;  // nothing to park

  // Prevent the UART task from re-enabling the arm during the park sequence.
  arm_parking = true;

  // ── Phase 1: Home all joints to 0 ─────────────────────────────────────────
  // Uses the existing sequential homing routine (J3→J2→J4→J5→J6→J1).
  // This is blocking but safe — called from the kill-switch handler where the
  // operator has deliberately decided to shut down.
  print("ARM SAFE PARK: homing all joints to 0 first...\r\n");
  ArmHomeSequence();

  // ── Phase 2: Park J2/J3 to gravity-safe positions ─────────────────────────
  print("ARM SAFE PARK: moving J2→%.1f J3→%.1f before disable...\r\n",
        PARK_J2_RAD, PARK_J3_RAD);

  // Freeze other joints at their current position (should be ~0 after homing).
  const float hold_j4 = arm_j4->GetTheta();
  const float hold_j5 = arm_j5->GetTheta();
  const float hold_j6 = arm_j6->GetTheta();

  // Release gripper immediately.
  gripper->SetOutput(0);
  control::MotorCANBase* grip_arr[] = {gripper};
  control::MotorCANBase::TransmitOutput(grip_arr, 1);

  uint32_t deadline = HAL_GetTick() + PARK_TIMEOUT_MS;
  while (HAL_GetTick() < deadline) {
    arm_j1->SetOutput(PARK_J1_RAD, ARM_VEL_LIM[0]);
    arm_j2->SetOutput(PARK_J2_RAD, ARM_VEL_LIM[1], J23_CURRENT_LIM);
    arm_j3->SetOutput(PARK_J3_RAD, ARM_VEL_LIM[2], J23_CURRENT_LIM);
    arm_j4->SetOutput(hold_j4, ARM_VEL_LIM[3]);
    arm_j5->SetOutput(hold_j5, ARM_VEL_LIM[4]);
    arm_j6->SetOutput(hold_j6, ARM_VEL_LIM[5], J6_CURRENT_LIM);

    control::Motor4310* j145[] = {arm_j1, arm_j4, arm_j5};
    control::MotorDMJ10010* j23[] = {arm_j2, arm_j3};
    control::MotorDMJ3507* j6arr[] = {arm_j6};
    control::Motor4310::TransmitOutput(j145, 3);
    control::MotorDMJ10010::TransmitOutput(j23, 2);
    control::MotorDMJ3507::TransmitOutput(j6arr, 1);

    bool j2_ok = fabsf(arm_j2->GetTheta() - PARK_J2_RAD) < PARK_THRESH_RAD;
    bool j3_ok = fabsf(arm_j3->GetTheta() - PARK_J3_RAD) < PARK_THRESH_RAD;
    bool j1_ok = fabsf(arm_j1->GetTheta() - PARK_J1_RAD) < PARK_THRESH_RAD;
    if (j2_ok && j3_ok && j1_ok) {
      print("ARM SAFE PARK: J2/J3 settled — disabling\r\n");
      break;
    }

    if (HAL_GetTick() % 500 < 5)
      print("ARM PARK: J2=%.3f→%.1f J3=%.3f→%.1f\r\n",
            arm_j2->GetTheta(), PARK_J2_RAD, arm_j3->GetTheta(), PARK_J3_RAD);

    osDelay(5);
  }

  if (HAL_GetTick() >= deadline)
    print("ARM SAFE PARK: timeout — forcing disable\r\n");

  // ── Phase 3: Disable all motors ────────────────────────────────────────────
  arm_j1->MotorDisable();
  arm_j2->MotorDisable();
  arm_j3->MotorDisable();
  arm_j4->MotorDisable();
  arm_j5->MotorDisable();
  arm_j6->MotorDisable();
  arm_enabled = false;
  arm_parking = false;
  arm_homed = false;  // arm may be moved by hand after disable
  print("ARM SAFE PARK: done\r\n");
}

// ── ArmHomeSequence ───────────────────────────────────────────────────────────

void ArmHomeSequence(float thresh_rad, uint32_t timeout_ms) {
  if (!arm_enabled) ArmEnable();

  // Sequence: J3 → J2 → J4 → J5 → J6 → J1
  // Each entry: { joint index 0-based, motor type tag }
  // Tags: 0 = Motor4310 (POS_VEL), 1 = MotorDMJ10010 (FORCE_POS), 2 = MotorDMJ3507 (FORCE_POS)
  struct Step {
    int joint;
    int tag;
    const char* name;
  };
  static const Step seq[] = {
      {2, 1, "J3"},
      {1, 1, "J2"},
      {3, 0, "J4"},
      {4, 0, "J5"},
      {5, 2, "J6"},
      {0, 0, "J1"},
  };

  // Helper: print one joint's status line.
  // mode_t values: MIT=0, POS_VEL=1, VEL=2, FORCE_POS=3
  // master_id = configured RX/feedback CAN ID (unambiguous); fb_id = lower nibble from frame.
  static const char* mode_name[] = {"MIT", "POS_VEL", "VEL", "FORCE_POS"};
  auto print_joint = [&](const char* name, uint16_t master_id, uint8_t fb_id, uint8_t err,
                         control::mode_t mode, bool conn,
                         float theta, float target, float omega, float torque,
                         bool active) {
    print(
        "  %s [master=0x%02X fb_id=0x%02X err=0x%X mode=%-8s conn=%d] "
        "pos=%+.3f tgt=%+.3f err=%+.3f  vel=%+.3f  trq=%+.3f%s\r\n",
        name, (unsigned)master_id, (unsigned)fb_id, (unsigned)err,
        mode < 4 ? mode_name[mode] : "?",
        (int)conn, theta, target, target - theta, omega, torque,
        active ? "  << ACTIVE" : "");
  };

  for (auto& step : seq) {
    print("ARM HOME: moving %s to 0...\r\n", step.name);
    uint32_t deadline = HAL_GetTick() + timeout_ms;
    uint32_t next_print = HAL_GetTick();  // print immediately on entry

    while (HAL_GetTick() < deadline) {
      // Snapshot targets once so SetOutput and the print block agree.
      // Active joint → 0.0 rad; all others → hold at current position.
      const float tgt1 = (step.joint == 0 ? 0.0f : arm_j1->GetTheta());
      const float tgt2 = (step.joint == 1 ? 0.0f : arm_j2->GetTheta());
      const float tgt3 = (step.joint == 2 ? 0.0f : arm_j3->GetTheta());
      const float tgt4 = (step.joint == 3 ? 0.0f : arm_j4->GetTheta());
      const float tgt5 = (step.joint == 4 ? 0.0f : arm_j5->GetTheta());
      const float tgt6 = (step.joint == 5 ? 0.0f : arm_j6->GetTheta());

      // J1, J4, J5 — Motor4310 POS_VEL.
      arm_j1->SetOutput(tgt1, ARM_VEL_LIM[0]);
      arm_j4->SetOutput(tgt4, ARM_VEL_LIM[3]);
      arm_j5->SetOutput(tgt5, ARM_VEL_LIM[4]);
      // J2, J3 — MotorDMJ10010 FORCE_POS
      arm_j2->SetOutput(tgt2, ARM_VEL_LIM[1], J23_CURRENT_LIM);
      arm_j3->SetOutput(tgt3, ARM_VEL_LIM[2], J23_CURRENT_LIM);
      // J6 — MotorDMJ3507 FORCE_POS
      arm_j6->SetOutput(tgt6, ARM_VEL_LIM[5], J6_CURRENT_LIM);

      control::Motor4310* j145[] = {arm_j1, arm_j4, arm_j5};
      control::MotorDMJ10010* j23[] = {arm_j2, arm_j3};
      control::MotorDMJ3507* j6arr[] = {arm_j6};
      control::Motor4310::TransmitOutput(j145, 3);
      control::MotorDMJ10010::TransmitOutput(j23, 2);
      control::MotorDMJ3507::TransmitOutput(j6arr, 1);

      // Print all joint statuses at ~5 Hz.
      if (HAL_GetTick() >= next_print) {
        next_print = HAL_GetTick() + 200;
        print("-- ARM HOME status (homing %s) --\r\n", step.name);
        // tgt1..tgt6 already computed above — reuse them here.
        print_joint("J1", J1_MASTER_ID, arm_j1->GetMotorID(), arm_j1->GetErr(), arm_j1->GetMode(),
                    arm_j1->connection_flag_,
                    arm_j1->GetTheta(), tgt1, arm_j1->GetOmega(), arm_j1->GetTorque(),
                    step.joint == 0);
        print_joint("J2", J2_MASTER_ID, arm_j2->GetMotorID(), arm_j2->GetErr(), arm_j2->GetMode(),
                    arm_j2->connection_flag_,
                    arm_j2->GetTheta(), tgt2, arm_j2->GetOmega(), arm_j2->GetTorque(),
                    step.joint == 1);
        print_joint("J3", J3_MASTER_ID, arm_j3->GetMotorID(), arm_j3->GetErr(), arm_j3->GetMode(),
                    arm_j3->connection_flag_,
                    arm_j3->GetTheta(), tgt3, arm_j3->GetOmega(), arm_j3->GetTorque(),
                    step.joint == 2);
        print_joint("J4", J4_MASTER_ID, arm_j4->GetMotorID(), arm_j4->GetErr(), arm_j4->GetMode(),
                    arm_j4->connection_flag_,
                    arm_j4->GetTheta(), tgt4, arm_j4->GetOmega(), arm_j4->GetTorque(),
                    step.joint == 3);
        print_joint("J5", J5_MASTER_ID, arm_j5->GetMotorID(), arm_j5->GetErr(), arm_j5->GetMode(),
                    arm_j5->connection_flag_,
                    arm_j5->GetTheta(), tgt5, arm_j5->GetOmega(), arm_j5->GetTorque(),
                    step.joint == 4);
        print_joint("J6", J6_MASTER_ID, arm_j6->GetMotorID(), arm_j6->GetErr(), arm_j6->GetMode(),
                    arm_j6->connection_flag_,
                    arm_j6->GetTheta(), tgt6, arm_j6->GetOmega(), arm_j6->GetTorque(),
                    step.joint == 5);
      }

      // Check settle condition for the active joint.
      float theta = 0.0f;
      switch (step.joint) {
        case 0:
          theta = arm_j1->GetTheta();
          break;
        case 1:
          theta = arm_j2->GetTheta();
          break;
        case 2:
          theta = arm_j3->GetTheta();
          break;
        case 3:
          theta = arm_j4->GetTheta();
          break;
        case 4:
          theta = arm_j5->GetTheta();
          break;
        case 5:
          theta = arm_j6->GetTheta();
          break;
      }
      if (fabsf(theta) < thresh_rad) break;

      osDelay(5);
    }

    if (HAL_GetTick() >= deadline)
      print("ARM HOME: %s timed out (theta=%.3f rad)\r\n", step.name,
            [&]() -> float {
              switch (step.joint) {
                case 0:
                  return arm_j1->GetTheta();
                case 1:
                  return arm_j2->GetTheta();
                case 2:
                  return arm_j3->GetTheta();
                case 3:
                  return arm_j4->GetTheta();
                case 4:
                  return arm_j5->GetTheta();
                default:
                  return arm_j6->GetTheta();
              } 
            }());
    else
      print("ARM HOME: %s at zero\r\n", step.name);
  }

  // Sync cmd_target_deg so subsequent ArmUpdate() calls hold at zero.
  for (int i = 0; i < 6; ++i) cmd_target_deg[i] = 0.0f;
  arm_homed = true;  // enable position-limit enforcement in ArmUpdate()
  print("ARM HOME: sequence complete\r\n");
}

// ── ArmUpdate ─────────────────────────────────────────────────────────────────

/**
 * @brief Motor-control tick — runs at ~200 Hz from the default RTOS task.
 *
 * Flow:
 *   1. Test mode (read-only) → early return
 *   2. Watchdog timeout → start non-blocking park
 *   3. Park state machine → drive J2/J3 to safe positions, then disable
 *   4. Bumpless enable → auto-enable on first valid UART frame
 *   5. Motor commands → set position/velocity targets for J1–J6
 *   6. Gripper FSM → close-then-hold
 *   7. CAN transmit
 *
 * UART RX/TX runs in ArmUartTask (arm_uart_task.cc), a separate RTOS thread.
 *
 * @param test_mode  If true, poll encoders only — no motor commands, no watchdog.
 */

void ArmUpdate(bool test_mode) {
  // ── 1. Test mode: read encoders without driving motors ───────────────────

  if (test_mode) {
    if (!arm_enabled) ArmEnable();

    // MotorDisable solicits CAN feedback without commanding motion.
    arm_j1->MotorDisable();
    arm_j2->MotorDisable();
    arm_j3->MotorDisable();
    arm_j4->MotorDisable();
    arm_j5->MotorDisable();
    arm_j6->MotorDisable();

    if (HAL_GetTick() % 1000 < 5) {
      print("ARM TEST | J1=%6.2f J2=%6.2f J3=%6.2f J4=%6.2f J5=%6.2f J6=%6.2f [rad]\r\n",
            arm_j1->GetTheta(), arm_j2->GetTheta(),
            arm_j3->GetTheta(), arm_j4->GetTheta(),
            arm_j5->GetTheta(), arm_j6->GetTheta());
    }

    return;  // nothing else to do in test mode
  }

  // ── 2. Watchdog → start non-blocking park ────────────────────────────────
  if (arm_enabled && !arm_parking &&
      (HAL_GetTick() - last_valid_rx_tick > WATCHDOG_MS)) {
    print("ARM WATCHDOG: no UART for >%lu ms — parking before disable\r\n",
          (unsigned long)WATCHDOG_MS);
    arm_parking = true;
    park_deadline = HAL_GetTick() + PARK_TIMEOUT_MS;
    park_hold_j1 = arm_j1->GetTheta();
    park_hold_j2 = arm_j2->GetTheta();
    park_hold_j3 = arm_j3->GetTheta();
    park_hold_j4 = arm_j4->GetTheta();
    park_hold_j5 = arm_j5->GetTheta();
    park_hold_j6 = arm_j6->GetTheta();
    gripper->SetOutput(0);
    control::MotorCANBase* grip_arr[] = {gripper};
    control::MotorCANBase::TransmitOutput(grip_arr, 1);
  }

  // ── 3. Park state machine ────────────────────────────────────────────────
  if (arm_parking) {
    // Abort park if UART resumes.
    if (HAL_GetTick() - last_valid_rx_tick < WATCHDOG_MS) {
      print("ARM PARK: UART resumed — aborting park\r\n");
      arm_parking = false;
      // Fall through to normal control below.
    } else {
      // A joint is done when it has moved to OR already past the park target
      // (theta <= target + threshold), matching the SetOutput logic below.
      bool j2_ok = arm_j2->GetTheta() <= PARK_J2_RAD + PARK_THRESH_RAD;
      bool j3_ok = arm_j3->GetTheta() <= PARK_J3_RAD + PARK_THRESH_RAD;
      bool timed_out = HAL_GetTick() >= park_deadline;

      if ((j2_ok && j3_ok) || timed_out) {
        print(timed_out ? "ARM PARK: timeout — forcing disable\r\n"
                        : "ARM PARK: J2/J3 settled — disabling\r\n");
        arm_j1->MotorDisable();
        arm_j2->MotorDisable();
        arm_j3->MotorDisable();
        arm_j4->MotorDisable();
        arm_j5->MotorDisable();
        arm_j6->MotorDisable();
        arm_enabled = false;
        arm_parking = false;
        arm_homed = false;  // arm may be moved by hand after disable
        return;
      }

      // Drive J2/J3 to park positions, hold everything else.
      arm_j1->SetOutput(park_hold_j1, ARM_VEL_LIM[0]);
      arm_j2->SetOutput(arm_j2->GetTheta()>PARK_J2_RAD ? PARK_J2_RAD:park_hold_j2, ARM_VEL_LIM[1], J23_CURRENT_LIM);
      arm_j3->SetOutput(arm_j3->GetTheta() > PARK_J3_RAD? PARK_J3_RAD:park_hold_j3, ARM_VEL_LIM[2], J23_CURRENT_LIM);
      arm_j4->SetOutput(park_hold_j4, ARM_VEL_LIM[3]);
      arm_j5->SetOutput(park_hold_j5, ARM_VEL_LIM[4]);
      arm_j6->SetOutput(park_hold_j6, ARM_VEL_LIM[5], J6_CURRENT_LIM);

      control::Motor4310* j145[] = {arm_j1, arm_j4, arm_j5};
      control::MotorDMJ10010* j23[] = {arm_j2, arm_j3};
      control::MotorDMJ3507* j6arr[] = {arm_j6};
      control::Motor4310::TransmitOutput(j145, 3);
      control::MotorDMJ10010::TransmitOutput(j23, 2);
      control::MotorDMJ3507::TransmitOutput(j6arr, 1);

      if (HAL_GetTick() % 500 < 5)
        print("ARM PARK: J2=%.3f→%.1f%s J3=%.3f→%.1f%s\r\n",
              arm_j2->GetTheta(),
              arm_j2->GetTheta() > PARK_J2_RAD ? PARK_J2_RAD : park_hold_j2,
              arm_j2->GetTheta() > PARK_J2_RAD ? "" : "(hold)",
              arm_j3->GetTheta(),
              arm_j3->GetTheta() > PARK_J3_RAD ? PARK_J3_RAD : park_hold_j3,
              arm_j3->GetTheta() > PARK_J3_RAD ? "" : "(hold)");
      return;
    }
  }

  // ── 4. Bumpless enable on first valid UART frame ─────────────────────────
  // Runs here (not in UART task) so it respects the kill switch — ArmUpdate
  // is not called when swr == DOWN.
  if (!arm_enabled && last_valid_rx_tick != 0 &&
      (HAL_GetTick() - last_valid_rx_tick < WATCHDOG_MS)) {
    for (int i = 0; i < 6; ++i) {
      float theta = 0.0f;
      switch (i) {
        case 0:
          theta = arm_j1->GetTheta();
          break;
        case 1:
          theta = arm_j2->GetTheta();
          break;
        case 2:
          theta = arm_j3->GetTheta();
          break;
        case 3:
          theta = arm_j4->GetTheta();
          break;
        case 4:
          theta = arm_j5->GetTheta();
          break;
        case 5:
          theta = arm_j6->GetTheta();
          break;
      }
      cmd_target_deg[i] = theta * RAD2DEG;
    }
    print("ARM: UART active — bumpless enable\r\n");
    ArmEnable();
  }

  if (!arm_enabled) return;

  // ── 5. Motor commands ────────────────────────────────────────────────────
  // After homing, clamp to per-joint motor-angle limits (second defence after
  // the RX filter).  Before homing the arm may legitimately sit outside the
  // operational range (startup / post-disable), so clamping is skipped then —
  // applying it would snap the arm to the limit boundary on first enable.
  const float t0 = arm_homed ? clamp((float)cmd_target_deg[0], ARM_CMD_MIN_DEG[0], ARM_CMD_MAX_DEG[0]) : (float)cmd_target_deg[0];
  const float t1 = arm_homed ? clamp((float)cmd_target_deg[1], ARM_CMD_MIN_DEG[1], ARM_CMD_MAX_DEG[1]) : (float)cmd_target_deg[1];
  const float t2 = arm_homed ? clamp((float)cmd_target_deg[2], ARM_CMD_MIN_DEG[2], ARM_CMD_MAX_DEG[2]) : (float)cmd_target_deg[2];
  const float t3 = arm_homed ? clamp((float)cmd_target_deg[3], ARM_CMD_MIN_DEG[3], ARM_CMD_MAX_DEG[3]) : (float)cmd_target_deg[3];
  const float t4 = arm_homed ? clamp((float)cmd_target_deg[4], ARM_CMD_MIN_DEG[4], ARM_CMD_MAX_DEG[4]) : (float)cmd_target_deg[4];
  const float t5 = arm_homed ? clamp((float)cmd_target_deg[5], ARM_CMD_MIN_DEG[5], ARM_CMD_MAX_DEG[5]) : (float)cmd_target_deg[5];
  arm_j1->SetOutput(t0 * DEG2RAD, ARM_VEL_LIM[0]);
  arm_j2->SetOutput(t1 * DEG2RAD, ARM_VEL_LIM[1], J23_CURRENT_LIM);
  arm_j3->SetOutput(t2 * DEG2RAD, ARM_VEL_LIM[2], J23_CURRENT_LIM);
  arm_j4->SetOutput(t3 * DEG2RAD, ARM_VEL_LIM[3]);
  arm_j5->SetOutput(t4 * DEG2RAD, ARM_VEL_LIM[4]);
  arm_j6->SetOutput(t5 * DEG2RAD, ARM_VEL_LIM[5], J6_CURRENT_LIM);
  
  // ── 6. Gripper FSM (edge-triggered via BoolEdgeDetector) ────────────────
  //   MID  posEdge → start opening
  //   DOWN posEdge → start closing (HOLDING is preserved)
  //   UP   posEdge → cancel active motion; IDLE
  // Stall while opening → IDLE (stable: no new posEdge while MID stays held)
  static BoolEdgeDetector swl_mid(dbus->swl == remote::MID);
  static BoolEdgeDetector swl_down(dbus->swl == remote::DOWN);
  static BoolEdgeDetector swl_up(dbus->swl == remote::UP);

  swl_mid.input(dbus->swl == remote::MID);
  swl_down.input(dbus->swl == remote::DOWN);
  swl_up.input(dbus->swl == remote::UP);

  if (swl_mid.posEdge()) {
    grip_open_stall_count = 0;
    grip_state = GripState::OPENING;
  } else if (swl_down.posEdge()) {
    if (grip_state != GripState::CLOSING && grip_state != GripState::HOLDING) {
      grip_state = GripState::CLOSING;
    }
  } else if (swl_up.posEdge()) {
    if (grip_state == GripState::OPENING || grip_state == GripState::CLOSING) {
      grip_state = GripState::IDLE;
    }
  }

  switch (grip_state) {
    case GripState::IDLE:
      grip_open_stall_count = 0;
      gripper->SetOutput(0);
      break;
    case GripState::OPENING:
      gripper->SetOutput(GRIP_OPEN_CURRENT);
      // Open current is negative; stall drives current further negative.
      if (gripper->GetCurr() < -GRIP_STALL_THRESH) {
        if (++grip_open_stall_count >= GRIP_STALL_DEBOUNCE) {
          grip_open_stall_count = 0;
          grip_state = GripState::IDLE;
          print("GRIPPER: open stall — idle\r\n");
        }
      } else {
        grip_open_stall_count = 0;
      }
      print("gripper current: %d \r\n", gripper->GetCurr());
      break;
    case GripState::CLOSING:
      gripper->SetOutput(GRIP_CLOSE_CURRENT);
      print("gripper current: %d \r\n",gripper->GetCurr());
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

  // ── 7. CAN transmit ─────────────────────────────────────────────────────
  control::Motor4310* j145[] = {arm_j1, arm_j4, arm_j5};
  control::MotorDMJ10010* j23[] = {arm_j2, arm_j3};
  control::MotorDMJ3507* j6arr[] = {arm_j6};
  control::Motor4310::TransmitOutput(j145, 3);
  control::MotorDMJ10010::TransmitOutput(j23, 2);
  control::MotorDMJ3507::TransmitOutput(j6arr, 1);
  // TODO: enable gripper CAN when ready
  control::MotorCANBase* grip[] = {gripper};  
  control::MotorCANBase::TransmitOutput(grip, 1);
}
