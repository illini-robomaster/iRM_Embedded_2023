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
 * @brief Engineer 2026 chassis controller for DM_MC_02 board.
 *
 * Chassis layout (top-down, front = positive X):
 *
 *          FRONT
 *   [FL swerve]  [FR swerve]     ← Motor3508 (drive) + Motor6020 (steer)
 *
 *   [RL omni  ]  [RR omni  ]     ← MotorDM3519 (forward/backward only)
 *          REAR
 *
 * Coordinate convention (robot frame):
 *   X = forward (positive), Y = left (positive), Z = up (CCW positive)
 *
 * Hardware (DM_MC_02 board):
 *   CAN  : hfdcan1  (all motors)
 *   DBUS : huart5   (remote controller)
 *   Print: huart10  (debug serial)
 *
 * DBUS channel mapping:
 *   ch0 → lateral   (vy): right stick horizontal, right = negative
 *   ch1 → forward   (vx): right stick vertical,   push forward = positive
 *   ch2 → rotation  (vw): left  stick horizontal,  push left   = positive (CCW)
 *   swr → DOWN = disabled, MID = enabled + lift down, UP = enabled + lift up
 *   swl → UP   = arm test mode (home all joints to 0°, no OrangePi needed)
 *          MID/DOWN = normal operation (arm commanded by OrangePi UART)
 */

#include <cmath>

#include "arm_mc02.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "main.h"
#include "motor.h"
#include "steering_6020.h"

#define ANGLE_READ

#ifdef ANGLE_READ
bool test_mode_printed = true;
#else
bool test_mode_printed = false;
#endif

// ── Chassis geometry ────────────────────────────────────────────────────────
// Front swerve modules are 40 cm apart → HALF_TRACK_FRONT = 0.20 m
// Rear omni wheels are 51 cm apart    → HALF_TRACK_REAR  = 0.255 m
// Front-to-rear axle distance is 40 cm → HALF_WHEELBASE  = 0.20 m
static const float HALF_WHEELBASE   = 0.20f;  // half front-to-rear axle distance [m]
static const float HALF_TRACK_FRONT = 0.20f;  // half track width at front swerve modules [m]
static const float HALF_TRACK_REAR  = 0.255f; // half track width at rear omni wheels [m]

// ── Input velocity limits ───────────────────────────────────────────────────
// M3508 max output-shaft speed ≈ 482 RPM → 50.5 rad/s, wheel radius 0.06 m
// → theoretical max wheel speed ≈ 3.0 m/s.  Start conservatively.
static const float VX_MAX = 1.5f;   // max forward speed  [m/s]
static const float VY_MAX = 1.5f;   // max lateral speed  [m/s]
static const float VW_MAX = 2.0f;   // max yaw rate       [rad/s]

// ── Rear DM3519 (VEL mode — setpoint unit is rotor speed scaled by the
//    ORIGINAL gearbox ratio 3591/187, NOT the replacement gearbox 268/17) ────
//
// The DM3519 encoder sits on the motor rotor.  In VEL mode the controller
// commands/reports "output shaft speed" computed as:
//   omega_reported = rotor_speed / (3591/187)   ← original-gearbox assumption
//
// With the replacement gearbox (268/17) the actual wheel speed is:
//   actual_output = rotor_speed / (268/17)
//                 = SetOutput_value × (3591/187) / (268/17)
//                 = SetOutput_value × original_ratio / new_ratio
//
// To achieve desired linear wheel speed L [m/s]:
//   SetOutput = (L / wheel_radius) × (new_ratio / original_ratio)
//
// REAR_VEL_SCALE = new_ratio / (original_ratio × wheel_radius)
//                = (268/17) / ((3591/187) × 0.07)
//                ≈ 11.73  [rad/s_cmd per (m/s)]
// Negate if a motor spins in the wrong direction.
static const float REAR_VEL_SCALE = (268.0f / 17.0f) / ((3591.0f / 187.0f) * 0.07f);

// ── Front M3508 drive velocity PID ─────────────────────────────────────────
// Motor3508::GetOmegaDelta() works in rad/s of the *rotor* shaft (motor.cc:
//   OMEGA_SCALE = 2π/60, so omega_ = raw_rpm * 2π/60).
// Conversion: linear speed [m/s] → rotor speed [rad/s]
//   = (v / WHEEL_RADIUS_FRONT) * GEAR_RATIO_3508
//   = (v / 0.06) * (3591/187)   ← wheel radius from engineer_steering.h,
//                                   gear ratio from Omni/main.cc
//   ≈ v * 320.1
static const float DRIVE_VEL_SCALE = (3591.0f / 187.0f) / 0.06f;  // ≈ 320.1 [rad_rotor/s / (m/s)]
static const float DRIVE_KP        = 40.0f;   // starting point — tune on the bench
static const float DRIVE_KI        = 0.0f;
static const float DRIVE_KD        = 5.0f;

// ── Front Motor6020 steering (Steering6020 omega-PID) ───────────────────────
// Values taken directly from vehicles/Engineer/chassis/src/chassis_task.cc
// and vehicles/Engineer/chassis/include/chassis_task.h.
static const float STEER_KP           = 80000.0f;
static const float STEER_KI           = 0.0f;
static const float STEER_KD           = 2000.0f;
static const float STEER_MAX_IOUT     = 30000.0f;
static const float STEER_MAX_OUT      = 15000.0f;
static const float STEER_MAX_SPEED    = 4.0f * (float)M_PI;    // RUN_SPEED   [rad/s]
static const float STEER_ACCELERATION = 100.0f * (float)M_PI;  // ACCELERATION [rad/s²]

// Physical install-angle offset of each steering module [rad].
// Measure by pointing both wheels straight forward and reading GetTheta().
// TODO: calibrate.
static const float FL_STEER_OFFSET = 2.837f;
static const float FR_STEER_OFFSET = 4.389f;

// Steering deadzone: hold current steer angle when commanded module speed
// is below this threshold to avoid atan2(0,0) instability [m/s].
static const float STEER_SPEED_DEADZONE = 0.05f;

// Lift soft-down threshold: when the lift angle is within this value of 0,
// the current limit is set to 0 so the motor does not fight gravity.
static const float LIFT_SOFT_DOWN_THRESHOLD = 0.02f;  // [rad]

// ── Global peripherals ──────────────────────────────────────────────────────
static bsp::CAN*     can  = nullptr;
static remote::DBUS* dbus = nullptr;

// Rear omniwheels (DM3519, velocity-controlled)
static control::MotorDM3519* rear_left_motor  = nullptr;
static control::MotorDM3519* rear_right_motor = nullptr;

// Lift motor (DM-J10010L-2EC, FORCE_POS mode)
// swr MID → pos = 0 rad (chassis low), swr UP → pos = -1 rad (chassis raised)
static control::MotorDMJ10010* lift_motor = nullptr;

// Front swerve — drive motors (Motor3508, current-controlled via velocity PID)
// TODO: replace CAN IDs (0x201–0x208) with your actual IDs.
static control::Motor3508* front_left_drive  = nullptr;
static control::Motor3508* front_right_drive = nullptr;

// Front swerve — steering motors (Motor6020 wrapped by Steering6020)
// Keep raw pointers so we can call MotorCANBase::TransmitOutput() on them.
static control::Motor6020*    front_left_steer_raw  = nullptr;
static control::Motor6020*    front_right_steer_raw = nullptr;
static control::Steering6020* front_left_steer  = nullptr;
static control::Steering6020* front_right_steer = nullptr;

// ── RTOS Init ───────────────────────────────────────────────────────────────

void RM_RTOS_Init() {
  // print_use_uart(&huart7);   // ST-Link VCP → /dev/ttyACM0 @ 921600 baud
  print_use_usb();  // USB CDC ACM → /dev/ttyUSB0 @ 115200 baud
  // MC02 uses FDCAN instead of classic CAN.
  can  = new bsp::CAN(&hfdcan1, 0);
  dbus = new remote::DBUS(&huart5);

  // ── Rear omniwheels (DM3519, VEL mode) ────────────────────────────────
  rear_left_motor  = new control::MotorDM3519(can, 0x20, 0x21, control::VEL);
  rear_right_motor = new control::MotorDM3519(can, 0x22, 0x23, control::VEL);

  // ── Lift motor (DM-J10010L-2EC, FORCE_POS mode) ───────────────────────
  // Velocity limit 0.5 rad/s keeps the lift slow and safe.
  // Current limit 0.5 (= 50% of 99.74 A max) — tune down if the motor gets warm.
  lift_motor = new control::MotorDMJ10010(can, 0x30, 0x31, control::FORCE_POS);

  // ── Front swerve drives (Motor3508) ───────────────────────────────────
  front_left_drive  = new control::Motor3508(can, 0x202);
  front_right_drive = new control::Motor3508(can, 0x201);

  // ── Front swerve steering (Motor6020 + Steering6020) ──────────────────
  front_left_steer_raw  = new control::Motor6020(can, 0x206);
  front_right_steer_raw = new control::Motor6020(can, 0x205);

  // Steering6020 overrides the Motor6020 CAN RX callback internally, so
  // motor data is updated through steering->UpdateData() each CAN frame.
  control::steering6020_t steer_cfg;
  steer_cfg.max_speed          = STEER_MAX_SPEED;
  steer_cfg.max_acceleration   = STEER_ACCELERATION;
  steer_cfg.transmission_ratio = 1.0f;
  steer_cfg.max_iout           = STEER_MAX_IOUT;
  steer_cfg.max_out            = STEER_MAX_OUT;

  // Each Steering6020 instance needs its own PID parameter array.
  steer_cfg.motor          = front_left_steer_raw;
  steer_cfg.install_offset = FL_STEER_OFFSET;
  steer_cfg.omega_pid_param = new float[3]{STEER_KP, STEER_KI, STEER_KD};
  front_left_steer = new control::Steering6020(steer_cfg);

  steer_cfg.motor          = front_right_steer_raw;
  steer_cfg.install_offset = FR_STEER_OFFSET;
  steer_cfg.omega_pid_param = new float[3]{STEER_KP, STEER_KI, STEER_KD};
  front_right_steer = new control::Steering6020(steer_cfg);

  // ── Arm controller (hfdcan2 + huart10) ────────────────────────────────
  ArmInit();
}

void RM_RTOS_Threads_Init(void) {
  // No extra threads — all control runs in the default task.
}

// ── Default Task ────────────────────────────────────────────────────────────

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  // Convenience arrays for bulk TransmitOutput calls.
  control::MotorDM3519*   rear_motors[]  = {rear_left_motor, rear_right_motor};
  control::MotorCANBase*  drive_motors[] = {front_left_drive, front_right_drive};
  control::MotorCANBase*  steer_motors[] = {front_left_steer_raw, front_right_steer_raw};
  control::MotorDMJ10010* lift_motors[]  = {lift_motor};

  // Velocity PIDs for the front M3508 drive motors.
  control::PIDController drive_pid_fl(DRIVE_KP, DRIVE_KI, DRIVE_KD);
  control::PIDController drive_pid_fr(DRIVE_KP, DRIVE_KI, DRIVE_KD);

  // ── Steer-offset calibration ─────────────────────────────────────────
  // With swr DOWN the robot is safe to handle.
  // 1. Manually rotate each front swerve module so its wheel points straight FORWARD.
  // 2. Read the printed FL/FR values below.
  // 3. Copy them into FL_STEER_OFFSET / FR_STEER_OFFSET at the top of this file.
  // 4. Re-flash, then lift swr to enable.
  print("=== CALIBRATION MODE (swr DOWN) ===\r\n");
  print("Point both front wheels straight FORWARD, then read offsets:\r\n");
  while (dbus->swr == remote::DOWN) {
    print("  FL_STEER_OFFSET = %.4ff;  FR_STEER_OFFSET = %.4ff;\r\n",
          front_left_steer_raw->GetTheta(),
          front_right_steer_raw->GetTheta());
    osDelay(500);
  }
  print("=== READY — flip swr UP to enable ===\r\n");

  // ── Connection check: block until every motor has sent ≥1 CAN frame ──
  // Keep swr DOWN while powering motors — robot is safe to handle.
  // Once all connection_flag_ are set, proceed to the enable sequence.
  // checkAllMotorsConnected(rear_left_motor, rear_right_motor, lift_motor);

  bool enabled = false;

  while (true) {
    // ── Kill switch: swr DOWN → disable all motors ─────────────────────
    if (dbus->swr == remote::DOWN) {
      if (enabled) {
        rear_left_motor->MotorDisable();
        rear_right_motor->MotorDisable();
        lift_motor->MotorDisable();

        front_left_drive->SetOutput(0);
        front_right_drive->SetOutput(0);
        control::MotorCANBase::TransmitOutput(drive_motors, 2);
        control::MotorCANBase::TransmitOutput(steer_motors, 2);
        control::MotorDM3519::TransmitOutput(rear_motors, 2);

        enabled = false;
        print("Disabled\r\n");
      }
      // Print raw steer angles at 2 Hz while disabled for re-calibration.
      if (HAL_GetTick() % 500 < 100) {
        print("  FL_STEER_OFFSET = %.4ff;  FR_STEER_OFFSET = %.4ff;\r\n",
              front_left_steer_raw->GetTheta(),
              front_right_steer_raw->GetTheta());
      }
      osDelay(100);
      continue;
    } else {
      if (!enabled) {
        // DM3519 enable is blocking — it waits for CAN feedback from each
        // motor.  Print before each call so you can see which one hangs if
        // a motor is not powered or has a wrong CAN ID.
        print("Enabling RL motor (0x20)...\r\n");
        rear_left_motor->SetZeroPos();
        rear_left_motor->MotorEnable();
        print("RL OK. Enabling RR motor (0x22)...\r\n");
        rear_right_motor->SetZeroPos();
        rear_right_motor->MotorEnable();
        print("RR OK. Enabling lift motor (0x30)...\r\n");
        lift_motor->MotorEnable();
        print("Lift OK. Enabled\r\n");
        enabled = true;
        osDelay(100);
      }
    }

    // ── Read DBUS inputs ───────────────────────────────────────────────
    // TODO: verify sign conventions match your physical robot orientation.
    float vx = clip<float>( dbus->ch1 / 660.0f * VX_MAX, -VX_MAX,  VX_MAX);  // forward
    float vy = clip<float>(-dbus->ch0 / 660.0f * VY_MAX, -VY_MAX,  VY_MAX);  // lateral
    float vw = clip<float>(-dbus->ch2 / 660.0f * VW_MAX, -VW_MAX,  VW_MAX);  // rotation

    // ── Rear omni kinematics ──────────────────────────────────────────
    // The rear omniwheels are fixed in the longitudinal (X) direction.
    // Their rollers passively absorb lateral forces.
    //
    // Module positions (robot frame):
    //   Rear-left  (RL): px = -HALF_WHEELBASE, py = +HALF_TRACK_REAR
    //   Rear-right (RR): px = -HALF_WHEELBASE, py = -HALF_TRACK_REAR
    //
    // Rotational contribution to longitudinal speed = -vw * py:
    //   RL: -vw * (+HALF_TRACK_REAR) = -vw * HALF_TRACK_REAR
    //   RR: -vw * (-HALF_TRACK_REAR) = +vw * HALF_TRACK_REAR
    // RR is negated: the right motor is mounted mirrored on the opposite side,
    // so its positive-spin direction is physically reversed relative to RL.
    float rl_speed =  (vx - vw * HALF_TRACK_REAR) * REAR_VEL_SCALE;  // [rad/s]
    float rr_speed = -(vx + vw * HALF_TRACK_REAR) * REAR_VEL_SCALE;  // [rad/s]

    // ── Front swerve kinematics ──────────────────────────────────────
    // Module positions (robot frame):
    //   Front-left  (FL): px = +HALF_WHEELBASE, py = +HALF_TRACK_FRONT
    //   Front-right (FR): px = +HALF_WHEELBASE, py = -HALF_TRACK_FRONT
    //
    // Total module velocity = translation + (rotation × position):
    //   v_module_x = vx - vw * py
    //   v_module_y = vy + vw * px
    float fl_vx = vx - vw * HALF_TRACK_FRONT;
    float fl_vy = vy + vw * HALF_WHEELBASE;

    float fr_vx = vx + vw * HALF_TRACK_FRONT;
    float fr_vy = vy + vw * HALF_WHEELBASE;

    float fl_speed = sqrtf(fl_vx * fl_vx + fl_vy * fl_vy);  // [m/s]
    float fr_speed = sqrtf(fr_vx * fr_vx + fr_vy * fr_vy);  // [m/s]

    // Swerve optimization: a wheel can spin either way, so we never need to
    // rotate more than 90°.  If the naive target is more than 90° from the
    // current angle, flip it by 180° and reverse the drive direction instead.
    float fl_raw_angle = atan2f(fl_vy, fl_vx);
    float fr_raw_angle = atan2f(fr_vy, fr_vx);

    // Wrap helper: fold angle difference into [-π, π].
    auto wrap_pi = [](float a) -> float {
      a = fmodf(a, 2.0f * (float)M_PI);
      if (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
      if (a < -(float)M_PI) a += 2.0f * (float)M_PI;
      return a;
    };

    float fl_diff = wrap_pi(fl_raw_angle - front_left_steer->GetTheta());
    float fr_diff = wrap_pi(fr_raw_angle - front_right_steer->GetTheta());

    float fl_opt_angle  = fl_raw_angle;
    float fr_opt_angle  = fr_raw_angle;
    float fl_drive_sign = 1.0f;
    float fr_drive_sign = 1.0f;

    if (fl_diff > (float)M_PI / 2.0f) {
      fl_opt_angle  = fl_raw_angle - (float)M_PI;
      fl_drive_sign = -1.0f;
    } else if (fl_diff < -(float)M_PI / 2.0f) {
      fl_opt_angle  = fl_raw_angle + (float)M_PI;
      fl_drive_sign = -1.0f;
    }
    if (fr_diff > (float)M_PI / 2.0f) {
      fr_opt_angle  = fr_raw_angle - (float)M_PI;
      fr_drive_sign = -1.0f;
    } else if (fr_diff < -(float)M_PI / 2.0f) {
      fr_opt_angle  = fr_raw_angle + (float)M_PI;
      fr_drive_sign = -1.0f;
    }

    if (fl_speed > STEER_SPEED_DEADZONE)
      front_left_steer->SetTarget(fl_opt_angle);
    if (fr_speed > STEER_SPEED_DEADZONE)
      front_right_steer->SetTarget(fr_opt_angle);

    front_left_steer->CalcOutput();
    front_right_steer->CalcOutput();

    // Drive velocity: scale by the flip sign so the wheel rolls in the correct
    // direction when the steering module is pointed in the flipped direction.
    float fl_target_vel = fl_speed * DRIVE_VEL_SCALE * fl_drive_sign;
    float fr_target_vel = fr_speed * DRIVE_VEL_SCALE * fr_drive_sign;

    front_left_drive->SetOutput(
        drive_pid_fl.ComputeConstrainedOutput(front_left_drive->GetOmegaDelta(fl_target_vel)));
    front_right_drive->SetOutput(
        drive_pid_fr.ComputeConstrainedOutput(front_right_drive->GetOmegaDelta(fr_target_vel)));

    // ── Set rear DM3519 outputs ──────────────────────────────────────
    rear_left_motor->SetOutput(rl_speed);
    rear_right_motor->SetOutput(rr_speed);

    // ── Lift motor ───────────────────────────────────────────────────
    // swr MID → chassis down (pos = 0), swr UP → chassis raised (pos = -1 rad).
    // Soft-down: once the lift reaches the down position (|theta| < threshold),
    // drop the current limit to 0 so the motor stops fighting gravity.
    // While still descending (|theta| >= threshold), use normal current to move.
    float lift_pos = (dbus->swr == remote::UP) ? -1.0f : 0.0f;
    float lift_cur;
    if (dbus->swr == remote::UP) {
      lift_cur = 0.5f;  // lifting or holding up
    } else if (fabsf(lift_motor->GetTheta()) < LIFT_SOFT_DOWN_THRESHOLD) {
      lift_cur = 0.0f;  // at rest in down position — release force
    } else {
      lift_cur = 0.5f;  // still descending toward 0
    }
    lift_motor->SetOutput(lift_pos, 0.5f, lift_cur);

    // ── Transmit all CAN frames ──────────────────────────────────────
    control::MotorDM3519::TransmitOutput(rear_motors, 2);
    control::MotorCANBase::TransmitOutput(drive_motors, 2);
    control::MotorCANBase::TransmitOutput(steer_motors, 2);
    control::MotorDMJ10010::TransmitOutput(lift_motors, 1);

    // ── Arm controller tick ───────────────────────────────────────────
    // if in test mode the ArmUpdate() function will print the current joint angles without sending any commands, which is useful for verifying the arm's physical response and tuning the steering PID without needing the OrangePi or UART communication. In normal mode the ArmUpdate() function will read commands from the OrangePi and control the arm accordingly.
    ArmUpdate(test_mode_printed);

    // ── Debug print (~1 Hz) ──────────────────────────────────────────
    if (HAL_GetTick() % 1000 < 5) {
      print("vx=%.2f vy=%.2f vw=%.2f | RL=%.2f RR=%.2f rad/s\r\n",
            vx, vy, vw, rl_speed, rr_speed);
      print("  FL: %.2fm/s tgt=%.1fdeg cur=%.1fdeg | FR: %.2fm/s tgt=%.1fdeg cur=%.1fdeg\r\n",
            fl_speed,
            fl_opt_angle * 180.0f / (float)M_PI,
            front_left_steer->GetTheta()  * 180.0f / (float)M_PI,
            fr_speed,
            fr_opt_angle * 180.0f / (float)M_PI,
            front_right_steer->GetTheta() * 180.0f / (float)M_PI);
    }

    osDelay(5);  // 200 Hz control loop
  }
}
