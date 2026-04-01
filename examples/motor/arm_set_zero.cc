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
 * @brief Arm joint zero-position setter for Engineer 2026.
 *
 * Reads all 6 arm joint encoders and lets the operator set zero positions
 * interactively via the remote controller.
 *
 * Remote mapping:
 *   swr DOWN  — safe idle, motors NOT enabled (move arm by hand freely)
 *   swr MID   — enable motors; hold current positions; display angles
 *   swr UP    — SET ZERO on all joints simultaneously (one-shot per flip)
 *
 *   swl UP    — set zero for J1 only
 *   swl MID   — set zero for J2 + J3 only
 *   swl DOWN  — set zero for J4 + J5 + J6 only
 *   (edges detected — flip away and back to re-trigger)
 *
 * CAN IDs match the Engineer 2026 arm (arm_mc02.cc):
 *   J1: Motor4310     master=0x10  can=0x11   POS_VEL
 *   J2: MotorDMJ10010 master=0x12  can=0x13   FORCE_POS
 *   J3: MotorDMJ10010 master=0x20  can=0x21   FORCE_POS
 *   J4: Motor4310     master=0x16  can=0x17   POS_VEL
 *   J5: Motor4310     master=0x18  can=0x19   POS_VEL
 *   J6: MotorDMJ3507  master=0x14  can=0x15   FORCE_POS
 *
 * Hardware: DM_MC_02 board, arm CAN bus on hfdcan2.
 */

#include <cmath>

#include "bsp_print.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "main.h"
#include "motor.h"

// ── CAN IDs ──────────────────────────────────────────────────────────────────
static constexpr uint16_t J1_MASTER = 0x10, J1_CAN = 0x11;
static constexpr uint16_t J2_MASTER = 0x12, J2_CAN = 0x13;
static constexpr uint16_t J3_MASTER = 0x20, J3_CAN = 0x21;
static constexpr uint16_t J4_MASTER = 0x16, J4_CAN = 0x17;
static constexpr uint16_t J5_MASTER = 0x18, J5_CAN = 0x19;
static constexpr uint16_t J6_MASTER = 0x14, J6_CAN = 0x15;

// ── Motion limits while holding position ─────────────────────────────────────
static constexpr float VEL_LIM_4310  = 0.5f;  // [rad/s] Motor4310 hold velocity
static constexpr float VEL_LIM_J23   = 0.5f;  // [rad/s] DMJ10010 hold velocity
static constexpr float CUR_LIM_J23   = 0.3f;  // fraction of motor max [0, 1]
static constexpr float VEL_LIM_J6    = 0.5f;  // [rad/s] DMJ3507 hold velocity
static constexpr float CUR_LIM_J6    = 0.3f;  // fraction of motor max [0, 1]

static bsp::CAN*               can = nullptr;
static control::Motor4310*     joint_1  = nullptr;
static control::MotorDMJ10010* joint_2  = nullptr;
static control::MotorDMJ10010* joint_3  = nullptr;
static control::Motor4310*     joint_4  = nullptr;
static control::Motor4310*     joint_5  = nullptr;
static control::MotorDMJ3507*  joint_6  = nullptr;
static remote::DBUS*           dbus = nullptr;

void RM_RTOS_Init() {
  print_use_usb();
  can = new bsp::CAN(&hfdcan2, 0);
  dbus = new remote::DBUS(&huart5);

  joint_1 = new control::Motor4310(can,     J1_MASTER, J1_CAN, control::POS_VEL);
  joint_2 = new control::MotorDMJ10010(can, J2_MASTER, J2_CAN, control::FORCE_POS);
  joint_3 = new control::MotorDMJ10010(can, J3_MASTER, J3_CAN, control::FORCE_POS);
  joint_4 = new control::Motor4310(can,     J4_MASTER, J4_CAN, control::POS_VEL);
  joint_5 = new control::Motor4310(can,     J5_MASTER, J5_CAN, control::POS_VEL);
  joint_6 = new control::MotorDMJ3507(can,  J6_MASTER, J6_CAN, control::FORCE_POS);
}

// ── Transmit helpers ──────────────────────────────────────────────────────────

static void TransmitAll() {
  control::Motor4310*     j145[] = {joint_1, joint_4, joint_5};
  control::MotorDMJ10010* j23[]  = {joint_2, joint_3};
  control::MotorDMJ3507*  j6arr[] = {joint_6};
  control::Motor4310::TransmitOutput(j145, 3);
  control::MotorDMJ10010::TransmitOutput(j23, 2);
  control::MotorDMJ3507::TransmitOutput(j6arr, 1);
}

static void DisableAll() {
  joint_1->MotorDisable();
  joint_2->MotorDisable();
  joint_3->MotorDisable();
  joint_4->MotorDisable();
  joint_5->MotorDisable();
  joint_6->MotorDisable();
}

static void HoldAll() {
  joint_1->SetOutput(joint_1->GetTheta(), VEL_LIM_4310);
  joint_2->SetOutput(joint_2->GetTheta(), VEL_LIM_J23, CUR_LIM_J23);
  joint_3->SetOutput(joint_3->GetTheta(), VEL_LIM_J23, CUR_LIM_J23);
  joint_4->SetOutput(joint_4->GetTheta(), VEL_LIM_4310);
  joint_5->SetOutput(joint_5->GetTheta(), VEL_LIM_4310);
  joint_6->SetOutput(joint_6->GetTheta(), VEL_LIM_J6, CUR_LIM_J6);
  TransmitAll();
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  print("=== Arm Zero-Position Tool ===\r\n");
  print("swr DOWN   = safe idle (free-move by hand)\r\n");
  print("swr MID/UP = hold position + display angles\r\n");
  print("swr UP     = SET ZERO all joints (one-shot per flip)\r\n");
  print("swl UP     = SET ZERO J1 only\r\n");
  print("swl MID    = SET ZERO J2+J3 only\r\n");
  print("swl DOWN   = SET ZERO J4+J5+J6 only\r\n");
  print("Keep swr DOWN until you are ready.\r\n\r\n");

  bool enabled = false;

  // Edge detectors for zero triggers
  remote::switch_t prev_swr = dbus->swr;
  remote::switch_t prev_swl = dbus->swl;

  while (true) {
    const remote::switch_t swr = dbus->swr;
    const remote::switch_t swl = dbus->swl;

    // ── Kill switch ──────────────────────────────────────────────────────
    if (swr == remote::DOWN) {
      if (enabled) {
        DisableAll();
        enabled = false;
        print("Motors DISABLED — arm is free to move by hand.\r\n");
      }
      prev_swr = swr;
      prev_swl = swl;
      osDelay(50);
      continue;
    }

    // ── Enable on transition out of DOWN ────────────────────────────────
    if (!enabled) {
      print("Enabling motors...\r\n");
      joint_1->MotorEnable();
      joint_2->MotorEnable();
      joint_3->MotorEnable();
      joint_4->MotorEnable();
      joint_5->MotorEnable();
      joint_6->MotorEnable();
      enabled = true;
      print("Motors ENABLED — hold position active.\r\n");
    }

    // ── swr rising edge to UP → zero ALL joints ──────────────────────────
    if (prev_swr != remote::UP && swr == remote::UP) {
      joint_1->SetZeroPos();
      joint_2->SetZeroPos();
      joint_3->SetZeroPos();
      joint_4->SetZeroPos();
      joint_5->SetZeroPos();
      joint_6->SetZeroPos();
      print(">>> SET ZERO: all joints (J1–J6) zeroed at current position\r\n");
      HoldAll();  // hold at zero immediately
    }

    // ── swl edges → zero individual groups ──────────────────────────────
    if (prev_swl != remote::UP && swl == remote::UP) {
      joint_1->SetZeroPos();
      print(">>> SET ZERO: J1 zeroed at %.4f rad\r\n", joint_1->GetTheta());
    }
    if (prev_swl != remote::MID && swl == remote::MID) {
      joint_2->SetZeroPos();
      joint_3->SetZeroPos();
      print(">>> SET ZERO: J2 zeroed at %.4f rad, J3 zeroed at %.4f rad\r\n",
            joint_2->GetTheta(), joint_3->GetTheta());
    }
    if (prev_swl != remote::DOWN && swl == remote::DOWN) {
      joint_4->SetZeroPos();
      joint_5->SetZeroPos();
      joint_6->SetZeroPos();
      print(">>> SET ZERO: J4=%.4f J5=%.4f J6=%.4f rad zeroed\r\n",
            joint_4->GetTheta(), joint_5->GetTheta(), joint_6->GetTheta());
    }

    prev_swr = swr;
    prev_swl = swl;

    // ── Hold current position ────────────────────────────────────────────
    HoldAll();

    // ── Display at ~2 Hz ─────────────────────────────────────────────────
    if (HAL_GetTick() % 500 < 5) {
      print("J1=%+7.3f  J2=%+7.3f  J3=%+7.3f  J4=%+7.3f  J5=%+7.3f  J6=%+7.3f  [rad]\r\n",
            joint_1->GetTheta(), joint_2->GetTheta(), joint_3->GetTheta(),
            joint_4->GetTheta(), joint_5->GetTheta(), joint_6->GetTheta());
    }

    osDelay(5);  // 200 Hz loop
  }
}
