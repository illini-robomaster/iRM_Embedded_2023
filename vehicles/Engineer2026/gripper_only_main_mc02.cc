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
 * @brief Engineer 2026 gripper-only bench test for DM_MC_02.
 *
 * This keeps the same gripper control scheme used in arm_mc02.cc:
 *   - Motor2006 on hfdcan2, RX 0x206
 *   - swl MID  posedge -> OPENING  with constant open current
 *   - swl DOWN posedge -> CLOSING  with constant close current
 *   - close stall       -> HOLDING encoder angle with ConstrainedPID
 *   - swl UP   posedge -> cancel active motion; HOLDING is preserved
 *
 * Safety:
 *   - swr DOWN or stale DBUS -> zero current
 *   - swr MID/UP             -> enable the gripper test loop
 */

#include "main.h"

#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "dbus.h"
#include "fdcan.h"
#include "motor.h"
#include "usart.h"
#include "utils.h"

namespace {

static constexpr uint16_t GRIP_RX_ID = 0x206;

// Match the gripper current-drive scheme in arm_mc02.cc.
static constexpr int16_t GRIP_CLOSE_CURRENT = 16384;
static constexpr int16_t GRIP_OPEN_CURRENT = -16384;
static constexpr int16_t GRIP_STALL_THRESH = 4000;
static constexpr uint8_t GRIP_STALL_DEBOUNCE = 3;

// Same gains and current limits as the production arm gripper hold loop.
static constexpr float GRIP_KP = 3000.0f;
static constexpr float GRIP_KI = 0.0f;
static constexpr float GRIP_KD = 100.0f;
static constexpr float GRIP_PID_MAX_IOUT = 8000.0f;
static constexpr float GRIP_PID_MAX_OUT = 16384.0f;

static constexpr uint32_t DBUS_TIMEOUT_MS = 100;
static constexpr uint32_t STATUS_PERIOD_MS = 100;
static constexpr uint32_t WAIT_PRINT_PERIOD_MS = 1000;

enum class GripState { IDLE, OPENING, CLOSING, HOLDING };

static bsp::CAN* arm_can = nullptr;
static remote::DBUS* dbus = nullptr;
static control::Motor2006* gripper = nullptr;
static control::ConstrainedPID* grip_pid = nullptr;

static GripState grip_state = GripState::IDLE;
static float grip_hold_pos = 0.0f;
static uint8_t grip_open_stall_count = 0;

static BoolEdgeDetector* swl_mid_edge = nullptr;
static BoolEdgeDetector* swl_down_edge = nullptr;
static BoolEdgeDetector* swl_up_edge = nullptr;

const char* GripStateName(GripState state) {
  switch (state) {
    case GripState::IDLE:
      return "IDLE";
    case GripState::OPENING:
      return "OPENING";
    case GripState::CLOSING:
      return "CLOSING";
    case GripState::HOLDING:
      return "HOLDING";
  }
  return "?";
}

bool DbusHealthy() {
  return dbus != nullptr && dbus->connection_flag_ &&
         (HAL_GetTick() - dbus->timestamp < DBUS_TIMEOUT_MS);
}

void TransmitGripperOutput() {
  control::MotorCANBase* motors[] = {gripper};
  control::MotorCANBase::TransmitOutput(motors, 1);
}

void ResetGripperControl() {
  grip_state = GripState::IDLE;
  grip_hold_pos = gripper != nullptr ? gripper->GetTheta() : 0.0f;
  grip_open_stall_count = 0;
  if (grip_pid != nullptr) grip_pid->Reset();
  if (gripper != nullptr) gripper->SetOutput(0);
}

void ResetSwitchEdgeDetectors() {
  delete swl_mid_edge;
  delete swl_down_edge;
  delete swl_up_edge;

  const bool swl_is_mid = DbusHealthy() && dbus->swl == remote::MID;
  const bool swl_is_down = DbusHealthy() && dbus->swl == remote::DOWN;
  const bool swl_is_up = DbusHealthy() && dbus->swl == remote::UP;

  swl_mid_edge = new BoolEdgeDetector(swl_is_mid);
  swl_down_edge = new BoolEdgeDetector(swl_is_down);
  swl_up_edge = new BoolEdgeDetector(swl_is_up);
}

void PrintControlHelp() {
  print("=== Engineer2026 gripper-only test ===\r\n");
  print("CAN: hfdcan2 | Motor2006 RX: 0x%03X\r\n", GRIP_RX_ID);
  print("Safety: swr DOWN = zero current, swr MID/UP = enable test\r\n");
  print("Control: swl MID edge = open, swl DOWN edge = close, swl UP edge = cancel motion\r\n");
  print("Close stall -> hold encoder position with PID\r\n");
}

void UpdateGripperStateMachine() {
  swl_mid_edge->input(dbus->swl == remote::MID);
  swl_down_edge->input(dbus->swl == remote::DOWN);
  swl_up_edge->input(dbus->swl == remote::UP);

  if (swl_mid_edge->posEdge()) {
    grip_open_stall_count = 0;
    grip_state = GripState::OPENING;
    print("GRIP TEST: OPENING\r\n");
  } else if (swl_down_edge->posEdge()) {
    if (grip_state != GripState::CLOSING && grip_state != GripState::HOLDING) {
      grip_state = GripState::CLOSING;
      print("GRIP TEST: CLOSING\r\n");
    }
  } else if (swl_up_edge->posEdge()) {
    if (grip_state == GripState::OPENING || grip_state == GripState::CLOSING) {
      grip_state = GripState::IDLE;
      print("GRIP TEST: motion cancelled -> IDLE\r\n");
    }
  }

  switch (grip_state) {
    case GripState::IDLE:
      grip_open_stall_count = 0;
      gripper->SetOutput(0);
      break;

    case GripState::OPENING:
      gripper->SetOutput(GRIP_OPEN_CURRENT);
      if (gripper->GetCurr() < -GRIP_STALL_THRESH) {
        if (++grip_open_stall_count >= GRIP_STALL_DEBOUNCE) {
          grip_open_stall_count = 0;
          grip_state = GripState::IDLE;
          print("GRIP TEST: open stall -> IDLE\r\n");
        }
      } else {
        grip_open_stall_count = 0;
      }
      break;

    case GripState::CLOSING:
      gripper->SetOutput(GRIP_CLOSE_CURRENT);
      if (gripper->GetCurr() > GRIP_STALL_THRESH) {
        grip_hold_pos = gripper->GetTheta();
        grip_state = GripState::HOLDING;
        print("GRIP TEST: close stall -> HOLDING at %.3f rad\r\n", grip_hold_pos);
      }
      break;

    case GripState::HOLDING: {
      const int16_t pid_out =
          grip_pid->ComputeConstrainedOutput(gripper->GetThetaDelta(grip_hold_pos));
      gripper->SetOutput(pid_out);
      break;
    }
  }
}

}  // namespace

void RM_RTOS_Init(void) {
  // print_use_usb();
  print_use_uart(&huart10);
  arm_can = new bsp::CAN(&hfdcan2, 0);
  dbus = new remote::DBUS(&huart5);
  gripper = new control::Motor2006(arm_can, GRIP_RX_ID);
  grip_pid = new control::ConstrainedPID(
      GRIP_KP, GRIP_KI, GRIP_KD, GRIP_PID_MAX_IOUT, GRIP_PID_MAX_OUT);

  ResetGripperControl();
  ResetSwitchEdgeDetectors();
  PrintControlHelp();
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  bool test_enabled = false;
  uint32_t next_status_tick = HAL_GetTick();
  uint32_t next_wait_print_tick = HAL_GetTick();

  while (true) {
    const bool dbus_ok = DbusHealthy();
    const bool enable_requested = dbus_ok && dbus->swr != remote::DOWN;

    if (!enable_requested) {
      if (test_enabled) {
        print("GRIP TEST: disabled -> zero current\r\n");
        ResetGripperControl();
      } else if (!dbus_ok && HAL_GetTick() >= next_wait_print_tick) {
        next_wait_print_tick = HAL_GetTick() + WAIT_PRINT_PERIOD_MS;
        print("GRIP TEST: waiting for DBUS frames on huart5...\r\n");
      }

      test_enabled = false;
      gripper->SetOutput(0);
      TransmitGripperOutput();
      osDelay(5);
      continue;
    }

    if (!test_enabled) {
      test_enabled = true;
      ResetGripperControl();
      ResetSwitchEdgeDetectors();
      print("GRIP TEST: enabled (swr=%d)\r\n", (int)dbus->swr);
    }

    UpdateGripperStateMachine();
    TransmitGripperOutput();

    if (HAL_GetTick() >= next_status_tick) {
      next_status_tick = HAL_GetTick() + STATUS_PERIOD_MS;
      print(
          "GRIP TEST | state=%-7s swl=%d curr=%6d theta=% .3f hold=% .3f conn=%d\r\n",
          GripStateName(grip_state), (int)dbus->swl, gripper->GetCurr(),
          gripper->GetTheta(), grip_hold_pos, (int)gripper->connection_flag_);
    }

    osDelay(5);
  }
}
