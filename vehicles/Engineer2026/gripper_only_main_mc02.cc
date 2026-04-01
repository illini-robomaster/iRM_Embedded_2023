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
 * Bench-tunes a homed gripper control scheme using the ServoMotor wrapper:
 *   - Motor2006 on hfdcan2, RX 0x206
 *   - press the MCU key to close until the hard stop and declare zero
 *   - swl MID  posedge -> move to the open reference
 *   - swl DOWN posedge -> move back to the closed reference
 *   - swl UP   posedge -> freeze at the current position
 *   - after homing, the ServoMotor wrapper keeps the claw on its target
 *
 * Safety:
 *   - swr DOWN or stale DBUS -> zero current
 *   - swr MID/UP             -> enable the gripper test loop
 */

#include "main.h"

#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "fdcan.h"
#include "motor.h"
#include "tim.h"
#include "usart.h"
#include "utils.h"

extern "C" unsigned long getRunTimeCounterValue(void);

namespace bsp {

void SetHighresClockTimer(TIM_HandleTypeDef* htim) { UNUSED(htim); }

uint32_t GetHighresTickMicroSec(void) { return getRunTimeCounterValue(); }

}  // namespace bsp

namespace {

static constexpr uint16_t GRIP_RX_ID = 0x206;

// Close into the hard stop to establish the closed zero reference.
static constexpr int16_t GRIP_HOME_CURRENT = 6000;
static constexpr int16_t GRIP_STALL_THRESH = 4000;
static constexpr uint8_t GRIP_STALL_DEBOUNCE = 3;

// Position targets are in output-shaft radians relative to the homed closed position.
static constexpr float GRIP_CLOSED_TARGET_POS = 0.0f;
static constexpr float GRIP_OPEN_TARGET_POS = -39.0f;

// ServoMotor wrapper tuning for the gripper.
static constexpr float GRIP_CLOSE_SERVO_MAX_SPEED = 30.0f;
static constexpr float GRIP_CLOSE_SERVO_MAX_ACCEL = 120.0f;
static constexpr float GRIP_OPEN_SERVO_MAX_SPEED = 30.0f;
static constexpr float GRIP_OPEN_SERVO_MAX_ACCEL = 120.0f;
static float GRIP_SERVO_PID_PARAM[3] = {220.0f, 0.0f, 30.0f};
static constexpr float GRIP_SERVO_MAX_IOUT = 12000.0f;
static constexpr float GRIP_SERVO_MAX_OUT = 16384.0f;
static constexpr float GRIP_SERVO_PROXIMITY_IN = 0.01f;
static constexpr float GRIP_SERVO_PROXIMITY_OUT = 0.03f;

static constexpr uint32_t DBUS_TIMEOUT_MS = 100;
static constexpr uint32_t STATUS_PERIOD_MS = 100;
static constexpr uint32_t WAIT_PRINT_PERIOD_MS = 1000;

enum class GripState { WAIT_HOME, ZEROING, OPENING, CLOSING, HOLDING };

static bsp::CAN* arm_can = nullptr;
static remote::DBUS* dbus = nullptr;
static control::Motor2006* gripper = nullptr;
control::MotorDMJ3507* arm_j6 = nullptr;
control::Motor4310* arm_j4 = nullptr;
static control::ServoMotor* grip_servo = nullptr;
static bsp::GPIO* grip_home_key = nullptr;

static GripState grip_state = GripState::WAIT_HOME;
static float grip_target_pos = GRIP_CLOSED_TARGET_POS;
static bool grip_homed = false;
static uint8_t grip_home_stall_count = 0;

static BoolEdgeDetector* grip_home_key_edge = nullptr;
static BoolEdgeDetector* swl_mid_edge = nullptr;
static BoolEdgeDetector* swl_down_edge = nullptr;
static BoolEdgeDetector* swl_up_edge = nullptr;

const char* GripStateName(GripState state) {
  switch (state) {
    case GripState::WAIT_HOME:
      return "WAIT_HOME";
    case GripState::ZEROING:
      return "ZEROING";
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

bool GripHomeKeyPressed() {
  return grip_home_key != nullptr && !grip_home_key->Read();
}

float GripPosition() {
  return grip_servo != nullptr ? grip_servo->GetTheta() : 0.0f;
}

float GripOmega() {
  if (grip_servo != nullptr) return grip_servo->GetOmega();
  return gripper != nullptr ? gripper->GetOmega() / M2006P36_RATIO : 0.0f;
}

void TransmitGripperOutput() {
  control::MotorCANBase* motors[] = {gripper};
  control::MotorCANBase::TransmitOutput(motors, 1);
}

void ResetGripHomeKeyEdgeDetector() {
  delete grip_home_key_edge;
  grip_home_key_edge = new BoolEdgeDetector(GripHomeKeyPressed());
}

void SetGripServoMotionProfile(float max_speed, float max_acceleration) {
  if (grip_servo == nullptr) return;
  grip_servo->SetMaxSpeed(max_speed);
  grip_servo->SetMaxAcceleration(max_acceleration);
}

void ConfigureGripServoZero() {
  if (grip_servo != nullptr || gripper == nullptr) return;

  control::servo_t servo_data;
  servo_data.motor = gripper;
  servo_data.max_speed = GRIP_CLOSE_SERVO_MAX_SPEED;
  servo_data.max_acceleration = GRIP_CLOSE_SERVO_MAX_ACCEL;
  servo_data.transmission_ratio = M2006P36_RATIO;
  servo_data.omega_pid_param = GRIP_SERVO_PID_PARAM;
  servo_data.max_iout = GRIP_SERVO_MAX_IOUT;
  servo_data.max_out = GRIP_SERVO_MAX_OUT;

  grip_servo = new control::ServoMotor(
      servo_data, gripper->GetTheta(), GRIP_SERVO_PROXIMITY_IN,
      GRIP_SERVO_PROXIMITY_OUT);
  SetGripServoMotionProfile(GRIP_CLOSE_SERVO_MAX_SPEED, GRIP_CLOSE_SERVO_MAX_ACCEL);
  grip_target_pos = GRIP_CLOSED_TARGET_POS;
  grip_servo->SetTarget(grip_target_pos, true);
  grip_homed = true;
}

void ResetGripperControl() {
  grip_home_stall_count = 0;
  grip_target_pos =
      grip_servo != nullptr ? grip_servo->GetTheta() : GRIP_CLOSED_TARGET_POS;
  grip_state = grip_homed ? GripState::HOLDING : GripState::WAIT_HOME;
  if (grip_servo != nullptr) grip_servo->SetTarget(grip_target_pos, true);
  if (gripper != nullptr) gripper->SetOutput(0);
  ResetGripHomeKeyEdgeDetector();
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
  print("Press MCU key (PA15, active low) to close into the hard stop and zero the claw\r\n");
  print("Control after homing: swl MID edge -> open target, swl DOWN edge -> closed target\r\n");
  print("Automatic targets: open = %.3f rad, close = %.3f rad\r\n",
        GRIP_OPEN_TARGET_POS, GRIP_CLOSED_TARGET_POS);
  print("The ServoMotor wrapper keeps holding the commanded target between switch events\r\n");
}

void UpdateGripperStateMachine() {
  swl_mid_edge->input(dbus->swl == remote::MID);
  swl_down_edge->input(dbus->swl == remote::DOWN);
  swl_up_edge->input(dbus->swl == remote::UP);
  if (grip_home_key_edge != nullptr) grip_home_key_edge->input(GripHomeKeyPressed());

  const bool home_cmd = grip_home_key_edge != nullptr && grip_home_key_edge->posEdge();
  const bool open_cmd = swl_mid_edge->posEdge();
  const bool close_cmd = swl_down_edge->posEdge();

  if (!grip_homed) {
    if (grip_state == GripState::WAIT_HOME && home_cmd) {
      grip_home_stall_count = 0;
      grip_state = GripState::ZEROING;
      print("GRIP TEST: ZEROING -> close into hard stop\r\n");
      arm_j4->SetZeroPos();
      arm_j6->SetZeroPos();
    } else if (open_cmd || close_cmd) {
      print("GRIP TEST: press the MCU key first to home/zero the gripper\r\n");
    }
  } else if (grip_servo != nullptr) {
    if (open_cmd) {
      SetGripServoMotionProfile(GRIP_OPEN_SERVO_MAX_SPEED, GRIP_OPEN_SERVO_MAX_ACCEL);
      grip_target_pos = GRIP_OPEN_TARGET_POS;
      grip_servo->SetTarget(grip_target_pos, true);
      grip_state = GripState::OPENING;
      print("GRIP TEST: OPENING -> %.3f rad\r\n", grip_target_pos);
    } else if (close_cmd) {
      SetGripServoMotionProfile(GRIP_CLOSE_SERVO_MAX_SPEED, GRIP_CLOSE_SERVO_MAX_ACCEL);
      grip_target_pos = GRIP_CLOSED_TARGET_POS;
      grip_servo->SetTarget(grip_target_pos, true);
      grip_state = GripState::CLOSING;
      print("GRIP TEST: CLOSING -> %.3f rad\r\n", grip_target_pos);
    }
  }

  switch (grip_state) {
    case GripState::WAIT_HOME:
      gripper->SetOutput(0);
      break;

    case GripState::ZEROING:
      print("arm_j4 zero pos: %.3f rad, arm_j6 zero pos: %.3f rad\r\n", arm_j4->GetTheta(),
             arm_j6->GetTheta());    
      // gripper->SetOutput(GRIP_HOME_CURRENT);
      if (gripper->GetCurr() > GRIP_STALL_THRESH) {
        if (++grip_home_stall_count >= GRIP_STALL_DEBOUNCE) {
          grip_home_stall_count = 0;
          ConfigureGripServoZero();
          grip_state = GripState::HOLDING;
          print("GRIP TEST: homed -> zero set at hard stop\r\n");
        }
      } else {
        grip_home_stall_count = 0;
      }
      break;

    case GripState::CLOSING: {
      if (grip_servo == nullptr) {
        grip_state = GripState::WAIT_HOME;
        gripper->SetOutput(0);
        break;
      }
      grip_servo->CalcOutput();
      if (grip_servo->Holding()) {
        grip_state = GripState::HOLDING;
        print("GRIP TEST: close reference engaged -> HOLDING at %.3f rad\r\n",
              GripPosition());
      }
      break;
    }

    case GripState::OPENING:
      if (grip_servo == nullptr) {
        grip_state = GripState::WAIT_HOME;
        gripper->SetOutput(0);
        break;
      }
      grip_servo->CalcOutput();
      if (grip_servo->Holding()) {
        grip_state = GripState::HOLDING;
        print("GRIP TEST: open target reached -> HOLDING at %.3f rad\r\n",
              GripPosition());
      }
      break;

    case GripState::HOLDING:
      if (grip_servo == nullptr) {
        grip_state = GripState::WAIT_HOME;
        gripper->SetOutput(0);
        break;
      }
      grip_servo->CalcOutput();
      break;
  }
}

}  // namespace
static constexpr uint16_t J6_MASTER_ID = 0x14, J6_CAN_ID = 0x15;  // MotorDMJ3507
static constexpr uint16_t J4_MASTER_ID = 0x16, J4_CAN_ID = 0x17;  // Motor4310
void RM_RTOS_Init(void) {
  print_use_usb();
  bsp::SetHighresClockTimer(&htim2);
  arm_can = new bsp::CAN(&hfdcan2, 0);
  dbus = new remote::DBUS(&huart5);
  gripper = new control::Motor2006(arm_can, GRIP_RX_ID);
  grip_home_key = new bsp::GPIO(GPIOA, GPIO_PIN_15);
  arm_j6 = new control::MotorDMJ3507(arm_can, J6_MASTER_ID, J6_CAN_ID, control::FORCE_POS);
  arm_j4 = new control::Motor4310(arm_can, J4_MASTER_ID, J4_CAN_ID, control::POS_VEL);

  ResetGripperControl();
  ResetGripHomeKeyEdgeDetector();
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
          "GRIP TEST | state=%-9s homed=%d key=%d swl=%d ch3=%4d curr=%6d pos=% .3f tgt=% .3f omega=% .3f conn=%d\r\n",
          GripStateName(grip_state), (int)grip_homed, (int)GripHomeKeyPressed(),
          (int)dbus->swl, dbus->ch3, gripper->GetCurr(), GripPosition(),
          grip_target_pos, GripOmega(), (int)gripper->connection_flag_);
    }

    osDelay(5);
  }
}
