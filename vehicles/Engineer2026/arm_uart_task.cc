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
 * @brief Dedicated RTOS thread for OrangePi <-> STM32 UART communication.
 *
 * This thread decouples UART I/O from the motor-control loop so that
 * encoder feedback is always transmitted at a consistent rate and command
 * reception is never blocked by motor state transitions.
 *
 * Shared state with arm_mc02.cc (all declared extern below):
 *   - arm_uart, arm_j1..arm_j6  (read — set once by ArmInit)
 *   - cmd_target_deg             (written here, read by ArmUpdate)
 *   - last_valid_rx_tick          (written here, read by ArmUpdate)
 *   - arm_enabled                (read/written by both — single-core safe)
 */

#include "arm_uart_task.h"

#include <cmath>
#include <cstdio>

#include "arm_mc02.h"
#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "motor.h"
#include "uart_framing.h"

// ── Extern shared state from arm_mc02.cc ─────────────────────────────────────
extern bsp::UART* arm_uart;

extern control::Motor4310*     arm_j1;
extern control::MotorDMJ10010* arm_j2;
extern control::MotorDMJ10010* arm_j3;
extern control::Motor4310*     arm_j4;
extern control::Motor4310*     arm_j5;
extern control::MotorDMJ3507*  arm_j6;

extern volatile float    cmd_target_deg[6];
extern volatile uint32_t last_valid_rx_tick;
extern volatile bool     arm_enabled;

extern bool test_ros_tx;
extern bool test_ros_rx;

// ── Constants ────────────────────────────────────────────────────────────────
// Per-joint position limits (ARM_CMD_MIN_DEG / ARM_CMD_MAX_DEG) come from arm_mc02.h.
// Max frame-to-frame jump — rejects physically impossible teleportation.
static constexpr float CMD_MAX_DELTA_DEG = 180.0f;
static constexpr float RAD2DEG = 180.0f / (float)M_PI;

// ── Thread attributes ────────────────────────────────────────────────────────
const osThreadAttr_t armUartTaskAttr = {.name = "armUartTask",
                                        .attr_bits = osThreadDetached,
                                        .cb_mem = nullptr,
                                        .cb_size = 0,
                                        .stack_mem = nullptr,
                                        .stack_size = 512 * 4,
                                        .priority = (osPriority_t)osPriorityAboveNormal,
                                        .tz_module = 0,
                                        .reserved = 0};

// ── ArmUartTask ──────────────────────────────────────────────────────────────

void ArmUartTask(void* arg) {
  UNUSED(arg);

  // Wait for ArmInit() to finish (arm_uart is set by RM_RTOS_Init).
  while (arm_uart == nullptr) osDelay(10);

  // Local frame accumulation (not shared with ArmUpdate).
  uint8_t local_rx_frame[UART_FRAME_LEN];
  int     local_rx_len = 0;

  // TX rate control: send encoder feedback every 20 ms (50 Hz).
  uint32_t next_tx_tick = HAL_GetTick();
  // Encoder poll rate when arm is not yet enabled: every 50 ms (20 Hz).
  uint32_t next_poll_tick = HAL_GetTick();

  while (true) {
    // ── 1. UART RX: accumulate bytes, parse complete frames ──────────────
    uint8_t* rx_buf = nullptr;
    int32_t rx_len = arm_uart->Read(&rx_buf);
    for (int32_t i = 0; i < rx_len; ++i) {
      uint8_t b = rx_buf[i];
      if (b == 0xA5u) {
        // SOF — always reset and start a new frame.
        local_rx_len = 0;
        local_rx_frame[local_rx_len++] = b;
      } else if (local_rx_len == 0) {
        // No SOF seen yet — skip junk bytes.
      } else {
        local_rx_frame[local_rx_len++] = b;
        if (local_rx_len == (int)UART_FRAME_LEN) {
          float new_targets[6];
          if (UartRxParseFrame(local_rx_frame, new_targets)) {
            // ── Sanity filter ─────────────────────────────────────────
            bool ok = true;
            for (int j = 0; j < 6 && ok; ++j) {
              if (!isfinite(new_targets[j])) {
                print("ARM CMD REJECT: J%d non-finite\r\n", j + 1);
                ok = false;
              } else if (new_targets[j] < ARM_CMD_MIN_DEG[j] ||
                         new_targets[j] > ARM_CMD_MAX_DEG[j]) {
                // Clamp to joint limit and warn — do NOT reject so that
                // last_valid_rx_tick still updates and the watchdog is kept alive.
                print("ARM CMD CLAMP: J%d %.2f deg clamped to [%.1f, %.1f]\r\n",
                      j + 1, new_targets[j], ARM_CMD_MIN_DEG[j], ARM_CMD_MAX_DEG[j]);
                new_targets[j] = new_targets[j] < ARM_CMD_MIN_DEG[j]
                                     ? ARM_CMD_MIN_DEG[j] : ARM_CMD_MAX_DEG[j];
              } else if (arm_enabled &&
                         fabsf(new_targets[j] - cmd_target_deg[j]) > CMD_MAX_DELTA_DEG) {
                print("ARM CMD REJECT: J%d delta %.2f deg exceeds limit\r\n",
                      j + 1, new_targets[j] - cmd_target_deg[j]);
                ok = false;
              }
            }
            if (ok) {
              for (int j = 0; j < 6; ++j) cmd_target_deg[j] = new_targets[j];
              last_valid_rx_tick = HAL_GetTick();
              if (test_ros_rx) {
                print("ARM UART RX | J1=%6.2f J2=%6.2f J3=%6.2f J4=%6.2f J5=%6.2f J6=%6.2f [deg]\r\n",
                      cmd_target_deg[0], cmd_target_deg[1], cmd_target_deg[2],
                      cmd_target_deg[3], cmd_target_deg[4], cmd_target_deg[5]);
              }
              // NOTE: ArmEnable() is NOT called here.  The UART task only
              // handles data (cmd_target_deg + last_valid_rx_tick).  The
              // bumpless-enable decision lives in ArmUpdate() so it naturally
              // respects the kill switch (ArmUpdate isn't called when swr==DOWN).
            }
          }
          local_rx_len = 0;
        }
      }
    }

    // ── 2. Pre-enable encoder polling ────────────────────────────────────
    // When the arm is not yet enabled, send MotorDisable frames to solicit
    // CAN feedback so GetTheta() returns real encoder data for TX below.
    if (!arm_enabled && HAL_GetTick() >= next_poll_tick) {
      next_poll_tick = HAL_GetTick() + 50;
      arm_j1->MotorDisable();
      arm_j2->MotorDisable();
      arm_j3->MotorDisable();
      arm_j4->MotorDisable();
      arm_j5->MotorDisable();
      arm_j6->MotorDisable();
    }

    // ── 3. UART TX: send encoder feedback at 50 Hz ──────────────────────
    if (HAL_GetTick() >= next_tx_tick) {
      next_tx_tick = HAL_GetTick() + 20;  // 50 Hz
      const float enc[6] = {
          arm_j1->GetTheta() * RAD2DEG,
          arm_j2->GetTheta() * RAD2DEG,
          arm_j3->GetTheta() * RAD2DEG,
          arm_j4->GetTheta() * RAD2DEG,
          arm_j5->GetTheta() * RAD2DEG,
          arm_j6->GetTheta() * RAD2DEG,
      };
      if (test_ros_tx) {
        print("ARM UART TX | J1=%6.2f J2=%6.2f J3=%6.2f J4=%6.2f J5=%6.2f J6=%6.2f [deg]\r\n",
              enc[0], enc[1], enc[2], enc[3], enc[4], enc[5]);
      }
      UartTxSendFeedback(arm_uart, enc);
    }

    osDelay(2);  // ~500 Hz polling — fast enough for 50 Hz TX and responsive RX
  }
}
