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
extern volatile bool     arm_enabling;  // true during MotorEnable() sequence
extern volatile uint32_t bumpless_holdoff_until;

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

// ── Diagnostic counters ──────────────────────────────────────────────────────
static uint32_t diag_rx_frames  = 0;  // valid RX frames parsed this period
static uint32_t diag_rx_bad     = 0;  // corrupt/rejected RX frames this period
static uint32_t diag_rx_bytes   = 0;  // total RX bytes this period
static uint32_t diag_tx_frames  = 0;  // TX frames sent this period
static uint32_t diag_tx_fails   = 0;  // TX write failures (short writes)
static bool     diag_ever_rx    = false;  // has any valid RX frame ever arrived?

// ── Startup delta-check gate ─────────────────────────────────────────────────
// cmd_target_deg is initialised from encoder positions at bumpless-enable time,
// not from a real ROS command.  The first real post-holdoff command may differ
// by any amount (operator set up EE at a different pose than the arm's parked
// position).  Skip the frame-to-frame delta check until at least one real
// command has been written so we don't reject that first legitimate frame.
static bool     cmd_target_real = false;  // true after first real post-holdoff cmd
static bool     prev_arm_enabled = false; // edge-detect arm_enabled rising edge

// ── UART error recovery ─────────────────────────────────────────────────────
// The BSP error handler only clears PE (parity error).  ORE (overrun), FE
// (framing), and NE (noise) flags are NOT cleared, which can silently stop
// the DMA from receiving further bytes.  We periodically clear all error
// flags when no RX data has arrived, as a belt-and-suspenders recovery.
extern UART_HandleTypeDef huart10;

static void UartClearErrorFlags() {
  __HAL_UART_CLEAR_OREFLAG(&huart10);   // overrun error
  __HAL_UART_CLEAR_FEFLAG(&huart10);    // framing error
  __HAL_UART_CLEAR_NEFLAG(&huart10);    // noise error
  __HAL_UART_CLEAR_PEFLAG(&huart10);    // parity error
}

// ── ArmUartTask ──────────────────────────────────────────────────────────────

void ArmUartTask(void* arg) {
  UNUSED(arg);

  // Wait for ArmInit() to finish (arm_uart is set by RM_RTOS_Init).
  while (arm_uart == nullptr) osDelay(10);

  // Local frame accumulation (not shared with ArmUpdate).
  uint8_t local_rx_frame[UART_FRAME_LEN];
  int     local_rx_len = 0;

  // TX rate control: send encoder feedback every 5 ms (200 Hz).
  uint32_t next_tx_tick = HAL_GetTick();
  // Encoder poll rate when arm is not yet enabled: every 50 ms (20 Hz).
  uint32_t next_poll_tick = HAL_GetTick();
  // Diagnostic print every 5 s.
  uint32_t next_diag_tick = HAL_GetTick() + 5000;
  // Error flag recovery every 2 s when no RX has arrived.
  uint32_t next_recovery_tick = HAL_GetTick() + 2000;

  // Debug hex dump rate limiter: print raw bytes once per second until first valid frame.
  uint32_t next_hex_dump_tick = HAL_GetTick();
  uint32_t diag_sof_seen = 0;   // how many 0xA5 bytes seen this debug period
  uint32_t diag_junk_bytes = 0; // bytes skipped (no SOF context)

  print("ARM UART: task started, waiting for OrangePi...\r\n");

  while (true) {
    // ── 1. UART RX: accumulate bytes, parse complete frames ──────────────
    uint8_t* rx_buf = nullptr;
    int32_t rx_len = arm_uart->Read(&rx_buf);
    diag_rx_bytes += (rx_len > 0) ? (uint32_t)rx_len : 0;

    // ── DEBUG: hex dump of raw DMA bytes (rate-limited, pre-first-frame only) ──
    if (!diag_ever_rx && rx_len > 0 && HAL_GetTick() >= next_hex_dump_tick) {
      next_hex_dump_tick = HAL_GetTick() + 1000;  // once per second
      // Print up to 32 bytes in hex
      int dump_len = (rx_len > 32) ? 32 : rx_len;
      print("ARM UART DBG | len=%d hex:", rx_len);
      for (int d = 0; d < dump_len; ++d) {
        print(" %02X", (unsigned)rx_buf[d]);
      }
      if (rx_len > 32) print(" ...(+%d)", rx_len - 32);
      print("\r\n");
      // Print UART error flag status (STM32H7 ISR register)
      uint32_t isr = huart10.Instance->ISR;
      print("ARM UART DBG | ISR=0x%08lX (ORE=%lu FE=%lu NE=%lu PE=%lu RXNE=%lu IDLE=%lu)\r\n",
            (unsigned long)isr,
            (unsigned long)((isr >> 3) & 1),  // ORE
            (unsigned long)((isr >> 1) & 1),  // FE
            (unsigned long)((isr >> 2) & 1),  // NE
            (unsigned long)((isr >> 0) & 1),  // PE
            (unsigned long)((isr >> 5) & 1),  // RXNE
            (unsigned long)((isr >> 4) & 1)); // IDLE
      print("ARM UART DBG | sof_seen=%lu junk=%lu accum_len=%d\r\n",
            (unsigned long)diag_sof_seen, (unsigned long)diag_junk_bytes,
            local_rx_len);
      diag_sof_seen = 0;
      diag_junk_bytes = 0;
    }

    for (int32_t i = 0; i < rx_len; ++i) {
      uint8_t b = rx_buf[i];
      if (local_rx_len == 0) {
        // ── Hunting for SOF ──────────────────────────────────────────
        if (b == 0xA5u) {
          diag_sof_seen++;
          local_rx_frame[local_rx_len++] = b;
        } else {
          diag_junk_bytes++;
        }
      } else if (local_rx_len == 1) {
        // ── Verify LEN byte (0x0C) right after SOF ──────────────────
        if (b == 0x0Cu) {
          local_rx_frame[local_rx_len++] = b;
        } else {
          // Bad LEN — false SOF.  Check if THIS byte is the real SOF.
          local_rx_len = 0;
          if (b == 0xA5u) {
            diag_sof_seen++;
            local_rx_frame[local_rx_len++] = b;
          }
        }
      } else {
        // ── Accumulating payload + CRC (no 0xA5 restart) ─────────────
        // Payload bytes can legitimately be 0xA5 (e.g. J3 = -75.15°
        // encodes as centidegrees 0xE2A5, low byte = 0xA5).
        local_rx_frame[local_rx_len++] = b;
        if (local_rx_len == (int)UART_FRAME_LEN) {
          float new_targets[6];
          if (UartRxParseFrame(local_rx_frame, new_targets)) {
            // ── Sanity filter ─────────────────────────────────────────
            // Detect arm_enabled rising edge — reset cmd_target_real so
            // the first real command after bumpless enable bypasses the
            // delta check (cmd_target_deg holds encoder snapshot, not a
            // prior ROS command, so any delta is expected and safe).
            if (arm_enabled && !prev_arm_enabled) {
              cmd_target_real = false;
            }
            prev_arm_enabled = arm_enabled;

            bool ok = true;
            for (int j = 0; j < 6 && ok; ++j) {
              if (!isfinite(new_targets[j])) {
                print("ARM CMD REJECT: J%d non-finite\r\n", j + 1);
                ok = false;
              } else if (arm_enabled && cmd_target_real &&
                         fabsf(new_targets[j] - cmd_target_deg[j]) > CMD_MAX_DELTA_DEG) {
                print("ARM CMD REJECT: J%d delta %.2f deg exceeds limit\r\n",
                      j + 1, new_targets[j] - cmd_target_deg[j]);
                ok = false;
              }
            }
            if (ok) {
              if (!diag_ever_rx) {
                diag_ever_rx = true;
                print("ARM UART: first RX from OrangePi at t=%lums\r\n",
                      (unsigned long)HAL_GetTick());
              }
              // Always update watchdog — keeps ArmUpdate from triggering the
              // 2 s timeout even during the bumpless holdoff window.
              last_valid_rx_tick = HAL_GetTick();
              diag_rx_frames++;

              // During bumpless holdoff, skip cmd_target_deg writes so the
              // encoder-position snapshot from ArmUpdate isn't overwritten
              // with stale/zero values from ROS (which hasn't received
              // encoder feedback yet).  The holdoff expires after 500 ms.
              if (HAL_GetTick() < bumpless_holdoff_until) {
                // Holdoff active — discard command but keep watchdog alive.
              } else {
                for (int j = 0; j < 6; ++j) cmd_target_deg[j] = new_targets[j];
                // First real command written — delta check active from next frame.
                cmd_target_real = true;
              }
              if (test_ros_rx) {
                print("ARM UART RX | J1=%6.2f J2=%6.2f J3=%6.2f J4=%6.2f J5=%6.2f J6=%6.2f [deg]\r\n",
                      cmd_target_deg[0], cmd_target_deg[1], cmd_target_deg[2],
                      cmd_target_deg[3], cmd_target_deg[4], cmd_target_deg[5]);
              }
              // NOTE: ArmEnable() is NOT called here.  The UART task only
              // handles data (cmd_target_deg + last_valid_rx_tick).  The
              // bumpless-enable decision lives in ArmUpdate() so it naturally
              // respects the kill switch (ArmUpdate isn't called when swr==DOWN).
            } else {
              diag_rx_bad++;
            }
          } else {
            diag_rx_bad++;
          }
          local_rx_len = 0;
        }
      }
    }

    // ── 2. Pre-enable encoder polling ────────────────────────────────────
    // When the arm is not yet enabled, send MotorDisable frames to solicit
    // CAN feedback so GetTheta() returns real encoder data for TX below.
    if (!arm_enabled && !arm_enabling && HAL_GetTick() >= next_poll_tick) {
      next_poll_tick = HAL_GetTick() + 50;
      arm_j1->MotorDisable();
      arm_j2->MotorDisable();
      arm_j3->MotorDisable();
      arm_j4->MotorDisable();
      arm_j5->MotorDisable();
      arm_j6->MotorDisable();
    }

    // ── 3. UART error flag recovery ─────────────────────────────────────
    // Periodically clear UART error flags (ORE/FE/NE/PE).  Run regardless
    // of whether any valid frame has arrived: a stuck error flag after the
    // first successful RX is just as capable of stopping DMA as one at
    // startup.  Clearing already-clear flags is a harmless no-op.
    if (HAL_GetTick() >= next_recovery_tick) {
      next_recovery_tick = HAL_GetTick() + 2000;
      UartClearErrorFlags();
    }

    // ── 4. UART TX: send encoder feedback at 200 Hz (every 5 ms) ────────
    if (HAL_GetTick() >= next_tx_tick) {
      next_tx_tick = HAL_GetTick() + 5;  // 200 Hz
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
      int32_t written = UartTxSendFeedback(arm_uart, enc);
      if (written == (int32_t)UART_FRAME_LEN) {
        diag_tx_frames++;
      } else {
        diag_tx_fails++;
      }
    }

    // ── 5. Diagnostics: print UART health every 5 s ─────────────────────
    if (HAL_GetTick() >= next_diag_tick) {
      next_diag_tick = HAL_GetTick() + 5000;
      uint32_t rx_age = (last_valid_rx_tick > 0)
                            ? (HAL_GetTick() - last_valid_rx_tick)
                            : 0xFFFFFFFFu;
      print("ARM UART DIAG | rx=%lu bad=%lu bytes=%lu tx=%lu tx_fail=%lu "
            "rx_age=%lums arm=%s\r\n",
            (unsigned long)diag_rx_frames, (unsigned long)diag_rx_bad,
            (unsigned long)diag_rx_bytes,
            (unsigned long)diag_tx_frames, (unsigned long)diag_tx_fails,
            (unsigned long)(rx_age == 0xFFFFFFFFu ? 99999u : rx_age),
            arm_enabled ? "ON" : "off");

      // Clear diagnostic message when no RX has ever arrived.
      if (!diag_ever_rx) {
        print("ARM UART: no RX from OrangePi yet — is uart_bridge_node running?\r\n");
      }

      diag_rx_frames = 0;
      diag_rx_bad    = 0;
      diag_rx_bytes  = 0;
      diag_tx_frames = 0;
      diag_tx_fails  = 0;
    }

    osDelay(2);  // ~500 Hz polling — fast enough for 200 Hz TX and responsive RX
  }
}
