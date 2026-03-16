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
 * @brief UART framing utilities for the OrangePi ↔ STM32 link.
 *
 * Binary fixed-length protocol (16 bytes, both directions):
 *
 *   Offset  Bytes  Field
 *   ──────  ─────  ────────────────────────────────────────────────
 *   0       1      SOF = 0xA5
 *   1       1      LEN = 0x0C  (payload byte count = 12)
 *   2–13    12     6 × int16_t, little-endian, unit = centidegrees
 *                  J1@[2:4]  J2@[4:6]  J3@[6:8]
 *                  J4@[8:10] J5@[10:12] J6@[12:14]
 *   14–15   2      CRC16-MODBUS over bytes [0..13], little-endian
 *   ──────  ─────  ────────────────────────────────────────────────
 *
 * These functions are intentionally stateless — all mutable arm state
 * lives in arm_mc02.cc and is passed / returned via arguments.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "bsp_uart.h"

/**
 * @brief CRC16-Modbus: polynomial 0x8005, initial value 0xFFFF, LSB-first
 *        (reflected input and output — identical to Modbus RTU).
 */
uint16_t crc16_modbus(const uint8_t* data, size_t len);

/** Total size of one binary frame in bytes. */
static constexpr size_t UART_FRAME_LEN = 16u;

/**
 * @brief Validate and decode a 16-byte binary frame.
 *
 * Checks SOF (0xA5), LEN (0x0C), and CRC16-Modbus over bytes [0..13], then
 * converts the six int16_t centidegree values to float degrees.
 *
 * @param buf         Pointer to exactly UART_FRAME_LEN bytes.
 * @param out_targets Destination for the six parsed joint angles [deg].
 *                    Written only when the return value is true.
 * @return true   Frame was valid and six angles were decoded.
 * @return false  Frame was malformed or CRC mismatch.
 */
bool UartRxParseFrame(const uint8_t* buf, float out_targets[6]);

/**
 * @brief Encode six encoder angles into a 16-byte binary frame and transmit.
 *
 * @param uart  Destination UART peripheral (must not be nullptr).
 * @param enc   Six joint angles in degrees.
 * @return Number of bytes written (UART_FRAME_LEN on success, less on failure).
 */
int32_t UartTxSendFeedback(bsp::UART* uart, const float enc[6]);
