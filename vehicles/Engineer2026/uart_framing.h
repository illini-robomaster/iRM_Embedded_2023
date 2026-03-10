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
 * Protocol: $payload*CCCC\n
 *   - payload : comma-separated ASCII floats (e.g. "1.23,4.56,...")
 *   - CCCC    : CRC16-Modbus of the payload bytes, 4 upper-case hex digits
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

/**
 * @brief Validate and parse a null-terminated "$payload*CCCC" line.
 *
 * Checks frame structure, validates the hex CRC field, recomputes CRC16-Modbus
 * over the payload, and — on a clean match — sscanfs six floats into
 * @p out_targets.  Diagnostic messages are printed on any error.
 *
 * @param line        Null-terminated line buffer.  The '*' character will be
 *                    replaced with '\0' in-place to split payload from CRC.
 * @param out_targets Destination for the six parsed joint angles [deg].
 *                    Written only when the return value is true.
 * @return true   Frame was valid and six angles were successfully parsed.
 * @return false  Frame was malformed, CRC mismatch, or sscanf failed.
 */
bool UartRxParseLine(char* line, float out_targets[6]);

/**
 * @brief Encode six encoder angles into a "$csv*CCCC\n" frame and transmit.
 *
 * @param uart  Destination UART peripheral (must not be nullptr).
 * @param enc   Six joint angles in degrees.
 */
void UartTxSendFeedback(bsp::UART* uart, const float enc[6]);
