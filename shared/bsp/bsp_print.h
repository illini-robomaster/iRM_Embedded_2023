/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2023 RoboMaster.                                          *
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

#pragma once

#include <cinttypes>

#include "usart.h"

/**
 * @brief use a uart port for debug print
 *
 * @param huart HAL uart handle
 */
void print_use_uart(UART_HandleTypeDef* huart);

/**
 * @brief use USB virtual com port for debug print
 */
void print_use_usb();

/**
 * @brief print debug message via USB-OTG-FS
 *
 * @param format  formatted string
 * @param ...     same argument lists as in printf
 *
 * @return  number of bytes printed
 *
 * @note    this function requires sufficient stack allocation
 * @note    maximum print length is 32
 * @note    will perform no-op in NDEBUG mode
 */
int32_t print(const char* format, ...);

/* escape codes helper functions -- http://www.termsys.demon.co.uk/vtansi.htm */

/**
 * @brief set the cursor with escape codes
 *
 * @param row row of the cursor
 * @param col column of the cursor
 */
void set_cursor(int row, int col);

/**
 * @brief clear uart screen
 */
void clear_screen(void);
