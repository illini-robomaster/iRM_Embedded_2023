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
 * @brief use a uart port for both debug print (TX) and input reading (RX)
 *
 * @param huart       HAL uart handle
 * @param rx_buf_size receive DMA buffer size in bytes
 */
void print_use_uart_rxtx(UART_HandleTypeDef* huart, uint32_t rx_buf_size = 64);

/**
 * @brief read pending bytes received on the uart set up by print_use_uart_rxtx
 *
 * @param data  pointer that will be set to the internal rx buffer
 * @return number of bytes available, 0 if none or RX not set up
 */
int32_t print_uart_read(uint8_t** data);

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
