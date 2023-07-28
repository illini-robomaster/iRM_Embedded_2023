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

#include "bsp_uart.h"

namespace remote {

const uint16_t WheelDigitalValue = 1684;

typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
  uint8_t l;
  uint8_t r;
} __packed mouse_t;

typedef union {
  uint16_t code;
  struct {
    uint16_t W : 1;
    uint16_t S : 1;
    uint16_t A : 1;
    uint16_t D : 1;
    uint16_t SHIFT : 1;
    uint16_t CTRL : 1;
    uint16_t Q : 1;
    uint16_t E : 1;
    uint16_t R : 1;
    uint16_t F : 1;
    uint16_t G : 1;
    uint16_t Z : 1;
    uint16_t X : 1;
    uint16_t C : 1;
    uint16_t V : 1;
    uint16_t B : 1;
  } __packed bit;
} __packed keyboard_t;

typedef struct {
  uint16_t wheel;
} __packed wheel_t;

typedef enum {
  UP = 1,
  DOWN = 2,
  MID = 3,
} switch_t;

class DBUS : public bsp::UART {
 public:
  /**
   * @brief intialize DBUS the same way as a generic UART peripheral
   * @note like uart, dbus needs time to initialize
   *
   * @param huart uart instance
   */
  DBUS(UART_HandleTypeDef* huart);

  // Add custom rx data handler
  void RxCompleteCallback() override final;

  // rocker channel information
  int16_t ch0;  // S1*             *S2
  int16_t ch1;  //   C3-^       ^-C1
  int16_t ch2;  // C2-<   >+ -<   >+C0
  int16_t ch3;  //     +v       v+
  // left and right switch information
  switch_t swl;
  switch_t swr;
  // mouse movement and button information
  mouse_t mouse;
  // keyboard key information
  keyboard_t keyboard;
  // dial wheel information
  wheel_t wheel;
  // timestamp of the update interrupt
  uint32_t timestamp;

  volatile bool connection_flag_ = false;

  static const int16_t ROCKER_MIN = -660;
  static const int16_t ROCKER_MAX = 660;
  uint16_t previous_wheel_value; 
};

} /* namespace remote */
