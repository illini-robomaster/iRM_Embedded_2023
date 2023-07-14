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

#include "dbus.h"

#include <cmath>
#include <cstring>

#include "bsp_error_handler.h"

/* rocker range and dead-zones */
#define RC_ROCKER_MID 1024
#define RC_ROCKER_ZERO_DRIFT 10  // Range of possible drift around initial position
// Range of possible drift around min or max position
#define RC_ROCKER_MIN_MAX_DRIFT (RC_ROCKER_MAX - RC_ROCKER_MID + 10)

namespace remote {

// helper struct for decoding raw bytes
typedef struct {
  /* rocker channel information */
  uint16_t ch0 : 11;  // S1*             *S2
  uint16_t ch1 : 11;  //   C3-^       ^-C1
  uint16_t ch2 : 11;  // C2-<   >+ -<   >+C0
  uint16_t ch3 : 11;  //     +v       v+
  /* left and right switch information */
  uint8_t swr : 2;
  uint8_t swl : 2;
  /* mouse movement and button information */
  remote::mouse_t mouse;
  /* keyboard key information */
  remote::keyboard_t keyboard;
  uint16_t reserved;
} __packed dbus_t;

DBUS::DBUS(UART_HandleTypeDef* huart) : bsp::UART(huart) { SetupRx(sizeof(dbus_t) + 1); }

void DBUS::RxCompleteCallback() {
  connection_flag_ = true;

  uint8_t* data;
  // data frame misalignment
  if (this->Read<true>(&data) != sizeof(dbus_t)) return;

  // re-interpret the data buffer and decode into class properties
  dbus_t* repr = reinterpret_cast<dbus_t*>(data);
  this->ch0 = repr->ch0 - RC_ROCKER_MID;
  this->ch1 = repr->ch1 - RC_ROCKER_MID;
  this->ch2 = repr->ch2 - RC_ROCKER_MID;
  this->ch3 = repr->ch3 - RC_ROCKER_MID;
  this->ch0 = abs(this->ch0) <= RC_ROCKER_ZERO_DRIFT ? 0 : this->ch0;
  this->ch1 = abs(this->ch1) <= RC_ROCKER_ZERO_DRIFT ? 0 : this->ch1;
  this->ch2 = abs(this->ch2) <= RC_ROCKER_ZERO_DRIFT ? 0 : this->ch2;
  this->ch3 = abs(this->ch3) <= RC_ROCKER_ZERO_DRIFT ? 0 : this->ch3;

  this->swl = static_cast<switch_t>(repr->swl);
  this->swr = static_cast<switch_t>(repr->swr);

  memcpy(&this->mouse, &repr->mouse, sizeof(mouse_t));
  memcpy(&this->keyboard, &repr->keyboard, sizeof(keyboard_t));

  this->timestamp = HAL_GetTick();
}

} /* namespace remote */
