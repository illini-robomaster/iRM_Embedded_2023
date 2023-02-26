/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2022 RoboMaster.                                          *
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

#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "crc8.h"

namespace communication {

// WARNING: THIS CLASS IS NOT THREAD SAFE!!!

class MiniPCProtocol {
 public:
  MiniPCProtocol();
  void Receive(const uint8_t* data, uint8_t len);
  // dummy send
  void Send();
  uint8_t get_valid_flag(void);
  float get_relative_yaw(void);
  float get_relative_pitch(void);

 private:
  /* For definitions of constants, check out the documentation at */
  static constexpr uint8_t PKG_LEN = 17;
  static constexpr int32_t INT_FP_SCALE = 1000000;
  static constexpr uint8_t SEQNUM_OFFSET = 2;
  static constexpr uint8_t REL_YAW_OFFSET = SEQNUM_OFFSET + 4;
  static constexpr uint8_t REL_PITCH_OFFSET = REL_YAW_OFFSET + 4;

  int index;
  uint8_t flag;
  uint8_t host_command[PKG_LEN];
  void handle();
  void process_data();

  float relative_yaw;
  float relative_pitch;
}; /* class MiniPCProtocol */

} /* namespace communication */
