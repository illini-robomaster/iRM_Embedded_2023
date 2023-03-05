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

#include "minipc.h"

#include <cstring>
#include <memory>

namespace communication {

MiniPCProtocol::MiniPCProtocol() {
  index = -1;
  flag = 0;
}

void MiniPCProtocol::Receive(const uint8_t* data, uint8_t length) {
  if (index >= 0) {
    // already found header
    int remain = std::min((int)PKG_LEN - index, (int)length);
    memcpy(host_command + index, data, remain);
    index += remain;

    if (index == PKG_LEN) {
      // done package reading
      index = -1;
      // package handling here!TODO:
      handle();
    }
  } else {
    for (int32_t i = 0; i < (int32_t)length; i++) {
      if ((data[i] == 'S' && data[i + 1] == 'T') || (data[i] == 'M' && data[i + 1] == 'Y')) {
        index = 0;
        index = std::min((int)PKG_LEN, (int)(length - i));
        memcpy(host_command, data + i, index);
        if (index == PKG_LEN) {
          // handling here! TODO:
          index = -1;
          handle();
          break;
        }
      }
    }
  }
}

void MiniPCProtocol::handle(void) {
  // TODO: implement thread-safe logic here (use a lock to handle changes from interrupt)
  // here we can assume that the package is complete
  // in the host_command buffer

  // check end of packet is 'ED'
  if (host_command[PKG_LEN - 2] != 'E' || host_command[PKG_LEN - 1] != 'D') {
    flag = 0;
    return;
  }

  if (verify_crc8_check_sum(host_command, PKG_LEN - 2)) {
    process_data();
    flag = 1;
  } else {
    flag = 0;
  }
}

float MiniPCProtocol::get_relative_yaw(void) {
  return relative_yaw;
}

float MiniPCProtocol::get_relative_pitch(void) {
  return relative_pitch;
}

void MiniPCProtocol::process_data() {
  // Assume that the host_command is a complete and verified message
  uint8_t* rel_yaw_start = host_command + this->REL_YAW_OFFSET;
  uint8_t* rel_pitch_start = host_command + this->REL_PITCH_OFFSET;

  relative_yaw = (*(int32_t *)rel_yaw_start) * 1.0f / this->INT_FP_SCALE;
  relative_pitch = *(int32_t *)rel_pitch_start *1.0f / this->INT_FP_SCALE;
}

uint8_t MiniPCProtocol::get_valid_flag(void) {
  uint8_t temp = flag;
  flag = 0;
  return temp;
}
}  // namespace communication
