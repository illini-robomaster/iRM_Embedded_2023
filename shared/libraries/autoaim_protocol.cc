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

#include "autoaim_protocol.h"

#include <cstring>
#include <memory>

namespace communication {

AutoaimProtocol::AutoaimProtocol() {
  index = 0; // current pointer to write
  flag = 0;
}

void AutoaimProtocol::Send(STMToJetsonData* packet, uint8_t color, float cur_yaw, float cur_pitch, uint32_t additional_info) {
  packet->header[0] = 'H';
  packet->header[1] = 'D';
  packet->my_color = color;
  packet->cur_yaw = (int32_t)(cur_yaw * INT_FP_SCALE);
  packet->cur_pitch = (int32_t)(cur_pitch * INT_FP_SCALE);
  packet->additional_info = additional_info;

  const int tail_offset = 3; // size of data minus uint8_t checksum and 2 uint8_t tail
  packet->crc8_checksum = get_crc8_check_sum((uint8_t*)packet,
                                              sizeof(STMToJetsonData) - tail_offset,
                                              0);

  packet->tail[0] = 'E';
  packet->tail[1] = 'D';
}

void AutoaimProtocol::Receive(const uint8_t* data, uint8_t length) {
  // Four cases
  // Case 1: everything is fresh with complete package(s)
  // Case 2: everything is fresh; package is incomplete
  // Case 3: package contains half previous package and half new package
  // Case 4: package contains half previous package and new package(s)

  if (index > 0) {
    // Case 3 and 4
    // copy the remaining bytes from previous package
    int remain = std::min((int)PKG_LEN - index, (int)length);
    memcpy(host_command + index, data, remain);
    index += remain;

    if (index == PKG_LEN) {
      // Case 3
      // done package reading
      index = 0;
      handle();
    } else {
      if (remain == length) {
        // exhausted current package already; and not reaching PKG_LEN
        return;
      }
    }
  }

  int i = 0;

  while (i < (int)length) {
    if (index == 0) {
      if (i == (int)length - 1) {
        // A special case to handle the last byte; index must be zero
        if (data[i] == 'S' || data[i] == 'M') {
          host_command[index++] = data[i];
        }
        return;
      }
      if ((data[i] == 'S' && data[i + 1] == 'T') ||
          (data[i] == 'M' && data[i + 1] == 'Y')) {
        // Detect packet head; start copying
        host_command[index++] = data[i++];
        host_command[index++] = data[i++];
      } else {
        i++;
      }
    } else {
      host_command[index++] = data[i++];
      if (index == PKG_LEN) {
        index = 0;
        handle();
      }
    }
  }
}

void AutoaimProtocol::handle(void) {
  // TODO: implement thread-safe logic here (use a lock to handle changes from interrupt)
  // here we can assume that the package is complete
  // in the host_command buffer

  // TODO: add a logic here such that when the checking fails; it moves the write pointer 'index'
  // to the next 'S' or 'M' for more robustness.
  // A minor issue with current implementation: imagine the following case:
  //  Two packets arrive in two UART calls.
  //  The first packet misses 1 byte, but the second one is complete.
  //  In this case, when the host_command buffer is filled
  //  (the last byte is 'S' or 'M' for the second packet), handle() will be called. The whole buffer
  //  would be tossed, resulting in two unusable packets. However, if we implement this logic, we would be
  //  able to recover the second packet.

  // This is a minor issue because
  //    1) we don't observe this even when packets are sent at 200Hz
  //    2) probability of this happening is very low. The second packet has to be sent in two slices to
  //       trigger this issue. (first slice: S/T is sent to host_command; second slide: the rest)

  // check end of packet is 'ED'
  if (host_command[PKG_LEN - 2] != 'E' || host_command[PKG_LEN - 1] != 'D') {
    flag = 0;
    return;
  }

  if (verify_crc8_check_sum(host_command, PKG_LEN - 2)) {
    process_data();
    valid_packet_cnt++;
    flag = 1;
  } else {
    flag = 0;
  }
}

float AutoaimProtocol::get_relative_yaw(void) {
  return relative_yaw;
}

float AutoaimProtocol::get_relative_pitch(void) {
  return relative_pitch;
}

uint32_t AutoaimProtocol::get_seqnum(void) {
  return seqnum;
}

uint32_t AutoaimProtocol::get_valid_packet_cnt(void) {
  return valid_packet_cnt;
}

void AutoaimProtocol::process_data() {
  // Assume that the host_command is a complete and verified message

  // char pointer because host_command is a byte array
  uint8_t* seq_num_start = host_command + this->SEQNUM_OFFSET;
  uint8_t* rel_yaw_start = host_command + this->REL_YAW_OFFSET;
  uint8_t* rel_pitch_start = host_command + this->REL_PITCH_OFFSET;

  seqnum = (*(uint32_t *)seq_num_start);
  relative_yaw = (*(int32_t *)rel_yaw_start) * 1.0f / this->INT_FP_SCALE;
  relative_pitch = *(int32_t *)rel_pitch_start *1.0f / this->INT_FP_SCALE;
}

uint8_t AutoaimProtocol::get_valid_flag(void) {
  uint8_t temp = flag;
  flag = 0;
  return temp;
}
}  // namespace communication