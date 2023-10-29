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

MinipcPort::MinipcPort() {
  index = 0; // current pointer to write
  flag = 0;
}

void MinipcPort::Pack(uint8_t* packet, void* data, uint8_t cmd_id) {
  switch (cmd_id) {
    case GIMBAL_CMD_ID:
      PackGimbalData(packet, static_cast<gimbal_data_t*>(data));
      break;
    case COLOR_CMD_ID:
      PackColorData(packet, static_cast<color_data_t*>(data));
      break;
    case CHASSIS_CMD_ID:
      PackChassisData(packet, static_cast<chassis_data_t*>(data));
      break;
  }
}

void MinipcPort::PackGimbalData(uint8_t* packet, gimbal_data_t* data) {
  AddHeaderTail(packet, GIMBAL_CMD_ID);
  int i = 0;
  for (i = 0; i < 4; i++) {
    packet[0 + DATA_OFFSET + i] = data->cur_yaw >> 8 * i;
  }
  for (i = 0; i < 4; i++) {
    packet[4 + DATA_OFFSET + i] = data->cur_pitch >> 8 * i;
  }
  for (i = 0; i < 4; i++) {
    packet[8 + DATA_OFFSET + i] = data->additional_info >> 8 * i;
  }
  AddCRC8(packet, GIMBAL_CMD_ID);
}

void MinipcPort::PackColorData(uint8_t* packet, color_data_t* data) {
  AddHeaderTail(packet, COLOR_CMD_ID);
  packet[DATA_OFFSET] = data->my_color;
  AddCRC8(packet, COLOR_CMD_ID);
}

void MinipcPort::PackChassisData(uint8_t* packet, chassis_data_t* data) {
  AddHeaderTail(packet, CHASSIS_CMD_ID);
  int i = 0;
  for (i = 0; i < 4; i++) {
    packet[0 + DATA_OFFSET + i] = data->vx >> 8 * i;
  }
  for (i = 0; i < 4; i++) {
    packet[4 + DATA_OFFSET + i] = data->vy >> 8 * i;
  }
  for (i = 0; i < 4; i++) {
    packet[8 + DATA_OFFSET + i] = data->vw >> 8 * i;
  }
  AddCRC8(packet, CHASSIS_CMD_ID);
}

uint8_t MinipcPort::GetPacketLen(uint8_t cmd_id) {
  if (cmd_id >= TOTAL_NUM_OF_ID) {
    return 0;
  }
  // Total length = Data length + header & tail (8 bytes) + crc checksum (1 byte)
  return CMD_TO_LEN[cmd_id] + 9;
}

void MinipcPort::Receive(const uint8_t* data, uint8_t length) {
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
      Handle();
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
        Handle();
      }
    }
  }
}

// 8 bytes
void MinipcPort::AddHeaderTail (uint8_t* packet, uint8_t cmd_id) {
  // Add header
  packet[0] = 'S';
  packet[1] = 'T';
  // dummy seq num
  packet[SEQNUM_OFFSET] = 0;
  packet[SEQNUM_OFFSET + 1] = 0 >> 8;
  packet[DATA_LENGTH_OFFSET] = CMD_TO_LEN[cmd_id];
  packet[CMD_ID_OFFSET] = cmd_id;

  // Add tail WITHOUT crc8
  const int EOF_OFFSET = DATA_OFFSET + CMD_TO_LEN[cmd_id] + 1;
  packet[EOF_OFFSET] = 'E';
  packet[EOF_OFFSET + 1] = 'D';
}

// 1 bytes
void MinipcPort::AddCRC8 (uint8_t* packet, int8_t cmd_id) {
  const int CRC8_OFFSET = DATA_OFFSET + CMD_TO_LEN[cmd_id];
  packet[CRC8_OFFSET] = get_crc8_check_sum(packet, CRC8_OFFSET, 0);
}

void MinipcPort::Handle(void) {
  // TODO: implement thread-safe logic here (use a lock to handle changes from interrupt)
  // here we can assume that the package is complete
  // in the host_command buffer

  // TODO: add a logic here such that when the checking fails; it moves the write pointer 'index'
  // to the next 'S' or 'M' for more robustness.
  // A minor issue with current implementation: imagine the following case:
  //  Two packets arrive in two UART calls.
  //  The first packet misses 1 byte, but the second one is complete.
  //  In this case, when the host_command buffer is filled
  //  (the last byte is 'S' or 'M' for the second packet), Handle() will be called. The whole buffer
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
    ProcessData();
    valid_packet_cnt++;
    flag = 1;
  } else {
    flag = 0;
  }
}

float MinipcPort::GetRelativeYaw(void) {
  return relative_yaw;
}

float MinipcPort::GetRelativePitch(void) {
  return relative_pitch;
}

uint16_t MinipcPort::GetSeqnum(void) {
  return seqnum;
}

uint32_t MinipcPort::GetValidPacketCnt(void) {
  return valid_packet_cnt;
}

void MinipcPort::ProcessData() {
  // Assume that the host_command is a complete and verified message

  // char pointer because host_command is a byte array
  uint8_t* seq_num_start = host_command + SEQNUM_OFFSET;
  uint8_t* rel_yaw_start = host_command + SEQNUM_OFFSET + sizeof(int32_t);
  uint8_t* rel_pitch_start = host_command + SEQNUM_OFFSET + sizeof(int32_t) * 2;

  seqnum = (*(uint16_t *)seq_num_start);
  relative_yaw = (*(int32_t *)rel_yaw_start) * 1.0f / this->INT_FP_SCALE;
  relative_pitch = *(int32_t *)rel_pitch_start *1.0f / this->INT_FP_SCALE;
}

uint8_t MinipcPort::GetValidFlag(void) {
  uint8_t temp = flag;
  flag = 0;
  return temp;
}
}  // namespace communication

