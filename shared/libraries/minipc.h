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

struct STMToJetsonData {
  char header[2];
  uint8_t my_color; // RED is 0; BLUE is one
  uint8_t crc8_checksum;
  char tail[2];
};

// WARNING: THIS CLASS IS NOT THREAD SAFE!!!

class MiniPCProtocol {
 public:
  MiniPCProtocol();
  void Receive(const uint8_t* data, uint8_t len);
  // dummy send
  void Send(STMToJetsonData* packet, uint8_t color);
  uint8_t get_valid_flag(void);
  float get_relative_yaw(void);
  float get_relative_pitch(void);
  uint32_t get_seqnum(void);
  uint32_t get_valid_packet_cnt(void);

 private:
  // For definitions of constants, check out the documentation at either
  // https://github.com/illini-robomaster/iRM_Vision_2023/blob/roger/crc_comm/docs/comm_protocol.md
  // or https://github.com/illini-robomaster/iRM_Vision_2023/tree/docs/comm_protocol.md
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
  uint32_t seqnum;
  uint32_t valid_packet_cnt = 0;
}; /* class MiniPCProtocol */

} /* namespace communication */
