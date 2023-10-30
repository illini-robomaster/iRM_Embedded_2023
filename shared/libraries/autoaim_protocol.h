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

#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "crc8.h"

namespace communication {

// WARNING: CRC8 works only for total length < 64 bytes.
typedef struct {
  char header[2];
  uint16_t seq_num;
  // data_length might not be necessary because all command length are fixed
  uint8_t data_length; // length of data as in bytes, data length as an arry * 4 = data_length
  uint8_t cmd_id;
  int32_t *data;
  uint8_t crc8_checksum;
  char tail[2];
} __packed minipc_data_t;
// This might be not necessary at all

// RED is 0; BLUE is one
typedef struct {
  uint8_t my_color;
} __packed color_data_t;

typedef struct {
  float rel_yaw;
  float rel_pitch;
  // Search Target is 0. Move Yoke is 1.
  uint8_t mode;
  uint8_t debug_info;
} __packed gimbal_data_t;

typedef struct {
  float vx;
  float vy;
  float vw;
} __packed chassis_data_t;

// GIMBAL_CMD_ID  : 0x00 Autoaim gimbal RelYaw RelPitch
// COLOR_CMD_ID   : 0x01
// CHASSIS_CMD_ID : 0x02
// TOTAL_NUM_OF_ID: length of the enum
enum CMD_ID {GIMBAL_CMD_ID,
             COLOR_CMD_ID,
             CHASSIS_CMD_ID,
             TOTAL_NUM_OF_ID};

// WARNING: THIS CLASS IS NOT THREAD SAFE!!!
class MinipcPort {
 public:
  MinipcPort();
  void Receive(const uint8_t* data, uint8_t len);
  uint8_t GetValidFlag(void);
  float GetRelativeYaw(void);
  float GetRelativePitch(void);
  uint16_t GetSeqnum(void);
  uint32_t GetValidPacketCnt(void);

  /**
   * @brief Pack data into a packet array
   * @note For the smallest length of the packet, see CMD_TO_LEN[] or GetPacketLength()
   */
  void Pack(uint8_t* packet, void* data, uint8_t cmd_id);
  void PackGimbalData(uint8_t* packet, gimbal_data_t* data);
  void PackColorData(uint8_t* packet, color_data_t* data);
  void PackChassisData(uint8_t* packet, chassis_data_t* data);

  /**
   * Length of the data section ONLY in bytes. Header/tail/crc8 (total len = 9) NOT included.
   * Gimbal  CMD: id = 0x00, length = 19 - 9 = 10
   * Color   CMD: id = 0x01, length = 10 - 9 = 1
   * Chassis CMD: id = 0x02, length = 21 - 9 = 12
   */
  static constexpr uint8_t CMD_TO_LEN[TOTAL_NUM_OF_ID] = {
                                                sizeof(gimbal_data_t),
                                                sizeof(color_data_t),
                                                sizeof(chassis_data_t),
                                                };
  static constexpr uint8_t MAX_PACKET_LENGTH = 21;
  static constexpr uint8_t MIN_PACKET_LENGTH = 10;
  /**
   * @brief Total length of packet in bytes
   *  Header/tail/crc8 included.
   */
  uint8_t GetPacketLen(uint8_t cmd_id);

 private:
  // For definitions of constants, check out the documentation at either
  // https://github.com/illini-robomaster/iRM_Vision_2023/blob/roger/crc_comm/docs/comm_protocol.md
  // or https://github.com/illini-robomaster/iRM_Vision_2023/tree/docs/comm_protocol.md
  static constexpr uint8_t SEQNUM_OFFSET = 2;
  static constexpr uint8_t DATA_LENGTH_OFFSET = SEQNUM_OFFSET + 2;
  static constexpr uint8_t CMD_ID_OFFSET = DATA_LENGTH_OFFSET + 1;
  static constexpr uint8_t DATA_OFFSET = CMD_ID_OFFSET + 1;


  /**
   * @brief Add header and tail to the packet array based on cmd_id
   * @note For the smallest length of the packet, see CMD_TO_LEN[]
   *       The sum of length for header and tail is 8 bytes
   */
  void AddHeaderTail (uint8_t* packet, uint8_t cmd_id);

  /**
   * @brief Add CRC8 checksum for the packet array based on cmd_id
   *       CRC8 calulated based on the entire array except the tail ('ED')
   * @note Only call this function after packet has data and header/tails written
   *       The length of CRC8 checksum is 1 byte
   */
  void AddCRC8 (uint8_t* packet, int8_t cmd_id);

  int index;
  uint8_t flag;
  //uint8_t host_command[PKG_LEN];
  void Handle();
  void ProcessData();

  float relative_yaw;
  float relative_pitch;
  uint16_t seqnum;
  uint32_t valid_packet_cnt = 0;
}; /* class AutoaimProtocol */

} /* namespace communication */

