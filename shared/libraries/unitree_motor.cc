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

#include "unitree_motor.h"

#include <cstring>

namespace control {

void UnitreeMotor::ModifyData() {
  send.data.head.start[0] = 0xFE;
  send.data.head.start[1] = 0xEE;
  send.data.head.motorID = send.id;
  send.data.head.reserved = 0x0;
  send.data.Mdata.mode = send.mode;
  send.data.Mdata.ModifyBit = 0xFF;
  send.data.Mdata.ReadBit = 0x0;
  send.data.Mdata.reserved = 0x0;
  send.data.Mdata.Modify.L = 0;
  send.data.Mdata.T = send.T * 256;
  send.data.Mdata.W = send.W * 128;
  send.data.Mdata.Pos = (int)((send.Pos / 6.2832) * 16384.0);
  send.data.Mdata.K_P = send.K_P * 2048;
  send.data.Mdata.K_W = send.K_W * 1024;
  send.data.Mdata.LowHzMotorCmdIndex = 0;
  send.data.Mdata.LowHzMotorCmdByte = 0;
  send.data.Mdata.Res[0] = send.Res;
  send.data.CRCdata.u32 = crc32_core((uint32_t*)(&(send.data)), 7);
}

bool UnitreeMotor::ExtractData(const communication::package_t package) {
  memcpy(&(recv.data), package.data, package.length);
  if (recv.data.CRCdata.u32 != crc32_core((uint32_t*)(&(recv.data)), 18)) {
    recv.correct = false;
    return recv.correct;
  } else {
    recv.id = recv.data.head.motorID;
    recv.mode = recv.data.Mdata.mode;
    recv.Temp = recv.data.Mdata.Temp;
    recv.MError = recv.data.Mdata.MError;
    recv.T = ((float)recv.data.Mdata.T) / 256;
    recv.W = ((float)recv.data.Mdata.W) / 128;
    recv.LW = recv.data.Mdata.LW;
    recv.Acc = (int)recv.data.Mdata.Acc;
    recv.Pos = 6.2832 * ((float)recv.data.Mdata.Pos) / 16384;
    recv.gyro[0] = ((float)recv.data.Mdata.gyro[0]) * 0.00107993176;
    recv.gyro[1] = ((float)recv.data.Mdata.gyro[1]) * 0.00107993176;
    recv.gyro[2] = ((float)recv.data.Mdata.gyro[2]) * 0.00107993176;
    recv.acc[0] = ((float)recv.data.Mdata.acc[0]) * 0.0023911132;
    recv.acc[1] = ((float)recv.data.Mdata.acc[1]) * 0.0023911132;
    recv.acc[2] = ((float)recv.data.Mdata.acc[2]) * 0.0023911132;
    recv.correct = true;
    return recv.correct;
  }
}

void UnitreeMotor::Stop(const unsigned short motor_id) {
  send.id = motor_id;
  send.mode = 0;
  send.T = 0.0;
  send.W = 0.0;
  send.Pos = 0.0;
  send.K_P = 0.0;
  send.K_W = 0.0;
  ModifyData();
}

void UnitreeMotor::Test(const unsigned short motor_id) {
  send.id = motor_id;
  send.mode = 5;
  send.T = 0.0;
  send.W = 0.0;
  send.Pos = 0.0;
  send.K_P = 0.0;
  send.K_W = 0.0;
  ModifyData();
}

void UnitreeMotor::Control(const unsigned short motor_id, const float torque, const float speed, const float position, const float Kp, const float Kd) {
  send.id = motor_id;
  send.mode = 10;
  send.T = torque;
  send.W = speed * gear_ratio;
  send.Pos = position * gear_ratio;
  send.K_P = Kp;
  send.K_W = Kd;
  ModifyData();
}

uint32_t UnitreeMotor::crc32_core(uint32_t* ptr, uint32_t len) {
  uint32_t xbit;
  uint32_t data;
  uint32_t CRC32 = 0xFFFFFFFF;
  const uint32_t dwPolynomial = 0x04c11db7;
  for (uint32_t i = 0; i < len; ++i) {
    xbit = 1 << 31;
    data = ptr[i];
    for (uint32_t bits = 0; bits < 32; ++bits) {
      if (CRC32 & 0x80000000) {
        CRC32 <<= 1;
        CRC32 ^= dwPolynomial;
      } else
        CRC32 <<= 1;
      if (data & xbit)
        CRC32 ^= dwPolynomial;
      xbit >>= 1;
    }
  }
  return CRC32;
}

}  // namespace control
