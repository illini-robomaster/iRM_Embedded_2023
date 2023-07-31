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

#include "lidar07.h"

namespace distance {

LIDAR07_UART::LIDAR07_UART(UART_HandleTypeDef* huart, lidar07_delay_t delay_func)
    : bsp::UART(huart) {
  SetupRx(100);
  SetupTx(100);
  delay_func_ = delay_func;
}

bool LIDAR07_UART::begin() {
  readVersionPacket.head = 0xF5;
  readVersionPacket.command = 0x43;
  readVersionPacket.data[0] = 0x00;
  readVersionPacket.data[1] = 0x00;
  readVersionPacket.data[2] = 0x00;
  readVersionPacket.data[3] = 0x00;
  readVersionPacket.checkData[0] = 0xAC;
  readVersionPacket.checkData[1] = 0x45;
  readVersionPacket.checkData[2] = 0x62;
  readVersionPacket.checkData[3] = 0x3B;

  setModePacket.head = 0xF5;
  setModePacket.command = 0xE1;
  setModePacket.data[0] = 0x00;
  setModePacket.data[1] = 0x00;
  setModePacket.data[2] = 0x00;
  setModePacket.data[3] = 0x00;
  setModePacket.checkData[0] = 0xA5;
  setModePacket.checkData[1] = 0x8D;
  setModePacket.checkData[2] = 0x89;
  setModePacket.checkData[3] = 0xA7;

  startPacket.head = 0xF5;
  startPacket.command = 0xE0;
  startPacket.data[0] = 0x01;
  startPacket.data[1] = 0x00;
  startPacket.data[2] = 0x00;
  startPacket.data[3] = 0x00;
  startPacket.checkData[0] = 0x9F;
  startPacket.checkData[1] = 0x70;
  startPacket.checkData[2] = 0xE9;
  startPacket.checkData[3] = 0x32;

  stopPacket.head = 0xF5;
  stopPacket.command = 0xE0;
  stopPacket.data[0] = 0x00;
  stopPacket.data[1] = 0x00;
  stopPacket.data[2] = 0x00;
  stopPacket.data[3] = 0x00;
  stopPacket.checkData[0] = 0x28;
  stopPacket.checkData[1] = 0xEA;
  stopPacket.checkData[2] = 0x84;
  stopPacket.checkData[3] = 0xEE;

  startFilterPacket.head = 0xF5;
  startFilterPacket.command = 0xD9;
  startFilterPacket.data[0] = 0x01;
  startFilterPacket.data[1] = 0x00;
  startFilterPacket.data[2] = 0x00;
  startFilterPacket.data[3] = 0x00;
  startFilterPacket.checkData[0] = 0xB7;
  startFilterPacket.checkData[1] = 0x1F;
  startFilterPacket.checkData[2] = 0xBA;
  startFilterPacket.checkData[3] = 0xBA;

  stopFilterPacket.head = 0xF5;
  stopFilterPacket.command = 0xD9;
  stopFilterPacket.data[0] = 0x00;
  stopFilterPacket.data[1] = 0x00;
  stopFilterPacket.data[2] = 0x00;
  stopFilterPacket.data[3] = 0x00;
  stopFilterPacket.checkData[0] = 0x00;
  stopFilterPacket.checkData[1] = 0x85;
  stopFilterPacket.checkData[2] = 0xD7;
  stopFilterPacket.checkData[3] = 0x66;

  bool ret = false;
  uint8_t* buff;

  Write((uint8_t*)&readVersionPacket, sizeof(SendPacket_t));
  Read(&buff);

  if (crc32Check(buff, 8)) {
    readValue(buff, LIDAR07_VERSION);
    ret = true;
  }
  return setMeasureMode() && ret;
}

bool LIDAR07_UART::setMeasureMode() {
  uint8_t* buff;

  delay_func_(20);

  Write((uint8_t*)&setModePacket, sizeof(SendPacket_t));
  Read(&buff);

  return crc32Check(buff, 8);
}

bool LIDAR07_UART::startMeasure() {
  uint8_t* buff;
  bool ret = false;

  Write((uint8_t*)&startPacket, sizeof(SendPacket_t));
  Read(&buff);
  if (crc32Check(buff, 20)) {
    readValue(buff, LIDAR07_MEASURE);
    ret = true;
  }
  return ret;
}

void LIDAR07_UART::stopMeasure() { Write((uint8_t*)&stopPacket, sizeof(SendPacket_t)); }

bool LIDAR07_UART::startFilter() {
  uint8_t* buff;

  Write((uint8_t*)&startFilterPacket, sizeof(SendPacket_t));
  Read(&buff);

  return crc32Check(buff, 8);
}

bool LIDAR07_UART::stopFilter() {
  uint8_t* buff;

  Write((uint8_t*)&stopFilterPacket, sizeof(SendPacket_t));
  Read(&buff);

  return crc32Check(buff, 8);
}

bool LIDAR07_UART::crc32Check(uint8_t* buff, uint8_t len) {
  uint8_t data;
  uint8_t ret = false;
  uint32_t checkData;
  uint32_t crc = 0xFFFFFFFF;  // Original value
  uint8_t buffCheck[len];

  checkData = (uint32_t)buff[len] | ((uint32_t)buff[len + 1] << 8) |
              ((uint32_t)buff[len + 2] << 16) | ((uint32_t)buff[len + 3] << 24);
  for (uint8_t i = 0; i < len; i++) buffCheck[i] = buff[i];
  for (uint8_t i = 0; i < len; i++) {
    data = buffCheck[i];
    crc = crc ^ ((uint32_t)data << 24);  // 8 bits XOR higher than the crc original value
    for (uint8_t y = 0; y < 8; y++) {    // Cycle 8 bits
      if (crc & 0x80000000)              // The bit shifted to the left is 1, and after shifting to the left, it
                                         // is XORed with the polynomial
        crc = (crc << 1) ^ 0x04C11DB7;
      else  // Or shift to left directly
        crc <<= 1;
    }
  }
  crc = crc ^ 0x00;  // At last, return to XOR with the XOR value of result
  if (crc == checkData) ret = true;
  return ret;  // Return the final check value
}

void LIDAR07_UART::readValue(uint8_t* buff, uint8_t type) {
  if (type == LIDAR07_VERSION) {
    version = (uint32_t)buff[4] | ((uint32_t)buff[5] << 8) | ((uint32_t)buff[6] << 16) |
              ((uint32_t)buff[7] << 24);
  } else if (type == LIDAR07_MEASURE) {
    distance = (uint16_t)buff[4] | ((uint16_t)buff[5] << 8);
    amplitude = (uint16_t)buff[8] | ((uint16_t)buff[9] << 8);
  }
}

LIDAR07_IIC::LIDAR07_IIC(I2C_HandleTypeDef* hi2c, uint16_t devAddr, lidar07_delay_t delay_func) {
  hi2c_ = hi2c;
  devAddr_ = devAddr << 1;
  delay_func_ = delay_func;
}

bool LIDAR07_IIC::IsReady() { return HAL_I2C_IsDeviceReady(hi2c_, devAddr_, 1, 10) == HAL_OK; }

bool LIDAR07_IIC::begin() {
  readVersionPacket.head = 0x00;
  readVersionPacket.command = 0x43;
  readVersionPacket.data[0] = 0x00;
  readVersionPacket.data[1] = 0x00;
  readVersionPacket.data[2] = 0x00;
  readVersionPacket.data[3] = 0x00;
  readVersionPacket.checkData[0] = 0x55;
  readVersionPacket.checkData[1] = 0x10;
  readVersionPacket.checkData[2] = 0xCD;
  readVersionPacket.checkData[3] = 0x9A;

  setModePacket.head = 0x00;
  setModePacket.command = 0xE1;
  setModePacket.data[0] = 0x00;
  setModePacket.data[1] = 0x00;
  setModePacket.data[2] = 0x00;
  setModePacket.data[3] = 0x00;
  setModePacket.checkData[0] = 0x5C;
  setModePacket.checkData[1] = 0xD8;
  setModePacket.checkData[2] = 0x26;
  setModePacket.checkData[3] = 0x06;

  startPacket.head = 0x00;
  startPacket.command = 0xE0;
  startPacket.data[0] = 0x01;
  startPacket.data[1] = 0x00;
  startPacket.data[2] = 0x00;
  startPacket.data[3] = 0x00;
  startPacket.checkData[0] = 0x66;
  startPacket.checkData[1] = 0x25;
  startPacket.checkData[2] = 0x46;
  startPacket.checkData[3] = 0x93;

  stopPacket.head = 0x00;
  stopPacket.command = 0xE0;
  stopPacket.data[0] = 0x00;
  stopPacket.data[1] = 0x00;
  stopPacket.data[2] = 0x00;
  stopPacket.data[3] = 0x00;
  stopPacket.checkData[0] = 0xD1;
  stopPacket.checkData[1] = 0xBF;
  stopPacket.checkData[2] = 0x2B;
  stopPacket.checkData[3] = 0x4F;

  startFilterPacket.head = 0x00;
  startFilterPacket.command = 0xD9;
  startFilterPacket.data[0] = 0x01;
  startFilterPacket.data[1] = 0x00;
  startFilterPacket.data[2] = 0x00;
  startFilterPacket.data[3] = 0x00;
  startFilterPacket.checkData[0] = 0x4E;
  startFilterPacket.checkData[1] = 0x4A;
  startFilterPacket.checkData[2] = 0x15;
  startFilterPacket.checkData[3] = 0x1B;

  stopFilterPacket.head = 0x00;
  stopFilterPacket.command = 0xD9;
  stopFilterPacket.data[0] = 0x00;
  stopFilterPacket.data[1] = 0x00;
  stopFilterPacket.data[2] = 0x00;
  stopFilterPacket.data[3] = 0x00;
  stopFilterPacket.checkData[0] = 0xF9;
  stopFilterPacket.checkData[1] = 0xD0;
  stopFilterPacket.checkData[2] = 0x78;
  stopFilterPacket.checkData[3] = 0xC7;

  bool ret = false;
  uint8_t buff[12];

  HAL_I2C_Master_Transmit(hi2c_, devAddr_, (uint8_t*)&readVersionPacket, sizeof(SendPacket_t), 10);
  delay_func_(20);
  HAL_I2C_Master_Receive(hi2c_, devAddr_, buff, 12, 10);

  if (crc32Check(buff, 8)) {
    readValue(buff, LIDAR07_VERSION);
    ret = true;
  }
  return setMeasureMode() && ret;
}

bool LIDAR07_IIC::setMeasureMode() {
  uint8_t buff[12];

  delay_func_(20);

  HAL_I2C_Master_Transmit(hi2c_, devAddr_, (uint8_t*)&setModePacket, sizeof(SendPacket_t), 10);
  HAL_I2C_Master_Receive(hi2c_, devAddr_, buff, 12, 10);

  return crc32Check(buff, 8);
}

bool LIDAR07_IIC::startMeasure() {
  uint8_t buff[24];
  bool ret = false;

  HAL_I2C_Master_Transmit(hi2c_, devAddr_, (uint8_t*)&setModePacket, sizeof(SendPacket_t), 10);
  delay_func_(20);
  HAL_I2C_Master_Receive(hi2c_, devAddr_, buff, 24, 10);

  if (crc32Check(buff, 20)) {
    readValue(buff, LIDAR07_MEASURE);
    ret = true;
  }
  return ret;
}

void LIDAR07_IIC::stopMeasure() {
  HAL_I2C_Master_Transmit(hi2c_, devAddr_, (uint8_t*)&stopPacket, sizeof(SendPacket_t), 10);
}

bool LIDAR07_IIC::startFilter() {
  uint8_t buff[12];

  HAL_I2C_Master_Transmit(hi2c_, devAddr_, (uint8_t*)&startFilterPacket, sizeof(SendPacket_t), 10);
  delay_func_(20);
  HAL_I2C_Master_Receive(hi2c_, devAddr_, buff, 12, 10);

  return crc32Check(buff, 8);
}

bool LIDAR07_IIC::stopFilter() {
  uint8_t buff[12];

  HAL_I2C_Master_Transmit(hi2c_, devAddr_, (uint8_t*)&stopFilterPacket, sizeof(SendPacket_t), 10);
  delay_func_(20);
  HAL_I2C_Master_Receive(hi2c_, devAddr_, buff, 12, 10);

  return crc32Check(buff, 8);
}

bool LIDAR07_IIC::crc32Check(uint8_t* buff, uint8_t len) {
  uint8_t data;
  uint8_t ret = false;
  uint32_t checkData;
  uint32_t crc = 0xFFFFFFFF;  // Original value
  uint8_t buffCheck[len];

  checkData = (uint32_t)buff[len - 1] | ((uint32_t)buff[len] << 8) |
              ((uint32_t)buff[len + 1] << 16) | ((uint32_t)buff[len + 2] << 24);
  buffCheck[0] = 0xFA;
  for (uint8_t i = 0; i < len - 1; i++) buffCheck[i + 1] = buff[i];
  for (uint8_t i = 0; i < len; i++) {
    data = buffCheck[i];
    crc = crc ^ ((uint32_t)data << 24);  // 8 bits XOR higher than the crc original value
    for (uint8_t y = 0; y < 8; y++) {    // Cycle 8 bits
      if (crc & 0x80000000)              // The bit shifted to the left is 1, and after shifting to the left, it
                                         // is XORed with the polynomial
        crc = (crc << 1) ^ 0x04C11DB7;
      else  // Or shift to left directly
        crc <<= 1;
    }
  }
  crc = crc ^ 0x00;  // At last, return to XOR with the XOR value of result
  if (crc == checkData) ret = true;
  return ret;  // Return the final check value
}

void LIDAR07_IIC::readValue(uint8_t* buff, uint8_t type) {
  if (type == LIDAR07_VERSION) {
    version = (uint32_t)buff[3] | ((uint32_t)buff[4] << 8) | ((uint32_t)buff[5] << 16) |
              ((uint32_t)buff[6] << 24);
  } else if (type == LIDAR07_MEASURE) {
    distance = (uint16_t)buff[3] | ((uint16_t)buff[4] << 8);
    amplitude = (uint16_t)buff[7] | ((uint16_t)buff[8] << 8);
  }
}

}  // namespace distance
