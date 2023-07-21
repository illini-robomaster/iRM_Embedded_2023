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
#include "i2c.h"

#define LIDAR07_VERSION 0
#define LIDAR07_MEASURE 1

namespace distance {

typedef struct {
  uint8_t head;
  uint8_t command;
  uint8_t data[4];
  uint8_t checkData[4];
} __packed SendPacket_t;

typedef void (*lidar07_delay_t)(uint32_t milli);

class LIDAR07_UART : public bsp::UART {
 public:
  LIDAR07_UART(
      UART_HandleTypeDef* huart,
      lidar07_delay_t delay_func = [](uint32_t milli) { HAL_Delay(milli); });

  bool begin();
  bool setMeasureMode();
  bool startMeasure();
  void stopMeasure();
  bool startFilter();
  bool stopFilter();

  uint16_t distance;
  uint16_t amplitude;
  uint32_t version;

 private:
  lidar07_delay_t delay_func_ = nullptr;

  bool crc32Check(uint8_t* buff, uint8_t len);
  void readValue(uint8_t* buff, uint8_t type);

  SendPacket_t readVersionPacket;
  SendPacket_t setIntervalPacket;
  SendPacket_t setModePacket;
  SendPacket_t startPacket;
  SendPacket_t stopPacket;
  SendPacket_t startFilterPacket;
  SendPacket_t stopFilterPacket;
};

// TODO: i2c is not working currently for lidar07, please fix it in the future. it may due to the
// 400k i2c frequency required by lidar07
class LIDAR07_IIC {
 public:
  LIDAR07_IIC(
      I2C_HandleTypeDef* hi2c, uint16_t devAddr,
      lidar07_delay_t delay_func = [](uint32_t milli) { HAL_Delay(milli); });
  bool IsReady();
  bool begin();
  bool setMeasureMode();
  bool startMeasure();
  void stopMeasure();
  bool startFilter();
  bool stopFilter();

  uint16_t distance = 0;
  uint16_t amplitude = 0;
  uint32_t version = 0;

 private:
  I2C_HandleTypeDef* hi2c_;
  uint16_t devAddr_;

  lidar07_delay_t delay_func_ = nullptr;

  bool crc32Check(uint8_t* buff, uint8_t len);
  void readValue(uint8_t* buff, uint8_t type);

  SendPacket_t readVersionPacket;
  SendPacket_t setIntervalPacket;
  SendPacket_t setModePacket;
  SendPacket_t startPacket;
  SendPacket_t stopPacket;
  SendPacket_t startFilterPacket;
  SendPacket_t stopFilterPacket;
};

}  // namespace distance
