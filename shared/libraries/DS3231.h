#pragma once

#include "main.h"
#include "i2c.h"

namespace time {

class DS3231 {
 public:
  DS3231(I2C_HandleTypeDef* hi2c);

  bool SetTime(uint8_t Yea,uint8_t Mon,uint8_t Dat,uint8_t Wee,uint8_t Hou,uint8_t Min,uint8_t Sec);
  bool ReadTime();

  int second = 0;
  int minute = 0;
  int hour = 0;
  int day = 0;
  int date = 0;
  int month = 0;
  int year = 0;

 private:
  I2C_HandleTypeDef* hi2c_;

  uint8_t BCD_DEC(uint8_t BCD_Data);
  uint8_t DEC_BCD(uint8_t DEC_Data);
  bool ReadData(uint8_t ReadAddr);
  bool WriteData(uint8_t WriteAddr,uint8_t Data);

  uint8_t DS3231Buffer = 0x00;

  const uint8_t I2C_WR = 0x00;
  const uint8_t I2C_RD = 0x01;
  const uint8_t DS3231_ADDRESS = 0xd0;

  const uint8_t Second_Register = 0x00;
  const uint8_t Minute_Register = 0x01;
  const uint8_t Hour_Register = 0x02;
  const uint8_t Day_Register = 0x03;
  const uint8_t Date_Register = 0x04;
  const uint8_t Month_Register = 0x05;
  const uint8_t Year_Register = 0x06;
};

}
