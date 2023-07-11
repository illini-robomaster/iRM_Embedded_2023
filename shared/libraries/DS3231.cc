#include "DS3231.h"

namespace time {

DS3231::DS3231(I2C_HandleTypeDef* hi2c) {
  hi2c_ = hi2c;
}

uint8_t DS3231::BCD_DEC(uint8_t BCD_Data) {
  uint8_t DEC_Data;

  DEC_Data = BCD_Data & 0x0f;
  BCD_Data >>= 4;
  BCD_Data &= 0x0f;
  BCD_Data *= 10;
  DEC_Data += BCD_Data;
  return DEC_Data;
}

uint8_t DS3231::DEC_BCD(uint8_t DEC_Data) {
  uint8_t BCD_DataA, BCD_DataB, BCD_Data;

  BCD_DataA = DEC_Data / 10;
  BCD_DataB = DEC_Data % 10;
  BCD_Data = BCD_DataB + (BCD_DataA << 4);
  return BCD_Data;
}

bool DS3231::ReadData(uint8_t ReadAddr) {
  uint8_t DS3231_DataAddr[1];
  DS3231_DataAddr[0] = ReadAddr;

  if (HAL_I2C_Master_Transmit(hi2c_, (DS3231_ADDRESS | I2C_WR), DS3231_DataAddr, sizeof(DS3231_DataAddr), 1000) != HAL_OK)
    return false;

  if (HAL_I2C_Master_Receive(hi2c_, (DS3231_ADDRESS | I2C_RD), &DS3231Buffer, sizeof(DS3231Buffer), 1000) != HAL_OK)
    return false;

  return true;
}

bool DS3231::WriteData(uint8_t WriteAddr, uint8_t Data) {
  uint8_t DS3231_Data[2];
  DS3231_Data[0] = WriteAddr;
  DS3231_Data[1] = Data;

  if(HAL_I2C_Master_Transmit(hi2c_,(DS3231_ADDRESS | I2C_WR),DS3231_Data,sizeof(DS3231_Data), 1000) != HAL_OK)
    return false;

  return true;
}

bool DS3231::SetTime(uint8_t Yea, uint8_t Mon, uint8_t Dat, uint8_t Wee, uint8_t Hou, uint8_t Min, uint8_t Sec) {
  bool ret = true;

  ret &= WriteData(Second_Register,DEC_BCD(Sec));
  ret &= WriteData(Minute_Register,DEC_BCD(Min));
  ret &= WriteData(Hour_Register,DEC_BCD(Hou));
  ret &= WriteData(Day_Register,DEC_BCD(Wee));
  ret &= WriteData(Date_Register,DEC_BCD(Dat));
  ret &= WriteData(Month_Register,DEC_BCD(Mon));
  ret &= WriteData(Year_Register,DEC_BCD(Yea));

  return ret;
}

bool DS3231::ReadTime() {
  bool ret = true;

  ret &= ReadData(Second_Register);
  second = BCD_DEC(DS3231Buffer);

  ret &= ReadData(Minute_Register);
  minute = BCD_DEC(DS3231Buffer);

  ret &= ReadData(Hour_Register);
  hour = BCD_DEC(DS3231Buffer);

  ret &= ReadData(Day_Register);
  day = BCD_DEC(DS3231Buffer);

  ret &= ReadData(Date_Register);
  date = BCD_DEC(DS3231Buffer);

  ret &= ReadData(Month_Register);
  month = BCD_DEC(DS3231Buffer);

  ret &= ReadData(Year_Register);
  year = BCD_DEC(DS3231Buffer);

  return ret;
}

}
