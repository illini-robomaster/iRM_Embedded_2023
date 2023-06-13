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

#include "bsp_error_handler.h"
#include "protocol.h"

namespace control {

typedef int16_t q15_t;

typedef union {
  int32_t  L;
  uint8_t  u8[4];
  uint16_t u16[2];
  uint32_t u32;
  float    F;
} COMData32_t; 

typedef struct {
  unsigned char start[2];
  unsigned char motorID;
  unsigned char reserved;
} __packed COMHead_t;

typedef struct {
  uint8_t     mode;
  uint8_t     ModifyBit;
  uint8_t     ReadBit;
  uint8_t     reserved;

  COMData32_t Modify;

  q15_t       T;
  q15_t       W;
  int32_t     Pos;

  q15_t       K_P;
  q15_t       K_W;

  uint8_t     LowHzMotorCmdIndex;
  uint8_t     LowHzMotorCmdByte;
	
  COMData32_t Res[1];
} __packed MasterComd_t;

typedef struct {
  COMHead_t    head;
  MasterComd_t Mdata;
  COMData32_t  CRCdata;
} __packed MasterComdData_t;

typedef struct {
  uint8_t     mode;
  uint8_t     ReadBit;
  int8_t      Temp;
  uint8_t     MError;
 
  COMData32_t Read;
  int16_t     T;

  int16_t     W;
  float       LW;

  int16_t     W2;
  float       LW2;

  int16_t     Acc;
  int16_t     OutAcc;
		 
  int32_t     Pos;
  int32_t     Pos2;

  int16_t     gyro[3];
  int16_t     acc[3];   

  int16_t     Fgyro[3];
  int16_t     Facc[3];
  int16_t     Fmag[3];
  uint8_t     Ftemp;
    
  int16_t     Force16;
  int8_t      Force8;
		
  uint8_t     FError;
		
  int8_t      Res[1];
} __packed ServoComd_t;

typedef struct {
  COMHead_t   head;
  ServoComd_t Mdata;
  COMData32_t CRCdata;
} __packed ServoComdData_t;

typedef struct{
  MasterComdData_t data;
  unsigned short   id;
  unsigned short   mode;
  float            T;
  float            W;
  float            Pos;
  float            K_P;
  float            K_W;
  COMData32_t      Res;
} motor_send_t;

typedef struct {
  ServoComdData_t data;
  bool            correct;
  int             motor_id;
  int             mode;
  int             Temp;
  int             MError;
  float           T;
  float           W;
  float           LW;
  int             Acc;
  float           Pos;
  float           gyro[3];
  float           acc[3];
} motor_recv_t;

class UnitreeMotor {
public:
  void ModifyData();
  bool ExtractData(communication::package_t package);

  motor_send_t send;
  motor_recv_t recv;

  const int send_length = 34;
  const int recv_length = 78;
private:
  uint32_t crc32_core(uint32_t* ptr, uint32_t len);
};

}
