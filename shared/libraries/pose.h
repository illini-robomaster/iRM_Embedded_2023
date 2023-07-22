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

#include "bsp_imu.h"

namespace control {

class Pose {
 public:
  /** @description: constructor for Pose
   *  @param: _imu: imu pointer
   **/
  Pose(bsp::MPU6500* _imu);

  /** @description: reset all estimated pose to 0 and refresh timestamp
   *
   **/
  void PoseInit(void);

  /** @description: naive calibration for all offsets
   *                read from IMU and compute the average for 100 times
   *  @param: num, IMU reads to compute the average offset
   *  @note : by default takes 1s to calibrate
   **/
  void Calibrate(void);

  /** @description: naive calibration for all offsets
   *                read from IMU and compute the average for given times
   *  @param: num, IMU reads to compute the average offset
   *  @note : it takes _num * 10ms to calibrate
   **/
  void Calibrate(int16_t _num);

  /** @description: naive estimation for gravity
   *  @note : average 100 measure of z acceleration. Takes 1s.
   **/
  float GetGravity(void);

  /** @description: set offset (bias correction) for the 6 inputs
   *  @param: Offset (Bias) for the 3 axis of acce and gyro
   *  @note: The mean of all 6 axis except acc_z_off should be 0 after setting this.
   *         NOTE: acc_z_off is suggested to be set to 0,
   *               see pose.cc Calibrate() for detail.
   **/
  void SetOffset(float acc_x_off, float acc_y_off, float acc_z_off, float gyro_x_off,
                 float gyro_y_off, float gyro_z_off);

  /** @description: read estimated pitch in RAD
   *  @note: this error gets larger as ROLL gets larger. When ROLL get close to +/-90 deg, this
   *value is not usable.
   **/
  float GetPitch(void);

  /** @description: read estimated roll in RAD
   *  @note: this error gets larger as PITCH gets larger. When PITCH get close to +/-90 deg, this
   *value is not usable.
   **/
  float GetRoll(void);

  /** @description: read estimated yaw in RAD
   *  @note: this function is only based on gyro and it will drift.
   **/
  float GetYaw(void);

  /** @description: set complementary filter weight
   *  @note: should be close to 1.0
   **/
  void SetAlpha(float _alpha);

  /** @description: update pose with complementary filter from IMU's gyro and acce
   *  @note: delay between 2 ComplementaryFilterUpdate() calls should be as small as possible
   *         ideally ~10ms
   *  @reference: https://www.pieter-jan.com/node/11
   **/
  void ComplementaryFilterUpdate(void);

 private:
  bsp::MPU6500* imu;
  // last pose
  float pitch;
  float roll;
  float yaw;

  // noise bias correction offset for 3 axis of acce and gyro
  float acc_x_off;
  float acc_y_off;
  float acc_z_off;
  float gyro_x_off;
  float gyro_y_off;
  float gyro_z_off;

  // last timestamp
  uint32_t timestamp;

  // alpha used in complementary filter
  float alpha;

  // pose estimated by acce alone
  float pitchAcc;
  float rollAcc;

};  // class Pose end

} /* namespace control */
