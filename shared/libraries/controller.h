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

/* NOTE(alvin): DSP libraries depends on macro definitions on FPU computability, so the
 *              main.h must be included before arm_math.h */
// clang-format off
#include "main.h"
// clang-format on

#include "arm_math.h"

namespace control {

/**
 * @brief simple PID controller
 */
class PIDController {
 public:
  /**
   * @brief PID controller default constructor
   */
  PIDController();

  /**
   * @brief PID controller constructor
   *
   * @param kp proportional gain
   * @param ki integral gain
   * @param kd derivative gain
   */
  PIDController(float kp, float ki, float kd);

  /**
   * @brief PID controller constructor
   *
   * @param param gains of PID controller, formated as [kp, ki, kd]
   */
  PIDController(float* param);

  /**
   * @brief compute output base on current error
   *
   * @param error error of the system, i.e. (target - actual)
   *
   * @return output value that could potentially drive the error to 0
   */
  float ComputeOutput(float error);

  /**
   * @brief compute output base on current error but constraint to range of int16_t
   *
   * @param error error of the system, i.e. (target - actual)
   *
   * @return output value that could potentially drive the error to 0,
   *         floored at -32768, ceiled at 32767
   */
  int16_t ComputeConstrainedOutput(float error);

  /**
   * @brief reinitialize the pid instance using another set of gains, but does not clear
   *        current status
   *
   * @param kp new proportional gain
   * @param ki new integral gain
   * @param kd new derivative gain
   */
  void Reinit(float kp, float ki, float kd);

  /**
   * @brief reinitialize the pid instance using another set of gains, but does not clear
   *        current status
   *
   * @param param gains of PID controller, formated as [kp, ki, kd]
   */
  void Reinit(float* param);

  /**
   * @brief clear the remembered states of the controller
   */
  void Reset();

 private:
  arm_pid_instance_f32 pid_f32_;
};

class ConstrainedPID {
 public:
  ConstrainedPID();
  /**
   * @brief PID controller constructor
   *
   * @param kp proportional gain
   * @param ki integral gain
   * @param kd derivative gain
   */
  ConstrainedPID(float kp, float ki, float kd, float max_iout, float max_out);

  /**
   * @brief PID controller constructor
   *
   * @param param gains of PID controller, formated as [kp, ki, kd]
   */
  ConstrainedPID(float* param, float max_iout, float max_out);

  /**
   * @brief compute output base on current error
   *
   * @param error   error of the system, i.e. (target - actual)
   * @param max_out maximum output possible for this pid
   *
   * @return output value that could potentially drive the error to 0
   */
  float ComputeOutput(float error);

  int16_t ComputeConstrainedOutput(float error);

  /**
   * @brief reinitialize the pid instance using another set of gains, but does not clear
   *        current status
   *
   * @param kp new proportional gain
   * @param ki new integral gain
   * @param kd new derivative gain
   */
  void Reinit(float kp, float ki, float kd, float max_iout, float max_out);

  /**
   * @brief reinitialize the pid instance using another set of gains, but does not clear
   *        current status
   *
   * @param param gains of PID controller, formated as [kp, ki, kd]
   */
  void Reinit(float* param, float max_iout, float max_out);

  /**
   * @brief clear the remembered states of the controller
   */
  void Reset();

  void ChangeMax(float max_iout, float max_out);

  float kp_;
  float ki_;
  float kd_;

  float cumulated_err_;

 private:
  float last_err_;

  float max_iout_;
  float max_out_;
};

} /* namespace control */
