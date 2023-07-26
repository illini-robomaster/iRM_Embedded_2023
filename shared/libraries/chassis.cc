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

#include "chassis.h"

#include "bsp_error_handler.h"

namespace control {

Chassis::Chassis(const chassis_t chassis) : pids_() {
  // acquired from user
  model_ = chassis.model;

  // data initialization using acquired model
  switch (chassis.model) {
    case CHASSIS_MECANUM_WHEEL:
    case CHASSIS_OMNI_WHEEL: {
      motors_ = new MotorCANBase*[FourWheel::motor_num];
      motors_[FourWheel::front_left] = chassis.motors[FourWheel::front_left];
      motors_[FourWheel::front_right] = chassis.motors[FourWheel::front_right];
      motors_[FourWheel::back_left] = chassis.motors[FourWheel::back_left];
      motors_[FourWheel::back_right] = chassis.motors[FourWheel::back_right];

      float* pid_param = new float[3]{40, 3, 0};
      float motor_max_iout = 2000;
      float motor_max_out = 20000;
      pids_[FourWheel::front_left].Reinit(pid_param, motor_max_iout, motor_max_out);
      pids_[FourWheel::front_right].Reinit(pid_param, motor_max_iout, motor_max_out);
      pids_[FourWheel::back_left].Reinit(pid_param, motor_max_iout, motor_max_out);
      pids_[FourWheel::back_right].Reinit(pid_param, motor_max_iout, motor_max_out);

      power_limit_ = new PowerLimit(FourWheel::motor_num);

      speeds_ = new float[FourWheel::motor_num];
      for (int i = 0; i < FourWheel::motor_num; ++i) speeds_[i] = 0;
      break;
    }
    case CHASSIS_ONE_WHEEL: {
      motors_ = new MotorCANBase*[OneWheel::motor_num];
      motors_[OneWheel::center] = chassis.motors[OneWheel::center];

      float* pid_param = new float[3]{140000, 20000, 0};
      float motor_max_iout = 30000;
      float motor_max_out = 30000;
      pids_[FourWheel::front_left].Reinit(pid_param, motor_max_iout, motor_max_out);

      power_limit_ = new PowerLimit(OneWheel::motor_num);

      speeds_ = new float[OneWheel::motor_num];
      for (int i = 0; i < OneWheel::motor_num; ++i) speeds_[i] = 0;
      break;
    }
    default:
      RM_ASSERT_TRUE(false, "Not Supported Chassis Mode\r\n");
  }
}

Chassis::~Chassis() {
  switch (model_) {
    case CHASSIS_MECANUM_WHEEL:
    case CHASSIS_OMNI_WHEEL: {
      motors_[FourWheel::front_left] = nullptr;
      motors_[FourWheel::front_right] = nullptr;
      motors_[FourWheel::back_left] = nullptr;
      motors_[FourWheel::back_right] = nullptr;
      delete[] motors_;
      motors_ = nullptr;

      delete[] speeds_;
      speeds_ = nullptr;
      break;
    }
    case CHASSIS_ONE_WHEEL: {
      motors_[OneWheel::center] = nullptr;
      delete[] motors_;
      motors_ = nullptr;

      delete[] speeds_;
      speeds_ = nullptr;
      break;
    }
    default:
      RM_ASSERT_TRUE(false, "Not Supported Chassis Mode\r\n");
  }
}

void Chassis::SetSpeed(const float x_speed, const float y_speed, const float turn_speed) {
  switch (model_) {
    case CHASSIS_MECANUM_WHEEL:
    case CHASSIS_OMNI_WHEEL: {
      constexpr int MAX_ABS_CURRENT = 12288;  // refer to MotorM3508 for details
      float move_sum = fabs(x_speed) + fabs(y_speed) + fabs(turn_speed);
      float scale = move_sum >= MAX_ABS_CURRENT ? MAX_ABS_CURRENT / move_sum : 1;

      speeds_[FourWheel::front_left] = scale * (y_speed + x_speed + turn_speed);
      speeds_[FourWheel::back_left] = scale * (y_speed - x_speed + turn_speed);
      speeds_[FourWheel::front_right] = -scale * (y_speed - x_speed - turn_speed);
      speeds_[FourWheel::back_right] = -scale * (y_speed + x_speed - turn_speed);
      break;
    }
    case CHASSIS_ONE_WHEEL: {
      speeds_[OneWheel::center] = x_speed;
      break;
    }
    default:
      RM_ASSERT_TRUE(false, "Not Supported Chassis Mode\r\n");
  }
}

void Chassis::Update(bool power_limit_on, float power_limit, float chassis_power,
                     float chassis_power_buffer) {
  switch (model_) {
    case CHASSIS_MECANUM_WHEEL:
    case CHASSIS_OMNI_WHEEL: {
      power_limit_info_.power_limit = power_limit;
      power_limit_info_.WARNING_power = power_limit * 0.9;
      power_limit_info_.WARNING_power_buff = 50;
      power_limit_info_.buffer_total_current_limit = 3500 * FourWheel::motor_num;
      power_limit_info_.power_total_current_limit =
          5000 * FourWheel::motor_num / 80.0 * power_limit;

      float PID_output[FourWheel::motor_num];
      float output[FourWheel::motor_num];

      PID_output[FourWheel::front_left] = pids_[FourWheel::front_left].ComputeOutput(
          motors_[FourWheel::front_left]->GetOmegaDelta(speeds_[FourWheel::front_left]));
      PID_output[FourWheel::back_left] = pids_[FourWheel::back_left].ComputeOutput(
          motors_[FourWheel::back_left]->GetOmegaDelta(speeds_[FourWheel::back_left]));
      PID_output[FourWheel::front_right] = pids_[FourWheel::front_right].ComputeOutput(
          motors_[FourWheel::front_right]->GetOmegaDelta(speeds_[FourWheel::front_right]));
      PID_output[FourWheel::back_right] = pids_[FourWheel::back_right].ComputeOutput(
          motors_[FourWheel::back_right]->GetOmegaDelta(speeds_[FourWheel::back_right]));

      power_limit_->Output(power_limit_on, power_limit_info_, chassis_power, chassis_power_buffer,
                           PID_output, output);

      motors_[FourWheel::front_left]->SetOutput(
          control::ClipMotorRange(output[FourWheel::front_left]));
      motors_[FourWheel::back_left]->SetOutput(
          control::ClipMotorRange(output[FourWheel::back_left]));
      motors_[FourWheel::front_right]->SetOutput(
          control::ClipMotorRange(output[FourWheel::front_right]));
      motors_[FourWheel::back_right]->SetOutput(
          control::ClipMotorRange(output[FourWheel::back_right]));
      break;
    }
    case CHASSIS_ONE_WHEEL: {
      power_limit_info_.power_limit = power_limit;
      power_limit_info_.WARNING_power = power_limit * 0.9;
      power_limit_info_.WARNING_power_buff = 50;
      power_limit_info_.buffer_total_current_limit = 180000 * OneWheel::motor_num;
      power_limit_info_.power_total_current_limit =
          257000 * OneWheel::motor_num / 80.0 * power_limit;

      float PID_output[OneWheel::motor_num];
      float output[OneWheel::motor_num];

      PID_output[OneWheel::center] = pids_[OneWheel::center].ComputeOutput(
          motors_[OneWheel::center]->GetOmegaDelta(speeds_[OneWheel::center]));

      power_limit_->Output(power_limit_on, power_limit_info_, chassis_power, chassis_power_buffer,
                           PID_output, output);

      motors_[OneWheel::center]->SetOutput(control::ClipMotorRange(output[OneWheel::center]));

      break;
    }
    default:
      RM_ASSERT_TRUE(false, "Not Supported Chassis Mode\r\n");
  }
}

} /* namespace control */
