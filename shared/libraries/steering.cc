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

#include "steering.h"

#include <cmath>

#include "controller.h"
#include "motor.h"

namespace control {
SteeringChassis::SteeringChassis(steering_chassis_t* _chassis) {
  if (_chassis == nullptr) {
    RM_ASSERT_TRUE(false, "Chassis pointer is null\r\n");
  }

  if (_chassis->fl_steer_motor == nullptr || _chassis->fr_steer_motor == nullptr ||
      _chassis->bl_steer_motor == nullptr || _chassis->br_steer_motor == nullptr ||
      _chassis->fl_wheel_motor == nullptr || _chassis->fr_wheel_motor == nullptr ||
      _chassis->bl_wheel_motor == nullptr || _chassis->br_wheel_motor == nullptr) {
    RM_ASSERT_TRUE(false, "At least one motor pointer is null\r\n");
  }

  // Init Steer Motors
  fl_steer_motor = _chassis->fl_steer_motor;
  fr_steer_motor = _chassis->fr_steer_motor;
  bl_steer_motor = _chassis->bl_steer_motor;
  br_steer_motor = _chassis->br_steer_motor;

  // Init Wheel Motors
  fl_wheel_motor = _chassis->fl_wheel_motor;
  fr_wheel_motor = _chassis->fr_wheel_motor;
  bl_wheel_motor = _chassis->bl_wheel_motor;
  br_wheel_motor = _chassis->br_wheel_motor;

  // safety lock
  fl_steer_motor->TurnRelative(0.0);
  fr_steer_motor->TurnRelative(0.0);
  bl_steer_motor->TurnRelative(0.0);
  br_steer_motor->TurnRelative(0.0);

  fl_steer_motor->CalcOutput();
  fr_steer_motor->CalcOutput();
  bl_steer_motor->CalcOutput();
  br_steer_motor->CalcOutput();

  fl_wheel_motor->SetOutput(0.0);
  fr_wheel_motor->SetOutput(0.0);
  bl_wheel_motor->SetOutput(0.0);
  br_wheel_motor->SetOutput(0.0);
  // safety lock ends

  // Init private variables starts
  vx = 0.0;
  vy = 0.0;
  vw = 0.0;

  v_fl_ = 0.0;
  v_fr_ = 0.0;
  v_bl_ = 0.0;
  v_br_ = 0.0;

  SteerThetaReset();
  // Init private variables ends

  // just for wheel speed pid (not for steer motor)
  float* pid_params = new float[3]{120, 3, 1};
  float motor_max_iout = 2000;
  float motor_max_out = 20000;
  for (int i = 0; i < MOTOR_NUM; i++) {
    pids[i].Reinit(pid_params, motor_max_iout, motor_max_out);
  }

  power_limit = new PowerLimit(MOTOR_NUM);
}

SteeringChassis::~SteeringChassis() {
  fl_steer_motor = nullptr;
  fr_steer_motor = nullptr;
  bl_steer_motor = nullptr;
  br_steer_motor = nullptr;
  fl_wheel_motor = nullptr;
  fr_wheel_motor = nullptr;
  bl_wheel_motor = nullptr;
  br_wheel_motor = nullptr;
}

void SteeringChassis::SetXSpeed(float _vx) { vx = _vx; }

void SteeringChassis::SetYSpeed(float _vy) { vy = _vy; }

void SteeringChassis::SetWSpeed(float _vw) { vw = _vw; }

void SteeringChassis::Update(float _power_limit, float _chassis_power,
                             float _chassis_power_buffer) {

  // Update Wheels
  float PID_output[MOTOR_NUM];
  float output[MOTOR_NUM];

  // compute PID output
  PID_output[0] = pids[0].ComputeOutput(fl_wheel_motor->GetOmegaDelta(v_fl_));
  PID_output[1] = pids[1].ComputeOutput(fr_wheel_motor->GetOmegaDelta(v_fr_));
  PID_output[2] = pids[2].ComputeOutput(bl_wheel_motor->GetOmegaDelta(v_bl_));
  PID_output[3] = pids[3].ComputeOutput(br_wheel_motor->GetOmegaDelta(v_br_));

  // compute power limit
  power_limit_info.power_limit = _power_limit;
  power_limit_info.WARNING_power = _power_limit * 0.9;
  power_limit_info.WARNING_power_buff = 50;
  power_limit_info.buffer_total_current_limit = 3500 * MOTOR_NUM;
  power_limit_info.power_total_current_limit = 5000 * MOTOR_NUM / 80.0 * _power_limit;
  power_limit->Output(true, power_limit_info, _chassis_power, _chassis_power_buffer, PID_output,
                      output);

  // set final output
  fl_wheel_motor->SetOutput(output[0]);
  fr_wheel_motor->SetOutput(output[1]);
  bl_wheel_motor->SetOutput(output[2]);
  br_wheel_motor->SetOutput(output[3]);
}

void SteeringChassis::SetSpeed(const float _vx, const float _vy, const float _vw) {
  vx = _vx;
  vy = _vy;
  vw = _vw;
}

void SteeringChassis::SteerThetaReset() {
  theta_fl_ = 0.0;
  theta_fr_ = 0.0;
  theta_bl_ = 0.0;
  theta_br_ = 0.0;

  ret_fl_ = 0;
  ret_fr_ = 0;
  ret_bl_ = 0;
  ret_br_ = 0;

  wheel_dir_fl_ = 1.0;
  wheel_dir_fr_ = 1.0;
  wheel_dir_bl_ = 1.0;
  wheel_dir_br_ = 1.0;
}

void SteeringChassis::SetWheelSpeed(float _v_fl, float _v_fr, float _v_bl, float _v_br) {
  v_fl_ = _v_fl;
  v_fr_ = _v_fr;
  v_bl_ = _v_bl;
  v_br_ = _v_br;
}

bool SteeringChassis::Calibrate() {
  bool alignment_complete_fl = fl_steer_motor->Calibrate();
  bool alignment_complete_fr = fr_steer_motor->Calibrate();
  bool alignment_complete_bl = bl_steer_motor->Calibrate();
  bool alignment_complete_br = br_steer_motor->Calibrate();

  return alignment_complete_fl && alignment_complete_fr &&
  alignment_complete_bl && alignment_complete_br;
}

int SteeringChassis::ReAlign() {
  int ret = 0;
  ret += fl_steer_motor->ReAlign();
  ret += fr_steer_motor->ReAlign();
  ret += bl_steer_motor->ReAlign();
  ret += br_steer_motor->ReAlign();

  return ret;
}

void SteeringChassis::SteerCalcOutput() {
  fl_steer_motor->CalcOutput();
  fr_steer_motor->CalcOutput();
  bl_steer_motor->CalcOutput();
  br_steer_motor->CalcOutput();
}

void SteeringChassis::SteerUpdateTarget() {
  // Stay at current position when no command is given
  if (vx == 0 && vy == 0 && vw == 0) {
    ret_fl_ = fl_steer_motor->TurnRelative(0);
    ret_fr_ = fr_steer_motor->TurnRelative(0);
    ret_bl_ = bl_steer_motor->TurnRelative(0);
    ret_br_ = br_steer_motor->TurnRelative(0);

  } else {
    // Compute 2 position proposals, theta and theta + PI.
    double _theta_fl = atan2(vy + vw * cos(PI / 4), vx - vw * sin(PI / 4));
    double _theta_fr = atan2(vy + vw * cos(PI / 4), vx + vw * sin(PI / 4));
    double _theta_bl = atan2(vy - vw * cos(PI / 4), vx - vw * sin(PI / 4));
    double _theta_br = atan2(vy - vw * cos(PI / 4), vx + vw * sin(PI / 4));

    double _theta_fl_alt = wrap<double>(_theta_fl + PI, -PI, PI);
    double _theta_fr_alt = wrap<double>(_theta_fr + PI, -PI, PI);
    double _theta_bl_alt = wrap<double>(_theta_bl + PI, -PI, PI);
    double _theta_br_alt = wrap<double>(_theta_br + PI, -PI, PI);

    wheel_dir_fl_ = 1.0;
    wheel_dir_fr_ = 1.0;
    wheel_dir_bl_ = 1.0;
    wheel_dir_br_ = 1.0;

    // Go to the closer proposal and change wheel direction accordingly
    if (abs(wrap<double>(_theta_bl - theta_bl_, -PI, PI)) <
        abs(wrap<double>(_theta_bl_alt - theta_bl_, -PI, PI))) {
      wheel_dir_bl_ = 1.0;
      ret_bl_ = bl_steer_motor->TurnRelative(wrap<double>(_theta_bl - theta_bl_, -PI, PI));
    } else {
      wheel_dir_bl_ = -1.0;
      ret_bl_ = bl_steer_motor->TurnRelative(wrap<double>(_theta_bl_alt - theta_bl_, -PI, PI));
    }
    if (abs(wrap<double>(_theta_br - theta_br_, -PI, PI)) <
        abs(wrap<double>(_theta_br_alt - theta_br_, -PI, PI))) {
      wheel_dir_br_ = 1.0;
      ret_br_ = br_steer_motor->TurnRelative(wrap<double>(_theta_br - theta_br_, -PI, PI));
    } else {
      wheel_dir_br_ = -1.0;
      ret_br_ = br_steer_motor->TurnRelative(wrap<double>(_theta_br_alt - theta_br_, -PI, PI));
    }
    if (abs(wrap<double>(_theta_fr - theta_fr_, -PI, PI)) <
        abs(wrap<double>(_theta_fr_alt - theta_fr_, -PI, PI))) {
      wheel_dir_fr_ = 1.0;
      ret_fr_ = fr_steer_motor->TurnRelative(wrap<double>(_theta_fr - theta_fr_, -PI, PI));
    } else {
      wheel_dir_fr_ = -1.0;
      ret_fr_ = fr_steer_motor->TurnRelative(wrap<double>(_theta_fr_alt - theta_fr_, -PI, PI));
    }
    if (abs(wrap<double>(_theta_fl - theta_fl_, -PI, PI)) <
        abs(wrap<double>(_theta_fl_alt - theta_fl_, -PI, PI))) {
      wheel_dir_fl_ = 1.0;
      ret_fl_ = fl_steer_motor->TurnRelative(wrap<double>(_theta_fl - theta_fl_, -PI, PI));
    } else {
      wheel_dir_fl_ = -1.0;
      ret_fl_ = fl_steer_motor->TurnRelative(wrap<double>(_theta_fl_alt - theta_fl_, -PI, PI));
    }

    // Update theta when TurnRelative complete
    if (ret_bl_ == 0) {
      theta_bl_ = wheel_dir_bl_ == 1.0 ? _theta_bl : _theta_bl_alt;
    }
    if (ret_br_ == 0) {
      theta_br_ = wheel_dir_br_ == 1.0 ? _theta_br : _theta_br_alt;
    }
    if (ret_fr_ == 0) {
      theta_fr_ = wheel_dir_fr_ == 1.0 ? _theta_fr : _theta_fr_alt;
    }
    if (ret_fl_ == 0) {
      theta_fl_ = wheel_dir_fl_ == 1.0 ? _theta_fl : _theta_fl_alt;
    }
  }
}

void SteeringChassis::WheelUpdateSpeed(float wheel_speed_factor) {
  // Stay at current position when no command is given
  if (vx == 0 && vy == 0 && vw == 0) {
    SetWheelSpeed(0,0,0,0);
  } else if (ret_bl_ == 0 && ret_br_ == 0 && ret_fr_ == 0 && ret_fl_ == 0) {
    // Wheels move only when all SteeringMotors are in position
    v_fl_ = wheel_dir_fl_ * sqrt(pow(vy + vw * cos(PI / 4), 2.0) + pow(vx - vw * sin(PI / 4), 2.0));
    v_fr_ = wheel_dir_fr_ * sqrt(pow(vy + vw * cos(PI / 4), 2.0) + pow(vx + vw * sin(PI / 4), 2.0));
    v_bl_ = wheel_dir_bl_ * sqrt(pow(vy - vw * cos(PI / 4), 2.0) + pow(vx - vw * sin(PI / 4), 2.0));
    v_br_ = wheel_dir_br_ * sqrt(pow(vy - vw * cos(PI / 4), 2.0) + pow(vx + vw * sin(PI / 4), 2.0));

    v_fl_ = v_fl_ * wheel_speed_factor;
    v_fr_ = v_fr_ * wheel_speed_factor;
    v_bl_ = v_bl_ * wheel_speed_factor;
    v_br_ = v_br_ * wheel_speed_factor;
  }
}

void SteeringChassis::SteerSetMaxSpeed(const float max_speed) {
  fl_steer_motor->SetMaxSpeed(max_speed);
  fr_steer_motor->SetMaxSpeed(max_speed);
  bl_steer_motor->SetMaxSpeed(max_speed);
  br_steer_motor->SetMaxSpeed(max_speed);
}

void SteeringChassis::PrintData() {
  clear_screen();
  set_cursor(0, 0);

  print("fl_steer_motor: \r\n");
  fl_steer_motor->PrintData();
  print("fr_steer_motor: \r\n");
  fr_steer_motor->PrintData();
  print("bl_steer_motor: \r\n");
  bl_steer_motor->PrintData();
  print("br_steer_motor: \r\n");
  br_steer_motor->PrintData();
}

void SteeringChassis::SteerAlignFalse() {
  fl_steer_motor->SetAlignFalse();
  fr_steer_motor->SetAlignFalse();
  bl_steer_motor->SetAlignFalse();
  br_steer_motor->SetAlignFalse();
}

bool SteeringChassis::SteerAlignCheck() {
  return (fl_steer_motor->CheckAlignment() && fr_steer_motor->CheckAlignment()
          && bl_steer_motor->CheckAlignment() && br_steer_motor->CheckAlignment());
}

}  // namespace control
