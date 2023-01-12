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
    RM_ASSERT_TRUE(false, "Chassis pointer is null\r\n");
  }
  // Init Steering Motors from CANBaseMotor
  control::steering_t steering_data;

  steering_data.run_speed = SPEED;
  steering_data.test_speed = TEST_SPEED;
  steering_data.max_acceleration = ACCELERATION;
  steering_data.transmission_ratio = M3508P19_RATIO;
  steering_data.omega_pid_param = new float[3]{140, 1.2, 25};
  steering_data.max_iout = 3000;
  steering_data.max_out = 13000;


  steering_data.motor = _chassis->fl_steer_motor;
  steering_data.align_detect_func = _chassis->fl_steer_motor_detect_func;
  // steering_data.calibrate_offset = -0.858458848;
  steering_data.calibrate_offset = 0;
  fl_steer_motor = new control::SteeringMotor(steering_data);


  steering_data.motor = _chassis->fr_steer_motor;
  steering_data.align_detect_func = _chassis->fr_steer_motor_detect_func;
  // steering_data.calibrate_offset = 0.858458848;
  steering_data.calibrate_offset = 0;
  fr_steer_motor = new control::SteeringMotor(steering_data);


  steering_data.motor = _chassis->bl_steer_motor;
  steering_data.align_detect_func = _chassis->bl_steer_motor_detect_func;
  // steering_data.calibrate_offset = -2.283133461;
  steering_data.calibrate_offset = 0;
  bl_steer_motor = new control::SteeringMotor(steering_data);


  steering_data.motor = _chassis->br_steer_motor;
  steering_data.align_detect_func = _chassis->br_steer_motor_detect_func;
  // steering_data.calibrate_offset = 2.283133461;
  steering_data.calibrate_offset = 0;
  br_steer_motor = new control::SteeringMotor(steering_data);
  // Init Steering Motors complete

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

  vx = 0.0;
  vy = 0.0;
  vw = 0.0;

  theta0 = 0.0;
  theta1 = 0.0;
  theta2 = 0.0;
  theta3 = 0.0;

  float* pid_params = new float[3]{40, 3, 0};
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
  //    UNUSED(_power_limit);
  //    UNUSED(_chassis_power);
  //    UNUSED(_chassis_power_buffer);

  // Update Steer

  // compute angle for steer motors
  float effort = sqrt(pow(vx, 2) + pow(vy, 2) + pow(vw, 2));

  float theta0_diff;
  float theta1_diff;
  float theta2_diff;
  float theta3_diff;

  // only if effort > 0.1 update theta difference
  // otherwise, diff = 0.0
  if (effort > 0.1) {
    float theta0_new = -atan2(vy - vw * cos(PI / 4), vx - vw * sin(PI / 4));
    float theta1_new = -atan2(vy - vw * cos(PI / 4), vx + vw * sin(PI / 4));
    float theta2_new = -atan2(vy + vw * cos(PI / 4), vx - vw * sin(PI / 4));
    float theta3_new = -atan2(vy + vw * cos(PI / 4), vx + vw * sin(PI / 4));

    theta0_diff = wrap<float>(theta0_new - theta0, -PI, PI);
    theta1_diff = wrap<float>(theta1_new - theta1, -PI, PI);
    theta2_diff = wrap<float>(theta2_new - theta2, -PI, PI);
    theta3_diff = wrap<float>(theta3_new - theta3, -PI, PI);

    theta0 = wrap<float>(theta0 + theta0_diff, -PI, PI);
    theta1 = wrap<float>(theta1 + theta1_diff, -PI, PI);
    theta2 = wrap<float>(theta2 + theta2_diff, -PI, PI);
    theta3 = wrap<float>(theta3 + theta3_diff, -PI, PI);
  } else {
    theta0_diff = 0;
    theta1_diff = 0;
    theta2_diff = 0;
    theta3_diff = 0;
  }

  fl_steer_motor->TurnRelative(theta0_diff);
  fr_steer_motor->TurnRelative(theta1_diff);
  bl_steer_motor->TurnRelative(theta2_diff);
  br_steer_motor->TurnRelative(theta3_diff);

  fl_steer_motor->CalcOutput();
  fr_steer_motor->CalcOutput();
  bl_steer_motor->CalcOutput();
  br_steer_motor->CalcOutput();

  // compute speed for wheel motors
  float v0 = sqrt(pow(vy + vw * cos(PI / 4), 2.0) + pow(vx - vw * sin(PI / 4), 2.0));
  float v1 = sqrt(pow(vy + vw * cos(PI / 4), 2.0) + pow(vx + vw * sin(PI / 4), 2.0));
  float v2 = sqrt(pow(vy - vw * cos(PI / 4), 2.0) + pow(vx - vw * sin(PI / 4), 2.0));
  float v3 = sqrt(pow(vy - vw * cos(PI / 4), 2.0) + pow(vx + vw * sin(PI / 4), 2.0));

  // 16 is a arbitrary factor
  v0 = v0 * WHEEL_SPEED_FACTOR;
  v1 = v1 * WHEEL_SPEED_FACTOR;
  v2 = v2 * WHEEL_SPEED_FACTOR;
  v3 = v3 * WHEEL_SPEED_FACTOR;

  // Update Wheels
  float PID_output[MOTOR_NUM];

  // compute PID output
  PID_output[0] = pids[0].ComputeOutput(fl_wheel_motor->GetOmegaDelta(v0));
  PID_output[1] = pids[1].ComputeOutput(fr_wheel_motor->GetOmegaDelta(v1));
  PID_output[2] = pids[2].ComputeOutput(bl_wheel_motor->GetOmegaDelta(v2));
  PID_output[3] = pids[3].ComputeOutput(br_wheel_motor->GetOmegaDelta(v3));

  float output[MOTOR_NUM];
  // compute power limit
  power_limit_info.power_limit = _power_limit;
  power_limit_info.WARNING_power = _power_limit * 0.9;
  power_limit_info.WARNING_power_buff = 50;
  power_limit_info.buffer_total_current_limit = 3500 * MOTOR_NUM;
  power_limit_info.power_total_current_limit = 5000 * MOTOR_NUM / 80.0 * _power_limit;
  power_limit->Output(true, power_limit_info, _chassis_power, _chassis_power_buffer, PID_output,
                      output);

  // set final output
  //    fl_wheel_motor->SetOutput(control::ClipMotorRange(output[0]));
  //    fr_wheel_motor->SetOutput(control::ClipMotorRange(output[1]));
  //    bl_wheel_motor->SetOutput(control::ClipMotorRange(output[2]));
  //    br_wheel_motor->SetOutput(control::ClipMotorRange(output[3]));
  fl_wheel_motor->SetOutput(output[0]);
  fr_wheel_motor->SetOutput(output[1]);
  bl_wheel_motor->SetOutput(output[2]);
  br_wheel_motor->SetOutput(output[3]);
}

bool SteeringChassis::AlignUpdate() {
  volatile bool fl_complete = fl_steer_motor->AlignUpdate();
  volatile bool fr_complete = fr_steer_motor->AlignUpdate();
  volatile bool bl_complete = bl_steer_motor->AlignUpdate();
  volatile bool br_complete = br_steer_motor->AlignUpdate();
  return fl_complete && fr_complete && bl_complete && br_complete;
}

void SteeringChassis::PrintData() {
  fl_steer_motor->PrintData();
  fr_steer_motor->PrintData();
  bl_steer_motor->PrintData();
  br_steer_motor->PrintData();
}

}  // namespace control
