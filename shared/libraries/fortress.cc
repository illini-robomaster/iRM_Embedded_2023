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

#include "fortress.h"

#define NOTCH (2 * PI / 4)
#define SPEED 200
#define ACCELERATION (80 * PI)

namespace control {

Fortress::Fortress(const fortress_t fortress) {
  leftSwitch_ = fortress.leftSwitch;
  rightSwitch_ = fortress.rightSwitch;
  leftElevatorMotor_ = fortress.leftElevatorMotor;
  rightElevatortMotor_ = fortress.rightElevatorMotor;

  control::servo_t servo_data;
  servo_data.max_speed = SPEED;
  servo_data.max_acceleration = ACCELERATION;
  servo_data.transmission_ratio = M3508P19_RATIO;
  servo_data.omega_pid_param = new float[3]{150, 1.2, 5};
  servo_data.max_iout = 1000;
  servo_data.max_out = 13000;

  servo_data.motor = leftElevatorMotor_;
  servo_left_ = new control::ServoMotor(servo_data);
  servo_data.motor = rightElevatortMotor_;
  servo_right_ = new control::ServoMotor(servo_data);

  left_edge_ = new BoolEdgeDetector(true);
  right_edge_ = new BoolEdgeDetector(true);

  fortressMotor_ = fortress.fortressMotor;

  MotorCANBase* motors_spinner[] = {fortressMotor_};
  chassis_t chassis_data;
  chassis_data.motors = motors_spinner;
  chassis_data.model = CHASSIS_ONE_WHEEL;
  spinner_ = new Chassis(chassis_data);
}

bool Fortress::Calibrate() {
  fortress_mode_ = false;

  left_edge_->input(leftSwitch_->Read());
  right_edge_->input(rightSwitch_->Read());

  if (!left_reach_ && left_edge_->negEdge()) {
    target_left_ = servo_left_->GetTheta();
    left_reach_ = true;
  } else if (!left_reach_)
    target_left_ -= 0.02;

  if (!right_reach_ && right_edge_->negEdge()) {
    target_right_ = servo_right_->GetTheta();
    right_reach_ = true;
  } else if (!right_reach_)
    target_right_ -= 0.02;

  servo_left_->SetTarget(target_left_, true);
  servo_right_->SetTarget(target_right_, true);
  servo_left_->CalcOutput();
  servo_right_->CalcOutput();

  return left_reach_ && right_reach_;
}

#define MAX_LEN 35.7

void Fortress::Transform(const bool fortress_mode) {
  if (fortress_mode != fortress_mode_) {
    fortress_mode_ = fortress_mode;

    target_left_ += fortress_mode ? MAX_LEN : -MAX_LEN;
    target_right_ += fortress_mode ? MAX_LEN : -MAX_LEN;

    servo_left_->SetTarget(target_left_, true);
    servo_right_->SetTarget(target_right_, true);
  }
  servo_left_->CalcOutput();
  servo_right_->CalcOutput();
}

void Fortress::Transform(const bool fortress_mode, const bool fortress_calibrating) {
  if (fortress_mode != fortress_mode_) {
    fortress_mode_ = fortress_mode;

    target_left_ += fortress_mode ? MAX_LEN : -MAX_LEN;
    target_right_ += fortress_mode ? MAX_LEN : -MAX_LEN;

    servo_left_->SetTarget(target_left_, true);
    servo_right_->SetTarget(target_right_, true);

  } else if (fortress_calibrating) {
    fortress_mode_ = false;

    target_left_ += fortress_mode ? MAX_LEN : -MAX_LEN;
    target_right_ += fortress_mode ? MAX_LEN : -MAX_LEN;

    servo_left_->SetTarget(target_left_, true);
    servo_right_->SetTarget(target_right_, true);
  }
  servo_left_->CalcOutput();
  servo_right_->CalcOutput();
}

void Fortress::Spin(bool power_limit_on, float power_limit, float chassis_power,
                    float chassis_power_buffer) {
  if (!fortress_mode_) return;
  spinner_->SetSpeed(-60);
  spinner_->Update(power_limit_on, power_limit, chassis_power, chassis_power_buffer);
}

bool Fortress::Error() {
  return abs(leftElevatorMotor_->GetCurr()) > 50000 || abs(rightElevatortMotor_->GetCurr()) > 50000;
}

void Fortress::Stop(const fortress_component_t component) {
  if (component == ELEVATOR) {
    leftElevatorMotor_->SetOutput(0);
    rightElevatortMotor_->SetOutput(0);
  } else if (component == SPINNER)
    fortressMotor_->SetOutput(0);
}

bool Fortress::Finished() {
  return abs(leftElevatorMotor_->GetOmega()) < 0.0001 &&
         abs(rightElevatortMotor_->GetOmega()) < 0.0001;
}

}  // namespace control
