/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2024 RoboMaster.                                          *
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

#include "shooterTask.h"

//==================================================================================================
// Shooter
//==================================================================================================
// Params Initialization

static const int KILLALL_DELAY = 100;

static control::Motor4310* load_motor = nullptr;

static control::MotorCANBase* shoot_front_motor = nullptr;
static control::MotorCANBase* shoot_back_motor = nullptr;

static control::MotorCANBase* force_motor = nullptr;
static control::ServoMotor* force_servo = nullptr;

static control::MotorCANBase* esca_motor = nullptr;
static control::ServoMotor* esca_servo = nullptr;


void shooterTask(void* arg) {
  UNUSED(arg);
  // shooter desired speed
  float shoot_speed = 30;
  // motor initialization
  control::MotorCANBase* can1_shooter_shoot[] = {shoot_front_motor, shoot_back_motor};
//  control::MotorCANBase* can1_shooter_force[] = {force_motor};
//  control::Motor4310* can1_shooter_load[] = {load_motor};

  // PID controller initialization
  float shoot_pid_params[3] = {20, 15, 10};
  control::ConstrainedPID shoot_pid(shoot_pid_params, 1000, 10000);
  float shoot_front_speed_diff = 0;
  float shoot_back_speed_diff = 0;
  int16_t shoot_front_out = 0;
  int16_t shoot_back_out = 0;
  while (true) {
    if (dbus->keyboard.bit.B || dbus->swr == remote::DOWN) break;
    osDelay(100);
  }
  print("shooterTask entering loop\r\n");
  while (true) {
    if (dbus->swr == remote::UP) {
    shoot_front_speed_diff = shoot_front_motor->GetOmegaDelta(shoot_speed);
    shoot_back_speed_diff = shoot_back_motor->GetOmegaDelta(shoot_speed);
    shoot_front_out = shoot_pid.ComputeConstrainedOutput(shoot_front_speed_diff);
    shoot_back_out = shoot_pid.ComputeConstrainedOutput(shoot_back_speed_diff);
    print("out %d %d\n", shoot_front_out, shoot_back_out);
    } else {
      shoot_front_speed_diff = 0;
      shoot_back_speed_diff = 0;
      shoot_front_out = 0;
      shoot_back_out = 0;
    }
    // TODO: needs a mode switch to control from moving mode to shooting mode / escalation mode
    // Bool Edge Detector for lob mode switch or osEventFlags wait for a signal from different threads
    if(lob_mode){
        // lob mode
        // force_motor->SetOutput(0);
        // force_servo->SetOutput(0);
        // esca_motor->SetOutput(0);
        // esca_servo->SetOutput(0);
        } else {
        // moving mode
        force_motor->SetOutput(0);
        force_servo->SetTarget(0);
        esca_motor->SetOutput(0);
        esca_servo->SetTarget(0);
    }
    shoot_front_motor->SetOutput(shoot_front_out);
    shoot_back_motor->SetOutput(shoot_back_out);
    control::MotorCANBase::TransmitOutput(can1_shooter_shoot, 2);
  }
}

void init_shooter() {
  //Shooter initialization
  load_motor = new control::Motor4310(can1, 0x02, 0x01, control::POS_VEL);
  shoot_front_motor = new control::Motor3508(can1, 0x201);
  shoot_back_motor = new control::Motor3508(can1, 0x202);
  force_motor = new control::Motor3508(can1, 0x203);

  // Servo control for each shooter motor
  control::servo_t servo_data;
  servo_data.motor = force_motor;
  servo_data.max_speed = 100 * PI; // TODO: params need test
  servo_data.max_acceleration = 100 * PI;
  servo_data.transmission_ratio = 1;
  servo_data.omega_pid_param = new float [3] {150, 1.2, 5};
  servo_data.max_iout = 1000;
  servo_data.max_out = 13000;
  force_servo = new control::ServoMotor(servo_data);

  // ESCALATION motors initialization
  esca_motor = new control::Motor3508(can1, 0x204);
  servo_data.motor = esca_motor;
  servo_data.max_speed = 100 * PI; // TODO: params need test
  servo_data.max_acceleration = 100 * PI;
  servo_data.transmission_ratio = M3508P19_RATIO;
  servo_data.omega_pid_param = new float [3] {150, 1.2, 5}; // TODO: PID params might need tuning
  servo_data.max_iout = 1000;
  servo_data.max_out = 13000;
  force_servo = new control::ServoMotor(servo_data);

}

void kill_shooter() {
  RM_EXPECT_TRUE(false, "Operation Killed!\r\n");
  control::MotorCANBase* shooter_motors[] = {shoot_front_motor, shoot_back_motor, force_motor, esca_motor};
  control::Motor4310* load_motors[] = {load_motor};
  while (true){
    shoot_front_motor->SetOutput(0);
    shoot_back_motor->SetOutput(0);
    force_motor->SetOutput(0);
    load_motor->SetOutput(0);
    esca_motor->SetOutput(0);
    control::MotorCANBase::TransmitOutput(shooter_motors, 4);
    control::Motor4310::TransmitOutput(load_motors, 1);
    osDelay(KILLALL_DELAY);
  }
}