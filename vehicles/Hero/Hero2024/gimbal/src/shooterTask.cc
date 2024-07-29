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
static const int SHOOTER_TASK_DELAY = 1;

static control::Motor4310* load_motor = nullptr;

static control::MotorCANBase* shoot_front_motor = nullptr;
static control::MotorCANBase* shoot_back_motor = nullptr;

static control::MotorCANBase* force_motor = nullptr;
static control::ServoMotor* force_servo = nullptr;

static BoolEdgeDetector key_sw = BoolEdgeDetector(false);

static BoolEdgeDetector slow_trigger(false);
static BoolEdgeDetector cool_trigger(false);

int s_f_out;
int s_b_out;


//static int FORCE_VALUE = 0;
using Note = bsp::BuzzerNote;
static bsp::BuzzerNoteDelayed levels[] ={{Note::Do1M,80},{Note::Mi3M,80},
                                         {Note::La6M,80},{Note::Do1H,80}};




static float shoot_speeds[] = {-600, -650,-700, -800};
int level = 0;
void shooter_task(void* args) {
  UNUSED(args);
  // shooter desired speed
  // motor initialization
  control::MotorCANBase* can1_shooter_shoot[] = {shoot_front_motor, shoot_back_motor};
//  control::MotorCANBase* can1_shooter_force[] = {force_motor};
//  control::Motor4310* can1_shooter_load[] = {load_motor};

  // PID controller initialization
  float shoot_back_pid_params[3] = {450, 4.6,20};
  control::ConstrainedPID shoot_back_pid(shoot_back_pid_params, 10000,15000);
  float shoot_back_speed_diff = 0;

  while (true) {
    if (dbus->keyboard.bit.B || dbus->swr == remote::DOWN) break;
    osDelay(100);
  }

  load_motor->SetZeroPos();
  load_motor->MotorEnable();


  while (true) {

    cool_trigger.input(send->cooling_heat1 >= send->cooling_limit1);

    if ((dbus->swr == remote::UP || dbus->mouse.l)) {
      // TODO: Heat control need to be added in the if statement above
      shoot_back_speed_diff = shoot_back_motor->GetOmegaDelta(shoot_speeds[level] - (level+1) * 10);
      s_b_out = shoot_back_pid.ComputeConstrainedOutput(shoot_back_speed_diff);
      s_f_out = s_b_out * 0.98;
      shoot_front_motor->SetOutput(s_f_out);
      shoot_back_motor->SetOutput(s_b_out);
      slow_trigger.input(true);
    } else if (slow_trigger.posEdge()){
      print("slow down triggered\r\n");
      s_f_out = 0;
      s_b_out = 0;
      shoot_front_motor->SetOutput(s_f_out);
      shoot_back_motor->SetOutput(s_b_out);
      control::MotorCANBase::TransmitOutput(can1_shooter_shoot, 2);
      osDelay(1000);
    } else if (dbus->swr==remote::DOWN) {
      s_f_out = 0;
      s_b_out = 0;
      shoot_front_motor->SetOutput(s_f_out);
      shoot_back_motor->SetOutput(s_b_out);
      control::MotorCANBase::TransmitOutput(can1_shooter_shoot, 2);
    }
    else {
      osDelay(SHOOTER_TASK_DELAY);
      shoot_back_speed_diff = shoot_back_motor->GetOmegaDelta(0);
      s_b_out = shoot_back_pid.ComputeConstrainedOutput(shoot_back_speed_diff);
      s_b_out = shoot_back_pid.ComputeConstrainedOutput(shoot_back_speed_diff);
      s_f_out = s_b_out * 0.98;
      shoot_front_motor->SetOutput(s_f_out);
      shoot_back_motor->SetOutput(s_b_out);
    }
    key_sw.input(dbus->swl == remote::DOWN);
    if(key_sw.posEdge()){
      level = (level + 1) % 4;
      for (int i = level; i >=0; i--){
        buzzer->SingTone(levels[level].note);
        [](uint32_t milli) { osDelay(milli); }(levels[level].delay);
        buzzer->SingTone(Note::Silent);
        [](uint32_t milli) { osDelay(milli); }(levels[level].delay);
      }
    }

    force_servo->CalcOutput();
//    control::MotorCANBase::TransmitOutput(can1_shooter_force, 1);
    control::MotorCANBase::TransmitOutput(can1_shooter_shoot, 2);

    osDelay(SHOOTER_TASK_DELAY);
  }
}

void init_shooter() {
  //Shooter initialization
  load_motor = new control::Motor4310(can1, 0x02, 0x03, control::POS_VEL);
  shoot_front_motor = new control::Motor3508(can1, 0x201);
  shoot_back_motor = new control::Motor3508(can1, 0x202);
  force_motor = new control::Motor3508(can1, 0x203);
  // Servo control for each shooter motor
  control::servo_t servo_data;
  servo_data.motor = force_motor;
  servo_data.max_speed = 100 * PI; // TODO: params need test
  servo_data.max_acceleration = 100 * PI;
  servo_data.omega_pid_param = new float [3] {150, 1.2, 5};
  servo_data.max_iout = 1000;
  servo_data.max_out = 13000;
  force_servo = new control::ServoMotor(servo_data);
}

void kill_shooter() {
  RM_EXPECT_TRUE(false, "Operation Killed!\r\n");
  control::MotorCANBase* shooter_motors[] = {shoot_front_motor, shoot_back_motor, force_motor};
  control::Motor4310* load_motors[] = {load_motor};
  while (true){
    shoot_front_motor->SetOutput(0);
    shoot_back_motor->SetOutput(0);
    force_motor->SetOutput(0);
    load_motor->SetOutput(0);
    load_motor->MotorDisable();
    control::MotorCANBase::TransmitOutput(shooter_motors, 3);
    control::Motor4310::TransmitOutput(load_motors, 1);
    osDelay(KILLALL_DELAY);
  }
}