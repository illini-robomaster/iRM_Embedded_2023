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
#include <utils.h>
#include <controller.h>

//==================================================================================================
// Shooter
//==================================================================================================
// Params Initialization

static const int KILLALL_DELAY = 100;
static const int SHOOTER_TASK_DELAY = 1;

// static control::Motor4310* load_motor = nullptr;

static control::MotorCANBase* shoot_front_motor = nullptr;
static control::MotorCANBase* shoot_back_motor = nullptr;

static control::MotorCANBase* force_motor = nullptr;
static control::ServoMotor* force_servo = nullptr;

static BoolEdgeDetector key_sw = BoolEdgeDetector(false);

static BoolEdgeDetector slow_trigger(false);
static BoolEdgeDetector cool_trigger(false);

static control::MotorCANBase* rfid_motor = nullptr;
static control::ConstrainedPID* rfid_motor_omega_pid = nullptr;

static BoolEdgeDetector rfid_upper(false);
static BoolEdgeDetector rfid_lower(false);

static control::MotorCANBase* esca_motor = nullptr;
static control::ServoMotor* escalation_servo = nullptr;




int s_f_out;
int s_b_out;
int esca_out;

bool lob_mode = false;




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
  control::MotorCANBase* can1_escalation[] = {esca_motor};  // TODO: change
  control::MotorCANBase* can1_shooter_shoot[] = {shoot_front_motor, shoot_back_motor};
  control::MotorCANBase* can1_rfid[] = {rfid_motor};

  UNUSED(can1_shooter_shoot);

//  control::MotorCANBase* can1_shooter_force[] = {force_motor};

  // PID controller initialization
  float shoot_pid_param[3] = {150, 1.2, 0};
  control::ConstrainedPID shoot_pid(shoot_pid_param, 10000, 15000);
  float shoot_back_speed_diff = 0;

  float rfid_motor_target1 = 5.55;
  float rfid_motor_target2 = 0.75;
  float rfid_motor_vel = 1.0;

  float calibrated_theta_esca = escalation_servo->GetTheta();



  while (true) {
    if (/*receive->keyboard.bit.B || */with_gimbal->start) break;
    print("Waiting for start signal\r\n");

    osDelay(100);
  }

  bool cw = true;

  shoot_front_motor->SetOutput(0);
  shoot_back_motor->SetOutput(0);
  force_motor->SetOutput(0);
  control::MotorCANBase::TransmitOutput(can1_shooter_shoot, 2);




  print("Shooter begin\r\n");

  while (true) {
    print("Shooter running\r\n");

    // TODO: heat control
    float omega_error = 0.0;
    float omega_pid_out = 0.0;
    if (with_gimbal->bus_swl == remote::UP /*|| receive->mouse_bit.l && with_chassis->cooling_heat1 <= with_chassis->cooling_limit1-100 */) {
      // TODO: Heat control need to be checked
      print("shooting\r\n");
      shoot_back_speed_diff = shoot_back_motor->GetOmegaDelta(shoot_speeds[level] - (level+1) * 10);
      s_b_out = shoot_pid.ComputeConstrainedOutput(shoot_back_speed_diff);
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
    } /* else if (dbus->swr==remote::DOWN) {
      s_f_out = 0;
      s_b_out = 0;
      shoot_front_motor->SetOutput(s_f_out);
      shoot_back_motor->SetOutput(s_b_out);
      control::MotorCANBase::TransmitOutput(can1_shooter_shoot, 2);
    } */
    else {
//      print("stopping\r\n");
      osDelay(SHOOTER_TASK_DELAY);
      shoot_back_speed_diff = shoot_back_motor->GetOmegaDelta(0);
      s_b_out = shoot_pid.ComputeConstrainedOutput(shoot_back_speed_diff);
      s_b_out = shoot_pid.ComputeConstrainedOutput(shoot_back_speed_diff);
      s_f_out = s_b_out * 0.98;
      shoot_front_motor->SetOutput(s_f_out);
      shoot_back_motor->SetOutput(s_b_out);
    }
    key_sw.input(with_gimbal->bus_swl == remote::DOWN);
    // we use can bus to change the input
    if(key_sw.posEdge()){
      print("level adjusted\r\n");
      level = (level + 1) % 4;
      for (int i = level; i >=0; i--){
        buzzer->SingTone(levels[level].note);
        [](uint32_t milli) { osDelay(milli); }(levels[level].delay);
        buzzer->SingTone(Note::Silent);
        [](uint32_t milli) { osDelay(milli); }(levels[level].delay);
      }

    }

    force_servo->CalcOutput();



    if(with_gimbal->lob_mode == remote::UP){
      // please make sure the calibration is all done
      // lob mode
      escalation_servo->SetTarget(calibrated_theta_esca + PI * 12.1);
      // maximum goes to 38.345 radians
      print("lob mode: %d\r\n",with_gimbal->lob_mode);
    } else {
      // moving mode
      // set escalation servo to original position
      escalation_servo->SetTarget(calibrated_theta_esca);
      print("regular\r\n");
    }

    escalation_servo->CalcOutput(&esca_out);
    control::MotorCANBase::TransmitOutput(can1_shooter_shoot, 2);

    /* rfid motor */
    rfid_upper.input(rfid_motor->GetTheta() > rfid_motor_target2);
    rfid_lower.input(rfid_motor->GetTheta() < rfid_motor_target1);

    if (rfid_upper.posEdge()) cw = false;
    if (rfid_lower.posEdge()) cw = true;

    if (cw) {
      omega_error = rfid_motor->GetOmegaDelta(rfid_motor_vel);
      omega_pid_out = rfid_motor_omega_pid->ComputeConstrainedOutput(omega_error);
      rfid_motor->SetOutput(omega_pid_out);
    } else {
      omega_error = rfid_motor->GetOmegaDelta(-rfid_motor_vel);
      omega_pid_out = rfid_motor_omega_pid->ComputeConstrainedOutput(omega_error);
      rfid_motor->SetOutput(omega_pid_out);
    }
    control::MotorCANBase::TransmitOutput(can1_rfid, 1);
    control::MotorCANBase::TransmitOutput(can1_escalation, 1);
//    print("cooling_heat1: %f cooling_limit1: %f gimbal_power: %d shooter_power: %d\r\n", with_chassis->cooling_heat1, with_chassis->cooling_limit1, with_chassis->gimbal_power, with_chassis->shooter_power);

    osDelay(SHOOTER_TASK_DELAY);
  }
}

void init_shooter() {
  //Shooter initialization
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
  rfid_motor = new control::Motor6020(can1, 0x209);

  float rfid_motor_pid_param_omega[] = {550.0, 10.0, 0.0};
  float rfid_motor_max_iout_omega = 25000;
  float rfid_motor_max_out_omega = 30000;
  rfid_motor_omega_pid = new control::ConstrainedPID(rfid_motor_pid_param_omega, rfid_motor_max_iout_omega, rfid_motor_max_out_omega);

  // ESCALATION motors initialization
  esca_motor = new control::Motor3508(can1, 0x204);
  control::servo_t esca_servo_data;
  esca_servo_data.motor = esca_motor;
  esca_servo_data.max_speed = 2 * PI; // TODO: params need test
  esca_servo_data.max_acceleration = 100 * PI;
  esca_servo_data.transmission_ratio = M3508P19_RATIO;
  esca_servo_data.omega_pid_param = new float [3] {150, 1.2, 5};
  //  esca_servo_data.omega_pid_param = new float [3] {150, 1.2, 5};
  esca_servo_data.max_iout = 2000;
  esca_servo_data.max_out = 10000;
  escalation_servo = new control::ServoMotor(esca_servo_data);


}

void kill_shooter() {
  RM_EXPECT_TRUE(false, "Operation Killed!\r\n");
  control::MotorCANBase* shooter_motors[] = {shoot_front_motor, shoot_back_motor, force_motor};
  while (true){
    shoot_front_motor->SetOutput(0);
    shoot_back_motor->SetOutput(0);
    force_motor->SetOutput(0);
    control::MotorCANBase::TransmitOutput(shooter_motors, 3);
    osDelay(KILLALL_DELAY);
  }
}