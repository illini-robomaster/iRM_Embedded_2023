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

#include "gimbalTask.h"

//==================================================================================================
// Gimbal
//==================================================================================================
// Params Initialization

float pitch_cmd;
float yaw_cmd;

static const int KILLALL_DELAY = 100;
static const int GIMBAL_TASK_DELAY = 1;

static control::MotorCANBase* esca_motor = nullptr;
static control::ServoMotor* escalation_servo = nullptr;
static control::MotorCANBase *esca_yaw_motor = nullptr;

static control::MotorCANBase* pitch_motor_L = nullptr;
static control::MotorCANBase* pitch_motor_R = nullptr;

static control::ServoMotor* pitch_servo_L = nullptr;
static control::ServoMotor* pitch_servo_R = nullptr;

static control::Motor4310* yaw_motor = nullptr;

static control::BRTEncoder *pitch_encoder = nullptr;
static distance::SEN_0366_DIST *distance_sensor = nullptr;



bool exit_flag = false;

float calibrated_theta_esca = 0;



float pitch_curr;
float pitch_target;

float curr_yaw;

void jam_callback(control::ServoMotor* servo, const control::servo_jam_t data) {
  UNUSED(data);
  control::MotorCANBase* can1_escalation[] = {esca_motor};
  esca_motor->SetOutput(0);
  control::MotorCANBase::TransmitOutput(can1_escalation, 1);
  osDelay(100);
  servo->ResetTheta();
//  print("Jam detected, resetting...\r\n");
  exit_flag = true;
}
void empty_callback(control::ServoMotor* servo, const control::servo_jam_t data) {
  UNUSED(servo);
  UNUSED(data);
}

void gimbal_task(void* args) {
  UNUSED(args);
  control::MotorCANBase* can1_escalation[] = {esca_motor};
  control::MotorCANBase* can1_pitch[] = {pitch_motor_L, pitch_motor_R};
  control::Motor4310* can1_yaw[] = {yaw_motor};

  while (!distance_sensor->begin()){
//    print("distance sensor initializing\r\n");
    osDelay(100);
  }

  while (true) {
    if (dbus->keyboard.bit.B || dbus->swr == remote::DOWN) break;
    osDelay(100);
  }

  distance_sensor->continuousMeasure();
  yaw_motor->SetZeroPos();
  yaw_motor->MotorEnable();

  /*
   * escalation reset
   */
  while (true){
    escalation_servo->SetTarget(escalation_servo->GetTheta() - PI * 2,true);
    escalation_servo->CalcOutput();
    control::MotorCANBase::TransmitOutput(can1_escalation, 1);
    osDelay(GIMBAL_TASK_DELAY);
    if (exit_flag){
//      print("Escalation motor jammed, resting...\r\n");
      osDelay(50);
      calibrated_theta_esca = escalation_servo->GetTheta();
      escalation_servo->SetTarget(calibrated_theta_esca, true);
      // update jam callback threshold
      escalation_servo->RegisterJamCallback(empty_callback, 0.205);
      osDelay(50);
      break;
    }
  }



//  print("gimbal_task entering loop\r\n");
//  print("calibrated_theta: %f\r\n", calibrated_theta);


  /*
   * pitch motors reset
   */
  exit_flag = false;
  while (true){
    pitch_servo_L->SetTarget(pitch_servo_L->GetTheta() - PI * 2,true);
    pitch_servo_L->CalcOutput();
    pitch_servo_R->SetTarget(pitch_servo_R->GetTheta() - PI * 2,true);
    pitch_servo_R->CalcOutput();
    control::MotorCANBase::TransmitOutput(can1_pitch, 2);
    osDelay(GIMBAL_TASK_DELAY);
    if (exit_flag){
      osDelay(50);
      pitch_servo_L->SetTarget(pitch_servo_L->GetTheta(),true);
      pitch_servo_R->SetTarget(pitch_servo_R->GetTheta(),true);
      pitch_servo_L->CalcOutput();
      pitch_servo_R->CalcOutput();
      control::MotorCANBase::TransmitOutput(can1_pitch,2);
      osDelay(50);
      break;
    }
  }



  pitch_target = 0.0;

  pitch_curr = -1.0; // make sure it is initialized.

  pitch_cmd = 0.0;

  yaw_cmd = 0.0;

  while (true) {
    // Bool Edge Detector for lob mode switch or osEventFlags wait for a signal from different threads
    lob_mode_sw.input(dbus->keyboard.bit.SHIFT || dbus->swl == remote::UP);
    if (lob_mode_sw.posEdge() && dbus->swr == remote::MID){
      lob_mode = !lob_mode;
      // TODO: after implementing chassis, uncomment the lob_mode bsp::CanBridge flag transmission
//      send->cmd.id = bsp::LOB_MODE;
//      send->cmd.data_bool = lob_mode;
//      send->TransmitOutput();
      escalation_servo->SetMaxSpeed(4 * PI);
//      print("lob_mode: %d\n", lob_mode);
    }
    if(lob_mode){
      // please make sure the calibration is all done
      // lob mode
      escalation_servo->SetTarget(calibrated_theta_esca + PI * 12.1);
      // maximum goes to 38.345 radians
    } else {
      // moving mode
      // set escalation servo to original position
      escalation_servo->SetTarget(calibrated_theta_esca);
    }

    pitch_cmd = dbus->ch3 / 500.0;
    yaw_cmd = clip<float>(dbus->ch2 / 220.0 * 30.0, -30, 30);

    distance_sensor->readValue();

    pitch_servo_L->SetTarget(pitch_servo_L->GetTheta() + pitch_cmd);
    pitch_servo_L->CalcOutput();
    pitch_servo_R->SetTarget(pitch_servo_R->GetTheta() + pitch_cmd);
    pitch_servo_R->CalcOutput();

    escalation_servo->CalcOutput();
    yaw_motor->SetOutput(yaw_cmd);

    pitch_curr = pitch_encoder->getData();
    control::MotorCANBase::TransmitOutput(can1_escalation, 1);
    control::MotorCANBase::TransmitOutput(can1_pitch, 2);
    control::Motor4310::TransmitOutput(can1_yaw, 1);
    osDelay(GIMBAL_TASK_DELAY);
  }
}

void init_gimbal() {
  esca_motor = new control::Motor3508(can1, 0x204);
  // ESCALATION motors initialization
  control::servo_t esca_servo_data;
  esca_servo_data.motor = esca_motor;
  esca_servo_data.max_speed = 2 * PI; // TODO: params need test
  esca_servo_data.max_acceleration = 100 * PI;
  esca_servo_data.transmission_ratio = M3508P19_RATIO;
  esca_servo_data.omega_pid_param = new float [3] {450, 3.6, 20};
//  esca_servo_data.omega_pid_param = new float [3] {150, 1.2, 5};
  esca_servo_data.max_iout = 1000;
  esca_servo_data.max_out = 13000;
  escalation_servo = new control::ServoMotor(esca_servo_data);
  escalation_servo->RegisterJamCallback(jam_callback, 0.205);

  pitch_motor_L = new control::Motor2006(can1, 0x205);
  control::servo_t pitch_servo_data;
  pitch_servo_data.motor = pitch_motor_L;
  pitch_servo_data.max_speed = 100 * PI;    // TODO: params need test
  pitch_servo_data.max_acceleration = 100 * PI;
  pitch_servo_data.transmission_ratio = M2006P36_RATIO;
  pitch_servo_data.omega_pid_param = new float [3] {150, 1.2, 5};
  pitch_servo_data.max_iout = 1000;
  pitch_servo_data.max_out = 13000;
  pitch_servo_L = new control::ServoMotor(pitch_servo_data);

  pitch_motor_R = new control::Motor2006(can1,0x206);
  pitch_servo_data.motor = pitch_motor_R;
  pitch_servo_R = new control::ServoMotor(pitch_servo_data);

  yaw_motor = new control::Motor4310(can1, 0x03, 0x02, control::VEL);

  pitch_encoder = new control::BRTEncoder(can1,0x01,false);

  distance_sensor = distance::SEN_0366_DIST::init(&huart1,0x80);

  esca_yaw_motor = new control::Motor6020(can1,0x208);
}

void kill_gimbal() {
  RM_EXPECT_TRUE(false, "Operation Killed!\r\n");
  control::MotorCANBase* can1_escalation[] = {esca_motor};
  while (true){
    esca_motor->SetOutput(0);
    control::MotorCANBase::TransmitOutput(can1_escalation, 1);
    osDelay(KILLALL_DELAY);
  }
}

