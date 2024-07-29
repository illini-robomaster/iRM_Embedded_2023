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
static const int GIMBAL_TASK_DELAY = 3;

static control::MotorCANBase* esca_motor = nullptr;
static control::MotorPWMBase* scope_motor = nullptr;
static control::ServoMotor* escalation_servo = nullptr;


static control::MotorCANBase* pitch_motor_L = nullptr;
static control::MotorCANBase* pitch_motor_R = nullptr;

static control::ServoMotor* pitch_servo_L = nullptr;
static control::ServoMotor* pitch_servo_R = nullptr;

static control::Motor4310* yaw_motor = nullptr;

static control::BRTEncoder *pitch_encoder = nullptr;

//static distance::SEN_0366_DIST *distance_sensor = nullptr;




float calibrated_theta_esca = 0;



float pitch_curr = 0;
float pitch_target = 0;

float curr_yaw = 0;

float vx_keyboard = 0;
float vy_keyboard = 0;
float vx_remote,vy_remote;
float vx_set,vy_set;

int p_l_out;
int p_r_out;
int esca_out;

control::servo_t pitch_servo_data;

BoolEdgeDetector scope_sw(false);
volatile bool scope_on = false;

void jam_callback(control::ServoMotor* servo, const control::servo_jam_t data) {
  UNUSED(data);
  control::MotorCANBase* can1_motor[] = {servo->GetMotor()};
  servo->GetMotor()->SetOutput(0);
  control::MotorCANBase::TransmitOutput(can1_motor, 1);
  osDelay(100);
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

//  while (!distance_sensor->begin()){
////    print("distance sensor initializing\r\n");
//    osDelay(100);
//  }

  while (true) {
    if (dbus->keyboard.bit.B || dbus->swr == remote::DOWN || !key->Read()) break;
    print("preparing gimbal \r\n");
    osDelay(100);

  }

//  distance_sensor->continuousMeasure();
  yaw_motor->SetZeroPos();
  yaw_motor->MotorEnable();

  /*
   * escalation reset
   */
  while (true){
    escalation_servo->SetTarget(escalation_servo->GetTheta() - PI * 2000,true);
    escalation_servo->CalcOutput(&esca_out);
    control::MotorCANBase::TransmitOutput(can1_escalation, 1);
    osDelay(GIMBAL_TASK_DELAY);
    if (escalation_servo->ZeroingExit()){
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

//  int count = 0;
//  int prev_angle_l = 0;
//  int prev_angle_r = 0;
//  while (true) {
//
//    while (pitch_motor_R->connection_flag_ == false || pitch_motor_L->connection_flag_ == false){
//      print("pitch motor not connected, status: pitch_l: %d, pitch_r: %d\r\n", pitch_motor_L->connection_flag_, pitch_motor_R->connection_flag_);
//      osDelay(100);
//    }
//    pitch_servo_L->SetTarget(pitch_servo_L->GetTheta() - PI * 2000, true);
//    pitch_servo_R->SetTarget(pitch_servo_R->GetTheta() - PI * 2000, true);
//    pitch_servo_L->CalcOutput(&p_l_out);
//    pitch_servo_R->CalcOutput(&p_r_out);
//
////    print("angles: %f, %f\r\n", pitch_servo_L->GetTheta(), pitch_servo_R->GetTheta());
//    if (abs(int (pitch_servo_L->GetTheta())) == abs(prev_angle_l) && abs(int (pitch_servo_R->GetTheta())) ==
//                                                                     abs(prev_angle_r)){
//      count++;
//    } else {
//      count = 0;
//    }
//
//    prev_angle_l = int (pitch_servo_L->GetTheta());
//    prev_angle_r = int (pitch_servo_R->GetTheta());
//    if (count > 100 || dbus->swr == remote::MID){
//      pitch_motor_L->SetOutput(0);
//      pitch_motor_R->SetOutput(0);
//      control::MotorCANBase::TransmitOutput(can1_pitch, 2);
//      print("pitch motors reset\r\n");
//      break;
//    }
//    control::MotorCANBase::TransmitOutput(can1_pitch, 2);
//    osDelay(GIMBAL_TASK_DELAY);
//    print("count: %d\r\n",count);
//  }



  pitch_servo_L->SetMaxCurrent(pitch_servo_data, 2000);
  pitch_servo_R->SetMaxCurrent(pitch_servo_data, 2000);

  pitch_servo_L->SetMaxSpeed(7*PI);
  pitch_servo_R->SetMaxSpeed(7*PI);




//  float pitch_L_val = pitch_servo_L->GetTheta();
//  float pitch_R_val = pitch_servo_R->GetTheta();

  pitch_servo_L->ResetTheta();
  pitch_servo_R->ResetTheta();


  pitch_target = 0.0;

  pitch_curr = -1.0; // make sure it is initialized.

  pitch_cmd = 0.0;

  yaw_cmd = 0.0;

//  bool drive_flag = false;

  while (true) {
    // Bool Edge Detector for lob mode switch or osEventFlags wait for a signal from different threads

    lob_mode_sw.input(dbus->keyboard.bit.SHIFT || dbus->swl == remote::UP || !key->Read());
    if (lob_mode_sw.posEdge() /*&& dbus->swr == remote::MID*/){
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
      print("lob\r\n");
      // maximum goes to 38.345 radians
    } else {
      // moving mode
      // set escalation servo to original position
      print("regular\r\n");
      escalation_servo->SetTarget(calibrated_theta_esca);
    }

    if (dbus->keyboard.bit.A) vx_keyboard -= 61.5;
    if (dbus->keyboard.bit.D) vx_keyboard += 61.5;
    if (dbus->keyboard.bit.W) vy_keyboard += 61.5;
    if (dbus->keyboard.bit.S) vy_keyboard -= 61.5;


    if (-35 <= vx_keyboard && vx_keyboard <= 35) vx_keyboard = 0;
    if (-35 <= vy_keyboard && vy_keyboard <= 35) vy_keyboard = 0;

    if (vx_keyboard > 0)
      vx_keyboard -= 60;
    else if (vx_keyboard < 0)
      vx_keyboard += 60;

    if (vy_keyboard > 0)
      vy_keyboard -= 60;
    else if (vy_keyboard < 0)
      vy_keyboard += 60;

    vx_keyboard = clip<float>(vx_keyboard, -1200, 1200);
    vy_keyboard = clip<float>(vy_keyboard, -1200, 1200);

    vx_remote = dbus->ch0;
    vy_remote = dbus->ch1;

    vx_set = vx_keyboard + vx_remote;
    vy_set = vy_keyboard + vy_remote;

//    send->cmd.id = bsp::VX;
//    send->cmd.data_float = Dead ? 0 : vx_set;
//    send->TransmitOutput();
//
//    send->cmd.id = bsp::VY;
//    send->cmd.data_float = Dead ? 0 : vy_set;
//    send->TransmitOutput();

    pitch_cmd = dbus->ch3 / 500.0;
    yaw_cmd = clip<float>(dbus->ch2 / 220.0 * 30.0, -30, 30);

    scope_sw.input(dbus->keyboard.bit.C);
    if (scope_sw.posEdge()){
      scope_on = !scope_on;
    }
    if (scope_on){
      scope_motor->SetOutput(1500);
    } else {
      scope_motor->SetOutput(3);
    }
    if (!forward_key->Read()){
      pitch_servo_L->SetTarget(pitch_servo_L->GetTheta() + PI * 100, true);
      pitch_servo_R->SetTarget(pitch_servo_R->GetTheta() + PI * 100, true);
      osDelay(GIMBAL_TASK_DELAY);
      pitch_servo_L->CalcOutput(&p_l_out);
      pitch_servo_R->CalcOutput(&p_r_out);
    } else if (!backward_key->Read()) {
      pitch_servo_L->SetTarget(pitch_servo_L->GetTheta() - PI * 100,true);
      pitch_servo_R->SetTarget(pitch_servo_R->GetTheta() - PI * 100,true);
      osDelay(GIMBAL_TASK_DELAY);
      pitch_servo_L->CalcOutput(&p_l_out);
      pitch_servo_R->CalcOutput(&p_r_out);
    }  else {
      p_l_out = 0;
      p_r_out = 0;

    }
//    print("pitch_L: %f, pitch_R: %f\r\n", pitch_servo_L->GetTheta(), pitch_servo_R->GetTheta());
    // TODO: considering change pitch motors to velocity loop
    osDelay(GIMBAL_TASK_DELAY);
    escalation_servo->CalcOutput(&esca_out);
    yaw_motor->SetOutput(yaw_cmd);

    pitch_motor_L->SetOutput(p_l_out);
    pitch_motor_R->SetOutput(p_r_out);

    pitch_curr = pitch_encoder->getData();

    control::MotorCANBase::TransmitOutput(can1_escalation, 1);
    control::MotorCANBase::TransmitOutput(can1_pitch, 2);
    control::Motor4310::TransmitOutput(can1_yaw, 1);

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
  esca_servo_data.omega_pid_param = new float [3] {150, 1.2, 5};
//  esca_servo_data.omega_pid_param = new float [3] {150, 1.2, 5};
  esca_servo_data.max_iout = 2000;
  esca_servo_data.max_out = 10000;
  escalation_servo = new control::ServoMotor(esca_servo_data);
  escalation_servo->RegisterJamCallback(jam_callback, 0.105);

  pitch_motor_L = new control::Motor2006(can1, 0x206);

  pitch_servo_data.motor = pitch_motor_L;
  pitch_servo_data.max_speed = PI * 4 ;    // TODO: params need test
  pitch_servo_data.max_acceleration = PI * 100;
  pitch_servo_data.transmission_ratio = M2006P36_RATIO;
  pitch_servo_data.omega_pid_param = new float [3] {450, 3.6, 0};
  pitch_servo_data.max_iout = 500;
  pitch_servo_data.max_out = 10000;
  pitch_servo_L = new control::ServoMotor(pitch_servo_data);
  pitch_motor_R = new control::Motor2006(can1,0x207);
  // DO NOT USE RX_ID: 0x205!!!!!!!!!
  pitch_servo_data.motor = pitch_motor_R;
  pitch_servo_R = new control::ServoMotor(pitch_servo_data);

  scope_motor = new control::MotorPWMBase(&htim1, TIM_CHANNEL_1, 1000000, 50, 1500);

  yaw_motor = new control::Motor4310(can1, 0x04, 0x05, control::MIT);

  pitch_encoder = new control::BRTEncoder(can1,0x01,false);

  // distance sensor initialization, remove if not connected, will stop the initialization process
//  distance_sensor = distance::SEN_0366_DIST::init(&huart1,0x80);

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

