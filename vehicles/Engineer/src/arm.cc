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

#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "dbus.h"
#include "sbus.h"
#include "main.h"
#include "motor.h"
#include "unitree_motor.h"
#include "utils.h"
#include "bsp_can_bridge.h"
#include "moving_average.h"

#include "arm.h"
#include "arm_config.h"
#include "encoder.h"
#include "bsp_relay.h"

#include "A1_motor_drive.h"

// static bsp::CAN* can1 = nullptr;
// static remote::DBUS* sbus = nullptr;
static control::BRTEncoder* encoder0= nullptr;
static control::BRTEncoder* encoder1= nullptr;
static float A1_zero[3] = {0.39, 0, 0};
// static bsp::Relay* pump = nullptr;


// Motor 4310
static control::Motor4310* forearm_rotate_motor_4 = nullptr;
static control::Motor4310* wrist_rotate_motor_5 = nullptr;
static control::Motor4310* hand_rotate_motor_6 = nullptr;

static const int ARM_TASK_DELAY = 2;

static joint_state_t current_joint_state = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
static joint_state_t target = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
static joint_state_t last_target = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


/**
 * forearm_rotate_motor     RX=0x02 TX=0x01
 * wrist_rotate_motor       RX=0x04 TX=0x03
 * hand_rotete_motor        RX=0x06 TX=0x05
 * 
 * elbow_rortate_motor      ID=0
 * upper_arm_motor          ID=1
 * base_motor               ID=2
*/

// entire arm

// TODO fix transmit output for 4310 multi motors

void init_arm_A1() {
  // print_use_uart(&huart4);
  bsp::SetHighresClockTimer(&htim5);
  encoder0 = new control::BRTEncoder(can1,0x0A, false);
  encoder1 = new control::BRTEncoder(can1,0x01, true);
  // pump = new bsp::Relay(K2_GPIO_Port,K2_Pin);

  // motor 4310
  forearm_rotate_motor_4 = new control::Motor4310(can1, 0x08, 0x07, control::MIT);
  wrist_rotate_motor_5 = new control::Motor4310(can1, 0x04, 0x03, control::MIT);
  hand_rotate_motor_6 = new control::Motor4310(can1, 0x06, 0x05, control::MIT);

  // Init M4310 * 3
  /* rx_id = Master id
   * tx_id = CAN id
   * see example/m4310_mit.cc
   */

  // Init A1 * 3

}

void armTask(void* args) {
  UNUSED(args);
  
  //Add 1s Time for 4310 start up.
  print("START\r\n");
  osDelay(1000);

  const float SBUS_CHANNEL_MAX = 660;
  const float A1_MAX_DELTA = float(1.0) / 1000 * (12);


  print("encoder(s) not connected");

  while(!encoder0->is_connected() || !encoder1->is_connected()){
    if(!encoder0->is_connected()){
      print("encoder 0 not connected\r\n");
    }
    if (!encoder1->is_connected()){
      print("encoder 1 not connected\r\n");
    }
    osDelay(100);
  }

  #ifdef USING_DBUS
  while(dbus->swr != remote::DOWN){
    osDelay(100);
    print("waiting for dbus swr to be down\r\n");
  }
  #else
   while(sbus->ch[9] < 100){
    osDelay(100);
    print("waiting for sbus channel 10 to be greater than 100\r\n");
   }
  #endif

  forearm_rotate_motor_4->SetZeroPos();
  forearm_rotate_motor_4->MotorEnable();
  wrist_rotate_motor_5->SetZeroPos();
  wrist_rotate_motor_5->MotorEnable();
  hand_rotate_motor_6->SetZeroPos();
  hand_rotate_motor_6->MotorEnable();

  #ifdef USING_DBUS
  while(dbus->swr != remote::DOWN){
    osDelay(100);
    print("waiting for dbus swr to be down\r\n");
  }
  #else
   while(sbus->ch[9] > -100){
    osDelay(100);
    print("waiting for sbus channel 5 to be greater than 100\r\n");
   }
  #endif

  joint_state_t MAX_POS = {0, PI/2, PI/4, PI/8, PI/2, PI/2, PI/2};
  joint_state_t MIN_POS = {0, -PI/2, -PI/4, -PI/8, -PI/2, -PI/2, -PI/2};

  MovingAverage moving_average[7];

  int loop_cnt = 0;
  while (true) {
    // pump->Off();
    // if (sbus->ch[10] > 0.5) {
      // pump->On();
    // }
  
    float temp[16];

    // read and filter sbus
#ifdef USING_DBUS
    moving_average[1].AddSample(dbus->ch1); // base_yaw
    moving_average[2].AddSample(dbus->ch2); // base_pitch
    moving_average[3].AddSample(dbus->ch3); // elbow_pitch
#else
    moving_average[1].AddSample(sbus->ch[3]); // base_yaw
    moving_average[2].AddSample(sbus->ch[4]); // base_pitch
    moving_average[3].AddSample(sbus->ch[5]); // elbow_pitch
    moving_average[4].AddSample(sbus->ch[0]); // forearm_roll
    moving_average[5].AddSample(sbus->ch[1]); // wrist
    moving_average[6].AddSample(sbus->ch[2]); // end
#endif

    temp[1] = clip<float>(moving_average[1].GetAverage(), -SBUS_CHANNEL_MAX, SBUS_CHANNEL_MAX);
    temp[2] = clip<float>(moving_average[2].GetAverage(), -SBUS_CHANNEL_MAX, SBUS_CHANNEL_MAX);
    temp[3] = clip<float>(moving_average[3].GetAverage(), -SBUS_CHANNEL_MAX, SBUS_CHANNEL_MAX);
    temp[4] = clip<float>(moving_average[4].GetAverage(), -SBUS_CHANNEL_MAX, SBUS_CHANNEL_MAX);
    temp[5] = clip<float>(moving_average[5].GetAverage(), -SBUS_CHANNEL_MAX, SBUS_CHANNEL_MAX);
    temp[6] = clip<float>(moving_average[6].GetAverage(), -SBUS_CHANNEL_MAX, SBUS_CHANNEL_MAX);


    joint_state_t field_angles = {0,0,0,0,0,0,0};
    // map sbus input to A1 range
    field_angles.base_yaw_rotate_1   = map<float>(temp[1]/SBUS_CHANNEL_MAX, MIN_POS.base_yaw_rotate_1,   MAX_POS.base_yaw_rotate_1);
    field_angles.base_pitch_rotate_2 = map<float>(temp[2]/SBUS_CHANNEL_MAX, MIN_POS.base_pitch_rotate_2, MAX_POS.base_pitch_rotate_2);
    field_angles.forearm_pitch_3     = map<float>(temp[3]/SBUS_CHANNEL_MAX, MIN_POS.forearm_pitch_3,     MAX_POS.forearm_pitch_3);
    field_angles.forearm_roll_4      = map<float>(temp[4]/SBUS_CHANNEL_MAX, MIN_POS.forearm_roll_4,      MAX_POS.forearm_roll_4);
    field_angles.wrist_5             = map<float>(temp[5]/SBUS_CHANNEL_MAX, MIN_POS.wrist_5,             MAX_POS.wrist_5);
    field_angles.end_6               = map<float>(temp[6]/SBUS_CHANNEL_MAX, MIN_POS.end_6,               MAX_POS.end_6);

    // Convert Encoder Value to A1 .
    target.base_yaw_rotate_1 = field_angles.base_yaw_rotate_1; // no abs encoder
    target.base_pitch_rotate_2 = field_angles.base_pitch_rotate_2; // has abs encoder
    target.forearm_pitch_3 = field_angles.forearm_pitch_3; // has abs encoder
    target.forearm_roll_4 = field_angles.forearm_roll_4;
    target.wrist_5 = field_angles.wrist_5;
    target.end_6 = field_angles.end_6;
    
    // Clip between last_target_pose +- MAX_DELTA
    target.base_yaw_rotate_1 =   clip<float>(target.base_yaw_rotate_1,   last_target.base_yaw_rotate_1   - A1_MAX_DELTA, last_target.base_yaw_rotate_1   + A1_MAX_DELTA); 
    target.base_pitch_rotate_2 = clip<float>(target.base_pitch_rotate_2, last_target.base_pitch_rotate_2 - A1_MAX_DELTA, last_target.base_pitch_rotate_2 + A1_MAX_DELTA);
    target.forearm_pitch_3 =     clip<float>(target.forearm_pitch_3,     last_target.forearm_pitch_3     - A1_MAX_DELTA, last_target.forearm_pitch_3     + A1_MAX_DELTA);
    target.forearm_roll_4 =      clip<float>(target.forearm_roll_4,      last_target.forearm_roll_4      - A1_MAX_DELTA, last_target.forearm_roll_4      + A1_MAX_DELTA);
    target.wrist_5 =             clip<float>(target.wrist_5,             last_target.wrist_5             - A1_MAX_DELTA, last_target.wrist_5             + A1_MAX_DELTA);
    target.end_6 =               clip<float>(target.end_6,               last_target.end_6               - A1_MAX_DELTA, last_target.end_6               + A1_MAX_DELTA);

    last_target.base_yaw_rotate_1   = target.base_yaw_rotate_1;
    last_target.base_pitch_rotate_2 = target.base_pitch_rotate_2;
    last_target.forearm_pitch_3     = target.forearm_pitch_3;
    last_target.forearm_roll_4      = target.forearm_roll_4;
    last_target.wrist_5             = target.wrist_5;
    last_target.end_6               = target.end_6;

    ArmTurnAbsolute(target);

    // update A1 rotor encoder reading
    // base_pitch_A1_rotor_encoder_reading = hard_wrap<float>((-encoder1->angle_ + BASE_PITCH_A1_ZERO_ENCODER_VAL)*A1->gear_ratio, 0, 2*PI);
    // elbow_pitch_A1_rotor_encoder_reading = hard_wrap<float>((-encoder0->angle_ + ELBOW_PITCH_A1_ZERO_ENCODER_VAL)*A1->gear_ratio, 0, 2*PI);
    // base_pitch_A1_init_target = base_pitch_A1_rotor_encoder_reading / A1->gear_ratio; // A1->gear_ratio is A1 gear ratio
    // elbow_pitch_A1_init_target = elbow_pitch_A1_rotor_encoder_reading / A1->gear_ratio; // A1->gear_ratio is A1 gear ratio


    if(loop_cnt % 100 == 0){
      set_cursor(0, 0);
      clear_screen();
      print("%f, %f, %f, %f, %f, %f \n",target.base_yaw_rotate_1, target.base_pitch_rotate_2, target.forearm_pitch_3, target.forearm_roll_4, target.wrist_5, target.end_6);
    
    }
    ArmTransmitOutput();
    osDelay(ARM_TASK_DELAY);
  }
}

static const float A1_Kp = 0.05, A1_Kw = 1.0;
static const float m4310_Kp= 3, m4310_Kd = 1;
int ArmTurnAbsolute(joint_state_t target) {
  // A1
  current_joint_state.base_yaw_rotate_1 = target.base_yaw_rotate_1;
  current_joint_state.base_pitch_rotate_2 = target.base_pitch_rotate_2;
  current_joint_state.forearm_pitch_3 = target.forearm_pitch_3;
  A1::modfiy_pos_cmd(&A1::MotorA1_send, 0, target.base_yaw_rotate_1, A1_Kp, A1_Kw);
  A1::modfiy_pos_cmd(&A1::MotorA1_send, 1, target.base_pitch_rotate_2, A1_Kp, A1_Kw);
  A1::modfiy_pos_cmd(&A1::MotorA1_send, 2, target.forearm_pitch_3, A1_Kp, A1_Kw);

  // 4310
  forearm_rotate_motor_4->SetOutput(target.forearm_roll_4, target.forearm_roll_4, 10, 0.5, 0);
  wrist_rotate_motor_5->SetOutput(target.wrist_5, target.wrist_5, 10, 0.5, 0);
  hand_rotate_motor_6->SetOutput(target.end_6, target.end_6, 10, 0.5, 0);

  return 0;
}

void ArmTransmitOutput() {
  A1::unitreeA1_rxtx(huart1);
  
  control::Motor4310* forearm_motors[3] = {forearm_rotate_motor_4, wrist_rotate_motor_5, hand_rotate_motor_6};
  control::Motor4310::TransmitOutput(forearm_motors, 3);
}


void kill_arm() {
  A1::modify_stop_cmd(&A1::MotorA1_send, 0);
  A1::modify_stop_cmd(&A1::MotorA1_send, 1);
  A1::modify_stop_cmd(&A1::MotorA1_send, 2);
  A1::unitreeA1_rxtx(huart1);

  forearm_rotate_motor_4->MotorDisable();
  wrist_rotate_motor_5->MotorDisable();
  hand_rotate_motor_6->MotorDisable();
}

