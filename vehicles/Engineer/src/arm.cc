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
// static bsp::Relay* pump = nullptr;


// Motor 4310
static control::Motor4310* forearm_rotate_motor_4 = nullptr;
static control::Motor4310* wrist_rotate_motor_5 = nullptr;
static control::Motor4310* hand_rotate_motor_6 = nullptr;

static const int ARM_TASK_DELAY = 2;

static joint_state_t last_targets = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
static joint_state_t current_joint_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
static joint_state_t joint_targets = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


static float A1_id2_offset; // encoder1 + encoder0 - A1_id2 start angle
static float A1_id1_offset; // encoder0 - A1_id1 start angle
static float A1_id0_offset; // -A1_id0 start angle

static const float encoder0_val_when_arm_upright = -2.43;
static const float encoder1_val_when_arm_upright = 3.23;

static bool killed = false;

/*
upright position:
(elbow) -----
        |     
        |      
        |
      -----
      |   | (base)
      -----
*/

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
  encoder1 = new control::BRTEncoder(can1,0x0A, false);
  encoder0 = new control::BRTEncoder(can1,0x01, true);
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

/** A legal arm position satisfies all of the following constraints:
 * encoder1 (elbow pitch) 2.9 to 3.9
 * encoder0 (base pitch) -2.1 to -3.5
 * encoder1+encoder0 0.1 to 1.8
 * @return true if the arm is in a legal position
 */
bool areJointTargetsLegal(joint_state_t &mechanical_angle) {
  if (mechanical_angle.base_pitch_rotate_2 > -2.1 || mechanical_angle.base_pitch_rotate_2 < -3.5) {
    return false;
  }
  if (mechanical_angle.forearm_pitch_3 < 0.1 || mechanical_angle.forearm_pitch_3 > 1.8) {
    return false;
  }
  if (mechanical_angle.forearm_pitch_3-mechanical_angle.base_pitch_rotate_2 < 2.9 || mechanical_angle.forearm_pitch_3-mechanical_angle.base_pitch_rotate_2 > 4.0) {
    return false;
  }
  return true;
}

void checkAllMotorsConnected(){
  while(true){
    // because A1 will only return motor states if a command is sent to it, so we send a stop command
    modify_stop_cmd(&MotorA1_send, 0);
    unitreeA1_rxtx(huart1);
    modify_stop_cmd(&MotorA1_send, 1);
    unitreeA1_rxtx(huart1);
    modify_stop_cmd(&MotorA1_send, 2);
    unitreeA1_rxtx(huart1);


    bool all_connected = true;

    // because if A1 send back data, temperature will very unlikely be 0
    if(!MotorA1_recv_id00.Temp != 0){
      print("A1 0 not connected\r\n");
      all_connected = false;
    }
    if(!MotorA1_recv_id01.Temp != 0){
      print("A1 1 not connected\r\n");
      all_connected = false;
    }
    if(!MotorA1_recv_id02.Temp != 0){
      print("A1 2 not connected\r\n");
      all_connected = false;
    }

    if(!forearm_rotate_motor_4->connection_flag_){
      print("forearm motor not connected\r\n");
      all_connected = false;
    }

    if(!wrist_rotate_motor_5->connection_flag_){
      print("wrist motor not connected\r\n");
      all_connected = false;
    }

    if(!hand_rotate_motor_6->connection_flag_){
      print("hand motor not connected\r\n");
      all_connected = false;
    }

    if(all_connected){
      print("all motors connected\r\n");
      return;
    }

    osDelay(100);
  }
}

void checkEncodersConnected(){
  while(!encoder0->is_connected() || !encoder1->is_connected()){
    if(!encoder0->is_connected()){
      print("encoder 0 not connected\r\n");
    }
    if (!encoder1->is_connected()){
      print("encoder 1 not connected\r\n");
    }
    osDelay(100);
  }
}

void armTask(void* args) {
  UNUSED(args);
  
  //Add 1s Time for 4310 start up.
  print("START\r\n");
  osDelay(1000);

  const float SBUS_CHANNEL_MAX = 660;

  // check if encoders are connected
  checkEncodersConnected();

  // manually enable motors
  #ifdef USING_DBUS
  while(dbus->swr != remote::DOWN){
    osDelay(100);
    print("waiting for dbus swr to be down\r\n");
  }
  #else
   print("waiting for sbus channel 10 to be greater than 100, now %d\r\n",sbus->ch[9]);
   while(sbus->ch[9] < 100){
    osDelay(100);
   }
  #endif
  
  print("motors enabling\r\n");


  // enable 4310
  // forearm_rotate_motor_4->SetZeroPos();
  forearm_rotate_motor_4->MotorEnable();
  // wrist_rotate_motor_5->SetZeroPos();
  wrist_rotate_motor_5->MotorEnable();
  // hand_rotate_motor_6->SetZeroPos();
  hand_rotate_motor_6->MotorEnable();

  print("now checking all motors connected\r\n");
  // check all motors connected 
  checkAllMotorsConnected();



  // read motor offsets
  // encoder1 - encoder0 = A1_id2 angle
  // encoder0 = A1_id1 angle
  A1_id2_offset = encoder1->getData() + encoder0->getData() - MotorA1_recv_id02.Pos;
  A1_id1_offset = encoder0->getData() - MotorA1_recv_id01.Pos;
  A1_id0_offset = -MotorA1_recv_id00.Pos;


  // TODO: mechanical angle definition

  // Mechanical Angle refers to the angle reference defined manually for easy kinematics/inverse kinematics and/or other purposes
  // for joint 2, mechanical angle is the reading of encoder0
  // for joint 3, mechanical angle is the reading of encoder1-encoder0
  // mechanical angle range
  joint_state_t MECHANICAL_MAX_POS = {0, PI/2, 0, 0, PI/2, PI/2, PI/2};
  joint_state_t MECHANICAL_MIN_POS = {0, -PI/2, 0, 0, -PI/2, -PI/2, -PI/2};
  MECHANICAL_MAX_POS.base_pitch_rotate_2 = -2.76;
  MECHANICAL_MIN_POS.base_pitch_rotate_2 = -2.1;
  MECHANICAL_MAX_POS.forearm_pitch_3 = 1.4;
  MECHANICAL_MIN_POS.forearm_pitch_3 = 0.1;

  ArmReceiveInput();
  last_targets = current_joint_positions;

  MovingAverage moving_average[7]; // filter for 7 "joint"s

  int loop_cnt = 0;
  float last_loop_time = HAL_GetTick();
  UNUSED(last_loop_time);
  while (true) {
    
    if(killed){
      osDelay(100);
      continue;
    }

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
    moving_average[1].AddSample(sbus->ch[0]); // base_yaw
    moving_average[2].AddSample(sbus->ch[1]); // base_pitch
    moving_average[3].AddSample(sbus->ch[2]); // elbow_pitch
    moving_average[4].AddSample(sbus->ch[3]); // forearm_roll
    moving_average[5].AddSample(sbus->ch[4]); // wrist
    moving_average[6].AddSample(sbus->ch[5]); // end
#endif

    temp[1] = clip<float>(moving_average[1].GetAverage(), -SBUS_CHANNEL_MAX, SBUS_CHANNEL_MAX)/SBUS_CHANNEL_MAX;
    temp[2] = clip<float>(moving_average[2].GetAverage(), -SBUS_CHANNEL_MAX, SBUS_CHANNEL_MAX)/SBUS_CHANNEL_MAX;
    temp[3] = clip<float>(moving_average[3].GetAverage(), -SBUS_CHANNEL_MAX, SBUS_CHANNEL_MAX)/SBUS_CHANNEL_MAX;
    temp[4] = clip<float>(moving_average[4].GetAverage(), -SBUS_CHANNEL_MAX, SBUS_CHANNEL_MAX)/SBUS_CHANNEL_MAX;
    temp[5] = clip<float>(moving_average[5].GetAverage(), -SBUS_CHANNEL_MAX, SBUS_CHANNEL_MAX)/SBUS_CHANNEL_MAX;
    temp[6] = clip<float>(moving_average[6].GetAverage(), -SBUS_CHANNEL_MAX, SBUS_CHANNEL_MAX)/SBUS_CHANNEL_MAX;


    joint_state_t mechanical_angle = {0,0,0,0,0,0,0};
    // map sbus input to mechanical angle range
    
    mechanical_angle.base_yaw_rotate_1   = map<float>(temp[1], MECHANICAL_MIN_POS.base_yaw_rotate_1,   MECHANICAL_MAX_POS.base_yaw_rotate_1);
    mechanical_angle.base_pitch_rotate_2 = map<float>(temp[2], MECHANICAL_MIN_POS.base_pitch_rotate_2, MECHANICAL_MAX_POS.base_pitch_rotate_2);
    mechanical_angle.forearm_pitch_3     = map<float>(temp[3], MECHANICAL_MIN_POS.forearm_pitch_3,     MECHANICAL_MAX_POS.forearm_pitch_3);
    mechanical_angle.forearm_roll_4      = map<float>(temp[4], MECHANICAL_MIN_POS.forearm_roll_4,      MECHANICAL_MAX_POS.forearm_roll_4);
    mechanical_angle.wrist_5             = map<float>(temp[5], MECHANICAL_MIN_POS.wrist_5,             MECHANICAL_MAX_POS.wrist_5);
    mechanical_angle.end_6               = map<float>(temp[6], MECHANICAL_MIN_POS.end_6,               MECHANICAL_MAX_POS.end_6);


    // TODO: verify
    if(!areJointTargetsLegal(mechanical_angle)){
      // don't change joint_targets if mechanical angle is illegal
      if(loop_cnt % 100 == 0){
        print("mechanical angle illegal\r\n");
      }
    }else{
      
      // Convert mechanical angle to motor command angle.
      joint_targets.base_yaw_rotate_1 = mechanical_angle.base_yaw_rotate_1 - A1_id0_offset; // no abs encoder
      joint_targets.base_pitch_rotate_2 = mechanical_angle.base_pitch_rotate_2 - A1_id1_offset; // has abs encoder
      joint_targets.forearm_pitch_3 = mechanical_angle.forearm_pitch_3 - A1_id2_offset; // has abs encoder
      joint_targets.forearm_roll_4 = mechanical_angle.forearm_roll_4;
      joint_targets.wrist_5 = mechanical_angle.wrist_5;
      joint_targets.end_6 = mechanical_angle.end_6;
    }


    // Clip between last_target_pose +- MAX_DELTA
    // e.g. 1.0f/1000*5 roughly corresponds to 1.0 radians per second (because roughly 5ms per loop)
    joint_state_t MAX_DELTA = {0, 0.5f/ 1000 * 5, 0.2f/1000*5, 0.2f/1000*5, 1.5f/1000*5, 2.0f/1000*5, 2.0f/1000*5};
    joint_state_t smoothed_targets = {0,0,0,0,0,0,0};
    smoothed_targets.base_yaw_rotate_1 =   clip<float>(joint_targets.base_yaw_rotate_1,   last_targets.base_yaw_rotate_1   - MAX_DELTA.base_yaw_rotate_1, last_targets.base_yaw_rotate_1   + MAX_DELTA.base_yaw_rotate_1); 
    smoothed_targets.base_pitch_rotate_2 = clip<float>(joint_targets.base_pitch_rotate_2, last_targets.base_pitch_rotate_2 - MAX_DELTA.base_pitch_rotate_2, last_targets.base_pitch_rotate_2 + MAX_DELTA.base_pitch_rotate_2);
    smoothed_targets.forearm_pitch_3 =     clip<float>(joint_targets.forearm_pitch_3,     last_targets.forearm_pitch_3     - MAX_DELTA.forearm_pitch_3, last_targets.forearm_pitch_3     + MAX_DELTA.forearm_pitch_3);
    smoothed_targets.forearm_roll_4 =      clip<float>(joint_targets.forearm_roll_4,      last_targets.forearm_roll_4      - MAX_DELTA.forearm_roll_4, last_targets.forearm_roll_4      + MAX_DELTA.forearm_roll_4);
    smoothed_targets.wrist_5 =             clip<float>(joint_targets.wrist_5,             last_targets.wrist_5             - MAX_DELTA.wrist_5, last_targets.wrist_5             + MAX_DELTA.wrist_5);
    smoothed_targets.end_6 =               clip<float>(joint_targets.end_6,               last_targets.end_6               - MAX_DELTA.end_6, last_targets.end_6               + MAX_DELTA.end_6);

    ArmSetTarget(smoothed_targets);
    last_targets = smoothed_targets;

    // print every 100 loops
    if(loop_cnt % 100 == 0){
      print("encoder0 %f, encoder1 %f\n", encoder0->getData(), encoder1->getData());
      // print("A1_offset %f, %f\n", A1_id1_offset, A1_id2_offset);
      print("mechanical: %f, %f, %f, %f, %f, %f \n", mechanical_angle.base_yaw_rotate_1, mechanical_angle.base_pitch_rotate_2, mechanical_angle.forearm_pitch_3, mechanical_angle.forearm_roll_4, mechanical_angle.wrist_5, mechanical_angle.end_6);
      // print("current: %f, %f, %f, %f, %f, %f \n", current_joint_positions.base_yaw_rotate_1, current_joint_positions.base_pitch_rotate_2, current_joint_positions.forearm_pitch_3, current_joint_positions.forearm_roll_4, current_joint_positions.wrist_5, current_joint_positions.end_6);
      // print("smoothed: %f, %f, %f, %f, %f, %f \n", smoothed_targets.base_yaw_rotate_1, smoothed_targets.base_pitch_rotate_2, smoothed_targets.forearm_pitch_3, smoothed_targets.forearm_roll_4, smoothed_targets.wrist_5, smoothed_targets.end_6);
      // print("targets: %f, %f, %f, %f, %f, %f \n",joint_targets.base_yaw_rotate_1, joint_targets.base_pitch_rotate_2, joint_targets.forearm_pitch_3, joint_targets.forearm_roll_4, joint_targets.wrist_5, joint_targets.end_6);
      // print("loop time %fms \r\n", (HAL_GetTick()-last_loop_time)/100.0);
      last_loop_time = HAL_GetTick();

    }

    // manually confirm on the first loop to prevent accidental movement
    if(loop_cnt == 0){
  #ifdef USING_DBUS
      while(dbus->swr != remote::DOWN){
        osDelay(100);
        print("waiting for dbus swr to be down\r\n");
  }
  #else
      print("waiting for sbus channel 5 to be smaller than -100\r\n");
      while(sbus->ch[9] > -100){
        osDelay(100);
      }
  #endif
    }

    ArmTransmitOutput();
    ArmReceiveInput();

    osDelay(ARM_TASK_DELAY);
    loop_cnt++;
  }
}

static const float A1_Kp = 0.03, A1_Kw = 2.0;
static const float m4310_Kp= 3, m4310_Kd = 1;

/**
 * @brief set the target joint angles for the arm
*/
int ArmSetTarget(joint_state_t target) {
  // // A1 
  modify_pos_cmd(&MotorA1_send, 0, target.base_yaw_rotate_1, 0.06, 8.0);
  // modify_stop_cmd(&MotorA1_send, 0);

  unitreeA1_rxtx(huart1);

  modify_pos_cmd(&MotorA1_send, 1, target.base_pitch_rotate_2, 0.06, 8.0);
  // modify_stop_cmd(&MotorA1_send, 1);

  unitreeA1_rxtx(huart1);

  modify_pos_cmd(&MotorA1_send, 2, target.forearm_pitch_3, 0.06, 8.0);
  // modify_stop_cmd(&MotorA1_send, 2);

  unitreeA1_rxtx(huart1);


  // 4310
  forearm_rotate_motor_4->SetOutput(target.forearm_roll_4, target.forearm_roll_4, 10, 0.5, 0);
  wrist_rotate_motor_5->SetOutput(target.wrist_5, target.wrist_5, 10, 0.5, 0);
  hand_rotate_motor_6->SetOutput(target.end_6, target.end_6, 10, 0.5, 0);

  return 0;
}

void ArmTransmitOutput() {
  // unitreeA1_rxtx(huart1);
  
  control::Motor4310* forearm_motors[3] = {forearm_rotate_motor_4, wrist_rotate_motor_5, hand_rotate_motor_6};
  control::Motor4310::TransmitOutput(forearm_motors, 3);
}


void ArmReceiveInput() {
  current_joint_positions.base_yaw_rotate_1 = MotorA1_recv_id00.Pos;
  current_joint_positions.base_pitch_rotate_2 = MotorA1_recv_id01.Pos;
  current_joint_positions.forearm_pitch_3 = MotorA1_recv_id02.Pos;
  current_joint_positions.forearm_roll_4 = forearm_rotate_motor_4->GetTheta(); 
  current_joint_positions.wrist_5 = wrist_rotate_motor_5->GetTheta();
  current_joint_positions.end_6 = hand_rotate_motor_6->GetTheta();

}

void kill_arm() {
  killed = true;

  modify_stop_cmd(&MotorA1_send, 0);
  unitreeA1_rxtx(huart1);

  modify_stop_cmd(&MotorA1_send, 1);  
  unitreeA1_rxtx(huart1);

  modify_stop_cmd(&MotorA1_send, 2);
  unitreeA1_rxtx(huart1);

  forearm_rotate_motor_4->MotorDisable();
  wrist_rotate_motor_5->MotorDisable();
  hand_rotate_motor_6->MotorDisable();
}

