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

#include "arm.h"
#include "arm_config.h"
#include "encoder.h"
#include "bsp_relay.h"

//#define SINGLE_BOARD

static bsp::CAN* can1 = nullptr;
static remote::SBUS* sbus = nullptr;
static control::BRTEncoder* encoder0= nullptr;
static control::BRTEncoder* encoder1= nullptr;
static float A1_zero[3] = {0.13, 0, 0};
// static bsp::Relay* pump = nullptr;
#ifndef SINGLE_BOARD
static bsp::CAN* can2 = nullptr;
static bsp::CanBridge* send = nullptr;
static const int ARM_TASK_DELAY = 2;

#endif

// 3508
//static control::MotorCANBase* motor1 = nullptr;
//static control::SteeringMotor* base_translate_motor = nullptr;

//static bsp::GPIO* base_translate_pe_sensor = nullptr;
//bool base_translate_align_detect() {
//  return base_translate_pe_sensor->Read() == true;
//}

/**
 * forearm_rotate_motor     RX=0x02 TX=0x01
 * wrist_rotate_motor       RX=0x04 TX=0x03
 * hand_rotete_motor        RX=0x06 TX=0x05
 * 
 * elbow_rortate_motor      ID=0
 * upper_arm_motor          ID=1
 * base_motor               ID=2
*/

 #define RX_SIGNAL (1 << 0)

 const osThreadAttr_t A1TaskAttribute = {.name = "A1Task",
         .attr_bits = osThreadDetached,
         .cb_mem = nullptr,
         .cb_size = 0,
         .stack_mem = nullptr,
         .stack_size = 128 * 4,
         .priority = (osPriority_t)osPriorityNormal,
         .tz_module = 0,
         .reserved = 0};

 osThreadId_t A1TaskHandle;

 class CustomUART: public bsp::UART {
 public:
     using bsp::UART::UART;
 protected:
     /* notify application when rx data is pending read */
     void RxCompleteCallback() override final { osThreadFlagsSet(A1TaskHandle, RX_SIGNAL); }
 };

  static CustomUART* A1_uart = nullptr;
  static control::UnitreeMotor* A1 = nullptr;

 void A1Task(void* arg) {
     UNUSED(arg);
     uint32_t length;
     uint8_t* data;

     while (true) {
         /* wait until rx data is available */
         uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
         if (flags & RX_SIGNAL) {  // unnecessary check
             /* time the non-blocking rx / tx calls (should be <= 1 osTick) */
             length = A1_uart->Read(&data);
             if ((int)length == A1->recv_length)
               A1->ExtractData(communication::package_t{data, (int)length});
         }
     }
 }

// 4310
static control::Motor4310* forearm_rotate_motor = nullptr;
static control::Motor4310* wrist_rotate_motor = nullptr;
static control::Motor4310* hand_rotate_motor = nullptr;

// entire arm
static joint_state_t current_joint_state = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// TODO fix transmit output for 4310 multi motors

void RM_RTOS_Init() {
  print_use_uart(&huart4);
  bsp::SetHighresClockTimer(&htim5);
//   control::steering_t steering_data;
  can1 = new bsp::CAN(&hcan1);
  sbus = new remote::SBUS(&huart3);
  encoder0 = new control::BRTEncoder(can1,0x0A);
  encoder1 = new control::BRTEncoder(can1,0x01);
  // pump = new bsp::Relay(K2_GPIO_Port,K2_Pin);

#ifndef SINGLE_BOARD
  can2 = new bsp::CAN(&hcan2, false);
  send = new bsp::CanBridge(can2,0x20A, 0x20B);
#endif

  // Init M4310 * 3
  /* rx_id = Master id
   * tx_id = CAN id
   * see example/m4310_mit.cc
   */
  forearm_rotate_motor = new control::Motor4310(can1, FOREARM_ROTATE_RX_ID, FOREARM_ROTATE_TX_ID, control::MIT);
  wrist_rotate_motor = new control::Motor4310(can1, WRIST_ROTATE_RX_ID, WRIST_ROTATE_TX_ID, control::MIT);
  hand_rotate_motor = new control::Motor4310(can1, HAND_ROTATE_RX_ID, HAND_ROTATE_TX_ID, control::MIT);

  // Init A1 * 3
   A1_uart = new CustomUART(A1_UART);
   A1_uart->SetupRx(300);
   A1_uart->SetupTx(300);

   A1 = new control::UnitreeMotor();

   A1->Stop(0);
   osDelay(A1_CONTROL_DELAY);
   A1->Stop(1);
   osDelay(A1_CONTROL_DELAY);
   A1->Stop(2);
   osDelay(A1_CONTROL_DELAY);

}

 void RM_RTOS_Threads_Init(void) {
   A1TaskHandle = osThreadNew(A1Task, nullptr, &A1TaskAttribute);
 }

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  //Add 1s Time for 4310 start up.
  print("START\r\n");
  osDelay(1000);

  // 4310 init state
  // TODO: config zero pos with 4310 config assist

  // forearm_rotate_motor->SetZeroPos();
  // osDelay(20);
  // forearm_rotate_motor->MotorEnable();
  // osDelay(20);
  // wrist_rotate_motor->SetZeroPos();
  // osDelay(20);
  // wrist_rotate_motor->MotorEnable();
  // osDelay(20);
  // hand_rotate_motor->SetZeroPos();
  // osDelay(20);
  // hand_rotate_motor->MotorEnable(); 
   
  //MOTOR 0-5
  //6lv  7lh  8rv
  //9 base


  const float SBUS_CHANNEL_MAX = 660;

  const float BASE_PITCH_FIELD_ZERO_ENCODER_VAL = 2.448; // the encoder value when the big arm is pointing upwards
  const float ELBOW_PITCH_FIELD_ZERO_ENCODER_VAL = 2.448; // TODO: the encoder value when the small arm is pointing upwards
 
  // When no power, A1 absolute rotor encoder can only store 0~2PI,
  // After powered on, A1 encoder will be able to record multiple turns.

  // ===========================================================================================================
  // CAUTION: If C board is powered but A1 lost power and moved. This will result in a wrong reading and A1 will suddenly move after powered on again.
  // ===========================================================================================================

  // Find A1 Rotor Encoder reading (0~2PI) in terms of A1 Rotor
  const float BASE_PITCH_A1_ZERO_ENCODER_VAL = 2.939107; // the base encoder value when A1 absolute encoder reads 0.
  const float ELBOW_PITCH_A1_ZERO_ENCODER_VAL = 3.804;// TODO: the elbow encoder value when A1 absolute encoder reads 0.

  // (+-) Current Encoder Reading = n* Encoder Delta Per A1 Rotor Turn + A1 Rotor Encoder Reading + Encoder Value When A1 Rotor Is Zero
  // -> A1 Rotor Encoder Reading = (+-) Current Encoder Reading - n* Encoder Delta Per A1 Rotor Turn - Encoder Value When A1 Rotor Is Zero

  float base_pitch_A1_rotor_encoder_reading = hard_wrap<float>((-encoder1->angle_ + BASE_PITCH_A1_ZERO_ENCODER_VAL)*9.1, 0, 2*PI);
  float elbow_pitch_A1_rotor_encoder_reading = hard_wrap<float>((-encoder0->angle_ + ELBOW_PITCH_A1_ZERO_ENCODER_VAL)*9.1, 0, 2*PI);
  float base_pitch_A1_target = base_pitch_A1_rotor_encoder_reading / 9.1; // 9.1 is A1 gear ratio
  float elbow_pitch_A1_target = elbow_pitch_A1_rotor_encoder_reading / 9.1; // 9.1 is A1 gear ratio


  joint_state_t target = {0, 0, base_pitch_A1_target, elbow_pitch_A1_target, 0, 0, 0}; 
  joint_state_t last_target = {0,0,base_pitch_A1_target,elbow_pitch_A1_target,0,0,0};

  // Convert Encoder Value to A1 relative encoder reading. 

  const float A1_MAX_DELTA = float(1.0) / 1000 * (12) ; // convert max velocity to maximum delta position for every loop, in radians, 12ms is time per loop (measured)
  // {not used, base_yaw, base_pitch, forearm_pitch}
  const float A1_MAX_POS[] = {0, PI/2, PI/2, PI/2}; // maximum position for every joint
  const float A1_MIN_POS[] = {0, -PI/2, -PI/2, -PI/2}; // minimum position for every joint
  int last_sbus_values[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // for filtering
  UNUSED(A1_MIN_POS);
  UNUSED(A1_MAX_POS);

  int loop_cnt = 0;
  uint32_t last = HAL_GetTick();

  while (true) {
  
// #ifndef SINGLE_BOARD
//     send->cmd.id = bsp::VX;
//     send->cmd.data_float = 0; // sbus->ch[0]
//     send->TransmitOutput();

//     send->cmd.id = bsp::VY;
//     send->cmd.data_float = 0; // sbus->ch[1]
//     send->TransmitOutput();

//     send->cmd.id = bsp::RELATIVE_ANGLE;
//     send->cmd.data_float = 0; // sbus->ch[2]
//     send->TransmitOutput();
// #endif
//
    // pump->Off();
    // if (sbus->ch[10] > 0.5) {
      // pump->On();
    // }
//     ArmPrintData();
//     forearm_rotate_motor->connection_flag_ = false;
//     wrist_rotate_motor->connection_flag_ = false;
//     hand_rotate_motor->connection_flag_ = false;
  
    float temp[16];

    UNUSED(temp);
    // read and filter sbus
    // TODO: moving average filter
    for(int i = 0; i < 16; i++){
      if(sbus->ch[i] > SBUS_CHANNEL_MAX || sbus->ch[i] < -SBUS_CHANNEL_MAX){
        temp[i] = last_sbus_values[i];
      } else {
        temp[i] = sbus->ch[i];
        last_sbus_values[i] = sbus->ch[i];
      }
    }


    // map sbus input to A1 range
    target.base_yaw_rotate_1 =   temp[1]/SBUS_CHANNEL_MAX * (A1_MAX_POS[1]-A1_MIN_POS[1])/2 + A1_MIN_POS[1] + (A1_MAX_POS[1] - A1_MIN_POS[1])/2; 
    target.base_pitch_rotate_2 = temp[2]/SBUS_CHANNEL_MAX * (A1_MAX_POS[2]-A1_MIN_POS[2])/2 + A1_MIN_POS[2] + (A1_MAX_POS[2] - A1_MIN_POS[2])/2;
    target.forearm_pitch_3 =     temp[3]/SBUS_CHANNEL_MAX * (A1_MAX_POS[3]-A1_MIN_POS[3])/2 + A1_MIN_POS[3] + (A1_MAX_POS[3] - A1_MIN_POS[3])/2;
    print("Mapped Target: %f %f %f\r\n", target.base_yaw_rotate_1, target.base_pitch_rotate_2, target.forearm_pitch_3);


    // clip between last_target_pose +- MAX_DELTA
    target.base_yaw_rotate_1 =   clip<float>(target.base_yaw_rotate_1,   last_target.base_yaw_rotate_1   - A1_MAX_DELTA, last_target.base_yaw_rotate_1   + A1_MAX_DELTA); 
    target.base_pitch_rotate_2 = clip<float>(target.base_pitch_rotate_2, last_target.base_pitch_rotate_2 - A1_MAX_DELTA, last_target.base_pitch_rotate_2 + A1_MAX_DELTA);
    target.forearm_pitch_3 =     clip<float>(target.forearm_pitch_3,     last_target.forearm_pitch_3     - A1_MAX_DELTA, last_target.forearm_pitch_3     + A1_MAX_DELTA);

    // print("Clipped Target: %f %f %f\r\n", target.base_yaw_rotate_2, target.base_pitch_rotate_3, target.forearm_pitch_4);

    last_target.base_yaw_rotate_1   = target.base_yaw_rotate_1;
    last_target.base_pitch_rotate_2 = target.base_pitch_rotate_2;
    last_target.forearm_pitch_3     = target.forearm_pitch_3;
  
    // target.forearm_roll_5 = (sbus->ch[3] - FOREARM_ROTATE_OFFSET) / sbus->CHANNEL_MAX * PI;
    // target.wrist_6 = (sbus->ch[4] - WRIST_ROTATE_OFFSET) / sbus->CHANNEL_MAX * PI;
    // target.end_7 = (sbus->ch[5] - HAND_ROTATE_OFFSET) / sbus->CHANNEL_MAX * PI;
    // target.base_pitch_rotate_3 =  ((sbus->ch[2] - FOREARM_ROTATE_OFFSET) / 2048.0) * PI;
    // target.base_yaw_rotate_2 = ((sbus->ch[1] - BASE_VERT_ENCODER_OFFSET) / 2048.0) * PI;
    // target.forearm_pitch_4 = -((sbus->ch[0] - ELBOW_ENCODER_OFFSET) / 2048.0) * PI;
    //    print("dbus: %f\r\n", target.base_hor_rotate);
    ArmTurnAbsolute(target);
//        print("Target : %f %f %f %f %f %f %f\r\n", target.base_translate, target.base_vert_rotate, target.base_hor_rotate,
//              target.elbow_rotate, target.forearm_rotate, target.wrist_rotate, target.hand_rotate);
//        print("Current: %f %f %f %f %f %f %f\r\n",
//              current_joint_state.base_translate, current_joint_state.base_vert_rotate, current_joint_state.base_hor_rotate,
//              current_joint_state.elbow_rotate, current_joint_state.forearm_rotate, current_joint_state.wrist_rotate,
//              current_joint_state.hand_rotate);
    // A1->connection_flag_[0] = false;
    // A1->connection_flag_[1] = false;
    // A1->connection_flag_[2] = false;
    // A1->Stop(0);
    // osDelay(A1_CONTROL_DELAY);
    // A1->Stop(1);
    // osDelay(A1_CONTROL_DELAY);
    // A1->Stop(2);
    // osDelay(A1_CONTROL_DELAY);
    // forearm_rotate_motor->SetOutput(0,M4310_VEL,0.1,0.1,0);
    // HAND ROTATE OFFSET : 402
    // WRIST ROTATE OFFSET : 756
    //FOREARM ROTATE OFFSET :1023
    //FPREARM OFFSET :-967
    //GREATARM OFFSET : 1023
    //BASE OFFSET: 320


    // update A1 rotor encoder reading
    base_pitch_A1_rotor_encoder_reading = hard_wrap<float>((-encoder1->angle_ + BASE_PITCH_A1_ZERO_ENCODER_VAL)*9.1, 0, 2*PI);
    elbow_pitch_A1_rotor_encoder_reading = hard_wrap<float>((-encoder0->angle_ + ELBOW_PITCH_A1_ZERO_ENCODER_VAL)*9.1, 0, 2*PI);
    // base_pitch_A1_target = base_pitch_A1_rotor_encoder_reading / 9.1; // 9.1 is A1 gear ratio
    // elbow_pitch_A1_target = elbow_pitch_A1_rotor_encoder_reading / 9.1; // 9.1 is A1 gear ratio


    if(loop_cnt % 100 == 0){
      set_cursor(0, 0);
      clear_screen();
      print("Target( J1:%.3f J2:%.3f J3:%3f )\r\n", target.base_yaw_rotate_1, target.base_pitch_rotate_2, target.forearm_pitch_3);
      // print("Current( J1:%.3f J2:%.3f J3:%.3f) \r\n ", current_joint_state.base_yaw_rotate_2,current_joint_state.base_pitch_rotate_3,current_joint_state.forearm_pitch_4);
      // // print("FLag : %s\r\n", hand_rotate_motor->connection_flag_==true ? "true" : "false");
      // print("CH0: %-4d CH1: %-4d CH2: %-4d CH3: %-4d \r\n", sbus->ch[0], sbus->ch[1], sbus->ch[2], sbus->ch[3]);
      // print("CH4: %-4d CH5: %-4d CH6: %-4d CH7: %-4d \r\n", sbus->ch[4], sbus->ch[5], sbus->ch[6], sbus->ch[7]);
      // print("CH8: %-4d CH9: %-4d CH10: %-4d CH11: %-4d \r\n", sbus->ch[8], sbus->ch[9], sbus->ch[10], sbus->ch[11]);
      // print("CH12: %-4d CH13: %-4d CH14: %-4d CH15: %-4d \r\n", sbus->ch[12], sbus->ch[13], sbus->ch[14], sbus->ch[15]);
      // print("ELBOW: %03f HOR: %03f\r\n" ,encoder0->angle_, encoder1->angle_);
      // print("Zero[0] %03f Zer0[1] %03f Zero[2] %03f \r\n", A1_zero[0], A1_zero[1], A1_zero[2]);    
      print("A1 Init Target: Base%.3f Elbow%.3f\r\n", base_pitch_A1_target, elbow_pitch_A1_target);
      print("Encoder Positions: %.6f %.6f\r\n", encoder0->angle_, encoder1->angle_);
      print("Delta T : %d\r\n", HAL_GetTick()- last); 

    }

    last = HAL_GetTick();
    ArmTransmitOutput();
    osDelay(ARM_TASK_DELAY);
  }
}

int ArmTurnRelative(joint_state_t target) {
  joint_state_t abs_target = joint_state_t();
  abs_target.base_translate_0 = current_joint_state.base_translate_0 + target.base_translate_0;
  abs_target.base_yaw_rotate_1 = current_joint_state.base_yaw_rotate_1 + target.base_yaw_rotate_1;
  abs_target.base_pitch_rotate_2 = current_joint_state.base_pitch_rotate_2 + target.base_pitch_rotate_2;
  abs_target.forearm_pitch_3 = current_joint_state.forearm_pitch_3 + target.forearm_pitch_3;
  abs_target.forearm_roll_4 = current_joint_state.forearm_roll_4 + target.forearm_roll_4;
  abs_target.wrist_5 = current_joint_state.wrist_5 + target.wrist_5;
  abs_target.end_6 = current_joint_state.end_6 + target.end_6;
  return ArmTurnAbsolute(abs_target);
}

static const float A1_Kp = 0.05, A1_Kd = 1.0;
static const float m4310_Kp= 3, m4310_Kd = 1;
int ArmTurnAbsolute(joint_state_t target) {
  // M3508
//   if (target->base_translate >= BASE_TRANSLATE_MAX) {
//     base_translate_motor->TurnRelative(BASE_TRANSLATE_MAX-current_joint_state.base_translate);
//   } else if (target->base_translate <= BASE_TRANSLATE_MIN) {
//     base_translate_motor->TurnRelative(BASE_TRANSLATE_MIN-current_joint_state.base_translate);
//   } else {
//     base_translate_motor->TurnRelative(target->base_translate-current_joint_state.base_translate);
//   }
//   current_joint_state.base_translate = target->base_translate;

  // A1
   current_joint_state.base_yaw_rotate_1 = target.base_yaw_rotate_1;
   A1->Control(BASE_YAW_ID,   0.0, 0.0, current_joint_state.base_yaw_rotate_1  +A1_zero[0], A1_Kp, A1_Kd);

   current_joint_state.base_pitch_rotate_2 = target.base_pitch_rotate_2;
   A1->Control(BASE_PITCH_ID, 0.0, 0.0, current_joint_state.base_pitch_rotate_2+A1_zero[1], A1_Kp, A1_Kd);

   current_joint_state.forearm_pitch_3 = target.forearm_pitch_3;
   A1->Control(ELBOW_PITCH_ID,0.0, 0.0, current_joint_state.forearm_pitch_3    +A1_zero[2], A1_Kp, A1_Kd);

  // M4310
  //TODO: Adjust VALUE For TESTING
 if (target.forearm_roll_4 > FOREARM_ROTATE_MAX) {
   forearm_rotate_motor->SetOutput(FOREARM_ROTATE_MAX, 0, m4310_Kp, m4310_Kd, 0);
   target.forearm_roll_4 = FOREARM_ROTATE_MAX;
 } else if (target.forearm_roll_4 < FOREARM_ROTATE_MIN) {
   forearm_rotate_motor->SetOutput(FOREARM_ROTATE_MIN, 0, m4310_Kp, m4310_Kd, 0);
   target.forearm_roll_4 = FOREARM_ROTATE_MIN;
 } else
   forearm_rotate_motor->SetOutput(target.forearm_roll_4, 0, m4310_Kp, m4310_Kd, 0);
 current_joint_state.forearm_roll_4 = target.forearm_roll_4;

 if (target.wrist_5 > WRIST_ROTATE_MAX) {
   wrist_rotate_motor->SetOutput(WRIST_ROTATE_MAX, 0, m4310_Kp, m4310_Kd, 0);
   target.wrist_5 = WRIST_ROTATE_MAX;
 } else if (target.wrist_5 < WRIST_ROTATE_MIN) {
   wrist_rotate_motor->SetOutput(WRIST_ROTATE_MIN, 0, m4310_Kp, m4310_Kd, 0);
   target.wrist_5 = WRIST_ROTATE_MIN;
 } else
   wrist_rotate_motor->SetOutput(target.wrist_5, 0, m4310_Kp, m4310_Kd, 0);
 current_joint_state.wrist_5 = target.wrist_5;

 if (target.end_6 > HAND_ROTATE_MAX) {
   hand_rotate_motor->SetOutput(HAND_ROTATE_MAX, 0, m4310_Kp, m4310_Kd, 0);
   target.end_6 = HAND_ROTATE_MAX;
 } else if (target.end_6 < HAND_ROTATE_MIN) {
   hand_rotate_motor->SetOutput(HAND_ROTATE_MIN, 0, m4310_Kp, m4310_Kd, 0);
   target.end_6 = HAND_ROTATE_MIN;
 } else
   hand_rotate_motor->SetOutput(target.end_6, 0, m4310_Kp, m4310_Kd, 0);
 current_joint_state.end_6 = target.end_6;

  return 0;
}

void ArmTransmitOutput() {
//  control::MotorCANBase::TransmitOutput(m3508s, 1);

   A1_uart->Write((uint8_t*)(&A1->send[0].data), A1->send_length);
   osDelay(A1_CONTROL_DELAY);
   A1_uart->Write((uint8_t*)(&A1->send[1].data), A1->send_length);
   osDelay(A1_CONTROL_DELAY);
   A1_uart->Write((uint8_t*)(&A1->send[2].data), A1->send_length);
   osDelay(A1_CONTROL_DELAY);

  //  control::Motor4310* forearm_motors[3] = {forearm_rotate_motor, wrist_rotate_motor, hand_rotate_motor};
  //  control::Motor4310::TransmitOutput(forearm_motors, 3);
}

void ArmPrintData() {
//   print("Base Translate   : %10.4f\r\n", current_joint_state.base_translate);
//   print("Base Vert Rotate : %10.4f\r\n", current_joint_state.base_vert_rotate);
//   print("Base Hor  Rotate : %10.4f\r\n", current_joint_state.base_hor_rotate);
//   print("Elbow Rotate     : %10.4f\r\n", current_joint_state.elbow_rotate);
//   print("Forearm Rotate   : %10.4f\r\n", current_joint_state.forearm_rotate);
//   print("Wrist Rotate     : %10.4f\r\n", current_joint_state.wrist_rotate);
//   print("Hand Rotate      : %10.4f\r\n", current_joint_state.hand_rotate);
   set_cursor(0, 0);
   clear_screen();
   print("ID 0, flag: %s, Pos %04f\r\n", A1->connection_flag_[0] ? "true" : "false", A1->recv[0].Pos);
   print("ID 1, flag: %s, Pos %04f\r\n", A1->connection_flag_[1] ? "true" : "false", A1->recv[1].Pos);
   print("ID 2, flag: %s, Pos %04f\r\n", A1->connection_flag_[2] ? "true" : "false", A1->recv[2].Pos);
//  print("Forearm POS: %04f\r\n",forearm_rotate_motor->GetTheta());
//  print("Wrist POS: %04f\r\n",wrist_rotate_motor->GetTheta());
//  print("Hand POS: %04f\r\n",hand_rotate_motor->GetTheta());
}
