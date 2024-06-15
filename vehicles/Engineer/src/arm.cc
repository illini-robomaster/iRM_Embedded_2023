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

static const int ARM_TASK_DELAY = 10;



/**
 * forearm_rotate_motor     RX=0x02 TX=0x01
 * wrist_rotate_motor       RX=0x04 TX=0x03
 * hand_rotete_motor        RX=0x06 TX=0x05
 * 
 * elbow_rortate_motor      ID=0
 * upper_arm_motor          ID=1
 * base_motor               ID=2
*/


// for receving data from A1, currently not functioning
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

//  void A1Task(void* arg) {
//      UNUSED(arg);
//      uint32_t length;
//      uint8_t* data;

//      while (true) {
//          /* wait until rx data is available */
//          uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
//          if (flags & RX_SIGNAL) {  // unnecessary check
//              /* time the non-blocking rx / tx calls (should be <= 1 osTick) */
//              length = A1_uart->Read(&data);
//              if ((int)length == A1->recv_length)
//                A1->ExtractData(communication::package_t{data, (int)length});
//          }
//      }
//  }


// entire arm
static joint_state_t current_joint_state = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

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

//  void RM_RTOS_Threads_Init(void) {
//    A1TaskHandle = osThreadNew(A1Task, nullptr, &A1TaskAttribute);

//  }

void armA1Task(void* args) {
  UNUSED(args);
  //Add 1s Time for 4310 start up.

  print("START\r\n");
  osDelay(1000);

  const float SBUS_CHANNEL_MAX = 660;

  const float BASE_PITCH_FIELD_ZERO_ENCODER_VAL = -2.448; // the encoder value when the big arm is pointing upwards
  
  const float ELBOW_PITCH_FIELD_ZERO_ENCODER_DIFF_VAL = 0.8; // TODO: the difference in encoder value when the small arm is pointing upwards
 
  // When no power, A1 absolute rotor encoder can only store 0~2PI,
  // After powered on, A1 encoder will be able to record multiple turns.

  // ===========================================================================================================
  // CAUTION: If C board is powered but A1 lost power and moved. This will result in a wrong reading and A1 will suddenly move after powered on again.
  // ===========================================================================================================

  // Find A1 Rotor Encoder reading (0~2PI) in terms of A1 Rotor
  const float BASE_PITCH_A1_ZERO_ENCODER_VAL = -2.252; // the base encoder value when A1 absolute encoder reads 0.
  // 3.632, 2.939, 1.552, 2.2519

  const float ELBOW_PITCH_A1_ZERO_ENCODER_DIFF_VAL = 0.94;// elbow encoder value - base encoder value when A1 absolute encoder reads 0.
  // the A1 angle of the elbow is determined using the difference of two encoders
  // At one of the zero positions (set A1 to go to position = 0):
  // base encoder reads 2.718, elbow encoder reads 3.6631
  // or base encoder reads 2.3746, elbow encoder reads 3.3073
  // -> difference is 0.9327, 0.9451 -> round to 0.94

  // (+-) Current Encoder Reading = n* Encoder Delta Per A1 Rotor Turn + A1 Rotor Encoder Reading + Encoder Value When A1 Rotor Is Zero
  // -> A1 Rotor Encoder Reading = (+-) Current Encoder Reading - n* Encoder Delta Per A1 Rotor Turn - Encoder Value When A1 Rotor Is Zero

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

  float base_pitch_A1_rotor_encoder_reading = 0;
  float elbow_pitch_A1_rotor_encoder_reading = 0;
  float base_pitch_A1_init_target = 0;
  float elbow_pitch_A1_init_target = 0;
  // float J4_pos = 0.0;
  // float J5_pos = 0.0;
  // float J6_pos = 0.0;

  UNUSED(base_pitch_A1_rotor_encoder_reading);
  UNUSED(elbow_pitch_A1_rotor_encoder_reading);
  UNUSED(base_pitch_A1_init_target);
  UNUSED(elbow_pitch_A1_init_target); 

  // make sure A1 not at the edge by speed mode
  base_pitch_A1_rotor_encoder_reading = hard_wrap<float>((encoder1->getData() - BASE_PITCH_A1_ZERO_ENCODER_VAL)*A1->gear_ratio, 0, 2*PI);
  base_pitch_A1_init_target = base_pitch_A1_rotor_encoder_reading / A1->gear_ratio; // A1->gear_ratio is A1 gear ratio  

  // safety
  while (base_pitch_A1_init_target < 0.05 || base_pitch_A1_init_target > 2*PI/A1->gear_ratio-0.05) {
    osDelay(100);
    print("base pitch safety\r\n");
  }
  
  elbow_pitch_A1_rotor_encoder_reading = hard_wrap<float>((encoder0->getData() + encoder1->getData() - ELBOW_PITCH_A1_ZERO_ENCODER_DIFF_VAL)*A1->gear_ratio, 0, 2*PI);
  elbow_pitch_A1_init_target = elbow_pitch_A1_rotor_encoder_reading / A1->gear_ratio; // A1->gear_ratio is A1 gear ratio
  const float base_pitch_a1_minus_encoder = base_pitch_A1_init_target - (encoder1->getData()); // external encoder have different direction
  const float elbow_pitch_a1_minus_encoder = elbow_pitch_A1_init_target - (encoder0->getData());
  
  
  while (elbow_pitch_A1_init_target < 0.05 || elbow_pitch_A1_init_target > 2*PI/A1->gear_ratio-0.05) {
    osDelay(100);
    print("elbow safety\r\n");
  }

  // The difference between the a1 encoder and external encoder is now fixed (powered on)
 

  joint_state_t target = {0, 0, base_pitch_A1_init_target, elbow_pitch_A1_init_target, 0, 0, 0}; 
  joint_state_t last_target = {0,0,base_pitch_A1_init_target,elbow_pitch_A1_init_target,0,0,0};
  // joint_state_t target = {0, 0, 0, 0, 0, 0, 0};
  // joint_state_t last_target = {0,0,0,0,0,0,0};


  // Convert Encoder Value to A1 relative encoder reading. 
  
  const float A1_MAX_DELTA = float(1.0) / 1000 * (12) ; // convert max velocity to maximum delta position for every loop, in radians, 12ms is time per loop (measured)
  // {not used, base_yaw, base_pitch, forearm_pitch}
  // const float A1_MAX_POS[] = {0, PI/2, PI/2, PI/2}; // maximum position for every joint
  // const float A1_MIN_POS[] = {0, -PI/2, -PI/2, -PI/2}; // minimum position for every joint

  joint_state_t MAX_POS = {0, PI/2, PI/4, PI/8, PI/2, PI/2, PI/2};
  joint_state_t MIN_POS = {0, -PI/2, -PI/4, -PI/8, -PI/2, -PI/2, -PI/2};

  int last_sbus_values[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // for filtering

  int loop_cnt = 0;
  // uint32_t last = HAL_GetTick();

  MovingAverage moving_average[7];

  

  while (true) {
    // pump->Off();
    // if (sbus->ch[10] > 0.5) {
      // pump->On();
    // }
  
    float temp[16];

    UNUSED(last_sbus_values);
    // read and filter sbus
    // TODO: moving average filter
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


    // field_angles.base_yaw_rotate_1 =   temp[1]/SBUS_CHANNEL_MAX * (MAX_POS.base_yaw_rotate_1-MIN_POS.base_yaw_rotate_1)/2 + MIN_POS.base_yaw_rotate_1 + (MAX_POS.base_yaw_rotate_1 - MIN_POS.base_yaw_rotate_1)/2; 
    // field_angles.base_pitch_rotate_2 = temp[2]/SBUS_CHANNEL_MAX * (MAX_POS.base_pitch_rotate_2-MIN_POS.base_pitch_rotate_2)/2 + MIN_POS.base_pitch_rotate_2 + (MAX_POS.base_pitch_rotate_2 - MIN_POS.base_pitch_rotate_2)/2;
    // field_angles.forearm_pitch_3 =     temp[3]/SBUS_CHANNEL_MAX * (MAX_POS.forearm_pitch_3-MIN_POS.forearm_pitch_3)/2 + MIN_POS.forearm_pitch_3 + (MAX_POS.forearm_pitch_3 - MIN_POS.forearm_pitch_3)/2;
    // print("Mapped Target: %f %f %f\r\n", target.base_yaw_rotate_1, target.base_pitch_rotate_2, target.forearm_pitch_3)

    // Convert field angle to Encoder value
    // Encoder value = Field Angle + Encoder value when big arm is pointing upward
    float target_encoder[4]; 
    UNUSED(BASE_PITCH_FIELD_ZERO_ENCODER_VAL);
    target_encoder[1] = field_angles.base_yaw_rotate_1; // no encoder, so assume original position is 0.
    target_encoder[2] = field_angles.base_pitch_rotate_2 + BASE_PITCH_FIELD_ZERO_ENCODER_VAL;
    target_encoder[3] = field_angles.forearm_pitch_3 + ELBOW_PITCH_FIELD_ZERO_ENCODER_DIFF_VAL;

    // Convert Encoder Value to A1 .
    target.base_yaw_rotate_1 = target_encoder[1];
    target.base_pitch_rotate_2 = base_pitch_a1_minus_encoder + target_encoder[2];
    target.forearm_pitch_3 = elbow_pitch_a1_minus_encoder + target_encoder[3];
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
    // print("Clipped Target: %f %f %f\r\n", target.base_yaw_rotate_2, target.base_pitch_rotate_3, target.forearm_pitch_4);

    last_target.base_yaw_rotate_1   = target.base_yaw_rotate_1;
    last_target.base_pitch_rotate_2 = target.base_pitch_rotate_2;
    last_target.forearm_pitch_3     = target.forearm_pitch_3;
    last_target.forearm_roll_4      = target.forearm_roll_4;
    last_target.wrist_5             = target.wrist_5;
    last_target.end_6               = target.end_6;

    // target.forearm_roll_5 = (sbus->ch[3] - FOREARM_ROTATE_OFFSET) / sbus->CHANNEL_MAX * PI;
    // target.wrist_6 = (sbus->ch[4] - WRIST_ROTATE_OFFSET) / sbus->CHANNEL_MAX * PI;
    // target.end_7 = (sbus->ch[5] - HAND_ROTATE_OFFSET) / sbus->CHANNEL_MAX * PI;
    // target.base_pitch_rotate_3 =  ((sbus->ch[2] - FOREARM_ROTATE_OFFSET) / 2048.0) * PI;
    // target.base_yaw_rotate_2 = ((sbus->ch[1] - BASE_VERT_ENCODER_OFFSET) / 2048.0) * PI;
    // target.forearm_pitch_4 = -((sbus->ch[0] - ELBOW_ENCODER_OFFSET) / 2048.0) * PI;
    //    print("dbus: %f\r\n", target.base_hor_rotate);

    // target.base_pitch_rotate_2 = base_pitch_A1_init_target;
    // target.base_pitch_rotate_2 = 0;

    // target.forearm_pitch_3 = 0;
    ArmA1TurnAbsolute(target);
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
    // base_pitch_A1_rotor_encoder_reading = hard_wrap<float>((-encoder1->angle_ + BASE_PITCH_A1_ZERO_ENCODER_VAL)*A1->gear_ratio, 0, 2*PI);
    // elbow_pitch_A1_rotor_encoder_reading = hard_wrap<float>((-encoder0->angle_ + ELBOW_PITCH_A1_ZERO_ENCODER_VAL)*A1->gear_ratio, 0, 2*PI);
    // base_pitch_A1_init_target = base_pitch_A1_rotor_encoder_reading / A1->gear_ratio; // A1->gear_ratio is A1 gear ratio
    // elbow_pitch_A1_init_target = elbow_pitch_A1_rotor_encoder_reading / A1->gear_ratio; // A1->gear_ratio is A1 gear ratio


    if(loop_cnt % 100 == 0){
      set_cursor(0, 0);
      clear_screen();
      // print("Field Angle: J1%.3f J2%.3f J3%.3f\r\n", field_angles.base_yaw_rotate_1, field_angles.base_pitch_rotate_2, field_angles.forearm_pitch_3);
      print("Target( J1:%.3f J2:%.3f J3:%3f )\r\n", target.base_yaw_rotate_1, target.base_pitch_rotate_2, target.forearm_pitch_3);
      print("J4 %.3f J5 %.3f J6 %.3f\r\n", target.forearm_roll_4, target.wrist_5, target.end_6);
      // print("Target Encoder( J1:%.3f J2:%.3f J3:%3f )\r\n", target_encoder[1], target_encoder[2], target_encoder[3]);
      // print("Current( J1:%.3f J2:%.3f J3:%.3f) \r\n ", current_joint_state.base_yaw_rotate_2,current_joint_state.base_pitch_rotate_3,current_joint_state.forearm_pitch_4);
      // // print("FLag : %s\r\n", hand_rotate_motor->connection_flag_==true ? "true" : "false");
      // print("CH0: %-4d CH1: %-4d CH2: %-4d CH3: %-4d \r\n", dbus->ch[0], sbus->ch[1], sbus->ch[2], sbus->ch[3]);
      // print("CH4: %-4d CH5: %-4d CH6: %-4d CH7: %-4d \r\n", sbus->ch[4], sbus->ch[5], sbus->ch[6], sbus->ch[7]);
      // print("CH8: %-4d CH9: %-4d CH10: %-4d CH11: %-4d \r\n", sbus->ch[8], sbus->ch[9], sbus->ch[10], sbus->ch[11]);
      // print("CH12: %-4d CH13: %-4d CH14: %-4d CH15: %-4d \r\n", sbus->ch[12], sbus->ch[13], sbus->ch[14], sbus->ch[15]);
      // print("ELBOW: %03f HOR: %03f\r\n" ,encoder0->angle_, encoder1->angle_);
      // print("Zero[0] %03f Zer0[1] %03f Zero[2] %03f \r\n", A1_zero[0], A1_zero[1], A1_zero[2]);    
      print("Diff A1: %.3f %.3f\r\n", base_pitch_a1_minus_encoder, elbow_pitch_a1_minus_encoder);
      print("A1 Init Target: Base%.3f Elbow%.3f\r\n", base_pitch_A1_init_target, elbow_pitch_A1_init_target);
      print("Encoder Positions: %.6f %.6f\r\n", encoder0->getData()+encoder1->getData(), encoder1->getData());
      // print("Delta T : %d\r\n", HAL_GetTick()- last); 
      print("HAL Get Tick: %d\r\n", HAL_GetTick());
      // print("data: %f %f %f %f %f\r\n", target.base_pitch_rotate_2, target_encoder[2], base_pitch_A1_init_target, base_pitch_a1_minus_encoder, encoder1->getData());

    }

    // last = HAL_GetTick();
    ArmTransmitOutput();
    osDelay(ARM_TASK_DELAY);
  }
}

static const float A1_Kp = 0.05, A1_Kd = 1.0;
static const float m4310_Kp= 3, m4310_Kd = 1;
int ArmA1TurnAbsolute(joint_state_t target) {
  // A1
   current_joint_state.base_yaw_rotate_1 = target.base_yaw_rotate_1;
   A1->Control(BASE_YAW_ID,   0.0, 0.0, current_joint_state.base_yaw_rotate_1  +A1_zero[0], A1_Kp, A1_Kd);

   current_joint_state.base_pitch_rotate_2 = target.base_pitch_rotate_2;
   A1->Control(BASE_PITCH_ID, 0.0, 0.0, current_joint_state.base_pitch_rotate_2+A1_zero[1], A1_Kp, A1_Kd);

   current_joint_state.forearm_pitch_3 = target.forearm_pitch_3;
   A1->Control(ELBOW_PITCH_ID,0.0, 0.0, current_joint_state.forearm_pitch_3    +A1_zero[2], A1_Kp, A1_Kd);

  // 4310
  forearm_rotate_motor_4->SetOutput(target.forearm_roll_4, target.forearm_roll_4, 10, 0.5, 0);
  wrist_rotate_motor_5->SetOutput(target.wrist_5, target.wrist_5, 10, 0.5, 0);
  hand_rotate_motor_6->SetOutput(target.end_6, target.end_6, 10, 0.5, 0);

  return 0;
}

void kill_arm_A1() {
  A1->Stop(0);
  A1->Stop(1);
  A1->Stop(2);
  A1_uart->Write((uint8_t*)(&A1->send[0].data), A1->send_length);
}

void ArmTransmitOutput() {
//  control::MotorCANBase::TransmitOutput(m3508s, 1);

   A1_uart->Write((uint8_t*)(&A1->send[0].data), A1->send_length);
   osDelay(A1_CONTROL_DELAY);
   A1_uart->Write((uint8_t*)(&A1->send[1].data), A1->send_length);
   osDelay(A1_CONTROL_DELAY);
   A1_uart->Write((uint8_t*)(&A1->send[2].data), A1->send_length);
   osDelay(A1_CONTROL_DELAY);
  
   control::Motor4310* forearm_motors[3] = {forearm_rotate_motor_4, wrist_rotate_motor_5, hand_rotate_motor_6};
  control::Motor4310::TransmitOutput(forearm_motors, 3);
}

void ArmA1PrintData() {

}
