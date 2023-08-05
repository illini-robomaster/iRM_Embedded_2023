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
static float A1_zero[3] = {0, 0, 0};
static bsp::Relay* pump = nullptr;
#ifndef SINGLE_BOARD
static bsp::CAN* can2 = nullptr;
static bsp::CanBridge* send = nullptr;
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
  print_use_uart(&huart8);
  bsp::SetHighresClockTimer(&htim5);
//   control::steering_t steering_data;
  can1 = new bsp::CAN(&hcan1);
  sbus = new remote::SBUS(&huart1);
  encoder0 = new control::BRTEncoder(can1,0x0A);
  encoder1 = new control::BRTEncoder(can1,0x01);
  pump = new bsp::Relay(K2_GPIO_Port,K2_Pin);

#ifndef SINGLE_BOARD
  can2 = new bsp::CAN(&hcan2, false);
  send = new bsp::CanBridge(can2,0x20A, 0x20B);
#endif
  // Init m3508 * 1
//   motor1 = new control::Motor3508(can1, BASE_TRANSLATE_ID);
//   base_translate_pe_sensor = new bsp::GPIO(BASE_TRANSLATE_CALI_GPIO_PORT,
//                                            BASE_TRANSLATE_CALI_GPIO_PIN);
//   steering_data.motor = motor1;
//   steering_data.max_speed = RUN_SPEED;
//   steering_data.max_acceleration = ACCELERATION;
//   // TODO make sure the gear ratio is correct
//   steering_data.transmission_ratio = M3508P19_RATIO;
//   steering_data.omega_pid_param = new float[3]{140, 1.2, 25};
//   steering_data.max_iout = 1000;
//   steering_data.max_out = 13000;
//   // TODO measure the calibrate offset for base translate motor
//   steering_data.calibrate_offset = 0;
//   steering_data.align_detect_func = base_translate_align_detect;
//   base_translate_motor = new control::SteeringMotor(steering_data);

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
   A1->Stop(1);
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
  osDelay(3000);
//   base_translate_motor->SetMaxSpeed(ALIGN_SPEED)

  // base translate calibration
//   bool base_translate_alignment_complete = false;
//   while (!base_translate_alignment_complete) {
//     base_translate_motor->CalcOutput();
//     control::MotorCANBase::TransmitOutput(m3508s, 1);
//     base_translate_alignment_complete = base_translate_motor->Calibrate();
//     osDelay(2);
//   }

//   base_translate_motor->ReAlign();
//   //base_translate_motor->SetMaxSpeed(RUN_SPEED);
//   control::MotorCANBase::TransmitOutput(m3508s, 1);

//   4310 init state
  // TODO: config zero pos with 4310 config assist

// forearm_rotate_motor->SetZeroPos();
// osDelay(20);
   forearm_rotate_motor->MotorEnable();
// osDelay(20);
// wrist_rotate_motor->SetZeroPos();
// osDelay(20);
   wrist_rotate_motor->MotorEnable();
// osDelay(20);
// hand_rotate_motor->SetZeroPos();
// osDelay(20);
   hand_rotate_motor->MotorEnable();

  // A1 init state
   A1->Stop(0);
   A1_uart->Write((uint8_t*)(&A1->send[0].data), A1->send_length);
   osDelay(A1_CONTROL_DELAY);
   A1->Stop(1);
   A1_uart->Write((uint8_t*)(&A1->send[1].data), A1->send_length);
   osDelay(A1_CONTROL_DELAY);
   A1->Stop(2);
   A1_uart->Write((uint8_t*)(&A1->send[2].data), A1->send_length);
   osDelay(A1_CONTROL_DELAY);
   A1_zero[0] = A1->recv[0].Pos+ELBOW_ENCODER_OFFSET-encoder0->angle_;
   A1_zero[1] = A1->recv[1].Pos+BASE_VERT_ENCODER_OFFSET-encoder1->angle_;
   A1_zero[2] = A1->recv[2].Pos;

//   joint_state_t target = {PI / 16, PI / 16, PI / 16, PI / 16, PI / 16, PI / 16, PI / 16};
  joint_state_t target = {0, 0, 0, 0, 0, 0, 0};
UNUSED(target);
//   turn each motor for PI/16 degree every 2 seconds.
//   Should stop at defined limitation, see arm_config.h
  //MOTOR 0-5
  //6lv  7lh  8rv
  //9 base
  while (true) {
#ifndef SINGLE_BOARD
    send->cmd.id = bsp::VX;
    send->cmd.data_float = sbus->ch[7];
    send->TransmitOutput();

    send->cmd.id = bsp::VY;
    send->cmd.data_float = sbus->ch[6];
    send->TransmitOutput();

    send->cmd.id = bsp::RELATIVE_ANGLE;
    send->cmd.data_float = sbus->ch[8];
    send->TransmitOutput();
#endif
//
    pump->Off();
    if (sbus->ch[10] > 0.5) {
      pump->On();
    }
    ArmPrintData();
    forearm_rotate_motor->connection_flag_ = false;
    wrist_rotate_motor->connection_flag_ = false;
    hand_rotate_motor->connection_flag_ = false;
//    UNUSED(A1_zero);

    target.forearm_rotate = (sbus->ch[3] - FOREARM_ROTATE_OFFSET) / sbus->CHANNEL_MAX * PI;
    target.wrist_rotate = (sbus->ch[4] - WRIST_ROTATE_OFFSET) / sbus->CHANNEL_MAX * PI;
    target.hand_rotate = (sbus->ch[5] - HAND_ROTATE_OFFSET) / sbus->CHANNEL_MAX * PI;
    target.base_hor_rotate =  ((sbus->ch[2] - FOREARM_ROTATE_OFFSET) / 2048.0) * PI;
    target.base_vert_rotate = ((sbus->ch[1] - BASE_VERT_ENCODER_OFFSET) / 2048.0) * PI;
    target.elbow_rotate = -((sbus->ch[0] - ELBOW_ENCODER_OFFSET) / 2048.0) * PI;
    //    print("dbus: %f\r\n", target.base_hor_rotate);
    ArmTurnAbsolute(target);
//        print("Target : %f %f %f %f %f %f %f\r\n", target.base_translate, target.base_vert_rotate, target.base_hor_rotate,
//              target.elbow_rotate, target.forearm_rotate, target.wrist_rotate, target.hand_rotate);
//        print("Current: %f %f %f %f %f %f %f\r\n",
//              current_joint_state.base_translate, current_joint_state.base_vert_rotate, current_joint_state.base_hor_rotate,
//              current_joint_state.elbow_rotate, current_joint_state.forearm_rotate, current_joint_state.wrist_rotate,
//              current_joint_state.hand_rotate);
    A1->connection_flag_[0] = false;
    A1->connection_flag_[1] = false;
    A1->connection_flag_[2] = false;
//    A1->Stop(0);
//    osDelay(A1_CONTROL_DELAY);
//    A1->Stop(1);
//    osDelay(A1_CONTROL_DELAY);
//    A1->Stop(2);
//    osDelay(A1_CONTROL_DELAY);
//        forearm_rotate_motor->SetOutput(0,M4310_VEL,0.1,0.1,0);
        // HAND ROTATE OFFSET : 402
        // WRIST ROTATE OFFSET : 756
        //FOREARM ROTATE OFFSET :1023
        //FPREARM OFFSET :-967
        //GREATARM OFFSET : 1023
        //BASE OFFSET: 320
//    set_cursor(0, 0);
//    clear_screen();
    print("Target( H:%.3f V:%.3f E:%.3f )", target.base_hor_rotate, target.base_vert_rotate, target.elbow_rotate);
    print("Current( H:%.3f W:%.3f F:%.3f\r\n )", current_joint_state.base_hor_rotate,current_joint_state.base_vert_rotate,current_joint_state.elbow_rotate);
    print("FLag : %s\r\n", hand_rotate_motor->connection_flag_==true ? "true" : "false");
    print("CH0: %-4d CH1: %-4d CH2: %-4d CH3: %-4d ", sbus->ch[0], sbus->ch[1], sbus->ch[2], sbus->ch[3]);
    print("CH4: %-4d CH5: %-4d CH6: %-4d CH7: %-4d ", sbus->ch[4], sbus->ch[5], sbus->ch[6], sbus->ch[7]);
    print("CH8: %-4d CH9: %-4d CH10: %-4d CH11: %-4d ", sbus->ch[8], sbus->ch[9], sbus->ch[10], sbus->ch[11]);
    print("CH12: %-4d CH13: %-4d CH14: %-4d CH15: %-4d ", sbus->ch[12], sbus->ch[13], sbus->ch[14], sbus->ch[15]);
//    print("\r\n");
    print("ELBOW: %03f HOR: %03f\r\n" ,encoder0->angle_, encoder1->angle_);
    print("Zero[0] %03f Zer0[1] %03f Zero[2] %03f", A1_zero[0], A1_zero[1], A1_zero[2]);
    ArmTransmitOutput();
    osDelay(100);
  }
}

int ArmTurnRelative(joint_state_t target) {
  joint_state_t abs_target = joint_state_t();
  abs_target.base_translate = current_joint_state.base_translate + target.base_translate;
  abs_target.base_vert_rotate = current_joint_state.base_vert_rotate + target.base_vert_rotate;
  abs_target.base_hor_rotate = current_joint_state.base_hor_rotate + target.base_hor_rotate;
  abs_target.elbow_rotate = current_joint_state.elbow_rotate + target.elbow_rotate;
  abs_target.forearm_rotate = current_joint_state.forearm_rotate + target.forearm_rotate;
  abs_target.wrist_rotate = current_joint_state.wrist_rotate + target.wrist_rotate;
  abs_target.hand_rotate = current_joint_state.hand_rotate + target.hand_rotate;
  return ArmTurnAbsolute(abs_target);
}

static const float A1_Kp = 0.002, A1_Kd = 0.03;
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

   if (target.base_vert_rotate > BASE_VERT_ROTATE_MAX)
     target.base_vert_rotate = BASE_VERT_ROTATE_MAX + A1_zero[1];
   else if (target.base_vert_rotate < BASE_VERT_ROTATE_MIN)
     target.base_vert_rotate = BASE_VERT_ROTATE_MIN + A1_zero[1];
   current_joint_state.base_vert_rotate = target.base_vert_rotate;
   A1->Control(BASE_VERT_ROTATE_ID, 0.0, 0.0, current_joint_state.base_vert_rotate+A1_zero[1],  A1_Kp, A1_Kd);

   if (target.base_hor_rotate > BASE_HOR_ROTATE_MAX)
     target.base_hor_rotate = BASE_HOR_ROTATE_MAX+ A1_zero[2];
   else if (target.base_hor_rotate < BASE_HOR_ROTATE_MIN)
     target.base_hor_rotate = BASE_HOR_ROTATE_MIN + A1_zero[2];
   current_joint_state.base_hor_rotate = target.base_hor_rotate;
   A1->Control(BASE_HOR_ROTATE_ID, 0.0, 0.0, current_joint_state.base_hor_rotate+A1_zero[2],  A1_Kp, A1_Kd);

   if (target.elbow_rotate > ELBOW_ROTATE_MAX)
     target.elbow_rotate = ELBOW_ROTATE_MAX + A1_zero[0];
   else if (target.elbow_rotate < ELBOW_ROTATE_MIN)
     target.elbow_rotate = ELBOW_ROTATE_MIN + A1_zero[0];
   current_joint_state.elbow_rotate = target.elbow_rotate;
   A1->Control(ELBOW_ROTATE_ID, 0.0, 0.0, current_joint_state.elbow_rotate+A1_zero[0],  A1_Kp, A1_Kd);

  // M4310
  //TODO: Adjust VALUE For TESTING
 if (target.forearm_rotate > FOREARM_ROTATE_MAX) {
   forearm_rotate_motor->SetOutput(FOREARM_ROTATE_MAX, 0, m4310_Kp, m4310_Kd, 0);
   target.forearm_rotate = FOREARM_ROTATE_MAX;
 } else if (target.forearm_rotate < FOREARM_ROTATE_MIN) {
   forearm_rotate_motor->SetOutput(FOREARM_ROTATE_MIN, 0, m4310_Kp, m4310_Kd, 0);
   target.forearm_rotate = FOREARM_ROTATE_MIN;
 } else
   forearm_rotate_motor->SetOutput(target.forearm_rotate, 0, m4310_Kp, m4310_Kd, 0);
 current_joint_state.forearm_rotate = target.forearm_rotate;

 if (target.wrist_rotate > WRIST_ROTATE_MAX) {
   wrist_rotate_motor->SetOutput(WRIST_ROTATE_MAX, 0, m4310_Kp, m4310_Kd, 0);
   target.wrist_rotate = WRIST_ROTATE_MAX;
 } else if (target.wrist_rotate < WRIST_ROTATE_MIN) {
   wrist_rotate_motor->SetOutput(WRIST_ROTATE_MIN, 0, m4310_Kp, m4310_Kd, 0);
   target.wrist_rotate = WRIST_ROTATE_MIN;
 } else
   wrist_rotate_motor->SetOutput(target.wrist_rotate, 0, m4310_Kp, m4310_Kd, 0);
 current_joint_state.wrist_rotate = target.wrist_rotate;

 if (target.hand_rotate > HAND_ROTATE_MAX) {
   hand_rotate_motor->SetOutput(HAND_ROTATE_MAX, 0, m4310_Kp, m4310_Kd, 0);
   target.hand_rotate = HAND_ROTATE_MAX;
 } else if (target.hand_rotate < HAND_ROTATE_MIN) {
   hand_rotate_motor->SetOutput(HAND_ROTATE_MIN, 0, m4310_Kp, m4310_Kd, 0);
   target.hand_rotate = HAND_ROTATE_MIN;
 } else
   hand_rotate_motor->SetOutput(target.hand_rotate, 0, m4310_Kp, m4310_Kd, 0);
 current_joint_state.hand_rotate = target.hand_rotate;

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

   control::Motor4310* forearm_motors[3] = {forearm_rotate_motor, wrist_rotate_motor, hand_rotate_motor};
   control::Motor4310::TransmitOutput(forearm_motors, 3);
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
