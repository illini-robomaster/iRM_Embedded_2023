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

#include "arm.h"
#include "arm_config.h"

static bsp::CAN* can = nullptr;

// 3508
//static control::MotorCANBase* motor1 = nullptr;
//static control::SteeringMotor* base_translate_motor = nullptr;

static bsp::GPIO* base_translate_pe_sensor = nullptr;
bool base_translate_align_detect() {
  return base_translate_pe_sensor->Read() == true;
}

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

static CustomUART* joint_uart = nullptr;

// 4310
static control::Motor4310* forearm_rotate_motor = nullptr;
static control::Motor4310* wrist_rotate_motor = nullptr;
static control::Motor4310* hand_rotate_motor = nullptr;

// entire arm
static joint_state_t current_joint_state = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// TODO fix transmit output for 4310 multi motors

// PACIFICRIM!
static remote::SBUS* sbus = nullptr;

void RM_RTOS_Init() {
  print_use_uart(&huart8);
  bsp::SetHighresClockTimer(&htim5);

//   control::steering_t steering_data;

  can = new bsp::CAN(&hcan1);

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
  forearm_rotate_motor = new control::Motor4310(
    can, FOREARM_ROTATE_RX_ID, FOREARM_ROTATE_TX_ID, control::MIT);
  wrist_rotate_motor = new control::Motor4310(
    can, WRIST_ROTATE_RX_ID, WRIST_ROTATE_TX_ID, control::MIT);
  hand_rotate_motor = new control::Motor4310(
    can, HAND_ROTATE_RX_ID, HAND_ROTATE_TX_ID, control::MIT);

  // Init A1 * 3
  joint_uart = new CustomUART(A1_UART);
  joint_uart->SetupRx(300);
  joint_uart->SetupTx(300);

  A1 = new control::UnitreeMotor();

  sbus = new remote::SBUS(&huart1);
}

void RM_RTOS_Threads_Init(void) {
  A1TaskHandle = osThreadNew(A1Task, nullptr, &A1TaskAttribute);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
//   base_translate_motor->SetMaxSpeed(ALIGN_SPEED);

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

  forearm_rotate_motor->SetZeroPos();
  wrist_rotate_motor->SetZeroPos();
  hand_rotate_motor->SetZeroPos();
  forearm_rotate_motor->MotorEnable();
  wrist_rotate_motor->MotorEnable();
  hand_rotate_motor->MotorEnable();

  // A1 init state
  A1->Stop(0);
  joint_uart->Write((uint8_t*)(&(A1->send[0].data)), A1->send_length);
  A1->Stop(1);
  joint_uart->Write((uint8_t*)(&(A1->send[1].data)), A1->send_length);
  A1->Stop(2);
  joint_uart->Write((uint8_t*)(&(A1->send[2].data)), A1->send_length);

  print("arm starts in 2 secs\r\n");
  // wait for 2 seconds
  osDelay(2000);

//   joint_state_t target = {PI / 16, PI / 16, PI / 16, PI / 16, PI / 16, PI / 16, PI / 16};

//   int i = 0;
//   turn each motor for PI/16 degree every 2 seconds.
//   Should stop at defined limitation, see arm_config.h
  while (true) {
   ArmPrintData();
    osDelay(100);
    // if (i <= 1000) {
    //   i++;
    // } else {
    //   ArmTurnRelative(&target);
    //   ArmPrintData();
    // }
    // ArmTransmitOutput();
    // osDelay(2);
  }
} // end default task

int ArmTurnRelative(joint_state_t* target) {
  joint_state_t* abs_target = new joint_state_t();
  abs_target->base_translate = current_joint_state.base_translate + target->base_translate;
  abs_target->base_vert_rotate = current_joint_state.base_vert_rotate + target->base_vert_rotate;
  abs_target->base_hor_rotate = current_joint_state.base_hor_rotate + target->base_hor_rotate;
  abs_target->elbow_rotate = current_joint_state.elbow_rotate + target->elbow_rotate;
  abs_target->forearm_rotate = current_joint_state.forearm_rotate + target->forearm_rotate;
  abs_target->wrist_rotate = current_joint_state.wrist_rotate + target->wrist_rotate;
  abs_target->hand_rotate = current_joint_state.hand_rotate + target->hand_rotate;

  return ArmTurnAbsolute(abs_target);
}

static const float A1_Kp = 0.001, A1_Kd = 0.001;
static const float m4310_Kp= 0.001, m4310_Kd = 0.001;
int ArmTurnAbsolute(joint_state_t* target) {
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
  if (target->base_vert_rotate >= BASE_VERT_ROTATE_MAX) {
   A1->Control(BASE_VERT_ROTATE_ID, 0.0, 0.0, BASE_VERT_ROTATE_MAX, A1_Kp, A1_Kd);
    target->base_vert_rotate = BASE_VERT_ROTATE_MAX;
  } else if (target->base_vert_rotate <= BASE_VERT_ROTATE_MIN) {
    A1->Control(BASE_VERT_ROTATE_ID, 0.0, 0.0, BASE_VERT_ROTATE_MIN,  A1_Kp, A1_Kd);
    target->base_vert_rotate = BASE_VERT_ROTATE_MIN;
  } else
    A1->Control(BASE_VERT_ROTATE_ID, 0.0, 0.0, target->base_vert_rotate,  A1_Kp, A1_Kd);
  current_joint_state.base_vert_rotate = target->base_vert_rotate;

  if (target->base_hor_rotate >= BASE_HOR_ROTATE_MAX) {
    A1->Control(BASE_HOR_ROTATE_ID, 0.0, 0.0, BASE_HOR_ROTATE_MAX,  A1_Kp, A1_Kd);
    target->base_hor_rotate = BASE_HOR_ROTATE_MAX;
  } else if (target->base_hor_rotate <= BASE_HOR_ROTATE_MIN) {
    A1->Control(BASE_HOR_ROTATE_ID, 0.0, 0.0, BASE_HOR_ROTATE_MIN,  A1_Kp, A1_Kd);
    target->base_hor_rotate = BASE_HOR_ROTATE_MIN;
  } else
    A1->Control(BASE_HOR_ROTATE_ID, 0.0, 0.0, target->base_hor_rotate,  A1_Kp, A1_Kd);
  current_joint_state.base_hor_rotate = target->base_hor_rotate;

  if (target->elbow_rotate >= ELBOW_ROTATE_MAX) {
    A1->Control(ELBOW_ROTATE_ID, 0.0, 0.0, ELBOW_ROTATE_MAX,  A1_Kp, A1_Kd);
    target->elbow_rotate = ELBOW_ROTATE_MAX;
  } else if (target->elbow_rotate <= ELBOW_ROTATE_MIN) {
    A1->Control(ELBOW_ROTATE_ID, 0.0, 0.0, ELBOW_ROTATE_MIN,  A1_Kp, A1_Kd);
    target->elbow_rotate = ELBOW_ROTATE_MIN;
  } else
    A1->Control(ELBOW_ROTATE_ID, 0.0, 0.0, target->elbow_rotate,  A1_Kp, A1_Kd);
  current_joint_state.elbow_rotate = target->elbow_rotate;

  // M4310
  //TODO: Adjust VALUE For TESTING
  if (target->forearm_rotate >= FOREARM_ROTATE_MAX) {
    forearm_rotate_motor->SetOutput(FOREARM_ROTATE_MAX, M4310_VEL, m4310_Kp, m4310_Kd, 0);
  } else if (target->forearm_rotate <= FOREARM_ROTATE_MIN) {
    forearm_rotate_motor->SetOutput(FOREARM_ROTATE_MIN, M4310_VEL, m4310_Kp, m4310_Kd, 0);
  } else
    forearm_rotate_motor->SetOutput(target->forearm_rotate, M4310_VEL, m4310_Kp, m4310_Kd, 0);
  current_joint_state.forearm_rotate = target->forearm_rotate;

  if (target->wrist_rotate >= WRIST_ROTATE_MAX) {
    wrist_rotate_motor->SetOutput(WRIST_ROTATE_MAX, M4310_VEL, m4310_Kp, m4310_Kd, 0);
  } else if (target->wrist_rotate <= WRIST_ROTATE_MIN) {
    wrist_rotate_motor->SetOutput(WRIST_ROTATE_MIN, M4310_VEL, m4310_Kp, m4310_Kd, 0);
  } else
    wrist_rotate_motor->SetOutput(target->wrist_rotate, M4310_VEL, m4310_Kp, m4310_Kd, 0);
  current_joint_state.wrist_rotate = target->wrist_rotate;

  if (target->hand_rotate >= HAND_ROTATE_MAX) {
    hand_rotate_motor->SetOutput(HAND_ROTATE_MAX, M4310_VEL, m4310_Kp, m4310_Kd, 0);
  } else if (target->hand_rotate <= HAND_ROTATE_MIN) {
    hand_rotate_motor->SetOutput(HAND_ROTATE_MIN, M4310_VEL, m4310_Kp, m4310_Kd, 0);
  } else
    hand_rotate_motor->SetOutput(target->hand_rotate, M4310_VEL, m4310_Kp, m4310_Kd, 0);
  current_joint_state.hand_rotate = target->hand_rotate;

  return 0;
}

void ArmTransmitOutput() {
//  control::MotorCANBase::TransmitOutput(m3508s, 1);
  control::Motor4310* forearm_motors[] = {forearm_rotate_motor, wrist_rotate_motor, hand_rotate_motor};

  joint_uart->Write((uint8_t*)(&(A1->send[0].data)), A1->send_length);
  joint_uart->Write((uint8_t*)(&(A1->send[1].data)), A1->send_length);
  joint_uart->Write((uint8_t*)(&(A1->send[2].data)), A1->send_length);

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
  print("ID: %d, Pos %04f\r\n", A1->recv[0].id, A1->recv[0].Pos);
  print("ID: %d, Pos %04f\r\n", A1->recv[1].id, A1->recv[1].Pos);
  print("ID: %d, Pos %04f\r\n", A1->recv[2].id, A1->recv[2].Pos);
  print("Forearm POS: %04f\r\n",forearm_rotate_motor->GetTheta());
  print("Wrist POS: %04f\r\n",wrist_rotate_motor->GetTheta());
  print("Hand POS: %04f\r\n",hand_rotate_motor->GetTheta());
}
