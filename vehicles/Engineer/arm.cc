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
#include "main.h"
#include "motor.h"
#include "unitree_motor.h"
#include "utils.h"

#include "arm.h"
#include "arm_config.h"

#define RX_SIGNAL (1 << 0)

namespace engineer {

/* see arm_config.h */
/* all constants subject to change should be defined in arm_config.h */
extern const float BASE_TRANSLATE_MAX;
extern const float BASE_VERT_ROTATE_MAX;
extern const float BASE_HOR_ROTATE_MAX;
extern const float ELBOW_ROTATE_MAX;
extern const float FOREARM_ROTATE_MAX;
extern const float WRIST_ROTATE_MAX;
extern const float HAND_ROTATE_MAX;
extern const float BASE_TRANSLATE_MIN;
extern const float BASE_VERT_ROTATE_MIN;
extern const float BASE_HOR_ROTATE_MIN;
extern const float ELBOW_ROTATE_MIN;
extern const float FOREARM_ROTATE_MIN;
extern const float WRIST_ROTATE_MIN;
extern const float HAND_ROTATE_MIN;

extern const int BASE_TRANSLATE_ID;
extern GPIO_TypeDef* BASE_TRANSLATE_CALI_GPIO_PORT;
extern const uint16_t BASE_TRANSLATE_CALI_GPIO_PIN;
extern const float RUN_SPEED;
extern const float ALIGN_SPEED;
extern const float ACCELERATION;

extern const float M4310_VEL;

extern const int BASE_HOR_ROTATE_ID;
extern const int BASE_VERT_ROTATE_ID;
extern const int ELBOW_ROTATE_ID;
extern UART_HandleTypeDef* BASE_HOR_ROTATE_UART;
extern UART_HandleTypeDef* BASE_VERT_ROTATE_UART;
extern UART_HandleTypeDef* ELBOW_ROTATE_UART;

extern const int FOREARM_ROTATE_RX_ID;
extern const int FOREARM_ROTATE_TX_ID;
extern const int WRIST_ROTATE_RX_ID;
extern const int WRIST_ROTATE_TX_ID;
extern const int HAND_ROTATE_RX_ID;
extern const int HAND_ROTATE_TX_ID;

/**
 * define params start
**/
bsp::CAN* can1 = nullptr;

// 3508
control::MotorCANBase* motor1 = nullptr;
control::SteeringMotor* base_translate_motor = nullptr;

bsp::GPIO* base_translate_pe_sensor = nullptr;
bool base_translate_align_detect() {
  return base_translate_pe_sensor->Read() == 1;
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

// A1
// TODO this part is unstable
extern osThreadId_t defaultTaskHandle;
class CustomUART : public bsp::UART {
 public:
  using bsp::UART::UART;
 protected:
  /* notify application when rx data is pending read */
  void RxCompleteCallback() override final { osThreadFlagsSet(defaultTaskHandle, RX_SIGNAL); }
};
static control::UnitreeMotor* base_vert_rotate_motor = nullptr;
static control::UnitreeMotor* base_hor_rotate_motor = nullptr;
static control::UnitreeMotor* elbow_rotate_motor = nullptr;
static CustomUART* base_vert_rotate_motor_uart = nullptr;
static CustomUART* base_hor_rotate_motor_uart = nullptr;
static CustomUART* elbow_rotate_motor_uart = nullptr;

// 4310
static control::Motor4310* forearm_rotate_motor = nullptr;
static control::Motor4310* wrist_rotate_motor = nullptr;
static control::Motor4310* hand_rotate_motor = nullptr;

// entire arm
static joint_state_t current_joint_state = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
static control::MotorCANBase* m3508s[] = {motor1};

// TODO fix transmitoutput for 4310 multi motors
static control::Motor4310* forearm_motors[] = {forearm_rotate_motor};
static control::Motor4310* wrist_motors[] = {wrist_rotate_motor};
static control::Motor4310* hand_motors[] = {hand_rotate_motor};
/**
 * define params ends
**/

void RM_RTOS_Init() {
  print_use_uart(&huart1);
  bsp::SetHighresClockTimer(&htim5);

  control::steering_t steering_data;

  can1 = new bsp::CAN(&hcan1, true);

  // Init m3508 * 1
  motor1 = new control::Motor3508(can1, BASE_TRANSLATE_ID);
  base_translate_pe_sensor = new bsp::GPIO(BASE_TRANSLATE_CALI_GPIO_PORT,
                                           BASE_TRANSLATE_CALI_GPIO_PIN);
  steering_data.motor = motor1;
  steering_data.max_speed = RUN_SPEED;
  steering_data.max_acceleration = ACCELERATION;
  // TODO make sure the gear ratio is correct
  steering_data.transmission_ratio = M3508P19_RATIO;
  steering_data.omega_pid_param = new float[3]{140, 1.2, 25};
  steering_data.max_iout = 1000;
  steering_data.max_out = 13000;
  // TODO measure the calibrate offset for base translate motor
  steering_data.calibrate_offset = 0;
  steering_data.align_detect_func = base_translate_align_detect;
  base_translate_motor = new control::SteeringMotor(steering_data);

  // Init M4310 * 3
  /* rx_id = Master id
   * tx_id = CAN id
   * see example/m4310_mit.cc
   */
  forearm_rotate_motor = new control::Motor4310(
    can1, FOREARM_ROTATE_RX_ID, FOREARM_ROTATE_TX_ID, control::MIT);
  wrist_rotate_motor = new control::Motor4310(
    can1, WRIST_ROTATE_RX_ID, WRIST_ROTATE_TX_ID, control::MIT);
  hand_rotate_motor = new control::Motor4310(
    can1, HAND_ROTATE_RX_ID, HAND_ROTATE_TX_ID, control::MIT);

  // Init A1 * 3
  base_vert_rotate_motor_uart = new CustomUART(BASE_HOR_ROTATE_UART);
  base_hor_rotate_motor_uart = new CustomUART(BASE_VERT_ROTATE_UART);
  elbow_rotate_motor_uart = new CustomUART(ELBOW_ROTATE_UART);

  base_vert_rotate_motor_uart->SetupRx(300);
  base_vert_rotate_motor_uart->SetupTx(300);
  base_hor_rotate_motor_uart->SetupRx(300);
  base_hor_rotate_motor_uart->SetupTx(300);
  elbow_rotate_motor_uart->SetupRx(300);
  elbow_rotate_motor_uart->SetupTx(300);

  base_vert_rotate_motor = new control::UnitreeMotor();
  base_hor_rotate_motor = new control::UnitreeMotor();
  elbow_rotate_motor = new control::UnitreeMotor();
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);


  base_translate_motor->SetMaxSpeed(ALIGN_SPEED);

  // base translate calibration
  bool base_translate_alignment_complete = false;
  while (!base_translate_alignment_complete) {
    base_translate_motor->CalcOutput();
    control::MotorCANBase::TransmitOutput(m3508s, 1);
    base_translate_alignment_complete = base_translate_motor->Calibrate();
    osDelay(2);
  }

  base_translate_motor->ReAlign();
  //base_translate_motor->SetMaxSpeed(RUN_SPEED);
  control::MotorCANBase::TransmitOutput(m3508s, 1);

  // 4310 init state
  // TODO: config zero pos with 4310 config assist
  forearm_rotate_motor->MotorEnable();
  wrist_rotate_motor->MotorEnable();
  hand_rotate_motor->MotorEnable();


  // A1 init state
  base_hor_rotate_motor->Stop(0);
  base_hor_rotate_motor_uart->Write((uint8_t*)(&(base_hor_rotate_motor->send.data)),
                                base_hor_rotate_motor->send_length);

  base_vert_rotate_motor->Stop(0);
  base_vert_rotate_motor_uart->Write((uint8_t*)(&(base_vert_rotate_motor->send.data)),
                                base_vert_rotate_motor->send_length);

  elbow_rotate_motor->Stop(0);
  elbow_rotate_motor_uart->Write((uint8_t*)(&(elbow_rotate_motor->send.data)),
                                elbow_rotate_motor->send_length);

  print("arm starts in 2 secs\r\n");
  // wait for 2 seconds
  osDelay(2000);

  joint_state_t target = {PI / 16, PI / 16, PI / 16, PI / 16, PI / 16, PI / 16, PI / 16};

  int i = 0;
  // turn each motor for PI/16 degree every 2 seconds.
  // Should stop at defined limitation, see arm_config.h
  while (true) {
    if (i <= 1000) {
      i++;
    } else {
      ArmTurnRelative(&target);
      ArmPrintData();
    }
    ArmTransmitOutput();
    osDelay(2);
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

int ArmTurnAbsolute(joint_state_t* target) {
  // M3508
  if (target->base_translate >= BASE_TRANSLATE_MAX) {
    base_translate_motor->TurnRelative(BASE_TRANSLATE_MAX-current_joint_state.base_translate);
  } else if (target->base_translate <= BASE_TRANSLATE_MIN) {
    base_translate_motor->TurnRelative(BASE_TRANSLATE_MIN-current_joint_state.base_translate);
  } else {
    base_translate_motor->TurnRelative(target->base_translate-current_joint_state.base_translate);
  }
  current_joint_state.base_translate = target->base_translate;

  // A1
  // motor_id motor ID
  // torque expect torque
  // speed expect speed
  // position expect position
  // Kp Kp parameter for the PD controller
  // Kd Kd parameter for the PD controller
  if (target->base_vert_rotate >= BASE_VERT_ROTATE_MAX) {
    base_vert_rotate_motor->Control(BASE_VERT_ROTATE_ID, 0.0, 0.0, BASE_VERT_ROTATE_MAX, 0.0, 3.0);
  } else if (target->base_vert_rotate <= BASE_VERT_ROTATE_MIN) {
    base_vert_rotate_motor->Control(BASE_VERT_ROTATE_ID, 0.0, 0.0, BASE_VERT_ROTATE_MIN, 0.0, 3.0);
  } else {
    base_vert_rotate_motor->Control(BASE_VERT_ROTATE_ID, 0.0, 0.0, target->base_vert_rotate, 0.0, 3.0);
  }
  current_joint_state.base_vert_rotate = target->base_vert_rotate;

  if (target->base_hor_rotate >= BASE_HOR_ROTATE_MAX) {
    base_hor_rotate_motor->Control(BASE_HOR_ROTATE_ID, 0.0, 0.0, BASE_HOR_ROTATE_MAX, 0.003, 0.003);
  } else if (target->base_hor_rotate <= BASE_HOR_ROTATE_MIN) {
    base_hor_rotate_motor->Control(BASE_HOR_ROTATE_ID, 0.0, 0.0, BASE_HOR_ROTATE_MIN, 0.003, 0.003);
  } else {
    base_hor_rotate_motor->Control(BASE_HOR_ROTATE_ID, 0.0, 0.0, target->base_hor_rotate, 0.003, 0.003);
  }
  current_joint_state.base_hor_rotate = target->base_hor_rotate;

  if (target->elbow_rotate >= ELBOW_ROTATE_MAX) {
    elbow_rotate_motor->Control(ELBOW_ROTATE_ID, 0.0, 0.0, ELBOW_ROTATE_MAX, 0.003, 0.003);
  } else if (target->elbow_rotate <= ELBOW_ROTATE_MIN) {
    elbow_rotate_motor->Control(ELBOW_ROTATE_ID, 0.0, 0.0, ELBOW_ROTATE_MIN, 0.003, 0.003);
  } else {
    elbow_rotate_motor->Control(ELBOW_ROTATE_ID, 0.0, 0.0, target->elbow_rotate, 0.003, 0.003);
  }
  current_joint_state.elbow_rotate = target->elbow_rotate;

  // M4310
  //TODO: Adjust VALUE For TESTING
  if (target->forearm_rotate >= FOREARM_ROTATE_MAX) {
    forearm_rotate_motor->SetOutput(FOREARM_ROTATE_MAX, M4310_VEL, 30, 0.5, 0);
  } else if (target->forearm_rotate <= FOREARM_ROTATE_MIN) {
    forearm_rotate_motor->SetOutput(FOREARM_ROTATE_MIN, M4310_VEL, 30, 0.5, 0);
  } else {
    forearm_rotate_motor->SetOutput(target->forearm_rotate, M4310_VEL, 30, 0.5, 0);
  }
  current_joint_state.forearm_rotate = target->forearm_rotate;

  if (target->wrist_rotate >= WRIST_ROTATE_MAX) {
    wrist_rotate_motor->SetOutput(WRIST_ROTATE_MAX, M4310_VEL, 30, 0.5, 0);
  } else if (target->wrist_rotate <= WRIST_ROTATE_MIN) {
    wrist_rotate_motor->SetOutput(WRIST_ROTATE_MIN, M4310_VEL, 30, 0.5, 0);
  } else {
    wrist_rotate_motor->SetOutput(target->wrist_rotate, M4310_VEL, 30, 0.5, 0);
  }
  current_joint_state.wrist_rotate = target->wrist_rotate;


  if (target->hand_rotate >= HAND_ROTATE_MAX) {
    hand_rotate_motor->SetOutput(HAND_ROTATE_MAX, M4310_VEL, 30, 0.5, 0);
  } else if (target->hand_rotate <= HAND_ROTATE_MIN) {
    hand_rotate_motor->SetOutput(HAND_ROTATE_MIN, M4310_VEL, 30, 0.5, 0);
  } else {
    hand_rotate_motor->SetOutput(target->hand_rotate, M4310_VEL, 30, 0.5, 0);
  }
  current_joint_state.hand_rotate = target->hand_rotate;

  return 0;
}

void ArmTransmitOutput() {

  control::MotorCANBase::TransmitOutput(m3508s, 1);

  base_hor_rotate_motor_uart->Write((uint8_t*)(&(base_hor_rotate_motor->send.data)),
                                base_hor_rotate_motor->send_length);
  base_vert_rotate_motor_uart->Write((uint8_t*)(&(base_vert_rotate_motor->send.data)),
                                base_vert_rotate_motor->send_length);
  elbow_rotate_motor_uart->Write((uint8_t*)(&(elbow_rotate_motor->send.data)),
                                elbow_rotate_motor->send_length);

  control::Motor4310::TransmitOutput(forearm_motors, 1);
  control::Motor4310::TransmitOutput(wrist_motors, 1);
  control::Motor4310::TransmitOutput(hand_motors, 1);
}

void ArmPrintData() {
  print("Base Translate   : %10.4f\r\n", current_joint_state.base_translate);
  print("Base Vert Rotate : %10.4f\r\n", current_joint_state.base_vert_rotate);
  print("Base Hor  Rotate : %10.4f\r\n", current_joint_state.base_hor_rotate);
  print("Elbow Rotate     : %10.4f\r\n", current_joint_state.elbow_rotate);
  print("Forearm Rotate   : %10.4f\r\n", current_joint_state.forearm_rotate);
  print("Wrist Rotate     : %10.4f\r\n", current_joint_state.wrist_rotate);
  print("Hand Rotate      : %10.4f\r\n", current_joint_state.hand_rotate);
}

} // end ns engineer

