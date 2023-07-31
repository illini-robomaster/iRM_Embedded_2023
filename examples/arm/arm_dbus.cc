
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
#include "dbus.h"

#include "arm.h"
#include "arm_config.h"

#define RX_SIGNAL (1 << 0)
#define A1_CONTROL_DELAY 100



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

static remote::DBUS* dbus;

extern osThreadId_t defaultTaskHandle;
class CustomUART : public bsp::UART {
 public:
  using bsp::UART::UART;
 protected:
  /* notify application when rx data is pending read */
  void RxCompleteCallback() override final { osThreadFlagsSet(defaultTaskHandle, RX_SIGNAL); }
};





/* A1 Motors */
static control::UnitreeMotor* base_vert_rotate_motor = nullptr;
static control::UnitreeMotor* base_hor_rotate_motor = nullptr;
static control::UnitreeMotor* elbow_rotate_motor = nullptr;
static CustomUART* A1 = nullptr;


void RM_RTOS_Init(void) {
    print_use_uart(&huart6);
    bsp::SetHighresClockTimer(&htim5);    

    dbus = new remote::DBUS(&huart3);

    can1 = new bsp::CAN(&hcan1, true);
    // control::steering_t steering_data;



    A1 = new CustomUART(BASE_HOR_ROTATE_UART);


    A1->SetupRx(300);
    A1->SetupTx(300);

    base_vert_rotate_motor = new control::UnitreeMotor();
    base_hor_rotate_motor = new control::UnitreeMotor();
    elbow_rotate_motor = new control::UnitreeMotor();

}

static joint_state_t current_joint_state = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
static joint_state_t target_state = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};

static int dataSend = 0;

void RM_RTOS_Default_Task(const void* args) {

    UNUSED(args);
    // A1 init state
    base_hor_rotate_motor->Stop( BASE_HOR_ROTATE_ID);
    dataSend = A1->Write((uint8_t*)(&(base_hor_rotate_motor->send.data)),
                                    base_hor_rotate_motor->send_length);
    osDelay(A1_CONTROL_DELAY);
    base_vert_rotate_motor->Stop(BASE_VERT_ROTATE_ID);
    A1->Write((uint8_t*)(&(base_vert_rotate_motor->send.data)),
                                    base_vert_rotate_motor->send_length);
    osDelay(A1_CONTROL_DELAY);

    elbow_rotate_motor->Stop(ELBOW_ROTATE_ID);
    A1->Write((uint8_t*)(&(elbow_rotate_motor->send.data)),
                                    elbow_rotate_motor->send_length);

    osDelay(A1_CONTROL_DELAY);

    
  // turn each motor for PI/16 degree every 2 seconds.
  // Should stop at defined limitation, see arm_config.h
    while (true) {

        print("CH0: %.04f CH1: %.04f CH2: %.04f CH3: %.04f ", dbus->ch0/600.0, dbus->ch1/600.0, dbus->ch2/600.0, dbus->ch3/600.0);
        print("SWL: %d SWR: %d DIAL: %d @ %d ms\r\n", dbus->swl, dbus->swr, dbus->wheel, dbus->timestamp);
        osDelay(100);
        if (dbus->swr == remote::DOWN){
          target_state = {0.0,(float)(dbus->ch0/600.0),(float)(dbus->ch1/600.0),(float)(dbus->ch2/600.0),0.0,0.0,0.0};
          ArmTurnRelative(&target_state);
          ArmPrintData();
          ArmTransmitOutput();
          osDelay(2);
        }
    }

}


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
  // A1
  // motor_id motor ID
  // torque expect torque
  // speed expect speed
  // position expect position
  // Kp Kp parameter for the PD controller
  // Kd Kd parameter for the PD controller
  if (target->base_vert_rotate >= BASE_VERT_ROTATE_MAX) {
    base_vert_rotate_motor->Control(BASE_VERT_ROTATE_ID, 0.0, 0.0, BASE_VERT_ROTATE_MAX, 0.003, 0.003);
  } else if (target->base_vert_rotate <= BASE_VERT_ROTATE_MIN) {
    base_vert_rotate_motor->Control(BASE_VERT_ROTATE_ID, 0.0, 0.0, BASE_VERT_ROTATE_MIN, 0.003, 0.003);
  } else {
    base_vert_rotate_motor->Control(BASE_VERT_ROTATE_ID, 0.0, 0.0, target->base_vert_rotate, 0.003, 0.003);
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

  return 0;
}

void ArmTransmitOutput() {

  A1->Write((uint8_t*)(&(base_hor_rotate_motor->send.data)),
                                base_hor_rotate_motor->send_length);
  osDelay(A1_CONTROL_DELAY);

  A1->Write((uint8_t*)(&(base_vert_rotate_motor->send.data)),
                                base_vert_rotate_motor->send_length);
  osDelay(A1_CONTROL_DELAY);
  
  A1->Write((uint8_t*)(&(elbow_rotate_motor->send.data)),
                                elbow_rotate_motor->send_length);
  osDelay(A1_CONTROL_DELAY);                          
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



