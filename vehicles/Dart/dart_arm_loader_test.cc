
/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2025 RoboMaster.                                          *
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
 #include "bsp_print.h"
 #include "cmsis_os.h"
 #include "controller.h"
 #include "dbus.h"
 #include "main.h"
 #include "math.h"
 #include "motor.h"
 #include "utils.h" 

#define KEY_GPIO_GROUP GPIOA
#define KEY_GPIO_PIN GPIO_PIN_0


#define DEFAULT_TASK_DELAY 100

#define JOINT1_PWM_CHANNEL 2
#define JOINT2_PWM_CHANNEL 3
#define JOINT3_PWM_CHANNEL 4
#define CLAW_MOTOR_PWM_CHANNEL 1

#define TIM_CLOCK_FREQ 1000000
#define MOTOR_OUT_FREQ 50

#define MAX_IOUT3508 16384
#define MAX_IOUT6020 60000
#define MAX_OUT 60000

#define MAP_RANGE(x, in_min, in_max, out_min, out_max) (((float)(x) - (float)(in_min)) * ((float)(out_max) - (float)(out_min)) / ((float)(in_max) - (float)(in_min)) + (float)(out_min))

osThreadId_t dartLoaderTestTaskHandle;
const osThreadAttr_t dartLoaderTestTaskAttribute = {.name = "dartLoaderTestTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = 256 * 4,
        .priority = (osPriority_t)osPriorityNormal,
        .tz_module = 0,
        .reserved = 0};



bsp::GPIO* key = nullptr;
control::MotorPWMBase* arm_claw = nullptr;
control::MotorCANBase* arm_yaw = nullptr;
control::MotorCANBase* arm_joint1 = nullptr;
control::MotorPWMBase* arm_joint2 = nullptr;
control::MotorPWMBase* arm_joint3 = nullptr;

control::MotorCANBase* slide_motor = nullptr;

static remote::DBUS *dbus = nullptr;

static bsp::CAN* can1 = nullptr;

float Kp = 50;
float Ki = 15;
float Kd = 65;
float diff_yaw_output = 0;
float diff_joint1_output = 0;

void dartLoaderTestTask(void* arg){
  UNUSED(arg);
  int yaw_output = 0;
  int joint1_output = 0;
  int joint2_output = 0;
  int joint3_output = 0;

  control::MotorCANBase* arm[] = {arm_yaw, arm_joint1};
  float diff_yaw = 0;
  float diff_joint1 = 0;
  float param[] = {Kp, Ki, Kd};
  control::ConstrainedPID pid3508(param, MAX_IOUT3508, MAX_OUT);
  control::ConstrainedPID pid6020(param, MAX_IOUT6020, MAX_OUT);

  while(1){
    if (dbus->swl == remote::UP) {
      arm_claw->SetOutput(1500); // 90 Degrees
    } else {
      arm_claw->SetOutput(500); // 0 Degrees  
    }

    if (dbus->swr == remote::UP) {
      // If Right Switch is up then Ch1 and 2 control first two joints.
      yaw_output = MAP_RANGE(dbus->ch1, -660, 660,-50, 50);
      joint1_output = MAP_RANGE(dbus->ch2, -660, 660, -50, 50);

      

      diff_yaw = arm_yaw->GetOmegaDelta(yaw_output);
      diff_joint1 = arm_joint1->GetOmegaDelta(joint1_output);
      diff_yaw_output = pid3508.ComputeConstrainedOutput(diff_yaw);
      diff_joint1_output = pid6020.ComputeConstrainedOutput(diff_joint1);
      arm_yaw->SetOutput(yaw_output);
      arm_joint1->SetOutput(joint1_output);
      control::MotorCANBase::TransmitOutput(arm, 2);

      // Keep the Servo PWM Constant
      arm_joint2->SetOutput(joint2_output);
      arm_joint3->SetOutput(joint3_output);

      print("Yaw speed: %d , diff_yaw: %.2f, yaw_output: %f \r\n", yaw_output, diff_yaw, diff_yaw_output);
      print("Joint 1 speed: %d , diff_joint1: %.2f, joint_output: %f \r\n", joint1_output, diff_joint1, diff_joint1_output);
      osDelay(10);

    } else if (dbus->swr == remote::DOWN) {
      // If Right Switch is down then Ch1 and 2 control last two joints
      joint2_output = MAP_RANGE(dbus->ch1, -660, 660, 500, 2500);
      joint3_output = MAP_RANGE(dbus->ch2, -660, 660, 500, 2500);

      arm_joint2->SetOutput(joint2_output);
      arm_joint3->SetOutput(joint3_output);

      print("Joint 2: %d\r\n", joint2_output);
      print("Joint 3: %d\r\n", joint3_output);
      osDelay(10);
    }
  }
}


void RM_RTOS_Init(){
  print_use_uart(&huart1);

  can1 = new bsp::CAN(&hcan1);

  key = new bsp::GPIO(KEY_GPIO_GROUP, KEY_GPIO_PIN);
  arm_claw = new control::MotorPWMBase(&htim1, CLAW_MOTOR_PWM_CHANNEL, TIM_CLOCK_FREQ, MOTOR_OUT_FREQ, 0);
  arm_joint2 = new control::MotorPWMBase(&htim1, JOINT2_PWM_CHANNEL, TIM_CLOCK_FREQ, MOTOR_OUT_FREQ, 0);
  arm_joint3 = new control::MotorPWMBase(&htim1, JOINT3_PWM_CHANNEL, TIM_CLOCK_FREQ, MOTOR_OUT_FREQ, 0);

  arm_yaw = new control::Motor6020(can1, 0x202);
  arm_joint1 = new control::Motor3508(can1, 0x203);

  dbus = new remote::DBUS(&huart3);
}


void RM_RTOS_Threads_Init(void) {
    dartLoaderTestTaskHandle = osThreadNew(dartLoaderTestTask, NULL, &dartLoaderTestTaskAttribute);
    if (dartLoaderTestTaskHandle == NULL) {
        print("Failed to create dart loader test task\r\n");
        Error_Handler();
    }
}

void RM_RTOS_Default_Task(const void* args){
    UNUSED(args);
    while(true){
      osDelay(DEFAULT_TASK_DELAY);
    }
}