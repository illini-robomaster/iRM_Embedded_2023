/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2022 RoboMaster.                                          *
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

#include <cstring>
#include <memory>

#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "controller.h"
#include "dbus.h"
#include "main.h"
#include "motor.h"
#include "utils.h"

#define KEY_GPIO_GROUP GPIOB
#define KEY_GPIO_PIN GPIO_PIN_2

#define NOTCH (2 * PI / 4)
#define SPEED 200
#define ACCELERATION (80 * PI)

bsp::CAN* can1 = nullptr;
control::MotorCANBase* motor = nullptr;
control::ServoMotor* servo = nullptr;
BoolEdgeDetector key_detector(false);

// remote::DBUS* dbus = nullptr;

bsp::GPIO* gpio_red;

const osThreadAttr_t baseTaskAttribute = {.name = "baseTask",
                                          .attr_bits = osThreadDetached,
                                          .cb_mem = nullptr,
                                          .cb_size = 0,
                                          .stack_mem = nullptr,
                                          .stack_size = 512 * 4,
                                          .priority = (osPriority_t)osPriorityRealtime,
                                          .tz_module = 0,
                                          .reserved = 0};
osThreadId_t baseTaskHandle;

// declare for the base Task
void baseTask(void* argument);

void RM_RTOS_Threads_Init(void) {
  baseTaskHandle = osThreadNew(baseTask, nullptr, &baseTaskAttribute);
}

void RM_RTOS_Init(void) {
  gpio_red = new bsp::GPIO(LED_R_GPIO_Port, LED_R_Pin);
  gpio_red->High();

  // print_use_uart(&huart8);
  bsp::SetHighresClockTimer(&htim5);

  can1 = new bsp::CAN(&hcan1, 0x201);

  motor = new control::Motor3508(can1, 0x201);
  control::servo_t servo_data;
  servo_data.motor = motor;
  servo_data.max_speed = SPEED;
  servo_data.max_acceleration = ACCELERATION;
  servo_data.transmission_ratio = M3508P19_RATIO;
  servo_data.omega_pid_param = new float[3]{140, 1.2, 25};
  servo_data.max_iout = 1000;
  servo_data.max_out = 13000;
  servo = new control::ServoMotor(servo_data);

  // dbus = new remote::DBUS(&huart3);
}

// dummy main thread to test multi-tasks
void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  osDelay(500);  // DBUS initialization needs time
  while (true) {
    gpio_red->Toggle();
    osDelay(500);
  }
}

void baseTask(void* argument) {
  UNUSED(argument);
  osDelay(500);  // DBUS initialization needs time

  // real pos
  float target = 0;
  // want-to-go pos
  float curr_target = 0.0;

  control::MotorCANBase* motors[] = {motor};
  float direction = 1.0;
  const float MAX = 2.0 * PI;
  const float MIN = -2.0 * PI;
  float step = PI;
  while (true) {
    // target = float(dbus->ch1) / remote::DBUS::ROCKER_MAX * 6 * PI;
    curr_target += direction * step;
    // servo->GetTheta(); you can use this to get the absolute pos of the motor
    // if servo command not rejected
    // use FALSE to wait until last command finished
    if (servo->SetTarget(curr_target, false) != 0) {
      target = curr_target;
      if (target >= MAX || target <= MIN) {
        direction = -direction;
      }
    } else {
      curr_target = target;
    }
    servo->CalcOutput();
    control::MotorCANBase::TransmitOutput(motors, 1);

    osDelay(2);
  }
}
