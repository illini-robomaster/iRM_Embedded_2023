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

#include "chassis.h"

#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "bsp_relay.h"
#include "cmsis_os.h"
#include "controller.h"
#include "dbus.h"
#include "main.h"
#include "motor.h"
#include "protocol.h"
#include "rgb.h"
#include "oled.h"
#include "bsp_buzzer.h"

// Global Variables
static remote::DBUS* dbus = nullptr;
static bsp::CAN* can1 = nullptr;

//==================================================================================================
// Shooter
//==================================================================================================
const osThreadAttr_t shooterTaskAttribute = {.name = "shooterTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 256 * 4,
                                             .priority = (osPriority_t)osPriorityNormal,
                                             .tz_module = 0,
                                             .reserved = 0};

osThreadId_t shooterTaskHandle;

// Params Initialization
static control::Motor4310* load_motor = nullptr;

static control::MotorCANBase* shoot_front_motor = nullptr;
static control::MotorCANBase* shoot_back_motor = nullptr;

static control::MotorCANBase* force_motor = nullptr;
static control::ServoMotor* force_servo = nullptr;

void shooterTask(void* arg) {
  UNUSED(arg);
  // shooter desired speed
  float shoot_speed = 30;
  // motor initialization
  control::MotorCANBase* can1_shooter_shoot[] = {shoot_front_motor, shoot_back_motor};
//  control::MotorCANBase* can1_shooter_force[] = {force_motor};
//  control::Motor4310* can1_shooter_load[] = {load_motor};
  // PID controller initialization
  float shoot_pid_params[3] = {20, 15, 10};
  control::ConstrainedPID shoot_pid(shoot_pid_params, 1000, 10000);
  float shoot_front_speed_diff = 0;
  float shoot_back_speed_diff = 0;
  int16_t shoot_front_out = 0;
  int16_t shoot_back_out = 0;
  print("11");
//  while (true) {
//    if (dbus->keyboard.bit.B || dbus->swr == remote::DOWN) break;
//    print("22");
//    osDelay(100);
//  }
  print("22");
  while (true) {
//    if (dbus->swr == remote::UP) {
      shoot_front_speed_diff = shoot_front_motor->GetOmegaDelta(shoot_speed);
      shoot_back_speed_diff = shoot_back_motor->GetOmegaDelta(shoot_speed);
      shoot_front_out = shoot_pid.ComputeConstrainedOutput(shoot_front_speed_diff);
      shoot_back_out = shoot_pid.ComputeConstrainedOutput(shoot_back_speed_diff);
      print("out %d %d\n", shoot_front_out, shoot_back_out);
//    } else {
//      shoot_front_speed_diff = 0;
//      shoot_back_speed_diff = 0;
//      shoot_front_out = 0;
//      shoot_back_out = 0;
//    }
    shoot_front_motor->SetOutput(shoot_front_out);
    shoot_back_motor->SetOutput(shoot_back_out);
    control::MotorCANBase::TransmitOutput(can1_shooter_shoot, 2);
  }
}

//==================================================================================================
// RM Init
//==================================================================================================

void RM_RTOS_Init() {
  // peripherals initialization
  print_use_uart(&huart1);
  bsp::SetHighresClockTimer(&htim5);
  // Dbus
  dbus = new remote::DBUS(&huart3);
  // Can
  can1 = new bsp::CAN(&hcan1, true);

  //Shooter initialization
  load_motor = new control::Motor4310(can1, 0x02, 0x01, control::POS_VEL);
  shoot_front_motor = new control::Motor3508(can1, 0x201);
  shoot_back_motor = new control::Motor3508(can1, 0x202);
  force_motor = new control::Motor3508(can1, 0x203);

  // Servo control for each shooter motor
  control::servo_t servo_data;
  servo_data.motor = force_motor;
  servo_data.max_speed = 100 * PI; // params need test
  servo_data.max_acceleration = 100 * PI;
  servo_data.transmission_ratio = M3508P19_RATIO;
  servo_data.omega_pid_param = new float [3] {150, 1.2, 5};
  servo_data.max_iout = 1000;
  servo_data.max_out = 13000;
  force_servo = new control::ServoMotor(servo_data);
}

//==================================================================================================
// RTOS Threads Init
//==================================================================================================

void RM_RTOS_Threads_Init(void) {
  shooterTaskHandle = osThreadNew(shooterTask, nullptr, &shooterTaskAttribute);
}

//==================================================================================================
// Kill All
//==================================================================================================

//void KillAll() {
//  RM_EXPECT_TRUE(false, "Operation Killed!\r\n");
//
//}

//==================================================================================================
// RTOS Default Task
//==================================================================================================

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  while (true) {
    osDelay(1000);
  }
}
