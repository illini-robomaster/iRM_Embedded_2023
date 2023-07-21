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

#include "main.h"

#include "bsp_print.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "gimbal.h"

#include "bsp_gpio.h"

#define NOTCH (2 * PI / 8)
#define SERVO_SPEED (PI)

#define KEY_GPIO_GROUP GPIOA
#define KEY_GPIO_PIN GPIO_PIN_0

bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;

bsp::GPIO* key = nullptr;
control::MotorCANBase* pitch_motor = nullptr;
control::MotorCANBase* yaw_motor = nullptr;

control::gimbal_t gimbal_init_data;
control::Gimbal* gimbal = nullptr;
remote::DBUS* dbus = nullptr;
bool status = false;

void RM_RTOS_Init() {
  print_use_uart(&huart1);
  can1 = new bsp::CAN(&hcan1, true);
  pitch_motor = new control::Motor6020(can1, 0x205);
  yaw_motor = new control::Motor6020(can1, 0x206);
  gimbal_init_data.pitch_motor = pitch_motor;
  gimbal_init_data.yaw_motor = yaw_motor;
  gimbal_init_data.model = control::GIMBAL_SENTRY;
  gimbal = new control::Gimbal(gimbal_init_data);

  dbus = new remote::DBUS(&huart3);
  key = new bsp::GPIO(KEY_GPIO_GROUP, KEY_GPIO_PIN);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  osDelay(500);  // DBUS initialization needs time

  control::MotorCANBase* motors[2] = {pitch_motor, yaw_motor};
  control::gimbal_data_t* gimbal_data = gimbal->GetData();

  while(!key->Read());
  while(key->Read());
  print("ok!\r\n");

  float pitch_ratio = 0.0;
  float yaw_ratio = 0.0;

  int i = 0;
  UNUSED(gimbal_data);

  while (true) {
    pitch_ratio = dbus->ch3 / 600.0;
    yaw_ratio = -dbus->ch2 / 600.0;
    if (dbus->swr == remote::MID) {
      gimbal->TargetRel(pitch_ratio / 30, yaw_ratio / 30);
    }

    i += 1;
    if (i >= 100) {
      print("p = %10.4f, y = %10.4f\r\n", pitch_ratio, yaw_ratio);
      i = 0;
    }

    // Kill switch
    if (dbus->swl == remote::UP || dbus->swl == remote::DOWN) {
      RM_ASSERT_TRUE(false, "Operation killed");
    }

    gimbal->Update();
    control::MotorCANBase::TransmitOutput(motors, 2);
    osDelay(10);
  }
}
