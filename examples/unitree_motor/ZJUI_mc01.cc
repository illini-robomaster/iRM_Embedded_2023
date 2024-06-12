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
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "sbus.h"
#include "bsp_gpio.h"
#include "A1_motor_drive.h"

static remote::SBUS* dbus;
bsp::GPIO* user_key = nullptr;

void RM_RTOS_Init(void) {
  print_use_uart(&huart4);
  print("Unitree Motor Test\r\n");
  dbus = new remote::SBUS(&huart3);
  user_key = new bsp::GPIO(GPIOA, GPIO_PIN_15);
}

void RM_RTOS_Threads_Init(void) {
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);

  print("Press key to start\r\n");
  while(user_key->Read()); // wait for key press
  print("Open loop spin test\r\n");
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET); // 485_1 in write mode
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET); // 485_2 in write mode
  // A1->Test(2);
  // A1_write_uart->Write((uint8_t*)(&(A1->send[2].data)), A1->send_length);
  // osDelay(3000);

  // print("Motor stop\r\n");
  // A1->Stop(2);
  // A1_write_uart->Write((uint8_t*)(&(A1->send[2].data)), A1->send_length);
  // osDelay(3000);

  // print("Constant speed mode\r\n");
  // A1->Control(2, 0.0, -1.0, 0.0, 0.0, 3.0); // constant speed mode
  // A1_write_uart->Write((uint8_t*)(&(A1->send[2].data)), A1->send_length);
  // osDelay(3000);

  // print("Motor stop\r\n");
  // A1->Stop(2);
  // A1_write_uart->Write((uint8_t*)(&(A1->send[2].data)), A1->send_length);
  // osDelay(3000);

  modfiy_speed_cmd(&MotorA1_send_left,2,0.1);
  unitreeA1_rxtx();
  osDelay(1);

}
