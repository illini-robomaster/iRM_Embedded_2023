/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2024 RoboMaster.                                          *
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
#include "bsp_relay.h"
#include "bsp_gpio.h"
#include "utils.h"
#include "cmsis_os.h"
#include "dbus.h"

GPIO_InitTypeDef Relay2_Init;

static bsp::Relay* relay1;
static bsp::Relay* relay2;
static bsp::Relay* relay3;
static bsp::Relay* relay4;

bsp::GPIO* key = nullptr;
BoolEdgeDetector key_detector(false);
bool key_toggle = false;

remote::DBUS *dbus = nullptr;

void RM_RTOS_Init(void) {

  Relay2_Init.Pin = RELAY_2_Pin;
  Relay2_Init.Mode = GPIO_MODE_OUTPUT_PP;
  Relay2_Init.Pull = GPIO_NOPULL;
  Relay2_Init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RELAY_2_GPIO_Port, &Relay2_Init);

  print_use_uart(&huart6);

  relay1 = new bsp::Relay(RELAY_1_GPIO_Port, RELAY_1_Pin); /* USE SPI2_CS */
  relay2 = new bsp::Relay(RELAY_2_GPIO_Port, RELAY_2_Pin); /* USE SPI2_CLK */
  relay3 = new bsp::Relay(RELAY_3_GPIO_Port, RELAY_3_Pin); /* USE SPI2_MISO */
  relay4 = new bsp::Relay(RELAY_4_GPIO_Port, RELAY_4_Pin); /* USE SPI2_MOSI */

  key = new bsp::GPIO(KEY_GPIO_Port, KEY_Pin);
  dbus = new remote::DBUS(&huart3);
  relay1->Off();
  relay2->Off();
  relay3->Off();
  relay4->Off();
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);
  print("initialized\r\n");
  osDelay(100);


  while (true) {
//    key_detector.input(dbus->swl==remote::UP);
    key_detector.input(!(key->Read()));

    if (key_detector.posEdge()) {
      key_toggle=!key_toggle;
      print("triggered\r\n");
    }
//    if(key_toggle){
//      relay1->On();
//      relay2->On();
//      relay3->On();
//      relay4->On();
//      print("relay on\r\n");
//    } else {
//      relay1->Off();
//      relay2->Off();
//      relay3->Off();
//      relay4->Off();
//      print("relay off\r\n");
//    }
    if (dbus->swl==remote::UP){
      relay1->Off();
      relay2->Off();
    } else if (dbus->swl==remote::MID){
      relay1->On();
      relay2->Off();
    } else if (dbus->swl==remote::DOWN){
      relay1->On();
      relay2->On();
    }

    if (dbus->swr==remote::UP){
      relay3->Off();
      relay4->Off();
    } else if (dbus->swr==remote::MID){
      relay3->On();
      relay4->Off();
    } else if (dbus->swr==remote::DOWN){
      relay3->On();
      relay4->On();
    }

    osDelay(100);
  }
}
