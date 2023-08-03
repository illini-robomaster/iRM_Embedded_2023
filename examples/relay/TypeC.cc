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



/**
 * TYPE C BOARD GPIO MAPPING
 *  PORT  | RECOMMEND | OFFICIAL | PREFERRED NAME 
 * ----------------------------------------------|
 * GPIO_1 | SPI2_CS   | PB12     |
 * GPIO_2 | GND       | -        | 
 * GPIO_3 | SPI2_CLK  | PB13     |
 * GPIO_4 | 3.3V      | -        | 
 * GPIO_5 | SPI2_MOSI | PB15     | ENABLE  |  
 * GPIO_6 | I2C2_SCL  | PF1      |
 * GPIO_7 | SPI2_MISO | PB14     | DIR     |
 * GPIO_8 | I2C2_SDA  | PF0      |
 * 
 * 
*/


#include "main.h"

#include "bsp_print.h"
#include "bsp_relay.h"
#include "cmsis_os.h"

static bsp::Relay* relay;

void RM_RTOS_Init(void) {
  print_use_uart(&huart1);
  relay = new bsp::Relay(RELAY_1_GPIO_Port, RELAY_1_Pin); /* USE GPIO_1 (PB14)->PIN 7 in the board*/
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);

  while (true) {
    set_cursor(0, 0);
    clear_screen();
    relay->On();
    print("relay on\r\n");
    osDelay(1000);
    set_cursor(0, 0);
    clear_screen();
    relay->Off();
    print("relay off\r\n");
    osDelay(1000);
  }
}
