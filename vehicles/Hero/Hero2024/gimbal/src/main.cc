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
#include "bsp_can_bridge.h"
#include "cmsis_os.h"
#include "controller.h"
#include "dbus.h"
#include "main.h"
#include "motor.h"
#include "protocol.h"
#include "rgb.h"
#include "oled.h"
#include "bsp_buzzer.h"
#include "shooterTask.h"

remote::DBUS* dbus = nullptr;
bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;
bsp::CanBridge* send = nullptr;

// lob mode switch
BoolEdgeDetector lob_mode_sw=BoolEdgeDetector(false);
volatile bool lob_mode=false;
//display::RGB* RGB = nullptr;

// Params Initialization
void RM_RTOS_Init(){
  print_use_uart(&huart1);
  // Initialize the CAN bus
  can1 = new bsp::CAN(&hcan1);
  can2 = new bsp::CAN(&hcan2);
  // Initialize the DBUS
  dbus = new remote::DBUS(&huart3);
  bsp::SetHighresClockTimer(&htim5);
  // Initialize the RGB LED
//  RGB=new display::RGB(&htim5,3,2,1,1000000);
  init_shooter();
}

void RM_RTOS_Threads_Init(){
    shooterTaskHandle = osThreadNew(shooterTask, nullptr,&shooterTaskAttribute);
}

void KillAll(){
  kill_shooter();
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  while (true) {
    osDelay(1000);
  }
}