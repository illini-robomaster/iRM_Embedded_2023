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

#include "bsp_buzzer.h"
#include "bsp_can_bridge.h"
#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "bsp_relay.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "controller.h"
#include "dbus.h"
#include "gimbalTask.h"
#include "motor.h"
#include "oled.h"
#include "protocol.h"
#include "rgb.h"
#include "shooterTask.h"

remote::DBUS* dbus = nullptr;
bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;
bsp::CanBridge* send = nullptr;

// lob mode switch
BoolEdgeDetector lob_mode_sw=BoolEdgeDetector(false);
volatile bool lob_mode=false;
//display::RGB* RGB = nullptr;

osThreadId_t shooterTaskHandle;
osThreadId_t gimbalTaskHandle;


// Params Initialization
void RM_RTOS_Init(){
  print_use_uart(&huart1);
  // Initialize the CAN bus
  can1 = new bsp::CAN(&hcan1, true);
  can2 = new bsp::CAN(&hcan2, false);
  // Initialize the DBUS
  dbus = new remote::DBUS(&huart3);
  bsp::SetHighresClockTimer(&htim5);
  send = new bsp::CanBridge(can2, 0x20A, 0x20B);
  // Initialize the RGB LED
//  RGB=new display::RGB(&htim5,3,2,1,1000000);
  // shooter initialization
  init_shooter();
  init_gimbal();
}

void RM_RTOS_Threads_Init(void){
    shooterTaskHandle = osThreadNew(shooterTask, nullptr,&shooterTaskAttribute);
    gimbalTaskHandle = osThreadNew(gimbalTask, nullptr,&gimbalTaskAttribute);
}

void KillAll(){
  kill_shooter();
  kill_gimbal();
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  while (true) {
    osDelay(100);
  }
}