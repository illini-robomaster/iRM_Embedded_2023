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

#define REFEREE
#include "main.h"

#include "bsp_os.h"
#include "can.h"
#include "dbus.h"
#include "rgb.h"
#include "chassisTask.h"
//  #include "ui_task.h"
#ifdef REFEREE
#include "refereeTask.h"
#endif
#include "bsp_print.h"
//#define SINGLEBOARD
//
osThreadId_t chassisTaskHandle;
#ifdef REFEREE
osThreadId_t refereeTaskHandle;
#endif


bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;

remote::DBUS* dbus = nullptr;
// display::RGB* RGB = nullptr;
#ifdef REFEREE
RefereeUART* referee_uart = nullptr;
 communication::Referee* referee = nullptr;
#endif

#ifndef SINGLEBOARD
bsp::CanBridge* receive = nullptr;
#endif




void RM_RTOS_Init() {
  print_use_uart(&huart1);
  bsp::SetHighresClockTimer(&htim5);

  dbus = new remote::DBUS(&huart3);

  can1 = new bsp::CAN(&hcan1, true);
  can2 = new bsp::CAN(&hcan2, false);
  // RGB = new display::RGB(&htim5, 3, 2, 1, 1000000);
#ifdef REFEREE
  referee_uart = new RefereeUART(&huart6);
  referee_uart->SetupRx(300);
  referee_uart->SetupTx(300);
  referee = new communication::Referee();
#endif

#ifndef SINGLEBOARD
  receive = new bsp::CanBridge(can2,0x20B,0x20A);
#endif
  init_chassis();
  set_cursor(0,0);
  clear_screen();
}


void RM_RTOS_Threads_Init(void) {
  chassisTaskHandle = osThreadNew(chassisTask,nullptr,&chassisTaskAttribute);
#ifdef REFEREE
  refereeTaskHandle = osThreadNew(refereeTask,nullptr,&refereeTaskAttribute);
#endif
//     UITaskHandle = osThreadNew(UITask,nullptr,&UITaskAttribute);
}


void KillAll() {
  kill_chassis();
}


void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  int loop_cnt = 0;
  while(true){
    if(loop_cnt == 200){
      // print("referee power limit: %f \r\n", (float)referee->game_robot_status.chassis_power_limit);
      print("p: %f \r\n", referee->power_heat_data.chassis_power);
      // print("chassis power buffer: %f \r\n", (float)(referee->power_heat_data.chassis_power_buffer));
      loop_cnt = 0;
    }
    loop_cnt++;
  }


  while(false){ //if want to print, make sure nothing is print somewhere else
    set_cursor(0,0);
    clear_screen();
    print("HAL Tick: %d \r\n", HAL_GetTick());
    print("vx: %d, vy: %d, wz; %d \r\n", dbus->ch0, dbus->ch1, dbus->ch2);
#ifdef REFEREE
    print("ROBOTID: %d",referee->game_robot_status.robot_id);
#endif
    osDelay(10);
  }
}