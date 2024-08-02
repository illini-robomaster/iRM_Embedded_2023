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

#include "shooterTask.h"
#include "cmsis_os.h"
#include "main.h"

bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;
bsp::CanBridge* with_gimbal = nullptr;
bsp::CanBridge* with_chassis = nullptr;
osThreadId_t shooterTaskHandle;
bsp::Buzzer *buzzer = nullptr;


void RM_RTOS_Init(){
    print_use_uart(&huart1);
    can1 = new bsp::CAN(&hcan1,true);
    can2 = new bsp::CAN(&hcan2,false);
    with_gimbal = new bsp::CanBridge(can2,0x20C,0x20A);
    with_chassis = new bsp::CanBridge(can2,0x20E,0x20D);
    buzzer = new bsp::Buzzer(&htim4, 3, 1000000);
    init_shooter();
}

void RM_RTOS_Threads_Init(void) {
  shooterTaskHandle = osThreadNew(shooter_task, nullptr, &shooterTaskAttribute);
//     UITaskHandle = osThreadNew(UITask,nullptr,&UITaskAttribute);
}
