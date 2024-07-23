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
// #define REFEREE
// #define CHASSIS
#define ARM_A1

#include "main.h"
#include "bsp_os.h"
#include "can.h"
#include "rgb.h"
#include "chassis_task.h"
#include "arm_translate_task.h"
#include "arm.h"
//  #include "ui_task.h"
#ifdef REFEREE
#include "referee_task.h"
#endif
#include "bsp_print.h"

#ifdef USING_DBUS
#include "dbus.h"
#else
#include "sbus.h"
#endif
//
osThreadId_t chassisTaskHandle;
osThreadId_t armTranslateTaskHandle;
osThreadId_t armA1TaskHandle;
osThreadId_t UITaskHandle;
osThreadId_t testTaskHandle;
#ifdef REFEREE
osThreadId_t refereeTaskHandle;
#endif


bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;
static bool engineerIsKilled = false;

#ifdef USING_DBUS
remote::DBUS* dbus = nullptr;
#else
remote::SBUS* sbus = nullptr;
#endif
#ifdef REFEREE
 RefereeUART* referee_uart = nullptr;
 communication::Referee* referee = nullptr;
#endif





void RM_RTOS_Init() {
    print_use_uart(&huart4);
    bsp::SetHighresClockTimer(&htim5);
#ifdef USING_DBUS
    dbus = new remote::DBUS(&huart3);
#else
    sbus = new remote::SBUS(&huart3);
#endif
    can1 = new bsp::CAN(&hcan1, true);
    can2 = new bsp::CAN(&hcan2, false);
#ifdef REFEREE
   referee_uart = new RefereeUART(&huart6);
   referee = new communication::Referee();
#endif

#ifdef CHASSIS
    init_chassis();
    // init_arm_translate();
#endif

#ifdef ARM_A1
    init_arm_A1();
#endif
    set_cursor(0,0);
    clear_screen();
}


void RM_RTOS_Threads_Init(void) {
#ifdef CHASSIS
    chassisTaskHandle = osThreadNew(chassisTask,nullptr,&chassisTaskAttribute);
    // armTranslateTaskHandle = osThreadNew(armTranslateTask, nullptr, &armTranslateAttribute);
#endif

#ifdef ARM_A1
    armA1TaskHandle = osThreadNew(armTask, nullptr, &armA1TaskAttribute);
#endif

#ifdef REFEREE
    refereeTaskHandle = osThreadNew(refereeTask,nullptr,&refereeTaskAttribute);
#endif
//     UITaskHandle = osThreadNew(UITask,nullptr,&UITaskAttribute);
}


void KillAll() {
#ifdef CHASSIS
    kill_chassis();
    // kill_arm_translate();

#endif

#ifdef ARM_A1
    kill_arm();
#endif

}

void ReviveAll(){
#ifdef CHASSIS

#endif

#ifdef ARM_A1
    revive_arm();
#endif
}


void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    while(true){ //if want to print, make sure nothing is print somewhere else
        if(sbus->ch[6]>100){
            if(!engineerIsKilled){
                print("killed\n");
            }
            engineerIsKilled = true;
            KillAll();
            // print("killed");
            osDelay(10);
        }else if(engineerIsKilled && sbus->ch[6]<=100){ // killed to revive
            ReviveAll();
            engineerIsKilled = false;
        }

#ifdef REFEREE
        print("ROBOTID: %d",referee->game_robot_status.robot_id);
#endif
        osDelay(10);
    }
}
