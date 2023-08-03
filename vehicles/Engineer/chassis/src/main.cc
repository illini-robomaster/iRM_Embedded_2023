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

#include "bsp_os.h"
#include "can.h"
#include "dbus.h"
#include "rgb.h"
#include "chassis_task.h"
// #include "ui_task.h"
// #include "referee_task.h"

osThreadId_t chassisTaskHandle;
osThreadId_t UITaskHandle;
osThreadId_t refereeTaskHandle;


bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;

remote::DBUS* dbus = nullptr;
// display::RGB* RGB = nullptr;

// RefereeUART* referee_uart = nullptr;
// communication::Referee* referee = nullptr;




void RM_RTOS_Init() {
    print_use_uart(&huart6);
    bsp::SetHighresClockTimer(&htim5);

    dbus = new remote::DBUS(&huart3);

    can1 = new bsp::CAN(&hcan1, true);
    can2 = new bsp::CAN(&hcan2, false);
    // RGB = new display::RGB(&htim5, 3, 2, 1, 1000000);

    // referee_uart = new RefereeUART(&huart6);
    // referee = new communication::Referee();



    init_chassis();
}


void RM_RTOS_Threads_Init(void) {
    chassisTaskHandle = osThreadNew(chassisTask,nullptr,&chassisTaskAttribute);
    // refereeTaskHandle = osThreadNew(refereeTask,nullptr,&refereeTaskAttribute);
    // UITaskHandle = osThreadNew(UITask,nullptr,&UITaskAttribute);
}


void KillAll() {

    kill_chassis();
}


void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
}
