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

#include "cmsis_os.h"
#include "main.h"
#include "cybergear.h"
#include "bsp_gpio.h"
#include "bsp_print.h"
#include <cstdlib>

static xiaomi::CAN* can = nullptr;
static xiaomi::CyberGear* cybergear = nullptr;
static bsp::GPIO* key = nullptr;

void RM_RTOS_Init() {
  print_use_usb();

  key = new bsp::GPIO(KEY_GPIO_Port, KEY_Pin);
  can = new xiaomi::CAN(&hcan1);
  cybergear = new xiaomi::CyberGear(can, 126, xiaomi::Motion_mode);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  cybergear->SetZeroPosition();

  while (true) {
    if (key->Read() == 0) {
      cybergear->Stop();
      osDelay(100);
    }

    cybergear->SendMotionCommand(.1, 0., 0., 0., 0.);
    print("angle: %.4f speed: %.4f torque: %.4f temp: %.4f master can ID: %d\r\n",
         cybergear->GetAngle(),
         cybergear->GetSpeed(),
         cybergear->GetTorque(),
         cybergear->GetTemperature(),
         cybergear->GetMasterCanID());
    osDelay(10);
  }
}
