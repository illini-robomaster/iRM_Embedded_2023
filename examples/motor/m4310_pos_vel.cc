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

#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "motor.h"
#include "dbus.h"

bsp::CAN* can = nullptr;
control::Motor4310* motor = nullptr;
remote::DBUS* dbus = nullptr;

void RM_RTOS_Init() {
  print_use_uart(&huart8);
  can = new bsp::CAN(&hcan1, 0x01, true);
  motor = new control::Motor4310(can, 0x02, 0x01, 1);
  dbus = new remote::DBUS(&huart1);
}

void RM_RTOS_Default_Task(const void* args) {
  // need to press reset to begin
  UNUSED(args);
  //  motor->SetZeroPos4310(motor);
  motor->Initialize4310(motor);

  float pos = 0;
  while (true) {
    float vel;
    vel = clip<float>(dbus->ch1 / 660.0 * 30.0, -30, 30);
    pos += vel / 200;
    pos = clip<float>(pos, -PI/4, PI/4);

    set_cursor(0, 0);
    clear_screen();
    print("Vel Set: %f  Pos Set: %f\n", vel, pos);

    motor->SetOutput4310(pos, vel);
    motor->TransmitOutput4310(motor);
    osDelay(10);
  }
}