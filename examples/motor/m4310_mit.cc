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

#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "motor.h"
#include "dbus.h"

static bsp::CAN* can = nullptr;
static control::Motor4310* motor = nullptr;
remote::DBUS* dbus = nullptr;

void RM_RTOS_Init() {
  print_use_uart(&huart1);
  can = new bsp::CAN(&hcan1, 0x01, true);

  /* rx_id = Master id
   * tx_id = CAN id
   * mode:
   *  MIT: MIT mode
   *  POS_VEL: position-velocity mode
   *  VEL: velocity mode  */
  motor = new control::Motor4310(can, 0x02, 0x01, control::MIT);
  dbus = new remote::DBUS(&huart3);
}

void RM_RTOS_Default_Task(const void* args) {
  // need to press reset to begin
  UNUSED(args);
  while(dbus->swr != remote::DOWN){}  // flip swr to start
  motor->SetZeroPos(motor);  // set zero position on startup;
  motor->MotorEnable(motor);

  float pos = 0;
  float min_pos = -PI/8;
  float max_pos = PI/6;
  while (true) {
    float vel;
    vel = clip<float>(dbus->ch1 / 660.0 * 15.0, -15, 15);
    pos += vel / 200;
    pos = clip<float>(pos, min_pos, max_pos);   // clipping position within a range

    set_cursor(0, 0);
    clear_screen();
    print("Vel Set: %f  Pos Set: %f\n", vel, pos);

    motor->SetOutput(pos, vel, 95, 0.5, 0);
    motor->TransmitOutput(motor);
    osDelay(10);
  }
}
