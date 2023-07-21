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
  HAL_Delay(100);
  print_use_usb();
  can = new bsp::CAN(&hcan1, 0x201, true);

  /* rx_id = Master id
   * tx_id = CAN id
   * mode:
   *  MIT: MIT mode
   *  POS_VEL: position-velocity mode
   *  VEL: velocity mode  */

  /* Make sure motor is set to the correct mode (in helper tool). Otherwise, motor won't start */
  motor = new control::Motor4310(can, 0x34, 0x35, control::VEL);
  dbus = new remote::DBUS(&huart3);
  HAL_Delay(100);
}

void RM_RTOS_Default_Task(const void* args) {
  /* press reset if no response */
  UNUSED(args);
  while(dbus->swr != remote::DOWN){}  // flip swr to start

  /* Use SetZeroPos if you want to set current motor position as zero position. If uncommented, the
   * zero position is the zero position set before */
  // motor->SetZeroPos(motor);
  motor->MotorEnable(motor);

  while (true) {
    float vel;
    vel = clip<float>(dbus->ch1 / 660.0 * 30.0, -30, 30);

    set_cursor(0, 0);
    clear_screen();
    // print("Vel: %f \n", vel);
    motor->PrintData();

    motor->SetOutput(vel);
    motor->TransmitOutput(motor);
    osDelay(10);
  }
}
