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
static control::Motor4310* motor1 = nullptr;
static control::Motor4310* motor2 = nullptr;
static control::Motor4310* motor3 = nullptr;
remote::DBUS* dbus = nullptr;

void RM_RTOS_Init() {
  print_use_uart(&huart1);
  can = new bsp::CAN(&hcan1, true);

  /* rx_id = Master id
   * tx_id = CAN id
   * mode:
   *  MIT: MIT mode
   *  POS_VEL: position-velocity mode
   *  VEL: velocity mode  */

  /* Make sure motor is set to the correct mode (in helper tool). Otherwise, motor won't start */
  motor1 = new control::Motor4310(can, 0x02, 0x01, control::MIT);
  motor2 = new control::Motor4310(can, 0x04, 0x03, control::MIT);
  motor3 = new control::Motor4310(can, 0x06, 0x05, control::MIT);
  dbus = new remote::DBUS(&huart3);
}

void RM_RTOS_Default_Task(const void* args) {
  /* press reset if no response */
  UNUSED(args);

  control::Motor4310* motors[] = {motor1, motor2, motor3};

  while(dbus->swr != remote::DOWN){}  // flip swr to start

  /* Use SetZeroPos if you want to set current motor position as zero position. If uncommented, the
   * zero position is the zero position set before */
  motor1->SetZeroPos();
  motor1->MotorEnable();
  motor2->SetZeroPos();
  motor2->MotorEnable();
  motor3->SetZeroPos();
  motor3->MotorEnable();

  float pos_1 = 0;
  float pos_2 = 0;
  float pos_3 = 0;
  float min_pos = -PI;
  float max_pos = PI;
  while (true) {
    float vel_1;
    float vel_2;
    float vel_3;

    vel_1 = clip<float>(dbus->ch0 / 660.0 * 15.0, -15, 15);
    pos_1 += vel_1 / 200;
    pos_1 = clip<float>(pos_1, min_pos, max_pos);   // clipping position within a range

    vel_2 = clip<float>(-dbus->ch3 / 660.0 * 15.0, -15, 15);
    pos_2 += vel_2 / 200;
    pos_2 = clip<float>(pos_2, -PI/2, PI/2);   // clipping position within a range

    vel_3 = clip<float>(dbus->ch2 / 660.0 * 15.0, -15, 15);
    pos_3 += vel_3 / 200;
    pos_3 = clip<float>(pos_3, min_pos, max_pos);   // clipping position within a range

    set_cursor(0, 0);
    clear_screen();
    print("Vel Set: %f  Pos Set: %f\n", vel_1, pos_1);

    motor1->SetOutput(pos_1, vel_1, 30, 0.5, 0);
    motor2->SetOutput(pos_2, vel_2, 30, 0.5, 0);
    motor3->SetOutput(pos_3, vel_3, 30, 0.5, 0);
    control::Motor4310::TransmitOutput(motors, 3);
    osDelay(10);
  }
}