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

#define TARGET_SPEED 30

static bsp::CAN* can1 = nullptr;
static bsp::CAN* can2 = nullptr;
static control::Motor4310* yaw_motor = nullptr;
static control::Motor4310* pitch_motor = nullptr;
static control::Motor3508* flywheel_bl = nullptr;
static control::Motor3508* flywheel_br = nullptr;
remote::DBUS* dbus = nullptr;

void RM_RTOS_Init() {
  print_use_uart(&huart1);
  can1 = new bsp::CAN(&hcan1, 0x201, true);
  can2 = new bsp::CAN(&hcan2, 0x205, false);

  /* rx_id = Master id
   * tx_id = CAN id
   * mode:
   *  MIT: MIT mode
   *  POS_VEL: position-velocity mode
   *  VEL: velocity mode  */

  /* Make sure motor is set to the correct mode (in helper tool). Otherwise, motor won't start */
  yaw_motor = new control::Motor4310(can2, 0x01, 0x01, control::MIT);
  pitch_motor = new control::Motor4310(can2, 0x02, 0x02, control::MIT);
  flywheel_bl = new control::Motor3508(can1, 0x202);
  flywheel_br = new control::Motor3508(can1, 0x204);
  dbus = new remote::DBUS(&huart3);
}

void RM_RTOS_Default_Task(const void* args) {
  /* press reset if no response */
  UNUSED(args);
  while(dbus->swr != remote::DOWN){}  // flip swr to start

  control::MotorCANBase* motors[] = {flywheel_bl, flywheel_br};
  control::PIDController pid_bl(20, 15, 30);
  control::PIDController pid_br(20, 15, 30);

  /* Use SetZeroPos if you want to set current motor position as zero position. If uncommented, the
   * zero position is the zero position set before */
  yaw_motor->SetZeroPos(yaw_motor);
  yaw_motor->MotorEnable(yaw_motor);

  pitch_motor->SetZeroPos(pitch_motor);
  pitch_motor->MotorEnable(pitch_motor);

  float yaw_pos = 0, pitch_pos = 0;
  float min_pos = -PI/8;
  float max_pos = PI/6;
  while (true) {
    float yaw_vel, pitch_vel;

    float diff_bl = flywheel_bl->GetOmegaDelta(TARGET_SPEED);
    int16_t out_bl = pid_bl.ComputeConstrainedOutput(diff_bl);
    flywheel_bl->SetOutput(out_bl);
    float diff_br = flywheel_br->GetOmegaDelta(TARGET_SPEED);
    int16_t out_br = pid_br.ComputeConstrainedOutput(diff_br);
    flywheel_br->SetOutput(out_br);

    yaw_vel = clip<float>(dbus->ch2 / 660.0 * 15.0, -15, 15);
    yaw_pos += yaw_vel / 200;
    yaw_pos = clip<float>(yaw_pos, min_pos, max_pos);   // clipping position within a range

    pitch_vel = clip<float>(dbus->ch3 / 660.0 * 15.0, -15, 15);
    pitch_pos += pitch_vel / 200;
    pitch_pos = clip<float>(pitch_pos, min_pos, max_pos);   // clipping position within a range

    set_cursor(0, 0);
    clear_screen();
    print("Vel Set: %f  Pos Set: %f\n", yaw_vel, yaw_pos);

    yaw_motor->SetOutput(yaw_pos, yaw_vel, 30, 0.5, 0);
    pitch_motor->SetOutput(pitch_pos, pitch_vel, 30, 0.5, 0);

    yaw_motor->TransmitOutput(yaw_motor);
    pitch_motor->TransmitOutput(pitch_motor);

    control::MotorCANBase::TransmitOutput(motors, 2);

    osDelay(10);
  }
}
