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
#include "dbus.h"
#include "gimbal.h"
#include "main.h"
#include "shooter.h"

bsp::CAN* can = nullptr;
control::MotorPWMBase* left_fly_motor = nullptr;
control::MotorPWMBase* right_fly_motor = nullptr;
control::MotorCANBase* load_motor = nullptr;
BoolEdgeDetector shoot_detector(false);
control::Shooter* shooter = nullptr;
remote::DBUS* dbus = nullptr;

void RM_RTOS_Init() {
  print_use_uart(&huart8);
  can = new bsp::CAN(&hcan1, 0x207);
  left_fly_motor = new control::MotorPWMBase(&htim4, 1, 1000000, 500, 1080);
  right_fly_motor = new control::MotorPWMBase(&htim4, 2, 1000000, 500, 1080);
  load_motor = new control::Motor2006(can, 0x207);

  control::shooter_t shooter_data;
  shooter_data.left_flywheel_motor = left_fly_motor;
  shooter_data.right_flywheel_motor = right_fly_motor;
  shooter_data.load_motor = load_motor;
  shooter_data.model = control::SHOOTER_SENTRY;
  shooter = new control::Shooter(shooter_data);

  dbus = new remote::DBUS(&huart1);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  control::MotorCANBase* motors[] = {load_motor};

  while (true) {
    // Kill switch for safety measure
    if (dbus->swl == remote::DOWN) RM_ASSERT_TRUE(false, "Operation killed");

    shoot_detector.input(dbus->swr == remote::UP);
    if (shoot_detector.posEdge())
      shooter->SetFlywheelSpeed(150);
    else if (shoot_detector.negEdge())
      shooter->SetFlywheelSpeed(0);

    shooter->Update();
    control::MotorCANBase::TransmitOutput(motors, 1);
    osDelay(10);
  }
}
