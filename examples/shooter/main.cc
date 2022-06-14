/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2022 RoboMaster.                                          *
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

#include "bsp_gpio.h"
#include "bsp_laser.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "shooter.h"

#define LASER_Pin GPIO_PIN_13
#define LASER_GPIO_Port GPIOG

bsp::CAN* can = nullptr;
control::MotorCANBase* left_flywheel_motor = nullptr;
control::MotorCANBase* right_flywheel_motor = nullptr;
control::MotorCANBase* load_motor = nullptr;

remote::DBUS* dbus = nullptr;
control::ServoMotor* load_servo = nullptr;
control::Shooter* shooter = nullptr;

void RM_RTOS_Init() {
  dbus = new remote::DBUS(&huart3);

  can = new bsp::CAN(&hcan1, 0x201);
  left_flywheel_motor = new control::Motor3508(can, 0x201);
  right_flywheel_motor = new control::Motor3508(can, 0x202);
  load_motor = new control::Motor3508(can, 0x203);

  control::shooter_t shooter_data;
  shooter_data.left_flywheel_motor = left_flywheel_motor;
  shooter_data.right_flywheel_motor = right_flywheel_motor;
  shooter_data.load_motor = load_motor;
  shooter_data.model = control::SHOOTER_STANDARD_2022;
  shooter = new control::Shooter(shooter_data);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  osDelay(500);  // DBUS initialization needs time

  control::MotorCANBase* motors[] = {left_flywheel_motor, right_flywheel_motor, load_motor};
  bsp::GPIO laser(LASER_GPIO_Port, LASER_Pin);
  laser.High();

  while (true) {
    if (dbus->swr == remote::DOWN) shooter->SetFlywheelSpeed(500);
    if (dbus->swr == remote::UP) shooter->SetFlywheelSpeed(0);
    if (dbus->ch3 > 500) shooter->LoadNext();
    shooter->Update();
    control::MotorCANBase::TransmitOutput(motors, 3);
    osDelay(10);
  }
}