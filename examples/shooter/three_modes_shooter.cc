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

#include "bsp_gpio.h"
#include "bsp_laser.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "shooter.h"
#include "bsp_os.h"

// #define LASER_Pin GPIO_PIN_13
// #define LASER_GPIO_Port GPIOG

static const int DELAY = 350;  // ms
static const float TARGET_SPEED_FLYWHEELS = 50;

bsp::CAN* can = nullptr;
control::MotorCANBase* left_flywheel_motor = nullptr;
control::MotorCANBase* right_flywheel_motor = nullptr;
control::MotorCANBase* load_motor = nullptr;

remote::DBUS* dbus = nullptr;
control::ServoMotor* load_servo = nullptr;
control::Shooter* shooter = nullptr;

void RM_RTOS_Init() {
  print_use_uart(&huart1);

  dbus = new remote::DBUS(&huart3);
  bsp::SetHighresClockTimer(&htim5);

  can = new bsp::CAN(&hcan1, true);
  left_flywheel_motor = new control::Motor3508(can, 0x201);
  right_flywheel_motor = new control::Motor3508(can, 0x202);
  load_motor = new control::Motor2006(can, 0x203);

  control::shooter_t shooter_data;
  shooter_data.left_flywheel_motor = left_flywheel_motor;
  shooter_data.right_flywheel_motor = right_flywheel_motor;
  shooter_data.load_motor = load_motor;
  shooter_data.model = control::SHOOTER_STANDARD;
  shooter = new control::Shooter(shooter_data);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  osDelay(500);  // DBUS initialization needs time

  control::MotorCANBase* motors[] = {left_flywheel_motor, right_flywheel_motor, load_motor};
  // bsp::GPIO laser(LASER_GPIO_Port, LASER_Pin);
  // laser.High();
  uint32_t start_time = 0;
  bool slow_shoot_detect = false;

  while (true) {
    if (dbus->swr != remote::DOWN) {
      shooter->SetFlywheelSpeed(TARGET_SPEED_FLYWHEELS);
    } else {
      shooter->SetFlywheelSpeed(0);
    }
    if (dbus->mouse.r) {
      shooter->FastContinueShoot();
    } else if (dbus->swr == remote::UP || dbus->mouse.l){
      if (bsp::GetHighresTickMicroSec() - start_time > DELAY) {
        shooter->SlowContinueShoot();
      } else {
        if (slow_shoot_detect == false) {
          slow_shoot_detect = true;
          shooter->DoubleShoot();
        }
      }
    } else {
      shooter->DialStop();
      start_time = bsp::GetHighresTickMicroSec();
      slow_shoot_detect = false;
    }

    shooter->Update();
    control::MotorCANBase::TransmitOutput(motors, 3);
    osDelay(1);

  }
}
