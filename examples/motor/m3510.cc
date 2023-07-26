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

static bsp::CAN* can = nullptr;
static control::MotorCANBase* motor = nullptr;

void RM_RTOS_Init() {
  print_use_uart(&huart1);
  can = new bsp::CAN(&hcan1, true);
  motor = new control::Motor3510(can, 0x205);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    control::MotorCANBase* motors[] = {motor};
    while (true) {
        motor->SetOutput(2000);
        motor->PrintData();
        control::MotorCANBase::TransmitOutput(motors, 1);
        osDelay(100);
    }
}
