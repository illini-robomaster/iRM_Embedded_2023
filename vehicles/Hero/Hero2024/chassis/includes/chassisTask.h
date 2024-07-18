/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2024 RoboMaster.                                          *
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
#pragma once
#include "cmsis_os.h"
#include "motor.h"
#include "heroSteering.h"
#include "steering_6020.h"
#include "can.h"
#include "rgb.h"
#include "protocol.h"
#include "bsp_can_bridge.h"

#include "dbus.h"

//MAGIC NUMBERS, Represent motor physical install angle offsets.
#define FL_MOTOR_OFFSET 4.663
#define FR_MOTOR_OFFSET 0.771
#define BL_MOTOR_OFFSET 6.110
#define BR_MOTOR_OFFSET 6.065
// #define FL_MOTOR_OFFSET 0
// #define FR_MOTOR_OFFSET 0
// #define BL_MOTOR_OFFSET 0
// #define BR_MOTOR_OFFSET 0


extern osThreadId_t chassisTaskHandle;
const osThreadAttr_t chassisTaskAttribute = {.name = "chassisTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = 512 * 4,
        .priority = (osPriority_t)osPriorityAboveNormal,
        .tz_module = 0,
        .reserved = 0};
void chassisTask(void* arg);
void init_chassis();
void kill_chassis();

extern remote::DBUS* dbus;


extern bsp::CAN* can1;
extern bsp::CAN* can2;
extern display::RGB* RGB;
extern communication::Referee* referee;
extern bsp::CanBridge* receive;


// speed for steering motors (rad/s)
constexpr float RUN_SPEED = (4 * PI);
constexpr float ALIGN_SPEED = (PI);
constexpr float ACCELERATION = (100 * PI);
// speed for chassis rotation (no unit)
constexpr float FOLLOW_SPEED = 40;
constexpr float CHASSIS_DEADZONE = 0.04;
