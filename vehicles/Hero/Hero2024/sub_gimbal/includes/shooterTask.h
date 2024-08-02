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

#include "chassis.h"

#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "bsp_relay.h"
#include "cmsis_os.h"
#include "controller.h"
#include "dbus.h"
#include "motor.h"
#include "protocol.h"
#include "bsp_can_bridge.h"
#include "rgb.h"
#include "oled.h"
#include "bsp_buzzer.h"

// parameters: could be proved useless  (╯‵□′)╯︵┻━┻
#define FORCE_0_ANGLE (0)
#define FORCE_1_ANGLE (2 * PI)
#define FORCE_2_ANGLE (4 * PI)
#define FORCE_3_ANGLE (6 * PI)

#define AMMO_HEAT 100
// TODO: actual value still required

extern bsp::Buzzer *buzzer;


extern osThreadId_t shooterTaskHandle;
const osThreadAttr_t shooterTaskAttribute = {.name = "shooter_task",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = 256 * 4,
        .priority = (osPriority_t)osPriorityRealtime,
        .tz_module = 0,
        .reserved = 0};



extern remote::DBUS* dbus;
extern bsp::CAN* can1;
extern bsp::CAN* can2;
extern bsp::GPIO* key;
extern bsp::CanBridge* with_gimbal;
extern bsp::CanBridge* with_chassis;

;

extern communication::Referee* referee;


void shooter_task(void* arg);
void init_shooter();
void kill_shooter();