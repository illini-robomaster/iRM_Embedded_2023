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
#include "bsp_can_bridge.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "bsp_relay.h"
#include "cmsis_os.h"
#include "controller.h"
#include "dbus.h"
#include "motor.h"
#include "protocol.h"
#include "rgb.h"
#include "oled.h"
#include "bsp_buzzer.h"
#include "encoder.h"

extern osThreadId_t gimbalTaskHandle;

const osThreadAttr_t gimbalTaskAttribute = {.name = "gimbal_task",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = 512 * 4,
        .priority = (osPriority_t)osPriorityRealtime,
        .tz_module = 0,
        .reserved = 0};


extern remote::DBUS* dbus;
extern bsp::CAN* can1;
extern bsp::CAN* can2;
extern bsp::CanBridge* with_chassis;
extern bsp::CanBridge* with_shooter;
extern bsp::GPIO* key;
extern bsp::GPIO* forward_key;
extern bsp::GPIO* backward_key;

extern volatile bool Dead;
extern BoolEdgeDetector lob_mode_sw;
extern volatile bool lob_mode;






void gimbal_task(void *arg);
void init_gimbal();
void kill_gimbal();