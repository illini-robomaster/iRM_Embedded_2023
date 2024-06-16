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
#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os2.h"
#include "dbus.h"
#include "main.h"
#include "protocol.h"

#define REFEREE_RX_SIGNAL (1 << 0)


extern osThreadId_t refereeTaskHandle;
const osThreadAttr_t refereeTaskAttribute = {.name = "refereeTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = 1024 * 4,
        .priority = (osPriority_t)osPriorityAboveNormal,
        .tz_module = 0,
        .reserved = 0};

extern communication::Referee* referee;
class RefereeUART : public bsp::UART {
public:
    using bsp::UART::UART;

protected:
    void RxCompleteCallback() final { osThreadFlagsSet(refereeTaskHandle, REFEREE_RX_SIGNAL); }
};
extern RefereeUART* referee_uart;