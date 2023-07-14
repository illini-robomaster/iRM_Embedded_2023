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

#include <string.h>

#include "bsp_sdio.h"
#include "cmsis_os.h"

static uint8_t tx1[] = "first message\n";
static uint8_t tx2[] = "second message\n";

static osThreadId_t sd_task_thread;

void sd_task(void* argu) {
  UNUSED(argu);
  bsp::SDFileLogger logger("log.txt");
  logger.Log(tx1, strlen((char*)tx1));
  logger.Log(tx2, strlen((char*)tx2));
  while (true)
    ;
}

const osThreadAttr_t sd_task_thread_attr = {.name = "sdTask",
                                            .attr_bits = osThreadDetached,
                                            .cb_mem = nullptr,
                                            .cb_size = 0,
                                            .stack_mem = nullptr,
                                            .stack_size = 128 * 4,
                                            .priority = (osPriority_t)osPriorityNormal,
                                            .tz_module = 0,
                                            .reserved = 0};

void RM_RTOS_Threads_Init(void) {
  sd_task_thread = osThreadNew(sd_task, NULL, &sd_task_thread_attr);
}
