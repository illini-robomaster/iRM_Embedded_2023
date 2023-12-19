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
#include "cmsis_os.h"
#include "main.h"
#include "utils.h"

#define KEY_GPIO_GROUP GPIOB
#define KEY_GPIO_PIN GPIO_PIN_2

#define VX_READY_SIGNAL (1 << 1)

/* Event flag example
 * Press user key on type A board and the green led should toggle.
 *
 * The difference between event flag VS thread flag is:
 * Event flag: thread -> event flag -> N threads
 * Thread flag: thread -> specific thread
 */

static bsp::GPIO *gpio_green;
BoolEdgeDetector key_detector(false);

osEventFlagsId_t key_event_flag;

static osThreadId_t LED_GREEN_TaskHandle;

const osThreadAttr_t LED2Task_attributes = {.name = "LEDGreenTask",
                                            .attr_bits = osThreadDetached,
                                            .cb_mem = nullptr,
                                            .cb_size = 0,
                                            .stack_mem = nullptr,
                                            .stack_size = 128 * 4,
                                            .priority = (osPriority_t)osPriorityNormal,
                                            .tz_module = 0,
                                            .reserved = 0};

void LED_GREEN_Task(void* argument) {
  UNUSED(argument);
  uint32_t flags;
  while (true) {

    flags = osEventFlagsWait(key_event_flag, VX_READY_SIGNAL, osFlagsWaitAny, osWaitForever);

    if (flags & VX_READY_SIGNAL) {
      print("\r\nOK!\r\n");
      gpio_green->Toggle();
      osDelay(200);
    }
  }
}

void RM_RTOS_Threads_Init(void) {
  LED_GREEN_TaskHandle = osThreadNew(LED_GREEN_Task, nullptr, &LED2Task_attributes);
}

void RM_RTOS_Init(void) {
  print_use_uart(&huart8);
  gpio_green = new bsp::GPIO(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
  gpio_green->Low();

  key_event_flag = osEventFlagsNew(nullptr);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  bsp::GPIO key(KEY_GPIO_GROUP, KEY_GPIO_PIN);
  while (true) {
    key_detector.input(key.Read());
    if (key_detector.posEdge()) {

      osEventFlagsSet(key_event_flag, VX_READY_SIGNAL);

      print("event flag sent!");
    }
  }
}
