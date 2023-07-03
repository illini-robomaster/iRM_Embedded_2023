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

#include "bsp_gpio.h"
#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "main.h"
#include "protocol.h"

#define RX_SIGNAL (1 << 0)

const osThreadAttr_t clientTaskAttribute = {.name = "clientTask",
                                            .attr_bits = osThreadDetached,
                                            .cb_mem = nullptr,
                                            .cb_size = 0,
                                            .stack_mem = nullptr,
                                            .stack_size = 128 * 4,
                                            .priority = (osPriority_t)osPriorityNormal,
                                            .tz_module = 0,
                                            .reserved = 0};
osThreadId_t clientTaskHandle;

class CustomUART : public bsp::UART {
 public:
  using bsp::UART::UART;

 protected:
  /* notify application when rx data is pending read */
  void RxCompleteCallback() final { osThreadFlagsSet(clientTaskHandle, RX_SIGNAL); }
};

static communication::Host* host = nullptr;
static CustomUART* host_uart = nullptr;
static bsp::GPIO *gpio_red, *gpio_green;

void clientTask(void* arg) {
  UNUSED(arg);
  uint32_t length;
  uint8_t* data;

  while (true) {
    /* wait until rx data is available */
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & RX_SIGNAL) {  // unnecessary check
      /* time the non-blocking rx / tx calls (should be <= 1 osTick) */
      length = host_uart->Read(&data);
      host->Receive(communication::package_t{data, (int)length});
      gpio_green->Low();
      osDelay(200);
      gpio_green->High();
    }
  }
}

void RM_RTOS_Init(void) {
  print_use_uart(&huart8);

  host_uart = new CustomUART(&huart6);
  host_uart->SetupRx(300);
  host_uart->SetupTx(300);

  host = new communication::Host;

  gpio_red = new bsp::GPIO(LED_RED_GPIO_Port, LED_RED_Pin);
  gpio_green = new bsp::GPIO(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
  gpio_red->High();
  gpio_green->High();
}

void RM_RTOS_Threads_Init(void) {
  clientTaskHandle = osThreadNew(clientTask, nullptr, &clientTaskAttribute);
}

void RM_RTOS_Default_Task(const void* argument) {
  UNUSED(argument);

  while (true) {
    set_cursor(0, 0);
    clear_screen();
    print("%s", host->pack.chars);
    osDelay(100);
  }
}
