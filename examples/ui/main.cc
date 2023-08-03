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

#include <stdio.h>
#include <string.h>

#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "protocol.h"
#include "user_interface.h"

#define RX_SIGNAL (1 << 0)

const osThreadAttr_t refereeTaskAttribute = {.name = "refereeTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 128 * 4,
                                             .priority = (osPriority_t)osPriorityNormal,
                                             .tz_module = 0,
                                             .reserved = 0};
osThreadId_t refereeTaskHandle;

class CustomUART : public bsp::UART {
 public:
  using bsp::UART::UART;

 protected:
  /* notify application when rx data is pending read */
  void RxCompleteCallback() final { osThreadFlagsSet(refereeTaskHandle, RX_SIGNAL); }
};

static communication::Referee* referee = nullptr;
static CustomUART* referee_uart = nullptr;

void refereeTask(void* arg) {
  UNUSED(arg);
  uint32_t length;
  uint8_t* data;

  while (true) {
    /* wait until rx data is available */
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & RX_SIGNAL) {  // unnecessary check
      /* time the non-blocking rx / tx calls (should be <= 1 osTick) */
      length = referee_uart->Read(&data);
      referee->Receive(communication::package_t{data, (int)length});
    }
  }
}

static communication::UserInterface* UI = nullptr;
static communication::StringGUI* uptimeGUI = nullptr;

void RM_RTOS_Init(void) {
  print_use_uart(&huart1);

  referee_uart = new CustomUART(&huart6);
  referee_uart->SetupRx(300);
  referee_uart->SetupTx(300);

  referee = new communication::Referee;

  UI = new communication::UserInterface(referee_uart, referee);
}

void RM_RTOS_Threads_Init(void) {
  refereeTaskHandle = osThreadNew(refereeTask, nullptr, &refereeTaskAttribute);
}

void RM_RTOS_Default_Task(const void* argument) {
  UNUSED(argument);
  osDelay(10000);
  UI->SetID(referee->game_robot_status.robot_id);
  osDelay(120);
  char tmp[15] = "Hello";
  uptimeGUI = new communication::StringGUI(UI, tmp, 300, 800);
  uptimeGUI->Init();
  osDelay(200);
  uptimeGUI->InitString();
  osDelay(1000);

  while (true) {
    set_cursor(0, 0);
    clear_screen();
    print("Chassis Volt: %.3f\r\n", referee->power_heat_data.chassis_volt / 1000.0);
    print("Chassis Curr: %.3f\r\n", referee->power_heat_data.chassis_current / 1000.0);
    print("Chassis Power: %.3f\r\n", referee->power_heat_data.chassis_power);
    print("\r\n");
    print("Shooter Cooling Heat: %hu\r\n", referee->power_heat_data.shooter_id1_17mm_cooling_heat);
    print("Bullet Frequency: %hhu\r\n", referee->shoot_data.bullet_freq);
    print("Bullet Speed: %.3f\r\n", referee->shoot_data.bullet_speed);
    float uptime = HAL_GetTick() * 1.0f / 1000;
    memset(tmp, ' ', 15);
    snprintf(tmp, 15, "Uptime: %.3f", uptime);
    uptimeGUI->Update(tmp);
    osDelay(100);
  }
}
