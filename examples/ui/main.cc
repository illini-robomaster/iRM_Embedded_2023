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
#include "dbus.h"
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

static remote::DBUS* dbus = nullptr;

void RM_RTOS_Init(void) {
  print_use_uart(&huart1);

  dbus = new remote::DBUS(&huart3);

  referee_uart = new CustomUART(&huart6);
  referee_uart->SetupRx(300);
  referee_uart->SetupTx(300);

  referee = new communication::Referee;

  UI = new communication::UserInterface(referee_uart, referee);
}

void RM_RTOS_Threads_Init(void) {
  refereeTaskHandle = osThreadNew(refereeTask, nullptr, &refereeTaskAttribute);
}

static communication::Bar* pitchBar = nullptr;
static communication::CrossairGUI* crossairGUI = nullptr;
static communication::CapGUI* forceGUI = nullptr;
static communication::CapGUI* reloadGUI = nullptr;
static communication::StringGUI* modeGUI = nullptr;

void RM_RTOS_Default_Task(const void* argument) {
  UNUSED(argument);
  osDelay(3000);
  UI->SetID(referee->game_robot_status.robot_id);
  osDelay(120);
  communication::graphic_data_t data[7];
  char tmp[15] = "PITCH         ";
  pitchBar = new communication::Bar(1830, 440, 30, 400,
                                           UI_Color_Green, UI_Color_Pink, true);
  data[0] = pitchBar->Init();
  data[1] = pitchBar->InitFrame();
  UI->FloatDraw(&data[2],"PI",UI_Graph_Add,1,UI_Color_Cyan,20,3,4,1600,800,114514);
  UI->GraphRefresh(5,data[0],data[1],data[2],data[0],data[1]);
  osDelay(110);
  crossairGUI = new communication::CrossairGUI(UI);
  osDelay(110);
  strcpy(tmp, "FORCE         ");
  forceGUI = new communication::CapGUI(UI,tmp,1500,460,310,20);
  osDelay(110);
  forceGUI->InitName();
  osDelay(110);
  strcpy(tmp, "RELOAD        ");
  reloadGUI = new communication::CapGUI(UI,tmp,1500,405,310,20);
  osDelay(110);
  reloadGUI->InitName();
  osDelay(110);
  strcpy(tmp, "MODE          ");
  modeGUI = new communication::StringGUI(UI,tmp,1550,370,UI_Color_Orange,20);
  modeGUI->Init();
  osDelay(110);
  modeGUI->InitString();
  osDelay(110);

  while (true) {
    set_cursor(0, 0);
    clear_screen();
    float pitch_val = dbus->ch2 / 660.0f / 2.0f + 0.5f;
    data[0] = pitchBar->Update(pitch_val);
    float reload_val = dbus->ch1 / 660.0f / 2.0f + 0.5f;
    reloadGUI->UpdateBulk(reload_val,&data[1],&data[2]);
    float force_val = dbus->ch3 / 660.0f / 2.0f + 0.5f;
    forceGUI->UpdateBulk(force_val,&data[3],&data[4]);
    UI->GraphRefresh(5,data[0],data[1],data[2],data[3],data[4]);
    osDelay(110);
  }
}
