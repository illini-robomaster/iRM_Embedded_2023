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

#include "main.h"
#include <cstring>

#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "utils.h"

#include "bsp_gpio.h"

void delay_us(uint32_t us) {
  uint32_t start = bsp::GetHighresTickMicroSec();
  while (bsp::GetHighresTickMicroSec() - start < us);
}

namespace display {

char font_lib[][5]  ={
 {0x7f,0x7f,0x41,0x7f,0x7f}, // 0
 {0x00,0x7f,0x7f,0x7f,0x00}, // 1
 {0x79,0x79,0x69,0x6f,0x6f}, // 2
 {0x6b,0x6b,0x6b,0x7f,0x7f}, // 3
 {0x1f,0x1f,0x18,0x7f,0x7f}, // 4
 {0x6f,0x6f,0x69,0x79,0x79}, // 5
 {0x7f,0x7f,0x49,0x79,0x79}, // 6
 {0x03,0x03,0x03,0x7f,0x7f}, // 7
 {0x7f,0x7f,0x49,0x7f,0x7f}, // 8
 {0x4f,0x4f,0x49,0x7f,0x7f}, // 9
 {0x00,0x00,0x36,0x36,0x00}, // :
 };

class VFD {
 public:
  VFD(bsp::GPIO* din, bsp::GPIO* clk, bsp::GPIO* cs, bsp::GPIO* rst, bsp::GPIO* en) {
    din_ = din;
    clk_ = clk;
    cs_ = cs;
    rst_ = rst;
    en_ = en;
  }

  void Show() {
    cs_->Low();
    write_data(0xe8);
    cs_->High();
  }

  void Init() {
    en_->High();
    rst_->Low();
    osDelay(5);
    rst_->High();

    cs_->Low();
    write_data(0xe0);
    osDelay(5);
    write_data(0x07);
    cs_->High();
    osDelay(5);

    SetBrightness(100);
  }

  void SetBrightness(int val) {
    val = val / 100.0f * 255;
    val = val < 0? 0 : val;
    val = val > 255? 255 : val;

    cs_->Low();
    write_data(0xe4);
    delay_us(3);
    write_data(val);
    cs_->High();
    delay_us(3);
  }

  void WriteOneChar(uint16_t pos, char chr) {
    if (pos >= 8) return ;

    cs_->Low();
    write_data(0x20 + pos);
    write_data(chr);
    cs_->High();
  }

  void WriteString(uint16_t pos, char* str, int len) {
    if (pos >= 8 || len <= 0 || pos + len > 8) return;

    cs_->Low();
    write_data(0x20 + pos);
    for (int i = 0; i < len; ++i)
      write_data(str[i]);
    cs_->High();
  }

  void Clear() {
    cs_->Low();
    write_data(0x20);
    for (int i = 0; i < 8; ++i)
      write_data(0x20);
    cs_->High();
  }

  void WriteCustom(uint16_t pos, char* str) {
    if (pos >= 8) return ;

    cs_->Low();
    write_data(0x40 + pos);
    for (int i = 0; i < 7; ++i)
      write_data(str[i]);
    cs_->High();

    cs_->Low();
    write_data(0x20 + pos);
    write_data(0x00 + pos);
    cs_->High();
  }

 private:
  bsp::GPIO* din_;
  bsp::GPIO* clk_;
  bsp::GPIO* cs_;
  bsp::GPIO* rst_;
  bsp::GPIO* en_;

  void write_data(unsigned char data) {
    for (int i = 0; i < 8; ++i) {
      if ((data & 0x01) == 0x01)
        din_->High();
      else
        din_->Low();
      clk_->Low();
      delay_us(5);
      data >>= 1;
      clk_->High();
      delay_us(5);
    }
  }
};

}

static bsp::GPIO* led = nullptr;

static bsp::GPIO* ccw = nullptr;
static bsp::GPIO* push = nullptr;
static bsp::GPIO* cw = nullptr;

static bsp::GPIO* din = nullptr;
static bsp::GPIO* clk = nullptr;
static bsp::GPIO* cs = nullptr;
static bsp::GPIO* rst = nullptr;
static bsp::GPIO* en = nullptr;
static display::VFD* vfd = nullptr;

static BoolEdgeDetector left(false);
static BoolEdgeDetector button(false);
static BoolEdgeDetector right(false);

const osThreadAttr_t switchTaskAttribute = {.name = "switchTask",
                                            .attr_bits = osThreadDetached,
                                            .cb_mem = nullptr,
                                            .cb_size = 0,
                                            .stack_mem = nullptr,
                                            .stack_size = 128 * 4,
                                            .priority = (osPriority_t)osPriorityNormal,
                                            .tz_module = 0,
                                            .reserved = 0};
osThreadId_t switchTaskHandle;

void switchTask(void* arg) {
  UNUSED(arg);

  while (true) {
    left.input(ccw->Read());
    button.input(push->Read());
    right.input(cw->Read());

    if (left.negEdge() || button.negEdge() || right.negEdge())
      led->Toggle();

    osDelay(50);
  }
}

void RM_RTOS_Init(void) {
  bsp::SetHighresClockTimer(&htim2);
  print_use_uart(&huart1);

  led = new bsp::GPIO(LED_GPIO_Port, LED_Pin);

  ccw = new bsp::GPIO(CCW_GPIO_Port, CCW_Pin);
  push = new bsp::GPIO(PUSH_GPIO_Port, PUSH_Pin);
  cw = new bsp::GPIO(CW_GPIO_Port, CW_Pin);

  din = new bsp::GPIO(DIN_GPIO_Port, DIN_Pin);
  clk = new bsp::GPIO(CLK_GPIO_Port, CLK_Pin);
  cs = new bsp::GPIO(CS_GPIO_Port, CS_Pin);
  rst = new bsp::GPIO(RST_GPIO_Port, RST_Pin);
  en = new bsp::GPIO(EN_GPIO_Port, EN_Pin);
  vfd = new display::VFD(din, clk, cs, rst, en);
}

void RM_RTOS_Threads_Init(void) {
  switchTaskHandle = osThreadNew(switchTask, nullptr, &switchTaskAttribute);
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);
  char string[] = "WWWWWWWW";
  vfd->Init();

  while (true) {
    vfd->WriteString(0, string, strlen(string));
    vfd->Show();
    osDelay(500);

    vfd->WriteCustom(0, display::font_lib[9]);
    vfd->WriteCustom(1, display::font_lib[1]);
    vfd->WriteCustom(2, display::font_lib[2]);
    vfd->WriteCustom(3, display::font_lib[3]);
    vfd->WriteCustom(4, display::font_lib[4]);
    vfd->WriteCustom(5, display::font_lib[5]);
    vfd->WriteCustom(6, display::font_lib[10]);
    vfd->WriteCustom(7, display::font_lib[7]);
    vfd->Show();
    osDelay(500);
  }
}
