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

class VFD {
 public:
  VFD(bsp::GPIO* din, bsp::GPIO* clk, bsp::GPIO* cs, bsp::GPIO* rst, bsp::GPIO* en) {
    din_ = din;
    clk_ = clk;
    cs_ = cs;
    rst_ = rst;
    en_ = en;

    en_->High();
    delay_us(100);
    rst_->Low();
    osDelay(5);
    rst_->High();
  }

  void Show() {
    cs_->Low();
    write_data(0xe8);
    cs_->High();
  }

  void Init() {
    cs_->Low();
    write_data(0xe0);
    osDelay(5);
    write_data(0x07);
    cs_->High();
    osDelay(5);

    cs_->Low();
    write_data(0xe4);
    osDelay(5);
    write_data(0xff);
    cs_->High();
    osDelay(5);
  }

  void WriteOneChar(unsigned char x, unsigned char chr) {
    cs_->Low();
    write_data(0x20 + x);
    write_data(chr);
    cs_->High();
    Show();
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

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);
//  vfd->Init();

  while (true) {
    left.input(ccw->Read());
    button.input(push->Read());
    right.input(cw->Read());

    if (left.negEdge() || button.negEdge() || right.negEdge())
      led->Toggle();
//    vfd->WriteOneChar(0, 0x30);
//    osDelay(5);
//    vfd->Show();
//    osDelay(5);
    din->Toggle();
    delay_us(1000);
//    set_cursor(0, 0);
//    clear_screen();

//    led->Low();
//    print("%ld\r\n", bsp::GetHighresTickMicroSec());
//    osDelay(100);
  }
}
