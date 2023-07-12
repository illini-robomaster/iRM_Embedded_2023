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
#include <cstdlib>

#include "bsp_os.h"
#include "cmsis_os.h"
#include "utils.h"

#include "bsp_gpio.h"
#include "bsp_uart.h"
#include "DS3231.h"
#include "ESP8266.h"

void delay_us(uint32_t us) {
  uint32_t start = bsp::GetHighresTickMicroSec();
  while (bsp::GetHighresTickMicroSec() - start < us);
}

namespace display {

struct {
  int second = 0;
  int minute = 0;
  int hour = 0;
  int day = 0;
  int date = 0;
  int month = 0;
  int year = 0;
} time;

void UpdateTime() {
  ++time.second;
  if (time.second >= 60) {
    time.second = 0;
    ++time.minute;
    if (time.minute >= 60) {
      time.minute = 0;
      ++time.hour;
      if (time.hour >= 24) {
        time.hour = 0;
      }
    }
  }
}

int num2ascii = 6;

enum font_idx {
  NUL,
  SOH,
  STX,
  ETX,
  EOT,
  ENQ,
  ACK,
  BEL,
  BS,
  HT,
  LF,
  VT,
  FF,
  CR,
  SO,
  SI,
  DLE,
  DC1,
  DC2,
  DC3,
  DC4,
  NAK,
  SYN,
  ETB,
  CAN,
  EM,
  SUB,
  ESC,
  FS,
  GS,
  RS,
  US,
  SP = 0,
  EXCL,
  QUOT,
  NUM,
  DOLLAR,
  PERCNT,
  AMP,
  APOS,
  LPAREN = 1,
  RPAREN = 2,
  AST = 3,
  PLUS,
  COMMA = 4,
  MINUS,
  PERIOD,
  SOL = 5,
  zero = 6,
  one,
  two,
  three,
  four,
  five,
  six,
  seven,
  eight,
  nine,
  COLON = 16,
  SEMI,
  LT = 17,
  EQUALS,
  GT = 18,
  QUEST,
  COMMAT,
  A,
  B,
  C,
  D,
  E = 19,
  F,
  G = 20,
  H = 21,
  I = 22,
  J,
  K = 23,
  L = 24,
  M,
  N = 25,
  O = 26,
  P,
  Q,
  R,
  S = 27,
  T = 28,
  U,
  V,
  W,
  X,
  Y = 29,
  Z,
  LSQB,
  BSOL,
  RSQB,
  HAT,
  LOWBAR,
  GRAVE = 30,
  a,
  b,
  c,
  d,
  e = 31,
  f,
  g,
  h,
  i = 32,
  j,
  k,
  l,
  m,
  n,
  o = 33,
  p,
  q,
  r,
  s,
  t,
  u,
  v,
  w,
  x,
  y,
  z,
  LCUB,
  VERBAR = 34,
  RCUB,
  TILDE,
  DEL,

  Phi = 35,
  Omega = 36,
  Dot = 37,
};

char font_lib[][5] = {
//    {}, // NUL
//    {}, // SOH
//    {}, // STX
//    {}, // ETX
//    {}, // EOT
//    {}, // ENQ
//    {}, // ACK
//    {}, // BEL
//    {}, // BS
//    {}, // HT
//    {}, // LF
//    {}, // VT
//    {}, // FF
//    {}, // CR
//    {}, // SO
//    {}, // SI
//    {}, // DLE
//    {}, // DC1
//    {}, // DC2
//    {}, // DC3
//    {}, // DC4
//    {}, // NAK
//    {}, // SYN
//    {}, // ETB
//    {}, // CAN
//    {}, // EM
//    {}, // SUB
//    {}, // ESC
//    {}, // FS
//    {}, // GS
//    {}, // RS
//    {}, // US
    {0x00, 0x00, 0x00, 0x00, 0x00}, // SP
//    {0x00, 0x00, 0x4f, 0x00, 0x00}, // !
//    {}, // "
//    {}, // #
//    {}, // $
//    {}, // %
//    {}, // &
//    {}, // '
    {}, // (
    {}, // )
    {}, // *
//    {}, // +
    {}, // ,
//    {}, // -
//    {}, // .
    {}, // /
    {0x7f, 0x7f, 0x41, 0x7f, 0x7f}, // 0
    {0x00, 0x7f, 0x7f, 0x7f, 0x00}, // 1
    {0x79, 0x79, 0x69, 0x6f, 0x6f}, // 2
    {0x6b, 0x6b, 0x6b, 0x7f, 0x7f}, // 3
    {0x1f, 0x1f, 0x18, 0x7f, 0x7f}, // 4
    {0x6f, 0x6f, 0x69, 0x79, 0x79}, // 5
    {0x7f, 0x7f, 0x49, 0x79, 0x79}, // 6
    {0x03, 0x03, 0x03, 0x7f, 0x7f}, // 7
    {0x7f, 0x7f, 0x49, 0x7f, 0x7f}, // 8
    {0x4f, 0x4f, 0x49, 0x7f, 0x7f}, // 9
    {0x00, 0x00, 0x36, 0x36, 0x00}, // :
//    {}, // ;
    {}, // <
//    {}, // =
    {}, // >
//    {}, // ?
//    {}, // @
//    {}, // A
//    {}, // B
//    {}, // C
//    {}, // D
    {}, // E
//    {}, // F
    {}, // G
    {}, // H
    {}, // I
//    {}, // J
    {}, // K
    {}, // L
//    {}, // M
    {}, // N
    {}, // O
//    {}, // P
//    {}, // Q
//    {}, // R
    {}, // S
    {}, // T
//    {}, // U
//    {}, // V
//    {}, // W
//    {}, // X
    {}, // Y
//    {}, // Z
//    {}, // [
//    {}, // \/
//    {}, // ]
//    {}, // ^
//    {}, // _
    {}, // `
//    {}, // a
//    {}, // b
//    {}, // c
//    {}, // d
    {}, // e
//    {}, // f
//    {}, // g
//    {}, // h
    {}, // i
//    {}, // j
//    {}, // k
//    {}, // l
//    {}, // m
//    {}, // n
    {}, // o
//    {}, // p
//    {}, // q
//    {}, // r
//    {}, // s
//    {}, // t
//    {}, // u
//    {}, // v
//    {}, // w
//    {}, // x
//    {}, // y
//    {}, // z
//    {}, // {
    {}, // |
//    {}, // }
//    {}, // ~
//    {}, // DEL

    {}, // Φ
    {}, // ω
    {}, // ･
 };

char vfd_buffer[8][5] = {};

class VFD {
 public:
  VFD(bsp::GPIO* din, bsp::GPIO* clk, bsp::GPIO* cs, bsp::GPIO* rst, bsp::GPIO* en) {
    din_ = din;
    clk_ = clk;
    cs_ = cs;
    rst_ = rst;
    en_ = en;

    srand(bsp::GetHighresTickMicroSec());
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
    brightness_ = val;
    brightness_ = brightness_ < 0? 0 : brightness_;
    brightness_ = brightness_ > 100? 100 : brightness_;

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

  void WriteOneChar(unsigned pos, char chr) {
    if (pos >= 8) return ;

    cs_->Low();
    write_data(0x20 + pos);
    write_data(chr);
    cs_->High();
  }

  void WriteString(unsigned pos, char* str, int len) {
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

  void WriteCustom(unsigned pos, char* str) {
    if (pos >= 8) return ;

    cs_->Low();
    write_data(0x40 + pos);
    for (int i = 0; i < 5; ++i)
      write_data(str[i]);
    cs_->High();

    cs_->Low();
    write_data(0x20);
    for (int i = 0; i < 8; ++i)
      write_data(0x00 + i);
    cs_->High();
  }

  void Font2Buffer(char* font, char* buffer) {
    memcpy(buffer, font, sizeof(char) * 5);
  }

  unsigned GetRandomFont() {
    return rand() % (sizeof(font_lib) / sizeof(char) / 5);
  }

  void VerticalMoveDown(unsigned pos, int step, unsigned new_font) {
    for (int i = 0; i < 5; ++i) {
        vfd_buffer[pos][i] = 0x7f & (vfd_buffer[pos][i] << 1);
        vfd_buffer[pos][i] |= font_lib[new_font][i] >> (7 - step);
    }
  }

  void GetTime() {
    NeedUpdate[0] = time.second % 10 != second_ % 10;
    NeedUpdate[1] = time.second / 10 != second_ / 10;
    NeedUpdate[2] = time.minute % 10 != minute_ % 10;
    NeedUpdate[3] = time.minute / 10 != minute_ / 10;
    NeedUpdate[4] = time.hour % 10 != hour_ % 10;
    NeedUpdate[5] = time.hour / 10 != hour_ / 10;

    UpdateFlag = NeedUpdate[0] || NeedUpdate[1] || NeedUpdate[2] || NeedUpdate[3] || NeedUpdate[4] || NeedUpdate[5];

    hour_ = time.hour;
    minute_ = time.minute;
    second_ = time.second;
  }

  bool NeedUpdate[8] = {};

  bool UpdateFlag = false;

  int hour_ = 0;
  int minute_ = 0;
  int second_ = 0;

  int brightness_ = 0;

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

static time::DS3231* clock = nullptr;

static bsp::UART* uart = nullptr;
static wifi::ESP8266* ESP8266 = nullptr;

static BoolEdgeDetector left(false);
static BoolEdgeDetector button(false);
static BoolEdgeDetector right(false);

static volatile bool clock_flag = false;

const osThreadAttr_t switchTaskAttribute = {.name = "switchTask",
                                            .attr_bits = osThreadDetached,
                                            .cb_mem = nullptr,
                                            .cb_size = 0,
                                            .stack_mem = nullptr,
                                            .stack_size = 32 * 4,
                                            .priority = (osPriority_t)osPriorityLow,
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

    if (ccw->Read())
      vfd->brightness_ -= 2;

    if (cw->Read())
      vfd->brightness_ += 2;

    osDelay(50);
  }
}

const osThreadAttr_t updateTimeTaskAttribute = {.name = "updateTimeTask",
    .attr_bits = osThreadDetached,
    .cb_mem = nullptr,
    .cb_size = 0,
    .stack_mem = nullptr,
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityHigh,
    .tz_module = 0,
    .reserved = 0};
osThreadId_t updateTimeTaskHandle;

void i2c_reset() {
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    for (int i = 0; i < 10; ++i) {
        if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET)
            break;
        else
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_SET);
        osDelay(10);
    }

    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    hi2c2.Instance->CR1 |= I2C_CR1_SWRST;
    hi2c2.Instance->CR1 &= ~I2C_CR1_SWRST;

    MX_I2C2_Init();
}

void updateTimeTask(void* arg) {
  UNUSED(arg);

  while (!clock->IsReady()) {
    clock_flag = false;
    i2c_reset();
    osDelay(100);
  }

//  clock->SetTime(23, 7, 11, 2, 2, 15, 20);

  while (true) {
    while (!clock->ReadTime()) {
            i2c_reset();
            osDelay(100);
    }
    clock_flag = true;

    display::time.second = clock->second;
    display::time.minute = clock->minute;
    display::time.hour = clock->hour;
    display::time.day = clock->day;
    display::time.date = clock->date;
    display::time.month = clock->month;
    display::time.year = clock->year;

    osDelay(100);
  }
}

//const osThreadAttr_t WiFiTaskAttribute = {.name = "WiFiTask",
//                                          .attr_bits = osThreadDetached,
//                                          .cb_mem = nullptr,
//                                          .cb_size = 0,
//                                          .stack_mem = nullptr,
//                                          .stack_size = 128 * 4,
//                                          .priority = (osPriority_t)osPriorityNormal,
//                                          .tz_module = 0,
//                                          .reserved = 0};
//osThreadId_t WiFiTaskHandle;
//
//void WiFiTask(void* arg) {
//  UNUSED(arg);
//
//  led->High();
//
//  ESP8266->IsReady();
//  osDelay(20);
//
//  while (!ESP8266->OK()) {
//    ESP8266->IsReady();
//    osDelay(20);
//  }
//
//  ESP8266->SetMode();
//  osDelay(20);
//
//  while (!ESP8266->OK())
//    osDelay(20);
//
//  ESP8266->Reset();
//  osDelay(20);
//
//  while (!ESP8266->OK())
//    osDelay(20);
//
//  ESP8266->ConnectWiFi();
//  osDelay(1000);
//
//  while (!ESP8266->OK())
//    osDelay(20);
//
//  ESP8266->SetConnectionType();
//  osDelay(300);
//
//  while (!ESP8266->OK())
//    osDelay(20);
//
//  led->Low();
//
//  while (true) {
//    ESP8266->IsReady();
//    osDelay(1000);
//  }
//}

const osThreadAttr_t displayTaskAttribute = {.name = "displayTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 128 * 4,
                                             .priority = (osPriority_t)osPriorityNormal,
                                             .tz_module = 0,
                                             .reserved = 0};
osThreadId_t displayTaskHandle;

void displayTask(void* arg) {
  UNUSED(arg);

  while (!clock_flag)
    osDelay(100);
  vfd->GetTime();

  vfd->Font2Buffer(display::font_lib[vfd->second_ % 10 + display::num2ascii], display::vfd_buffer[7]);
  vfd->Font2Buffer(display::font_lib[vfd->second_ / 10 + display::num2ascii], display::vfd_buffer[6]);

  vfd->Font2Buffer(display::font_lib[display::COLON], display::vfd_buffer[5]);

  vfd->Font2Buffer(display::font_lib[vfd->minute_ % 10 + display::num2ascii], display::vfd_buffer[4]);
  vfd->Font2Buffer(display::font_lib[vfd->minute_ / 10 + display::num2ascii], display::vfd_buffer[3]);

  vfd->Font2Buffer(display::font_lib[display::COLON], display::vfd_buffer[2]);

  vfd->Font2Buffer(display::font_lib[vfd->hour_ % 10 + display::num2ascii], display::vfd_buffer[1]);
  vfd->Font2Buffer(display::font_lib[vfd->hour_ / 10 + display::num2ascii], display::vfd_buffer[0]);

  while (true) {
    vfd->GetTime();

    for (int i = 1; i <= 7; ++i) {
      if (vfd->NeedUpdate[0])
        vfd->VerticalMoveDown(7, i, vfd->second_ % 10 + display::num2ascii);
      if (vfd->NeedUpdate[1])
        vfd->VerticalMoveDown(6, i, vfd->second_ / 10 + display::num2ascii);
      if (vfd->NeedUpdate[2])
        vfd->VerticalMoveDown(4, i, vfd->minute_ % 10 + display::num2ascii);
      if (vfd->NeedUpdate[3])
        vfd->VerticalMoveDown(3, i, vfd->minute_ / 10 + display::num2ascii);
      if (vfd->NeedUpdate[4])
        vfd->VerticalMoveDown(1, i, vfd->hour_ % 10 + display::num2ascii);
      if (vfd->NeedUpdate[5])
        vfd->VerticalMoveDown(0, i, vfd->hour_ / 10 + display::num2ascii);
      if (vfd->UpdateFlag)
        osDelay(500 / 7);
    }

    osDelay(500);
  }
}

void RM_RTOS_Init(void) {
  bsp::SetHighresClockTimer(&htim2);

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

  clock = new time::DS3231(&hi2c2);

  uart = new bsp::UART(&huart1);
  uart->SetupRx(300);
  uart->SetupTx(300);
  ESP8266 = new wifi::ESP8266(uart);
}

void RM_RTOS_Threads_Init(void) {
  switchTaskHandle = osThreadNew(switchTask, nullptr, &switchTaskAttribute);
  updateTimeTaskHandle = osThreadNew(updateTimeTask, nullptr, &updateTimeTaskAttribute);
//  WiFiTaskHandle = osThreadNew(WiFiTask, nullptr, &WiFiTaskAttribute);
  displayTaskHandle = osThreadNew(displayTask, nullptr, &displayTaskAttribute);
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);
  vfd->Init();

  while (true) {
    vfd->SetBrightness(vfd->brightness_);

    vfd->WriteCustom(0, display::vfd_buffer[0]);
    vfd->WriteCustom(1, display::vfd_buffer[1]);
    vfd->WriteCustom(2, display::vfd_buffer[2]);
    vfd->WriteCustom(3, display::vfd_buffer[3]);
    vfd->WriteCustom(4, display::vfd_buffer[4]);
    vfd->WriteCustom(5, display::vfd_buffer[5]);
    vfd->WriteCustom(6, display::vfd_buffer[6]);
    vfd->WriteCustom(7, display::vfd_buffer[7]);

    vfd->Show();
    osDelay(10);
  }
}
