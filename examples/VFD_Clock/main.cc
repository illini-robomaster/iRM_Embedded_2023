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
  one = 7,
  two = 8,
  three = 9,
  four = 10,
  five = 11,
  six = 12,
  seven = 13,
  eight = 14,
  nine = 15,
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
  BSOL = 30,
  RSQB,
  HAT = 31,
  LOWBAR = 32,
  GRAVE = 33,
  a,
  b,
  c,
  d,
  e = 34,
  f,
  g,
  h,
  i = 35,
  j,
  k,
  l,
  m,
  n,
  o = 36,
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
  VERBAR = 37,
  RCUB,
  TILDE,
  DEL,

  Phi = 38,
  Omega = 39,
  Dot = 40,
  Intersection = 41,
  Angle = 42,
  Waa = 43,
  Left_Corner_Bracket = 44,
  Right_Corner_Bracket = 45,
  Sigma = 46,
  Hiragana = 47,
  Degree = 48,
  Cyrillic = 49,
  Bold_LT = 50,
  Bold_GT = 51,
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
//    {}, // !
//    {}, // "
//    {}, // #
//    {}, // $
//    {}, // %
//    {}, // &
//    {}, // '
    {0x00, 0x1c, 0x22, 0x41, 0x00}, // (
    {0x00, 0x41, 0x22, 0x1c, 0x00}, // )
    {0x00, 0x14, 0x08, 0x14, 0x00}, // *
//    {}, // +
    {0x00, 0x40, 0x30, 0x00, 0x00}, // ,
//    {}, // -
//    {}, // .
    {0x40, 0x30, 0x0c, 0x02, 0x00}, // /
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
    {0x08, 0x14, 0x22, 0x00, 0x00}, // <
//    {}, // =
    {0x00, 0x00, 0x22, 0x14, 0x08}, // >
//    {}, // ?
//    {}, // @
//    {}, // A
//    {}, // B
//    {}, // C
//    {}, // D
    {0x7f, 0x49, 0x49, 0x49, 0x49}, // E
//    {}, // F
    {0x3e, 0x41, 0x49, 0x49, 0x3a}, // G
    {0x7f, 0x08, 0x08, 0x08, 0x7f}, // H
    {0x00, 0x41, 0x7f, 0x41, 0x00}, // I
//    {}, // J
    {0x7f, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7f, 0x40, 0x40, 0x40, 0x40}, // L
//    {}, // M
    {0x7f, 0x04, 0x08, 0x10, 0x7f}, // N
    {0x3e, 0x41, 0x41, 0x41, 0x3e}, // O
//    {}, // P
//    {}, // Q
//    {}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x01, 0x01, 0x7f, 0x01, 0x01}, // T
//    {}, // U
//    {}, // V
//    {}, // W
//    {}, // X
    {0x07, 0x08, 0x70, 0x08, 0x07}, // Y
//    {}, // Z
//    {}, // [
    {0x00, 0x02, 0x0c, 0x30, 0x40}, // \/
//    {}, // ]
    {0x00, 0x02, 0x01, 0x02,0x00}, // ^
    {0x40, 0x40, 0x40, 0x40, 0x40}, // _
    {0x00, 0x00, 0x02, 0x04, 0x00}, // `
//    {}, // a
//    {}, // b
//    {}, // c
//    {}, // d
    {0x38, 0x54, 0x54, 0x54, 0x58}, // e
//    {}, // f
//    {}, // g
//    {}, // h
    {0x00, 0x48, 0x7a, 0x40, 0x00}, // i
//    {}, // j
//    {}, // k
//    {}, // l
//    {}, // m
//    {}, // n
    {0x38, 0x44, 0x44, 0x44, 0x38}, // o
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
    {0x00, 0x00, 0x7f, 0x00, 0x00}, // |
//    {}, // }
//    {}, // ~
//    {}, // DEL

    {0x0c, 0x12, 0x3f, 0x12, 0x0c}, // Φ
    {0x18, 0x20, 0x10, 0x20, 0x18}, // ω
    {0x00, 0x00, 0x08, 0x00, 0x00}, // ･
    {0x78, 0x04, 0x04, 0x04, 0x78}, // ∩
    {0x00, 0x60, 0x50, 0x48, 0x44}, // ∠
    {0x02, 0x10, 0x28, 0x45, 0x7c}, // ᐛ
    {0x00, 0x3f, 0x01, 0x01, 0x00}, // 「
    {0x00, 0x40, 0x40, 0x7e, 0x00}, // 」
    {0x00, 0x62, 0x55, 0x49, 0x41}, // Σ
    {0x08, 0x04, 0x44, 0x24, 0x18}, // っ
    {0x00, 0x04, 0x0a, 0x04, 0x00}, // °
    {0x40, 0x38, 0x24, 0x38, 0x40}, // Д
    {0x08, 0x14, 0x2a, 0x55, 0x22}, // bold <
    {0x22, 0x55, 0x2a, 0x14, 0x08}, // bold >
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

static volatile enum mode {
  START,
  DISPLAY_TIME,
  SETTING_TIME,
} VFD_Clock_Mode;

static volatile enum unit {
  HOUR,
  MINUTE,
  SECOND,
} Setting_Unit;

static volatile int flash_count;
static volatile bool flash_flag;

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
    if (VFD_Clock_Mode == DISPLAY_TIME) {
      left.input(ccw->Read());
      button.input(push->Read());
      right.input(cw->Read());
    }

    if (VFD_Clock_Mode == DISPLAY_TIME && !ccw->Read())
      vfd->brightness_ += 2;

    if (VFD_Clock_Mode == DISPLAY_TIME && !cw->Read())
      vfd->brightness_ -= 2;

    if (VFD_Clock_Mode == DISPLAY_TIME && button.negEdge()) {
      VFD_Clock_Mode = SETTING_TIME;
      Setting_Unit = HOUR;
      flash_count = 0;
      flash_flag = false;
    }

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

  while (true) {
    if (VFD_Clock_Mode == START) {
            // 「LeeNeo」
            vfd->Font2Buffer(display::font_lib[display::Left_Corner_Bracket], display::vfd_buffer[0]);
            vfd->Font2Buffer(display::font_lib[display::L], display::vfd_buffer[1]);
            vfd->Font2Buffer(display::font_lib[display::e], display::vfd_buffer[2]);
            vfd->Font2Buffer(display::font_lib[display::e], display::vfd_buffer[3]);
            vfd->Font2Buffer(display::font_lib[display::N], display::vfd_buffer[4]);
            vfd->Font2Buffer(display::font_lib[display::e], display::vfd_buffer[5]);
            vfd->Font2Buffer(display::font_lib[display::o], display::vfd_buffer[6]);
            vfd->Font2Buffer(display::font_lib[display::Right_Corner_Bracket], display::vfd_buffer[7]);
            osDelay(1000);

            // YSYSYLYN
            vfd->Font2Buffer(display::font_lib[display::Y], display::vfd_buffer[0]);
            vfd->Font2Buffer(display::font_lib[display::S], display::vfd_buffer[1]);
            vfd->Font2Buffer(display::font_lib[display::Y], display::vfd_buffer[2]);
            vfd->Font2Buffer(display::font_lib[display::S], display::vfd_buffer[3]);
            vfd->Font2Buffer(display::font_lib[display::Y], display::vfd_buffer[4]);
            vfd->Font2Buffer(display::font_lib[display::L], display::vfd_buffer[5]);
            vfd->Font2Buffer(display::font_lib[display::Y], display::vfd_buffer[6]);
            vfd->Font2Buffer(display::font_lib[display::N], display::vfd_buffer[7]);
            osDelay(1000);

            srand(display::time.second);

            int start_idx = rand() % 9;

            switch (start_idx) {
              case 0:
                // (/>w<)/
                vfd->Font2Buffer(display::font_lib[display::LPAREN], display::vfd_buffer[0]);
                vfd->Font2Buffer(display::font_lib[display::SOL], display::vfd_buffer[1]);
                vfd->Font2Buffer(display::font_lib[display::GT], display::vfd_buffer[2]);
                vfd->Font2Buffer(display::font_lib[display::Omega], display::vfd_buffer[3]);
                vfd->Font2Buffer(display::font_lib[display::LT], display::vfd_buffer[4]);
                vfd->Font2Buffer(display::font_lib[display::RPAREN], display::vfd_buffer[5]);
                vfd->Font2Buffer(display::font_lib[display::SOL], display::vfd_buffer[6]);
                vfd->Font2Buffer(display::font_lib[display::SP], display::vfd_buffer[7]);
                break ;
              case 1:
                // (*ΦωΦ*)
                vfd->Font2Buffer(display::font_lib[display::LPAREN], display::vfd_buffer[0]);
                vfd->Font2Buffer(display::font_lib[display::AST], display::vfd_buffer[1]);
                vfd->Font2Buffer(display::font_lib[display::Phi], display::vfd_buffer[2]);
                vfd->Font2Buffer(display::font_lib[display::Omega], display::vfd_buffer[3]);
                vfd->Font2Buffer(display::font_lib[display::Phi], display::vfd_buffer[4]);
                vfd->Font2Buffer(display::font_lib[display::AST], display::vfd_buffer[5]);
                vfd->Font2Buffer(display::font_lib[display::RPAREN], display::vfd_buffer[6]);
                vfd->Font2Buffer(display::font_lib[display::SP], display::vfd_buffer[7]);
                break ;
              case 2:
                // |･ω･｀)/
                vfd->Font2Buffer(display::font_lib[display::VERBAR], display::vfd_buffer[0]);
                vfd->Font2Buffer(display::font_lib[display::Dot], display::vfd_buffer[1]);
                vfd->Font2Buffer(display::font_lib[display::Omega], display::vfd_buffer[2]);
                vfd->Font2Buffer(display::font_lib[display::Dot], display::vfd_buffer[3]);
                vfd->Font2Buffer(display::font_lib[display::GRAVE], display::vfd_buffer[4]);
                vfd->Font2Buffer(display::font_lib[display::RPAREN], display::vfd_buffer[5]);
                vfd->Font2Buffer(display::font_lib[display::SOL], display::vfd_buffer[6]);
                vfd->Font2Buffer(display::font_lib[display::SP], display::vfd_buffer[7]);
                break ;
              case 3:
                // (//ω//)
                vfd->Font2Buffer(display::font_lib[display::LPAREN], display::vfd_buffer[0]);
                vfd->Font2Buffer(display::font_lib[display::SOL], display::vfd_buffer[1]);
                vfd->Font2Buffer(display::font_lib[display::SOL], display::vfd_buffer[2]);
                vfd->Font2Buffer(display::font_lib[display::Omega], display::vfd_buffer[3]);
                vfd->Font2Buffer(display::font_lib[display::SOL], display::vfd_buffer[4]);
                vfd->Font2Buffer(display::font_lib[display::SOL], display::vfd_buffer[5]);
                vfd->Font2Buffer(display::font_lib[display::RPAREN], display::vfd_buffer[6]);
                vfd->Font2Buffer(display::font_lib[display::SP], display::vfd_buffer[7]);
                break ;
              case 4:
                // (*/ω\*)
                vfd->Font2Buffer(display::font_lib[display::LPAREN], display::vfd_buffer[0]);
                vfd->Font2Buffer(display::font_lib[display::AST], display::vfd_buffer[1]);
                vfd->Font2Buffer(display::font_lib[display::SOL], display::vfd_buffer[2]);
                vfd->Font2Buffer(display::font_lib[display::Omega], display::vfd_buffer[3]);
                vfd->Font2Buffer(display::font_lib[display::BSOL], display::vfd_buffer[4]);
                vfd->Font2Buffer(display::font_lib[display::AST], display::vfd_buffer[5]);
                vfd->Font2Buffer(display::font_lib[display::RPAREN], display::vfd_buffer[6]);
                vfd->Font2Buffer(display::font_lib[display::SP], display::vfd_buffer[7]);
                break ;
              case 5:
                // (^･ω･^)
                vfd->Font2Buffer(display::font_lib[display::LPAREN], display::vfd_buffer[0]);
                vfd->Font2Buffer(display::font_lib[display::HAT], display::vfd_buffer[1]);
                vfd->Font2Buffer(display::font_lib[display::Dot], display::vfd_buffer[2]);
                vfd->Font2Buffer(display::font_lib[display::Omega], display::vfd_buffer[3]);
                vfd->Font2Buffer(display::font_lib[display::Dot], display::vfd_buffer[4]);
                vfd->Font2Buffer(display::font_lib[display::HAT], display::vfd_buffer[5]);
                vfd->Font2Buffer(display::font_lib[display::RPAREN], display::vfd_buffer[6]);
                vfd->Font2Buffer(display::font_lib[display::SP], display::vfd_buffer[7]);
                break ;
              case 6:
                // (∩>ω<∩)
                vfd->Font2Buffer(display::font_lib[display::LPAREN], display::vfd_buffer[0]);
                vfd->Font2Buffer(display::font_lib[display::Intersection], display::vfd_buffer[1]);
                vfd->Font2Buffer(display::font_lib[display::GT], display::vfd_buffer[2]);
                vfd->Font2Buffer(display::font_lib[display::Omega], display::vfd_buffer[3]);
                vfd->Font2Buffer(display::font_lib[display::LT], display::vfd_buffer[4]);
                vfd->Font2Buffer(display::font_lib[display::Intersection], display::vfd_buffer[5]);
                vfd->Font2Buffer(display::font_lib[display::RPAREN], display::vfd_buffer[6]);
                vfd->Font2Buffer(display::font_lib[display::SP], display::vfd_buffer[7]);
                break ;
              case 7:
                // ∠(ᐛ 」∠)_
                vfd->Font2Buffer(display::font_lib[display::Angle], display::vfd_buffer[0]);
                vfd->Font2Buffer(display::font_lib[display::LPAREN], display::vfd_buffer[1]);
                vfd->Font2Buffer(display::font_lib[display::Waa], display::vfd_buffer[2]);
                vfd->Font2Buffer(display::font_lib[display::Right_Corner_Bracket], display::vfd_buffer[3]);
                vfd->Font2Buffer(display::font_lib[display::Angle], display::vfd_buffer[4]);
                vfd->Font2Buffer(display::font_lib[display::RPAREN], display::vfd_buffer[5]);
                vfd->Font2Buffer(display::font_lib[display::LOWBAR], display::vfd_buffer[6]);
                vfd->Font2Buffer(display::font_lib[display::SP], display::vfd_buffer[7]);
                break ;
              case 8:
                // Σ(っ °Д °)っ
                vfd->Font2Buffer(display::font_lib[display::Sigma], display::vfd_buffer[0]);
                vfd->Font2Buffer(display::font_lib[display::LPAREN], display::vfd_buffer[1]);
                vfd->Font2Buffer(display::font_lib[display::Hiragana], display::vfd_buffer[2]);
                vfd->Font2Buffer(display::font_lib[display::Degree], display::vfd_buffer[3]);
                vfd->Font2Buffer(display::font_lib[display::Cyrillic], display::vfd_buffer[4]);
                vfd->Font2Buffer(display::font_lib[display::Degree], display::vfd_buffer[5]);
                vfd->Font2Buffer(display::font_lib[display::RPAREN], display::vfd_buffer[6]);
                vfd->Font2Buffer(display::font_lib[display::Hiragana], display::vfd_buffer[7]);
                break ;
              default: ;
            }
            osDelay(1000);

            VFD_Clock_Mode = DISPLAY_TIME;

            vfd->GetTime();

            vfd->Font2Buffer(display::font_lib[vfd->second_ % 10 + display::num2ascii], display::vfd_buffer[7]);
            vfd->Font2Buffer(display::font_lib[vfd->second_ / 10 + display::num2ascii], display::vfd_buffer[6]);

            vfd->Font2Buffer(display::font_lib[display::COLON], display::vfd_buffer[5]);

            vfd->Font2Buffer(display::font_lib[vfd->minute_ % 10 + display::num2ascii], display::vfd_buffer[4]);
            vfd->Font2Buffer(display::font_lib[vfd->minute_ / 10 + display::num2ascii], display::vfd_buffer[3]);

            vfd->Font2Buffer(display::font_lib[display::COLON], display::vfd_buffer[2]);

            vfd->Font2Buffer(display::font_lib[vfd->hour_ % 10 + display::num2ascii], display::vfd_buffer[1]);
            vfd->Font2Buffer(display::font_lib[vfd->hour_ / 10 + display::num2ascii], display::vfd_buffer[0]);
    } else if (VFD_Clock_Mode == DISPLAY_TIME) {
            led->High();
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
    } else if (VFD_Clock_Mode == SETTING_TIME) {
            led->Low();
            left.input(ccw->Read());
            button.input(push->Read());
            right.input(cw->Read());

            if (left.negEdge()) {
              if (Setting_Unit == HOUR) {
                vfd->hour_ += 1;
                vfd->hour_ %= 24;
              } else if (Setting_Unit == MINUTE) {
                vfd->minute_ += 1;
                vfd->minute_ %= 60;
              } else if (Setting_Unit == SECOND) {
                vfd->second_ += 1;
                vfd->second_ %= 60;
              }
            }

            if (right.negEdge()) {
              if (Setting_Unit == HOUR) {
                vfd->hour_ -= 1;
                vfd->hour_ = vfd->hour_ < 0 ? 23 : vfd->hour_;
              } else if (Setting_Unit == MINUTE) {
                vfd->minute_ -= 1;
                vfd->minute_ = vfd->minute_ < 0 ? 59 : vfd->minute_;
              } else if (Setting_Unit == SECOND) {
                vfd->second_ -= 1;
                vfd->second_ = vfd->second_ < 0 ? 59 : vfd->second_;
              }
            }

            flash_count += 1;
            if (flash_count % 30 == 0)
              flash_flag = !flash_flag;

            if (Setting_Unit == HOUR) {
              vfd->Font2Buffer(display::font_lib[flash_flag? vfd->hour_ % 10 + display::num2ascii : display::SP], display::vfd_buffer[1]);
              vfd->Font2Buffer(display::font_lib[flash_flag? vfd->hour_ / 10 + display::num2ascii : display::SP], display::vfd_buffer[0]);
              vfd->Font2Buffer(display::font_lib[vfd->minute_ % 10 + display::num2ascii], display::vfd_buffer[4]);
              vfd->Font2Buffer(display::font_lib[vfd->minute_ / 10 + display::num2ascii], display::vfd_buffer[3]);
              vfd->Font2Buffer(display::font_lib[vfd->second_ % 10 + display::num2ascii], display::vfd_buffer[7]);
              vfd->Font2Buffer(display::font_lib[vfd->second_ / 10 + display::num2ascii], display::vfd_buffer[6]);
            } else if (Setting_Unit == MINUTE) {
              vfd->Font2Buffer(display::font_lib[vfd->hour_ % 10 + display::num2ascii], display::vfd_buffer[1]);
              vfd->Font2Buffer(display::font_lib[vfd->hour_ / 10 + display::num2ascii], display::vfd_buffer[0]);
              vfd->Font2Buffer(display::font_lib[flash_flag? vfd->minute_ % 10 + display::num2ascii : display::SP], display::vfd_buffer[4]);
              vfd->Font2Buffer(display::font_lib[flash_flag? vfd->minute_ / 10 + display::num2ascii : display::SP], display::vfd_buffer[3]);
              vfd->Font2Buffer(display::font_lib[vfd->second_ % 10 + display::num2ascii], display::vfd_buffer[7]);
              vfd->Font2Buffer(display::font_lib[vfd->second_ / 10 + display::num2ascii], display::vfd_buffer[6]);
            } else if (Setting_Unit == SECOND) {
              vfd->Font2Buffer(display::font_lib[vfd->hour_ % 10 + display::num2ascii], display::vfd_buffer[1]);
              vfd->Font2Buffer(display::font_lib[vfd->hour_ / 10 + display::num2ascii], display::vfd_buffer[0]);
              vfd->Font2Buffer(display::font_lib[vfd->minute_ % 10 + display::num2ascii], display::vfd_buffer[4]);
              vfd->Font2Buffer(display::font_lib[vfd->minute_ / 10 + display::num2ascii], display::vfd_buffer[3]);
              vfd->Font2Buffer(display::font_lib[flash_flag? vfd->second_ % 10 + display::num2ascii : display::SP], display::vfd_buffer[7]);
              vfd->Font2Buffer(display::font_lib[flash_flag? vfd->second_ / 10 + display::num2ascii : display::SP], display::vfd_buffer[6]);
            }

            if (button.negEdge()) {
              if (Setting_Unit == HOUR)
                Setting_Unit = MINUTE;
              else if (Setting_Unit == MINUTE)
                Setting_Unit = SECOND;
              else if (Setting_Unit == SECOND) {
                VFD_Clock_Mode = DISPLAY_TIME;
                clock->SetTime(21, 6, 5, 6, vfd->hour_, vfd->minute_, vfd->second_);
              }
            }
    }
    osDelay(10);
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

  VFD_Clock_Mode = START;
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
