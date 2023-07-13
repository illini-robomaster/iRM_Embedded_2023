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

static void delay_us(uint32_t us) {
  uint32_t start = bsp::GetHighresTickMicroSec();
  while (bsp::GetHighresTickMicroSec() - start < us);
}

static volatile bool refresh_all_flag;

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

int num2ascii = 7;

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
  MINUS = 5,
  PERIOD,
  SOL = 6,
  zero = 7,
  one = 8,
  two = 9,
  three = 10,
  four = 11,
  five = 12,
  six = 13,
  seven = 14,
  eight = 15,
  nine = 16,
  COLON = 17,
  SEMI,
  LT = 18,
  EQUALS,
  GT = 19,
  QUEST,
  COMMAT,
  A,
  B,
  C,
  D,
  E = 20,
  F = 21,
  G = 22,
  H = 23,
  I = 24,
  J,
  K = 25,
  L = 26,
  M = 27,
  N = 28,
  O = 29,
  P,
  Q,
  R,
  S = 30,
  T = 31,
  U,
  V,
  W = 32,
  X,
  Y = 33,
  Z,
  LSQB,
  BSOL = 34,
  RSQB,
  HAT = 35,
  LOWBAR = 36,
  GRAVE = 37,
  a = 38,
  b,
  c,
  d = 39,
  e = 40,
  f,
  g,
  h = 41,
  i = 42,
  j,
  k,
  l,
  m,
  n = 43,
  o = 44,
  p,
  q,
  r = 45,
  s,
  t = 46,
  u = 47,
  v,
  w,
  x,
  y,
  z,
  LCUB,
  VERBAR = 48,
  RCUB,
  TILDE,
  DEL,

  Phi = 49,
  Omega = 50,
  Dot = 51,
  Intersection = 52,
  Angle = 53,
  Waa = 54,
  Left_Corner_Bracket = 55,
  Right_Corner_Bracket = 56,
  Sigma = 57,
  Hiragana = 58,
  Degree = 59,
  Cyrillic = 60,
  Bold_LT = 61,
  Bold_GT = 62,
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
    {0x00, 0x08, 0x08, 0x08, 0x00}, // -
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
    {0x7f, 0x09, 0x09, 0x09, 0x01}, // F
    {0x3e, 0x41, 0x49, 0x49, 0x3a}, // G
    {0x7f, 0x08, 0x08, 0x08, 0x7f}, // H
    {0x00, 0x41, 0x7f, 0x41, 0x00}, // I
//    {}, // J
    {0x7f, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7f, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7f, 0x02, 0x0c, 0x02, 0x7f}, // M
    {0x7f, 0x04, 0x08, 0x10, 0x7f}, // N
    {0x3e, 0x41, 0x41, 0x41, 0x3e}, // O
//    {}, // P
//    {}, // Q
//    {}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x01, 0x01, 0x7f, 0x01, 0x01}, // T
//    {}, // U
//    {}, // V
    {0x3f, 0x40, 0x38, 0x40, 0x3f}, // W
//    {}, // X
    {0x07, 0x08, 0x70, 0x08, 0x07}, // Y
//    {}, // Z
//    {}, // [
    {0x00, 0x02, 0x0c, 0x30, 0x40}, // \/
//    {}, // ]
    {0x00, 0x02, 0x01, 0x02,0x00}, // ^
    {0x40, 0x40, 0x40, 0x40, 0x40}, // _
    {0x00, 0x00, 0x02, 0x04, 0x00}, // `
    {0x20, 0x54, 0x54, 0x54, 0x78}, // a
//    {}, // b
//    {}, // c
    {0x30, 0x48, 0x48, 0x48, 0x7e}, // d
    {0x38, 0x54, 0x54, 0x54, 0x58}, // e
//    {}, // f
//    {}, // g
    {0x7e, 0x08, 0x08, 0x08, 0x70}, // h
    {0x00, 0x48, 0x7a, 0x40, 0x00}, // i
//    {}, // j
//    {}, // k
//    {}, // l
//    {}, // m
    {0x7c, 0x08, 0x04, 0x04, 0x78}, // n
    {0x38, 0x44, 0x44, 0x44, 0x38}, // o
//    {}, // p
//    {}, // q
    {0x7c, 0x08, 0x04, 0x04, 0x08}, // r
//    {}, // s
    {0x08, 0x3e, 0x48, 0x48, 0x28}, // t
    {0x3c, 0x40, 0x40, 0x40, 0x7c}, // u
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
    {0x00, 0x63, 0x55, 0x49, 0x41}, // Σ
    {0x08, 0x04, 0x44, 0x24, 0x18}, // っ
    {0x00, 0x06, 0x09, 0x09, 0x06}, // °
    {0x40, 0x38, 0x24, 0x38, 0x40}, // Д
    {0x08, 0x1c, 0x3e, 0x77, 0x22}, // bold <
    {0x22, 0x77, 0x3e, 0x1c, 0x08}, // bold >
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

  static void Font2Buffer(char* font, char* buffer) {
    memcpy(buffer, font, sizeof(char) * 5);
  }

  static unsigned GetRandomFont() {
    return rand() % (sizeof(font_lib) / sizeof(char) / 5);
  }

  static void MoveDown(int pos, int step, unsigned new_font) {
    if (!(0 <= pos && pos <= 7 && 1 <= step && step <= 7))
        return ;
    for (int i = 0; i < 5; ++i) {
        vfd_buffer[pos][i] = 0x7f & (vfd_buffer[pos][i] << 1);
        vfd_buffer[pos][i] |= font_lib[new_font][i] >> (7 - step);
    }
  }

  static void Move3Right(int step, unsigned font0, unsigned font1, unsigned font2) {
    if (!(1 <= step && step <= 15))
        return ;
    for (int i = 3; i >= 1; --i) {
        for (int j = 4; j >= 1; --j)
          vfd_buffer[i][j] = vfd_buffer[i][j - 1];
        if (i != 1)
          vfd_buffer[i][0] = vfd_buffer[i - 1][4];
        else {
          if (step <= 5)
            vfd_buffer[i][0] = font_lib[font2][5 - step];
          else if (step <= 10)
            vfd_buffer[i][0] = font_lib[font1][10 - step];
          else
            vfd_buffer[i][0] = font_lib[font0][15 - step];
        }

    }
  }

  static void Move3Left(int step, unsigned font0, unsigned font1, unsigned font2) {
    if (!(1 <= step && step <= 15))
        return ;
    for (int i = 4; i <= 6; ++i) {
        for (int j = 0; j <= 3; ++j)
          vfd_buffer[i][j] = vfd_buffer[i][j + 1];
        if (i != 6)
          vfd_buffer[i][4] = vfd_buffer[i + 1][0];
        else {
          if (step <= 5)
            vfd_buffer[i][4] = font_lib[font0][step - 1];
          else if (step <= 10)
            vfd_buffer[i][4] = font_lib[font1][step - 6];
          else
            vfd_buffer[i][4] = font_lib[font2][step - 11];
        }
    }
  }

  static void MoveCircularRight(int pos, int step, unsigned font) {
    if (!(0 <= pos && pos <= 7 && 0 <= step && step < 5))
        return ;
    for (int i = 0; i < 5; ++i) {
        int idx = i - step;
        if (idx < 0) idx += 5;
        vfd_buffer[pos][i] = font_lib[font][idx];
    }
  }

  static void MoveCircularLeft(int pos, int step, unsigned font) {
    if (!(0 <= pos && pos <= 7 && 0 <= step && step < 5))
        return ;
    for (int i = 0; i < 5; ++i) {
        int idx = i + step;
        if (idx >= 5) idx -= 5;
        vfd_buffer[pos][i] = font_lib[font][idx];
    }
  }

  static void ClearScreen() {
    memset(vfd_buffer, 0, sizeof(vfd_buffer));
  }

  void GetTime() {
    if (refresh_all_flag) {
        memset(NeedUpdate, true, sizeof(NeedUpdate));
        UpdateFlag = true;
    } else {
        NeedUpdate[0] = time.second % 10 != second_ % 10;
        NeedUpdate[1] = time.second / 10 != second_ / 10;
        NeedUpdate[2] = time.minute % 10 != minute_ % 10;
        NeedUpdate[3] = time.minute / 10 != minute_ / 10;
        NeedUpdate[4] = time.hour % 10 != hour_ % 10;
        NeedUpdate[5] = time.hour / 10 != hour_ / 10;
        UpdateFlag = NeedUpdate[0] || NeedUpdate[1] || NeedUpdate[2] || NeedUpdate[3] || NeedUpdate[4] || NeedUpdate[5];
    }

    year_ = time.year;
    month_ = time.month;
    date_ = time.date;
    day_ = time.day;

    hour_ = time.hour;
    minute_ = time.minute;
    second_ = time.second;
  }

  bool NeedUpdate[8] = {};

  bool UpdateFlag = false;

  int year_ = 0;
  int month_ = 0;
  int date_ = 0;
  int day_ = 0;

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

static volatile int display_calendar_flag = 0; // 0 for time, 1 for date, 2 for day

static volatile long move_count;

static volatile enum mode {
  START,
  DISPLAY_TIME,
  SETTING_TIME,
} VFD_Clock_Mode;

static volatile enum unit {
  YEAR,
  MONTH,
  DATE,
  DAY,
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
      int press_count = 0;
      while (true) {
        osDelay(50);
        button.input(push->Read());
        ++press_count;
        if (button.posEdge()) {
            move_count = 0;
            if (display_calendar_flag == 0)
              display_calendar_flag = 1;
            else if (display_calendar_flag == 1)
              display_calendar_flag = 2;
            else if (display_calendar_flag == 2) {
              display_calendar_flag = 0;
              refresh_all_flag = true;
            }
            break;
        } else if (press_count > 20) {
          VFD_Clock_Mode = SETTING_TIME;
          Setting_Unit = YEAR;
          flash_count = 0;
          flash_flag = false;
          break;
        }
      }
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

//#define SET_TIME

void updateTimeTask(void* arg) {
  UNUSED(arg);


  while (!clock->IsReady()) {
    clock_flag = false;
    i2c_reset();
    osDelay(100);
  }

#ifdef SET_TIME
  if (!clock->SetTime(21, 6, 5, 6, 5, 20, 0)) {
    i2c_reset();
    osDelay(100);
  }
#endif

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

    osDelay(10);
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
            display::VFD::Font2Buffer(display::font_lib[display::Left_Corner_Bracket], display::vfd_buffer[0]);
            display::VFD::Font2Buffer(display::font_lib[display::Right_Corner_Bracket], display::vfd_buffer[7]);
            osDelay(500);

            for (int i = 1; i <= 15; ++i) {
              display::VFD::Move3Right(i, display::L, display::e, display::e);
              display::VFD::Move3Left(i, display::N, display::e, display::o);
              osDelay(100);
            }
            osDelay(500);

            // YSYSYLYN
            for (int i = 1; i <= 7; ++i) {
              display::VFD::MoveDown(0, i, display::Y);
              osDelay(200 / 7);
            }
            for (int i = 1; i <= 7; ++i) {
              display::VFD::MoveDown(1, i, display::S);
              osDelay(200 / 7);
            }
            for (int i = 1; i <= 7; ++i) {
              display::VFD::MoveDown(2, i, display::Y);
              osDelay(200 / 7);
            }
            for (int i = 1; i <= 7; ++i) {
              display::VFD::MoveDown(3, i, display::S);
              osDelay(200 / 7);
            }
            for (int i = 1; i <= 7; ++i) {
              display::VFD::MoveDown(4, i, display::Y);
              osDelay(200 / 7);
            }
            for (int i = 1; i <= 7; ++i) {
              display::VFD::MoveDown(5, i, display::L);
              osDelay(200 / 7);
            }
            for (int i = 1; i <= 7; ++i) {
              display::VFD::MoveDown(6, i, display::Y);
              osDelay(200 / 7);
            }
            for (int i = 1; i <= 7; ++i) {
              display::VFD::MoveDown(7, i, display::N);
              osDelay(200 / 7);
            }
            while (vfd->brightness_ > 0) {
              --vfd->brightness_;
              osDelay(10);
            }
            display::VFD::ClearScreen();
            osDelay(100);

            srand(display::time.second);
            int start_idx = rand() % 9;
            int emoji[8];

            switch (start_idx) {
              case 0:
                // (/>w<)/
                emoji[0] = display::LPAREN;
                emoji[1] = display::SOL;
                emoji[2] = display::GT;
                emoji[3] = display::Omega;
                emoji[4] = display::LT;
                emoji[5] = display::RPAREN;
                emoji[6] = display::SOL;
                emoji[7] = display::SP;
                break ;
              case 1:
                // (*ΦωΦ*)
                emoji[0] = display::LPAREN;
                emoji[1] = display::AST;
                emoji[2] = display::Phi;
                emoji[3] = display::Omega;
                emoji[4] = display::Phi;
                emoji[5] = display::AST;
                emoji[6] = display::RPAREN;
                emoji[7] = display::SP;
                break ;
              case 2:
                // |･ω･｀)/
                emoji[0] = display::VERBAR;
                emoji[1] = display::Dot;
                emoji[2] = display::Omega;
                emoji[3] = display::Dot;
                emoji[4] = display::GRAVE;
                emoji[5] = display::RPAREN;
                emoji[6] = display::SOL;
                emoji[7] = display::SP;
                break ;
              case 3:
                // (//ω//)
                emoji[0] = display::LPAREN;
                emoji[1] = display::SOL;
                emoji[2] = display::SOL;
                emoji[3] = display::Omega;
                emoji[4] = display::SOL;
                emoji[5] = display::SOL;
                emoji[6] = display::RPAREN;
                emoji[7] = display::SP;
                break ;
              case 4:
                // (*/ω\*)
                emoji[0] = display::LPAREN;
                emoji[1] = display::AST;
                emoji[2] = display::SOL;
                emoji[3] = display::Omega;
                emoji[4] = display::BSOL;
                emoji[5] = display::AST;
                emoji[6] = display::RPAREN;
                emoji[7] = display::SP;
                break ;
              case 5:
                // (^･ω･^)
                emoji[0] = display::LPAREN;
                emoji[1] = display::HAT;
                emoji[2] = display::Dot;
                emoji[3] = display::Omega;
                emoji[4] = display::Dot;
                emoji[5] = display::HAT;
                emoji[6] = display::RPAREN;
                emoji[7] = display::SP;
                break ;
              case 6:
                // (∩>ω<∩)
                emoji[0] = display::LPAREN;
                emoji[1] = display::Intersection;
                emoji[2] = display::GT;
                emoji[3] = display::Omega;
                emoji[4] = display::LT;
                emoji[5] = display::Intersection;
                emoji[6] = display::RPAREN;
                emoji[7] = display::SP;
                break ;
              case 7:
                // ∠(ᐛ 」∠)_
                emoji[0] = display::Angle;
                emoji[1] = display::LPAREN;
                emoji[2] = display::Waa;
                emoji[3] = display::Right_Corner_Bracket;
                emoji[4] = display::Angle;
                emoji[5] = display::RPAREN;
                emoji[6] = display::LOWBAR;
                emoji[7] = display::SP;
                break ;
              case 8:
                // Σ(っ °Д °)っ
                emoji[0] = display::Sigma;
                emoji[1] = display::LPAREN;
                emoji[2] = display::Hiragana;
                emoji[3] = display::Degree;
                emoji[4] = display::Cyrillic;
                emoji[5] = display::Degree;
                emoji[6] = display::RPAREN;
                emoji[7] = display::Hiragana;
                break ;
              default:
                break ;
            }

            while (vfd->brightness_ < 100) {
              if (vfd->brightness_ % 5 == 0)
                for (auto & i : display::vfd_buffer)
                  display::VFD::Font2Buffer(display::font_lib[display::VFD::GetRandomFont()], i);
              ++vfd->brightness_;
              osDelay(20);
            }

            for (int i = 0; i < 8; ++i) {
              for (int j = 0; j < 3; ++j) {
                for (int k = 0; k < 8; ++k)
                  display::VFD::Font2Buffer(display::font_lib[i >= k ? emoji[k] : display::VFD::GetRandomFont()], display::vfd_buffer[k]);
                osDelay(100);
              }
            }
            osDelay(1000);

            while (vfd->brightness_ > 0) {
              --vfd->brightness_;
              osDelay(10);
            }
            display::VFD::ClearScreen();
            osDelay(100);
            vfd->brightness_ = 100;

            VFD_Clock_Mode = DISPLAY_TIME;
            refresh_all_flag = true;
    } else if (VFD_Clock_Mode == DISPLAY_TIME) {
            led->High();
            vfd->GetTime();

            if (display_calendar_flag == 0) {
              for (int i = 1; i <= 7; ++i) {
                if (refresh_all_flag) {
                  display::VFD::MoveDown(2, i, display::COLON);
                  display::VFD::MoveDown(5, i, display::COLON);
                }
                if (vfd->NeedUpdate[0])
                  display::VFD::MoveDown(7, i, vfd->second_ % 10 + display::num2ascii);
                if (vfd->NeedUpdate[1])
                  display::VFD::MoveDown(6, i, vfd->second_ / 10 + display::num2ascii);
                if (vfd->NeedUpdate[2])
                  display::VFD::MoveDown(4, i, vfd->minute_ % 10 + display::num2ascii);
                if (vfd->NeedUpdate[3])
                  display::VFD::MoveDown(3, i, vfd->minute_ / 10 + display::num2ascii);
                if (vfd->NeedUpdate[4])
                  display::VFD::MoveDown(1, i, vfd->hour_ % 10 + display::num2ascii);
                if (vfd->NeedUpdate[5])
                  display::VFD::MoveDown(0, i, vfd->hour_ / 10 + display::num2ascii);
                if (vfd->UpdateFlag)
                  osDelay(500 / 7);
              }
            } else if (display_calendar_flag == 1) {
              display::VFD::Font2Buffer(display::font_lib[vfd->year_ / 10 + display::num2ascii], display::vfd_buffer[0]);
              display::VFD::Font2Buffer(display::font_lib[vfd->year_ % 10 + display::num2ascii], display::vfd_buffer[1]);
              display::VFD::MoveCircularRight(2, (move_count / 10) % 5, display::MINUS);
              display::VFD::Font2Buffer(display::font_lib[vfd->month_ / 10 + display::num2ascii], display::vfd_buffer[3]);
              display::VFD::Font2Buffer(display::font_lib[vfd->month_ % 10 + display::num2ascii], display::vfd_buffer[4]);
              display::VFD::MoveCircularRight(5, (move_count / 10) % 5, display::MINUS);
              display::VFD::Font2Buffer(display::font_lib[vfd->date_ / 10 + display::num2ascii], display::vfd_buffer[6]);
              display::VFD::Font2Buffer(display::font_lib[vfd->date_ % 10 + display::num2ascii], display::vfd_buffer[7]);

              ++move_count;
            } else if (display_calendar_flag == 2) {
              display::VFD::MoveCircularRight(0, (move_count / 10) % 5, display::GT);
              display::VFD::MoveCircularRight(1, (move_count / 10) % 5, display::Bold_GT);
              display::VFD::MoveCircularLeft(5, (move_count / 10) % 5, display::Bold_LT);
              display::VFD::MoveCircularLeft(6, (move_count / 10) % 5, display::LT);
              display::VFD::Font2Buffer(display::font_lib[display::SP], display::vfd_buffer[7]);

              switch (vfd->day_) {
                case 1:
                  display::VFD::Font2Buffer(display::font_lib[display::M], display::vfd_buffer[2]);
                  display::VFD::Font2Buffer(display::font_lib[display::o], display::vfd_buffer[3]);
                  display::VFD::Font2Buffer(display::font_lib[display::n], display::vfd_buffer[4]);
                  break ;
                case 2:
                  display::VFD::Font2Buffer(display::font_lib[display::T], display::vfd_buffer[2]);
                  display::VFD::Font2Buffer(display::font_lib[display::u], display::vfd_buffer[3]);
                  display::VFD::Font2Buffer(display::font_lib[display::e], display::vfd_buffer[4]);
                  break ;
                case 3:
                  display::VFD::Font2Buffer(display::font_lib[display::W], display::vfd_buffer[2]);
                  display::VFD::Font2Buffer(display::font_lib[display::e], display::vfd_buffer[3]);
                  display::VFD::Font2Buffer(display::font_lib[display::d], display::vfd_buffer[4]);
                  break ;
                case 4:
                  display::VFD::Font2Buffer(display::font_lib[display::T], display::vfd_buffer[2]);
                  display::VFD::Font2Buffer(display::font_lib[display::h], display::vfd_buffer[3]);
                  display::VFD::Font2Buffer(display::font_lib[display::u], display::vfd_buffer[4]);
                  break ;
                case 5:
                  display::VFD::Font2Buffer(display::font_lib[display::F], display::vfd_buffer[2]);
                  display::VFD::Font2Buffer(display::font_lib[display::r], display::vfd_buffer[3]);
                  display::VFD::Font2Buffer(display::font_lib[display::i], display::vfd_buffer[4]);
                  break ;
                case 6:
                  display::VFD::Font2Buffer(display::font_lib[display::S], display::vfd_buffer[2]);
                  display::VFD::Font2Buffer(display::font_lib[display::a], display::vfd_buffer[3]);
                  display::VFD::Font2Buffer(display::font_lib[display::t], display::vfd_buffer[4]);
                  break ;
                case 7:
                  display::VFD::Font2Buffer(display::font_lib[display::S], display::vfd_buffer[2]);
                  display::VFD::Font2Buffer(display::font_lib[display::u], display::vfd_buffer[3]);
                  display::VFD::Font2Buffer(display::font_lib[display::n], display::vfd_buffer[4]);
                  break ;
                default:
                  break ;
              }

              ++move_count;
            }

            refresh_all_flag = false;
    } else if (VFD_Clock_Mode == SETTING_TIME) {
            led->Low();
            left.input(ccw->Read());
            button.input(push->Read());
            right.input(cw->Read());

            if (left.negEdge()) {
              if (Setting_Unit == YEAR) {
                vfd->year_ += 1;
                vfd->year_ = vfd->year_ > 99? 00 : vfd->year_;
              } else if (Setting_Unit == MONTH) {
                vfd->month_ += 1;
                vfd->month_ = vfd->month_ > 12? 1 : vfd->month_;
              } else if (Setting_Unit == DATE) {
                vfd->date_ += 1;
                vfd->date_ = vfd->date_ > 31? 1 : vfd->date_;
              } else if (Setting_Unit == DAY) {
                vfd->day_ += 1;
                vfd->day_ = vfd->day_ > 7? 1 : vfd->day_;
              } else if (Setting_Unit == HOUR) {
                vfd->hour_ += 1;
                vfd->hour_ = vfd->hour_ > 23? 0 : vfd->hour_;
              } else if (Setting_Unit == MINUTE) {
                vfd->minute_ += 1;
                vfd->minute_  = vfd->minute_ > 59? 0 : vfd->minute_;
              } else if (Setting_Unit == SECOND) {
                vfd->second_ += 1;
                vfd->second_  = vfd->second_ > 59? 0 : vfd->second_;
              }
            }

            if (right.negEdge()) {
              if (Setting_Unit == YEAR) {
                vfd->year_ -= 1;
                vfd->year_ = vfd->year_ < 0? 99 : vfd->year_;
              } else if (Setting_Unit == MONTH) {
                vfd->month_ -= 1;
                vfd->month_ = vfd->month_ < 1? 12 : vfd->month_;
              } else if (Setting_Unit == DATE) {
                vfd->date_ -= 1;
                vfd->date_ = vfd->date_ < 1? 31 : vfd->date_;
              } else if (Setting_Unit == DAY) {
                vfd->day_ -= 1;
                vfd->day_ = vfd->day_ < 1? 7 : vfd->day_;
              } else if (Setting_Unit == HOUR) {
                vfd->hour_ -= 1;
                vfd->hour_ = vfd->hour_ < 0? 23 : vfd->hour_;
              } else if (Setting_Unit == MINUTE) {
                vfd->minute_ -= 1;
                vfd->minute_ = vfd->minute_ < 0? 59 : vfd->minute_;
              } else if (Setting_Unit == SECOND) {
                vfd->second_ -= 1;
                vfd->second_ = vfd->second_ < 0? 59 : vfd->second_;
              }
            }

            flash_count += 1;
            if (flash_count % 30 == 0)
              flash_flag = !flash_flag;

            if (Setting_Unit == YEAR) {
              display::VFD::Font2Buffer(display::font_lib[flash_flag? vfd->year_ / 10 + display::num2ascii : display::SP], display::vfd_buffer[0]);
              display::VFD::Font2Buffer(display::font_lib[flash_flag? vfd->year_ % 10 + display::num2ascii : display::SP], display::vfd_buffer[1]);
              display::VFD::Font2Buffer(display::font_lib[display::MINUS], display::vfd_buffer[2]);
              display::VFD::Font2Buffer(display::font_lib[vfd->month_ / 10 + display::num2ascii], display::vfd_buffer[3]);
              display::VFD::Font2Buffer(display::font_lib[vfd->month_ % 10 + display::num2ascii], display::vfd_buffer[4]);
              display::VFD::Font2Buffer(display::font_lib[display::MINUS], display::vfd_buffer[5]);
              display::VFD::Font2Buffer(display::font_lib[vfd->date_ / 10 + display::num2ascii], display::vfd_buffer[6]);
              display::VFD::Font2Buffer(display::font_lib[vfd->date_ % 10 + display::num2ascii], display::vfd_buffer[7]);
            } else if (Setting_Unit == MONTH) {
              display::VFD::Font2Buffer(display::font_lib[vfd->year_ / 10 + display::num2ascii], display::vfd_buffer[0]);
              display::VFD::Font2Buffer(display::font_lib[vfd->year_ % 10 + display::num2ascii], display::vfd_buffer[1]);
              display::VFD::Font2Buffer(display::font_lib[display::MINUS], display::vfd_buffer[2]);
              display::VFD::Font2Buffer(display::font_lib[flash_flag? vfd->month_ / 10 + display::num2ascii : display::SP], display::vfd_buffer[3]);
              display::VFD::Font2Buffer(display::font_lib[flash_flag? vfd->month_ % 10 + display::num2ascii : display::SP], display::vfd_buffer[4]);
              display::VFD::Font2Buffer(display::font_lib[display::MINUS], display::vfd_buffer[5]);
              display::VFD::Font2Buffer(display::font_lib[vfd->date_ / 10 + display::num2ascii], display::vfd_buffer[6]);
              display::VFD::Font2Buffer(display::font_lib[vfd->date_ % 10 + display::num2ascii], display::vfd_buffer[7]);
            } else if (Setting_Unit == DATE) {
              display::VFD::Font2Buffer(display::font_lib[vfd->year_ / 10 + display::num2ascii], display::vfd_buffer[0]);
              display::VFD::Font2Buffer(display::font_lib[vfd->year_ % 10 + display::num2ascii], display::vfd_buffer[1]);
              display::VFD::Font2Buffer(display::font_lib[display::MINUS], display::vfd_buffer[2]);
              display::VFD::Font2Buffer(display::font_lib[vfd->month_ / 10 + display::num2ascii], display::vfd_buffer[3]);
              display::VFD::Font2Buffer(display::font_lib[vfd->month_ % 10 + display::num2ascii], display::vfd_buffer[4]);
              display::VFD::Font2Buffer(display::font_lib[display::MINUS], display::vfd_buffer[5]);
              display::VFD::Font2Buffer(display::font_lib[flash_flag? vfd->date_ / 10 + display::num2ascii : display::SP], display::vfd_buffer[6]);
              display::VFD::Font2Buffer(display::font_lib[flash_flag? vfd->date_ % 10 + display::num2ascii : display::SP], display::vfd_buffer[7]);
            } else if (Setting_Unit == DAY) {
              display::VFD::Font2Buffer(display::font_lib[display::GT], display::vfd_buffer[0]);
              display::VFD::Font2Buffer(display::font_lib[display::Bold_GT], display::vfd_buffer[1]);
              switch (vfd->day_) {
                case 1:
                  display::VFD::Font2Buffer(display::font_lib[flash_flag? display::M : display::SP], display::vfd_buffer[2]);
                  display::VFD::Font2Buffer(display::font_lib[flash_flag? display::o : display::SP], display::vfd_buffer[3]);
                  display::VFD::Font2Buffer(display::font_lib[flash_flag? display::n : display::SP], display::vfd_buffer[4]);
                  break ;
                case 2:
                  display::VFD::Font2Buffer(display::font_lib[flash_flag? display::T : display::SP], display::vfd_buffer[2]);
                  display::VFD::Font2Buffer(display::font_lib[flash_flag? display::u : display::SP], display::vfd_buffer[3]);
                  display::VFD::Font2Buffer(display::font_lib[flash_flag? display::e : display::SP], display::vfd_buffer[4]);
                  break ;
                case 3:
                  display::VFD::Font2Buffer(display::font_lib[flash_flag? display::W : display::SP], display::vfd_buffer[2]);
                  display::VFD::Font2Buffer(display::font_lib[flash_flag? display::e : display::SP], display::vfd_buffer[3]);
                  display::VFD::Font2Buffer(display::font_lib[flash_flag? display::d : display::SP], display::vfd_buffer[4]);
                  break ;
                case 4:
                  display::VFD::Font2Buffer(display::font_lib[flash_flag? display::T : display::SP], display::vfd_buffer[2]);
                  display::VFD::Font2Buffer(display::font_lib[flash_flag? display::h : display::SP], display::vfd_buffer[3]);
                  display::VFD::Font2Buffer(display::font_lib[flash_flag? display::u : display::SP], display::vfd_buffer[4]);
                  break ;
                case 5:
                  display::VFD::Font2Buffer(display::font_lib[flash_flag? display::F : display::SP], display::vfd_buffer[2]);
                  display::VFD::Font2Buffer(display::font_lib[flash_flag? display::r : display::SP], display::vfd_buffer[3]);
                  display::VFD::Font2Buffer(display::font_lib[flash_flag? display::i : display::SP], display::vfd_buffer[4]);
                  break ;
                case 6:
                  display::VFD::Font2Buffer(display::font_lib[flash_flag? display::S : display::SP], display::vfd_buffer[2]);
                  display::VFD::Font2Buffer(display::font_lib[flash_flag? display::a : display::SP], display::vfd_buffer[3]);
                  display::VFD::Font2Buffer(display::font_lib[flash_flag? display::t : display::SP], display::vfd_buffer[4]);
                  break ;
                case 7:
                  display::VFD::Font2Buffer(display::font_lib[flash_flag? display::S : display::SP], display::vfd_buffer[2]);
                  display::VFD::Font2Buffer(display::font_lib[flash_flag? display::u : display::SP], display::vfd_buffer[3]);
                  display::VFD::Font2Buffer(display::font_lib[flash_flag? display::n : display::SP], display::vfd_buffer[4]);
                  break ;
                default:
                  break ;
              }
              display::VFD::Font2Buffer(display::font_lib[display::Bold_LT], display::vfd_buffer[5]);
              display::VFD::Font2Buffer(display::font_lib[display::LT], display::vfd_buffer[6]);
              display::VFD::Font2Buffer(display::font_lib[display::SP], display::vfd_buffer[7]);
            } else if (Setting_Unit == HOUR) {
              display::VFD::Font2Buffer(display::font_lib[flash_flag? vfd->hour_ / 10 + display::num2ascii : display::SP], display::vfd_buffer[0]);
              display::VFD::Font2Buffer(display::font_lib[flash_flag? vfd->hour_ % 10 + display::num2ascii : display::SP], display::vfd_buffer[1]);
              display::VFD::Font2Buffer(display::font_lib[display::COLON], display::vfd_buffer[2]);
              display::VFD::Font2Buffer(display::font_lib[vfd->minute_ / 10 + display::num2ascii], display::vfd_buffer[3]);
              display::VFD::Font2Buffer(display::font_lib[vfd->minute_ % 10 + display::num2ascii], display::vfd_buffer[4]);
              display::VFD::Font2Buffer(display::font_lib[display::COLON], display::vfd_buffer[5]);
              display::VFD::Font2Buffer(display::font_lib[vfd->second_ / 10 + display::num2ascii], display::vfd_buffer[6]);
              display::VFD::Font2Buffer(display::font_lib[vfd->second_ % 10 + display::num2ascii], display::vfd_buffer[7]);
            } else if (Setting_Unit == MINUTE) {
              display::VFD::Font2Buffer(display::font_lib[vfd->hour_ / 10 + display::num2ascii], display::vfd_buffer[0]);
              display::VFD::Font2Buffer(display::font_lib[vfd->hour_ % 10 + display::num2ascii], display::vfd_buffer[1]);
              display::VFD::Font2Buffer(display::font_lib[display::COLON], display::vfd_buffer[2]);
              display::VFD::Font2Buffer(display::font_lib[flash_flag? vfd->minute_ / 10 + display::num2ascii : display::SP], display::vfd_buffer[3]);
              display::VFD::Font2Buffer(display::font_lib[flash_flag? vfd->minute_ % 10 + display::num2ascii : display::SP], display::vfd_buffer[4]);
              display::VFD::Font2Buffer(display::font_lib[display::COLON], display::vfd_buffer[5]);
              display::VFD::Font2Buffer(display::font_lib[vfd->second_ / 10 + display::num2ascii], display::vfd_buffer[6]);
              display::VFD::Font2Buffer(display::font_lib[vfd->second_ % 10 + display::num2ascii], display::vfd_buffer[7]);
            } else if (Setting_Unit == SECOND) {
              display::VFD::Font2Buffer(display::font_lib[vfd->hour_ / 10 + display::num2ascii], display::vfd_buffer[0]);
              display::VFD::Font2Buffer(display::font_lib[vfd->hour_ % 10 + display::num2ascii], display::vfd_buffer[1]);
              display::VFD::Font2Buffer(display::font_lib[display::COLON], display::vfd_buffer[2]);
              display::VFD::Font2Buffer(display::font_lib[vfd->minute_ / 10 + display::num2ascii], display::vfd_buffer[3]);
              display::VFD::Font2Buffer(display::font_lib[vfd->minute_ % 10 + display::num2ascii], display::vfd_buffer[4]);
              display::VFD::Font2Buffer(display::font_lib[display::COLON], display::vfd_buffer[5]);
              display::VFD::Font2Buffer(display::font_lib[flash_flag? vfd->second_ / 10 + display::num2ascii : display::SP], display::vfd_buffer[6]);
              display::VFD::Font2Buffer(display::font_lib[flash_flag? vfd->second_ % 10 + display::num2ascii : display::SP], display::vfd_buffer[7]);
            }

            if (button.negEdge()) {
              if (Setting_Unit == YEAR)
                Setting_Unit = MONTH;
              else if (Setting_Unit == MONTH)
                Setting_Unit = DATE;
              else if (Setting_Unit == DATE)
                Setting_Unit = DAY;
              else if (Setting_Unit == DAY)
                Setting_Unit = HOUR;
              else if (Setting_Unit == HOUR)
                Setting_Unit = MINUTE;
              else if (Setting_Unit == MINUTE)
                Setting_Unit = SECOND;
              else if (Setting_Unit == SECOND) {
                VFD_Clock_Mode = DISPLAY_TIME;
                clock->SetTime(vfd->year_, vfd->month_, vfd->date_, vfd->day_, vfd->hour_, vfd->minute_, vfd->second_);
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
