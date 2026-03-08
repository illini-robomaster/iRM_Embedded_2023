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

#include "bsp_can.h"
#include "bsp_print.h"
#include "bsp_uart.h"
#include "can.h"
#include "cmsis_os.h"
#include "controller.h"
#include "i2c.h"
#include "motor.h"
#include "oled.h"

#define MOTOR_SPEED 200  // RPM, positive = forward

static display::OLED*          OLED  = nullptr;
static bsp::CAN*               can   = nullptr;
static control::MotorCANBase*  motor = nullptr;
static control::PIDController* pid   = nullptr;

void RM_RTOS_Init(void) {
  print_use_uart(&huart1);
  OLED  = new display::OLED(&hi2c2, 0x3C);
  can   = new bsp::CAN(&hcan1, true);
  motor = new control::Motor3508(can, 0x202);
  pid   = new control::PIDController(20, 15, 30);

  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

// ── helpers ───────────────────────────────────────────────────────────────

static bool ReadKey(uint16_t pin) {
  return HAL_GPIO_ReadPin(GPIOC, pin) == GPIO_PIN_RESET;
}

// Invert all pixels in text row `text_row` (0-based, each row is 12 px tall)
// over the full screen width [0, 127].  Called *after* Printf so the normal
// white-on-black text is flipped to black-on-white (highlight effect).
static void InvertTextRow(uint8_t text_row) {
  const uint8_t y_start = text_row * 12;
  const uint8_t y_end   = y_start + 11;
  for (uint8_t y = y_start; y <= y_end; ++y) {
    OLED->DrawLine(0, y, 127, y, display::PEN_INVERSION);
  }
}

// ── content ───────────────────────────────────────────────────────────────

static const uint8_t NUM_ITEMS = 2;
static const char* ITEM_LABEL[]  = {"Joey is GOAT", "Daniel is Qu"};
static const char* ITEM_DETAIL[] = {"GOAT",          "Qu"};

// ── rendering ─────────────────────────────────────────────────────────────

static void DrawMenu(uint8_t cursor) {
  OLED->OperateGram(display::PEN_CLEAR);
  OLED->Printf(0, 2, "--- MENU ---");
  for (uint8_t i = 0; i < NUM_ITEMS; ++i) {
    OLED->Printf(i + 1, 2, ITEM_LABEL[i]);  // items at row 1 and 2
  }
  OLED->Printf(4, 0, "K1^ K2v K3:OK");
  // Highlight selected row: items live at text rows (cursor+1)
  InvertTextRow(cursor + 1);
  OLED->RefreshGram();
}

static void DrawDetail(uint8_t item) {
  OLED->OperateGram(display::PEN_CLEAR);
  OLED->Printf(0, 2, ITEM_LABEL[item]);
  OLED->Printf(2, 2, ITEM_DETAIL[item]);
  OLED->Printf(4, 0, "K4:Back");
  OLED->RefreshGram();
}

// ── task ──────────────────────────────────────────────────────────────────

void RM_RTOS_Default_Task(const void* arg) {
  UNUSED(arg);

  OLED->ShowRMLOGO();
  osDelay(2000);
  OLED->ShowIlliniRMLOGO();
  osDelay(2000);

  typedef enum { STATE_MENU, STATE_DETAIL } AppState;

  AppState state    = STATE_MENU;
  uint8_t  cursor   = 0;
  bool     need_redraw = true;

  bool     prev_k1 = false, prev_k2 = false,
           prev_k3 = false, prev_k4 = false;
  uint32_t last_k1 = 0,    last_k2 = 0,
           last_k3 = 0,    last_k4 = 0;

  while (true) {
    const bool     k1  = ReadKey(GPIO_PIN_0);
    const bool     k2  = ReadKey(GPIO_PIN_1);
    const bool     k3  = ReadKey(GPIO_PIN_2);
    const bool     k4  = ReadKey(GPIO_PIN_3);
    const uint32_t now = HAL_GetTick();

    // K1 (PC0) – move cursor up
    if (k1 && !prev_k1 && (now - last_k1 >= 200)) {
      last_k1 = now;
      if (state == STATE_MENU) {
        cursor = (cursor == 0) ? (NUM_ITEMS - 1) : cursor - 1;
        need_redraw = true;
      }
    }
    // K2 (PC1) – move cursor down
    if (k2 && !prev_k2 && (now - last_k2 >= 200)) {
      last_k2 = now;
      if (state == STATE_MENU) {
        cursor = (cursor + 1) % NUM_ITEMS;
        need_redraw = true;
      }
    }
    // K3 (PC2) – confirm
    if (k3 && !prev_k3 && (now - last_k3 >= 200)) {
      last_k3 = now;
      if (state == STATE_MENU) {
        state = STATE_DETAIL;
        need_redraw = true;
      }
    }
    // K4 (PC3) – cancel / back
    if (k4 && !prev_k4 && (now - last_k4 >= 200)) {
      last_k4 = now;
      if (state == STATE_DETAIL) {
        state = STATE_MENU;
        need_redraw = true;
      }
    }

    prev_k1 = k1;  prev_k2 = k2;
    prev_k3 = k3;  prev_k4 = k4;

    if (need_redraw) {
      if (state == STATE_MENU) DrawMenu(cursor);
      else                      DrawDetail(cursor);
      need_redraw = false;
    }

    // ── Motor control ────────────────────────────────────────────────────
    {
      control::MotorCANBase* motors[] = {motor};
      if (state == STATE_DETAIL) {
        // cursor 0 = Joey is GOAT → forward, cursor 1 = Daniel is Qu → reverse
        float   target = (cursor == 0) ? MOTOR_SPEED : -MOTOR_SPEED;
        float   diff   = motor->GetOmegaDelta(target);
        int16_t out    = pid->ComputeConstrainedOutput(diff);
        motor->SetOutput(out);
      } else {
        motor->SetOutput(0);
      }
      control::MotorCANBase::TransmitOutput(motors, 1);
    }

    osDelay(10);
  }
}
