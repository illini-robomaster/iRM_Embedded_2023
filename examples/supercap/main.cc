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

#include "bsp_print.h"
#include "cmsis_os.h"
#include "oled.h"
#include "supercap.h"

static bsp::CAN* can = nullptr;
static control::SuperCap* supercap = nullptr;
static display::OLED* OLED = nullptr;

void RM_RTOS_Init(void) {
  print_use_uart(&huart1);

  can = new bsp::CAN(&hcan2, false);
  supercap = new control::SuperCap(can, 0x301);
  OLED = new display::OLED(&hi2c2, 0x3C);
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);

  OLED->ShowIlliniRMLOGO();
  osDelay(200);
  OLED->OperateGram(display::PEN_CLEAR);

  while (true) {
    OLED->Printf(0, 0, "Supercap: %.2f %.2f", supercap->info.voltage, supercap->info.chassis_power);
    osDelay(100);
    OLED->RefreshGram();

    set_cursor(0, 0);
    clear_screen();
    print("Supercap\r\nVoltage: %.2f, Energy: %.2f\r\n", supercap->info.voltage,
          supercap->info.chassis_power);
  }
}
