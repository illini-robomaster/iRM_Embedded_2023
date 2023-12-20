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
#include "iwdg.h"
#include "cmsis_os.h"
#include "bsp_print.h"
#include "stm32f4xx_hal_iwdg.h"
long reloop = 0;

void RM_RTOS_Init(void){
    print_use_uart(&huart1);
    // default &hiwdg got prescaler of 32 and reload of 1000
    // hiwdg is a IWDG_HandleTypeDef
    // Calculate the reload value according to the formula:
    // Reload_Value = ((Desired_Time_Out * 32kHz) / (Prescaler_Value * 4 * 1000)) - 1
}
bool is_LSI_Enabled() {
    // Check if LSI is enabled and ready
    if ((RCC->CSR & RCC_CSR_LSION) && (RCC->CSR & RCC_CSR_LSIRDY)) {
        return true;  // LSI is enabled and ready
    } else {
        return false; // LSI is not enabled
    }
}
void RM_RTOS_Default_Task(const void* arguments){
    UNUSED(arguments);
    HAL_Init();
//    reloop = 0;
    MX_IWDG_Init();
    // expected behavior: triggers a hard reset if not refreshed within the expected timeout
    while(true){
        print("%d\n", HAL_GetTick());
        osDelay(10);
        HAL_IWDG_Refresh(&hiwdg);
    }
}