/****************************************************************************
*                                                                          *
*  Copyright (C) 2024 RoboMaster.                                          *
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

#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "motor.h"
#include "dbus.h"
#include "sbus.h"

#define SBUS_START_IDX 5
#define WINDOW_SIZE 50
#define FILTER_THRESHOLD 850

static bsp::CAN* can = nullptr;
static control::Motor4310* motor1 = nullptr;
static control::Motor4310* motor2 = nullptr;
static control::Motor4310* motor3 = nullptr;
//static remote::DBUS* dbus = nullptr;
static remote::SBUS* sbus = nullptr;
static bsp::GPIO* key = nullptr;
static float sbus_prev[6] = {0, 0, 0, 0, 0, 0};
static int sbus_prev_2d[6][WINDOW_SIZE] = {{0}};
static float sbus_avg[6] = {0, 0, 0, 0, 0, 0};

void RM_RTOS_Init() {
 print_use_uart(&huart1);
 can = new bsp::CAN(&hcan1, true);

 /* rx_id = Master id
  * tx_id = CAN id
  * mode:
  *  MIT: MIT mode
  *  POS_VEL: position-velocity mode
  *  VEL: velocity mode  */

 /* Make sure motor is set to the correct mode (in helper tool). Otherwise, motor won't start */
 motor1 = new control::Motor4310(can, 0x02, 0x01, control::MIT);
 motor2 = new control::Motor4310(can, 0x04, 0x03, control::MIT);
 motor3 = new control::Motor4310(can, 0x06, 0x05, control::MIT);
// dbus = new remote::DBUS(&huart3);
 sbus = new remote::SBUS(&huart3);
 key = new bsp::GPIO(GPIOA, GPIO_PIN_0);
}

void RM_RTOS_Default_Task(const void* args) {
 /* press reset if no response */
 UNUSED(args);
 control::Motor4310* motors[] = {motor1, motor2, motor3};

 print("Press key to start\r\n");
 while(key->Read());
 print("Start\r\n");

 /* Use SetZeroPos if you want to set current motor position as zero position.
  * If uncommented, the zero position is the zero position set before */
 motor1->SetZeroPos();
 motor1->MotorEnable();
 motor2->SetZeroPos();
 motor2->MotorEnable();
 motor3->SetZeroPos();
 motor3->MotorEnable();

 float pos_1 = 0.0;
 float pos_2 = 0.0;
 float pos_3 = 0.0;
 float vel_1 = 0.0;
 float vel_2 = 0.0;
 float vel_3 = 0.0;

 float min_pos1 = -PI/2;
 float max_pos1 = PI/2;
 float min_pos2 = -PI/2;
 float max_pos2 = PI/2;
 float min_pos3 = -PI/2;
 float max_pos3 = PI/2;

 float impulse_filtered[6] = {0, 0, 0, 0, 0, 0};
 float sbus_derivative[6] = {0, 0, 0, 0, 0, 0};

 while (true) {
   print("test\r\n");
   /* Average Filter */
   for (int i = SBUS_START_IDX; i < SBUS_START_IDX + 6; i++) {
     int index = i - SBUS_START_IDX;  // Adjust index for zero-based array indexing

     for (int j = 0; j < WINDOW_SIZE - 1; j++) {
       sbus_prev_2d[index][j] = sbus_prev_2d[index][j + 1];
     }
     sbus_prev_2d[index][WINDOW_SIZE-1] = sbus->ch[i];

     float sum = 0;
     for (int j = 0; j < WINDOW_SIZE; j++) {
       sum += sbus_prev_2d[index][j];
     }
     sbus_avg[index] = sum / static_cast<float>(WINDOW_SIZE);
   }

   print("test2\r\n");
   /* Impulse Filter */
   for (int i = SBUS_START_IDX; i < SBUS_START_IDX + 6; i++) {
     int index = i - SBUS_START_IDX;  // Adjust index for zero-based array indexing
     impulse_filtered[index] = sbus_avg[index];

     // impulse filtering
     sbus_derivative[index] = (sbus_avg[index] - sbus_prev[index]);  // TODO: check range
     if (abs(sbus_derivative[index]) > 30.0) {
       impulse_filtered[index] = sbus_prev[index];  // use previous value if impulse is too high
     } else {
       sbus_prev[index] = sbus_avg[index];
     }
     // update previous value only if signal is within range
     //     if (abs(sbus->ch[i]) < FILTER_THRESHOLD) sbus_prev[index] = sbus->ch[i];
   }

//   for (int i = SBUS_START_IDX; i < SBUS_START_IDX + 6; i++) {
//     int index = i - SBUS_START_IDX;  // Adjust index for zero-based array indexing
//     sbus_filtered[index] = impulse_filter(sbus->ch[i], &sbus_prev[index], SBUS_THRESHOLD);
//   }

   vel_1 = clip<float>(impulse_filtered[5] / 680.0 * PI/2, -PI/2, PI/2);
   pos_1 += vel_1 / 300;
   pos_1 = clip<float>(pos_1, min_pos1, max_pos1);   // clipping position within a range

   vel_2 = clip<float>(impulse_filtered[6] / 680.0 * PI/2, -PI/2, PI/2);
   pos_2 += vel_2 / 300;
   pos_2 = clip<float>(pos_2, min_pos2, max_pos2);   // clipping position within a range

   vel_3 = clip<float>(impulse_filtered[7] / 680.0 * PI/2, -PI/2, PI/2);
   pos_3 += vel_3 / 300;
   pos_3 = clip<float>(pos_3, min_pos3, max_pos3);   // clipping position within a range

   set_cursor(0, 0);
   clear_screen();
//   print("d: %-4d, %-4d, %-4d, %-4d, %-4d, %-4d\n", sbus->ch[10], sbus->ch[11], sbus->ch[12], sbus->ch[13], sbus->ch[14], sbus->ch[15]);
   print("d: %-4d, %-4f, %-4f, %-4f\n", sbus->ch[5], sbus_avg[0], sbus_derivative[0], impulse_filtered[0]);
//   print("d: %-4d, %-4d, %-4d\n", sbus->ch[0], sbus_actual, sbus_derivative);
//   print("d: %-4d, %-4d, %-4d\n", sbus->ch[1], sbus_actual2, sbus_derivative2);

   //   print("Vel Get: %f,  Pos Get: %f\n", vel_get, pos_get);

//   motor1->SetOutput(vel_1, float(sbus_filtered[0]), 10, 0.5, 0);
//   motor2->SetOutput(vel_2, float(sbus_filtered[0]), 10, 0.5, 0);
//   motor3->SetOutput(vel_3, vel_3, 10, 0.5, 0);
   control::Motor4310::TransmitOutput(motors, 3);
   osDelay(10);
 }
}
