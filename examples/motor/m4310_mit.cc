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

#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "motor.h"
#include "sbus.h"
#include "dbus.h"

// static bsp::CAN* can = nullptr;
// static control::Motor4310* motor = nullptr;
// remote::SBUS* dbus = nullptr;
// bsp::GPIO* key = nullptr;

// void RM_RTOS_Init() {
//   print_use_uart(&huart1);
//   can = new bsp::CAN(&hcan1, true);
//   key = new bsp::GPIO(GPIOA, GPIO_PIN_15);

//   /* rx_id = Master id
//    * tx_id = CAN id
//    * mode:
//    *  MIT: MIT mode
//    *  POS_VEL: position-velocity mode
//    *  VEL: velocity mode  */

//   /* Make sure motor is set to the correct mode (in helper tool). Otherwise, motor won't start */
//   motor = new control::Motor4310(can, 0x04, 0x05, control::MIT);
//   dbus = new remote::SBUS(&huart3);
// }

// void RM_RTOS_Default_Task(const void* args) {
//   /* press reset if no response */
//   UNUSED(args);

//   control::Motor4310* motors[] = {motor};
//   // while(dbus->swr != remote::DOWN){}  // flip swr to start
//   print("start\r\n");

//   /* Use SetZeroPos if you want to set current motor position as zero position. If uncommented, the
//    * zero position is the zero position set before */
//   // motor->SetZeroPos();

//   while(dbus->ch[5] < 100){}  // flip swr t;
//   // motor->SetZeroPos();
//   // osDelay(100);

//   motor->MotorEnable();

//   while (motor->GetTheta() == 0.0) {
//     print("Waiting for motor to initialize...\r\n");
//   }

//   float curr_motor_pos = motor->GetTheta();
//   float motor_pos_delta = motor->GetTheta() / 100.0;
  
//   for (int i = 0; i < 100; i++) {
//     motor->SetOutput(curr_motor_pos, 1, 30, 0.5, 0);
//     print("motor pos: %f\r\n", curr_motor_pos);
//     control::Motor4310::TransmitOutput(motors, 1);
//     curr_motor_pos -= motor_pos_delta;
//     osDelay(10);
//   }
  
//   print("motor enabled\r\n" );

//   float pos = 0;
//   // pitch: -0.3327 ~ 0.2514
//   // yaw: -1.3053 ~ 1.6151
//   float min_pos = -0.3327;    
//   float max_pos = 0.2514;
//   while (true) {
//     float vel;
//     vel = clip<float>(dbus->ch[1] / 660.0 * 15.0, -15, 15);
//     pos += vel / 200;
//     pos = clip<float>(pos, min_pos, max_pos);   // clipping position within a range

//     set_cursor(0, 0);
//     clear_screen();
//     // print("Vel Set: %f  Pos Set: %f\n", vel, pos);
//     print("Theta: %.4f   Omega: %.4f\n", motor->GetTheta(), motor->GetOmega());

//     motor->SetOutput(pos, vel, 30, 0.5, 0);
//     control::Motor4310::TransmitOutput(motors, 1);
//     osDelay(10);
//   }
// }






static bsp::CAN* can = nullptr;
static control::Motor4310* motor = nullptr;
remote::DBUS* dbus = nullptr;
// bsp::GPIO* key = nullptr;

void RM_RTOS_Init() {
  print_use_uart(&huart1);
  can = new bsp::CAN(&hcan1, true);
  // key = new bsp::GPIO(GPIOA, GPIO_PIN_15);

  /* rx_id = Master id
   * tx_id = CAN id
   * mode:
   *  MIT: MIT mode
   *  POS_VEL: position-velocity mode
   *  VEL: velocity mode  */

  /* Make sure motor is set to the correct mode (in helper tool). Otherwise, motor won't start */
  motor = new control::Motor4310(can, 0x04, 0x05, control::MIT);
  dbus = new remote::DBUS(&huart3);
}

void RM_RTOS_Default_Task(const void* args) {
  /* press reset if no response */
  UNUSED(args);

  control::Motor4310* motors[] = {motor};
  // while(dbus->swr != remote::DOWN){}  // flip swr to start
  print("start\r\n");

  /* Use SetZeroPos if you want to set current motor position as zero position. If uncommented, the
   * zero position is the zero position set before */
  // motor->SetZeroPos();

  while(dbus->swr != remote::DOWN){} // flip swr t;
  // motor->SetZeroPos();
  // osDelay(100);

  motor->MotorEnable();
  print("here\r\n");

  while (motor->GetTheta() == 0.0) {
    print("Waiting for motor to initialize...\r\n");
  }

  float curr_motor_pos = motor->GetTheta();
  float motor_pos_delta = motor->GetTheta() / 100.0;
  
  for (int i = 0; i < 100; i++) {
    motor->SetOutput(curr_motor_pos, 1, 30, 0.5, 0);
    print("motor pos: %f\r\n", curr_motor_pos);
    control::Motor4310::TransmitOutput(motors, 1);
    curr_motor_pos -= motor_pos_delta;
    osDelay(10);
  }
  
  print("motor enabled\r\n" );

  float pos = 0;
  // pitch: -0.3327 ~ 0.2514
  // yaw: -1.3053 ~ 1.6151
  float min_pos = -0.3327;    
  float max_pos = 0.2514;
  // float min_pos = -1.3053;
  // float max_pos = 1.6151;
  while (true) {
    float vel;
    vel = clip<float>(dbus->ch1 / 660.0 * 15.0, -15, 15);
    pos += vel / 200;
    pos = clip<float>(pos, min_pos, max_pos);   // clipping position within a range

    set_cursor(0, 0);
    clear_screen();
    // print("Vel Set: %f  Pos Set: %f\n", vel, pos);
    print("Theta: %.4f   Omega: %.4f\n", motor->GetTheta(), motor->GetOmega());

    motor->SetOutput(pos, vel, 30, 0.5, 0);
    control::Motor4310::TransmitOutput(motors, 1);
    osDelay(10);
  }
}
