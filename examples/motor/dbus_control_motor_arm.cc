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

#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "dbus.h"
#include "main.h"
#include "motor.h"
#include "utils.h"

//#define KEY_GPIO_GROUP GPIOB
//#define KEY_GPIO_PIN GPIO_PIN_2

constexpr float RUN_SPEED = (10 * PI);
constexpr float ALIGN_SPEED = (0.5 * PI / 10);
constexpr float ACCELERATION = (100 * PI);

bsp::CAN* can1 = nullptr;
control::MotorCANBase* motor1 = nullptr;
control::MotorCANBase* motor3 = nullptr;
control::SteeringMotor* steering1 = nullptr;
control::SteeringMotor* steering3 = nullptr;

static remote::DBUS *dbus = nullptr;

//bsp::GPIO* key1 = nullptr;
//bsp::GPIO* key3 = nullptr;

//oolEdgeDetector key_detector(false);

// temporal alignment sensor for testing
// bool steering_align_detect1() {
//   return key1->Read() == 1;
// }

// bool steering_align_detect3() {
//   return key3->Read() == 1;
// }

void RM_RTOS_Init() {
  print_use_uart(&huart8);
  bsp::SetHighresClockTimer(&htim5);

  can1 = new bsp::CAN(&hcan1, 0x201, true);
  motor1 = new control::Motor3508(can1, 0x201);
  motor3 = new control::Motor3508(can1, 0x202);

  dbus = new remote::DBUS(&huart1);

  /* Usage:
   *   The 'key' is the white button on TypeA boards
   *   Press key to start alignment and then press key again to finish motor1 alignment.
   *   Now the align angle is recorded:
   *   Press key once will turn the two motors to their angles.
   *   Press key again will turn them back to their align angles.
  **/
  // key1 = new bsp::GPIO(KEY_GPIO_GROUP, KEY_GPIO_PIN);
  // // GPIO L1 to align motor3. The switch should be wired from L1 pin to R2 pin (5V VCC)
  // key3 = new bsp::GPIO(GPIOC, GPIO_PIN_2);

  control::steering_t steering_data;
  steering_data.motor = motor1;
  steering_data.max_speed = RUN_SPEED;
  steering_data.max_acceleration = ACCELERATION;
  steering_data.transmission_ratio = M3508P19_RATIO;
  steering_data.omega_pid_param = new float[3]{140, 1.2, 25};
  steering_data.max_iout = 1000;
  steering_data.max_out = 13000;
  steering_data.calibrate_offset = 0;
  //steering_data.calibrate_offset = PI/2;

  //steering_data.align_detect_func = steering_align_detect1;
  steering1 = new control::SteeringMotor(steering_data);
  steering_data.motor = motor3;
  //steering_data.align_detect_func = steering_align_detect3;
  steering3 = new control::SteeringMotor(steering_data);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  control::MotorCANBase* motors[] = {motor1, motor3};
  //control::PIDController pid1(20, 15, 30);
  //control::PIDController pid3(20, 15, 30);
  //control::MotorCANBase* motors[] = {motor1};

  osDelay(500);  // DBUS initialization needs time

  // Press Key to start aligning. Else sudden current change when power is switched on might break
  // the board.
  // while(!key1->Read());

  // // wait for release because align_detect also is key press here
  // while(key1->Read());

  print("Alignment Begin\r\n");
  steering1->SetMaxSpeed(ALIGN_SPEED);
  steering3->SetMaxSpeed(ALIGN_SPEED);
  control::MotorCANBase::TransmitOutput(motors, 2);

  // Wait for key to release
  osDelay(500);

  print("\r\nOK!\r\n");

  float pos = 0;
  float pos2 = 0;

  while (true) {
    /*float diff1 = motor1->GetOmegaDelta(dbus->ch1 / 50);
    float diff2 = motor3->GetOmegaDelta(dbus->ch3 / 50);
    int16_t out1 = pid1.ComputeConstrainedOutput(diff1);
    int16_t out2 = pid3.ComputeConstrainedOutput(diff2);
    motor1->SetOutput(out1);
    motor3->SetOutput(out2);*/

    float vel;
    float vel2;

    //key_detector.input(key1->Read());
    //if (key_detector.posEdge()){
      // Motor should turn the give angle
      
      vel = clip<float>(dbus->ch1 / 660.0 * 30.0, -30, 30);
      vel2 = clip<float>(dbus->ch3 / 660.0 * 30.0, -30, 30);
      pos += vel / 1000;
      pos2 += vel2 / 1000;
      pos = clip<float>(pos, -PI/4, PI/4);
      pos2 = clip<float>(pos2, -PI/4, PI/4);

      clear_screen();
      set_cursor(0, 0);
      print("vel set: %f pos set: %f\r\n", vel, pos);
      print("vel2 set: %f pos2 set: %f\r\n", vel2, pos2);
 
      steering1->TurnRelative(PI/4);
      steering3->TurnRelative(PI/4);
    //}

    
    steering1->CalcOutput();
    steering3->CalcOutput();

    control::MotorCANBase::TransmitOutput(motors, 2);
    osDelay(2);
  }
}