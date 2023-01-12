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

#define KEY_GPIO_GROUP GPIOB
#define KEY_GPIO_PIN GPIO_PIN_2

#define SPEED (10 * PI)
#define TEST_SPEED (0.5 * PI)
#define ACCELERATION (100 * PI)

bsp::CAN* can1 = nullptr;
control::MotorCANBase* motor1 = nullptr;
control::MotorCANBase* motor3 = nullptr;
control::SteeringMotor* steering1 = nullptr;
control::SteeringMotor* steering3 = nullptr;

bsp::GPIO* key = nullptr;

BoolEdgeDetector key_detector(false);

// temporal alignment sensor for testing
bool steering_align_detect() {
  return key->Read() == 1;
}

void RM_RTOS_Init() {
  print_use_uart(&huart8);
  bsp::SetHighresClockTimer(&htim5);

  can1 = new bsp::CAN(&hcan1, 0x201, true);
  motor1 = new control::Motor3508(can1, 0x201);
  motor3 = new control::Motor3508(can1, 0x203);

  /* Usage:
   *   The 'key' is the white button on TypeA boards
   *   Press key to start alignment and then press key again to finish alignment.
   *   Now the align angle is recorded:
   *   Press key once will turn the motors to an angle.
   *   Press key again will turn them to the align angle.
  **/
  key = new bsp::GPIO(KEY_GPIO_GROUP, KEY_GPIO_PIN);

  control::steering_t steering_data;
  steering_data.motor = motor1;
  steering_data.run_speed = SPEED;
  steering_data.test_speed = TEST_SPEED;
  steering_data.max_acceleration = ACCELERATION;
  steering_data.transmission_ratio = M3508P19_RATIO;
  steering_data.omega_pid_param = new float[3]{140, 1.2, 25};
  steering_data.max_iout = 1000;
  steering_data.max_out = 13000;
  steering_data.align_detect_func = steering_align_detect;
  steering_data.calibrate_offset = 0;
  //steering_data.calibrate_offset = PI/2;

  steering1 = new control::SteeringMotor(steering_data);
  steering_data.motor = motor3;
  steering3 = new control::SteeringMotor(steering_data);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  control::MotorCANBase* motors[] = {motor1, motor3};

  osDelay(500);  // DBUS initialization needs time

  // Press Key to start aligning. Else sudden current change when power is switched on might break
  // the board.
  while(!key->Read());

  // wait for release because align_detect also is key press here
  while(key->Read());

  print("Alignment Begin\r\n");
  // Don't put AlignUpdate() in the while case
  bool alignment_complete1 = false;
  bool alignment_complete3 = false;
  while (!alignment_complete1 || !alignment_complete3) {
    steering1->CalcOutput();
    steering3->CalcOutput();
    control::MotorCANBase::TransmitOutput(motors, 2);
    alignment_complete1 = steering1->AlignUpdate();
    alignment_complete3 = steering3->AlignUpdate();
    osDelay(2);
  }

  steering1->TurnRelative(0, true);
  steering3->TurnRelative(0, true);

  print("\r\nAlignment End\r\n");

  // Wait for key to release
  osDelay(500);

  print("\r\nOK!\r\n");

  int dir = 1;

  while (true) {
    key_detector.input(key->Read());
    if (key_detector.posEdge()){
      if (dir == 1) {
        // Motor should turn the give angle
        steering1->TurnRelative(PI * 4 + PI / 4);
        steering3->TurnRelative(PI / 3);
      } else {
        // Motor should go to align angle
        steering1->AlignUpdate();
        steering3->AlignUpdate();
      }
      dir *= -1;
    }

    steering1->CalcOutput();
    steering3->CalcOutput();

    control::MotorCANBase::TransmitOutput(motors, 2);
    osDelay(2);
  }
}
