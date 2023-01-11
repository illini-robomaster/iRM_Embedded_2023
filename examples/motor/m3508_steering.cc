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
control::MotorCANBase* motor = nullptr;
control::SteeringMotor* steering = nullptr;

bsp::GPIO* key = nullptr;

BoolEdgeDetector key_detector(false);

bool steering_align_detect() {
  // float theta = wrap<float>(steering->GetRawTheta(), 0, 2 * PI);
  // return abs(theta - 3) < 0.05;
  return key->Read() == 1;
  // return true;
}

void RM_RTOS_Init() {
  print_use_uart(&huart8);
  bsp::SetHighresClockTimer(&htim5);

  can1 = new bsp::CAN(&hcan1, 0x201, true);
  motor = new control::Motor3508(can1, 0x203);

  key = new bsp::GPIO(KEY_GPIO_GROUP, KEY_GPIO_PIN);

  control::steering_t steering_data;
  steering_data.motor = motor;
  steering_data.max_speed = SPEED;
  steering_data.test_speed = TEST_SPEED;
  steering_data.max_acceleration = ACCELERATION;
  steering_data.transmission_ratio = M3508P19_RATIO;
  steering_data.omega_pid_param = new float[3]{140, 1.2, 25};
  steering_data.max_iout = 1000;
  steering_data.max_out = 13000;
  steering_data.align_detect_func = steering_align_detect;
  steering_data.calibrate_offset = 0;
  //steering_data.calibrate_offset = PI/2;
  steering = new control::SteeringMotor(steering_data);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  control::MotorCANBase* motors[] = {motor};

  osDelay(500);  // DBUS initialization needs time

  // Press Key to start aligning. Else sudden current change when power is switched on might break
  // the board.
  while(!key->Read());

  // wait for release because align_detect also is key press here
  while(key->Read());

  print("Alignment Begin\r\n");
  // Don't put AlignUpdate() in the while case
  bool alignment_complete = false;
  while (!alignment_complete) {
    alignment_complete = steering->AlignUpdate();
    if (!alignment_complete) {
      control::MotorCANBase::TransmitOutput(motors, 1);
      osDelay(2);
    }
  }

  print("\r\nAlignment End\r\n");

  osDelay(500);

  print("\r\nOK!\r\n");

  int dir = 1;

  while (true) {
    key_detector.input(key->Read());
    if (key_detector.posEdge()){
      if (dir == 1) {
        // Motor should turn the give angle
        steering->TurnRelative(PI * 4 + PI / 4);
        steering->PrintData();
      } else {
        // Motor should go to align angle
        steering->AlignUpdate();
      }
      dir *= -1;
    }

    steering->CalcOutput();
    control::MotorCANBase::TransmitOutput(motors, 1);
    osDelay(2);
  }
}
