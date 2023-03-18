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

// #define WITH_CONTROLLER

/* This is example is for using three kinds of mode of shooting by using the dbus.
 * dbus->swr == MID: Stationary state, Dial doesn't move
 * dbus->swr == UP: Fast continue shooting mode, Dial runs fast to supply the bullets
 * dbus->swr == DOWN(less than 0.5s, then switch back to MID): Double shooting mode, Dial runs pi/2 to supply 2 bullets at once.
 * dbus->swr == DOWN(more than 0.5s): Slowly continue shooting mode, Dial runs slowly to supply the bullets
 */

#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "main.h"
#include "motor.h"
#include "utils.h"
#include "dbus.h"


#define KEY_GPIO_GROUP GPIOB
#define KEY_GPIO_PIN GPIO_PIN_2

#define NOTCH (2 * PI / 8)
#define LOAD_ANGLE_CONTINUE (2 * PI / 8)
#define LOAD_ANGLE_DOUBLE (2 * PI / 4)
#define SPEED (6 * PI)
#define ACCELERATION_DOUBLE (100 * PI)
#define ACCELERATION_CONTINUE (200 * PI)
#define ACCELERATION_CONTINUE_SLOWLY (20 * PI)
#define DELAY (5e5)

bsp::CAN* can1 = nullptr;
control::MotorCANBase* motor = nullptr;
control::ServoMotor* servo = nullptr;

remote::DBUS* dbus = nullptr;

void jam_callback(control::ServoMotor* servo, const control::servo_jam_t data) {
  UNUSED(data);
  float servo_target = servo->GetTarget();
  if (servo_target < servo->GetTheta()) {
    print("Antijam in operation\r\n");
  } else {
    servo->SetTarget(servo->GetTheta(), true);
    float prev_target = servo->GetTarget() - NOTCH;
    servo->SetTarget(prev_target, true);
    print("Antijam engage\r\n");
  }
}

void RM_RTOS_Init() {
  print_use_uart(&huart8);
  bsp::SetHighresClockTimer(&htim2);

  can1 = new bsp::CAN(&hcan1, 0x201);
  motor = new control::Motor2006(can1, 0x202);

  control::servo_t servo_data;
  servo_data.motor = motor;
  servo_data.max_speed = SPEED;
  servo_data.max_acceleration = ACCELERATION_DOUBLE;
  servo_data.transmission_ratio = M2006P36_RATIO;
  servo_data.omega_pid_param = new float[3]{150, 4, 0};
  servo_data.max_iout = 2000;
  servo_data.max_out = 10000;
  servo = new control::ServoMotor(servo_data);

  servo->RegisterJamCallback(jam_callback, 0.304);
  dbus = new remote::DBUS(&huart1);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  control::MotorCANBase* motors[] = {motor};
  uint32_t start_time = 0;
  int slow_shoot_detect = 0;

  while (true) {
    if (dbus->swr == remote::UP) {
      servo->SetTarget(servo->GetTarget() + LOAD_ANGLE_CONTINUE, false);
      servo->SetMaxSpeed(SPEED);
      servo->SetMaxAcceleration(ACCELERATION_CONTINUE);
    } else if (dbus->swr == remote::MID){
      servo->SetMaxSpeed(0);
      start_time = bsp::GetHighresTickMicroSec();
      slow_shoot_detect = 0;
    } else {
      if (bsp::GetHighresTickMicroSec() - start_time > DELAY) {
        servo->SetTarget(servo->GetTarget() + LOAD_ANGLE_CONTINUE, false);
        servo->SetMaxSpeed(SPEED);
        servo->SetMaxAcceleration(ACCELERATION_CONTINUE_SLOWLY);
      } else {
        if (slow_shoot_detect == 0) {
          slow_shoot_detect = 1;
          servo->SetTarget(servo->GetTarget() + LOAD_ANGLE_DOUBLE);
          servo->SetMaxSpeed(SPEED);
          servo->SetMaxAcceleration(ACCELERATION_DOUBLE);
        }
      }
    }
    servo->CalcOutput();
    control::MotorCANBase::TransmitOutput(motors, 1);
    osDelay(2);
  }
}
