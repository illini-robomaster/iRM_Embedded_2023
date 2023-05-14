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
#include "bsp_os.h"
#include "cmsis_os.h"
#include "main.h"
#include "motor.h"
#include "dbus.h"

#define TARGET_SPEED 30
#define NOTCH (2 * PI / 4)
#define SPEED 200
#define ACCELERATION (80 * PI)

static bsp::CAN* can = nullptr;
static control::Motor4310* motor = nullptr;
control::MotorCANBase* motor1 = nullptr;
control::MotorCANBase* motor2 = nullptr;
control::ServoMotor* servo = nullptr;
control::ServoMotor* servo2 = nullptr;
remote::DBUS* dbus = nullptr;

void RM_RTOS_Init() {
  print_use_uart(&huart8);
  bsp::SetHighresClockTimer(&htim5);

  can = new bsp::CAN(&hcan1, 0x201, true);
  dbus = new remote::DBUS(&huart1);

  motor = new control::Motor4310(can, 0x02, 0x01, control::MIT);

  motor1 = new control::Motor3508(can, 0x202);
  control::servo_t servo_data;
  servo_data.motor = motor1;
  servo_data.max_speed = SPEED;
  servo_data.max_acceleration = ACCELERATION;
  servo_data.transmission_ratio = M3508P19_RATIO;
  servo_data.omega_pid_param = new float[3]{150, 1.2, 5};
  servo_data.max_iout = 1000;
  servo_data.max_out = 13000;
  servo = new control::ServoMotor(servo_data);

  motor2 = new control::Motor3508(can, 0x203);
  control::servo_t servo_data2;
  servo_data2.motor = motor2;
  servo_data2.max_speed = SPEED;
  servo_data2.max_acceleration = ACCELERATION;
  servo_data2.transmission_ratio = M3508P19_RATIO;
  servo_data2.omega_pid_param = new float[3]{150, 1.2, 5};
  servo_data2.max_iout = 1000;
  servo_data2.max_out = 13000;
  servo2 = new control::ServoMotor(servo_data2);
}

void RM_RTOS_Default_Task(const void* args) {
  // need to press reset to begin
  UNUSED(args);
  osDelay(500);  // DBUS initialization needs time
  while(dbus->swr != remote::DOWN){}  // flip swr to start

  motor->SetZeroPos(motor);  // set zero position on startup; uncomment if a desired position is set
  motor->MotorEnable(motor);

  float pos = 0;
  float min_pos = -10;
  float max_pos = 10;
  float pos1 = 0;
  float min_pos1 = -10;
  float max_pos1 = 10;
  float pos2 = 0;
  float min_pos2 = -10;
  float max_pos2 = 10;

  while (true) {
    // m4310
    float vel;
    vel = clip<float>(dbus->ch0 / 660.0 * 15.0, -15, 15);
    pos += vel / 900;
    pos = clip<float>(pos, min_pos, max_pos);   // clipping position within a range

    set_cursor(0, 0);
    clear_screen();
    print("%d %d %d %d \n", dbus->ch0, dbus->ch1, dbus->ch2, dbus->ch3);

    motor->SetOutput(pos, vel, 95, 0.5, 0);
    motor->TransmitOutput(motor);

    // 1st m3508
    float vel1;
    vel1 = clip<float>(dbus->ch3 / 660.0 * 5, -10, 10);
    pos1 -= vel1 / 400;
    pos1 = clip<float>(pos1, min_pos1, max_pos1);   // clipping position within a range

    set_cursor(0, 0);
    clear_screen();
    print("ch1: %d\n", dbus->ch3);
    print("pos1: %f vel1: %f\n", pos1, vel1);

    servo->SetTarget(pos1, true);
    servo->CalcOutput();

    // 2nd m3508
    float vel2;
    vel2 = clip<float>(dbus->ch1 / 660.0 * 5, -10, 10);
    pos2 += vel2 / 400;
    pos2 = clip<float>(pos2, min_pos2, max_pos2);   // clipping position within a range

    set_cursor(0, 0);
    clear_screen();
    print("ch1: %d\n", dbus->ch1);
    print("pos1: %f vel1: %f\n", pos2, vel2);

    servo2->SetTarget(pos2, true);
    servo2->CalcOutput();

    control::MotorCANBase* motors[] = {motor1, motor2};
    control::MotorCANBase::TransmitOutput(motors, 2);

    osDelay(2);
  }
}
