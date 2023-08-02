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

#include "chassis.h"

#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "dbus.h"
#include "main.h"
#include "motor.h"
#include "protocol.h"
#include "rgb.h"
#include "oled.h"
#include "bsp_buzzer.h"
#include "bsp_laser.h"

// Global Variables
static remote::DBUS* dbus = nullptr;
static bsp::CAN* can1 = nullptr;
static display::RGB* RGB = nullptr;

// Special Modes
static BoolEdgeDetector LoadDetect(false);
static BoolEdgeDetector ForceDetect(false);

static control::MotorCANBase* force_motor = nullptr;
static control::MotorCANBase* load_motor = nullptr;
static control::MotorCANBase* reload_motor = nullptr;
static control::PDIHV* trigger = nullptr; // CCW for positive angle, CW for negative angle
static control::ServoMotor* load_servo = nullptr;
static control::ServoMotor* reload_servo = nullptr;
static control::ServoMotor* force_servo = nullptr;

void RM_RTOS_Init() {
  // peripherals initialization
  print_use_uart(&huart1);
  bsp::SetHighresClockTimer(&htim5);
  // Dbus
  dbus = new remote::DBUS(&huart3);
  // Can
  can1 = new bsp::CAN(&hcan1, true);
  // RGB
  RGB = new display::RGB(&htim5, 3, 2, 1, 1000000);

  //Shooter initialization
  load_motor = new control::Motor3508(can1, 0x201);
  reload_motor = new control::Motor3508(can1, 0x202);
  force_motor = new control::Motor3508(can1, 0x203);
  // magic number from the data test for this servo
  trigger = new control::PDIHV(&htim1, 1, 1980000, 500, 1500);

  // Servo control for each shooter motor
  control::servo_t servo_data;
  servo_data.motor = load_motor;
  servo_data.max_speed = 4 * PI; // params need test
  servo_data.max_acceleration = 10 * PI;
  servo_data.transmission_ratio = M3508P19_RATIO;
  servo_data.omega_pid_param = new float [3] {1, 1, 1}; // pid need test
  servo_data.max_iout = 1000;
  servo_data.max_out = 13000;
  load_servo = new control::ServoMotor(servo_data);

  servo_data.motor = reload_motor;
  servo_data.max_speed = 4 * PI; // params need test
  servo_data.max_acceleration = 10 * PI;
  servo_data.omega_pid_param = new float [3] {1, 1, 1}; // pid need test
  reload_servo = new control::ServoMotor(servo_data);

  servo_data.motor = force_motor;
  servo_data.max_speed = 6 * PI; // params need test
  servo_data.max_acceleration = 50 * PI;
  servo_data.omega_pid_param = new float [3] {150, 1.2, 5}; // pid need test
  force_servo = new control::ServoMotor(servo_data);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  control::MotorCANBase* can2_reloader[] = { reload_motor};
//  control::MotorCANBase* can2_loader[] = { load_motor};
//  control::MotorCANBase* can2_force[] = { force_motor};

  // Variable initialization (params need test)
  // reload variables
//  bool reload_pull = false;
//  bool reload_push = false;
  float reload_pos = 10*PI;
  // load variable
//  bool loading = false;
//  float load_angle = 10*PI;
  int i = 0;

//   waiting for the start signal
  while (true) {
    if (dbus->keyboard.bit.B || dbus->swr == remote::DOWN) break;
    osDelay(100);
  }

  while (true) {
    // the shoot and load process is automatic
    // 1. using servo pull the trigger (shooting process)
    // 2. reload motor pull the bullet board to the desire position (load process below)
    // 3. using servo push the trigger the hold the bullet board
    // 4. load motor load one bullet on the board
    // 5. reload motor release to the original position to prepare for the next load
    // 6. optional: determine whether we need to change the force motor position
    // 7. repeat 1-6
    i = 0;
    // Load Detector
    LoadDetect.input(dbus->swr == remote::UP);
    if (LoadDetect.posEdge()) {
      // step 1
        trigger->SetOutPutAngle(20);  // need test the trigger angle
        osDelay(1000);                 // need test the delay time(wait for the)
       // step 2
      while (true) {
        // break condition (reach the desire position)
        if (++i > 10 / 2 && abs(reload_servo->GetOmega()) <= 0.0001) break;
        // set target pull position once
//        if (!reload_pull) {
//          reload_pull = true;
          reload_servo->SetTarget(reload_pos, true);
//        }
        reload_servo->CalcOutput();
        control::MotorCANBase::TransmitOutput(can2_reloader, 1);
        osDelay(2);
      }
      // after reload pulling
      i = 0;
//      reload_pull = false;
      osDelay(300); // need test the delay time(wait for the)
      // step 3
      trigger->SetOutPutAngle(-80); // need test the trigger angle
      osDelay(1000); // need test the delay time(wait for the)
      // step 4
//      while (true) {
//        // break condition (loading)
//        if (++i > 10 / 2 && abs(load_servo->GetOmega()) <= 0.0001) break;
//        // loading once
//        if (!loading) {
//          loading = true;
//          load_servo->SetTarget(load_servo->GetTarget() + load_angle, true);
//        }
//        load_servo->CalcOutput();
//        control::MotorCANBase::TransmitOutput(can2_loader, 1);
//        osDelay(2);
//      }
//      // after loading
//      loading = false;
//      osDelay(300); // need test the delay time(wait for the
      // step 5
//      while (true) {
//        // break condition (reach the desire position)
//        if (reload_servo->GetOmega() <= 0.0001) break;
//        // set target push position once
//        if (!reload_push) {
//          reload_push = true;
//          reload_servo->SetTarget(0, true);
//        }
//        reload_servo->CalcOutput();
//        control::MotorCANBase::TransmitOutput(motors_can2_shooter, 3);
//        osDelay(2);
//      }
//      // after reload pushing
//      reload_push = false;
//      osDelay(300); // need test the delay time(wait for the)
//      // step 6(TODO)
    }
    osDelay(10);
  }
}
