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

#define KEY_GPIO_GROUP GPIOA
#define KEY_GPIO_PIN GPIO_PIN_0

bsp::CAN* can1 = NULL;
bsp::GPIO* key = nullptr;
// control::MotorCANBase* motor1 = NULL;
control::MotorCANBase* motor = NULL;

static control::ConstrainedPID* rfid_motor_theta_pid = nullptr;
static control::ConstrainedPID* rfid_motor_omega_pid = nullptr;
static BoolEdgeDetector rfid_upper(false);
static BoolEdgeDetector rfid_lower(false);

void RM_RTOS_Init() {
  print_use_uart(&huart1);

  can1 = new bsp::CAN(&hcan1, true);
  // motor1 = new control::Motor6020(can1, 0x205);
  motor = new control::Motor6020(can1, 0x207);
  key = new bsp::GPIO(KEY_GPIO_GROUP, KEY_GPIO_PIN);

  float rfid_motor_pid_param_theta[] = {100.0, 0.0, 0.6};
  float rfid_motor_max_iout_theta = 0;
  float rfid_motor_max_out_theta = 10;
  rfid_motor_theta_pid = new control::ConstrainedPID(rfid_motor_pid_param_theta, rfid_motor_max_iout_theta, rfid_motor_max_out_theta);

  float rfid_motor_pid_param_omega[] = {550.0, 10.0, 0.0};
  float rfid_motor_max_iout_omega = 25000;
  float rfid_motor_max_out_omega = 30000;
  rfid_motor_omega_pid = new control::ConstrainedPID(rfid_motor_pid_param_omega, rfid_motor_max_iout_omega, rfid_motor_max_out_omega);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  control::MotorCANBase* motors[] = {motor};

  while(!key->Read());

  while(key->Read());

  print("ok!\r\n");

  float target1 = 5.55;
  float target2 = 0.75;
  UNUSED(target1);
  UNUSED(target2);
  float cmd_vel = 1.0;
  // int count = 0;
  bool cw = true;

  while (true) {
    print("theta: %f\r\n", motor->GetTheta());
    rfid_upper.input(motor->GetTheta() > target2);
    rfid_lower.input(motor->GetTheta() < target1);
    if (rfid_upper.posEdge()) {
      cw = false;
    }
    if (rfid_lower.posEdge()) {
      cw = true;
    }

    if (cw) {
      float omega_error = motor->GetOmegaDelta(cmd_vel);
      float omega_pid_out = rfid_motor_omega_pid->ComputeConstrainedOutput(omega_error);
      motor->SetOutput(omega_pid_out);
    } else {
      float omega_error = motor->GetOmegaDelta(-cmd_vel);
      float omega_pid_out = rfid_motor_omega_pid->ComputeConstrainedOutput(omega_error);
      motor->SetOutput(omega_pid_out);
    }

    // if (count < 700) {
    //   float omega_error = motor->GetOmegaDelta(cmd_vel);
    //   print("omega_error1: %.3f\r\n", omega_error);
    //   float omega_pid_out = rfid_motor_omega_pid->ComputeConstrainedOutput(omega_error);
    //   motor->SetOutput(omega_pid_out);
    //   count++;
    // } else if (count >= 700 && count < 1400) {
    //   float omega_error = motor->GetOmegaDelta(-cmd_vel);
    //   print("omega_error2: %.3f\r\n", omega_error);
    //   float omega_pid_out = rfid_motor_omega_pid->ComputeConstrainedOutput(omega_error);
    //   motor->SetOutput(omega_pid_out);
    //   count++;
    // } else {
    //   count = 0;
    // }

    UNUSED(motors);
    control::MotorCANBase::TransmitOutput(motors, 1);

    // while (motor->GetTheta() < target2) {
    //   float omega_error = motor->GetOmegaDelta(cmd_vel);
    //   float omega_pid_out = rfid_motor_omega_pid->ComputeConstrainedOutput(omega_error);
    //   motor->SetOutput(omega_pid_out);
    //   control::MotorCANBase::TransmitOutput(motors, 1);
    //   osDelay(2);
    // }

    // while (motor->GetTheta() > target1) {
    //   float omega_error = motor->GetOmegaDelta(-cmd_vel);
    //   float omega_pid_out = rfid_motor_omega_pid->ComputeConstrainedOutput(omega_error);
    //   motor->SetOutput(omega_pid_out);
    //   control::MotorCANBase::TransmitOutput(motors, 1);
    //   osDelay(2);
    // } 



/*     float curr_target = motor->GetTheta();
    for (int i = 0; i < 500; i++) {
      curr_target -= (motor->GetTheta() - target1) / 500.0;
      print("curr target: %.4f\r\n", curr_target);
      float theta_error = motor->GetThetaDelta(target1);
      float theta_pid_out = rfid_motor_theta_pid->ComputeOutput(theta_error);
      float omega_error = motor->GetOmegaDelta(theta_pid_out);
      float omega_pid_out = rfid_motor_omega_pid->ComputeConstrainedOutput(omega_error);
      motor->SetOutput(omega_pid_out);
      control::MotorCANBase::TransmitOutput(motors, 1);
      osDelay(5);
    }

    osDelay(500);

    curr_target = motor->GetTheta();
    for (int i = 0; i < 500; i++) {
      curr_target -= (motor->GetTheta() - target2) / 500.0;
      print("curr target: %.4f\r\n", curr_target);
      float theta_error = motor->GetThetaDelta(target2);
      float theta_pid_out = rfid_motor_theta_pid->ComputeOutput(theta_error);
      float omega_error = motor->GetOmegaDelta(theta_pid_out);
      float omega_pid_out = rfid_motor_omega_pid->ComputeConstrainedOutput(omega_error);
      motor->SetOutput(omega_pid_out);
      control::MotorCANBase::TransmitOutput(motors, 1);
      osDelay(5);
    }

    osDelay(500); */



    // while (abs(target1 - motor->GetTheta()) > 0.0005) {
    //   print("%10.4f\r\n", abs(target1 - motor->GetTheta()));
    //   float theta_error = motor->GetThetaDelta(target1);
    //   float theta_pid_out = rfid_motor_theta_pid->ComputeOutput(theta_error);
    //   float omega_error = motor->GetOmegaDelta(theta_pid_out);
    //   float omega_pid_out = rfid_motor_omega_pid->ComputeConstrainedOutput(omega_error);
    //   motor->SetOutput(omega_pid_out);
    //   control::MotorCANBase::TransmitOutput(motors, 1);
    //   osDelay(2);
    // }
    
    // osDelay(500);

    // while (abs(target2 - motor->GetTheta()) > 0.0005) {
    //   print("%10.4f\r\n", abs(target2 - motor->GetTheta()));
    //   float theta_error = motor->GetThetaDelta(target2);
    //   float theta_pid_out = rfid_motor_theta_pid->ComputeOutput(theta_error);
    //   float omega_error = motor->GetOmegaDelta(theta_pid_out);
    //   float omega_pid_out = rfid_motor_omega_pid->ComputeConstrainedOutput(omega_error);
    //   motor->SetOutput(omega_pid_out);
    //   control::MotorCANBase::TransmitOutput(motors, 1);
    //   osDelay(1);
    // }

    // osDelay(500);
    

    // theta_error = motor->GetThetaDelta(0);
    // theta_pid_out = rfid_motor_theta_pid->ComputeOutput(theta_error);
    // omega_error = motor->GetOmegaDelta(theta_pid_out);
    // omega_pid_out = rfid_motor_omega_pid->ComputeConstrainedOutput(omega_error);

    // motor->SetOutput(omega_pid_out);

    // if (key->Read()) {
    //   motor2->SetOutput(0);
    // } else {
    //   motor2->SetOutput(800);
    //   print("%10.4f\r\n", motor2->GetTheta());
    // }
    // control::MotorCANBase::TransmitOutput(motors, 1);
    osDelay(2);
  }
}
