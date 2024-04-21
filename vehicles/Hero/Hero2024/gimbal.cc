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
#include "bsp_os.h"
#include "bsp_print.h"
#include "bsp_relay.h"
#include "bsp_can_bridge.h"
#include "cmsis_os.h"
#include "controller.h"
#include "dbus.h"
#include "main.h"
#include "motor.h"
#include "protocol.h"
#include "rgb.h"
#include "oled.h"
#include "bsp_buzzer.h"



static remote::DBUS* dbus = nullptr;
static bsp::CAN* can1 = nullptr;
static bsp::CAN* can2 = nullptr;
static bsp::CanBridge* send = nullptr;

// lob mode switch
BoolEdgeDetector lob_mode_sw = BoolEdgeDetector(false);
volatile bool lob_mode = false;
//display::RGB* RGB = nullptr;

//==================================================================================================
// Shooter
//==================================================================================================


const osThreadAttr_t shooterTaskAttribute = {.name = "shooterTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = 256 * 4,
        .priority = (osPriority_t)osPriorityNormal,
        .tz_module = 0,
        .reserved = 0};

osThreadId_t shooterTaskHandle;

static const int KILLALL_DELAY = 100;
static const int SHOOTER_TASK_DELAY = 1;

static control::Motor4310* load_motor = nullptr;

static control::MotorCANBase* shoot_front_motor = nullptr;
static control::MotorCANBase* shoot_back_motor = nullptr;

static control::MotorCANBase* force_motor = nullptr;
static control::ServoMotor* force_servo = nullptr;

void shooterTask(void* args) {
  UNUSED(args);
  // shooter desired speed
  float shoot_speed = -400;
  // motor initialization
  control::MotorCANBase* can1_shooter_shoot[] = {shoot_front_motor, shoot_back_motor};
//  control::MotorCANBase* can1_shooter_force[] = {force_motor};
//  control::Motor4310* can1_shooter_load[] = {load_motor};

  // PID controller initialization
  float shoot_pid_params[3] = {20, 15, 10};
  control::ConstrainedPID shoot_pid(shoot_pid_params, 5000, 10000);
  float shoot_front_speed_diff = 0;
  float shoot_back_speed_diff = 0;
  int16_t shoot_front_out = 0;
  int16_t shoot_back_out = 0;

  while (true) {
    if (dbus->keyboard.bit.B || dbus->swr == remote::DOWN) break;
//    print("%f\r\n",escalation_servo->GetTheta());
    osDelay(100);
  }


  print("shooterTask entering loop\r\n");
  while (true) {
    if (dbus->swr == remote::UP) {
      shoot_front_speed_diff = shoot_front_motor->GetOmegaDelta(shoot_speed);
      shoot_back_speed_diff = shoot_back_motor->GetOmegaDelta(shoot_speed);
      shoot_front_out = shoot_pid.ComputeConstrainedOutput(shoot_front_speed_diff);
      shoot_back_out = shoot_pid.ComputeConstrainedOutput(shoot_back_speed_diff);
//      shoot_front_motor->SetOutput(-2300);
//      shoot_back_motor->SetOutput(-2300);
      shoot_back_motor->SetOutput(shoot_back_out);
      shoot_front_motor->SetOutput(shoot_front_out);
    } else {
      shoot_front_speed_diff = 0;
      shoot_back_speed_diff = 0;
      shoot_front_out = 0;
      shoot_back_out = 0;
      shoot_front_motor->SetOutput(0);
      shoot_back_motor->SetOutput(0);
    }
//    print("shoot_front_out: %d, shoot_back_out: %d\r\n", shoot_front_out, shoot_back_out);

    control::MotorCANBase::TransmitOutput(can1_shooter_shoot, 2);
    osDelay(SHOOTER_TASK_DELAY);
  }
}

//==================================================================================================
// Gimbal
//==================================================================================================

// Params Initialization
const osThreadAttr_t gimbalTaskAttribute = {.name = "gimbalTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = 512 * 4,
        .priority = (osPriority_t)osPriorityNormal,
        .tz_module = 0,
        .reserved = 0};

osThreadId_t gimbalTaskHandle;

static const int GIMBAL_TASK_DELAY = 1;

static control::MotorCANBase* esca_motor = nullptr;
static control::ServoMotor* escalation_servo = nullptr;

bool exit_flag = false;

float calibrated_theta = 0;


void jam_callback(control::ServoMotor* servo, const control::servo_jam_t data) {
  UNUSED(data);
  control::MotorCANBase* can1_escalation[] = {esca_motor};
  esca_motor->SetOutput(0);
  control::MotorCANBase::TransmitOutput(can1_escalation, 1);
  osDelay(100);
  servo->ResetTheta();
  print("Jam detected, resetting...\r\n");
  exit_flag = true;
}

void gimbalTask(void* args) {
  UNUSED(args);
  control::MotorCANBase* can1_escalation[] = {esca_motor};
  while (true) {
    if (dbus->keyboard.bit.B || dbus->swr == remote::DOWN) break;
//    print("%f\r\n",escalation_servo->GetTheta());
    osDelay(100);
  }
//
  while (true){
    escalation_servo->SetTarget(escalation_servo->GetTheta() - PI / 2,true);
    escalation_servo->CalcOutput();
    control::MotorCANBase::TransmitOutput(can1_escalation, 1);
    osDelay(10);
//      print("Omega: %f\r\n", escalation_servo->GetOmega());
    if (exit_flag){
      print("Escalation motor jammed, reseting...\r\n");
      break;
    }
  }
//  calibrated_theta = escalation_servo->GetTheta();
//  print("gimbalTask entering loop\r\n");
//  print("calibrated_theta: %f\r\n", calibrated_theta);
//  escalation_servo->RegisterJamCallback(jam_callback, 0.405);
//  escalation_servo->SetTarget(calibrated_theta,true);
  // update jam callback threshold
  while (true) {
    // Bool Edge Detector for lob mode switch or osEventFlags wait for a signal from different threads
//    lob_mode_sw.input(dbus->keyboard.bit.SHIFT || dbus->swl == remote::UP);
//    if (lob_mode_sw.posEdge()){
//      lob_mode = !lob_mode;
//      // TODO: after implementing chassis, uncomment the lob_mode bsp::CanBridge flag transmission
////      send->cmd.id = bsp::LOB_MODE;
////      send->cmd.data_bool = lob_mode;
////      send->TransmitOutput();
//      escalation_servo->SetMaxSpeed(4 * PI);
//      print("lob_mode: %d\n", lob_mode);
//    }
//    if(lob_mode){
//      // please make sure the calibration is all done
//      // lob mode
//      escalation_servo->SetTarget(calibrated_theta + PI * 12.1);
//      // maximum goes to 38.345 radians
//    } else {
////         moving mode
////         set escalation servo to original position
//      escalation_servo->SetTarget(calibrated_theta);
//    }
//    print("current theta: %f\r\n", escalation_servo->GetTheta());
//    escalation_servo->CalcOutput();
//    control::MotorCANBase::TransmitOutput(can1_escalation, 1);
//    osDelay(GIMBAL_TASK_DELAY);
  }
}



void RM_RTOS_Init() {
  print_use_uart(&huart1);
  // Initialize the CAN bus
  can1 = new bsp::CAN(&hcan1, true);
  can2 = new bsp::CAN(&hcan2, false);
  // Initialize the DBUS
  dbus = new remote::DBUS(&huart3);
  bsp::SetHighresClockTimer(&htim5);
  send = new bsp::CanBridge(can2, 0x20A, 0x20B);
  // Initialize the RGB LED
//  RGB=new display::RGB(&htim5,3,2,1,1000000);
  //Shooter initialization
  load_motor = new control::Motor4310(can1, 0x02, 0x01, control::POS_VEL);
  shoot_front_motor = new control::Motor3508(can1, 0x201);
  shoot_back_motor = new control::Motor3508(can1, 0x202);
  force_motor = new control::Motor3508(can1, 0x203);
  // Servo control for each shooter motor
  control::servo_t servo_data;
  servo_data.motor = force_motor;
  servo_data.max_speed = 100 * PI; // TODO: params need test
  servo_data.max_acceleration = 100 * PI;
  servo_data.transmission_ratio = 1;
  servo_data.omega_pid_param = new float [3] {150, 1.2, 5};
  servo_data.max_iout = 1000;
  servo_data.max_out = 13000;
  force_servo = new control::ServoMotor(servo_data);

  // Ecalation initialization
  esca_motor = new control::Motor3508(can1, 0x204);
  // ESCALATION motors initialization
  control::servo_t esca_servo_data;
  esca_servo_data.motor = esca_motor;
  esca_servo_data.max_speed = 2 * PI; // TODO: params need test
  esca_servo_data.max_acceleration = 30 * PI;
  esca_servo_data.transmission_ratio = M3508P19_RATIO;
  esca_servo_data.omega_pid_param = new float [3] {150, 1.2, 5}; // TODO: PID params might need tuning
  esca_servo_data.max_iout = 1000;
  esca_servo_data.max_out = 13000;
  escalation_servo = new control::ServoMotor(esca_servo_data);
  escalation_servo->RegisterJamCallback(jam_callback, 0.205);
}

void RM_RTOS_Threads_Init(){
  shooterTaskHandle = osThreadNew(shooterTask, nullptr,&shooterTaskAttribute);
  gimbalTaskHandle = osThreadNew(gimbalTask, nullptr,&gimbalTaskAttribute);
}

void KillAll(){
  RM_EXPECT_TRUE(false, "Operation Killed!\r\n");
  control::MotorCANBase* shooter_motors[] = {shoot_front_motor, shoot_back_motor, force_motor, esca_motor};
  control::Motor4310* load_motors[] = {load_motor};
  while (true){
    shoot_front_motor->SetOutput(0);
    shoot_back_motor->SetOutput(0);
    force_motor->SetOutput(0);
    load_motor->SetOutput(0);
    esca_motor->SetOutput(0);
    control::MotorCANBase::TransmitOutput(shooter_motors, 4);
    control::Motor4310::TransmitOutput(load_motors, 1);
    osDelay(KILLALL_DELAY);
  }
}