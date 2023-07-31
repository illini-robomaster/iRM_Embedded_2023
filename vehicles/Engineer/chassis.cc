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

#include "bsp_can_bridge.h"
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
#include "steering.h"
#include "supercap.h"
#include <cmath>

static bsp::CAN* can1 = nullptr;
static bsp::CAN* can2 = nullptr;
static display::RGB* RGB = nullptr;
static BoolEdgeDetector ReCali(false);
static BoolEdgeDetector Revival(false);

static remote::DBUS* dbus = nullptr;

//==================================================================================================
// SelfTest
//==================================================================================================

static bool fl_steer_motor_flag = false;
static bool fr_steer_motor_flag = false;
static bool bl_steer_motor_flag = false;
static bool br_steer_motor_flag = false;
static bool fl_wheel_motor_flag = false;
static bool fr_wheel_motor_flag = false;
static bool bl_wheel_motor_flag = false;
static bool br_wheel_motor_flag = false;

static bool transmission_flag = true;
const osThreadAttr_t selfTestingTask = {.name = "selfTestTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 256 * 4,
                                             .priority = (osPriority_t)osPriorityBelowNormal,
                                             .tz_module = 0,
                                             .reserved = 0};
osThreadId_t selfTestTaskHandle;


static BoolEdgeDetector FakeDeath(false);
static volatile bool Dead = false;

// static bsp::CanBridge* receive = nullptr;
static unsigned int flag_summary = 0;
static const int KILLALL_DELAY = 100;
static const int DEFAULT_TASK_DELAY = 100;
static const int CHASSIS_TASK_DELAY = 2;

// speed for steering motors (rad/s)
constexpr float RUN_SPEED = (4 * PI);
constexpr float ALIGN_SPEED = (PI);
constexpr float ACCELERATION = (100 * PI);


// speed for chassis rotation (no unit)
constexpr float FOLLOW_SPEED = 40;


//==================================================================================================
// Referee
//==================================================================================================

#define REFEREE_RX_SIGNAL (1 << 1)


const osThreadAttr_t refereeTaskAttribute = {.name = "refereeTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 1024 * 4,
                                             .priority = (osPriority_t)osPriorityAboveNormal,
                                             .tz_module = 0,
                                             .reserved = 0};

osThreadId_t refereeTaskHandle;
osThreadId_t chassisTaskHandle;
class RefereeUART : public bsp::UART {
 public:
  using bsp::UART::UART;

 protected:
  void RxCompleteCallback() final { osThreadFlagsSet(refereeTaskHandle, REFEREE_RX_SIGNAL); }
};

static communication::Referee* referee = nullptr;
static RefereeUART* referee_uart = nullptr;

void refereeTask(void* arg) {
  UNUSED(arg);
  uint32_t length;
  uint8_t* data;

  while (true) {
    uint32_t flags = osThreadFlagsWait(REFEREE_RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & REFEREE_RX_SIGNAL) {
      length = referee_uart->Read(&data);
      referee->Receive(communication::package_t{data, (int)length});
    }
  }
}

//==================================================================================================
// Chassis
//==================================================================================================

const osThreadAttr_t chassisTaskAttribute = {.name = "chassisTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 256 * 4,
                                             .priority = (osPriority_t)osPriorityNormal,
                                             .tz_module = 0,
                                             .reserved = 0};

static control::MotorCANBase* motor1 = nullptr;
static control::MotorCANBase* motor2 = nullptr;
static control::MotorCANBase* motor3 = nullptr;
static control::MotorCANBase* motor4 = nullptr;
static control::MotorCANBase* motor5 = nullptr;
static control::MotorCANBase* motor6 = nullptr;
static control::MotorCANBase* motor7 = nullptr;
static control::MotorCANBase* motor8 = nullptr;

static control::SteeringMotor* steering_motor1 = nullptr;
static control::SteeringMotor* steering_motor2 = nullptr;
static control::SteeringMotor* steering_motor3 = nullptr;
static control::SteeringMotor* steering_motor4 = nullptr;

static control::steering_chassis_t* chassis_data;
static control::SteeringChassis* chassis;


static const float CHASSIS_DEADZONE = 0.04;

void chassisTask(void* arg) {
  UNUSED(arg);

  control::MotorCANBase* steer_motors[] = {motor1, motor2, motor3, motor4};
  control::MotorCANBase* wheel_motors[] = {motor5, motor6, motor7, motor8};

  // while (!receive->start) osDelay(100);

  // while (receive->start < 0.5) osDelay(100);


  chassis->SteerSetMaxSpeed(RUN_SPEED);
  chassis->SteerThetaReset();
  chassis->SetWheelSpeed(0,0,0,0);

  while (true) {
    // float relative_angle = receive->relative_angle;
    float relative_angle = 0;
    float sin_yaw, cos_yaw, vx_set, vy_set;
    float vx, vy, wz;
    
    // vx_set = -receive->vy;
    // vy_set = receive->vx;
    vx_set = dbus->ch3;
    vy_set = dbus->ch2;


    chassis->SteerSetMaxSpeed(RUN_SPEED);
    sin_yaw = sin(relative_angle);
    cos_yaw = cos(relative_angle);
    vx = cos_yaw * vx_set + sin_yaw * vy_set;
    vy = -sin_yaw * vx_set + cos_yaw * vy_set;
    wz = std::min(FOLLOW_SPEED, FOLLOW_SPEED * relative_angle);       /* TODO : ASK IF GIMBAL EXIST, HOW CHASSIS MOVE */
    if (-CHASSIS_DEADZONE < relative_angle && relative_angle < CHASSIS_DEADZONE) wz = 0;
    
    chassis->SetSpeed(vx / 10, vy / 10, wz);
    chassis->SteerUpdateTarget();
    constexpr float WHEEL_SPEED_FACTOR = 4;
    chassis->WheelUpdateSpeed(WHEEL_SPEED_FACTOR);
    chassis->SteerCalcOutput();

   chassis->Update((float)referee->game_robot_status.chassis_power_limit,
                     referee->power_heat_data.chassis_power,
                     (float)referee->power_heat_data.chassis_power_buffer);

    if (Dead) {
      chassis->SetSpeed(0,0,0);
      motor5->SetOutput(0);
      motor6->SetOutput(0);
      motor7->SetOutput(0);
      motor8->SetOutput(0);
    }

    control::MotorCANBase::TransmitOutput(wheel_motors, 4);
    control::MotorCANBase::TransmitOutput(steer_motors, 4);

    // receive->cmd.id = bsp::SHOOTER_POWER;
    // receive->cmd.data_bool = referee->game_robot_status.mains_power_shooter_output;
    // receive->TransmitOutput();

    // receive->cmd.id = bsp::COOLING_HEAT1;
    // receive->cmd.data_float = (float)referee->power_heat_data.shooter_id1_17mm_cooling_heat;
    // receive->TransmitOutput();

    // receive->cmd.id = bsp::COOLING_HEAT2;
    // receive->cmd.data_float = (float)referee->power_heat_data.shooter_id2_17mm_cooling_heat;
    // receive->TransmitOutput();

    // receive->cmd.id = bsp::COOLING_LIMIT1;
    // receive->cmd.data_float = (float)referee->game_robot_status.shooter_id1_17mm_cooling_limit;
    // receive->TransmitOutput();

    // receive->cmd.id = bsp::COOLING_LIMIT2;
    // receive->cmd.data_float = (float)referee->game_robot_status.shooter_id2_17mm_cooling_limit;
    // receive->TransmitOutput();

    // receive->cmd.id = bsp::SPEED_LIMIT1;
    // receive->cmd.data_float = (float)referee->game_robot_status.shooter_id1_17mm_speed_limit;
    // receive->TransmitOutput();

    // receive->cmd.id = bsp::SPEED_LIMIT2;
    // receive->cmd.data_float = (float)referee->game_robot_status.shooter_id2_17mm_speed_limit;
    // receive->TransmitOutput();

    osDelay(CHASSIS_TASK_DELAY);

  }
}
void self_Check_Task(void* arg){
  UNUSED(arg);

  while(true){
    osDelay(100);
    motor8->connection_flag_ = false;
    motor7->connection_flag_ = false;
    motor6->connection_flag_ = false;
    motor5->connection_flag_ = false;
    motor4->connection_flag_ = false;
    motor3->connection_flag_ = false;
    motor2->connection_flag_ = false;
    motor1->connection_flag_ = false;
    osDelay(100);
    fl_wheel_motor_flag = motor8->connection_flag_;
    fr_wheel_motor_flag = motor7->connection_flag_;
    bl_wheel_motor_flag = motor6->connection_flag_;
    br_wheel_motor_flag = motor5->connection_flag_;
    fl_steer_motor_flag = motor4->connection_flag_;
    fr_steer_motor_flag = motor3->connection_flag_;
    br_steer_motor_flag = motor2->connection_flag_;
    bl_steer_motor_flag = motor1->connection_flag_;
    flag_summary = bl_steer_motor_flag|
                   br_steer_motor_flag<<1|
                   fr_steer_motor_flag<<2|
                   fl_steer_motor_flag<<3|
                   br_wheel_motor_flag<<4|
                   bl_wheel_motor_flag<<5|
                   fr_wheel_motor_flag<<6|
                   fl_wheel_motor_flag<<7;
    osDelay(100);
    // if(transmission_flag){
    //   receive->cmd.id = bsp::CHASSIS_FLAG;
    //   receive->cmd.data_uint = (unsigned int)flag_summary;
    //   receive->TransmitOutput();
    // }
    transmission_flag = !transmission_flag;
  }
}

void RM_RTOS_Init() {
  print_use_uart(&huart1);
  bsp::SetHighresClockTimer(&htim5);

  dbus = new remote::DBUS(&huart3);

  can1 = new bsp::CAN(&hcan1, true);
  can2 = new bsp::CAN(&hcan2, false);
  RGB = new display::RGB(&htim5, 3, 2, 1, 1000000);

  motor1 = new control::Motor6020(can1, 0x205);
  motor2 = new control::Motor6020(can1, 0x206);
  motor3 = new control::Motor6020(can1, 0x207);
  motor4 = new control::Motor6020(can1, 0x208);

  motor5 = new control::Motor3508(can2, 0x201);
  motor6 = new control::Motor3508(can2, 0x202);
  motor7 = new control::Motor3508(can2, 0x203);
  motor8 = new control::Motor3508(can2, 0x204);


  chassis_data = new control::steering_chassis_t();



  control::steering_t steering_motor_data;
  steering_motor_data.motor = motor1;
  steering_motor_data.max_speed = RUN_SPEED;
  steering_motor_data.max_acceleration = ACCELERATION;
  steering_motor_data.transmission_ratio = 8;
  steering_motor_data.omega_pid_param = new float[3]{200, 7, 1};
  steering_motor_data.max_iout = 1000;
  steering_motor_data.max_out = 13000;
  steering_motor_data.calibrate_offset = 0;

  /*ALIGNMENT IS NOT APPLICABLE IN THIS VEHICLE*/
  steering_motor_data.align_detect_func = nullptr;
  steering_motor1 = new control::SteeringMotor(steering_motor_data);
  steering_motor_data.motor = motor2;
  steering_motor_data.align_detect_func = nullptr;
  steering_motor2 = new control::SteeringMotor(steering_motor_data);
  steering_motor_data.motor = motor3;
  steering_motor_data.align_detect_func = nullptr;
  steering_motor3 = new control::SteeringMotor(steering_motor_data);
  steering_motor_data.motor = motor4;
  steering_motor_data.align_detect_func = nullptr;
  steering_motor4 = new control::SteeringMotor(steering_motor_data);

  chassis_data = new control::steering_chassis_t();

  chassis_data->fl_steer_motor = steering_motor4;
  chassis_data->fr_steer_motor = steering_motor3;
  chassis_data->bl_steer_motor = steering_motor1;
  chassis_data->br_steer_motor = steering_motor2;

  chassis_data->fl_wheel_motor = motor8;
  chassis_data->fr_wheel_motor = motor7;
  chassis_data->bl_wheel_motor = motor5;
  chassis_data->br_wheel_motor = motor6;

  chassis = new control::SteeringChassis(chassis_data);

  referee_uart = new RefereeUART(&huart6);
  referee_uart->SetupRx(300);
  referee_uart->SetupTx(300);
  referee = new communication::Referee;
  // receive = new bsp::CanBridge(can2, 0x20B, 0x20A);
}

void RM_RTOS_Threads_Init(void) {
  refereeTaskHandle = osThreadNew(refereeTask, nullptr, &refereeTaskAttribute);
  chassisTaskHandle = osThreadNew(chassisTask, nullptr, &chassisTaskAttribute);
  selfTestTaskHandle = osThreadNew(self_Check_Task, nullptr, &selfTestingTask);
}

void KillAll() {
  RM_EXPECT_TRUE(false, "Operation Killed!\r\n");

  control::MotorCANBase* wheel_motors[] = {motor5, motor6, motor7, motor8};

  RGB->Display(display::color_blue);

  chassis->SteerAlignFalse();   // set alignment status of each wheel to false

  while (true) {
    // if (!receive->dead) {
    //   Dead = false;
    //   RGB->Display(display::color_green);
    //   break;
    // }

    motor5->SetOutput(0);
    motor6->SetOutput(0);
    motor7->SetOutput(0);
    motor8->SetOutput(0);

    control::MotorCANBase::TransmitOutput(wheel_motors, 4);

    osDelay(KILLALL_DELAY);
  }
}

static bool debug = false;

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  while (true) {
    // if (receive->dead) {
    //   Dead = true;
    //   KillAll();
    // }

    // receive->cmd.id = bsp::GIMBAL_POWER;
    // receive->cmd.data_uint = referee->game_robot_status.mains_power_gimbal_output;
    // receive->TransmitOutput();

    // receive->cmd.id = bsp::IS_MY_COLOR_BLUE;
    // receive->cmd.data_bool = (referee->game_robot_status.robot_id >= 100) ? true : false;
    // receive->TransmitOutput();

    print("type: %d\r\n", referee->robot_hurt.hurt_type);

    if (debug) {
      set_cursor(0, 0);
      clear_screen();
    //   print("vx: %f, vy: %f, angle: %f, mode: %f, dead: %f\r\n", receive->vx, receive->vy,
    //         receive->relative_angle, receive->mode, receive->dead);
    // }
    osDelay(DEFAULT_TASK_DELAY);
  }
}
