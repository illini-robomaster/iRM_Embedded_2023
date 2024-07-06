/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2024 RoboMaster.                                          *
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

#include <cmath>

#include "bsp_can_bridge.h"
#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "dbus.h"
#include "fortress.h"
#include "main.h"
#include "motor.h"
#include "protocol.h"
#include "rgb.h"
#include "supercap.h"
#include "user_interface.h"

static bsp::CAN* can1 = nullptr;
static bsp::CAN* can2 = nullptr;
static display::RGB* RGB = nullptr;

static BoolEdgeDetector FakeDeath(false);
static volatile bool Dead = false;
static BoolEdgeDetector ChangeSpinMode(false);
static volatile bool SpinMode = false;

static bsp::CanBridge* receive = nullptr;
static unsigned int flag_summary = 0;
static const int KILLALL_DELAY = 100;
static const int DEFAULT_TASK_DELAY = 100;
static const int CHASSIS_TASK_DELAY = 2;

// TODO: Mecanum wheel need different speed???
// speed for steering motors (rad/s)
constexpr float RUN_SPEED = (4 * PI);
// TODO：
// constexpr float ALIGN_SPEED = (PI);
constexpr float ACCELERATION = (100 * PI);

// speed for chassis rotation (no unit)
// TODO: the speed for the fortress chassis
constexpr float SPIN_SPEED = 500;
constexpr float FOLLOW_SPEED = 360;
// constexpr float SPIN_SPEED = 80;
// constexpr float FOLLOW_SPEED = 40;

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
// Chassis(TODO)
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
osThreadId_t chassisTaskHandle;

static control::MotorCANBase* fl_motor = nullptr;
static control::MotorCANBase* fr_motor = nullptr;
static control::MotorCANBase* bl_motor = nullptr;
static control::MotorCANBase* br_motor = nullptr;
static control::Chassis* chassis = nullptr;

 static control::SuperCap* supercap = nullptr;
 static float SPIN_DOWN_SPEED_FACTOR = 0.0;

static const float CHASSIS_DEADZONE = 0.04;

void chassisTask(void* arg) {
  UNUSED(arg);

  control::MotorCANBase* motors[] = {fl_motor, fr_motor, bl_motor, br_motor};

  while (!receive->start) osDelay(100);

  float WHEEL_SPEED_FACTOR = 0.0;
  float power_limit = 0.0;
  float power_limit_lowerbound = 45.0;

  while (true) {
    float relative_angle = receive->relative_angle;
    float sin_yaw, cos_yaw, vx_set, vy_set;
    float vx, vy, wz;

    vx_set = -receive->vx;
    vy_set = -receive->vy;

    float supercap_voltage = (float)(supercap->info.voltage / 1000.0);
    float maximum_energy = 0.5 * pow(27.0,2) * 6.0;

    float current_energy = pow(supercap_voltage,2) * 6 / 2;

    //consider using uart printing to check the power limit's value
    //log values out as files to obtain its trend
    //WHEEL_SPEED_FACTOR separated for two modes
    if (current_energy <= 0.1 * maximum_energy) {
      // case when remaining energy of capacitor is below 10%, recharge the super capacitor
      power_limit = power_limit_lowerbound;
      WHEEL_SPEED_FACTOR = 4.0;
    } else if (current_energy <= 0.2 * maximum_energy && current_energy > 0.1 * maximum_energy) {
      // case when remaining energy of capacitor is between 10% and 20%
      power_limit = power_limit_lowerbound + (power_limit_lowerbound / (0.1 * maximum_energy)) * (current_energy - 0.1 * maximum_energy);
      WHEEL_SPEED_FACTOR = 6.0 + (4.0 / ((0.1 * maximum_energy))) * (current_energy - 0.1 * maximum_energy);
    } else if (current_energy <= 0.3 * maximum_energy && current_energy > 0.2 * maximum_energy) {
      // case when remaining energy of capacitor is between 20% and 30%
      power_limit = 80.0 + (60.0 / (0.1 * maximum_energy)) * (current_energy - 0.2 * maximum_energy);
      WHEEL_SPEED_FACTOR = 10.0 + (2.0 / ((0.1 * maximum_energy))) * (current_energy - 0.2 * maximum_energy);
    } else if (current_energy > 0.3 * maximum_energy) {
      // case when remaining energy of capacitor is above 30%
      power_limit = 120;
      WHEEL_SPEED_FACTOR = 12.0;
      // WHEEL SPEED FACTOR will be 12
    }

    SPIN_DOWN_SPEED_FACTOR = 4.0 / WHEEL_SPEED_FACTOR;
    WHEEL_SPEED_FACTOR = receive->mode== 1 ? 4.0 : WHEEL_SPEED_FACTOR;


    if (receive->mode == 1) {  // spin mode
      sin_yaw = arm_sin_f32(relative_angle);
      cos_yaw = arm_cos_f32(relative_angle);
      vx = cos_yaw * vx_set + sin_yaw * vy_set;
      vy = -sin_yaw * vx_set + cos_yaw * vy_set;
      wz = SPIN_SPEED;
    } else {
      sin_yaw = arm_sin_f32(relative_angle);
      cos_yaw = arm_cos_f32(relative_angle);
      vx = cos_yaw * vx_set + sin_yaw * vy_set;
      vy = -sin_yaw * vx_set + cos_yaw * vy_set;
      wz = std::min(FOLLOW_SPEED, FOLLOW_SPEED * relative_angle);
      if (-CHASSIS_DEADZONE < relative_angle && relative_angle < CHASSIS_DEADZONE) wz = 0;
    }

    chassis->SetSpeed(vx, vy, wz);
    chassis->Update(false, power_limit,
                    referee->power_heat_data.chassis_power,
                    (float)referee->power_heat_data.chassis_power_buffer);

    // TODO: for fortress mode and dead
    if (Dead || receive->fortress_mode) {
      fl_motor->SetOutput(0);
      fr_motor->SetOutput(0);
      bl_motor->SetOutput(0);
      br_motor->SetOutput(0);
    }

    control::MotorCANBase::TransmitOutput(motors, 4);

    receive->cmd.id = bsp::SHOOTER_POWER;
    receive->cmd.data_bool = referee->game_robot_status.mains_power_shooter_output;
    receive->TransmitOutput();

    receive->cmd.id = bsp::COOLING_HEAT1;
    receive->cmd.data_float = (float)referee->power_heat_data.shooter_id1_17mm_cooling_heat;
    receive->TransmitOutput();

    receive->cmd.id = bsp::COOLING_HEAT2;
    receive->cmd.data_float = (float)referee->power_heat_data.shooter_id2_17mm_cooling_heat;
    receive->TransmitOutput();

    receive->cmd.id = bsp::COOLING_LIMIT1;
    receive->cmd.data_float = (float)referee->game_robot_status.shooter_id1_17mm_cooling_limit;
    receive->TransmitOutput();

    receive->cmd.id = bsp::COOLING_LIMIT2;
    receive->cmd.data_float = (float)referee->game_robot_status.shooter_id2_17mm_cooling_limit;
    receive->TransmitOutput();

    receive->cmd.id = bsp::SPEED_LIMIT1;
    receive->cmd.data_float = (float)referee->game_robot_status.shooter_id1_17mm_speed_limit;
    receive->TransmitOutput();

    receive->cmd.id = bsp::SPEED_LIMIT2;
    receive->cmd.data_float = (float)referee->game_robot_status.shooter_id2_17mm_speed_limit;
    receive->TransmitOutput();

    osDelay(CHASSIS_TASK_DELAY);
  }
}


//==================================================================================================
// SelfTest
//==================================================================================================

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

static bool fl_motor_flag = false;
static bool fr_motor_flag = false;
static bool bl_motor_flag = false;
static bool br_motor_flag = false;
static bool elevator_left_motor_flag = false;
static bool elevator_right_motor_flag = false;
static bool fortress_motor_flag = false;

static bool transmission_flag = true;

void self_Check_Task(void* arg) {
  UNUSED(arg);

  while (true) {
    osDelay(100);
    // TODO：fortress self check(7 motor)
    fl_motor->connection_flag_ = false;
    fr_motor->connection_flag_ = false;
    bl_motor->connection_flag_ = false;
    br_motor->connection_flag_ = false;

    osDelay(100);

    fl_motor_flag = fl_motor->connection_flag_;
    fr_motor_flag = fr_motor->connection_flag_;
    bl_motor_flag = bl_motor->connection_flag_;
    br_motor_flag = br_motor->connection_flag_;

    flag_summary = fl_motor_flag |
                   fr_motor_flag << 1 |
                   bl_motor_flag << 2 |
                   br_motor_flag << 3 |
                   elevator_left_motor_flag << 4 |
                   elevator_right_motor_flag << 5 |
                   fortress_motor_flag << 6;

    osDelay(100);
    if (transmission_flag) {
      receive->cmd.id = bsp::CHASSIS_FLAG;
      receive->cmd.data_uint = (unsigned int)flag_summary;
      receive->TransmitOutput();
      receive->cmd.id = bsp :: SUPERCAP_VOLTAGE;
      receive->cmd.data_float = supercap->info.voltage / 1000;
      receive->TransmitOutput();
    }
    transmission_flag = !transmission_flag;
  }
}

//==================================================================================================
// RM Init(TODO)
//==================================================================================================

void RM_RTOS_Init() {
  print_use_uart(&huart1);
  bsp::SetHighresClockTimer(&htim5);

  can1 = new bsp::CAN(&hcan1, true);
  can2 = new bsp::CAN(&hcan2, false);
  RGB = new display::RGB(&htim5, 3, 2, 1, 1000000);

  fl_motor = new control::Motor3508(can1, 0x201);
  fr_motor = new control::Motor3508(can1, 0x202);
  bl_motor = new control::Motor3508(can1, 0x203);
  br_motor = new control::Motor3508(can1, 0x204);

  control::MotorCANBase* motors[control::FourWheel::motor_num];
  motors[control::FourWheel::front_left] = fl_motor;
  motors[control::FourWheel::front_right] = fr_motor;
  motors[control::FourWheel::back_left] = bl_motor;
  motors[control::FourWheel::back_right] = br_motor;

  control::chassis_t chassis_data;
  chassis_data.motors = motors;
  chassis_data.model = control::CHASSIS_MECANUM_WHEEL;
  chassis = new control::Chassis(chassis_data);

  referee_uart = new RefereeUART(&huart6);
  referee_uart->SetupRx(300);
  referee_uart->SetupTx(300);
  referee = new communication::Referee;

  receive = new bsp::CanBridge(can2, 0x20B, 0x20A);

  supercap = new control::SuperCap(can1, 0x301);
}

//==================================================================================================
// RTOS Threads Init(TODO)
//==================================================================================================

void RM_RTOS_Threads_Init(void) {
  // TODO: need to add the fortress thread(do we need UI?)
  refereeTaskHandle = osThreadNew(refereeTask, nullptr, &refereeTaskAttribute);
  chassisTaskHandle = osThreadNew(chassisTask, nullptr, &chassisTaskAttribute);
  selfTestTaskHandle = osThreadNew(self_Check_Task, nullptr, &selfTestingTask);
}

//==================================================================================================
// Kill All (TODO:)
//==================================================================================================

void KillAll() {
  RM_EXPECT_TRUE(false, "Operation Killed!\r\n");

  control::MotorCANBase* motors_can1_chassis[] = {fl_motor, fr_motor, bl_motor, br_motor};
  // TODO
  //  control::MotorCANBase* motors_can2_elevator[] = {elevator_left_motor, elevator_right_motor};
  //  control::MotorCANBase* motors_can2_fortress[] = {fortress_motor};

  RGB->Display(display::color_blue);

  // TODO: do we need fake death here(I think no need, fortress dead?)
  while (true) {
    if (!receive->dead) {
      SpinMode = false;
      Dead = false;
      RGB->Display(display::color_green);
      break;
    }
    fl_motor->SetOutput(0);
    bl_motor->SetOutput(0);
    fr_motor->SetOutput(0);
    br_motor->SetOutput(0);
    control::MotorCANBase::TransmitOutput(motors_can1_chassis, 4);

    osDelay(KILLALL_DELAY);
  }
}

//==================================================================================================
// RTOS Default Task (TODO???)
//==================================================================================================

static bool debug = false;

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  while (true) {
    if (receive->dead) {
      Dead = true;
      KillAll();
    }

    receive->cmd.id = bsp::GIMBAL_POWER;
    receive->cmd.data_uint = referee->game_robot_status.mains_power_gimbal_output;
    receive->TransmitOutput();
    //    print("out: %d\r\n", referee->game_robot_status.mains_power_gimbal_output);
    //    print("test");
    if (debug) {
      set_cursor(0, 0);
      clear_screen();
      //      print("vx: %f, vy: %f, angle: %f, mode: %f, dead: %f, start: %f\r\n", receive->vx, receive->vy,
      //            receive->relative_angle, receive->mode, receive->dead, receive->start);
      print("power limit: %.3f chassis power: %.3f power buffer: %.3f\r\n", (float)referee->game_robot_status.chassis_power_limit,
            referee->power_heat_data.chassis_power,
            (float)referee->power_heat_data.chassis_power_buffer);
    }
    osDelay(DEFAULT_TASK_DELAY);
  }
}
