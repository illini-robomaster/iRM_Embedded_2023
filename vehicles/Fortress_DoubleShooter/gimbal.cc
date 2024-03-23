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

#include "gimbal.h"

#include <cstdio>

#include "bsp_buzzer.h"
#include "bsp_can.h"
#include "bsp_can_bridge.h"
#include "bsp_imu.h"
#include "bsp_laser.h"
#include "bsp_os.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "i2c.h"
#include "main.h"
#include "motor.h"
#include "oled.h"
#include "protocol.h"
#include "rgb.h"
#include "shooter.h"
#include "stepper.h"

static bsp::CAN* can1 = nullptr;
static bsp::CAN* can2 = nullptr;
static remote::DBUS* dbus = nullptr;
static display::RGB* RGB = nullptr;

static const int GIMBAL_TASK_DELAY = 5;
static const int CHASSIS_TASK_DELAY = 2;
static const int SHOOTER_TASK_DELAY = 10;
static const int SELFTEST_TASK_DELAY = 100;
static const int KILLALL_DELAY = 100;
static const int DEFAULT_TASK_DELAY = 100;
static const int SOFT_START_CONSTANT = 500;
static const int SOFT_KILL_CONSTANT = 200;
static const float START_PITCH_POS = 0.48f;
// TODO: the start position of the yaw motor
static const float START_YAW_POS = 0;
static const int INFANTRY_INITIAL_HP = 100;
static const int GIMBAL_START_DELAY = 1000;

static bsp::CanBridge* send = nullptr;

static BoolEdgeDetector FakeDeath(false);
static BoolEdgeDetector GimbalDeath(true);
static volatile bool Dead = false;
static volatile bool GimbalDead = false;
static BoolEdgeDetector ChangeSpinMode(false);
static volatile bool SpinMode = false;
// TODO: Fortress mode
static BoolEdgeDetector ChangeFortressMode(false);
static volatile bool FortressMode = false;

static BoolEdgeDetector Antijam(false);

static volatile float relative_angle = 0;

static unsigned int chassis_flag_bitmap = 0;

static volatile float pitch_pos = START_PITCH_POS;
//static volatile float yaw_pos = 0;

static volatile unsigned int gimbal_alive = 0;
// TODO: need to measure the specific angle
// static volatile float yaw_pos = START_YAW_POS;

// do we need to move this below? just self test use.
static volatile bool pitch_motor_flag = false;
static volatile bool yaw_motor_flag = false;
static volatile bool left_top_flywheel_flag = false;
static volatile bool left_bottom_flywheel_flag = false;
static volatile bool left_dial_flag = false;
static volatile bool right_top_flywheel_flag = false;
static volatile bool right_bottom_flywheel_flag = false;
static volatile bool right_dial_flag = false;
static volatile bool fl_motor_flag = false;
static volatile bool fr_motor_flag = false;
static volatile bool bl_motor_flag = false;
static volatile bool br_motor_flag = false;
static volatile bool elevator_left_motor_flag = false;
static volatile bool elevator_right_motor_flag = false;
static volatile bool fortress_motor_flag = false;
static volatile bool calibration_flag = false;
// static volatile bool referee_flag = false;
static volatile bool dbus_flag = false;
static volatile bool lidar_flag = false;
static volatile bool pitch_reset = false;
// TODO:
// static volatile bool yaw_reset = false;

static volatile bool selftestStart = false;

static volatile bool robot_hp_begin = false;

#define GEAR_GIMBAL   // comment this line if the gimbal is using belt

//==================================================================================================
// IMU
//==================================================================================================

#define IMU_RX_SIGNAL (1 << 0)

const osThreadAttr_t imuTaskAttribute = {.name = "imuTask",
                                         .attr_bits = osThreadDetached,
                                         .cb_mem = nullptr,
                                         .cb_size = 0,
                                         .stack_mem = nullptr,
                                         .stack_size = 512 * 4,
                                         .priority = (osPriority_t)osPriorityRealtime,
                                         .tz_module = 0,
                                         .reserved = 0};
osThreadId_t imuTaskHandle;

class IMU : public bsp::IMU_typeC {
 public:
  using bsp::IMU_typeC::IMU_typeC;

 protected:
  void RxCompleteCallback() final { osThreadFlagsSet(imuTaskHandle, IMU_RX_SIGNAL); }
};

static IMU* imu = nullptr;

void imuTask(void* arg) {
  UNUSED(arg);

  while (true) {
    uint32_t flags = osThreadFlagsWait(IMU_RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & IMU_RX_SIGNAL) imu->Update();
  }
}

//==================================================================================================
// Gimbal(TODO:)
//==================================================================================================

const osThreadAttr_t gimbalTaskAttribute = {.name = "gimbalTask",
                                            .attr_bits = osThreadDetached,
                                            .cb_mem = nullptr,
                                            .cb_size = 0,
                                            .stack_mem = nullptr,
                                            .stack_size = 512 * 4,
                                            .priority = (osPriority_t)osPriorityHigh,
                                            .tz_module = 0,
                                            .reserved = 0};
osThreadId_t gimbalTaskHandle;

static control::Motor4310* pitch_motor = nullptr;
static control::Motor4310* yaw_motor = nullptr;
static bsp::Laser* laser = nullptr;

// 4310 PID
//static control::ConstrainedPID* yaw_pid_position = nullptr;
static control::ConstrainedPID* yaw_theta_pid = nullptr;
static control::ConstrainedPID* yaw_omega_pid = nullptr;

void gimbalTask(void* arg) {
  UNUSED(arg);

  control::Motor4310* motors_can1_gimbal[] = {pitch_motor, yaw_motor};

  print("Wait for beginning signal...\r\n");
  RGB->Display(display::color_red);
  laser->On();

  while (true) {
    if (dbus->keyboard.bit.B || dbus->swr == remote::DOWN) break;
    osDelay(100);
  }

  osDelay(GIMBAL_START_DELAY);

  // the start code for motor 4310
  pitch_motor->SetZeroPos();
  pitch_motor->MotorEnable();
//  yaw_motor->SetZeroPos();
  yaw_motor->MotorEnable();

  float yaw_target = 0;
  float yaw_error;
  float yaw_output_theta;

  float tmp_pitch_pos = 0;
  for (int j = 0; j < SOFT_START_CONSTANT; j++) {
    tmp_pitch_pos += START_PITCH_POS / SOFT_START_CONSTANT;  // increase position gradually
    pitch_motor->SetOutput(tmp_pitch_pos, 1, 115, 0.5, 0);
    // Caluclate the PID output of the yaw motor
    yaw_error = wrap<float>(-(yaw_motor->GetTheta() - yaw_target), -PI, PI);
    yaw_output_theta = yaw_theta_pid->ComputeOutput(yaw_error);
    yaw_motor->SetOutput(yaw_output_theta);
    control::Motor4310::TransmitOutput(motors_can1_gimbal, 2);
    osDelay(GIMBAL_TASK_DELAY);
  }

  print("Start Calibration.\r\n");
  RGB->Display(display::color_yellow);
  laser->Off();
  pitch_motor->SetOutput(tmp_pitch_pos, 1, 115, 0.5, 0);
  yaw_motor->SetOutput(0, 0, 0, 0, 0);
  control::Motor4310::TransmitOutput(motors_can1_gimbal, 2);

  osDelay(GIMBAL_START_DELAY);
  imu->Calibrate();

  while (!imu->DataReady() || !imu->CaliDone()) {
    pitch_motor->SetOutput(tmp_pitch_pos, 1, 115, 0.5, 0);
    yaw_motor->SetOutput(0, 0, 0, 0, 0);
    control::Motor4310::TransmitOutput(motors_can1_gimbal, 2);
    osDelay(GIMBAL_TASK_DELAY);
  }

  print("Gimbal Begin!\r\n");
  RGB->Display(display::color_green);
  laser->On();

  send->cmd.id = bsp::START;
  send->cmd.data_bool = true;
  send->TransmitOutput();

  float pitch_vel_range = 5.0;

  while (true) {
    while (Dead || GimbalDead) osDelay(100);

    // pitch calculation
    float pitch_vel;
    pitch_vel = clip<float>(dbus->ch3 / 660.0 * pitch_vel_range, -pitch_vel_range, pitch_vel_range);
    pitch_pos += pitch_vel / 200 - dbus->mouse.y / 20000.0;
    pitch_pos = clip<float>(pitch_pos, 0.05, 0.73);  // measured range

    if (pitch_reset) {
      // 4310 soft start
      tmp_pitch_pos = 0;
      for (int j = 0; j < SOFT_START_CONSTANT; j++){
        tmp_pitch_pos += START_PITCH_POS / SOFT_START_CONSTANT;  // increase position gradually
        pitch_motor->SetOutput(tmp_pitch_pos, 1, 115, 0.5, 0);
        yaw_motor->SetOutput(0, 0, 0, 0, 0);
        control::Motor4310::TransmitOutput(motors_can1_gimbal, 2);
        osDelay(GIMBAL_TASK_DELAY);
      }
      pitch_pos = tmp_pitch_pos;
      pitch_reset = false;
    }

    pitch_motor->SetOutput(pitch_pos, pitch_vel, 115, 0.5, 0);

    float yaw_ratio;
    yaw_ratio = -dbus->mouse.x / 10000.0;
    yaw_ratio += -dbus->ch2 / 20000.0;
    yaw_ratio = clip<float>(yaw_ratio, -PI, PI);
    yaw_target = wrap<float>(yaw_ratio + yaw_target, -PI, PI);
    yaw_error = wrap<float>(yaw_target - imu->INS_angle[0], -PI, PI);
    float yaw_theta_out = yaw_theta_pid->ComputeOutput(yaw_error);
    yaw_theta_out = clip<float>(yaw_theta_out, -15, 15);

    #ifdef GEAR_GIMBAL
      yaw_motor->SetOutput(0, -yaw_theta_out, 0, 1.5, 0);
    #else
      yaw_motor->SetOutput(0, yaw_theta_out, 0, 1.5, 0);
    #endif

    control::Motor4310::TransmitOutput(motors_can1_gimbal, 2);
    osDelay(GIMBAL_TASK_DELAY);
  }
}

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
// Shooter(TODO:)
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

static control::MotorCANBase* left_top_flywheel = nullptr;
static control::MotorCANBase* left_bottom_flywheel = nullptr;
static control::MotorCANBase* left_dial = nullptr;
static control::Shooter* left_shooter = nullptr;

static control::MotorCANBase* right_top_flywheel = nullptr;
static control::MotorCANBase* right_bottom_flywheel = nullptr;
static control::MotorCANBase* right_dial = nullptr;
static control::Shooter* right_shooter = nullptr;

static volatile bool leftflywheelFlag = false;
static volatile bool rightflywheelFlag = false;

void shooterTask(void* arg) {
 UNUSED(arg);

 control::MotorCANBase* motors_can1_shooter_left[] = {left_top_flywheel, left_bottom_flywheel, left_dial};
 control::MotorCANBase* motors_can1_shooter_right[] = {right_top_flywheel, right_bottom_flywheel, right_dial};

 bool triple_shoot_detect_left = false;
 bool triple_shoot_detect_right = false;

 while (true) {
   if (dbus->keyboard.bit.B || dbus->swr == remote::DOWN) break;
   osDelay(100);
 }

 while (!imu->CaliDone()) osDelay(100);

 while (true) {
   while (Dead) osDelay(100);

  // print("power1: %d, cooling heat: %.4f, cooling limit: %.4f\r\n", send->shooter_power, send->cooling_heat1, send->cooling_limit1);
  // print("power2: %d, cooling heat: %.4f, cooling limit: %.4f\r\n", send->shooter_power, send->cooling_heat2, send->cooling_limit2);
  // print("send->speed_limit1: %f, send->speed_limit2: %f\r\n", send->speed_limit1, send->speed_limit2);

//    left shooter(dial part)
   if (send->shooter_power && send->cooling_heat1 >= send->cooling_limit1 - 20) {
     left_top_flywheel->SetOutput(0);
     left_bottom_flywheel->SetOutput(0);
     left_dial->SetOutput(0);
   } else if (GimbalDead) {
     left_shooter->DialStop();
   } else if (send->shooter_power) {
     // for manual antijam
     Antijam.input(dbus->keyboard.bit.G);
     // slow shooting
     if (dbus->mouse.l || dbus->swr == remote::UP) {
       left_shooter->SlowContinueShoot();
       // fast shooting
     } else if ((dbus->mouse.r || dbus->wheel.wheel > remote::WheelDigitalValue)
                && send->cooling_heat1 < send->cooling_limit1 - 24) {
       left_shooter->FastContinueShoot();
       // triple shooting
     } else if (dbus->wheel.wheel == remote::WheelDigitalValue
                && dbus->previous_wheel_value == remote::WheelDigitalValue) {
       if (!triple_shoot_detect_left) {
         triple_shoot_detect_left = true;
         left_shooter->TripleShoot();
       }
       // manual antijam
     } else if (Antijam.posEdge()) {
       left_shooter->Antijam();
       // stop
     } else {
       left_shooter->DialStop();
       triple_shoot_detect_left = false;
     }
     dbus->previous_wheel_value = dbus->wheel.wheel;
   }

   // right shooter(dial part)
   if (send->shooter_power && send->cooling_heat2 >= send->cooling_limit2 - 20) {
     right_top_flywheel->SetOutput(0);
     right_bottom_flywheel->SetOutput(0);
     right_dial->SetOutput(0);
   } else if (GimbalDead) {
     right_shooter->DialStop();
   } else if (send->shooter_power) {
     // for manual antijam
     Antijam.input(dbus->keyboard.bit.G);
     // slow shooting
     if (dbus->mouse.l || dbus->swr == remote::UP) {
       right_shooter->SlowContinueShoot();
       // fast shooting
     } else if ((dbus->mouse.r || dbus->wheel.wheel > remote::WheelDigitalValue)
                && send->cooling_heat2 < send->cooling_limit2 - 24) {
       right_shooter->FastContinueShoot();
       // triple shooting
     } else if (dbus->wheel.wheel == remote::WheelDigitalValue
                && dbus->previous_wheel_value == remote::WheelDigitalValue) {
       if (!triple_shoot_detect_right) {
         triple_shoot_detect_right = true;
         right_shooter->TripleShoot();
       }
       // manual antijam
     } else if (Antijam.posEdge()) {
       right_shooter->Antijam();
       // stop
     } else {
       right_shooter->DialStop();
       triple_shoot_detect_right = false;
     }
     dbus->previous_wheel_value = dbus->wheel.wheel;
   }

   if (GimbalDead) {
     leftflywheelFlag = false;
     left_shooter->SetFlywheelSpeed(0);
     rightflywheelFlag = false;
     right_shooter->SetFlywheelSpeed(0);
   } else {
     // flywheel part for left shooter
     if (!send->shooter_power || dbus->keyboard.bit.Q || dbus->swr == remote::DOWN) {
       leftflywheelFlag = false;
       left_shooter->SetFlywheelSpeed(0);
     } else {
       if (send->speed_limit1 == 15.0) {
         leftflywheelFlag = true;
         left_shooter->SetFlywheelSpeed(485);  // 445 MAX
       } else if (send->speed_limit1 == 18.0) {
         leftflywheelFlag = true;
         left_shooter->SetFlywheelSpeed(530);  // 490 MAX
       } else if (send->speed_limit1 == 30.0) {
         leftflywheelFlag = true;
         left_shooter->SetFlywheelSpeed(770);
       } else {
         leftflywheelFlag = false;
         left_shooter->SetFlywheelSpeed(0);
       }
     }
     // flywheel part for right shooter
     if (!send->shooter_power || dbus->keyboard.bit.Q || dbus->swr == remote::DOWN) {
       rightflywheelFlag = false;
       right_shooter->SetFlywheelSpeed(0);
     } else {
       if (send->speed_limit2 == 15.0) {
         rightflywheelFlag = true;
         right_shooter->SetFlywheelSpeed(485);  // 445 MAX
       } else if (send->speed_limit2 == 18.0) {
         rightflywheelFlag = true;
         right_shooter->SetFlywheelSpeed(530);  // 490 MAX
       } else if (send->speed_limit2 == 30.0) {
         rightflywheelFlag = true;
         right_shooter->SetFlywheelSpeed(770);
       } else {
         rightflywheelFlag = false;
         right_shooter->SetFlywheelSpeed(0);
       }
     }
   }

   left_shooter->Update();
   control::MotorCANBase::TransmitOutput(motors_can1_shooter_left, 3);
   right_shooter->Update();
   control::MotorCANBase::TransmitOutput(motors_can1_shooter_right, 3);

   osDelay(SHOOTER_TASK_DELAY);
 }
}

//==================================================================================================
// Chassis(TODO:)
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

void chassisTask(void* arg) {
  UNUSED(arg);

  while (true) {
    if (dbus->keyboard.bit.B || dbus->swr == remote::DOWN) break;
    osDelay(100);
  }

  while (!imu->CaliDone()) osDelay(100);

  float vx_keyboard = 0, vy_keyboard = 0;
  float vx_remote, vy_remote;
  float vx_set, vy_set;

  while (true) {
    ChangeSpinMode.input(dbus->keyboard.bit.SHIFT || dbus->swl == remote::UP);
    if (ChangeSpinMode.posEdge()) SpinMode = !SpinMode;

    send->cmd.id = bsp::MODE;
    send->cmd.data_int = SpinMode ? 1 : 0;
    send->TransmitOutput();

    if (dbus->keyboard.bit.A) vx_keyboard += 61.5;
    if (dbus->keyboard.bit.D) vx_keyboard -= 61.5;
    if (dbus->keyboard.bit.W) vy_keyboard -= 61.5;
    if (dbus->keyboard.bit.S) vy_keyboard += 61.5;

    if (-35 <= vx_keyboard && vx_keyboard <= 35) vx_keyboard = 0;
    if (-35 <= vy_keyboard && vy_keyboard <= 35) vy_keyboard = 0;

    if (vx_keyboard > 0)
      vx_keyboard -= 60;
    else if (vx_keyboard < 0)
      vx_keyboard += 60;

    if (vy_keyboard > 0)
      vy_keyboard -= 60;
    else if (vy_keyboard < 0)
      vy_keyboard += 60;

    vx_keyboard = clip<float>(vx_keyboard, -1200, 1200);
    vy_keyboard = clip<float>(vy_keyboard, -1200, 1200);

    vx_remote = -dbus->ch0;
    vy_remote = -dbus->ch1;

    vx_set = vx_keyboard + vx_remote;
    vy_set = vy_keyboard + vy_remote;

    send->cmd.id = bsp::VX;
    send->cmd.data_float = Dead ? 0 : vx_set;
    send->TransmitOutput();

    send->cmd.id = bsp::VY;
    send->cmd.data_float = Dead ? 0 : vy_set;
    send->TransmitOutput();

    // TODO
    // the angle difference between the gimbal and the chassis
    relative_angle = wrap<float>(yaw_motor->GetTheta(), -PI, PI);

    send->cmd.id = bsp::RELATIVE_ANGLE;
    #ifdef GEAR_GIMBAL
      send->cmd.data_float = relative_angle;
    #else
      send->cmd.data_float = -relative_angle;
    #endif
    send->TransmitOutput();

    osDelay(CHASSIS_TASK_DELAY);
  }
}

//==================================================================================================
// Fortress(TODO)
//==================================================================================================

const osThreadAttr_t fortressTaskAttribute = {.name = "fortressTask",
                                              .attr_bits = osThreadDetached,
                                              .cb_mem = nullptr,
                                              .cb_size = 0,
                                              .stack_mem = nullptr,
                                              .stack_size = 256 * 4,
                                              .priority = (osPriority_t)osPriorityNormal,
                                              .tz_module = 0,
                                              .reserved = 0};

osThreadId_t fortressTaskHandle;

void fortressTask(void* arg) {
  UNUSED(arg);

  while (true) {
    if (dbus->keyboard.bit.V || dbus->swr == remote::DOWN) break;
    osDelay(100);
  }

  while (!imu->CaliDone()) osDelay(100);

  // wait for fortress calibrated
  while (!send->fortress_calibrated) osDelay(100);

  while (true) {
    ChangeFortressMode.input(dbus->keyboard.bit.X);
    if (ChangeFortressMode.posEdge()) FortressMode = !FortressMode;

    send->cmd.id = bsp::FORTRESS_MODE;
    send->fortress_mode = FortressMode ? 1 : 0;
    send->TransmitOutput();
  }
}

//==================================================================================================
// SelfTest(TODO)
//==================================================================================================

const osThreadAttr_t selfTestTaskAttribute = {.name = "selfTestTask",
                                              .attr_bits = osThreadDetached,
                                              .cb_mem = nullptr,
                                              .cb_size = 0,
                                              .stack_mem = nullptr,
                                              .stack_size = 256 * 4,
                                              .priority = (osPriority_t)osPriorityBelowNormal,
                                              .tz_module = 0,
                                              .reserved = 0};

osThreadId_t selfTestTaskHandle;

using Note = bsp::BuzzerNote;

static bsp::BuzzerNoteDelayed Mario[] = {
    {Note::Mi3M, 80}, {Note::Silent, 80}, {Note::Mi3M, 80}, {Note::Silent, 240}, {Note::Mi3M, 80}, {Note::Silent, 240}, {Note::Do1M, 80}, {Note::Silent, 80}, {Note::Mi3M, 80}, {Note::Silent, 240}, {Note::So5M, 80}, {Note::Silent, 560}, {Note::So5L, 80}, {Note::Silent, 0}, {Note::Finish, 0}};

static bsp::Buzzer* buzzer = nullptr;
static display::OLED* OLED = nullptr;
// simple bitmask function for chassis flag
void selfTestTask(void* arg) {
  UNUSED(arg);
  osDelay(100);
  // The self test task for chassis will not update after the first check.
  OLED->ShowIlliniRMLOGO();
  buzzer->SingSong(Mario, [](uint32_t milli) { osDelay(milli); });
  OLED->OperateGram(display::PEN_CLEAR);
  // TODO: position of checkboxes and terms need update
  // TODO: need add the right shooter self test displayment
  // Lidar, Calibration, dbus, and temperature
  OLED->ShowString(2, 5, (uint8_t*)"Ldr");
  OLED->ShowString(3, 0, (uint8_t*)"Cal");
  OLED->ShowString(3, 6, (uint8_t*)"Dbs");
  OLED->ShowString(4, 0, (uint8_t*)"Temp:");
  //  OLED->ShowString(4, 0, (uint8_t*)"Ref");

  // Fortress mode motors self test
  OLED->ShowString(0, 12, (uint8_t*)"EL");
  OLED->ShowString(0, 17, (uint8_t*)"ER");
  OLED->ShowString(1, 17, (uint8_t*)"FM");

  // chassis motors self test
  OLED->ShowString(1, 12, (uint8_t*)"FL");
  OLED->ShowString(2, 12, (uint8_t*)"FR");
  OLED->ShowString(3, 12, (uint8_t*)"BL");
  OLED->ShowString(4, 12, (uint8_t*)"BR");

  // Shooters
  OLED->ShowString(0, 0, (uint8_t*)"LT");
  OLED->ShowString(0, 5, (uint8_t*)"LB");
  OLED->ShowString(1, 0, (uint8_t*)"LD");
  OLED->ShowString(1, 5, (uint8_t*)"RT");
  OLED->ShowString(2, 0, (uint8_t*)"RB");
  OLED->ShowString(4, 5, (uint8_t*)"RD");
  char temp[6] = "";
  while (true) {
    osDelay(100);

    // gimbal
    pitch_motor->connection_flag_ = false;
    // TODO
    //    yaw_motor->connection_flag_ = false;

    // left shooter
    left_top_flywheel->connection_flag_ = false;
    left_bottom_flywheel->connection_flag_ = false;
    left_dial->connection_flag_ = false;

    // right shooter
    right_top_flywheel->connection_flag_ = false;
    right_bottom_flywheel->connection_flag_ = false;
    right_dial->connection_flag_ = false;

    referee->connection_flag_ = false;
    dbus->connection_flag_ = false;
    osDelay(SELFTEST_TASK_DELAY);

    // gimbal
    pitch_motor_flag = pitch_motor->connection_flag_;
    // TODO
    //    yaw_motor_flag = yaw_motor->connection_flag_;

    // left shooter
    left_top_flywheel_flag = left_top_flywheel->connection_flag_;
    left_bottom_flywheel_flag = left_bottom_flywheel->connection_flag_;
    left_dial_flag = left_dial->connection_flag_;

    // right shooter
    right_top_flywheel_flag = right_top_flywheel->connection_flag_;
    right_bottom_flywheel_flag = right_bottom_flywheel->connection_flag_;
    right_dial_flag = right_dial->connection_flag_;

    chassis_flag_bitmap = send->chassis_flag;

    fl_motor_flag = (0x01 & chassis_flag_bitmap);
    fr_motor_flag = (0x02 & chassis_flag_bitmap);
    bl_motor_flag = (0x04 & chassis_flag_bitmap);
    br_motor_flag = (0x08 & chassis_flag_bitmap);
    elevator_left_motor_flag = (0x10 & chassis_flag_bitmap);
    elevator_right_motor_flag = (0x20 & chassis_flag_bitmap);
    fortress_motor_flag = (0x40 & chassis_flag_bitmap);

    // TODO: position of checkboxes and terms need update
    OLED->ShowBlock(1, 15, fl_motor_flag);

    OLED->ShowBlock(2, 15, fr_motor_flag);

    OLED->ShowBlock(3, 15, bl_motor_flag);

    OLED->ShowBlock(4, 15, br_motor_flag);

    OLED->ShowBlock(0, 15, elevator_left_motor_flag);

    OLED->ShowBlock(0, 20, elevator_right_motor_flag);

    OLED->ShowBlock(1, 20, fortress_motor_flag);

    //    fl_wheel_flag = send->selfCheck_flag;
    calibration_flag = imu->CaliDone();
    //    referee_flag = referee->connection_flag_;
    dbus_flag = dbus->connection_flag_;

    // TODO: need to add the show block of right shooter(3 motors)
    OLED->ShowBlock(0, 2, pitch_motor_flag);
    OLED->ShowBlock(0, 7, yaw_motor_flag);
    // LEFT shooter
    OLED->ShowBlock(1, 2, left_top_flywheel_flag);
    OLED->ShowBlock(1, 7, left_bottom_flywheel_flag);
    OLED->ShowBlock(2, 2, left_dial_flag);
    // RIGHT shooter (location still need to be updated
    OLED->ShowBlock(3, 2, right_top_flywheel_flag);
    OLED->ShowBlock(3, 7, right_bottom_flywheel_flag);
    OLED->ShowBlock(4, 2, right_dial_flag);
    OLED->ShowBlock(2, 8, lidar_flag);
    OLED->ShowBlock(3, 3, imu->CaliDone());
    OLED->ShowBlock(3, 9, dbus_flag);
    snprintf(temp, 6, "%.2f", imu->Temp);
    OLED->ShowString(4, 6, (uint8_t*)temp);
    OLED->RefreshGram();

    selftestStart = send->self_check_flag;
  }
}

//==================================================================================================
// RTOS Init(TODO:)
//==================================================================================================

void RM_RTOS_Init(void) {
  print_use_uart(&huart1);
  bsp::SetHighresClockTimer(&htim5);

  can1 = new bsp::CAN(&hcan1, true);
  can2 = new bsp::CAN(&hcan2, false);
  dbus = new remote::DBUS(&huart3);
  RGB = new display::RGB(&htim5, 3, 2, 1, 1000000);

  bsp::IST8310_init_t IST8310_init;
  IST8310_init.hi2c = &hi2c3;
  IST8310_init.int_pin = DRDY_IST8310_Pin;
  IST8310_init.rst_group = GPIOG;
  IST8310_init.rst_pin = GPIO_PIN_6;
  bsp::BMI088_init_t BMI088_init;
  BMI088_init.hspi = &hspi1;
  BMI088_init.CS_ACCEL_Port = CS1_ACCEL_GPIO_Port;
  BMI088_init.CS_ACCEL_Pin = CS1_ACCEL_Pin;
  BMI088_init.CS_GYRO_Port = CS1_GYRO_GPIO_Port;
  BMI088_init.CS_GYRO_Pin = CS1_GYRO_Pin;
  bsp::heater_init_t heater_init;
  heater_init.htim = &htim10;
  heater_init.channel = 1;
  heater_init.clock_freq = 1000000;
  heater_init.temp = 45;
  bsp::IMU_typeC_init_t imu_init;
  imu_init.IST8310 = IST8310_init;
  imu_init.BMI088 = BMI088_init;
  imu_init.heater = heater_init;
  imu_init.hspi = &hspi1;
  imu_init.hdma_spi_rx = &hdma_spi1_rx;
  imu_init.hdma_spi_tx = &hdma_spi1_tx;
  imu_init.Accel_INT_pin_ = INT1_ACCEL_Pin;
  imu_init.Gyro_INT_pin_ = INT1_GYRO_Pin;
  imu = new IMU(imu_init, false);

  laser = new bsp::Laser(LASER_GPIO_Port, LASER_Pin);
  pitch_motor = new control::Motor4310(can2, 0x32, 0x33, control::MIT);
  // TODO: initialize the yaw motor
  yaw_motor = new control::Motor4310(can2, 0x34, 0x35, control::MIT);

  //  control::gimbal_t gimbal_data;
  //  gimbal_data.pitch_motor_4310_ = pitch_motor;
  // gimbal_data.yaw_motor = yaw_motor;
  //   gimbal_data.model = control::GIMBAL_FORTRESS_4310;
  //  gimbal = new control::Gimbal(gimbal_data);
  //  gimbal_param = gimbal->GetData();

  referee_uart = new RefereeUART(&huart6);
  referee_uart->SetupRx(300);
  referee_uart->SetupTx(300);
  referee = new communication::Referee;
  // left shooter
  left_top_flywheel = new control::Motor3508(can1, 0x201);
  left_bottom_flywheel = new control::Motor3508(can1, 0x202);
  left_dial = new control::Motor2006(can1, 0x203);
  control::shooter_t left_shooter_data;
  left_shooter_data.left_flywheel_motor = left_top_flywheel;
  left_shooter_data.right_flywheel_motor = left_bottom_flywheel;
  left_shooter_data.load_motor = left_dial;
  left_shooter_data.dial_direction = 1;
  left_shooter_data.model = control::SHOOTER_STANDARD;
  left_shooter = new control::Shooter(left_shooter_data);

  // right shooter
  right_top_flywheel = new control::Motor3508(can1, 0x208);
  right_bottom_flywheel = new control::Motor3508(can1, 0x207);
  right_dial = new control::Motor2006(can1, 0x206);
  control::shooter_t right_shooter_data;
  right_shooter_data.left_flywheel_motor = right_top_flywheel;
  right_shooter_data.right_flywheel_motor = right_bottom_flywheel;
  right_shooter_data.load_motor = right_dial;
  // TODO：need test the -1 direction functionality
  right_shooter_data.dial_direction = -1;
  right_shooter_data.model = control::SHOOTER_STANDARD;
  right_shooter = new control::Shooter(right_shooter_data);

  // stepper = new control::Stepper(&htim1, 1, 1000000, DIR_GPIO_Port, DIR_Pin, ENABLE_GPIO_Port,
  //                                ENABLE_Pin);

  buzzer = new bsp::Buzzer(&htim4, 3, 1000000);
  OLED = new display::OLED(&hi2c2, 0x3C);

  send = new bsp::CanBridge(can2, 0x20A, 0x20B);

  // 4310 PID Init
//  float yaw_pid_param_position[] = {2.5, 10.0, 3.0};
//  float yaw_max_iout_position = 0;
//  float yaw_max_out_position = 30;
//  yaw_pid_position = new control::ConstrainedPID(yaw_pid_param_position, yaw_max_iout_position, yaw_max_out_position);

  float yaw_pid_param_theta[] = {6.0, 1.0, 0.05};
  float yaw_max_iout_theta = 0;
  float yaw_max_out_theta = 15;
  yaw_theta_pid = new control::ConstrainedPID(yaw_pid_param_theta, yaw_max_iout_theta, yaw_max_out_theta);

  float yaw_pid_param_omega[] = {1.0, 0.0, 0.0};
  float yaw_max_iout_omega = 0;
  float yaw_max_out_omega = 18;
  yaw_omega_pid = new control::ConstrainedPID(yaw_pid_param_omega, yaw_max_iout_omega, yaw_max_out_omega);
}

//==================================================================================================
// RTOS Threads Init
//==================================================================================================

void RM_RTOS_Threads_Init(void) {
  imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
  gimbalTaskHandle = osThreadNew(gimbalTask, nullptr, &gimbalTaskAttribute);
  refereeTaskHandle = osThreadNew(refereeTask, nullptr, &refereeTaskAttribute);
  shooterTaskHandle = osThreadNew(shooterTask, nullptr, &shooterTaskAttribute);
  chassisTaskHandle = osThreadNew(chassisTask, nullptr, &chassisTaskAttribute);
  selfTestTaskHandle = osThreadNew(selfTestTask, nullptr, &selfTestTaskAttribute);
//  fortressTaskHandle = osThreadNew(fortressTask, nullptr, &fortressTaskAttribute);
}

//==================================================================================================
// Kill All (TODO:)
//==================================================================================================

void KillAll() {
  RM_EXPECT_TRUE(false, "Operation Killed!\r\n");
  // TODO: change kill all for 4310 yaw motor

  control::MotorCANBase* motors_can1_shooter_left[] = {left_top_flywheel, left_bottom_flywheel, left_dial};
  control::MotorCANBase* motors_can1_shooter_right[] = {right_top_flywheel, right_bottom_flywheel, right_dial};
  control::Motor4310* motor[] = {pitch_motor};

  RGB->Display(display::color_blue);
  laser->Off();

  while (true) {
    send->cmd.id = bsp::DEAD;
    send->cmd.data_bool = true;
    send->TransmitOutput();

    FakeDeath.input(dbus->swl == remote::DOWN);
    if (FakeDeath.posEdge()) {
      SpinMode = false;
      Dead = false;
      RGB->Display(display::color_green);
      laser->On();
      pitch_motor->MotorEnable();
      // TODO: whether the 4310 yaw motor need to enable ???
       yaw_motor->MotorEnable();
      break;
    }

    // 4310 soft kill
    float tmp_pos = pitch_pos;
    for (int j = 0; j < SOFT_KILL_CONSTANT; j++){
      tmp_pos -= START_PITCH_POS / SOFT_KILL_CONSTANT;  // decrease position gradually
      pitch_motor->SetOutput(tmp_pos, 1, 115, 0.5, 0);
      control::Motor4310::TransmitOutput(motor, 1);
      osDelay(GIMBAL_TASK_DELAY);
    }

    pitch_reset = true;
    pitch_motor->MotorDisable();
    // TODO:
//     yaw_motor->MotorDisable();

//     yaw_motor->SetOutput(0);
    // control::MotorCANBase::TransmitOutput(motors_can2_gimbal, 1);

    left_top_flywheel->SetOutput(0);
    left_bottom_flywheel->SetOutput(0);
    left_dial->SetOutput(0);
    right_top_flywheel->SetOutput(0);
    right_bottom_flywheel->SetOutput(0);
    right_dial->SetOutput(0);

    control::MotorCANBase::TransmitOutput(motors_can1_shooter_left, 3);
    control::MotorCANBase::TransmitOutput(motors_can1_shooter_right, 3);

    osDelay(KILLALL_DELAY);
  }
}

void KillGimbal() {
  control::Motor4310* motor[] = {pitch_motor, yaw_motor};

  while (true) {
    left_shooter->DialStop();
    right_shooter->DialStop();
    GimbalDead = true;
    GimbalDeath.input(send->gimbal_power);
    if (GimbalDeath.posEdge() && robot_hp_begin) {
      GimbalDead = false;
      pitch_motor->MotorEnable();
      break;
    }
    print("gimbal killed\r\n");

    // 4310 soft kill
    float tmp_pos = pitch_pos;
    for (int j = 0; j < SOFT_KILL_CONSTANT; j++) {
      tmp_pos -= START_PITCH_POS / SOFT_KILL_CONSTANT;  // decrease position gradually
      pitch_motor->SetOutput(tmp_pos, 1, 115, 0.5, 0);
//      yaw_motor->SetOutput(0);
      control::Motor4310::TransmitOutput(motor, 2);
      osDelay(GIMBAL_TASK_DELAY);
    }

    pitch_reset = true;
    pitch_motor->MotorDisable();
//    yaw_motor->SetOutput(0);
    control::Motor4310::TransmitOutput(motor, 2);

    osDelay(KILLALL_DELAY);
  }
}

static bool debug = false;

//==================================================================================================
// RTOS Default Task (TODO???)
//==================================================================================================

void RM_RTOS_Default_Task(const void* arg) {
  UNUSED(arg);

  while (true) {
    if (send->gimbal_power == 1) robot_hp_begin = true;
    gimbal_alive = robot_hp_begin ? send->gimbal_power : 1;

    FakeDeath.input(dbus->swl == remote::DOWN);
    if (FakeDeath.posEdge()) {
      Dead = true;
      KillAll();
    }

    GimbalDeath.input(gimbal_alive);
    if (GimbalDeath.negEdge()) {
      KillGimbal();
    }

    send->cmd.id = bsp::DEAD;
    send->cmd.data_bool = false;
    send->TransmitOutput();

    if (debug) {
      set_cursor(0, 0);
      clear_screen();

      print("# %.2f s, IMU %s\r\n", HAL_GetTick() / 1000.0,
            imu->CaliDone() ? "\033[1;42mReady\033[0m" : "\033[1;41mNot Ready\033[0m");
      print("Temp: %.2f, Effort: %.2f\r\n", imu->Temp, imu->TempPWM);
      print("Euler Angles: %.2f, %.2f, %.2f\r\n", imu->INS_angle[0] / PI * 180,
            imu->INS_angle[1] / PI * 180, imu->INS_angle[2] / PI * 180);

      print("\r\n");

      print("CH0: %-4d CH1: %-4d CH2: %-4d CH3: %-4d ", dbus->ch0, dbus->ch1, dbus->ch2, dbus->ch3);
      print("SWL: %d SWR: %d @ %d ms\r\n", dbus->swl, dbus->swr, dbus->timestamp);

      print("theta: %.4f, omega: %.4f\r\n", yaw_motor->GetTheta(), yaw_motor->GetOmega());
    }

    osDelay(DEFAULT_TASK_DELAY);
  }
}
