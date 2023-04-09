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
#include "steering.h"
#include "stepper.h"

static bsp::CAN* can1 = nullptr;
static bsp::CAN* can2 = nullptr;
static remote::DBUS* dbus = nullptr;
static display::RGB* RGB = nullptr;

static const int GIMBAL_TASK_DELAY = 1;
static const int CHASSIS_TASK_DELAY = 2;
static const int SHOOTER_TASK_DELAY = 10;
static const int SELFTEST_TASK_DELAY = 100;
static const int KILLALL_DELAY = 100;
static const int DEFAULT_TASK_DELAY = 100;

static bsp::CanBridge* send = nullptr;
static bsp::CanBridge* receive = nullptr;

static BoolEdgeDetector FakeDeath(false);
static volatile bool Dead = false;
static BoolEdgeDetector ChangeSpinMode(false);
static volatile bool SpinMode = false;

static volatile float relative_angle = 0;
static unsigned int chassis_flag_summary = 0;
static bool volatile pitch_motor_flag = false;
static bool volatile yaw_motor_flag = false;
static bool volatile sl_motor_flag = false;
static bool volatile sr_motor_flag = false;
static bool volatile ld_motor_flag = false;
// static bool volatile fl_motor_flag = false;
// static bool volatile fr_motor_flag = false;
// static bool volatile bl_motor_flag = false;
// static bool volatile br_motor_flag = false;
static bool volatile calibration_flag = false;
// static bool volatile referee_flag = false;
static bool volatile dbus_flag = false;
static bool volatile lidar_flag = false;

static volatile bool selftestStart = false;

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
// Gimbal
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

static control::MotorCANBase* pitch_motor = nullptr;
static control::MotorCANBase* yaw_motor = nullptr;
static control::Gimbal* gimbal = nullptr;
static control::gimbal_data_t* gimbal_param = nullptr;
static bsp::Laser* laser = nullptr;

void gimbalTask(void* arg) {
  UNUSED(arg);

  control::MotorCANBase* motors_can1_gimbal[] = {pitch_motor, yaw_motor};

  print("Wait for beginning signal...\r\n");
  RGB->Display(display::color_red);
  laser->On();

  while (true) {
    if (dbus->keyboard.bit.V || dbus->swr == remote::DOWN) break;
    osDelay(100);
  }

  int i = 0;
  while (i < 1000 || !imu->DataReady()) {
    gimbal->TargetAbs(0, 0);
    gimbal->Update();
    control::MotorCANBase::TransmitOutput(motors_can1_gimbal, 2);
    osDelay(GIMBAL_TASK_DELAY);
    ++i;
  }

  print("Start Calibration.\r\n");
  RGB->Display(display::color_yellow);
  laser->Off();
  imu->Calibrate();

  while (!imu->DataReady() || !imu->CaliDone()) {
    gimbal->TargetAbs(0, 0);
    gimbal->Update();
    control::MotorCANBase::TransmitOutput(motors_can1_gimbal, 2);
    osDelay(GIMBAL_TASK_DELAY);
  }

  print("Gimbal Begin!\r\n");
  RGB->Display(display::color_green);
  laser->On();

  send->cmd.id = bsp::START;
  send->cmd.data_bool = true;
  send->TransmitOutput();

  float pitch_ratio, yaw_ratio;
  float pitch_curr, yaw_curr;
  float pitch_target = 0, yaw_target = 0;
  float pitch_diff, yaw_diff;

  while (true) {
    while (Dead) osDelay(100);

    pitch_ratio = dbus->mouse.y / 32767.0;
    yaw_ratio = -dbus->mouse.x / 32767.0;

    pitch_ratio += -dbus->ch3 / 660.0 / 210.0;
    yaw_ratio += -dbus->ch2 / 660.0 / 210.0;

    pitch_target = clip<float>(pitch_target + pitch_ratio, -gimbal_param->pitch_max_,
                               gimbal_param->pitch_max_);
    yaw_target = wrap<float>(yaw_target + yaw_ratio, -PI, PI);

    pitch_curr = imu->INS_angle[1];
    yaw_curr = imu->INS_angle[0];

    pitch_diff = clip<float>(pitch_target - pitch_curr, -PI, PI);
    yaw_diff = wrap<float>(yaw_target - yaw_curr, -PI, PI);

    gimbal->TargetRel(-pitch_diff, yaw_diff);

    gimbal->Update();
    control::MotorCANBase::TransmitOutput(motors_can1_gimbal, 2);
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

static control::MotorCANBase* sl_motor = nullptr;
static control::MotorCANBase* sr_motor = nullptr;
static control::MotorCANBase* ld_motor = nullptr;
static control::Shooter* shooter = nullptr;
static control::Stepper* stepper = nullptr;

static volatile bool flywheelFlag = false;

static unsigned stepper_length = 700;
static unsigned stepper_speed = 1000;
static bool stepper_direction = true;

void shooterTask(void* arg) {
  UNUSED(arg);

  control::MotorCANBase* motors_can1_shooter[] = {sl_motor, sr_motor, ld_motor};

  while (true) {
    if (dbus->keyboard.bit.V || dbus->swr == remote::DOWN) break;
    osDelay(100);
  }

  while (!imu->CaliDone()) osDelay(100);

  for (int i = 0; i < 2; ++i) {
    stepper->Move(control::FORWARD, stepper_speed);
    osDelay(stepper_length);
    stepper->Move(control::BACKWARD, stepper_speed);
    osDelay(stepper_length);
  }
  stepper->Stop();

  while (true) {
    while (Dead) osDelay(100);

    if (send->shooter_power && send->cooling_heat1 > send->cooling_limit1 - 20) {
      sl_motor->SetOutput(0);
      sr_motor->SetOutput(0);
      ld_motor->SetOutput(0);
      control::MotorCANBase::TransmitOutput(motors_can1_shooter, 3);
      osDelay(100);
      if (stepper_direction) {
        stepper->Move(control::FORWARD, stepper_speed);
        osDelay(stepper_length);
        stepper->Stop();
      } else {
        stepper->Move(control::BACKWARD, stepper_speed);
        osDelay(stepper_length);
        stepper->Stop();
      }
      stepper_direction = !stepper_direction;
    }

    if (send->shooter_power && send->cooling_heat1 < send->cooling_limit1 - 20 &&
        (dbus->mouse.l || dbus->swr == remote::UP))
      shooter->LoadNext();
    if (!send->shooter_power || dbus->keyboard.bit.Q || dbus->swr == remote::DOWN) {
      flywheelFlag = false;
      shooter->SetFlywheelSpeed(0);
    } else {
      if (14 < send->speed_limit1 && send->speed_limit1 < 16) {
        flywheelFlag = true;
        shooter->SetFlywheelSpeed(437);  // 445 MAX
      } else if (send->speed_limit1 >= 18) {
        flywheelFlag = true;
        shooter->SetFlywheelSpeed(482);  // 490 MAX
      } else {
        flywheelFlag = false;
        shooter->SetFlywheelSpeed(0);
      }
    }

    shooter->Update();
    control::MotorCANBase::TransmitOutput(motors_can1_shooter, 3);
    osDelay(SHOOTER_TASK_DELAY);
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
osThreadId_t chassisTaskHandle;

void chassisTask(void* arg) {
  UNUSED(arg);

  while (true) {
    if (dbus->keyboard.bit.V || dbus->swr == remote::DOWN) break;
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

    if (dbus->keyboard.bit.A) vx_keyboard -= 61.5;
    if (dbus->keyboard.bit.D) vx_keyboard += 61.5;
    if (dbus->keyboard.bit.W) vy_keyboard += 61.5;
    if (dbus->keyboard.bit.S) vy_keyboard -= 61.5;

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

    vx_remote = dbus->ch0;
    vy_remote = dbus->ch1;

    vx_set = vx_keyboard + vx_remote;
    vy_set = vy_keyboard + vy_remote;

    send->cmd.id = bsp::VX;
    send->cmd.data_float = Dead ? 0 : vx_set;
    send->TransmitOutput();

    send->cmd.id = bsp::VY;
    send->cmd.data_float = Dead ? 0 : vy_set;
    send->TransmitOutput();

    osDelay(CHASSIS_TASK_DELAY);
  }
}

//==================================================================================================
// SelfTest
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

void selfTestTask(void* arg) {
  UNUSED(arg);
  delete(send);
  receive = new bsp::CanBridge(can2, 0x20B, 0x20A);
  receive->cmd.id = bsp::CHASSIS_FLAG;
  chassis_flag_summary = receive->chassis_flag;
  receive->TransmitOutput();
  delete(receive);
  send = new bsp::CanBridge(can2, 0x20A, 0x20B);
  //Could need more time to test it out.
  OLED->ShowIlliniRMLOGO();
  buzzer->SingSong(Mario, [](uint32_t milli) { osDelay(milli); });
  OLED->OperateGram(display::PEN_CLEAR);

  OLED->ShowString(0, 0, (uint8_t*)"GP");
  OLED->ShowString(0, 5, (uint8_t*)"GY");
  OLED->ShowString(1, 0, (uint8_t*)"SL");
  OLED->ShowString(1, 5, (uint8_t*)"SR");
  OLED->ShowString(1, 10, (uint8_t*)"LD");
  //  OLED->ShowString(2, 0, (uint8_t*)"FL");
  //  OLED->ShowString(2, 5, (uint8_t*)"FR");
  //  OLED->ShowString(2, 10, (uint8_t*)"BL");
  //  OLED->ShowString(2, 15, (uint8_t*)"BR");
  OLED->ShowString(3, 0, (uint8_t*)"Cali");
  OLED->ShowString(3, 7, (uint8_t*)"Temp:");
  //  OLED->ShowString(4, 0, (uint8_t*)"Ref");
  OLED->ShowString(4, 6, (uint8_t*)"Dbus");
  OLED->ShowString(4, 13, (uint8_t*)"Lidar");

  char temp[6] = "";
  while (true) {
    pitch_motor->connection_flag_ = false;
    yaw_motor->connection_flag_ = false;
    sl_motor->connection_flag_ = false;
    sr_motor->connection_flag_ = false;
    ld_motor->connection_flag_ = false;

    //    fl_motor->connection_flag_ = false;
    //    fr_motor->connection_flag_ = false;
    //    bl_motor->connection_flag_ = false;
    //    br_motor->connection_flag_ = false;
    referee->connection_flag_ = false;
    dbus->connection_flag_ = false;
    osDelay(SELFTEST_TASK_DELAY);
    pitch_motor_flag = pitch_motor->connection_flag_;
    yaw_motor_flag = yaw_motor->connection_flag_;
    sl_motor_flag = sl_motor->connection_flag_;
    sr_motor_flag = sr_motor->connection_flag_;
    ld_motor_flag = ld_motor->connection_flag_;

    //    fl_motor_flag = fl_motor->connection_flag_;
    //    fr_motor_flag = fr_motor->connection_flag_;
    //    bl_motor_flag = bl_motor->connection_flag_;
    //    br_motor_flag = br_motor->connection_flag_;
    calibration_flag = imu->CaliDone();
    //    referee_flag = referee->connection_flag_;
    dbus_flag = dbus->connection_flag_;

    OLED->ShowBlock(0, 2, pitch_motor_flag);
    OLED->ShowBlock(0, 7, yaw_motor_flag);
    OLED->ShowBlock(1, 2, sl_motor_flag);
    OLED->ShowBlock(1, 7, sr_motor_flag);
    OLED->ShowBlock(1, 12, ld_motor_flag);
    //    OLED->ShowBlock(2, 2, fl_motor_flag);
    //    OLED->ShowBlock(2, 7, fr_motor_flag);
    //    OLED->ShowBlock(2, 12, bl_motor_flag);
    //    OLED->ShowBlock(2, 17, br_motor_flag);
    OLED->ShowBlock(3, 4, imu->CaliDone());
    snprintf(temp, 6, "%.2f", imu->Temp);
    OLED->ShowString(3, 12, (uint8_t*)temp);
    //    OLED->ShowBlock(4, 3, referee_flag);
    OLED->ShowBlock(4, 10, dbus_flag);
    OLED->ShowBlock(4, 18, lidar_flag);

    OLED->RefreshGram();

    selftestStart = true;
  }
}

void RM_RTOS_Init(void) {
  print_use_uart(&huart1);
  bsp::SetHighresClockTimer(&htim5);

  can1 = new bsp::CAN(&hcan1, 0x201, true);
  can2 = new bsp::CAN(&hcan2, 0x201, false);
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
  pitch_motor = new control::Motor6020(can1, 0x205);
  yaw_motor = new control::Motor6020(can1, 0x206);
  control::gimbal_t gimbal_data;
  gimbal_data.pitch_motor = pitch_motor;
  gimbal_data.yaw_motor = yaw_motor;
  gimbal_data.model = control::GIMBAL_STEERING;
  gimbal = new control::Gimbal(gimbal_data);
  gimbal_param = gimbal->GetData();

  referee_uart = new RefereeUART(&huart6);
  referee_uart->SetupRx(300);
  referee_uart->SetupTx(300);
  referee = new communication::Referee;

  sl_motor = new control::Motor3508(can1, 0x201);
  sr_motor = new control::Motor3508(can1, 0x202);
  ld_motor = new control::Motor2006(can1, 0x203);
  control::shooter_t shooter_data;
  shooter_data.left_flywheel_motor = sl_motor;
  shooter_data.right_flywheel_motor = sr_motor;
  shooter_data.load_motor = ld_motor;
  shooter_data.model = control::SHOOTER_STANDARD;
  shooter = new control::Shooter(shooter_data);
  stepper = new control::Stepper(&htim1, 1, 1000000, DIR_GPIO_Port, DIR_Pin, ENABLE_GPIO_Port,
                                 ENABLE_Pin);

  buzzer = new bsp::Buzzer(&htim4, 3, 1000000);
  OLED = new display::OLED(&hi2c2, 0x3C);

  send = new bsp::CanBridge(can2, 0x20A, 0x20B);
}

void RM_RTOS_Threads_Init(void) {
  imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
  gimbalTaskHandle = osThreadNew(gimbalTask, nullptr, &gimbalTaskAttribute);
  refereeTaskHandle = osThreadNew(refereeTask, nullptr, &refereeTaskAttribute);
  shooterTaskHandle = osThreadNew(shooterTask, nullptr, &shooterTaskAttribute);
  chassisTaskHandle = osThreadNew(chassisTask, nullptr, &chassisTaskAttribute);
  selfTestTaskHandle = osThreadNew(selfTestTask, nullptr, &selfTestTaskAttribute);
}

void KillAll() {
  RM_EXPECT_TRUE(false, "Operation Killed!\r\n");

  control::MotorCANBase* motors_can1_gimbal[] = {pitch_motor};
  control::MotorCANBase* motors_can2_gimbal[] = {yaw_motor};
  control::MotorCANBase* motors_can1_shooter[] = {sl_motor, sr_motor, ld_motor};

  RGB->Display(display::color_blue);
  laser->Off();

  while (true) {
    send->cmd.id = bsp::DEAD;
    send->cmd.data_bool = true;
    send->TransmitOutput();

    FakeDeath.input(dbus->keyboard.bit.B || dbus->swl == remote::DOWN);
    if (FakeDeath.posEdge()) {
      SpinMode = false;
      Dead = false;
      RGB->Display(display::color_green);
      laser->On();
      break;
    }

    pitch_motor->SetOutput(0);
    yaw_motor->SetOutput(0);
    control::MotorCANBase::TransmitOutput(motors_can1_gimbal, 1);
    control::MotorCANBase::TransmitOutput(motors_can2_gimbal, 1);

    sl_motor->SetOutput(0);
    sr_motor->SetOutput(0);
    ld_motor->SetOutput(0);
    control::MotorCANBase::TransmitOutput(motors_can1_shooter, 3);

    osDelay(KILLALL_DELAY);
  }
}

static bool debug = true;

void RM_RTOS_Default_Task(const void* arg) {
  UNUSED(arg);

  while (true) {
    FakeDeath.input(dbus->keyboard.bit.B || dbus->swl == remote::DOWN);
    if (FakeDeath.posEdge()) {
      Dead = true;
      KillAll();
    }
    send->cmd.id = bsp::DEAD;
    send->cmd.data_bool = false;
    send->TransmitOutput();

    relative_angle = yaw_motor->GetThetaDelta(gimbal_param->yaw_offset_);
    send->cmd.id = bsp::RELATIVE_ANGLE;
    send->cmd.data_float = relative_angle;
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
    }

    osDelay(DEFAULT_TASK_DELAY);
  }
}
