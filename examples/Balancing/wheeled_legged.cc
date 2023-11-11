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

#include "bsp_imu.h"
#include "bsp_print.h"
#include "bsp_can.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "main.h"
#include "spi.h"
#include "motor.h"
#include "dbus.h"
#include "utils.h"
#include "wheeled_legged.h"

#define RX_SIGNAL (1 << 0)
static bsp::CAN* can1 = nullptr;
static bsp::CAN* can2 = nullptr;
static control::MotorCANBase* left_wheel_motor = nullptr;
static control::MotorCANBase* right_wheel_motor = nullptr;
static control::Motor4310* left_frout_leg_motor = nullptr;
static control::Motor4310* left_back_leg_motor = nullptr;
static control::Motor4310* right_frout_leg_motor = nullptr;
static control::Motor4310* right_back_leg_motor = nullptr;
static remote::DBUS* dbus = nullptr;

const osThreadAttr_t imuTaskAttribute = {.name = "imuTask",
                                         .attr_bits = osThreadDetached,
                                         .cb_mem = nullptr,
                                         .cb_size = 0,
                                         .stack_mem = nullptr,
                                         .stack_size = 256 * 4,
                                         .priority = (osPriority_t)osPriorityNormal,
                                         .tz_module = 0,
                                         .reserved = 0};
osThreadId_t imuTaskHandle;

class IMU : public bsp::IMU_typeC {
 public:
  using bsp::IMU_typeC::IMU_typeC;

 protected:
  void RxCompleteCallback() final { osThreadFlagsSet(imuTaskHandle, RX_SIGNAL); }
};

static IMU* imu = nullptr;

void imuTask(void* arg) {
  UNUSED(arg);

  while (true) {
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & RX_SIGNAL) {  // unnecessary check
      imu->Update();
    }
  }
}

void RM_RTOS_Init(void) {
  //dbus and print initialization
  print_use_uart(&huart1);
  dbus = new remote::DBUS(&huart3);

  // can initialization
  can1 = new bsp::CAN(&hcan1, true);
  can2 = new bsp::CAN(&hcan2, false);

  // wheel motor initialization
  left_wheel_motor = new control::Motor3508(can1, 0x201);
  right_wheel_motor = new control::Motor3508(can1, 0x202);

  // leg motor initialization
  left_frout_leg_motor = new control::Motor4310(can2, 0x02, 0x01, control::MIT);
  left_back_leg_motor = new control::Motor4310(can2, 0x04, 0x03, control::MIT);
  right_frout_leg_motor = new control::Motor4310(can2, 0x06, 0x05, control::MIT);
  right_back_leg_motor = new control::Motor4310(can2, 0x08, 0x07, control::MIT);

  // imu initialization
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
}

void RM_RTOS_Threads_Init(void) {
  imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
}

void RM_RTOS_Default_Task(const void* arg) {
  UNUSED(arg);
  // start switch
  while(true){
    if (dbus->swr == remote::UP) {
      break;
    }
  }

  imu->Calibrate();

  while (imu->CaliDone() && imu->DataReady());
  osDelay(10);
  print("Calibration done\r\n");

  // leg motor activation
  left_frout_leg_motor->SetZeroPos();
  left_frout_leg_motor->MotorEnable();
  left_back_leg_motor->SetZeroPos();
  left_back_leg_motor->MotorEnable();
  right_frout_leg_motor->SetZeroPos();
  right_frout_leg_motor->MotorEnable();
  right_back_leg_motor->SetZeroPos();
  right_back_leg_motor->MotorEnable();

  control::MotorCANBase* wheel_motors[] = {left_wheel_motor, right_wheel_motor};
  control::Motor4310* leg_motors[] = {left_frout_leg_motor, left_back_leg_motor, right_frout_leg_motor, right_back_leg_motor};

  while (true) {
    // Update wheel omega and robot velocity
    left_wheel_omega = left_wheel_motor->GetOmega() * rpm_rads;
    right_wheel_omega = right_wheel_motor->GetOmega() * rpm_rads;
    left_wheel_speed = left_wheel_omega * wheel_radius;
    right_wheel_speed = right_wheel_motor->GetOmega() * rpm_rads * wheel_radius;
    // need to check the sign for each speed
    robot_speed = (left_wheel_speed + right_wheel_speed) / 2;

    // update IMU data(roll, pitch, yaw)
    //TODO: test the axis of IMU
    yaw = imu->INS_angle[1];
    pitch = imu->INS_angle[0];
    roll = imu->INS_angle[2];
    yaw_omega = imu->GetGyro()[1];
    pitch_omega = imu->GetGyro()[0];
    roll_omega = imu->GetGyro()[2];

    // LQR control
//    torque_left_wheel = -(K2*(robot_speed - vx_set) + K3 )

    osDelay(10);
  }
}
