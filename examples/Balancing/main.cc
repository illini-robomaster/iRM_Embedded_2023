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

#define RX_SIGNAL (1 << 0)
static bsp::CAN* can1 = nullptr;
static control::MotorCANBase* left_motor = nullptr;
static control::MotorCANBase* right_motor = nullptr;
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
  print_use_uart(&huart1);
  dbus = new remote::DBUS(&huart3);
  can1 = new bsp::CAN(&hcan1, true);

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

  left_motor = new control::Motor3508(can1, 0x201);
  right_motor = new control::Motor3508(can1, 0x202);
}

void RM_RTOS_Threads_Init(void) {
  imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
}
static float balance_pid[3]{130,0,20};

void RM_RTOS_Default_Task(const void* arg) {
  UNUSED(arg);
  while(true){
    if (dbus->swr == remote::UP) {
      break;
    }
  }

  imu->Calibrate();

  while (imu->CaliDone() && imu->DataReady());
  osDelay(10);

  float balance_angle = -1.0;
  float output = 0.0;
  float balance_difference = 0.0;
  int16_t max_output = 5000;
  control::MotorCANBase* motors[] = {left_motor, right_motor};

  while (true) {
    if (dbus->swr == remote::DOWN || imu->INS_angle[1] / PI * 180 < -75 || imu->INS_angle[1] / PI * 180 > 75) {
      left_motor->SetOutput(0);
      right_motor->SetOutput(0);
    } else{
      balance_difference = imu->INS_angle[1] / PI * 180 - balance_angle;
      output = balance_pid[0] * balance_difference + balance_pid[2] * imu->GetGyro()[1];
      print("angle: %.2f, gyro: %.2f, output: %.2f\r\n", balance_difference, imu->GetGyro()[1], output);
      print("111111\n");
      if (output > max_output) {
        output = max_output;
      } else if (output < -max_output) {
        output = -max_output;
      }
      left_motor->SetOutput(-(int16_t)output);
      right_motor->SetOutput((int16_t)output);
    }

    control::MotorCANBase::TransmitOutput(motors, 2);
    osDelay(10);


//    set_cursor(0, 0);
//    clear_screen();
//    print("# %.2f s, IMU %s\r\n", HAL_GetTick() / 1000.0,
//          imu->DataReady() ? "\033[1;42mReady\033[0m" : "\033[1;41mNot Ready\033[0m");
//    print("Temp: %.2f\r\n", imu->Temp);
//    print("Euler Angles: %.2f, %.2f, %.2f\r\n", imu->INS_angle[0] / PI * 180,
//          imu->INS_angle[1] / PI * 180, imu->INS_angle[2] / PI * 180);
//    osDelay(50);
  }
}
