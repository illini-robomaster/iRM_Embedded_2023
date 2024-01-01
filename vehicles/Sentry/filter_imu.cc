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
#include "cmsis_os.h"
#include "i2c.h"
#include "main.h"
#include "spi.h"

#include "filter.h"

#define RX_SIGNAL (1 << 0)

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

void RM_RTOS_Init(void) {/*{{{*/
  print_use_uart(&huart1);

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
}/*}}}*/

void RM_RTOS_Threads_Init(void) {
  imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
}

extern const int order;

void RM_RTOS_Default_Task(const void* arg) {
  UNUSED(arg);
  imu->Calibrate();
  Filter_t acce0_filter;
  Filter_t acce1_filter;
  Filter_t acce2_filter;

  while (!imu->DataReady()) {
    print("%b", imu->DataReady());
  }
  print("\r\n%b\r\n", imu->DataReady());
  float acce0_filtered = 0;
  float acce1_filtered = 0;
  float acce2_filtered = 0;

  for (int i = order - 1; i >=0; i--) {
    acce0_filter.xbuf[i] = imu->ODOM_accel[0];
    acce1_filter.xbuf[i] = imu->ODOM_accel[1];
    acce2_filter.xbuf[i] = imu->ODOM_accel[2];
    acce0_filter.ybuf[i] = imu->ODOM_accel[0];
    acce1_filter.ybuf[i] = imu->ODOM_accel[1];
    acce2_filter.ybuf[i] = imu->ODOM_accel[2];
    osDelay(50);
  }

  while (true) {
    acce0_filter.raw_value = imu->ODOM_accel[0];
    acce1_filter.raw_value = imu->ODOM_accel[1];
    acce2_filter.raw_value = imu->ODOM_accel[2];
    acce0_filtered = Chebyshev50HzLPF(&acce0_filter);
    acce1_filtered = Chebyshev50HzLPF(&acce1_filter);
    acce2_filtered = Chebyshev50HzLPF(&acce2_filter);
    print("%.2f, %.2f, %.2f\r\n", acce0_filtered, acce1_filtered, acce2_filtered);
    //print("%.2f, %.2f, %.2f\r\n", imu->ODOM_accel[0], imu->ODOM_accel[1], imu->ODOM_accel[2]);
    osDelay(50);
  }
}
