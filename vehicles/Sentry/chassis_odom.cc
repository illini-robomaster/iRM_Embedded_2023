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

#include "main.h"

#include "bsp_print.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "i2c.h"
#include "spi.h"
#include "bsp_imu.h"
#include "minipc_protocol.h"

#include <cmath>
#include <memory>

bsp::CAN* can = nullptr;
control::MotorCANBase* fl_motor = nullptr;
control::MotorCANBase* fr_motor = nullptr;
control::MotorCANBase* bl_motor = nullptr;
control::MotorCANBase* br_motor = nullptr;

control::Chassis* chassis = nullptr;
remote::DBUS* dbus = nullptr;

#define RX_SIGNAL (1 << 0)

const osThreadAttr_t imuTaskAttribute = {.name = "imuTask",/*{{{*/
                                         .attr_bits = osThreadDetached,
                                         .cb_mem = nullptr,
                                         .cb_size = 0,
                                         .stack_mem = nullptr,
                                         .stack_size = 256 * 4,
                                         .priority = (osPriority_t)osPriorityNormal,
                                         .tz_module = 0,
                                         .reserved = 0};
osThreadId_t imuTaskHandle;

extern osThreadId_t defaultTaskHandle;
class CustomUART : public bsp::UART {
 public:
  using bsp::UART::UART;

 protected:
  /* notify application when rx data is pending read */
  void RxCompleteCallback() override final { osThreadFlagsSet(defaultTaskHandle, RX_SIGNAL); }
};

class IMU : public bsp::IMU_typeC {
 public:
  using bsp::IMU_typeC::IMU_typeC;

 protected:
  void RxCompleteCallback() final { osThreadFlagsSet(imuTaskHandle, RX_SIGNAL); }
};/*}}}*/

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

void RM_RTOS_Init() {/*{{{*/
  can = new bsp::CAN(&hcan1);
  fl_motor = new control::Motor3508(can, 0x201);
  fr_motor = new control::Motor3508(can, 0x204);
  bl_motor = new control::Motor3508(can, 0x202);
  br_motor = new control::Motor3508(can, 0x203);

  control::MotorCANBase* motors[control::FourWheel::motor_num];
  motors[control::FourWheel::front_left] = fl_motor;
  motors[control::FourWheel::front_right] = fr_motor;
  motors[control::FourWheel::back_left] = bl_motor;
  motors[control::FourWheel::back_right] = br_motor;

  control::chassis_t chassis_data;
  chassis_data.motors = motors;
  chassis_data.model = control::CHASSIS_MECANUM_WHEEL;
  chassis = new control::Chassis(chassis_data);

  dbus = new remote::DBUS(&huart3);

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

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  imu->Calibrate();

  osDelay(500);  // DBUS initialization needs time

  control::MotorCANBase* motors[] = {fl_motor, fr_motor, bl_motor, br_motor};
  while (!imu->DataReady()) {
    print("%b", imu->DataReady());
    osDelay(20);
  }

  float x = 0;
  float y = 0;
  float w = 0;

  float sin_yaw = 0;
  float cos_yaw = 0;
  float vx_set = 0;
  float vy_set = 0;
  auto uart = std::make_unique<CustomUART>(&huart1); 
  uart->SetupRx(50);
  uart->SetupTx(50);

  auto minipc_session = communication::MinipcPort();

  communication::chassis_data_t chassis_data;

  uint8_t packet_to_send[minipc_session.MAX_PACKET_LENGTH];

  const float FACTOR = 1.0 / 330.0;
  float deltaT = HAL_GetTick();
  float time = HAL_GetTick();
  while (true) {
    float vx = dbus->ch1;
    float vy = -dbus->ch0;
    float vw = dbus->ch2;
    chassis->SetSpeed(vx, vy, vw);

    // Kill switch
    if (dbus->swr == remote::UP || dbus->swr == remote::DOWN) {
      RM_ASSERT_TRUE(false, "Operation killed");
    }

    deltaT = (HAL_GetTick() - time) / 1000.0;
    time = HAL_GetTick();
    w = imu->INS_angle[0];

    sin_yaw = sin(-w);
    cos_yaw = cos(-w);
    vx_set = cos_yaw * vx + sin_yaw * vy;
    vy_set = -sin_yaw * vx + cos_yaw * vy;
    x = FACTOR * vx_set * deltaT + x;
    y = FACTOR * vy_set * deltaT + y;

    chassis_data.vx = x;
    chassis_data.vy = y;
    chassis_data.vw = w;
    minipc_session.Pack(packet_to_send, (void*)&chassis_data, communication::CHASSIS_CMD_ID);
    uart->Write(packet_to_send, minipc_session.GetPacketLen(communication::CHASSIS_CMD_ID));
    chassis->Update(false, 30, 20, 60);
    control::MotorCANBase::TransmitOutput(motors, 4);
    osDelay(10);
  }
}
