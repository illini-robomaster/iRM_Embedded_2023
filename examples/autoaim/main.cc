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

#include "main.h"

#include <memory>

#include "autoaim_protocol.h"
#include "bsp_gpio.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "filtering.h"
#include "gimbal.h"
#include "rgb.h"
#include "i2c.h"
#include "bsp_imu.h"

/* Define Gimabal-related parameters */

#define NOTCH (2 * PI / 8)
#define SERVO_SPEED (PI)

#define KEY_GPIO_GROUP GPIOA
#define KEY_GPIO_PIN GPIO_PIN_0

bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;

bsp::GPIO* key = nullptr;
control::MotorCANBase* pitch_motor = nullptr;
control::MotorCANBase* yaw_motor = nullptr;

control::gimbal_t gimbal_init_data;
control::Gimbal* gimbal = nullptr;
remote::DBUS* dbus = nullptr;
bool status = false;

extern osThreadId_t defaultTaskHandle;

/* Define COMM-related parameters */

#define RX_SIGNAL (1 << 0)

const osThreadAttr_t jetsonCommTaskAttribute = {.name = "jetsonCommTask",
                                                .attr_bits = osThreadDetached,
                                                .cb_mem = nullptr,
                                                .cb_size = 0,
                                                .stack_mem = nullptr,
                                                .stack_size = 128 * 4,
                                                .priority = (osPriority_t)osPriorityHigh,
                                                .tz_module = 0,
                                                .reserved = 0};
osThreadId_t jetsonCommTaskHandle;

class CustomUART : public bsp::UART {
 public:
  using bsp::UART::UART;

 protected:
  /* notify application when rx data is pending read */
  void RxCompleteCallback() override final { osThreadFlagsSet(jetsonCommTaskHandle, RX_SIGNAL); }
};

static display::RGB* led = nullptr;

/* Initialize IMU parameters */

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

/* Initialize autoaim parameters */

// TODO: this is NOT thread-safe!
float relative_yaw = 0;
float relative_pitch = 0;
float last_timestamp = 0;  // in milliseconds

void RM_RTOS_Init() {
  can1 = new bsp::CAN(&hcan1, true);
  pitch_motor = new control::Motor6020(can1, 0x205);
  yaw_motor = new control::Motor6020(can1, 0x206);
  gimbal_init_data.pitch_motor = pitch_motor;
  gimbal_init_data.yaw_motor = yaw_motor;
  gimbal_init_data.model = control::GIMBAL_SENTRY;
  gimbal = new control::Gimbal(gimbal_init_data);

  dbus = new remote::DBUS(&huart3);
  key = new bsp::GPIO(KEY_GPIO_GROUP, KEY_GPIO_PIN);

  led = new display::RGB(&htim5, 3, 2, 1, 1000000);

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

void jetsonCommTask(void* arg) {
  UNUSED(arg);

  uint32_t length;
  uint8_t* data;

  auto uart = std::make_unique<CustomUART>(&huart1);  // see cmake for which uart
  uart->SetupRx(50);
  uart->SetupTx(50);

  auto miniPCreceiver = communication::AutoaimProtocol();
  int total_processed_bytes = 0;

  while (!imu->DataReady() || !imu->CaliDone()) {
    osDelay(1);
  }

  while (true) {
    // pitch_curr = imu->INS_angle[1];
    // yaw_curr = imu->INS_angle[0];
    /* wait until rx data is available */
    // led->Display(0xFF0000FF);
    
    // uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    uint32_t flags = osThreadFlagsGet();
    if (flags & RX_SIGNAL) {
      /* time the non-blocking rx / tx calls (should be <= 1 osTick) */

      // max length of the UART buffer at 150Hz is ~50 bytes
      length = uart->Read(&data);
      total_processed_bytes += length;

      miniPCreceiver.Receive(data, length);

      if (miniPCreceiver.get_valid_flag()) {
        // there is at least one unprocessed valid packet
        relative_yaw = miniPCreceiver.get_relative_yaw();
        relative_pitch = miniPCreceiver.get_relative_pitch();
        last_timestamp = HAL_GetTick() / 1000.0;
      }
    }
    // send IMU data anyway
    communication::STMToJetsonData packet_to_send;
    uint8_t my_color = 1; // blue
    const float pitch_curr = imu->INS_angle[1];
    const float yaw_curr = imu->INS_angle[0];
    miniPCreceiver.Send(&packet_to_send, my_color, yaw_curr, pitch_curr, 0);
    uart->Write((uint8_t*)&packet_to_send, sizeof(communication::STMToJetsonData));
    osDelay(2);
  }
}

/* ========================= IMU Task ========================= */

void imuTask(void* arg) {
  UNUSED(arg);

  while (true) {
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & RX_SIGNAL) {  // unnecessary check
      imu->Update();
    }
  }
}
/* ======================== End IMU Task =======================*/

void RM_RTOS_Threads_Init(void) {
  jetsonCommTaskHandle = osThreadNew(jetsonCommTask, nullptr, &jetsonCommTaskAttribute);
  imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
}

// gimbal task
void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  osDelay(500);  // DBUS initialization needs time

  control::MotorCANBase* motors[2] = {pitch_motor, yaw_motor};
  control::gimbal_data_t* gimbal_data = gimbal->GetData();

  while (!key->Read())
    ;
  while (key->Read())
    ;

  UNUSED(gimbal_data);

  // set gimbal to initial position
  int i = 0;
  while (i < 1000 || !imu->DataReady()) {
    gimbal->TargetAbsWOffset(0, 0);
    gimbal->Update();
    control::MotorCANBase::TransmitOutput(motors, 2);
    osDelay(1);
    ++i;
  }
  
  // calibrate IMU with heater
  imu->Calibrate();
  while (!imu->DataReady() || !imu->CaliDone()) {
    gimbal->TargetAbsWOffset(0, 0);
    gimbal->Update();
    control::MotorCANBase::TransmitOutput(motors, 2);
    osDelay(1);
  }

  // IMU data assumed to be valid at this point

  float abs_autoaim_pitch = gimbal->GetTargetPitchAngle();
  float abs_autoaim_yaw = gimbal->GetTargetYawAngle();

  KalmanFilter yaw_filter(abs_autoaim_yaw, HAL_GetTick() / 1000.0);
  KalmanFilter pitch_filter(abs_autoaim_pitch, HAL_GetTick() / 1000.0);

  while (true) {
    // TODO: WANING: this is NOT thread-safe!
    const float rel_pitch_buffer = relative_pitch;
    const float rel_yaw_buffer = relative_yaw;

    // clear after read
    relative_pitch = 0;
    relative_yaw = 0;

    // naive implementation
    // abs_autoaim_pitch = gimbal->ComputePitchRel(rel_pitch_buffer, abs_autoaim_pitch);
    // abs_autoaim_yaw = gimbal->ComputeYawRel(rel_yaw_buffer, abs_autoaim_yaw);
    // gimbal->TargetAbsNoOffset(abs_autoaim_pitch, abs_autoaim_yaw);

    // kalman filter implementation
    if (rel_pitch_buffer != 0 && rel_yaw_buffer != 0) {
      // new data comes in
      abs_autoaim_pitch = gimbal->ComputePitchRel(rel_pitch_buffer, abs_autoaim_pitch);
      abs_autoaim_yaw = gimbal->ComputeYawRel(rel_yaw_buffer, abs_autoaim_yaw);

      pitch_filter.register_state(abs_autoaim_pitch, last_timestamp);
      yaw_filter.register_state(abs_autoaim_yaw, last_timestamp);
    }
    float abs_pitch_filtered = pitch_filter.iter_and_get_estimation();
    float abs_yaw_filtered = yaw_filter.iter_and_get_estimation();
    gimbal->TargetAbsNoOffset(abs_pitch_filtered, abs_yaw_filtered);

    // TODO: add this option to allow user-select autoaim mode
    // if (dbus->swr == remote::MID) {
    // gimbal->TargetRel(rel_pitch_buffer, rel_yaw_buffer);

    // Kill switch
    // if (dbus->swl == remote::UP || dbus->swl == remote::DOWN) {
    //   RM_ASSERT_TRUE(false, "Operation killed");
    // }

    gimbal->Update();
    control::MotorCANBase::TransmitOutput(motors, 2);
    osDelay(10);
  }
}