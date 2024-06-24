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
#include <memory>
#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "minipc_protocol.h"
#include "rgb.h"
#include "chassis.h"
#include "dbus.h"
#include "tim.h"

#define RX_SIGNAL (1 << 0)
#define DATA_READY_SIGNAL (1 << 1)
extern osThreadId_t defaultTaskHandle;

static display::RGB* led = nullptr;
bsp::CAN* can = nullptr;
control::MotorCANBase* fl_motor = nullptr;
control::MotorCANBase* fr_motor = nullptr;
control::MotorCANBase* bl_motor = nullptr;
control::MotorCANBase* br_motor = nullptr;

control::Chassis* chassis = nullptr;
remote::DBUS* dbus = nullptr;

osEventFlagsId_t chassis_flag_id;

static float vx = 0;
static float vy = 0;
static float vw = 0;

class CustomUART : public bsp::UART {
 public:
  using bsp::UART::UART;

 protected:
  /* notify application when rx data is pending read */
  void RxCompleteCallback() override final { osThreadFlagsSet(defaultTaskHandle, RX_SIGNAL); }
};

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

void chassisTask(void* argument) {
  UNUSED(argument);
  uint32_t flags;
  control::MotorCANBase* motors[] = {fl_motor, fr_motor, bl_motor, br_motor};

  float vx_keyboard = 0, vy_keyboard = 0, wz_keyboard = 0;
  float vx_remote, vy_remote, wz_remote;
  float vx_set, vy_set, wz_set;

  while (true) {
    // Wait time = 50 ticks, 50ms?
    flags = 0;
    flags = osEventFlagsWait(chassis_flag_id, DATA_READY_SIGNAL, osFlagsWaitAny, 50);

    vx_keyboard = vx;
    vy_keyboard = vy;
    wz_keyboard = vw;

    uint32_t RGB_color[3] = {0xFFFF0000, 0xFF00FF00, 0xFF0000FF};

    // When timeout it returns -2 so we need extra checks here
    if (flags != osFlagsErrorTimeout && flags & DATA_READY_SIGNAL) {
      vx_keyboard = clip<float>(vx_keyboard, -1200, 1200);
      vy_keyboard = clip<float>(vy_keyboard, -1200, 1200);

      vx_remote = dbus->ch0;
      vy_remote = dbus->ch1;
      wz_remote = dbus->ch2;

      vx_set = vx_keyboard + vx_remote;
      vy_set = vy_keyboard + vy_remote;
      wz_set = wz_keyboard + wz_remote;

      chassis->SetSpeed(vx_set, vy_set, wz_set);
      chassis->Update(false, 30, 20, 60);

      led->Display(RGB_color[1]);
    } else {
      // if timeout (no packet, stop the motor)
      fl_motor->SetOutput(0);
      fr_motor->SetOutput(0);
      bl_motor->SetOutput(0);
      br_motor->SetOutput(0);

      led->Display(RGB_color[0]);
    }

    control::MotorCANBase::TransmitOutput(motors, 4);
    osDelay(10);
  }
}

void RM_RTOS_Init(void) {
  led = new display::RGB(&htim5, 3, 2, 1, 1000000);

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

  chassis_flag_id = osEventFlagsNew(nullptr);
}

void RM_RTOS_Threads_Init(void) {
  chassisTaskHandle = osThreadNew(chassisTask, nullptr, &chassisTaskAttribute);
}

void RM_RTOS_Default_Task(const void* argument) {
  UNUSED(argument);

  auto uart = std::make_unique<CustomUART>(&huart1);
  uart->SetupRx(50);
  uart->SetupTx(50);

  auto minipc_session = communication::MinipcPort();
  const communication::status_data_t* status_data;

  uint8_t *data;
  int32_t length;

  while (true) {
    // Wait until first packet from minipc.
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & RX_SIGNAL) {
      length = uart->Read(&data);
      minipc_session.ParseUartBuffer(data, length);
      status_data = minipc_session.GetStatus();

      vx = status_data->vx;
      vy = status_data->vy;
      vw = status_data->vw;

      osEventFlagsSet(chassis_flag_id, DATA_READY_SIGNAL);
    }
    osDelay(10);
  }
}
