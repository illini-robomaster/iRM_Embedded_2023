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

#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "fortress.h"
#include "motor.h"
#include "protocol.h"

static remote::DBUS* dbus = nullptr;

static bsp::GPIO* left = nullptr;
static bsp::GPIO* right = nullptr;

static bsp::CAN* can1 = nullptr;
static bsp::CAN* can2 = nullptr;

static control::MotorCANBase* motor_left = nullptr;
static control::MotorCANBase* motor_right = nullptr;
static control::MotorCANBase* motor_fortress = nullptr;

static control::Fortress* fortress = nullptr;

#define REFEREE_RX_SIGNAL (1 << 0)

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

void RM_RTOS_Init() {
  print_use_uart(&huart1);
  bsp::SetHighresClockTimer(&htim5);

  dbus = new remote::DBUS(&huart3);

  left = new bsp::GPIO(IN1_GPIO_Port, IN1_Pin);
  right = new bsp::GPIO(IN2_GPIO_Port, IN2_Pin);

  can1 = new bsp::CAN(&hcan1, true);
  can2 = new bsp::CAN(&hcan2, false);

  motor_left = new control::Motor3508(can2, 0x205);
  motor_right = new control::Motor3508(can2, 0x208);
  motor_fortress = new control::Motor6020(can2, 0x207);

  referee_uart = new RefereeUART(&huart6);
  referee_uart->SetupRx(300);
  referee_uart->SetupTx(300);
  referee = new communication::Referee;

  control::fortress_t fortress_data;
  fortress_data.leftSwitch = left;
  fortress_data.rightSwitch = right;
  fortress_data.leftElevatorMotor = motor_left;
  fortress_data.rightElevatorMotor = motor_right;
  fortress_data.fortressMotor = motor_fortress;
  fortress = new control::Fortress(fortress_data);
}

void RM_RTOS_Threads_Init(void) {
  refereeTaskHandle = osThreadNew(refereeTask, nullptr, &refereeTaskAttribute);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  control::MotorCANBase* motors_can2_fortress[] = {motor_left, motor_right, motor_fortress};

  while (true) {
    if (dbus->keyboard.bit.V || dbus->swr == remote::DOWN) break;
    osDelay(100);
  }

  while (!fortress->Calibrate()) {
    if (fortress->Error()) {
      while (true) {
        fortress->Stop(control::ELEVATOR);
        control::MotorCANBase::TransmitOutput(motors_can2_fortress, 3);
        osDelay(100);
      }
    }
    control::MotorCANBase::TransmitOutput(motors_can2_fortress, 3);
    osDelay(2);
  }

  int i = 0;
  while (true) {
    if (++i > 100 / 2 && fortress->Finished()) break;
    if (fortress->Error()) {
      while (true) {
        fortress->Stop(control::ELEVATOR);
        control::MotorCANBase::TransmitOutput(motors_can2_fortress, 3);
        osDelay(100);
      }
    }
    fortress->Stop(control::SPINNER);
    fortress->Transform(true);
    control::MotorCANBase::TransmitOutput(motors_can2_fortress, 3);
    osDelay(2);
  }

  int j = 0;
  int z = 0;
  while (true) {
    if (++j > 5000) break;
    if (++z >= 50) {
      z = 0;
      set_cursor(0, 0);
      clear_screen();
      motor_fortress->PrintData();
      print("Power Limit: %.2f / %.2f\r\nPower Buffer: %.2f\r\n",
            referee->power_heat_data.chassis_power,
            (float)referee->game_robot_status.chassis_power_limit,
            (float)referee->power_heat_data.chassis_power_buffer);
    }
    fortress->Stop(control::ELEVATOR);
    fortress->Spin(true, (float)referee->game_robot_status.chassis_power_limit,
                   referee->power_heat_data.chassis_power,
                   (float)referee->power_heat_data.chassis_power_buffer);
    control::MotorCANBase::TransmitOutput(motors_can2_fortress, 3);
    osDelay(2);
  }

  while (true) {
    fortress->Stop(control::ELEVATOR);
    fortress->Stop(control::SPINNER);
    control::MotorCANBase::TransmitOutput(motors_can2_fortress, 3);
    osDelay(2);
  }
}
