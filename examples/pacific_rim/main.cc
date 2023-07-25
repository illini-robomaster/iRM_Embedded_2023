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
#include "arm_math.h"

#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "unitree_motor.h"
#include "sbus.h"

static control::UnitreeMotor* A1 = nullptr;
static remote::SBUS* sbus;

#define RX_SIGNAL (1 << 0)

const osThreadAttr_t A1TaskAttribute = {.name = "A1Task",
                                        .attr_bits = osThreadDetached,
                                        .cb_mem = nullptr,
                                        .cb_size = 0,
                                        .stack_mem = nullptr,
                                        .stack_size = 128 * 4,
                                        .priority = (osPriority_t)osPriorityNormal,
                                        .tz_module = 0,
                                        .reserved = 0};

osThreadId_t A1TaskHandle;

class CustomUART: public bsp::UART {
 public:
  using bsp::UART::UART;

 protected:
  /* notify application when rx data is pending read */
  void RxCompleteCallback() override final { osThreadFlagsSet(A1TaskHandle, RX_SIGNAL); }
};

static CustomUART* A1_uart = nullptr;

void A1Task(void* arg) {
  UNUSED(arg);
  uint32_t length;
  uint8_t* data;

  while (true) {
    /* wait until rx data is available */
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & RX_SIGNAL) {  // unnecessary check
      /* time the non-blocking rx / tx calls (should be <= 1 osTick) */
      length = A1_uart->Read(&data);
      if ((int)length == A1->recv_length)
        A1->ExtractData(communication::package_t{data, (int)length});
    }
  }
}

void RM_RTOS_Init(void) {
  print_use_uart(&huart8);

  A1_uart = new CustomUART(&huart6);
  A1_uart->SetupRx(300);
  A1_uart->SetupTx(300);

  A1 = new control::UnitreeMotor();

  sbus = new remote::SBUS(&huart1);
}

void RM_RTOS_Threads_Init(void) {
  A1TaskHandle = osThreadNew(A1Task, nullptr, &A1TaskAttribute);
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);

//  A1->Test(0);
//  A1_uart->Write((uint8_t*)(&(A1->send.data)), A1->send_length);
//  osDelay(3000);
//
  A1->Stop(0);
  A1_uart->Write((uint8_t*)(&(A1->send.data)), A1->send_length);
  osDelay(3000);
//
//  A1->Control(0, 0.0, -1.0, 0.0, 0.0, 3.0); // constant speed mode
//  for (int i = 0; i < 30; ++i) {
//    A1_uart->Write((uint8_t*)(&(A1->send.data)), A1->send_length);
//    set_cursor(0, 0);
//    clear_screen();
//    print("Motor ID: %d\r\n", A1->recv.id);
//    print("Mode    : %d\r\n", A1->recv.mode);
//    print("Temp    : %d\r\n", A1->recv.Temp);
//    print("MError  : %d\r\n", A1->recv.MError);
//    print("Torque  : %.3f\r\n", A1->recv.T);
//    print("Speed   : %.3f\r\n", A1->recv.W);
//    print("Accel   : %d\r\n", A1->recv.Acc);
//    print("Position: %.3f\r\n", A1->recv.Pos);
//    osDelay(100);
//  }

//  A1->Control(0, 0.0, 0.0, 0.0, 0.0, 0.0); // zero torque mode
  while (true) {
//    set_cursor(0, 0);
//    clear_screen();
//
//    print("%f\n", (sbus->ch[3] + 1024.0f) / 2048 * 2*PI);
//    print("CH0: %-4d CH1: %-4d CH2: %-4d CH3: %-4d ", sbus->ch[0], sbus->ch[1], sbus->ch[2], sbus->ch[3]);
//    print("CH4: %-4d CH5: %-4d CH6: %-4d CH7: %-4d ", sbus->ch[4], sbus->ch[5], sbus->ch[6], sbus->ch[7]);
//    print("CH8: %-4d CH9: %-4d CH10: %-4d CH11: %-4d ", sbus->ch[8], sbus->ch[9], sbus->ch[10], sbus->ch[11]);
//    print("CH12: %-4d CH13: %-4d CH14: %-4d CH15: %-4d ", sbus->ch[12], sbus->ch[13], sbus->ch[14], sbus->ch[15]);
//    print("@ %d ms\r\n", sbus->timestamp);

    A1->Control(0, 0.0, 0.0, (sbus->ch[0] + 1024.0f) / 2048 * 2*PI, 0.02, 0.1); // zero torque mode
    A1_uart->Write((uint8_t*)(&(A1->send.data)), A1->send_length);

//    set_cursor(0, 0);
//    clear_screen();
//    print("Motor ID: %d\r\n", A1->recv.id);
//    print("Mode    : %d\r\n", A1->recv.mode);
//    print("Temp    : %d\r\n", A1->recv.Temp);
//    print("MError  : %d\r\n", A1->recv.MError);
//    print("Torque  : %.3f\r\n", A1->recv.T);
//    print("Speed   : %.3f\r\n", A1->recv.W);
//    print("Accel   : %d\r\n", A1->recv.Acc);
//    print("Position: %.3f\r\n", A1->recv.Pos);

    osDelay(10);
  }
}
