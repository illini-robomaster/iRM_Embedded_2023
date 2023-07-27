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

#include "bsp_print.h"
#include "bsp_usb.h"
#include "cmsis_os.h"

/**
 * sample client Python code to verify transmission correctness of this example program
 *
 * ```
 * import serial
 *
 * ser = serial.Serial('/dev/ttyUSB0', baudrate=115200)
 * some_str = 'this is my data!'
 *
 * for i in range(1000):
 *     ser.write(some_str)
 *     while ser.in_waiting < 3 * len(some_str):
 *         continue
 *     ret = ser.read_all()
 *     assert ret == 3 * some_str
 * ```
 */

#define RX_SIGNAL (1 << 0)

extern osThreadId_t defaultTaskHandle;

static bsp::VirtualUSB* usb;

class CustomUSBCallback : public bsp::VirtualUSB {
 protected:
  void RxCompleteCallback() override final { osThreadFlagsSet(defaultTaskHandle, RX_SIGNAL); }
};

void RM_RTOS_Init(void) {
  usb = new CustomUSBCallback();
  usb->SetupTx(2048);
  usb->SetupRx(2048);
}

void RM_RTOS_Default_Task(const void* argument) {
  uint32_t length;
  uint8_t* data;

  UNUSED(argument);

  while (true) {
    /* wait until rx data is available */
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (flags & RX_SIGNAL) {  // unnecessary check
      length = usb->Read(&data);
      usb->Write(data, length);
      usb->Write(data, length);
      usb->Write(data, length);
    }
  }
}
