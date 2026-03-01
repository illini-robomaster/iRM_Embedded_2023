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

#include <cstring>

#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "weighscale.h"

// CAN bus instance (using CAN2)
static bsp::CAN* can2 = nullptr;

// Weighing scale instance
static control::WeighScale* scale = nullptr;

// Button for manual tare (K1 on TypeA board)
static bsp::GPIO* key = nullptr;

// Number of channels
static const uint8_t NUM_CHANNELS = 4;

void RM_RTOS_Init(void) {
  // Initialize print output via USB
  print_use_usb();

  // Initialize CAN2
  can2 = new bsp::CAN(&hcan2, false);

  // Initialize weighing scale with address 1, standard frame, using CAN2
  // Constructor automatically registers CAN callbacks for weight responses
  scale = new control::WeighScale(can2, 1, control::WeighScaleFrameType::STANDARD, NUM_CHANNELS);

  // Initialize button (K1 on TypeA: GPIOE PIN6)
  key = new bsp::GPIO(K1_GPIO_Port, K1_Pin);
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);
  
  osDelay(500);  // Wait for system to stabilize

  print("=== WeighScale Test ===\r\n");
  print("CAN2, Address: 1\r\n");
  print("Channels: %d\r\n", NUM_CHANNELS);
  print("Press K1 to Tare\r\n");
  print("=======================\r\n\r\n");

  // Initial tare
  print("Initial Tare...\r\n");
  scale->Tare(control::WEIGHSCALE_ALL_CHANNELS);
  osDelay(500);

  uint32_t loop_count = 0;
  bool last_key_state = false;

  while (true) {
    // Check button press for manual tare
    bool key_pressed = (key->Read() == 0);  // Active low
    if (key_pressed && !last_key_state) {
      print(">>> Manual Tare (all channels)...\r\n");
      scale->Tare(control::WEIGHSCALE_ALL_CHANNELS);
      osDelay(200);
    }
    last_key_state = key_pressed;

    // Send ReadWeights request (responses handled by internal callbacks)
    control::WeighScaleData_t temp_data;
    print("[%lu] TX ReadWeights -> ID=0x301\r\n", loop_count);
    scale->ReadWeights(&temp_data, NUM_CHANNELS);

    // Wait for responses (300ms window for multiple frames)
    osDelay(300);

    // Print all received raw frames from internal buffer
    uint8_t rx_frame_count = scale->GetRxFrameCount();
    print("       RX frames (%d):\r\n", rx_frame_count);
    control::WeighScaleRxFrame_t frame;
    while (scale->GetRxFrame(&frame)) {
      print("         ID=0x%03X [%02X %02X %02X %02X %02X %02X %02X %02X]\r\n",
            frame.id,
            frame.data[0], frame.data[1],
            frame.data[2], frame.data[3],
            frame.data[4], frame.data[5],
            frame.data[6], frame.data[7]);
    }

    // Get parsed weight data from the scale object
    const control::WeighScaleData_t& weight_data = scale->GetData();

    // Print parsed weight values
    print("\r\n       Weights (parsed):\r\n");
    for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
      print("         CH%d: %.3f kg\r\n", ch + 1, weight_data.weight[ch] / 1000.0);
    }

    print("\r\n");

    // Wait before next reading
    osDelay(500);
    loop_count++;
  }
}
