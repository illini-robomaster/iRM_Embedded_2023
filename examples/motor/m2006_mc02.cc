/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2025 RoboMaster.                                          *
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

/**
 * @brief Motor 2006 stiff position control on DM_MC_02.
 * 'w' = +5 rad, 's' = -5 rad.
 */

#include "bsp_print.h"
#include "bsp_usb.h"
#include "cmsis_os.h"
#include "controller.h"
#include "main.h"
#include "motor.h"

#define RX_SIGNAL (1 << 0)
#define POSITION_STEP 5.0f  // Position step in rad per keypress

extern osThreadId_t defaultTaskHandle;

// For DM_MC_02, bsp::CAN is aliased to bsp::FDCAN via bsp_can_bridge.h
static bsp::CAN* can = nullptr;
static control::MotorCANBase* motor = nullptr;
static bsp::VirtualUSB* usb = nullptr;

// Target position in rad (updated by USB input, starts at 0 = idle)
static volatile float target_position = 0.0f;

/**
 * @brief Custom USB callback class
 */
class CustomUSB : public bsp::VirtualUSB {
 protected:
  void RxCompleteCallback() override final { 
    osThreadFlagsSet(defaultTaskHandle, RX_SIGNAL); 
  }
};

void RM_RTOS_Init() {
  // Initialize USB
  usb = new CustomUSB();
  usb->SetupTx(512);
  usb->SetupRx(512);
  
  // DM_MC_02 uses FDCAN instead of CAN
  can = new bsp::CAN(&hfdcan1, 0);
  
  // Motor 2006 with CAN ID 0x206
  motor = new control::Motor2006(can, 0x206);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  
  control::MotorCANBase* motors[] = {motor};
  char msg[256];
  int len;

  osDelay(1000);  // Wait for USB to initialize

  len = snprintf(msg, sizeof(msg),
                 "w = +5 rad | s = -5 rad\r\n");
  usb->Write((uint8_t*)msg, len);

  // Cascade PID: position (outer) -> velocity (inner)
  // Stiff position PID
  float position_pid_params[3] = {60.0f, 0.0f, 2.0f};
  control::ConstrainedPID position_pid(position_pid_params, 0, 60.0f);

  // Stiff velocity PID
  float velocity_pid_params[3] = {500.0f, 20.0f, 0.0f};
  control::ConstrainedPID velocity_pid(velocity_pid_params, 10000, 30000);

  int print_counter = 0;
  target_position = motor->GetTheta();  // Start at current position to avoid jumps
  while (true) {
    // Check for USB input with short timeout
    uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, 10);

    if (flags & RX_SIGNAL) {
      uint8_t* data;
      uint32_t length = usb->Read(&data);

      if (length > 0) {
        char key = data[0];

        if (key == 'w' || key == 'W') {
          target_position += POSITION_STEP;
        } else if (key == 's' || key == 'S') {
          target_position -= POSITION_STEP;
        }
        len = snprintf(msg, sizeof(msg), "Target: %.2f rad\r\n", target_position);
        usb->Write((uint8_t*)msg, len);
      }
    }

    // Cascade PID
    float position_error = motor->GetThetaDelta(target_position);
    float ref_velocity = position_pid.ComputeOutput(position_error);
    float velocity_error = motor->GetOmegaDelta(ref_velocity);
    int16_t output = static_cast<int16_t>(velocity_pid.ComputeConstrainedOutput(velocity_error));

    motor->SetOutput(output);
    control::MotorCANBase::TransmitOutput(motors, 1);

    // Print status every 500ms
    if (++print_counter >= 250) {
      print_counter = 0;
      len = snprintf(msg, sizeof(msg), "Tgt: %6.2f | Pos: %6.2f | Out: %5d\r\n",
                     target_position, motor->GetTheta(), output);
      usb->Write((uint8_t*)msg, len);
    }
  }

  osDelay(2);  // Motor control at 500Hz
}
