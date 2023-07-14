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

#include <cmath>

#include "bsp_gpio.h"
#include "bsp_imu.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "pose.h"

#define ONBOARD_IMU_SPI hspi5
#define ONBOARD_IMU_CS_GROUP GPIOF
#define ONBOARD_IMU_CS_PIN GPIO_PIN_6
#define PRING_UART huart8

static bsp::MPU6500* imu;

void RM_RTOS_Init(void) {
  bsp::SetHighresClockTimer(&htim2);
  print_use_uart(&PRING_UART);
}

void RM_RTOS_Default_Task(const void* arguments) {
  UNUSED(arguments);

  // init IMU instance
  bsp::GPIO chip_select(ONBOARD_IMU_CS_GROUP, ONBOARD_IMU_CS_PIN);
  imu = new bsp::MPU6500(&ONBOARD_IMU_SPI, chip_select, MPU6500_IT_Pin);
  osDelay(3000);
  print("IMU Initialized!\r\n");

  // init pose estimator instance
  control::Pose poseEstimator(imu);

  // Set alpha for the complementary filter in the pose estimator
  poseEstimator.SetAlpha(0.95);

  // init params
  float roll = 0;
  float pitch = 0;
  float yaw = 0;
  uint32_t i = 0;
  float degRoll = 0;
  float degPitch = 0;
  float degYaw = 0;
  // calibrate the Offset for IMU acce meter and gyro
  poseEstimator.Calibrate();

  // reset timer and pose for IMU
  poseEstimator.PoseInit();

  /* NOTE: IMU SHOULD BE PLACED ON A FLAT PLANE WHILE RUNNING THE FUNCTIONS ABOVE */

  while (true) {
    // update estimated pose with complementary filter
    poseEstimator.ComplementaryFilterUpdate();
    osDelay(10);

    // print pose every 200ms
    i += 1;
    if (i >= 20) {
      set_cursor(0, 0);
      clear_screen();
      // get roll and pitch, in RAD
      roll = poseEstimator.GetRoll();
      pitch = poseEstimator.GetPitch();
      yaw = poseEstimator.GetYaw();
      print("PITCH: %6.4f ROLL: %6.4f \r\n", pitch, roll);
      print("YAW: %6.4f \r\n", yaw);
      // convert to DEG
      degRoll = roll * 180 / 3.14;
      degPitch = pitch * 180 / 3.14;
      degYaw = yaw * 180 / 3.14;
      print("P_DEG: %6.4f R_DEG: %6.4f \r\n", degPitch, degRoll);
      print("Y_DEG: %6.4f \r\n", degYaw);

      i = 0;
    }
  }
}
