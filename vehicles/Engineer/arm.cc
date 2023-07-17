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

#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "dbus.h"
#include "main.h"
#include "motor.h"
#include "utils.h"

#define KEY_GPIO_GROUP GPIOA
#define KEY_GPIO_PIN GPIO_PIN_0


// Motor enable list
#define BASE_TRANSLATE_MOTOR_ENABLE
#define FOREARM_ROTATE_MOTOR_ENABLE

constexpr float RUN_SPEED = (2 * PI);
constexpr float ALIGN_SPEED = (0.5 * PI);
constexpr float ACCELERATION = (100 * PI);

bsp::CAN* can1 = nullptr;

#ifdef BASE_TRANSLATE_MOTOR_ENABLE
static int base_translate_motor_enable = 1;
#else
static int base_translate_motor_enable = 0;
#endif

#ifdef FOREARM_ROTATE_MOTOR_ENABLE
static int forearm_rotate_motor_enable = 1;
#else
static int forearm_rotate_motor_enable = 0;
#endif

// base translate motor params
#ifdef BASE_TRANSLATE_MOTOR_ENABLE

control::MotorCANBase* motor1 = nullptr;
control::SteeringMotor* base_translate_motor = nullptr;

bsp::GPIO* key = nullptr;
bsp::GPIO* base_translate_pe_sensor = nullptr;

BoolEdgeDetector key_detector(false);

bool base_translate_align_detect() {
  return base_translate_pe_sensor->Read() == 1;
}
#endif

#ifdef FOREARM_ROTATE_MOTOR_ENABLE
constexpr float M4310_VEL = 4.0; // magic number, see m4310_mit example
static control::Motor4310* forearm_rotate_motor = nullptr;
#endif

void RM_RTOS_Init() {
  print_use_uart(&huart1);
  bsp::SetHighresClockTimer(&htim5);

  /* Usage:
   *   The 'key' is the white button on TypeC boards
  **/
  key = new bsp::GPIO(KEY_GPIO_GROUP, KEY_GPIO_PIN);

  control::steering_t steering_data;

  can1 = new bsp::CAN(&hcan1, 0x201, true);

  #ifdef BASE_TRANSLATE_MOTOR_ENABLE
  motor1 = new control::Motor3508(can1, 0x201);

  // TODO assign a GPIO pin to the PE sensor
  base_translate_pe_sensor = new bsp::GPIO(GPIOC, GPIO_PIN_2);

  steering_data.motor = motor1;
  steering_data.max_speed = RUN_SPEED;
  steering_data.max_acceleration = ACCELERATION;
  // TODO make sure the gear ratio is correct
  steering_data.transmission_ratio = M3508P19_RATIO;
  steering_data.omega_pid_param = new float[3]{140, 1.2, 25};
  steering_data.max_iout = 1000;
  steering_data.max_out = 13000;
  // TODO measure the calibrate offset for base translate motor
  steering_data.calibrate_offset = 0;
  #endif

  #ifdef FOREARM_ROTATE_MOTOR_ENABLE
  static const int RX_ID = 0x02;
  static const int TX_ID = 0x01;

  /* rx_id = Master id
   * tx_id = CAN id
   * see example/m4310_mit.cc
   */
  forearm_rotate_motor = new control::Motor4310(can1, RX_ID, TX_ID, control::MIT);
  #endif

}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);

  // 4310 doesn't follow this control pipeline
  control::MotorCANBase* motors[] = {
    #ifdef BASE_TRANSLATE_MOTOR_ENABLE
    motor1,
    #endif
  };
  int motor_num = base_translate_motor_enable;

  // Press Key to start aligning. Else sudden current change when power is switched on might break
  // the board.
  while(!key->Read());

  // wait for release because align_detect also is key press here
  while(key->Read());

  if (base_translate_motor_enable) {
    print("base translate motor enabled\n");
  }

  if (forearm_rotate_motor_enable) {
    print("forearm rotate motor enabled\n");
  }

  print("total motor num = %d\n", motor_num + forearm_rotate_motor_enable);

  // base translate motor calibrate
  #ifdef BASE_TRANSLATE_MOTOR_ENABLE
  base_translate_motor->SetMaxSpeed(ALIGN_SPEED);

  // Don't put Calibrate() in the while condition
  bool base_translate_alignment_complete = false;
  while (!base_translate_alignment_complete) {
    base_translate_motor->CalcOutput();
    control::MotorCANBase::TransmitOutput(motors, motor_num);
    base_translate_alignment_complete = base_translate_motor->Calibrate();
    osDelay(2);
  }

  base_translate_motor->ReAlign();
  //base_translate_motor->SetMaxSpeed(RUN_SPEED);
  print("base translate motor alignment complete\n");
  #endif

  // forearm rotate motor calibrate
  #ifdef FOREARM_ROTATE_MOTOR_ENABLE
  // TODO: config zero pos with 4310 config assist
  forearm_rotate_motor->SetZeroPos(forearm_rotate_motor);
  forearm_rotate_motor->MotorEnable(forearm_rotate_motor);
  #endif

  if (motor_num >= 1) {
    control::MotorCANBase::TransmitOutput(motors, motor_num);
  }

  // Wait for key to release
  osDelay(500);

  int dir = 1;

  while (motor_num >= 1) {
    key_detector.input(key->Read());
    if (key_detector.posEdge()){
      if (dir == 1) {
        #ifdef BASE_TRANSLATE_MOTOR_ENABLE
        base_translate_motor->TurnRelative(PI / 4);
        #endif

        #ifdef FOREARM_ROTATE_MOTOR_ENABLE
        forearm_rotate_motor->SetOutput(PI / 4, M4310_VEL, 30, 0.5, 0);
        #endif

      } else {
        #ifdef BASE_TRANSLATE_MOTOR_ENABLE
        base_translate_motor->ReAlign();
        #endif

        #ifdef FOREARM_ROTATE_MOTOR_ENABLE
        forearm_rotate_motor->SetOutput(0, M4310_VEL, 30, 0.5, 0);
        #endif
     }
      dir *= -1;
    }

    #ifdef BASE_TRANSLATE_MOTOR_ENABLE
    base_translate_motor->CalcOutput();
    #endif

    #ifdef FOREARM_ROTATE_MOTOR_ENABLE
    forearm_rotate_motor->TransmitOutput(forearm_rotate_motor);
    #endif

    control::MotorCANBase::TransmitOutput(motors, motor_num);
    osDelay(2);
  }

}

