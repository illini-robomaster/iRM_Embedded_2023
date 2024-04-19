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

#include "bsp_print.h"
#include "bsp_os.h"
#include "bsp_gpio.h"
#include "cmsis_os2.h"
#include "main.h"

#include "cybergear.h"

#define MOTOR_BASE_ID 127
#define MOTOR_ARM1_ID 126
#define MOTOR_ARM2_ID 125

#define MOTOR_WEIGHT 0.317
#define ARM_WEIGHT 0.1
#define ARM_LENGTH 0.13
#define GRAVITY 9.8

#include <Eigen/Dense>

namespace sym {

/**
 * This function was autogenerated. Do not modify by hand.
 *
 * Args:
 *     theta2: Scalar
 *     theta3: Scalar
 *     m_motor: Scalar
 *     m_arm: Scalar
 *     g: Scalar
 *     l: Scalar
 *
 * Outputs:
 *     t2: Scalar
 *     t3: Scalar
 */
template <typename Scalar>
void Forward(const Scalar theta2, const Scalar theta3, const Scalar m_motor, const Scalar m_arm,
             const Scalar g, const Scalar l, Scalar* const t2 = nullptr,
             Scalar* const t3 = nullptr) {
  // Total ops: 27

  // Input arrays

  // Intermediate terms (7)
  const Scalar _tmp0 = std::sin(theta2);
  const Scalar _tmp1 = _tmp0 * l;
  const Scalar _tmp2 = g * m_motor;
  const Scalar _tmp3 = g * m_arm;
  const Scalar _tmp4 = l * (_tmp0 * std::cos(theta3) - std::sin(theta3) * std::cos(theta2));
  const Scalar _tmp5 = (Scalar(1) / Scalar(2)) * _tmp4;
  const Scalar _tmp6 = -_tmp1;

  // Output terms (2)
  if (t2 != nullptr) {
    Scalar& _t2 = (*t2);

    _t2 = _tmp1 * _tmp2 + (Scalar(1) / Scalar(2)) * _tmp1 * _tmp3 - _tmp2 * (-_tmp4 + _tmp6) -
          _tmp3 * (-_tmp5 + _tmp6);
  }

  if (t3 != nullptr) {
    Scalar& _t3 = (*t3);

    _t3 = -_tmp2 * _tmp4 - _tmp3 * _tmp5;
  }
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym

osTimerId_t controlTimer;
const osTimerAttr_t controlTimerAttribute = {
  .name = "controlTimer",
  .attr_bits = 0,
  .cb_mem = nullptr,
  .cb_size = 0,
};

static xiaomi::CAN* can = nullptr;
//static xiaomi::CyberGear* motor_base = nullptr;
static xiaomi::CyberGear* motor_arm1 = nullptr;
static xiaomi::CyberGear* motor_arm2 = nullptr;
static bsp::GPIO* key = nullptr;

void controlTask(void* arg) {
  UNUSED(arg);

  float theta2 = motor_arm1->GetAngle();
  float theta3 = motor_arm2->GetAngle();
  float t2, t3;

  sym::Forward<float>(theta2, theta3, MOTOR_WEIGHT, ARM_WEIGHT, GRAVITY, ARM_LENGTH, &t2, &t3);

  print("t2: %.4f t3: %.4f\r\n", t2, t3);

  motor_arm1->SendMotionCommand(-t2, 0, 0, 0, 0);
  motor_arm2->SendMotionCommand(-t3, 0, 0, 0, 0);
}

//==================================================================================================
// RM Init
//==================================================================================================

void RM_RTOS_Init(void) {
  print_use_usb();

  bsp::SetHighresClockTimer(&htim5);

  key = new bsp::GPIO(KEY_GPIO_Port, KEY_Pin);
  can = new xiaomi::CAN(&hcan1);
  //motor_base = new xiaomi::CyberGear(can, MOTOR_BASE_ID, xiaomi::Motion_mode);
  motor_arm1 = new xiaomi::CyberGear(can, MOTOR_ARM1_ID, xiaomi::Motion_mode);
  motor_arm2 = new xiaomi::CyberGear(can, MOTOR_ARM2_ID, xiaomi::Motion_mode);
}

void RM_RTOS_Timers_Init(void) {
  controlTimer = osTimerNew(controlTask, osTimerPeriodic, nullptr, &controlTimerAttribute);
}

//==================================================================================================
// RM Default Task
//==================================================================================================

void RM_RTOS_Default_Task(const void* arg) {
  UNUSED(arg);

  //motor_base->SetZeroPosition();
  motor_arm1->SetZeroPosition();
  motor_arm2->SetZeroPosition();

  osDelay(1000);

  osTimerStart(controlTimer, 10U);

  while (true) {
    osDelay(10);
  }
}

//==================================================================================================
// END
//==================================================================================================