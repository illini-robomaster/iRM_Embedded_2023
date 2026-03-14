
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

#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "dbus.h"
#include "main.h"
#include "math.h"
#include "motor.h"
#include "utils.h"

#define KEY_GPIO_GROUP GPIOA
#define KEY_GPIO_PIN GPIO_PIN_0


#define DEFAULT_TASK_DELAY 100

#define CLAW_PWM_CHANNEL 4         // Pin PD15 = TIM4_CH4
#define CLAW_ROTATE_PWM_CHANNEL 3  // Pin PD14 = TIM4_CH3
#define ARM_ROLL_PWM_CHANNEL 2     // Pin PD13 = TIM4_CH2

#define TIM_CLOCK_FREQ 1000000  // Using TIM4 (prescaler=83 → counter at 1 MHz)
#define SERVO_OUT_FREQ 333

#define MAX_IOUT2006 10000
#define MAX_OUT 10000

bsp::GPIO* key = nullptr;
control::MotorPWMBase* arm_claw = nullptr;
control::MotorPWMBase* arm_claw_rotate = nullptr;
control::Motor2006* arm_slide_motor = nullptr;
control::ServoMotor* arm_slide = nullptr;
control::MotorPWMBase* arm_roll = nullptr;

BoolEdgeDetector control_inputs[9] = {BoolEdgeDetector(false), BoolEdgeDetector(false), BoolEdgeDetector(false), BoolEdgeDetector(false), BoolEdgeDetector(false), BoolEdgeDetector(false), BoolEdgeDetector(false), BoolEdgeDetector(false), BoolEdgeDetector(false)};

// static remote::DBUS *dbus = nullptr;

static bsp::CAN* can1 = nullptr;

void RM_RTOS_Init() {
  print_use_uart_rxtx(&huart8);  // huart8: TX for print output, RX for key input

  can1 = new bsp::CAN(&hcan1);

  key = new bsp::GPIO(KEY_GPIO_GROUP, KEY_GPIO_PIN);
  arm_claw = new control::MotorPWMBase(&htim4, CLAW_PWM_CHANNEL, TIM_CLOCK_FREQ, SERVO_OUT_FREQ, 0);
  arm_claw_rotate = new control::MotorPWMBase(&htim4, CLAW_ROTATE_PWM_CHANNEL, TIM_CLOCK_FREQ, SERVO_OUT_FREQ, 0);
  arm_roll = new control::MotorPWMBase(&htim4, ARM_ROLL_PWM_CHANNEL, TIM_CLOCK_FREQ, SERVO_OUT_FREQ, 0);

  arm_slide_motor = new control::Motor2006(can1, 0x203);
  float omega_pid_params[3] = {0.0f, 0.0f, 0.0f};  // unused in direct PD mode (pos_kp > 0)
  control::servo_t slide_servo = {
      .motor = arm_slide_motor,
      .max_speed = 5.0f,  // output-shaft rad/s — caps travel speed via P-term clamping
      .max_acceleration = 50.0f,
      .transmission_ratio = 36.0f,  // M2006P36 gear ratio
      .omega_pid_param = omega_pid_params,
      .max_iout = MAX_IOUT2006,
      .max_out = MAX_OUT,
      .omega_lpf_alpha = 0.5f,
      .pos_kp = 80000.0f,  // full torque at 0.125 rad (~7°) position error
      .pos_kd = 2000.0f,   // damping: ~1400 counts at max output speed (~0.7 rad/s)
  };
  // align_angle=-1 → auto-latch on first CAN packet
  arm_slide = new control::ServoMotor(slide_servo, -1);

  // dbus = new remote::DBUS(&huart3);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  // int slide_output = 0;
  int16_t arm_roll_output = 800;
  int16_t arm_claw_rotate_output = 1500;
  int16_t arm_claw_output = 1000;

  control::MotorCANBase* arm[] = {arm_slide_motor};

  // Wait for first CAN feedback so GetTheta() returns the real position
  osDelay(100);
  float slide_target = arm_slide->GetTheta();  // lock onto starting position
  arm_slide->SetTarget(slide_target);          // arm servo to hold start position

  uint32_t length;
  uint8_t* data;

  while (1) {
    /*
    const char buf[] = "alive\n";

    usb_printf(buf, *uart);
    */

    /* wait until rx data is available */
    // uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (1) {  // unnecessary check
      /* time the non-blocking rx / tx calls (should be <= 1 osTick) */
      length = print_uart_read(&data);

      control_inputs[0].input(length > 0 && *data == 'z');  // Claw Close
      control_inputs[1].input(length > 0 && *data == 'x');  // Claw Open
      control_inputs[2].input(length > 0 && *data == 'q');  // Claw Rotate Left
      control_inputs[3].input(length > 0 && *data == 'e');  // Claw Rotate Right
      control_inputs[4].input(length > 0 && *data == 'a');  // Arm Rotate Left
      control_inputs[5].input(length > 0 && *data == 'd');  // Arm Rotate Right
      control_inputs[6].input(length > 0 && *data == 'w');  // Slide Forward
      control_inputs[7].input(length > 0 && *data == 's');  // Slide Backward
      control_inputs[8].input(length > 0 && *data == 'p');  // Reset
    }

    if (control_inputs[0].posEdge()) {
      arm_claw_output += 50;
    } else if (control_inputs[1].posEdge()) {
      arm_claw_output -= 50;
    }

    if (control_inputs[2].posEdge()) {
      arm_claw_rotate_output += 10;
    } else if (control_inputs[3].posEdge()) {
      arm_claw_rotate_output -= 10;
    }

    if (control_inputs[4].posEdge()) {
      arm_roll_output += 10;
    } else if (control_inputs[5].posEdge()) {
      arm_roll_output -= 10;
    }

    if (control_inputs[6].posEdge()) {
      // slide_target += 0.1f;
      // This Go Up
      slide_target = -6.7f;  // temporary hard limit to prevent hitting the wall
      arm_slide->SetTarget(slide_target, true);
    } else if (control_inputs[7].posEdge()) {
      // slide_target -= 0.1f;
      // This Go Down
      slide_target = -0.6f;  // temporary hard limit to prevent hitting the
      arm_slide->SetTarget(slide_target, true);
    }

    if (control_inputs[8].posEdge()) {
      arm_claw_output = 1500;
      arm_claw_rotate_output = 1500;
      arm_roll_output = 800;
      // arm_slide set
    }

    arm_slide->CalcOutput();
    control::MotorCANBase::TransmitOutput(arm, 1);

    arm_claw_output = clip<int16_t>(arm_claw_output, 500, 2500);
    arm_claw_rotate_output = clip<int16_t>(arm_claw_rotate_output, 500, 2500);
    arm_roll_output = clip<int16_t>(arm_roll_output, 500, 2500);
    arm_claw->SetOutput(arm_claw_output);
    arm_claw_rotate->SetOutput(arm_claw_rotate_output);
    arm_roll->SetOutput(arm_roll_output);
    set_cursor(0,0);
    clear_screen();
    print("Arm Claw: %d\r\n", arm_claw_output);
    print("Arm Claw Rotate: %d\r\n", arm_claw_rotate_output);
    print("Arm Roll: %d\r\n", arm_roll_output);
    float slide_pos_err = slide_target - arm_slide->GetTheta();
    print("Slide: theta=%.3f tgt=%.3f err=%.3f vel=%.3f\r\n",
          arm_slide->GetTheta(), slide_target, slide_pos_err, arm_slide->GetOmega());
    osDelay(10);
  }
}