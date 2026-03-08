
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

 #include <memory>

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

#define CLAW_PWM_CHANNEL 4 // Pin PH10
#define CLAW_ROTATE_PWM_CHANNEL 3 // Pin PH11
#define ARM_ROLL_PWM_CHANNEL 2 // Pin PH12

#define TIM_CLOCK_FREQ 84000000 // Using TIM5
#define SERVO_OUT_FREQ 333

#define MAX_IOUT2060 10000
#define MAX_OUT 60000

#define RX_SIGNAL (1 << 0)

extern osThreadId_t defaultTaskHandle;

bsp::GPIO* key = nullptr;
control::MotorPWMBase* arm_claw = nullptr;
control::MotorPWMBase* arm_claw_rotate = nullptr;
//control::MotorCANBase* arm_slide = nullptr;
control::MotorPWMBase* arm_roll = nullptr;

BoolEdgeDetector control_inputs[9] = {BoolEdgeDetector(false), BoolEdgeDetector(false), BoolEdgeDetector(false), BoolEdgeDetector(false), BoolEdgeDetector(false), BoolEdgeDetector(false), BoolEdgeDetector(false), BoolEdgeDetector(false), BoolEdgeDetector(false)};

static remote::DBUS *dbus = nullptr;

static bsp::CAN* can1 = nullptr;

float Kp = 50;
float Ki = 15;
float Kd = 65;
float diff_slide_output = 0;

class CustomUART : public bsp::UART {
  public:
   using bsp::UART::UART;
 
  protected:
   /* notify application when rx data is pending read */
   void RxCompleteCallback() override final { osThreadFlagsSet(defaultTaskHandle, RX_SIGNAL); }
 };

void RM_RTOS_Init(){
  print_use_usb();

  can1 = new bsp::CAN(&hcan1);

  key = new bsp::GPIO(KEY_GPIO_GROUP, KEY_GPIO_PIN);
  arm_claw = new control::MotorPWMBase(&htim4, CLAW_PWM_CHANNEL, TIM_CLOCK_FREQ, SERVO_OUT_FREQ, 0);
  arm_claw_rotate = new control::MotorPWMBase(&htim4, CLAW_ROTATE_PWM_CHANNEL, TIM_CLOCK_FREQ, SERVO_OUT_FREQ, 0);
  arm_roll = new control::MotorPWMBase(&htim4, ARM_ROLL_PWM_CHANNEL, TIM_CLOCK_FREQ, SERVO_OUT_FREQ, 0);

//arm_slide = new control::Motor2006(can1, 0x202);

  dbus = new remote::DBUS(&huart3);
}

void RM_RTOS_Default_Task(const void* args){
  UNUSED(args);
  //int slide_output = 0;
  int16_t arm_roll_output = 1500;
  int16_t arm_claw_rotate_output = 1500;
  int16_t arm_claw_output = 1500;

  /*control::MotorCANBase* arm[] = {arm_slide};
  float diff_slide = 0;
  float param[] = {Kp, Ki, Kd};
  control::ConstrainedPID pid(param, MAX_IOUT2060, MAX_OUT);*/

  uint32_t length;
  uint8_t* data;

  auto uart = std::make_unique<CustomUART>(&huart8);
  uart->SetupRx(50);
  uart->SetupTx(50);

  

  while(1){
    /*
    const char buf[] = "alive\n";

    usb_printf(buf, *uart);
    */

    /* wait until rx data is available */
    // uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
    if (1) {  // unnecessary check
      /* time the non-blocking rx / tx calls (should be <= 1 osTick) */
      length = uart->Read(&data);
      uart->Write(data, length);

      control_inputs[0].input(*data == 'z'); // Claw Close
      control_inputs[1].input(*data == 'x'); // Claw Open
      control_inputs[2].input(*data == 'q'); // Claw Rotate Left
      control_inputs[3].input(*data == 'e'); // Claw Rotate Right
      control_inputs[4].input(*data == 'a'); // Arm Rotate Left
      control_inputs[5].input(*data == 'd'); // Arm Rotate Right
      control_inputs[6].input(*data == 'w'); // Slide Forward
      control_inputs[7].input(*data == 's'); // Slide Backward
      control_inputs[8].input(*data == 'p'); // Reset

      *data = '\0';
    }

    if (control_inputs[0].posEdge()) {
      arm_claw_output += 50;
    } else if (control_inputs[1].posEdge()) {
      arm_claw_output -= 50;
    }

    if (control_inputs[2].posEdge()) {
      arm_claw_rotate_output += 50;
    } else if (control_inputs[3].posEdge()) {
      arm_claw_rotate_output -= 50;
    }

    if (control_inputs[4].posEdge()) {
      arm_roll_output += 50;
    } else if (control_inputs[5].posEdge()) {
      arm_roll_output -= 50;
    }

    if (control_inputs[6].posEdge()) {
      // arm _slide plus
    } else if (control_inputs[7].posEdge()) {
      // arm_slide minus
    }

    if (control_inputs[8].posEdge()) {
      arm_claw_output = 1500;
      arm_claw_rotate_output = 1500;
      arm_roll_output = 1500;
      // arm_slide set
    }
    

    /*
    slide_output = MAP_RANGE(dbus->ch0, -660, 660,-50, 50);

    diff_slide = arm_slide->GetOmegaDelta(slide_output);
    diff_slide_output = pid.ComputeConstrainedOutput(diff_slide);
    arm_slide->SetOutput(slide_output);
    control::MotorCANBase::TransmitOutput(arm, 1);
    */

    // print("slide speed: %d , diff_slide: %.2f, slide_output: %f \r\n", slide_output, diff_slide, diff_slide_output);

    arm_claw->SetOutput(arm_claw_output);
    // arm_roll->SetOutput(arm_roll_output);
    // arm_claw_rotate->SetOutput(arm_claw_rotate_output);

    // print("Arm Claw: %d\r\n", arm_claw_output);
    print("Arm Claw Rotate: %d\r\n", arm_claw_rotate_output);
    // print("Arm Roll: %d\r\n", arm_roll_output);
    osDelay(10);
  }
}