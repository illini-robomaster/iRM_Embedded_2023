#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "motor.h"
#include "controller.h"
#include "can.h"

/**
 * @brief when installing, please use this example to set angle to 0.0 degree for calibration.
 * The current program is functioning, however, might not be 100% accurate
 */

// All of these following parameters are tuned for this servo.
control::MotorCANBase *motor1 = nullptr;
control::ServoMotor *servo1 = nullptr;
bsp::CAN *can1 = nullptr;

bsp::GPIO *key = nullptr;

void RM_RTOS_Init(){
  can1 = new bsp::CAN(&hcan1, true);
  motor1 = new control::Motor2006(can1, 0x201);
  control::servo_t servo_data;
  servo_data.motor = motor1;
  servo_data.max_speed = 6 * PI;
  servo_data.max_acceleration = 200 * PI;
  servo_data.transmission_ratio = M2006P36_RATIO;
  servo_data.omega_pid_param = new float[3]{150, 4, 0};
  servo_data.max_iout = 2000;
  servo_data.max_out = 10000;
  servo1 = new control::ServoMotor(servo_data);

  key = new bsp::GPIO(GPIOB, GPIO_PIN_2);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  BoolEdgeDetector key_detector(false);
  while (true) {
    key_detector.input(!key->Read());
    if (key_detector.posEdge()) {
      print("Key pressed, start calibration\r\n");
      servo1->SetTarget(servo1->GetTheta() + 2 * PI);
      servo1->CalcOutput();
      control::MotorCANBase::TransmitOutput(&motor1, 1);
//        osDelay(1000);
      osDelay(10);
      print("Calibration finished\r\n");
    }
    osDelay(10);


  }
}