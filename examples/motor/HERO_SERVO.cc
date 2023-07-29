#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "motor.h"



uint8_t PWM_CHANNEL = 1;
uint32_t TIM_CLOCK_FREQ = 2000000;
// Magic Number
uint32_t MOTOR_OUT_FREQ = 500;
// Magic number
uint32_t PULSE_WIDTH = 1500;
// PULSE_WIDTH when servo is idle

bsp::GPIO* key = nullptr;

#define KEY_GPIO_GROUP GPIOA
#define KEY_GPIO_PIN GPIO_PIN_0

control::PDIHV* motor1;

void RM_RTOS_Init(){
  print_use_uart(&huart1);
  key = new bsp::GPIO(KEY_GPIO_GROUP, KEY_GPIO_PIN);
  motor1 = new control::PDIHV(&htim1, PWM_CHANNEL,TIM_CLOCK_FREQ,MOTOR_OUT_FREQ, PULSE_WIDTH);
  motor1->SetOutput(1500);
  osDelay(300);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  float angle = 0.0;
  int16_t power = 2000;
  while(true){
//    motor1->SetOutPutAngle(angle);
    motor1->SetOutput(power);
    if(!key->Read()){
      angle += 10.0;
      power += 10;
    }
//    power += 10;
//    osDelay(1000);
//    angle += 1.0;
    print("angle: %f\r\n", angle);
    print("power: %d\r\n", power);


  }
  }
