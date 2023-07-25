#include <analogWrite.h>

const int servoPin = A3;
const int pwmPin = A4;

void setup() {
  Serial.begin(9600);

  pinMode(pwmPin, OUTPUT);
  ledcSetup(8, 50, 16);
  ledcAttachPin(A4, 8);
}

int calculatePWM(int degree) {
  const float deadZone = 2880;
  const float max = 7080;

  return (int)(((max -  deadZone) / 360 * degree + deadZone));
}

// [16] 2880, 4940~5020, 7080
int angle = 0;

void loop() {
  angle = analogRead(servoPin);
  Serial.println(angle / 4096.0 * 360);
  ledcWrite(8, calculatePWM(angle / 4096.0 * 360));

  // int pwm0 = 2880;
  // Serial.printf("PWM0: %d\n", pwm0);
  // ledcWrite(8, pwm0);
  // delay(1000);

  // int pwm1 = 5020;
  // Serial.printf("PWM1: %d\n", pwm1);
  // ledcWrite(8, pwm1);
  // delay(1000);

  // int pwm2 = 7080;
  // Serial.printf("PWM2: %d\n", pwm2);
  // ledcWrite(8, pwm2);
  delay(10);
}


