#include <analogWrite.h>

const int testPin = A4;

void setup() {
  pinMode(testPin, OUTPUT);
  Serial.begin(9600);
  ledcSetup(8, 50, 16);
  ledcAttachPin(A4, 8);
}

int calculatePWM(int degree) {
  const float deadZone = 0.5 / 20 * 4096;
  const float max = 2.5 / 20 * 4096;

  return (int)(((max -  deadZone) / 180 * degree + deadZone));
}

// [12] 180, 311, 443
// [16] 2880, 4940~5020, 7080

void loop() {
  // int pwm0 = 2880;
  // Serial.printf("PWM0: %d\n", pwm0);
  // ledcWrite(8, pwm0);
  // delay(1000);

  // int pwm1 = 5020;
  // Serial.printf("PWM1: %d\n", pwm1);
  // ledcWrite(8, pwm1);
  // delay(1000);

  int pwm2 = 7080;
  Serial.printf("PWM2: %d\n", pwm2);
  ledcWrite(8, pwm2);
  delay(1000);
}