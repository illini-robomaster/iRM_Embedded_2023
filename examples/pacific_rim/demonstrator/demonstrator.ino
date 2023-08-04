#include <Wire.h>
#include <DFRobot_ADS1115.h>

DFRobot_ADS1115 ads1(&Wire);
DFRobot_ADS1115 ads2(&Wire);

const int pwm0Pin = D9;
const int pwm1Pin = D7;
const int pwm2Pin = D6;
const int pwm3Pin = D5;
const int pwm4Pin = D3;
const int pwm5Pin = D2;

void setup() {
  Serial.begin(115200);

  pinMode(pwm0Pin, OUTPUT);
  pinMode(pwm1Pin, OUTPUT);
  pinMode(pwm2Pin, OUTPUT);
  pinMode(pwm3Pin, OUTPUT);
  pinMode(pwm4Pin, OUTPUT);
  pinMode(pwm5Pin, OUTPUT);
  ledcSetup(0, 50, 16);
  ledcSetup(1, 50, 16);
  ledcSetup(2, 50, 16);
  ledcSetup(3, 50, 16);
  ledcSetup(4, 50, 16);
  ledcSetup(5, 50, 16);
  ledcAttachPin(pwm0Pin, 0);
  ledcAttachPin(pwm1Pin, 1);
  ledcAttachPin(pwm2Pin, 2);
  ledcAttachPin(pwm3Pin, 3);
  ledcAttachPin(pwm4Pin, 4);
  ledcAttachPin(pwm5Pin, 5);

  ads1.setAddr_ADS1115(ADS1115_IIC_ADDRESS0);
  ads1.setGain(eGAIN_TWOTHIRDS);
  ads1.setMode(eMODE_SINGLE);
  ads1.setRate(eRATE_128);
  ads1.setOSMode(eOSMODE_SINGLE);
  ads1.init();

  ads2.setAddr_ADS1115(ADS1115_IIC_ADDRESS1);
  ads2.setGain(eGAIN_TWOTHIRDS);
  ads2.setMode(eMODE_SINGLE);
  ads2.setRate(eRATE_128);
  ads2.setOSMode(eOSMODE_SINGLE);
  ads2.init();
}

/* 2880, 4940~5020, 7080 */
int calculatePWM(float degree) {
  const float deadZone = 2880;
  const float max = 7080;
  return (int)(((max -  deadZone) / 360.0 * degree + deadZone));
}

void loop() {
  int adc0 = 0, adc1 = 0, adc2 = 0, adc3 = 0, adc4 = 0, adc5 = 0;
  if (ads1.checkADS1115()) {
    adc0 = ads1.readVoltage(0);
    ledcWrite(0, calculatePWM(adc0 / 3300.0 * 360.0));
    adc1 = ads1.readVoltage(1);
    ledcWrite(1, calculatePWM(adc1 / 3300.0 * 360.0));
    adc2 = ads1.readVoltage(2);
    ledcWrite(2, calculatePWM(adc2 / 3300.0 * 360.0));
    adc3 = ads1.readVoltage(3);
    ledcWrite(3, calculatePWM(adc3 / 3300.0 * 360.0));
  } else
    Serial.println("ADS1 Disconnected!");
  if (ads2.checkADS1115()) {
    adc4 = ads2.readVoltage(0);
    ledcWrite(4, calculatePWM(adc4 / 2050.0 * 180.0));
    adc5 = ads2.readVoltage(1);
    ledcWrite(5, calculatePWM(adc5 / 2800.0 * 180.0));
  } else
    Serial.println("ADS2 Disconnected!");
}
