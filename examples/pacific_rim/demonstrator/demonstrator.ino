#include <Wire.h>
#include <DFRobot_ADS1115.h>

DFRobot_ADS1115 ads1(&Wire);
DFRobot_ADS1115 ads2(&Wire);

const int pwm1Pin = A4;
const int pwm2Pin = A3;

void setup() {
  Serial.begin(115200);

  pinMode(pwm1Pin, OUTPUT);
  pinMode(pwm2Pin, OUTPUT);
  ledcSetup(0, 50, 16);
  ledcSetup(1, 50, 16);
  ledcAttachPin(pwm1Pin, 0);
  ledcAttachPin(pwm2Pin, 1);

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

int calculatePWM(int degree) {
  const float deadZone = 2880;
  const float max = 7080;
  return (int)(((max -  deadZone) / 360 * degree + deadZone));
}

// [16] 2880, 4940~5020, 7080

void loop() {
  if (ads1.checkADS1115()) {
    int16_t adc0, adc1, adc2, adc3;
    adc0 = ads1.readVoltage(0);
    adc1 = ads1.readVoltage(1);
    // Serial.println(adc0);
    ledcWrite(0, calculatePWM(adc0 / 3297.0 * 360));
    ledcWrite(1, calculatePWM(adc1 / 3297.0 * 360));

  } else
    Serial.println("ADS1115 Disconnected!");
}


