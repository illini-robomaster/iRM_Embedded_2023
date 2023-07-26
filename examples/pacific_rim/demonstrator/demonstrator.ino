#include <Wire.h>
#include <DFRobot_ADS1115.h>

DFRobot_ADS1115 ads(&Wire);
const int pwmPin = A4;

void setup() {
  Serial.begin(115200);

  pinMode(pwmPin, OUTPUT);
  ledcSetup(8, 50, 16);
  ledcAttachPin(pwmPin, 8);

  ads.setAddr_ADS1115(ADS1115_IIC_ADDRESS0);
  ads.setGain(eGAIN_TWOTHIRDS);
  ads.setMode(eMODE_SINGLE);
  ads.setRate(eRATE_128);
  ads.setOSMode(eOSMODE_SINGLE);
  ads.init();
}

int calculatePWM(int degree) {
  const float deadZone = 2880;
  const float max = 7080;
  return (int)(((max -  deadZone) / 360 * degree + deadZone));
}

// [16] 2880, 4940~5020, 7080

void loop() {
  if (ads.checkADS1115()) {
    int16_t adc0, adc1, adc2, adc3;
    adc0 = ads.readVoltage(0);
    Serial.println(adc0);
    ledcWrite(8, calculatePWM(adc0 / 3297.0 * 360));
  } else
    Serial.println("ADS1115 Disconnected!");
}


