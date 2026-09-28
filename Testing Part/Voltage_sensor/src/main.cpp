#include <Arduino.h>

const int adcPin = 1; // GPIO0

void setup() {

  Serial.begin(115200);

  analogReadResolution(12);

  analogSetAttenuation(ADC_11db);

  delay(1000);

  Serial.println("Voltage Monitor");
}

void loop() {

  uint32_t total = 0;

  for(int i=0;i<32;i++){
    total += analogReadMilliVolts(adcPin);
    delay(2);
  }

  float mv = total / 32.0;

  // tegangan ADC
  float adcVoltage = mv / 1000.0;

  // divider sensor 30K : 7.5K = x5
  float inputVoltage = adcVoltage * 5.0;

  Serial.print("ADC Voltage: ");
  Serial.print(adcVoltage, 3);

  Serial.print(" V   Input Voltage: ");
  Serial.print(inputVoltage, 2);

  Serial.println(" V");

  delay(500);
}