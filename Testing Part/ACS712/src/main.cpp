#include <Arduino.h>

const int sensorPin = 0;

const float sensitivity = 0.100;

// offset in 0A
const float zeroCurrentVoltage = 2.572;

void setup() {

  Serial.begin(115200);

  analogReadResolution(12);

  analogSetAttenuation(ADC_11db);

  delay(1000);

  Serial.println("ACS712 Current Monitor");
}

void loop() {

  uint32_t total = 0;

  for(int i=0;i<128;i++){
    total += analogReadMilliVolts(sensorPin);
    delay(2);
  }

  float voltage = (total / 128.0) / 1000.0;

  // current back direction
  float current = (zeroCurrentVoltage - voltage) / sensitivity;

  // deadband noise
  if(abs(current) < 0.05){
    current = 0;
  }

  if(voltage < 1.0){

    Serial.println("Sensor OFF");

  } else {

    Serial.print("Voltage: ");
    Serial.print(voltage, 3);

    Serial.print(" V  Current: ");
    Serial.print(current, 2);

    Serial.println(" A");
  }

  delay(500);
}