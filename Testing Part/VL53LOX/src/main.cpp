#include <Wire.h>
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

const int offset = -43; // mm come from calibration (sensor not perfectly aligned with front of robot)

void setup() {

  Serial.begin(115200);

  Wire.begin();
  
  if (!lox.begin()) {

    Serial.println("Failed to boot VL53L0X");
  }

  Serial.println("VL53L0X Ready");
}

void loop() {

  VL53L0X_RangingMeasurementData_t measure;

  lox.rangingTest(&measure, false);

  if (measure.RangeStatus != 4 || measure.RangeMilliMeter != 28166) {

    Serial.print("Distance: ");
    Serial.print(measure.RangeMilliMeter + offset);
    Serial.println(" mm");

  } else {

    Serial.println("Out of range / invalid");
  }

  delay(100);
}