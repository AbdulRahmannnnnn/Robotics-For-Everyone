#include <Arduino.h>
#include <Wire.h>
#include <MPU9250_asukiaaa.h>

MPU9250_asukiaaa mySensor;

void setup() {

  Serial.begin(115200);

  Wire.begin(21, 22);

  mySensor.setWire(&Wire);

  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();

  Serial.println("MPU9250 Ready");
}

void loop() {

  mySensor.accelUpdate();
  mySensor.gyroUpdate();
  mySensor.magUpdate();

  Serial.print("ACC X: ");
  Serial.print(mySensor.accelX());
  Serial.print(" Y: ");
  Serial.print(mySensor.accelY());
  Serial.print(" Z: ");
  Serial.println(mySensor.accelZ());

  Serial.print("GYRO X: ");
  Serial.print(mySensor.gyroX());
  Serial.print(" Y: ");
  Serial.print(mySensor.gyroY());
  Serial.print(" Z: ");
  Serial.println(mySensor.gyroZ());

  Serial.println("----------------");

  delay(200);
}