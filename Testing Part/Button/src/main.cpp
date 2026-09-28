#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  Serial.println("Testing Button");
    pinMode(34, INPUT_PULLUP);
}
void loop() {
  if (digitalRead(34) == HIGH){
    Serial.println("Button Pressed");
  }else {
    Serial.println("Button Released");
    }
  delay(100);
}