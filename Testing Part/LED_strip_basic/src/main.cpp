#include <Arduino.h>
#include <FastLED.h>

#define LED_PIN  2
#define NUM_LEDS 8

CRGB leds[NUM_LEDS];

void setup() {
  FastLED.addLeds<WS2812B, LED_PIN, BRG>(leds, NUM_LEDS);
  FastLED.setBrightness(200);
}

void loop() {
  // PURE RED (255, 0, 0)
  fill_solid(leds, NUM_LEDS, CRGB(255, 0, 0));
  FastLED.show();
  delay(2000);

  // PURE GREEN (0, 255, 0)
  fill_solid(leds, NUM_LEDS, CRGB(0, 255, 0));
  FastLED.show();
  delay(2000);

  // PURE BLUE (0, 0, 255)
  fill_solid(leds, NUM_LEDS, CRGB(0, 0, 255));
  FastLED.show();
  delay(2000);
}