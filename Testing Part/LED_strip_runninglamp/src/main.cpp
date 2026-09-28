#include <FastLED.h>
#define LED_PIN 18
#define NUM_LEDS 2
#define LED_TYPE WS2811
#define COLOR_ORDER RGB   
CRGB leds[NUM_LEDS];
void setup() {
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 50);
}
void loop() {
  for (int i = 0; i < NUM_LEDS; i++) {
    fadeToBlackBy(leds, NUM_LEDS, 2);
    leds[i] = CRGB::Green;
    FastLED.show();
    delay(30);
  }
}
