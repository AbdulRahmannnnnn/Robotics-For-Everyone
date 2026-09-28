#include <FastLED.h>
#define LED_PIN 5
#define NUM_LEDS 2
#define LED_TYPE WS2811
#define COLOR_ORDER RGB   

CRGB leds[NUM_LEDS];

void setup() {
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 10);
}
void loop() {
  // Forward
  for (int i = 0; i < NUM_LEDS; i++) {
    FastLED.clear();
    leds[i] = CRGB::Blue;
    FastLED.show();
    delay(25);
  }

  // Backward
  for (int i = NUM_LEDS - 1; i >= 0; i--) {
    FastLED.clear();
    leds[i] = CRGB::Blue;
    FastLED.show();
    delay(25);
  }
}