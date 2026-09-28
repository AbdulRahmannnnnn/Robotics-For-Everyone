#include <Arduino.h>

// =========================
// BTS7960 PIN CONFIG
// =========================

#define R_RPWM_PIN 26
#define R_LPWM_PIN 25

#define L_RPWM_PIN 18
#define L_LPWM_PIN 19

// =========================
// PWM CONFIG ESP32
// =========================

#define PWM_FREQ       20000
#define PWM_RESOLUTION 8

#define R_RPWM_CHANNEL 0
#define R_LPWM_CHANNEL 1

#define L_RPWM_CHANNEL 2
#define L_LPWM_CHANNEL 3

// =========================
// FUNCTION
// =========================

void motorStop()
{
  ledcWrite(R_RPWM_CHANNEL, 0);
  ledcWrite(R_LPWM_CHANNEL, 0);
  ledcWrite(L_RPWM_CHANNEL, 0);
  ledcWrite(L_LPWM_CHANNEL, 0);
}

void motorForward(int speedValue)
{
  speedValue = constrain(speedValue, 0, 255);

  ledcWrite(R_RPWM_CHANNEL, speedValue);
  ledcWrite(R_LPWM_CHANNEL, 0);
  ledcWrite(L_RPWM_CHANNEL, speedValue);
  ledcWrite(L_LPWM_CHANNEL, 0);
}

void motorBackward(int speedValue)
{
  speedValue = constrain(speedValue, 0, 255);

  ledcWrite(R_RPWM_CHANNEL, 0);
  ledcWrite(R_LPWM_CHANNEL, speedValue);
  ledcWrite(L_RPWM_CHANNEL, 0);
  ledcWrite(L_LPWM_CHANNEL, speedValue);
}

// =========================
// SETUP
// =========================

void setup()
{
  Serial.begin(115200);

  // setup PWM
  ledcSetup(R_RPWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(R_LPWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(L_RPWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(L_LPWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);

  // attach pin
  ledcAttachPin(R_RPWM_PIN, R_RPWM_CHANNEL);
  ledcAttachPin(R_LPWM_PIN, R_LPWM_CHANNEL);
  ledcAttachPin(L_RPWM_PIN, L_RPWM_CHANNEL);
  ledcAttachPin(L_LPWM_PIN, L_LPWM_CHANNEL);

  motorStop();

  Serial.println("BTS7960 Motor Control Ready");
}

// =========================
// LOOP
// =========================

void loop()
{
  // =========================
  // Forward Slowly
  // =========================

  Serial.println("Forward Speed 80");

  motorForward(80);

  delay(3000);

  // =========================
  // Forward Fast
  // =========================

  Serial.println("Forward Speed 200");

  motorForward(200);

  delay(3000);

  // =========================
  // STOP
  // =========================

  Serial.println("STOP");

  motorStop();

  delay(2000);

  // =========================
  // Backward Slowly
  // =========================

  Serial.println("Backward Speed 100");

  motorBackward(100);

  delay(3000);

  // =========================
  // Backqward Fast
  // =========================

  Serial.println("Backward Speed 255");

  motorBackward(255);

  delay(3000);

  // =========================
  // STOP
  // =========================

  Serial.println("STOP");

  motorStop();

  delay(3000);
}