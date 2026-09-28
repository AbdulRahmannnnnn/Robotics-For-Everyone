#include <Arduino.h>

// =====================================================
// ENCODER PIN
// =====================================================

// RIGHT MOTOR ENCODER
#define ENC_R_A 14
#define ENC_R_B 27

// LEFT MOTOR ENCODER
#define ENC_L_A 33
#define ENC_L_B 32

// =====================================================
// BTS7960 PIN
// =====================================================

// RIGHT MOTOR DRIVER
#define R_RPWM_PIN 26
#define R_LPWM_PIN 25

// LEFT MOTOR DRIVER
#define L_RPWM_PIN 18
#define L_LPWM_PIN 19

// =====================================================
// PWM CONFIG
// =====================================================

#define PWM_FREQ 10000
#define PWM_RESOLUTION 8

#define R_RPWM_CHANNEL 0
#define R_LPWM_CHANNEL 1

#define L_RPWM_CHANNEL 2
#define L_LPWM_CHANNEL 3

// =====================================================
// MOTOR SPEC
// =====================================================

#define ENCODER_PPR 11
#define QUADRATURE_FACTOR 4
#define GEAR_RATIO 56

#define PPR_EFFECTIVE (ENCODER_PPR * QUADRATURE_FACTOR * GEAR_RATIO)

// =====================================================
// ENCODER VARIABLE
// =====================================================

volatile long encoderRight = 0;
volatile long encoderLeft = 0;

long lastRight = 0;
long lastLeft = 0;

float rpmRight = 0;
float rpmLeft = 0;

// =====================================================
// ENCODER ISR RIGHT
// =====================================================

void IRAM_ATTR encoderRightISR()
{
  static uint8_t oldState = 0;

  static int8_t table[] = {
    0, 1, -1, 0,
   -1, 0, 0, 1,
    1, 0, 0, -1,
    0, -1, 1, 0
  };

  uint8_t newState =
      (digitalRead(ENC_R_A) << 1) |
       digitalRead(ENC_R_B);

  uint8_t index = (oldState << 2) | newState;

  encoderRight += table[index];

  oldState = newState;
}

// =====================================================
// ENCODER ISR LEFT
// =====================================================

void IRAM_ATTR encoderLeftISR()
{
  static uint8_t oldState = 0;

  static int8_t table[] = {
    0, 1, -1, 0,
   -1, 0, 0, 1,
    1, 0, 0, -1,
    0, -1, 1, 0
  };

  uint8_t newState =
      (digitalRead(ENC_L_A) << 1) |
       digitalRead(ENC_L_B);

  uint8_t index = (oldState << 2) | newState;

  encoderLeft += table[index];

  oldState = newState;
}

// =====================================================
// MOTOR FUNCTION
// =====================================================

void stopMotor()
{
  ledcWrite(R_RPWM_CHANNEL, 0);
  ledcWrite(R_LPWM_CHANNEL, 0);

  ledcWrite(L_RPWM_CHANNEL, 0);
  ledcWrite(L_LPWM_CHANNEL, 0);
}

void forwardMotor(int speedValue)
{
  speedValue = constrain(speedValue, 0, 255);

  // RIGHT
  ledcWrite(R_RPWM_CHANNEL, speedValue);
  ledcWrite(R_LPWM_CHANNEL, 0);

  // LEFT
  ledcWrite(L_RPWM_CHANNEL, speedValue);
  ledcWrite(L_LPWM_CHANNEL, 0);
}

void backwardMotor(int speedValue)
{
  speedValue = constrain(speedValue, 0, 255);

  // RIGHT
  ledcWrite(R_RPWM_CHANNEL, 0);
  ledcWrite(R_LPWM_CHANNEL, speedValue);

  // LEFT
  ledcWrite(L_RPWM_CHANNEL, 0);
  ledcWrite(L_LPWM_CHANNEL, speedValue);
}

// =====================================================
// RPM CALCULATION
// =====================================================

void updateRPM()
{
  static unsigned long lastUpdate = 0;

  unsigned long now = millis();

  if (now - lastUpdate >= 200)
  {
    long currentRight = encoderRight;
    long currentLeft = encoderLeft;

    long deltaRight = currentRight - lastRight;
    long deltaLeft = currentLeft - lastLeft;

    float dt = (now - lastUpdate) / 1000.0;

    rpmRight =
      (abs(deltaRight) * 60.0) /
      (PPR_EFFECTIVE * dt);

    rpmLeft =
      (abs(deltaLeft) * 60.0) /
      (PPR_EFFECTIVE * dt);

    lastRight = currentRight;
    lastLeft = currentLeft;

    lastUpdate = now;
  }
}

// =====================================================
// DISPLAY
// =====================================================

void displayData()
{
  static unsigned long lastDisplay = 0;

  if (millis() - lastDisplay >= 1000)
  {
    Serial.println("================================");

    Serial.print("RIGHT RPM : ");
    Serial.print(rpmRight, 1);

    Serial.print(" | Count : ");
    Serial.println(encoderRight);

    Serial.print("LEFT  RPM : ");
    Serial.print(rpmLeft, 1);

    Serial.print(" | Count : ");
    Serial.println(encoderLeft);

    Serial.println("================================");

    lastDisplay = millis();
  }
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);

  delay(2000);

  Serial.println("Dual Motor Encoder Test");

  // =====================================================
  // ENCODER
  // =====================================================

  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(ENC_R_A),
    encoderRightISR,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(ENC_R_B),
    encoderRightISR,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(ENC_L_A),
    encoderLeftISR,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(ENC_L_B),
    encoderLeftISR,
    CHANGE
  );

  // =====================================================
  // PWM
  // =====================================================

  ledcSetup(R_RPWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(R_LPWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);

  ledcSetup(L_RPWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(L_LPWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(R_RPWM_PIN, R_RPWM_CHANNEL);
  ledcAttachPin(R_LPWM_PIN, R_LPWM_CHANNEL);

  ledcAttachPin(L_RPWM_PIN, L_RPWM_CHANNEL);
  ledcAttachPin(L_LPWM_PIN, L_LPWM_CHANNEL);

  stopMotor();

  Serial.println("Ready");
  Serial.println("");

  Serial.println("Command:");
  Serial.println("f = forward");
  Serial.println("b = backward");
  Serial.println("s = stop");
  Serial.println("1-9 speed");
}

// =====================================================
// LOOP
// =====================================================

int currentSpeed = 0;

void loop()
{
  updateRPM();

  displayData();

  if (Serial.available())
  {
    char cmd = Serial.read();

    switch (cmd)
    {
      case 'f':

        forwardMotor(currentSpeed);

        Serial.print("Forward Speed : ");
        Serial.println(currentSpeed);

        break;

      case 'b':

        backwardMotor(currentSpeed);

        Serial.print("Backward Speed : ");
        Serial.println(currentSpeed);

        break;

      case 's':

        stopMotor();

        Serial.println("STOP");

        break;

      case 'r':

        encoderRight = 0;
        encoderLeft = 0;

        Serial.println("Encoder Reset");

        break;

      case '1'...'9':

        currentSpeed = map(
          cmd - '0',
          1,
          9,
          40,
          255
        );

        Serial.print("Speed Set : ");
        Serial.println(currentSpeed);

        break;
    }
  }

  delay(10);
}