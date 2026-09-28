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
#define L_RPWM_PIN 26
#define L_LPWM_PIN 25

// LEFT MOTOR DRIVER
#define R_RPWM_PIN 18
#define R_LPWM_PIN 19

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
// GLOBAL VARIABLES
// =====================================================

volatile long encoderRight = 0;
volatile long encoderLeft = 0;

long lastRight = 0;
long lastLeft = 0;

float rpmRight = 0;
float rpmLeft = 0;

// Moving average filter untuk rpmRight/rpmLeft (check updateRPM())
// always reset when the motor is stopped
#define RPM_FILTER_N 5
float rpmRightBuf[RPM_FILTER_N] = {0};
float rpmLeftBuf[RPM_FILTER_N]  = {0};
int   rpmBufIdx = 0;

int TargetRPM = 100;

float kpRight = 1.2132 , kiRight = 12.7705, kdRight = 0.0;
float kpLeft  = 1.2508, kiLeft  = 14.7149, kdLeft  = 0.0;

float integralRight = 0, integralLeft = 0;
float lastErrorRight = 0, lastErrorLeft = 0;

int pwmRight = 0;
int pwmLeft  = 0;

bool runPID = false;
bool testForward = true; // Fast PID mode (command 1-9), toggle with 'z'

// =====================================================
// AUTO TUNING VARIABLES (RELAY METHOD)
// =====================================================

bool isAutoTuning = false;
int autoTuneStepPWM = 40;      
int autoTuneBasePWM = 120;      // Base PWM
int currentMotorTuning = 0;     // 0 = Right, 1 = Left

float maxRPMMeasured = 0;
float minRPMMeasured = 9999;
int zeroCrossCount = 0;
unsigned long firstCrossTime = 0;
unsigned long lastCrossTime = 0;
bool lastSign = false;

// =====================================================
// SYSTEM ID (DEADBAND + STEP RESPONSE + IMC TUNING)
// =====================================================

#define MAX_STEP_SAMPLES 1000
#define STEP_SAMPLE_INTERVAL_MS 5
float stepTimeS[MAX_STEP_SAMPLES];
float stepRpmRight[MAX_STEP_SAMPLES];
float stepRpmLeft[MAX_STEP_SAMPLES];
int   stepSampleCount = 0;

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

  uint8_t newState = (digitalRead(ENC_R_A) << 1) | digitalRead(ENC_R_B);
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

  uint8_t newState = (digitalRead(ENC_L_A) << 1) | digitalRead(ENC_L_B);
  uint8_t index = (oldState << 2) | newState;

  encoderLeft += table[index];
  oldState = newState;
}

// =====================================================
// MOTOR FUNCTIONS
// =====================================================

void stopMotor()
{
  ledcWrite(R_RPWM_CHANNEL, 0);
  ledcWrite(R_LPWM_CHANNEL, 0);

  ledcWrite(L_RPWM_CHANNEL, 0);
  ledcWrite(L_LPWM_CHANNEL, 0);

  pwmRight = 0;
  pwmLeft = 0;
  integralRight = 0;
  integralLeft = 0;

  // Reset buffer moving-average RPM 
  for (int i = 0; i < RPM_FILTER_N; i++) {
    rpmRightBuf[i] = 0;
    rpmLeftBuf[i]  = 0;
  }
  rpmBufIdx = 0;
  rpmRight = 0;
  rpmLeft  = 0;
}

void setMotorPWM(int rightPWM, int leftPWM)
{
  rightPWM = constrain(rightPWM, -255, 255);
  leftPWM  = constrain(leftPWM, -255, 255);

  if (rightPWM >= 0) {
    ledcWrite(R_RPWM_CHANNEL, rightPWM);
    ledcWrite(R_LPWM_CHANNEL, 0);
  } else {
    ledcWrite(R_RPWM_CHANNEL, 0);
    ledcWrite(R_LPWM_CHANNEL, -rightPWM);
  }

  if (leftPWM >= 0) {
    ledcWrite(L_RPWM_CHANNEL, leftPWM);
    ledcWrite(L_LPWM_CHANNEL, 0);
  } else {
    ledcWrite(L_RPWM_CHANNEL, 0);
    ledcWrite(L_LPWM_CHANNEL, -leftPWM);
  }
}

// =====================================================
// RPM CALCULATION (+ moving average filter)
// =====================================================

void updateRPM()
{
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();

  if (now - lastUpdate >= 20)
  {
    long currentRight = encoderRight;
    long currentLeft = encoderLeft;

    long deltaRight = currentRight - lastRight;
    long deltaLeft = currentLeft - lastLeft;

    float dt = (now - lastUpdate) / 1000.0;

    float rawRpmRight = (abs(deltaRight) * 60.0) / (PPR_EFFECTIVE * dt);
    float rawRpmLeft  = (abs(deltaLeft) * 60.0)  / (PPR_EFFECTIVE * dt);

    rpmRightBuf[rpmBufIdx] = rawRpmRight;
    rpmLeftBuf[rpmBufIdx]  = rawRpmLeft;
    rpmBufIdx = (rpmBufIdx + 1) % RPM_FILTER_N;

    float sumR = 0, sumL = 0;
    for (int i = 0; i < RPM_FILTER_N; i++) {
      sumR += rpmRightBuf[i];
      sumL += rpmLeftBuf[i];
    }
    rpmRight = sumR / RPM_FILTER_N;
    rpmLeft  = sumL / RPM_FILTER_N;

    lastRight = currentRight;
    lastLeft = currentLeft;

    lastUpdate = now;
  }
}

// =====================================================
// AUTO TUNING PROCESS
// =====================================================

void startAutoTune(int motorSelect)
{
  currentMotorTuning = motorSelect; // 0 = Right, 1 = Left
  isAutoTuning = true;
  runPID = false;

  maxRPMMeasured = 0;
  minRPMMeasured = 9999;
  zeroCrossCount = 0;
  firstCrossTime = 0;
  lastCrossTime = 0;
  lastSign = false;

  Serial.println("==========================================");
  Serial.print("Starting Auto-Tuning Motor ");
  Serial.println(motorSelect == 0 ? "RIGHT" : "LEFT");
  Serial.print("Target RPM: "); Serial.println(TargetRPM);
  Serial.println("Please wait until oscillation is complete...");
  Serial.println("==========================================");
}

void processAutoTune()
{
  if (!isAutoTuning) return;

  float currentRPM = (currentMotorTuning == 0) ? rpmRight : rpmLeft;
  unsigned long now = millis();

  // Relay action: output High if currentRPM is below target, Low if currentRPM is above target
  bool currentSign = (currentRPM >= TargetRPM);
  int outputPWM = currentSign ? (autoTuneBasePWM - autoTuneStepPWM) : (autoTuneBasePWM + autoTuneStepPWM);

  if (currentMotorTuning == 0) {
    setMotorPWM(outputPWM, 0);
  } else {
    setMotorPWM(0, outputPWM);
  }

  // write Amplitudo RPM (Peak & Valley)
  if (currentRPM > maxRPMMeasured) maxRPMMeasured = currentRPM;
  if (currentRPM < minRPMMeasured) minRPMMeasured = currentRPM;

  // Detect Zero-Crossing
  if (currentSign != lastSign) {
    lastSign = currentSign;
    zeroCrossCount++;

    if (zeroCrossCount == 2) {
      firstCrossTime = now; // write start time of stable cycle
    }
    if (zeroCrossCount > 2) {
      lastCrossTime = now;
    }
  }

  // Evaluate after 12 times crossing (~5 osilasi)
  if (zeroCrossCount >= 12) {
    stopMotor();
    isAutoTuning = false;

    float A = (maxRPMMeasured - minRPMMeasured) / 2.0; // Amplitudo RPM
    float cycles = (zeroCrossCount - 2) / 2.0;
    float Pu = ((lastCrossTime - firstCrossTime) / 1000.0) / cycles; // Ultimate Period
    float d = autoTuneStepPWM;

    float Ku = (4.0 * d) / (PI * A); // Ultimate Gain

    // Ziegler-Nichols Rules for PI Control (free overshoot on motor)
    float tunedKp = 0.45 * Ku;
    float tunedKi = tunedKp / (0.83 * Pu);

    Serial.println("\n==========================================");
    Serial.println("    RESULTS OF AUTO-TUNING PID (Ziegler-Nichols)");
    Serial.println("==========================================");
    Serial.print("Motor             : "); Serial.println(currentMotorTuning == 0 ? "RIGHT" : "LEFT");
    Serial.print("Amplitudo RPM (A) : "); Serial.println(A, 2);
    Serial.print("Period (Pu)       : "); Serial.print(Pu, 4); Serial.println(" seconds");
    Serial.print("Ultimate Gain (Ku): "); Serial.println(Ku, 4);
    Serial.println("------------------------------------------");
    Serial.print("Recommended Kp    : "); Serial.println(tunedKp, 4);
    Serial.print("Recommended Ki    : "); Serial.println(tunedKi, 4);
    Serial.println("==========================================\n");

    // Apply results to control variables
    if (currentMotorTuning == 0) {
      kpRight = tunedKp;
      kiRight = tunedKi;
    } else {
      kpLeft = tunedKp;
      kiLeft = tunedKi;
    }
  }
}

// =====================================================
// DEADBAND FINDER
// =====================================================
// Ramp PWM slow from 0 until the wheel starts rotating (RPM > threshold
// small), to find the minimum PWM that can overcome static friction.

// motorSelect: 0 = Right, 1 = Left | dir: +1 = forward, -1 = backward
void findDeadbandOneMotor(int motorSelect, int dir)
{
  const float RPM_THRESHOLD = 3.0;
  const int   STEP_DELAY_MS = 150;

  Serial.println("==========================================");
  Serial.print("Deadband Finder - Motor ");
  Serial.print(motorSelect == 0 ? "RIGHT" : "LEFT");
  Serial.print(" (");
  Serial.print(dir > 0 ? "FORWARD" : "BACKWARD");
  Serial.println(")");
  Serial.println("==========================================");

  encoderRight = 0;
  encoderLeft  = 0;
  lastRight = 0;
  lastLeft  = 0;

  for (int mag = 0; mag <= 200; mag += 2) {
    int pwm = mag * dir;
    if (motorSelect == 0) setMotorPWM(pwm, 0);
    else                  setMotorPWM(0, pwm);

    unsigned long tStart = millis();
    while (millis() - tStart < STEP_DELAY_MS) {
      updateRPM();
    }

    float rpm = (motorSelect == 0) ? rpmRight : rpmLeft; //  abs() on updateRPM
    Serial.print("  PWM="); Serial.print(pwm);
    Serial.print("  RPM="); Serial.println(rpm, 2);

    if (rpm > RPM_THRESHOLD) {
      stopMotor();
      Serial.println("------------------------------------------");
      Serial.print(">>> DEADBAND PWM (");
      Serial.print(motorSelect == 0 ? "RIGHT" : "LEFT");
      Serial.print(" "); Serial.print(dir > 0 ? "FORWARD" : "BACKWARD");
      Serial.print(") = "); Serial.println(pwm);
      Serial.println("==========================================\n");
      return;
    }
  }

  stopMotor();
  Serial.println(">>> Wheel does not move until PWM=200, check wiring/motor.");
}

void findDeadband()
{
  findDeadbandOneMotor(0, 1);
  delay(500);
  findDeadbandOneMotor(0, -1);
  delay(500);
  findDeadbandOneMotor(1, 1);
  delay(500);
  findDeadbandOneMotor(1, -1);
}

// =====================================================
// STEP RESPONSE TEST + MODEL FOPDT (K, tau) + IMC TUNING
// =====================================================

void printIMCRecommendation(const char *label, float K, float tau)
{
  if (fabs(K) < 1e-6 || tau <= 0) {
    Serial.print("  ["); Serial.print(label); Serial.println("] Invalid data (K or tau is zero), skipping.");
    return;
  }

  Serial.print("  ["); Serial.print(label); Serial.println("]");
  Serial.print("    Model: K="); Serial.print(K, 4);
  Serial.print(" RPM/PWM , tau="); Serial.print(tau, 4); Serial.println(" s");

  // Lambda tuning: try several aggressiveness options for the closed-loop time constant
  float lambdas[3] = { 0.5f * tau, 1.0f * tau, 2.0f * tau };
  const char *names[3] = { "Aggressive (lambda=0.5*tau)", "Nominal (lambda=1.0*tau)", "Conservative (lambda=2.0*tau)" };

  for (int i = 0; i < 3; i++) {
    float lambda = lambdas[i];
    if (lambda <= 0) continue;
    float Kc = tau / (K * lambda);
    float Ki = Kc / tau; // Ti = tau -> Ki = Kc/Ti

    Serial.print("    "); Serial.print(names[i]); Serial.println(":");
    Serial.print("      Kp="); Serial.print(Kc, 4);
    Serial.print("  Ki="); Serial.println(Ki, 4);
  }
}

void runStepTest(int pwmBefore, int pwmAfter, int holdMs)
{
  pwmBefore = constrain(pwmBefore, -255, 255);
  pwmAfter  = constrain(pwmAfter, -255, 255);
  holdMs    = constrain(holdMs, 500, MAX_STEP_SAMPLES * STEP_SAMPLE_INTERVAL_MS - STEP_SAMPLE_INTERVAL_MS);

  bool forward = (pwmAfter >= 0 && pwmBefore >= 0);
  bool backward = (pwmAfter <= 0 && pwmBefore <= 0);
  if (!forward && !backward) {
    Serial.println(">>> pwmBefore and pwmAfter must be both positive (forward) or both negative (backward).");
    return;
  }

  Serial.println("==========================================");
  Serial.print("Step Response Test ("); Serial.print(forward ? "FORWARD" : "BACKWARD");
  Serial.print("): PWM "); Serial.print(pwmBefore);
  Serial.print(" -> "); Serial.println(pwmAfter);
  Serial.println("==========================================");

  // 1) Settle di pwmBefore
  Serial.println("Settling at initial PWM...");
  setMotorPWM(pwmBefore, pwmBefore);
  unsigned long tSettle = millis();
  while (millis() - tSettle < 2000) {
    updateRPM();
  }

  // 2) Ukur baseline RPM langsung dari delta encoder (window 100ms, presisi
  // cukup untuk rata-rata, tidak perlu resolusi tinggi di sini)
  long baseEncR0 = encoderRight, baseEncL0 = encoderLeft;
  unsigned long tBase0 = millis();
  delay(300);
  long baseEncR1 = encoderRight, baseEncL1 = encoderLeft;
  float baseDt = (millis() - tBase0) / 1000.0f;
  float baseR = (abs(baseEncR1 - baseEncR0) * 60.0f) / (PPR_EFFECTIVE * baseDt);
  float baseL = (abs(baseEncL1 - baseEncL0) * 60.0f) / (PPR_EFFECTIVE * baseDt);

  Serial.print("Baseline RPM  R="); Serial.print(baseR, 2);
  Serial.print("  L="); Serial.println(baseL, 2);

  // 3) Step ke pwmAfter, catat kurva RPM dengan sampling resolusi tinggi
  // (baca delta encoder langsung tiap STEP_SAMPLE_INTERVAL_MS, TIDAK lewat
  // updateRPM() yang punya gate internal 20ms sendiri - supaya tau tidak
  // ke-alias/kuantisasi ke kelipatan 20ms)
  stepSampleCount = 0;
  setMotorPWM(pwmAfter, pwmAfter);
  unsigned long t0 = millis();
  unsigned long lastSampleT = t0;
  long lastEncR = encoderRight;
  long lastEncL = encoderLeft;

  while ((int)(millis() - t0) < holdMs && stepSampleCount < MAX_STEP_SAMPLES) {
    unsigned long tNow = millis();
    if (tNow - lastSampleT < STEP_SAMPLE_INTERVAL_MS) continue;

    long curR = encoderRight;
    long curL = encoderLeft;
    float dt = (tNow - lastSampleT) / 1000.0f;

    stepTimeS[stepSampleCount]    = (tNow - t0) / 1000.0f;
    stepRpmRight[stepSampleCount] = (abs(curR - lastEncR) * 60.0f) / (PPR_EFFECTIVE * dt);
    stepRpmLeft[stepSampleCount]  = (abs(curL - lastEncL) * 60.0f) / (PPR_EFFECTIVE * dt);
    stepSampleCount++;

    lastEncR = curR;
    lastEncL = curL;
    lastSampleT = tNow;
  }

  stopMotor();

  if (stepSampleCount < 10) {
    Serial.println(">>> Sample terlalu sedikit, tes gagal.");
    return;
  }

  // 4) Steady-state akhir = rata-rata 20% sample terakhir
  int tailStart = (int)(stepSampleCount * 0.8f);
  float ssR = 0, ssL = 0;
  int tailN = 0;
  for (int i = tailStart; i < stepSampleCount; i++) {
    ssR += stepRpmRight[i];
    ssL += stepRpmLeft[i];
    tailN++;
  }
  ssR /= max(tailN, 1);
  ssL /= max(tailN, 1);

  Serial.print("Steady-state RPM  R="); Serial.print(ssR, 2);
  Serial.print("  L="); Serial.println(ssL, 2);

  // 5) Gain proses K = delta_RPM / delta_PWM (pakai magnitude PWM supaya K
  // tetap positif terlepas dari arah maju/mundur - RPM yang direkam juga
  // sudah magnitude/abs())
  float deltaPWM = (float)(abs(pwmAfter) - abs(pwmBefore));
  float Kr = (ssR - baseR) / deltaPWM;
  float Kl = (ssL - baseL) / deltaPWM;

  // 6) Time constant tau = waktu mencapai 63.2% dari perubahan
  float threshR = baseR + 0.632f * (ssR - baseR);
  float threshL = baseL + 0.632f * (ssL - baseL);

  float tauR = -1, tauL = -1;
  for (int i = 0; i < stepSampleCount; i++) {
    if (tauR < 0 && ((ssR >= baseR && stepRpmRight[i] >= threshR) ||
                      (ssR <  baseR && stepRpmRight[i] <= threshR))) {
      tauR = stepTimeS[i];
    }
    if (tauL < 0 && ((ssL >= baseL && stepRpmLeft[i] >= threshL) ||
                      (ssL <  baseL && stepRpmLeft[i] <= threshL))) {
      tauL = stepTimeS[i];
    }
  }

  Serial.println("------------------------------------------");
  Serial.println("RESULTS OF SYSTEM IDENTIFICATION (FOPDT)");
  Serial.println("------------------------------------------");
  printIMCRecommendation("RIGHT", Kr, tauR);
  printIMCRecommendation("LEFT", Kl, tauL);
  Serial.println("==========================================\n");
  Serial.println("Note: choose one (Aggressive/Nominal/Conservative),");
  Serial.println("then manually fill in kpRight/kiRight/kpLeft/kiLeft in the code.");
  Serial.println();
}

// =====================================================
// SPEED CONTROL (PID)
// =====================================================

void updateSpeedControl()
{
  static unsigned long lastControl = 0;

  if (!runPID || isAutoTuning)
    return;

  unsigned long now = millis();
  float dt = (now - lastControl) / 1000.0;

  if (dt < 0.02) // 20 ms
    return;

  lastControl = now;

  // -------------------------------------
  // RIGHT MOTOR PID
  // -------------------------------------
  float errorRight = TargetRPM - rpmRight;
  integralRight += errorRight * dt;
  integralRight = constrain(integralRight, -15, 15);
  float derivRight = (errorRight - lastErrorRight) / dt;

  int magPwmRight = (int)(kpRight * errorRight + kiRight * integralRight + kdRight * derivRight);
  magPwmRight = constrain(magPwmRight, 0, 255);
  lastErrorRight = errorRight;

  // -------------------------------------
  // LEFT MOTOR PID
  // -------------------------------------
  float errorLeft = TargetRPM - rpmLeft;
  integralLeft += errorLeft * dt;
  integralLeft = constrain(integralLeft, -15, 15);
  float derivLeft = (errorLeft - lastErrorLeft) / dt;

  int magPwmLeft = (int)(kpLeft * errorLeft + kiLeft * integralLeft + kdLeft * derivLeft);
  magPwmLeft = constrain(magPwmLeft, 0, 255);
  lastErrorLeft = errorLeft;

  // Error/integral tetap dihitung dari magnitude (rpmRight/rpmLeft selalu
  // positif dari encoder abs()) - arah cuma diterapkan di output akhir.
  pwmRight = testForward ? magPwmRight : -magPwmRight;
  pwmLeft  = testForward ? magPwmLeft  : -magPwmLeft;

  // OUTPUT
  setMotorPWM(pwmRight, pwmLeft);
}

// =====================================================
// DISPLAY
// =====================================================

void displayData()
{
  static unsigned long lastDisplay = 0;

  if (millis() - lastDisplay >= 500)
  {
    if (isAutoTuning) {
      Serial.print("[AUTO-TUNING] Motor: ");
      Serial.print(currentMotorTuning == 0 ? "R" : "L");
      Serial.print(" | Current RPM: ");
      Serial.print((currentMotorTuning == 0) ? rpmRight : rpmLeft, 1);
      Serial.print(" | Target: ");
      Serial.print(TargetRPM);
      Serial.print(" | Crosses: ");
      Serial.println(zeroCrossCount);
    } else if (runPID) {
      Serial.print("RPM R: ");
      Serial.print(rpmRight, 1);
      Serial.print(" (PWM: "); Serial.print(pwmRight); Serial.print(")");

      Serial.print(" | RPM L: ");
      Serial.print(rpmLeft, 1);
      Serial.print(" (PWM: "); Serial.print(pwmLeft); Serial.print(")");

      Serial.print(" | Target: ");
      Serial.print(TargetRPM);

      Serial.print(" | Kp_R: "); Serial.print(kpRight, 3);
      Serial.print(" Ki_R: "); Serial.print(kiRight, 3);
      Serial.print(" | Kp_L: "); Serial.print(kpLeft, 3);
      Serial.print(" Ki_L: "); Serial.println(kiLeft, 3);
    }

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

  Serial.println("==========================================");
  Serial.println("  ESP32 Dual Motor PID & Auto-Tuner Ready ");
  Serial.println("==========================================");

  // ENCODER PINS
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_R_A), encoderRightISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_B), encoderRightISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), encoderLeftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_B), encoderLeftISR, CHANGE);

  // PWM SETUP
  ledcSetup(R_RPWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(R_LPWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(L_RPWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(L_LPWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(R_RPWM_PIN, R_RPWM_CHANNEL);
  ledcAttachPin(R_LPWM_PIN, R_LPWM_CHANNEL);
  ledcAttachPin(L_RPWM_PIN, L_RPWM_CHANNEL);
  ledcAttachPin(L_LPWM_PIN, L_LPWM_CHANNEL);

  stopMotor();

  Serial.println("Perintah Serial:");
  Serial.println("  t : Auto-Tune Motor RIGHT (Ziegler-Nichols relay)");
  Serial.println("  y : Auto-Tune Motor LEFT (Ziegler-Nichols relay)");
  Serial.println("  1-9 : Set Target RPM & Run PID");
  Serial.println("  z : Toggle arah mode 1-9 (forward/backward, default forward)");
  Serial.println("  s : Stop Motor");
  Serial.println("  r : Reset Encoder Counter");
  Serial.println();
  Serial.println("System ID (System Identification) manual:");
  Serial.println("  d : Deadband Finder - automatically Test RIGHT & LEFT, FORWARD & BACKWARD (4 kombinasi)");
  Serial.println("  step <pwmBefore> <pwmAfter> <holdMs> : Step response test");
  Serial.println("      contoh forward : step 80 160 3000");
  Serial.println("      contoh backward (pwm negatif, should negative): step -80 -160 3000");
  Serial.println("      -> calculate gain (K), time constant (tau), then recomend Kp/Ki via IMC/Lambda tuning (3 choices of aggressiveness)");
  Serial.println("         Kp/Ki via IMC/Lambda tuning (3 pilihan agresivitas)");
  Serial.println("      Run for FORWARD dan BACKWARD separately, compare K/tau -");
  Serial.println("      if different, motor asimetris and need different gain per direction.");
  Serial.println();
}

// =====================================================
// LOOP
// =====================================================

void loop()
{
  updateRPM();

  if (isAutoTuning) {
    processAutoTune();
  } else {
    updateSpeedControl();
  }

  displayData();

  if (Serial.available())
  {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.length() == 0) {
      // no-op
    }
    else if (line == "s") {
      stopMotor();
      runPID = false;
      isAutoTuning = false;
      Serial.println(">>> MOTOR STOP <<<");
    }
    else if (line == "r") {
      encoderRight = 0;
      encoderLeft = 0;
      Serial.println(">>> Encoder Reset <<<");
    }
    else if (line == "t") {
      startAutoTune(0); // Auto-tune Motor Right
    }
    else if (line == "y") {
      startAutoTune(1); // Auto-tune Motor Left
    }
    else if (line == "d") {
      findDeadband();
    }
    else if (line == "z") {
      testForward = !testForward;
      stopMotor(); // reset integral/buffer 
      Serial.print(">>> Direction mode 1-9 Now: ");
      Serial.println(testForward ? "FORWARD" : "BACKWARD");
    }
    else if (line.startsWith("step ")) {
      int pwmBefore = 0, pwmAfter = 0, holdMs = 0;
      if (sscanf(line.c_str(), "%*s %d %d %d", &pwmBefore, &pwmAfter, &holdMs) == 3) {
        runStepTest(pwmBefore, pwmAfter, holdMs);
      } else {
        Serial.println(">>> Wrong Format. Example: step 80 160 3000");
      }
    }
    else if (line.length() == 1 && line[0] >= '1' && line[0] <= '9') {
      TargetRPM = map(line[0] - '0', 1, 9, 20, 170);
      runPID = true;
      isAutoTuning = false;
      Serial.print(">>> Target RPM Set : ");
      Serial.println(TargetRPM);
    }
    else {
      Serial.println(">>> Wrong Command. Type 'h' for help.");
    }
  }

  delay(10);
}