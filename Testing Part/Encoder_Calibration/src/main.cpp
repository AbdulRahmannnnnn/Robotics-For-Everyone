#include <Arduino.h>

#define ENC_R_A 14
#define ENC_R_B 27
#define ENC_L_A 33
#define ENC_L_B 32

#define R_RPWM_PIN 26
#define R_LPWM_PIN 25
#define L_RPWM_PIN 18
#define L_LPWM_PIN 19

#define PWM_FREQ       20000
#define PWM_RESOLUTION 8

#define R_RPWM_CHANNEL 0
#define R_LPWM_CHANNEL 1
#define L_RPWM_CHANNEL 2
#define L_LPWM_CHANNEL 3

#define ENCODER_PPR_ASSUMED   11
#define QUADRATURE_FACTOR     4
#define GEAR_RATIO_ASSUMED    56
#define PPR_EFFECTIVE_ASSUMED (ENCODER_PPR_ASSUMED * QUADRATURE_FACTOR * GEAR_RATIO_ASSUMED)

// =====================================================
// ENCODER VARIABLE
// =====================================================

volatile long encoderRight = 0;
volatile long encoderLeft  = 0;

void IRAM_ATTR encoderRightISR()
{
  static uint8_t oldState = 0;
  static int8_t table[] = {
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0
  };
  uint8_t newState = (digitalRead(ENC_R_A) << 1) | digitalRead(ENC_R_B);
  encoderRight += table[(oldState << 2) | newState];
  oldState = newState;
}

void IRAM_ATTR encoderLeftISR()
{
  static uint8_t oldState = 0;
  static int8_t table[] = {
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0
  };
  uint8_t newState = (digitalRead(ENC_L_A) << 1) | digitalRead(ENC_L_B);
  encoderLeft += table[(oldState << 2) | newState];
  oldState = newState;
}

// =====================================================
// MOTOR DRIVER
// =====================================================

void setMotorRight(int pwm)
{
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    ledcWrite(R_RPWM_CHANNEL, pwm);
    ledcWrite(R_LPWM_CHANNEL, 0);
  } else if (pwm < 0) {
    ledcWrite(R_RPWM_CHANNEL, 0);
    ledcWrite(R_LPWM_CHANNEL, -pwm);
  } else {
    ledcWrite(R_RPWM_CHANNEL, 0);
    ledcWrite(R_LPWM_CHANNEL, 0);
  }
}

void setMotorLeft(int pwm)
{
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    ledcWrite(L_RPWM_CHANNEL, pwm);
    ledcWrite(L_LPWM_CHANNEL, 0);
  } else if (pwm < 0) {
    ledcWrite(L_RPWM_CHANNEL, 0);
    ledcWrite(L_LPWM_CHANNEL, -pwm);
  } else {
    ledcWrite(L_RPWM_CHANNEL, 0);
    ledcWrite(L_LPWM_CHANNEL, 0);
  }
}

void stopMotors()
{
  setMotorRight(0);
  setMotorLeft(0);
}

// =====================================================
// MENU
// =====================================================

void printMenu()
{
  Serial.println();
  Serial.println(F("========== ENCODER CALIBRATION TOOL =========="));
  Serial.printf("Asumsi firmware saat ini: ENCODER_PPR=%d, GEAR_RATIO=%d\n",
                ENCODER_PPR_ASSUMED, GEAR_RATIO_ASSUMED);
  Serial.printf("-> PPR_EFFECTIVE_ASSUMED = %d count/putaran roda\n", PPR_EFFECTIVE_ASSUMED);
  Serial.println();
  Serial.println(F("MODE A - Kalibrasi manual (PALING AKURAT, disarankan):"));
  Serial.println(F("  1. Beri tanda spidol pada roda & chassis sebagai acuan 0 derajat"));
  Serial.println(F("  2. Ketik 'r' lalu Enter  -> reset counter ke 0"));
  Serial.println(F("  3. Putar roda dengan TANGAN, pelan-pelan, sejumlah N putaran PENUH"));
  Serial.println(F("     (disarankan N >= 10 supaya error pembulatan kecil)"));
  Serial.println(F("     Nilai counter akan tercetak live agar bisa dipantau"));
  Serial.println(F("  4. Setelah selesai tepat N putaran, ketik angka N lalu Enter"));
  Serial.println(F("     -> program menghitung PPR_EFFECTIVE & GEAR_RATIO yang SEBENARNYA"));
  Serial.println();
  Serial.println(F("MODE B - Uji jalan motor (cross-check, opsional):"));
  Serial.println(F("  f <pwm> <detik>  -> motor MAJU pada PWM tsb selama sekian detik"));
  Serial.println(F("  b <pwm> <detik>  -> motor MUNDUR pada PWM tsb selama sekian detik"));
  Serial.println(F("     contoh: f 80 5   (jalan maju pwm=80 selama 5 detik)"));
  Serial.println(F("     Hitung sendiri putaran roda sebenarnya (mis. dengan tally counter/"));
  Serial.println(F("     stopwatch) selama motor jalan, lalu bandingkan dengan hasil MODE A"));
  Serial.println();
  Serial.println(F("MODE C - Uji rotasi (motor berlawanan arah, cek noise EMI):"));
  Serial.println(F("  j <pwm> <detik>  -> kanan MAJU + kiri MUNDUR bersamaan"));
  Serial.println(F("  l <pwm> <detik>  -> kanan MUNDUR + kiri MAJU bersamaan"));
  Serial.println(F("     contoh: j 80 5   Bandingkan kemulusan kenaikan pulsa (kolom 'd')"));
  Serial.println(F("     dengan hasil MODE B untuk cek indikasi noise saat rotasi."));
  Serial.println();
  Serial.println(F("Perintah lain:"));
  Serial.println(F("  p -> cetak counter saat ini tanpa reset"));
  Serial.println(F("  s -> paksa stop motor"));
  Serial.println(F("  h -> tampilkan menu ini lagi"));
  Serial.println(F("================================================"));
  Serial.println();
}

// =====================================================
// HITUNG HASIL KALIBRASI MODE A
// =====================================================

void computeCalibration(long N)
{
  long r = encoderRight;
  long l = encoderLeft;

  Serial.println();
  Serial.println(F("========= HASIL KALIBRASI (MODE A) ========="));
  Serial.printf("Jumlah putaran manual (N) = %ld\n", N);

  if (labs(r) >= 5) {
    float pprReal  = (float)labs(r) / (float)N;
    float gearReal = pprReal / (ENCODER_PPR_ASSUMED * QUADRATURE_FACTOR);
    Serial.println(F("-- RIGHT --"));
    Serial.printf("  raw count           = %ld\n", r);
    Serial.printf("  PPR_EFFECTIVE real  = %.1f\n", pprReal);
    Serial.printf("  GEAR_RATIO real     = %.2f  (asumsi lama: %d)\n", gearReal, GEAR_RATIO_ASSUMED);
  } else {
    Serial.println(F("-- RIGHT -- (dilewati, roda kanan sepertinya tidak diputar)"));
  }

  if (labs(l) >= 5) {
    float pprReal  = (float)labs(l) / (float)N;
    float gearReal = pprReal / (ENCODER_PPR_ASSUMED * QUADRATURE_FACTOR);
    Serial.println(F("-- LEFT --"));
    Serial.printf("  raw count           = %ld\n", l);
    Serial.printf("  PPR_EFFECTIVE real  = %.1f\n", pprReal);
    Serial.printf("  GEAR_RATIO real     = %.2f  (asumsi lama: %d)\n", gearReal, GEAR_RATIO_ASSUMED);
  } else {
    Serial.println(F("-- LEFT -- (dilewati, roda kiri sepertinya tidak diputar)"));
  }

  Serial.println();
  Serial.println(F("Catatan: masukkan nilai PPR_EFFECTIVE / GEAR_RATIO di atas ke"));
  Serial.println(F("ESP32-Node/src/main.cpp (gantikan #define GEAR_RATIO lama),"));
  Serial.println(F("lalu PID (kp/ki/kd) kemungkinan perlu di-tuning ulang karena"));
  Serial.println(F("skala feedback kecepatan berubah."));
  Serial.println(F("=============================================="));
  Serial.println();
}

// =====================================================
// MODE B - UJI JALAN MOTOR
// =====================================================

void runMotorTest(bool forward, int pwm, int seconds)
{
  pwm = constrain(pwm, 0, 255);
  seconds = constrain(seconds, 1, 60);

  encoderRight = 0;
  encoderLeft  = 0;

  Serial.printf("\nMenjalankan motor %s, PWM=%d, selama %d detik...\n",
                forward ? "MAJU" : "MUNDUR", pwm, seconds);
  Serial.println(F("(hitung putaran roda sebenarnya sekarang jika ingin cross-check)"));

  int signedPwm = forward ? pwm : -pwm;
  setMotorRight(signedPwm);
  setMotorLeft(signedPwm);

  unsigned long tStart = millis();
  unsigned long tLastPrint = tStart;
  while (millis() - tStart < (unsigned long)seconds * 1000UL) {
    if (millis() - tLastPrint >= 300) {
      tLastPrint = millis();
      Serial.printf("  [LIVE] Right=%ld  Left=%ld\n", encoderRight, encoderLeft);
    }
  }

  stopMotors();

  long r = encoderRight;
  long l = encoderLeft;
  float rpmRight = (labs(r) * 60.0f) / (PPR_EFFECTIVE_ASSUMED * (float)seconds);
  float rpmLeft  = (labs(l) * 60.0f) / (PPR_EFFECTIVE_ASSUMED * (float)seconds);

  Serial.println();
  Serial.println(F("========= HASIL UJI JALAN (MODE B) ========="));
  Serial.printf("Durasi              = %d detik\n", seconds);
  Serial.printf("RIGHT: pulsa=%ld  RPM(asumsi lama)=%.1f\n", r, rpmRight);
  Serial.printf("LEFT : pulsa=%ld  RPM(asumsi lama)=%.1f\n", l, rpmLeft);
  Serial.println(F("Bandingkan RPM di atas dengan hitungan manual (mis. stopwatch"));
  Serial.println(F("+ tally counter) untuk mengecek konsistensi terhadap MODE A."));
  Serial.println(F("=============================================="));
  Serial.println();
}

// =====================================================
// MODE C - UJI ROTASI (motor berlawanan arah, simulasi j/l)
// =====================================================

void runRotationTest(bool ccw, int pwm, int seconds)
{
  pwm = constrain(pwm, 0, 255);
  seconds = constrain(seconds, 1, 60);

  encoderRight = 0;
  encoderLeft  = 0;

  Serial.printf("\nMenjalankan rotasi %s, PWM=%d, selama %d detik...\n",
                ccw ? "CCW (kanan maju, kiri mundur)" : "CW (kanan mundur, kiri maju)",
                pwm, seconds);
  Serial.println(F("(perhatikan apakah pulsa naik mulus atau melonjak liar)"));

  int rightPwm = ccw ?  pwm : -pwm;
  int leftPwm  = ccw ? -pwm :  pwm;
  setMotorRight(rightPwm);
  setMotorLeft(leftPwm);

  unsigned long tStart = millis();
  unsigned long tLastPrint = tStart;
  long lastR = 0, lastL = 0;
  while (millis() - tStart < (unsigned long)seconds * 1000UL) {
    if (millis() - tLastPrint >= 300) {
      tLastPrint = millis();
      long r = encoderRight;
      long l = encoderLeft;
      Serial.printf("  [LIVE] Right=%ld (d=%ld)  Left=%ld (d=%ld)\n",
                    r, r - lastR, l, l - lastL);
      lastR = r;
      lastL = l;
    }
  }

  stopMotors();

  long r = encoderRight;
  long l = encoderLeft;
  float rpmRight = (labs(r) * 60.0f) / (PPR_EFFECTIVE_ASSUMED * (float)seconds);
  float rpmLeft  = (labs(l) * 60.0f) / (PPR_EFFECTIVE_ASSUMED * (float)seconds);

  Serial.println();
  Serial.println(F("========= HASIL UJI ROTASI (MODE C) ========="));
  Serial.printf("Durasi              = %d detik\n", seconds);
  Serial.printf("RIGHT: pulsa=%ld  RPM(asumsi lama)=%.1f\n", r, rpmRight);
  Serial.printf("LEFT : pulsa=%ld  RPM(asumsi lama)=%.1f\n", l, rpmLeft);
  Serial.println(F("Bandingkan kenaikan pulsa per 300ms di atas (kolom 'd') dengan"));
  Serial.println(F("hasil MODE B (searah). Kalau di sini lompatannya jauh lebih besar/"));
  Serial.println(F("tidak konsisten dibanding MODE B, itu indikasi noise EMI saat"));
  Serial.println(F("kedua motor switching berlawanan arah bersamaan."));
  Serial.println(F("=============================================="));
  Serial.println();
}

// =====================================================
// PARSING PERINTAH
// =====================================================

bool isAllDigits(const String &s)
{
  if (s.length() == 0) return false;
  for (size_t i = 0; i < s.length(); i++) {
    if (i == 0 && s[i] == '-') continue;
    if (!isDigit(s[i])) return false;
  }
  return true;
}

void handleCommand(String line)
{
  line.trim();
  if (line.length() == 0) return;

  if (line == "r") {
    encoderRight = 0;
    encoderLeft  = 0;
    Serial.println(F("Counter direset ke 0. Silakan putar roda dengan tangan."));
  }
  else if (line == "p") {
    Serial.printf("Right=%ld  Left=%ld\n", encoderRight, encoderLeft);
  }
  else if (line == "s") {
    stopMotors();
    Serial.println(F("Motor distop."));
  }
  else if (line == "h") {
    printMenu();
  }
  else if (line.startsWith("f ") || line.startsWith("b ")) {
    int pwm = 0, secs = 0;
    if (sscanf(line.c_str(), "%*c %d %d", &pwm, &secs) == 2) {
      runMotorTest(line[0] == 'f', pwm, secs);
    } else {
      Serial.println(F("Format salah. Contoh: f 80 5"));
    }
  }
  else if (line.startsWith("j ") || line.startsWith("l ")) {
    int pwm = 0, secs = 0;
    if (sscanf(line.c_str(), "%*c %d %d", &pwm, &secs) == 2) {
      runRotationTest(line[0] == 'j', pwm, secs);
    } else {
      Serial.println(F("Format salah. Contoh: j 80 5"));
    }
  }
  else if (isAllDigits(line)) {
    long N = line.toInt();
    if (N <= 0) {
      Serial.println(F("N harus lebih besar dari 0."));
    } else {
      computeCalibration(N);
    }
  }
  else {
    Serial.println(F("Perintah tidak dikenali. Ketik 'h' untuk bantuan."));
  }
}

// =====================================================
// SETUP & LOOP
// =====================================================

void setup()
{
  Serial.begin(115200);
  delay(1000);

  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_R_A), encoderRightISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_B), encoderRightISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), encoderLeftISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_B), encoderLeftISR,  CHANGE);

  ledcSetup(R_RPWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(R_LPWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(L_RPWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(L_LPWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(R_RPWM_PIN, R_RPWM_CHANNEL);
  ledcAttachPin(R_LPWM_PIN, R_LPWM_CHANNEL);
  ledcAttachPin(L_RPWM_PIN, L_RPWM_CHANNEL);
  ledcAttachPin(L_LPWM_PIN, L_LPWM_CHANNEL);

  stopMotors();

  printMenu();
}

void loop()
{
  static unsigned long lastLivePrint = 0;
  if (millis() - lastLivePrint >= 300) {
    lastLivePrint = millis();
    Serial.printf("[LIVE] Right=%ld  Left=%ld\n", encoderRight, encoderLeft);
  }

  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleCommand(line);
  }
}
