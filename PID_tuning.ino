#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Preferences.h>

constexpr uint8_t SDA_PIN = 21;
constexpr uint8_t SCL_PIN = 22;
constexpr uint8_t PCA9685_ADDR = 0x40;
constexpr uint8_t PCA9548A_ADDR = 0x70;

constexpr uint8_t QTR_COUNT = 8;
constexpr uint8_t QTR_PINS[QTR_COUNT] = {4, 16, 17, 18, 19, 27, 32, 33};

constexpr uint8_t FL_PWM = 13;
constexpr uint8_t FR_PWM = 14;
constexpr uint8_t RL_PWM = 25;
constexpr uint8_t RR_PWM = 26;

constexpr uint8_t FL_A = 3;
constexpr uint8_t FL_B = 4;
constexpr uint8_t FR_A = 5;
constexpr uint8_t FR_B = 6;
constexpr uint8_t RL_A = 7;
constexpr uint8_t RL_B = 8;
constexpr uint8_t RR_A = 9;
constexpr uint8_t RR_B = 10;
constexpr uint8_t STBY_CH = 11;

bool motorInvert[4] = {false, true, false, true};
int8_t motorDirState[4] = {0, 0, 0, 0};

constexpr uint8_t TOF_CHANNEL[3] = {0, 1, 2};
constexpr uint8_t TOF_COUNT = 3;
constexpr uint8_t TOF_ADDR = 0x29;

constexpr uint8_t LEFT_TOF = 0;
constexpr uint8_t FRONT_TOF = 1;
constexpr uint8_t RIGHT_TOF = 2;

constexpr bool LINE_IS_WHITE = true;
constexpr bool SENSOR_ORDER_REVERSED = false;

constexpr uint16_t QTR_CHARGE_US = 10;
constexpr uint16_t QTR_TIMEOUT_US = 3000;
constexpr uint16_t QTR_CAL_MS = 8000;
constexpr uint16_t LINE_VALID_TOTAL = 140;
constexpr uint16_t LINE_MIN_STRENGTH = 80;
constexpr uint32_t LINE_PERIOD_US = 2500;
constexpr uint16_t LINE_LOST_MS = 180;

constexpr uint16_t TOF_PERIOD_MS = 20;
constexpr uint16_t TOF_TIMEOUT_MS = 60;
constexpr uint16_t DEFAULT_WALL_TARGET_MM = 100;
constexpr uint16_t DEFAULT_FRONT_STOP_MM = 90;

constexpr uint8_t ERROR_HISTORY = 6;

constexpr float DEFAULT_LINE_KP = 80.0f;
constexpr float DEFAULT_LINE_KI = 0.0f;
constexpr float DEFAULT_LINE_KD = 0.03f;
constexpr float DEFAULT_LINE_BASE = 110.0f;

constexpr float DEFAULT_WALL_KP = 2.0f;
constexpr float DEFAULT_WALL_KI = 0.0f;
constexpr float DEFAULT_WALL_KD = 0.08f;
constexpr float DEFAULT_WALL_BASE = 110.0f;

constexpr float DEFAULT_HEADING_KP = 2.5f;
constexpr float DEFAULT_HEADING_KI = 0.0f;
constexpr float DEFAULT_HEADING_KD = 0.12f;
constexpr float DEFAULT_HEADING_BASE = 90.0f;

struct PIDConfig {
  float kp;
  float ki;
  float kd;
  float base;
  float integralLimit;
  float dFilter;
  float maxCorrection;
  float sign;
};

struct LineState {
  uint16_t raw[QTR_COUNT];
  uint16_t strength[QTR_COUNT];
  uint8_t mask;
  float error;
  bool valid;
};

struct ErrorSample {
  float error;
  uint32_t timeUs;
};

Adafruit_PWMServoDriver pca(PCA9685_ADDR);
VL53L0X tof[TOF_COUNT];
Adafruit_MPU6050 mpu;
Preferences prefs;

PIDConfig lineCfg{
  DEFAULT_LINE_KP,
  DEFAULT_LINE_KI,
  DEFAULT_LINE_KD,
  DEFAULT_LINE_BASE,
  0.8f,
  0.65f,
  255.0f,
  1.0f
};

PIDConfig wallCfg{
  DEFAULT_WALL_KP,
  DEFAULT_WALL_KI,
  DEFAULT_WALL_KD,
  DEFAULT_WALL_BASE,
  0.8f,
  0.65f,
  255.0f,
  1.0f
};

PIDConfig headingCfg{
  DEFAULT_HEADING_KP,
  DEFAULT_HEADING_KI,
  DEFAULT_HEADING_KD,
  DEFAULT_HEADING_BASE,
  200.0f,
  0.65f,
  255.0f,
  1.0f
};

LineState lineState{};
ErrorSample errorHistory[ERROR_HISTORY]{};

uint8_t errorHead = 0;
uint8_t errorCount = 0;
float integral = 0.0f;
float filteredDerivative = 0.0f;
float lastLineError = 0.0f;
int8_t lastLineSide = 1;
uint32_t lastValidLineMs = 0;
uint32_t lastLineControlUs = 0;

uint16_t tofMM[TOF_COUNT] = {0, 0, 0};
bool tofValid[TOF_COUNT] = {false, false, false};
uint32_t lastToFMs = 0;
uint16_t wallTargetMM = DEFAULT_WALL_TARGET_MM;
uint16_t frontStopMM = DEFAULT_FRONT_STOP_MM;

float gyroBiasZ = 0.0f;
float yawDeg = 0.0f;
float headingTargetDeg = 0.0f;
float gyroScale = 1.0f;
uint32_t lastIMUUs = 0;
bool imuReady = false;

uint8_t wallMode = 0;
bool frontSafety = true;
bool adaptiveLine = false;
uint32_t lastPrintMs = 0;
uint32_t lastMotorUs = 0;
uint32_t lastPidUs = 0;
uint8_t tofRoundRobin = 0;
uint16_t qtrMinUs[QTR_COUNT]{};
uint16_t qtrMaxUs[QTR_COUNT]{};
bool qtrCalLoaded = false;
String commandLine;

enum ControlMode : uint8_t {
  MODE_STOP,
  MODE_LINE,
  MODE_WALL,
  MODE_HEADING,
  MODE_TURN
};

ControlMode mode = MODE_STOP;

float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

int clampi(int x, int lo, int hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

float wrapAngle(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

PIDConfig &activeCfg() {
  if (mode == MODE_LINE) return lineCfg;
  if (mode == MODE_WALL) return wallCfg;
  return headingCfg;
}

void selectMux(uint8_t channel) {
  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write((uint8_t)(1u << channel));
  Wire.endTransmission();
}

void deselectMux() {
  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write((uint8_t)0);
  Wire.endTransmission();
}

void pcaHigh(uint8_t ch) {
  pca.setPin(ch, 4095, false);
}

void pcaLow(uint8_t ch) {
  pca.setPin(ch, 0, false);
}

void setMotorDirection(uint8_t index, bool forward) {
  const uint8_t a[4] = {FL_A, FR_A, RL_A, RR_A};
  const uint8_t b[4] = {FL_B, FR_B, RL_B, RR_B};

  if (motorInvert[index]) forward = !forward;

  if (forward) {
    pcaHigh(a[index]);
    pcaLow(b[index]);
  } else {
    pcaLow(a[index]);
    pcaHigh(b[index]);
  }
}

void setMotor(uint8_t index, int command) {
  const uint8_t pwm[4] = {FL_PWM, FR_PWM, RL_PWM, RR_PWM};
  const uint8_t a[4] = {FL_A, FR_A, RL_A, RR_A};
  const uint8_t b[4] = {FL_B, FR_B, RL_B, RR_B};

  command = clampi(command, -255, 255);

  if (command == 0) {
    if (motorDirState[index] != 0) {
      pcaLow(a[index]);
      pcaLow(b[index]);
      motorDirState[index] = 0;
    }
    ledcWrite(pwm[index], 0);
    return;
  }

  int8_t wanted = command > 0 ? 1 : -1;
  if (motorDirState[index] != wanted) {
    setMotorDirection(index, command > 0);
    motorDirState[index] = wanted;
  }

  ledcWrite(pwm[index], abs(command));
}

void setTank(int left, int right) {
  setMotor(0, left);
  setMotor(2, left);
  setMotor(1, right);
  setMotor(3, right);
}

void stopMotors() {
  setTank(0, 0);
}

void readQTR(uint16_t out[QTR_COUNT]) {
  for (uint8_t i = 0; i < QTR_COUNT; ++i) {
    out[i] = 0;
    pinMode(QTR_PINS[i], OUTPUT);
    digitalWrite(QTR_PINS[i], HIGH);
  }

  delayMicroseconds(QTR_CHARGE_US);

  for (uint8_t i = 0; i < QTR_COUNT; ++i) {
    pinMode(QTR_PINS[i], INPUT);
  }

  uint8_t pending = QTR_COUNT;
  uint32_t start = micros();

  while (pending) {
    uint32_t elapsed = micros() - start;
    if (elapsed >= QTR_TIMEOUT_US) break;

    for (uint8_t i = 0; i < QTR_COUNT; ++i) {
      if (out[i] == 0 && digitalRead(QTR_PINS[i]) == LOW) {
        out[i] = (uint16_t)elapsed;
        --pending;
      }
    }
  }

  for (uint8_t i = 0; i < QTR_COUNT; ++i) {
    if (out[i] == 0) out[i] = QTR_TIMEOUT_US;
  }
}

void resetPID() {
  memset(errorHistory, 0, sizeof(errorHistory));
  errorHead = 0;
  errorCount = 0;
  integral = 0.0f;
  filteredDerivative = 0.0f;
  lastPidUs = 0;
}

void calibrateQTR() {
  stopMotors();

  uint16_t minUs[QTR_COUNT];
  uint16_t maxUs[QTR_COUNT];
  uint16_t sample[QTR_COUNT];

  for (uint8_t i = 0; i < QTR_COUNT; ++i) {
    minUs[i] = QTR_TIMEOUT_US;
    maxUs[i] = 0;
  }

  Serial.println("CAL START");

  uint32_t start = millis();
  while (millis() - start < QTR_CAL_MS) {
    readQTR(sample);
    for (uint8_t i = 0; i < QTR_COUNT; ++i) {
      if (sample[i] < minUs[i]) minUs[i] = sample[i];
      if (sample[i] > maxUs[i]) maxUs[i] = sample[i];
    }
  }

  prefs.begin("qtr", false);
  prefs.putBool("valid", true);

  bool good = true;

  for (uint8_t i = 0; i < QTR_COUNT; ++i) {
    if (maxUs[i] <= minUs[i] + 30) good = false;

    char a[5];
    char b[5];
    snprintf(a, sizeof(a), "n%d", i);
    snprintf(b, sizeof(b), "x%d", i);

    prefs.putUShort(a, minUs[i]);
    prefs.putUShort(b, maxUs[i]);
  }

  prefs.putBool("valid", good);
  prefs.end();

  for (uint8_t i = 0; i < QTR_COUNT; ++i) {
    qtrMinUs[i] = minUs[i];
    qtrMaxUs[i] = maxUs[i];
  }
  qtrCalLoaded = good;

  Serial.println(good ? "CAL OK" : "CAL BAD");
}

uint16_t normalizedQTR(uint16_t raw, uint8_t i) {
  uint16_t lo = qtrMinUs[i];
  uint16_t hi = qtrMaxUs[i];

  if (hi <= lo + 5) return 0;

  raw = clampi(raw, lo, hi);

  float value;
  if (LINE_IS_WHITE) {
    value = (float)(hi - raw) / (float)(hi - lo);
  } else {
    value = (float)(raw - lo) / (float)(hi - lo);
  }

  return (uint16_t)(clampf(value, 0.0f, 1.0f) * 1000.0f);
}

bool qtrCalibrationValid() {
  return qtrCalLoaded;
}

void loadQTRCalibration() {
  prefs.begin("qtr", true);
  qtrCalLoaded = prefs.getBool("valid", false);

  if (qtrCalLoaded) {
    for (uint8_t i = 0; i < QTR_COUNT; ++i) {
      char a[5];
      char b[5];
      snprintf(a, sizeof(a), "n%d", i);
      snprintf(b, sizeof(b), "x%d", i);
      qtrMinUs[i] = prefs.getUShort(a, 0);
      qtrMaxUs[i] = prefs.getUShort(b, QTR_TIMEOUT_US);
      if (qtrMaxUs[i] <= qtrMinUs[i] + 30) qtrCalLoaded = false;
    }
  }

  prefs.end();
}

void sampleLine() {
  static const int16_t weights[8] = {
    -3500, -2500, -1500, -500,
     500,  1500,  2500,  3500
  };

  readQTR(lineState.raw);

  uint32_t total = 0;
  int32_t weighted = 0;
  uint8_t mask = 0;

  for (uint8_t i = 0; i < QTR_COUNT; ++i) {
    uint8_t logical = SENSOR_ORDER_REVERSED ? (QTR_COUNT - 1 - i) : i;
    lineState.strength[logical] = normalizedQTR(lineState.raw[i], i);

    if (lineState.strength[logical] >= LINE_MIN_STRENGTH) {
      mask |= (uint8_t)(1u << logical);
    }
  }

  for (uint8_t i = 2; i <= 5; ++i) {
    weighted += (int32_t)lineState.strength[i] * weights[i];
    total += lineState.strength[i];
  }

  lineState.mask = mask;
  lineState.valid = total >= LINE_VALID_TOTAL;

  if (lineState.valid) {
    lineState.error = clampf(
      (float)weighted / ((float)total * 3500.0f),
      -1.0f,
      1.0f
    );

    lastValidLineMs = millis();
    lastLineError = lineState.error;

    if (lineState.error > 0.03f) lastLineSide = 1;
    if (lineState.error < -0.03f) lastLineSide = -1;
  } else {
    lineState.error = lastLineError;
  }
}

void updateDerivative(float error) {
  uint32_t now = micros();

  errorHistory[errorHead] = {error, now};
  errorHead = (errorHead + 1) % ERROR_HISTORY;

  if (errorCount < ERROR_HISTORY) errorCount++;

  if (errorCount < 2) {
    filteredDerivative = 0.0f;
    return;
  }

  uint8_t newest = (errorHead + ERROR_HISTORY - 1) % ERROR_HISTORY;
  uint8_t oldest = (errorHead + ERROR_HISTORY - errorCount) % ERROR_HISTORY;

  uint32_t dtUs = errorHistory[newest].timeUs - errorHistory[oldest].timeUs;
  if (dtUs < 1000) dtUs = 1000;

  float rawD =
    (errorHistory[newest].error - errorHistory[oldest].error) /
    ((float)dtUs * 0.000001f);

  rawD = clampf(rawD, -100.0f, 100.0f);

  PIDConfig &cfg = activeCfg();

  filteredDerivative =
    cfg.dFilter * filteredDerivative +
    (1.0f - cfg.dFilter) * rawD;
}

float runPID(float error, PIDConfig &cfg) {
  updateDerivative(error);

  uint32_t now = micros();
  if (lastPidUs == 0) lastPidUs = now;

  uint32_t dtUs = now - lastPidUs;
  lastPidUs = now;

  float dt = clampf((float)dtUs * 0.000001f, 0.0005f, 0.05f);

  float p = cfg.kp * error;
  float d = cfg.kd * filteredDerivative;

  float candidateIntegral =
    clampf(
      integral + error * dt,
      -cfg.integralLimit,
      cfg.integralLimit
    );

  float unsaturated =
    cfg.sign * (p + cfg.ki * candidateIntegral + d);

  bool saturated = fabsf(unsaturated) > cfg.maxCorrection;

  if (!saturated || ((unsaturated > 0.0f) != (error * cfg.sign > 0.0f))) {
    integral = candidateIntegral;
  }

  float correction =
    cfg.sign * (
      p +
      cfg.ki * integral +
      d
    );

  return clampf(
    correction,
    -cfg.maxCorrection,
    cfg.maxCorrection
  );
}

float adaptiveBase(float error) {
  const float x[11] = {
    0.00f, 0.10f, 0.20f, 0.30f, 0.40f, 0.50f,
    0.60f, 0.70f, 0.80f, 0.90f, 1.00f
  };

  const float y[11] = {
    1.00f, 0.99f, 0.97f, 0.94f, 0.90f, 0.85f,
    0.78f, 0.70f, 0.60f, 0.50f, 0.40f
  };

  error = clampf(fabsf(error), 0.0f, 1.0f);

  float factor = y[10];

  for (uint8_t i = 1; i < 11; ++i) {
    if (error <= x[i]) {
      float t = (error - x[i - 1]) / (x[i] - x[i - 1]);
      factor = y[i - 1] + t * (y[i] - y[i - 1]);
      break;
    }
  }

  return clampf(
    lineCfg.base * factor,
    0.0f,
    255.0f
  );
}

void lineControlStep() {
  uint32_t now = micros();

  if (now - lastLineControlUs < LINE_PERIOD_US) return;
  lastLineControlUs = now;

  sampleLine();

  if (!lineState.valid) {
    if (millis() - lastValidLineMs > LINE_LOST_MS) {
      int search = lastLineSide > 0 ? 75 : -75;
      setTank(-search, search);
    } else {
      int base = (int)lineCfg.base;
      setTank(base, base);
    }
    return;
  }

  float correction = runPID(lineState.error, lineCfg);

  float base = adaptiveLine
    ? adaptiveBase(lineState.error)
    : clampf(lineCfg.base, 0.0f, 255.0f);

  int left = clampi((int)lroundf(base + correction), -255, 255);
  int right = clampi((int)lroundf(base - correction), -255, 255);

  setTank(left, right);
}

bool initToFs() {
  for (uint8_t i = 0; i < TOF_COUNT; ++i) {
    selectMux(TOF_CHANNEL[i]);
    tof[i].setBus(&Wire);
    tof[i].setTimeout(TOF_TIMEOUT_MS);

    if (!tof[i].init()) {
      deselectMux();
      return false;
    }

    tof[i].setMeasurementTimingBudget(20000);
    tof[i].startContinuous(TOF_PERIOD_MS);
  }

  deselectMux();
  return true;
}

bool readToFOne(uint8_t i) {
  selectMux(TOF_CHANNEL[i]);

  uint16_t d = tof[i].readRangeContinuousMillimeters();
  bool timeout = tof[i].timeoutOccurred();

  deselectMux();

  if (timeout || d == 0 || d == 65535) {
    tofValid[i] = false;
    return false;
  }

  tofMM[i] = d;
  tofValid[i] = true;
  return true;
}

void readAllToFsOnce() {
  for (uint8_t i = 0; i < TOF_COUNT; ++i) {
    readToFOne(i);
  }
}

void updateToFs() {
  uint32_t now = millis();

  if (now - lastToFMs < 5) return;
  lastToFMs = now;

  readToFOne(tofRoundRobin);
  tofRoundRobin = (tofRoundRobin + 1) % TOF_COUNT;
}

void updateIMU() {
  if (!imuReady) return;

  sensors_event_t a;
  sensors_event_t g;
  sensors_event_t t;

  mpu.getEvent(&a, &g, &t);

  uint32_t now = micros();

  if (lastIMUUs == 0) {
    lastIMUUs = now;
    return;
  }

  uint32_t dtUs = now - lastIMUUs;
  lastIMUUs = now;

  if (dtUs > 100000) return;

  float dt = (float)dtUs * 0.000001f;

  float zDegPerSec =
    g.gyro.z * 180.0f / PI;

  zDegPerSec -= gyroBiasZ;
  zDegPerSec *= gyroScale;

  yawDeg = wrapAngle(
    yawDeg + zDegPerSec * dt
  );
}

void calibrateGyro() {
  stopMotors();

  Serial.println("GYRO STILL 2 SEC");

  delay(500);

  float sum = 0.0f;
  uint16_t count = 0;

  uint32_t start = millis();

  while (millis() - start < 2000) {
    sensors_event_t a;
    sensors_event_t g;
    sensors_event_t t;

    mpu.getEvent(&a, &g, &t);
    sum += g.gyro.z * 180.0f / PI;
    ++count;
    delay(2);
  }

  if (count) {
    gyroBiasZ = sum / (float)count;
  }

  yawDeg = 0.0f;
  lastIMUUs = micros();

  Serial.printf("GYRO BIAS %.5f DEG/S\n", gyroBiasZ);
}

float wallError() {
  if (wallMode == 0) {
    if (!tofValid[LEFT_TOF] || !tofValid[RIGHT_TOF]) return 0.0f;
    return ((float)tofMM[LEFT_TOF] - (float)tofMM[RIGHT_TOF]) * 0.5f;
  }

  if (wallMode == 1) {
    if (!tofValid[LEFT_TOF]) return 0.0f;
    return (float)wallTargetMM - (float)tofMM[LEFT_TOF];
  }

  if (!tofValid[RIGHT_TOF]) return 0.0f;
  return (float)tofMM[RIGHT_TOF] - (float)wallTargetMM;
}

void wallControlStep() {
  updateToFs();

  if (
    frontSafety &&
    tofValid[FRONT_TOF] &&
    tofMM[FRONT_TOF] <= frontStopMM
  ) {
    stopMotors();
    return;
  }

  float error = wallError();
  float correction = runPID(error, wallCfg);

  int base = clampi((int)wallCfg.base, 0, 255);
  int left = clampi((int)lroundf(base + correction), -255, 255);
  int right = clampi((int)lroundf(base - correction), -255, 255);

  setTank(left, right);
}

void headingControlStep(bool turnMode) {
  updateIMU();

  float error = wrapAngle(
    headingTargetDeg - yawDeg
  );

  float correction = runPID(
    error,
    headingCfg
  );

  if (turnMode) {
    int left = clampi((int)lroundf(correction), -255, 255);
    int right = clampi((int)lroundf(-correction), -255, 255);

    if (fabsf(error) <= 1.5f) {
      stopMotors();
      mode = MODE_STOP;
      resetPID();
      return;
    }

    setTank(left, right);
    return;
  }

  int base = clampi((int)headingCfg.base, 0, 255);
  int left = clampi((int)lroundf(base + correction), -255, 255);
  int right = clampi((int)lroundf(base - correction), -255, 255);

  setTank(left, right);
}

void saveConfig() {
  prefs.begin("pid", false);

  const char *names[3] = {"line", "wall", "head"};
  PIDConfig *configs[3] = {&lineCfg, &wallCfg, &headingCfg};

  for (uint8_t i = 0; i < 3; ++i) {
    char k[24];

    snprintf(k, sizeof(k), "%s_kp", names[i]);
    prefs.putFloat(k, configs[i]->kp);
    snprintf(k, sizeof(k), "%s_ki", names[i]);
    prefs.putFloat(k, configs[i]->ki);
    snprintf(k, sizeof(k), "%s_kd", names[i]);
    prefs.putFloat(k, configs[i]->kd);
    snprintf(k, sizeof(k), "%s_base", names[i]);
    prefs.putFloat(k, configs[i]->base);
    snprintf(k, sizeof(k), "%s_ilim", names[i]);
    prefs.putFloat(k, configs[i]->integralLimit);
    snprintf(k, sizeof(k), "%s_df", names[i]);
    prefs.putFloat(k, configs[i]->dFilter);
    snprintf(k, sizeof(k), "%s_max", names[i]);
    prefs.putFloat(k, configs[i]->maxCorrection);
    snprintf(k, sizeof(k), "%s_sign", names[i]);
    prefs.putFloat(k, configs[i]->sign);
  }

  prefs.putUShort("wall_mm", wallTargetMM);
  prefs.putUShort("front_mm", frontStopMM);

  prefs.end();

  Serial.println("SAVE OK");
}

void loadConfig() {
  prefs.begin("pid", true);

  const char *names[3] = {"line", "wall", "head"};
  PIDConfig *configs[3] = {&lineCfg, &wallCfg, &headingCfg};

  for (uint8_t i = 0; i < 3; ++i) {
    char k[24];

    snprintf(k, sizeof(k), "%s_kp", names[i]);
    configs[i]->kp = prefs.getFloat(k, configs[i]->kp);
    snprintf(k, sizeof(k), "%s_ki", names[i]);
    configs[i]->ki = prefs.getFloat(k, configs[i]->ki);
    snprintf(k, sizeof(k), "%s_kd", names[i]);
    configs[i]->kd = prefs.getFloat(k, configs[i]->kd);
    snprintf(k, sizeof(k), "%s_base", names[i]);
    configs[i]->base = prefs.getFloat(k, configs[i]->base);
    snprintf(k, sizeof(k), "%s_ilim", names[i]);
    configs[i]->integralLimit = prefs.getFloat(k, configs[i]->integralLimit);
    snprintf(k, sizeof(k), "%s_df", names[i]);
    configs[i]->dFilter = prefs.getFloat(k, configs[i]->dFilter);
    snprintf(k, sizeof(k), "%s_max", names[i]);
    configs[i]->maxCorrection = prefs.getFloat(k, configs[i]->maxCorrection);
    snprintf(k, sizeof(k), "%s_sign", names[i]);
    configs[i]->sign = prefs.getFloat(k, configs[i]->sign);
  }

  wallTargetMM = prefs.getUShort("wall_mm", wallTargetMM);
  frontStopMM = prefs.getUShort("front_mm", frontStopMM);

  prefs.end();
}

void printConfig() {
  PIDConfig &c = activeCfg();
  const char *name =
    mode == MODE_LINE ? "LINE" :
    mode == MODE_WALL ? "WALL" :
    mode == MODE_HEADING ? "HEADING" :
    mode == MODE_TURN ? "TURN" : "STOP";

  Serial.printf(
    "%s KP=%.5f KI=%.5f KD=%.5f BASE=%.1f ILIM=%.3f DF=%.3f MAX=%.1f SIGN=%.1f\n",
    name,
    c.kp,
    c.ki,
    c.kd,
    c.base,
    c.integralLimit,
    c.dFilter,
    c.maxCorrection,
    c.sign
  );

  Serial.printf(
    "WALLMODE=%u TARGET=%u FRONTSTOP=%u FRONTSAFETY=%u ADAPT=%u\n",
    wallMode,
    wallTargetMM,
    frontStopMM,
    frontSafety,
    adaptiveLine
  );
}

void printSensors() {
  if (mode == MODE_LINE) {
    sampleLine();

    Serial.print("QTR:");
    for (uint8_t i = 0; i < QTR_COUNT; ++i) {
      Serial.printf(" %u", lineState.raw[i]);
    }

    Serial.print(" S:");
    for (uint8_t i = 0; i < QTR_COUNT; ++i) {
      Serial.printf(" %u", lineState.strength[i]);
    }

    Serial.printf(
      " MASK=0x%02X ERR=%.4f VALID=%u\n",
      lineState.mask,
      lineState.error,
      lineState.valid
    );
    return;
  }

  readAllToFsOnce();

  Serial.printf(
    "TOF L=%u(%u) F=%u(%u) R=%u(%u) WALLERR=%.2f\n",
    tofMM[LEFT_TOF],
    tofValid[LEFT_TOF],
    tofMM[FRONT_TOF],
    tofValid[FRONT_TOF],
    tofMM[RIGHT_TOF],
    tofValid[RIGHT_TOF],
    wallError()
  );

  updateIMU();

  Serial.printf(
    "YAW=%.3f TARGET=%.3f ERR=%.3f\n",
    yawDeg,
    headingTargetDeg,
    wrapAngle(headingTargetDeg - yawDeg)
  );
}

void setMode(ControlMode newMode) {
  stopMotors();
  resetPID();
  mode = newMode;

  if (mode == MODE_HEADING) {
    headingTargetDeg = yawDeg;
  }

  if (mode == MODE_TURN) {
    headingTargetDeg = yawDeg;
  }

  printConfig();
}

void parseFloatCommand(String key, float value) {
  if (key == "kp") activeCfg().kp = value;
  else if (key == "ki") activeCfg().ki = value;
  else if (key == "kd") activeCfg().kd = value;
  else if (key == "base") activeCfg().base = clampf(value, 0.0f, 255.0f);
  else if (key == "ilim") activeCfg().integralLimit = fabsf(value);
  else if (key == "df") activeCfg().dFilter = clampf(value, 0.0f, 0.99f);
  else if (key == "max") activeCfg().maxCorrection = clampf(fabsf(value), 1.0f, 255.0f);
  else if (key == "sign") activeCfg().sign = value < 0 ? -1.0f : 1.0f;
  else if (key == "target") headingTargetDeg = wrapAngle(value);
  else if (key == "wall") wallTargetMM = clampi((int)value, 20, 1000);
  else if (key == "front") frontStopMM = clampi((int)value, 20, 1000);
  else return;

  resetPID();
  printConfig();
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();

  if (cmd == "line") {
    setMode(MODE_LINE);
    return;
  }

  if (cmd == "wall") {
    setMode(MODE_WALL);
    return;
  }

  if (cmd == "heading") {
    setMode(MODE_HEADING);
    return;
  }

  if (cmd == "stop") {
    setMode(MODE_STOP);
    return;
  }

  if (cmd == "run") {
    if (mode == MODE_STOP) {
      Serial.println("SET MODE FIRST");
    }
    resetPID();
    return;
  }

  if (cmd == "cal") {
    calibrateQTR();
    return;
  }

  if (cmd == "gyrocal") {
    calibrateGyro();
    return;
  }

  if (cmd == "zero") {
    yawDeg = 0.0f;
    headingTargetDeg = 0.0f;
    resetPID();
    Serial.println("YAW ZERO");
    return;
  }

  if (cmd.startsWith("turn=")) {
    float delta = cmd.substring(5).toFloat();
    headingTargetDeg = wrapAngle(yawDeg + delta);
    resetPID();
    mode = MODE_TURN;
    Serial.printf("TURN TARGET %.2f\n", headingTargetDeg);
    return;
  }

  if (cmd == "save") {
    saveConfig();
    return;
  }

  if (cmd == "load") {
    loadConfig();
    resetPID();
    printConfig();
    return;
  }

  if (cmd == "sensors") {
    printSensors();
    return;
  }

  if (cmd == "p") {
    printConfig();
    return;
  }

  if (cmd == "center") {
    wallMode = 0;
    resetPID();
    printConfig();
    return;
  }

  if (cmd == "leftwall") {
    wallMode = 1;
    resetPID();
    printConfig();
    return;
  }

  if (cmd == "rightwall") {
    wallMode = 2;
    resetPID();
    printConfig();
    return;
  }

  if (cmd == "frontsafe=0") {
    frontSafety = false;
    printConfig();
    return;
  }

  if (cmd == "frontsafe=1") {
    frontSafety = true;
    printConfig();
    return;
  }

  if (cmd == "adapt=0") {
    adaptiveLine = false;
    printConfig();
    return;
  }

  if (cmd == "adapt=1") {
    adaptiveLine = true;
    printConfig();
    return;
  }

  if (cmd.startsWith("invfl=")) {
    motorInvert[0] = cmd.substring(6).toInt() != 0;
    return;
  }

  if (cmd.startsWith("invfr=")) {
    motorInvert[1] = cmd.substring(6).toInt() != 0;
    return;
  }

  if (cmd.startsWith("invrl=")) {
    motorInvert[2] = cmd.substring(6).toInt() != 0;
    return;
  }

  if (cmd.startsWith("invrr=")) {
    motorInvert[3] = cmd.substring(6).toInt() != 0;
    return;
  }

  int eq = cmd.indexOf('=');
  if (eq > 0) {
    String key = cmd.substring(0, eq);
    float value = cmd.substring(eq + 1).toFloat();
    parseFloatCommand(key, value);
    return;
  }

  if (cmd == "help" || cmd == "?") {
    Serial.println("line wall heading turn=90 stop");
    Serial.println("cal gyrocal zero sensors p save load");
    Serial.println("kp= ki= kd= base= ilim= df= max= sign=");
    Serial.println("target= wall= front=");
    Serial.println("center leftwall rightwall");
    Serial.println("frontsafe=0|1 adapt=0|1");
    Serial.println("invfl=0|1 invfr=0|1 invrl=0|1 invrr=0|1");
    return;
  }

  Serial.println("BAD CMD");
}

void readSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      handleCommand(commandLine);
      commandLine = "";
    } else if (commandLine.length() < 100) {
      commandLine += c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  if (!pca.begin()) {
    Serial.println("PCA9685 FAIL");
    while (true) delay(1000);
  }

  pca.setPWMFreq(50);
  pcaHigh(STBY_CH);

  ledcAttach(FL_PWM, 20000, 8);
  ledcAttach(FR_PWM, 20000, 8);
  ledcAttach(RL_PWM, 20000, 8);
  ledcAttach(RR_PWM, 20000, 8);

  for (uint8_t i = 0; i < QTR_COUNT; ++i) {
    pinMode(QTR_PINS[i], INPUT);
  }

  loadConfig();
  loadQTRCalibration();
  deselectMux();

  if (!mpu.begin()) {
    Serial.println("MPU FAIL");
    imuReady = false;
  } else {
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    calibrateGyro();
    imuReady = true;
  }

  bool tofOk = initToFs();
  Serial.println(tofOk ? "TOF OK" : "TOF FAIL");

  bool qtrOk = qtrCalibrationValid();
  Serial.println(qtrOk ? "QTR CAL LOADED" : "QTR CAL NEEDED");

  stopMotors();
  mode = MODE_STOP;

  Serial.println("URC UNIVERSAL PID TUNER");
  Serial.println("TYPE HELP");
  printConfig();
}

void loop() {
  readSerial();

  if (mode == MODE_LINE) {
    lineControlStep();
  } else if (mode == MODE_WALL) {
    wallControlStep();
  } else if (mode == MODE_HEADING) {
    headingControlStep(false);
  } else if (mode == MODE_TURN) {
    headingControlStep(true);
  } else {
    stopMotors();
  }

  uint32_t now = millis();
  if (now - lastPrintMs >= 200) {
    lastPrintMs = now;

    if (mode == MODE_LINE) {
      Serial.printf(
        "LINE e=%.4f d=%.4f\n",
        lineState.error,
        filteredDerivative
      );
    } else if (mode == MODE_WALL) {
      Serial.printf(
        "WALL L=%u F=%u R=%u e=%.2f d=%.2f\n",
        tofMM[LEFT_TOF],
        tofMM[FRONT_TOF],
        tofMM[RIGHT_TOF],
        wallError(),
        filteredDerivative
      );
    } else if (mode == MODE_HEADING || mode == MODE_TURN) {
      Serial.printf(
        "HEAD yaw=%.2f target=%.2f e=%.2f d=%.2f\n",
        yawDeg,
        headingTargetDeg,
        wrapAngle(headingTargetDeg - yawDeg),
        filteredDerivative
      );
    }
  }
}
