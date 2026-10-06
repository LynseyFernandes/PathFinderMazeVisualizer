#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>

constexpr uint8_t SDA_PIN = 21;
constexpr uint8_t SCL_PIN = 22;
constexpr uint8_t PCA_ADDR = 0x40;

constexpr uint8_t QTR_COUNT = 8;
const uint8_t QTR_PINS[QTR_COUNT] = {4, 16, 17, 18, 19, 27, 32, 33};

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

constexpr bool MOTOR_INVERT_FL = false;
constexpr bool MOTOR_INVERT_FR = true;
constexpr bool MOTOR_INVERT_RL = false;
constexpr bool MOTOR_INVERT_RR = true;

constexpr bool LINE_IS_WHITE = true;
constexpr bool SENSOR_ORDER_REVERSED = false;

constexpr uint16_t QTR_CHARGE_US = 10;
constexpr uint16_t QTR_TIMEOUT_US = 3000;
constexpr uint16_t CALIBRATION_MS = 8000;

constexpr uint8_t TRACK_0 = 2;
constexpr uint8_t TRACK_1 = 3;
constexpr uint8_t TRACK_2 = 4;
constexpr uint8_t TRACK_3 = 5;

constexpr uint16_t LINE_MIN_STRENGTH = 80;
constexpr uint16_t LINE_VALID_TOTAL = 140;
constexpr uint16_t SIDE_TURN_STRENGTH = 600;
constexpr uint16_t LOST_TIMEOUT_MS = 250;

constexpr uint8_t ERROR_BUFFER_SIZE = 4;
constexpr uint32_t CONTROL_PERIOD_US = 3000;

constexpr int16_t PWM_MIN = 0;
constexpr int16_t PWM_MAX = 255;

constexpr float DEFAULT_KP = 80.0f;
constexpr float DEFAULT_KI = 0.0f;
constexpr float DEFAULT_KD = 0.03f;
constexpr float DEFAULT_BASE = 135.0f;
constexpr float DEFAULT_MIN_BASE = 90.0f;
constexpr float DEFAULT_MAX_BASE = 230.0f;
constexpr float DEFAULT_D_FILTER = 0.65f;

struct Calibration {
  uint16_t minUs[QTR_COUNT];
  uint16_t maxUs[QTR_COUNT];
  bool valid;
};

struct LineSample {
  uint16_t rawUs[QTR_COUNT];
  uint16_t strength[QTR_COUNT];
  uint8_t mask;
  float error;
  bool valid;
};

struct ErrorSample {
  float error;
  uint32_t timeUs;
};

struct PIDConfig {
  float kp;
  float ki;
  float kd;
  float base;
  float minBase;
  float maxBase;
  float dFilter;
  float integralLimit;
};

Adafruit_PWMServoDriver pca(PCA_ADDR);
Preferences prefs;
Calibration cal{};
LineSample line{};
PIDConfig cfg{
  DEFAULT_KP,
  DEFAULT_KI,
  DEFAULT_KD,
  DEFAULT_BASE,
  DEFAULT_MIN_BASE,
  DEFAULT_MAX_BASE,
  DEFAULT_D_FILTER,
  0.8f
};

ErrorSample history[ERROR_BUFFER_SIZE]{};
uint8_t historyHead = 0;
uint8_t historyCount = 0;
float integral = 0.0f;
float filteredDerivative = 0.0f;
float lastSeenError = 0.0f;
int8_t lastLineSide = 1;
uint8_t motorState[4] = {0, 0, 0, 0};
uint32_t lastControlUs = 0;
uint32_t lastValidLineMs = 0;
uint32_t lastPrintMs = 0;
bool running = false;
bool adaptiveEnabled = true;
bool initialized = false;
String commandLine;

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

void pcaHigh(uint8_t ch) {
  pca.setPin(ch, 4095, false);
}

void pcaLow(uint8_t ch) {
  pca.setPin(ch, 0, false);
}

void setMotorDir(uint8_t a, uint8_t b, bool forward, bool invert) {
  if (invert) forward = !forward;
  if (forward) {
    pcaHigh(a);
    pcaLow(b);
  } else {
    pcaLow(a);
    pcaHigh(b);
  }
}

void setMotorOutput(uint8_t index, uint8_t pwmPin, uint8_t a, uint8_t b, bool invert, int command) {
  command = clampi(command, -PWM_MAX, PWM_MAX);

  if (command == 0) {
    if (motorState[index] != 0) {
      pcaLow(a);
      pcaLow(b);
      motorState[index] = 0;
    }
    ledcWrite(pwmPin, 0);
    return;
  }

  uint8_t state = command > 0 ? 1 : 2;
  if (motorState[index] != state) {
    setMotorDir(a, b, command > 0, invert);
    motorState[index] = state;
  }

  ledcWrite(pwmPin, abs(command));
}

void setTank(int left, int right) {
  setMotorOutput(0, FL_PWM, FL_A, FL_B, MOTOR_INVERT_FL, left);
  setMotorOutput(1, FR_PWM, FR_A, FR_B, MOTOR_INVERT_FR, right);
  setMotorOutput(2, RL_PWM, RL_A, RL_B, MOTOR_INVERT_RL, left);
  setMotorOutput(3, RR_PWM, RR_A, RR_B, MOTOR_INVERT_RR, right);
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

void resetControllerState() {
  memset(history, 0, sizeof(history));
  historyHead = 0;
  historyCount = 0;
  integral = 0.0f;
  filteredDerivative = 0.0f;
  lastSeenError = 0.0f;
  lastLineSide = 1;
}

void saveCalibration() {
  prefs.begin("qtr", false);
  prefs.putBool("valid", cal.valid);
  for (uint8_t i = 0; i < QTR_COUNT; ++i) {
    char k1[5], k2[5];
    snprintf(k1, sizeof(k1), "n%d", i);
    snprintf(k2, sizeof(k2), "x%d", i);
    prefs.putUShort(k1, cal.minUs[i]);
    prefs.putUShort(k2, cal.maxUs[i]);
  }
  prefs.end();
}

bool loadCalibration() {
  prefs.begin("qtr", true);
  cal.valid = prefs.getBool("valid", false);
  if (cal.valid) {
    for (uint8_t i = 0; i < QTR_COUNT; ++i) {
      char k1[5], k2[5];
      snprintf(k1, sizeof(k1), "n%d", i);
      snprintf(k2, sizeof(k2), "x%d", i);
      cal.minUs[i] = prefs.getUShort(k1, 0);
      cal.maxUs[i] = prefs.getUShort(k2, QTR_TIMEOUT_US);
      if (cal.maxUs[i] <= cal.minUs[i] + 30) cal.valid = false;
    }
  }
  prefs.end();
  return cal.valid;
}

void calibrateQTR() {
  stopMotors();
  for (uint8_t i = 0; i < QTR_COUNT; ++i) {
    cal.minUs[i] = QTR_TIMEOUT_US;
    cal.maxUs[i] = 0;
  }

  Serial.println("CAL START");
  Serial.println("MOVE SENSOR OVER LINE AND FLOOR FOR 8 SEC");

  uint32_t start = millis();
  uint16_t samples[QTR_COUNT];
  while (millis() - start < CALIBRATION_MS) {
    readQTR(samples);
    for (uint8_t i = 0; i < QTR_COUNT; ++i) {
      if (samples[i] < cal.minUs[i]) cal.minUs[i] = samples[i];
      if (samples[i] > cal.maxUs[i]) cal.maxUs[i] = samples[i];
    }
  }

  cal.valid = true;
  for (uint8_t i = 0; i < QTR_COUNT; ++i) {
    if (cal.maxUs[i] <= cal.minUs[i] + 30) cal.valid = false;
  }

  if (cal.valid) {
    saveCalibration();
    Serial.println("CAL OK");
  } else {
    Serial.println("CAL BAD");
  }
}

uint16_t normalize(uint16_t raw, uint8_t i) {
  uint16_t lo = cal.minUs[i];
  uint16_t hi = cal.maxUs[i];
  if (hi <= lo + 5) return 0;

  raw = clampi(raw, lo, hi);

  float v;
  if (LINE_IS_WHITE) {
    v = (float)(hi - raw) / (float)(hi - lo);
  } else {
    v = (float)(raw - lo) / (float)(hi - lo);
  }

  return (uint16_t)(clampf(v, 0.0f, 1.0f) * 1000.0f);
}

void sampleLine() {
  static const int16_t weights[8] = {
    -3500, -2500, -1500, -500,
     500,  1500,  2500,  3500
  };

  int32_t weighted = 0;
  uint32_t total = 0;
  uint8_t mask = 0;

  readQTR(line.rawUs);

  for (uint8_t i = 0; i < QTR_COUNT; ++i) {
    uint8_t logical = SENSOR_ORDER_REVERSED ? (QTR_COUNT - 1 - i) : i;
    line.strength[logical] = normalize(line.rawUs[i], logical);

    if (line.strength[logical] >= LINE_MIN_STRENGTH) {
      mask |= (uint8_t)(1u << logical);
    }
  }

  for (uint8_t i = TRACK_0; i <= TRACK_3; ++i) {
    weighted += (int32_t)line.strength[i] * weights[i];
    total += line.strength[i];
  }

  line.mask = mask;
  line.valid = total >= LINE_VALID_TOTAL;

  if (line.valid) {
    line.error = clampf((float)weighted / ((float)total * 3500.0f), -1.0f, 1.0f);
    lastSeenError = line.error;
    lastValidLineMs = millis();
    if (line.error > 0.03f) lastLineSide = 1;
    if (line.error < -0.03f) lastLineSide = -1;
  } else {
    line.error = lastSeenError;
  }
}

void updateDerivative(float error) {
  uint32_t now = micros();
  history[historyHead] = {error, now};
  historyHead = (historyHead + 1) % ERROR_BUFFER_SIZE;
  if (historyCount < ERROR_BUFFER_SIZE) historyCount++;

  if (historyCount < 2) {
    filteredDerivative = 0.0f;
    return;
  }

  uint8_t newest = (historyHead + ERROR_BUFFER_SIZE - 1) % ERROR_BUFFER_SIZE;
  uint8_t oldest = (historyHead + ERROR_BUFFER_SIZE - historyCount) % ERROR_BUFFER_SIZE;
  uint32_t dtUs = history[newest].timeUs - history[oldest].timeUs;
  if (dtUs < 1000) dtUs = 1000;

  float rawD = (history[newest].error - history[oldest].error) / ((float)dtUs * 0.000001f);
  rawD = clampf(rawD, -80.0f, 80.0f);
  filteredDerivative =
      cfg.dFilter * filteredDerivative
    + (1.0f - cfg.dFilter) * rawD;
}

float adaptiveBase(float absError) {
  const float x[] = {0.00f, 0.10f, 0.20f, 0.30f, 0.40f, 0.50f, 0.60f, 0.70f, 0.80f, 0.90f, 1.00f};
  const float y[] = {1.00f, 0.99f, 0.97f, 0.94f, 0.90f, 0.85f, 0.78f, 0.70f, 0.60f, 0.50f, 0.40f};

  absError = clampf(absError, 0.0f, 1.0f);

  float factor = y[10];
  for (uint8_t i = 1; i < 11; ++i) {
    if (absError <= x[i]) {
      float t = (absError - x[i - 1]) / (x[i] - x[i - 1]);
      factor = y[i - 1] + t * (y[i] - y[i - 1]);
      break;
    }
  }

  float base = cfg.base * factor;
  return clampf(base, cfg.minBase, cfg.maxBase);
}

void lineStep() {
  uint32_t now = micros();
  if ((uint32_t)(now - lastControlUs) < CONTROL_PERIOD_US) return;
  lastControlUs = now;

  sampleLine();

  if (!line.valid) {
    if (millis() - lastValidLineMs > LOST_TIMEOUT_MS) {
      int search = lastLineSide > 0 ? 85 : -85;
      setTank(-search, search);
    } else {
      setTank((int)cfg.base, (int)cfg.base);
    }
    return;
  }

  updateDerivative(line.error);

  float dt = CONTROL_PERIOD_US * 0.000001f;
  integral += line.error * dt;
  integral = clampf(integral, -cfg.integralLimit, cfg.integralLimit);

  float correction =
      cfg.kp * line.error
    + cfg.ki * integral
    + cfg.kd * filteredDerivative;

  correction = clampf(correction, -255.0f, 255.0f);

  float base = adaptiveEnabled
    ? adaptiveBase(fabsf(line.error))
    : clampf(cfg.base, cfg.minBase, cfg.maxBase);

  int left = clampi((int)lroundf(base + correction), -255, 255);
  int right = clampi((int)lroundf(base - correction), -255, 255);

  setTank(left, right);
}

void printSensors() {
  sampleLine();
  Serial.print("raw:");
  for (uint8_t i = 0; i < QTR_COUNT; ++i) {
    Serial.printf(" %u", line.rawUs[i]);
  }
  Serial.print(" strength:");
  for (uint8_t i = 0; i < QTR_COUNT; ++i) {
    Serial.printf(" %u", line.strength[i]);
  }
  Serial.printf(" mask=0x%02X err=%.4f valid=%d\n", line.mask, line.error, line.valid);
}

void printConfig() {
  Serial.printf(
    "KP=%.5f KI=%.5f KD=%.5f BASE=%.1f MIN=%.1f MAX=%.1f DF=%.3f I=%.3f\n",
    cfg.kp, cfg.ki, cfg.kd, cfg.base, cfg.minBase, cfg.maxBase, cfg.dFilter, cfg.integralLimit
  );
}

void saveConfig() {
  prefs.begin("pid", false);
  prefs.putFloat("kp", cfg.kp);
  prefs.putFloat("ki", cfg.ki);
  prefs.putFloat("kd", cfg.kd);
  prefs.putFloat("base", cfg.base);
  prefs.putFloat("min", cfg.minBase);
  prefs.putFloat("max", cfg.maxBase);
  prefs.putFloat("df", cfg.dFilter);
  prefs.putFloat("ilim", cfg.integralLimit);
  prefs.end();
  Serial.println("PID SAVE OK");
}

bool loadConfig() {
  prefs.begin("pid", true);
  cfg.kp = prefs.getFloat("kp", DEFAULT_KP);
  cfg.ki = prefs.getFloat("ki", DEFAULT_KI);
  cfg.kd = prefs.getFloat("kd", DEFAULT_KD);
  cfg.base = prefs.getFloat("base", DEFAULT_BASE);
  cfg.minBase = prefs.getFloat("min", DEFAULT_MIN_BASE);
  cfg.maxBase = prefs.getFloat("max", DEFAULT_MAX_BASE);
  cfg.dFilter = prefs.getFloat("df", DEFAULT_D_FILTER);
  cfg.integralLimit = prefs.getFloat("ilim", 0.8f);
  prefs.end();
  return true;
}

bool setNamedFloat(const String &name, float value) {
  if (name == "kp") cfg.kp = value;
  else if (name == "ki") cfg.ki = value;
  else if (name == "kd") cfg.kd = value;
  else if (name == "base") cfg.base = value;
  else if (name == "min") cfg.minBase = value;
  else if (name == "max") cfg.maxBase = value;
  else if (name == "df") cfg.dFilter = clampf(value, 0.0f, 0.99f);
  else if (name == "ilim") cfg.integralLimit = fabsf(value);
  else return false;
  resetControllerState();
  return true;
}

void help() {
  Serial.println("c=calibrate");
  Serial.println("r=run");
  Serial.println("s=stop");
  Serial.println("p=print config");
  Serial.println("v=print sensors");
  Serial.println("save=save pid");
  Serial.println("load=load pid");
  Serial.println("adapt=0|1");
  Serial.println("kp=val kd=val ki=val base=val min=val max=val df=val ilim=val");
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  if (cmd.length() == 0) return;

  if (cmd == "c") {
    running = false;
    stopMotors();
    calibrateQTR();
    resetControllerState();
    return;
  }

  if (cmd == "r") {
    if (!cal.valid) {
      Serial.println("NO CAL");
      return;
    }
    resetControllerState();
    running = true;
    Serial.println("RUN");
    return;
  }

  if (cmd == "s") {
    running = false;
    stopMotors();
    Serial.println("STOP");
    return;
  }

  if (cmd == "p") {
    printConfig();
    return;
  }

  if (cmd == "v") {
    printSensors();
    return;
  }

  if (cmd == "save") {
    saveConfig();
    return;
  }

  if (cmd == "load") {
    loadConfig();
    resetControllerState();
    printConfig();
    return;
  }

  if (cmd.startsWith("adapt=")) {
    adaptiveEnabled = cmd.substring(6).toInt() != 0;
    Serial.printf("ADAPT=%d\n", adaptiveEnabled);
    return;
  }

  if (cmd == "help" || cmd == "?") {
    help();
    return;
  }

  int sep = cmd.indexOf('=');
  if (sep > 0) {
    String name = cmd.substring(0, sep);
    float value = cmd.substring(sep + 1).toFloat();
    if (setNamedFloat(name, value)) {
      printConfig();
      return;
    }
  }

  Serial.println("BAD CMD");
}

void readSerialCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      handleCommand(commandLine);
      commandLine = "";
    } else if (commandLine.length() < 80) {
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
    while (true) {
      stopMotors();
      delay(1000);
    }
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

  loadCalibration();
  loadConfig();
  resetControllerState();
  stopMotors();

  initialized = true;

  Serial.println("URC LINE PID TUNER");
  Serial.println("115200");
  Serial.println(cal.valid ? "QTR CAL LOADED" : "QTR CAL NEEDED");
  printConfig();
  help();
}

void loop() {
  if (!initialized) return;

  readSerialCommands();

  if (running) {
    lineStep();
  } else {
    stopMotors();
  }

  if (millis() - lastPrintMs >= 250) {
    lastPrintMs = millis();
    if (running) {
      Serial.printf("e=%.4f d=%.3f base=%.1f mask=0x%02X\n", line.error, filteredDerivative, adaptiveEnabled ? adaptiveBase(fabsf(line.error)) : clampf(cfg.base, cfg.minBase, cfg.maxBase), line.mask);
    }
  }
}
