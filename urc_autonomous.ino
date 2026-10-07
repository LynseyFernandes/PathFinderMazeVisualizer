#include <Arduino.h>
#include <Wire.h>
#include <QTRSensors.h>
#include <VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>
#include <Bluepad32.h>

constexpr uint8_t DAY_PROFILE = 1;

constexpr uint8_t SDA_PIN = 21;
constexpr uint8_t SCL_PIN = 22;
constexpr uint8_t PCA9685_ADDR = 0x40;
constexpr uint8_t PCA9548A_ADDR = 0x70;

constexpr uint8_t QTR_COUNT = 8;
constexpr uint8_t MOTOR_COUNT = 4;
constexpr uint8_t TOF_COUNT = 3;
constexpr uint8_t MAX_W = 10;
constexpr uint8_t MAX_H = 10;
constexpr uint16_t MAX_CELLS = MAX_W * MAX_H;
constexpr uint16_t MAX_ASTAR_STATES = MAX_CELLS * 4;
constexpr uint16_t MAX_HISTORY = MAX_CELLS;
constexpr uint16_t MAX_PLAN = MAX_CELLS;

const uint8_t QTR_PINS[QTR_COUNT] = {4, 16, 17, 18, 19, 27, 32, 33};
const uint8_t MOTOR_PWM_PINS[MOTOR_COUNT] = {13, 14, 25, 26};
const uint8_t PCA_MOTOR_DIR_A[MOTOR_COUNT] = {3, 5, 7, 9};
const uint8_t PCA_MOTOR_DIR_B[MOTOR_COUNT] = {4, 6, 8, 10};
const bool MOTOR_INVERT[MOTOR_COUNT] = {false, true, false, true};
constexpr uint8_t PCA_STBY_CH = 11;
const uint8_t TOF_MUX_CHANNEL[TOF_COUNT] = {0, 1, 2};

constexpr uint8_t DIR_N = 0;
constexpr uint8_t DIR_E = 1;
constexpr uint8_t DIR_S = 2;
constexpr uint8_t DIR_W = 3;
const int8_t DX[4] = {0, 1, 0, -1};
const int8_t DY[4] = {1, 0, -1, 0};
const uint8_t WALL_BIT[4] = {1, 2, 4, 8};
const uint8_t WALL_OPP[4] = {4, 8, 1, 2};

constexpr uint8_t DAY1_W = 10;
constexpr uint8_t DAY1_H = 5;
constexpr uint8_t DAY2_W = 10;
constexpr uint8_t DAY2_H = 10;

constexpr uint8_t DAY1_GOAL_X0 = 8;
constexpr uint8_t DAY1_GOAL_X1 = 9;
constexpr uint8_t DAY1_GOAL_Y0 = 2;
constexpr uint8_t DAY1_GOAL_Y1 = 3;

constexpr uint8_t DAY2_GOAL_X0 = 4;
constexpr uint8_t DAY2_GOAL_X1 = 5;
constexpr uint8_t DAY2_GOAL_Y0 = 4;
constexpr uint8_t DAY2_GOAL_Y1 = 5;

constexpr uint8_t START_X = 0;
constexpr uint8_t START_Y = 0;
constexpr uint8_t START_HEADING = DIR_E;

constexpr uint16_t QTR_TIMEOUT_US = 2200;
constexpr uint16_t QTR_CALIBRATION_MS = 5000;
constexpr uint16_t LINE_PRESENT = 85;
constexpr uint16_t LINE_STRONG = 650;
constexpr uint16_t LINE_CENTER_STRONG = 500;
constexpr uint16_t LINE_DARK_FLOOR_STRONG = 760;
constexpr uint8_t LINE_DARK_MIN_COUNT = 7;
constexpr bool LINE_IS_WHITE = false;
constexpr bool QTR_S0_IS_RIGHT = true;

constexpr uint16_t LINE_ENTRY_DARK_MS = 140;
constexpr uint16_t LINE_ENTRY_TIMEOUT_MS = 1800;
constexpr uint8_t LINE_ENTRY_TOF_CONFIRMATIONS = 3;
constexpr uint16_t LINE_TRANSITION_BLEND_MS = 320;
constexpr uint16_t LINE_LOST_HOLD_MS = 420;
constexpr uint16_t LINE_LOST_SEARCH_MS = 1300;
constexpr uint16_t LINE_LOST_FAULT_MS = 2800;
constexpr uint16_t LINE_SEARCH_PWM = 82;
constexpr uint16_t TRANSITION_FORWARD_PWM = 108;
constexpr uint16_t LINE_SHARP_INNER_PWM = 55;
constexpr uint16_t LINE_SHARP_OUTER_PWM = 145;
constexpr float LINE_KP = 165.0f;
constexpr float LINE_KI = 0.0f;
constexpr float LINE_KD = 0.022f;
constexpr float LINE_MAX_CORRECTION = 195.0f;
constexpr uint8_t ERROR_BUFFER_SIZE = 5;

constexpr uint16_t TOF_TIMING_US = 25000;
constexpr uint16_t TOF_PERIOD_MS = 20;
constexpr uint16_t TOF_TIMEOUT_MS = 35;
constexpr uint16_t TOF_WALL_ON_MM = 145;
constexpr uint16_t TOF_WALL_OFF_MM = 175;
constexpr uint16_t ENTRY_SIDE_MIN_MM = 25;
constexpr uint16_t ENTRY_SIDE_MAX_MM = 230;
constexpr uint16_t ENTRY_SIDE_SUM_MIN_MM = 100;
constexpr uint16_t ENTRY_SIDE_SUM_MAX_MM = 460;
constexpr uint16_t ENTRY_FRONT_MAX_MM = 340;
constexpr uint16_t TOF_STALE_MS = 160;

constexpr uint16_t MAZE_PWM = 145;
constexpr uint16_t MAZE_TURN_PWM = 150;
constexpr uint16_t MAZE_TURN_PWM_SLOW = 88;
constexpr uint16_t MAZE_CELL_MS = 520;
constexpr uint16_t MAZE_CELL_MIN_MS = 235;
constexpr uint16_t MAZE_FRONT_BRAKE_MM = 105;
constexpr uint16_t MAZE_FRONT_STOP_MM = 72;
constexpr float WALL_KP = 1.10f;
constexpr float WALL_KD = 0.018f;
constexpr int16_t WALL_MAX_CORRECTION = 55;
constexpr float HEADING_KP = 2.1f;
constexpr float HEADING_KD = 0.06f;
constexpr int16_t HEADING_MAX_CORRECTION = 38;
constexpr uint16_t MAZE_TURN_TIMEOUT_MS = 1200;
constexpr uint16_t TURN_COST_90 = 8;
constexpr uint8_t FLOOD_INF = 255;
constexpr bool USE_CHEBYSHEV = false;

constexpr uint8_t JUNCTION_STRAIGHT_FIRST = 0;
constexpr uint8_t JUNCTION_LEFT_FIRST = 1;
constexpr uint8_t JUNCTION_RIGHT_FIRST = 2;
constexpr uint8_t DAY2_JUNCTION_POLICY = JUNCTION_STRAIGHT_FIRST;

constexpr bool DEBUG_SERIAL = true;
constexpr uint16_t CONTROLLER_STOP_HOLD_MS = 350;
constexpr uint16_t CONTROLLER_RESET_HOLD_MS = 1200;
constexpr uint16_t CONTROLLER_FORCE_MAZE_HOLD_MS = 1200;
constexpr uint16_t CONTROLLER_CALIBRATE_HOLD_MS = 1500;

QTRSensors qtr;
Adafruit_PWMServoDriver pca(PCA9685_ADDR);
Adafruit_MPU6050 mpu;
Preferences prefs;
VL53L0X tof[TOF_COUNT];

struct LineState {
  uint16_t raw[QTR_COUNT]{};
  uint16_t strength[QTR_COUNT]{};
  uint8_t mask = 0;
  uint32_t total = 0;
  float error = 0.0f;
  float previousError = 0.0f;
  float derivative = 0.0f;
  bool detected = false;
};

struct ToFState {
  uint16_t mm[TOF_COUNT]{};
  bool valid[TOF_COUNT]{};
  bool wall[TOF_COUNT]{};
  uint32_t stamp[TOF_COUNT]{};
};

struct MazeCell {
  uint8_t known = 0;
  uint8_t blocked = 0;
  uint8_t flags = 0;
};

struct ParentCell {
  int8_t x = -1;
  int8_t y = -1;
  bool valid = false;
};

struct GoalRegion {
  uint8_t x0;
  uint8_t x1;
  uint8_t y0;
  uint8_t y1;
};

struct ANode {
  uint16_t g = 65535;
  uint16_t f = 65535;
  int16_t parent = -1;
  int16_t heapPos = -1;
  bool closed = false;
};

struct MotionSegment {
  uint8_t type = 0;
  uint8_t count = 0;
  int8_t turn = 0;
};

LineState lineState;
ToFState tofState;
MazeCell maze[MAX_H][MAX_W];
ParentCell parents[MAX_H][MAX_W];
uint8_t flood[MAX_H][MAX_W];
uint16_t floodQueue[MAX_CELLS];
float errorBuf[ERROR_BUFFER_SIZE];
uint32_t errorTime[ERROR_BUFFER_SIZE];
uint8_t errorHead = 0;
uint8_t errorCount = 0;

uint16_t qtrRaw[QTR_COUNT];
uint16_t qtrMin[QTR_COUNT];
uint16_t qtrMax[QTR_COUNT];
bool calibrationValid = false;

ANode astar[MAX_ASTAR_STATES];
uint16_t astarHeap[MAX_ASTAR_STATES];
uint16_t astarHeapSize = 0;
int16_t astarSolution[MAX_ASTAR_STATES];
uint16_t astarSolutionSize = 0;
MotionSegment plan[MAX_PLAN];
uint16_t planSize = 0;

uint16_t history[MAX_HISTORY];
uint16_t historySize = 0;

uint8_t mazeW = DAY1_W;
uint8_t mazeH = DAY1_H;
GoalRegion goal{DAY1_GOAL_X0, DAY1_GOAL_X1, DAY1_GOAL_Y0, DAY1_GOAL_Y1};

int8_t mazeX = START_X;
int8_t mazeY = START_Y;
uint8_t mazeHeading = START_HEADING;
uint8_t targetHeading = START_HEADING;

uint32_t lastTofService = 0;
uint8_t tofServiceIndex = 0;
uint32_t lineDarkStart = 0;
uint8_t entryTofHits = 0;
uint32_t transitionStart = 0;
int16_t transitionLeft = 120;
int16_t transitionRight = 120;
uint32_t lineLostStart = 0;
uint8_t lineTurnState = 0;
uint32_t lineTurnStart = 0;
uint32_t junctionStart = 0;
uint8_t junctionStage = 0;
bool lineTurnSawGap = false;
bool lineTurnLeft = false;
uint8_t junctionCounter = 0;

float gyroBiasZ = 0.0f;
float yawDeg = 0.0f;
float turnTargetDeg = 0.0f;
float turnSign = 0.0f;
float wallPrevError = 0.0f;
uint32_t wallPrevTime = 0;
float headingPrevError = 0.0f;
uint32_t headingPrevTime = 0;
uint32_t yawTimeUs = 0;
uint32_t turnStartMs = 0;
uint32_t moveStartMs = 0;

bool pcaReady = false;
bool tofReady = false;
bool mpuReady = false;
bool motorDirValid[MOTOR_COUNT]{};
bool motorDirState[MOTOR_COUNT]{};
int16_t lastLeftCommand = 0;
int16_t lastRightCommand = 0;

bool startupCalibrationNeeded = false;
ControllerPtr controllerSlots[BP32_MAX_GAMEPADS]{};
ControllerPtr primaryController = nullptr;
uint8_t controllerComboAction = 0;
uint32_t controllerComboStart = 0;
bool controllerComboConsumed = false;

enum RobotState : uint8_t {
  LINE_FOLLOW,
  LINE_TO_MAZE,
  MAZE_SELECT,
  MAZE_TURN,
  MAZE_MOVE,
  FINISHED,
  OPERATOR_STOP,
  FAULT
};

RobotState state = LINE_FOLLOW;
RobotState stateBeforeOperatorStop = LINE_FOLLOW;

float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

int clampi(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

bool inMaze(int x, int y) {
  return x >= 0 && x < mazeW && y >= 0 && y < mazeH;
}

bool inGoal(int x, int y) {
  return x >= goal.x0 && x <= goal.x1 && y >= goal.y0 && y <= goal.y1;
}

void pcaDigital(uint8_t ch, bool on) {
  if (!pcaReady) return;
  pca.setPin(ch, on ? 4095 : 0, false);
}

void motorDirection(uint8_t i, bool forward) {
  if (MOTOR_INVERT[i]) forward = !forward;
  if (motorDirValid[i] && motorDirState[i] == forward) return;
  pcaDigital(PCA_MOTOR_DIR_A[i], forward);
  pcaDigital(PCA_MOTOR_DIR_B[i], !forward);
  motorDirState[i] = forward;
  motorDirValid[i] = true;
}

void setMotor(uint8_t i, int value) {
  value = clampi(value, -255, 255);
  if (value == 0) {
    ledcWrite(MOTOR_PWM_PINS[i], 0);
    pcaDigital(PCA_MOTOR_DIR_A[i], false);
    pcaDigital(PCA_MOTOR_DIR_B[i], false);
    motorDirValid[i] = false;
    return;
  }
  motorDirection(i, value > 0);
  ledcWrite(MOTOR_PWM_PINS[i], abs(value));
}

void setDrive(int left, int right) {
  setMotor(0, left);
  setMotor(2, left);
  setMotor(1, right);
  setMotor(3, right);
  lastLeftCommand = left;
  lastRightCommand = right;
}

void stopDrive() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    ledcWrite(MOTOR_PWM_PINS[i], 0);
    pcaDigital(PCA_MOTOR_DIR_A[i], false);
    pcaDigital(PCA_MOTOR_DIR_B[i], false);
    motorDirValid[i] = false;
  }
  lastLeftCommand = 0;
  lastRightCommand = 0;
}

void resetYawReference();
void resetMaze();
void resetLineControl();
void serviceToFs(bool forceAll);
void updateMazeMap();

void hardFault() {
  stopDrive();
  pcaDigital(PCA_STBY_CH, false);
  state = FAULT;
}

void resetMotionControl() {
  resetYawReference();
  wallPrevError = 0.0f;
  wallPrevTime = millis();
  headingPrevError = 0.0f;
  headingPrevTime = millis();
}

void resetMission() {
  stopDrive();
  pcaDigital(PCA_STBY_CH, true);
  resetLineControl();
  resetMaze();
  resetMotionControl();
  state = LINE_FOLLOW;
}

void forceMazeStart() {
  stopDrive();
  pcaDigital(PCA_STBY_CH, true);
  resetLineControl();
  resetMaze();
  resetMotionControl();
  serviceToFs(true);
  updateMazeMap();
  state = MAZE_SELECT;
}

void selectMux(uint8_t ch) {
  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write((uint8_t)(1u << ch));
  Wire.endTransmission();
}

bool initToFs() {
  for (uint8_t i = 0; i < TOF_COUNT; ++i) {
    selectMux(TOF_MUX_CHANNEL[i]);
    if (!tof[i].init()) return false;
    tof[i].setTimeout(TOF_TIMEOUT_MS);
    if (!tof[i].setMeasurementTimingBudget(TOF_TIMING_US)) return false;
    tof[i].startContinuous(TOF_PERIOD_MS);
    tofState.valid[i] = false;
    tofState.wall[i] = false;
    tofState.stamp[i] = 0;
  }
  return true;
}

void readToF(uint8_t i) {
  selectMux(TOF_MUX_CHANNEL[i]);
  uint16_t d = tof[i].readRangeContinuousMillimeters();
  uint32_t now = millis();
  if (tof[i].timeoutOccurred() || d == 0 || d == 65535) {
    tofState.valid[i] = false;
    return;
  }
  tofState.valid[i] = true;
  tofState.mm[i] = d;
  tofState.stamp[i] = now;
  if (tofState.wall[i]) {
    if (d >= TOF_WALL_OFF_MM) tofState.wall[i] = false;
  } else {
    if (d <= TOF_WALL_ON_MM) tofState.wall[i] = true;
  }
}

void serviceToFs(bool forceAll = false) {
  uint32_t now = millis();
  if (forceAll) {
    for (uint8_t i = 0; i < TOF_COUNT; ++i) readToF(i);
    tofServiceIndex = 0;
    lastTofService = now;
    return;
  }
  if (now - lastTofService < TOF_PERIOD_MS) return;
  lastTofService = now;
  readToF(tofServiceIndex);
  tofServiceIndex = (tofServiceIndex + 1) % TOF_COUNT;
}

bool tofFresh(uint8_t i) {
  return tofState.valid[i] && (millis() - tofState.stamp[i] <= TOF_STALE_MS);
}

void loadCalibration() {
  prefs.begin("qtr", true);
  calibrationValid = prefs.getBool("valid", false);
  for (uint8_t i = 0; i < QTR_COUNT; ++i) {
    char a[8];
    char b[8];
    snprintf(a, sizeof(a), "n%u", i);
    snprintf(b, sizeof(b), "x%u", i);
    qtrMin[i] = prefs.getUShort(a, 0);
    qtrMax[i] = prefs.getUShort(b, QTR_TIMEOUT_US);
    if (qtrMax[i] <= qtrMin[i] + 40) calibrationValid = false;
  }
  prefs.end();
}

void saveCalibration() {
  prefs.begin("qtr", false);
  prefs.putBool("valid", calibrationValid);
  for (uint8_t i = 0; i < QTR_COUNT; ++i) {
    char a[8];
    char b[8];
    snprintf(a, sizeof(a), "n%u", i);
    snprintf(b, sizeof(b), "x%u", i);
    prefs.putUShort(a, qtrMin[i]);
    prefs.putUShort(b, qtrMax[i]);
  }
  prefs.end();
}

void calibrateQTR() {
  stopDrive();
  uint16_t lo[QTR_COUNT];
  uint16_t hi[QTR_COUNT];
  for (uint8_t i = 0; i < QTR_COUNT; ++i) {
    lo[i] = QTR_TIMEOUT_US;
    hi[i] = 0;
  }
  uint32_t start = millis();
  while (millis() - start < QTR_CALIBRATION_MS) {
    qtr.read(qtrRaw);
    for (uint8_t i = 0; i < QTR_COUNT; ++i) {
      lo[i] = min(lo[i], qtrRaw[i]);
      hi[i] = max(hi[i], qtrRaw[i]);
    }
    delay(5);
  }
  calibrationValid = true;
  for (uint8_t i = 0; i < QTR_COUNT; ++i) {
    qtrMin[i] = lo[i];
    qtrMax[i] = hi[i];
    if (hi[i] <= lo[i] + 40) calibrationValid = false;
  }
  if (calibrationValid) saveCalibration();
}

uint16_t normalizedDark(uint8_t i, uint16_t raw) {
  if (!calibrationValid) return 0;
  uint16_t lo = qtrMin[i];
  uint16_t hi = qtrMax[i];
  if (hi <= lo + 1) return 0;
  raw = clampi(raw, lo, hi);
  return (uint16_t)(((uint32_t)(raw - lo) * 1000u) / (uint32_t)(hi - lo));
}

uint16_t targetLineStrength(uint8_t i, uint16_t raw) {
  uint16_t dark = normalizedDark(i, raw);
  return LINE_IS_WHITE ? (uint16_t)(1000u - dark) : dark;
}

void resetLineControl() {
  memset(&lineState, 0, sizeof(lineState));
  memset(errorBuf, 0, sizeof(errorBuf));
  memset(errorTime, 0, sizeof(errorTime));
  errorHead = 0;
  errorCount = 0;
  lineDarkStart = 0;
  entryTofHits = 0;
  lineLostStart = 0;
  lineTurnState = 0;
  lineTurnStart = 0;
  lineTurnSawGap = false;
  junctionCounter = 0;
  junctionStart = 0;
  junctionStage = 0;
}

void readLine() {
  static const float rightWeights[8] = {
    1.0f, 0.72f, 0.44f, 0.14f,
   -0.14f,-0.44f,-0.72f,-1.0f
  };
  static const float leftWeights[8] = {
   -1.0f,-0.72f,-0.44f,-0.14f,
    0.14f, 0.44f, 0.72f, 1.0f
  };
  const float *weights = QTR_S0_IS_RIGHT ? rightWeights : leftWeights;

  qtr.read(qtrRaw);
  uint32_t total = 0;
  float weighted = 0.0f;
  uint8_t mask = 0;

  for (uint8_t i = 0; i < QTR_COUNT; ++i) {
    lineState.raw[i] = qtrRaw[i];
    lineState.strength[i] = targetLineStrength(i, qtrRaw[i]);
    total += lineState.strength[i];
    if (lineState.strength[i] >= LINE_PRESENT) mask |= (uint8_t)(1u << i);
    weighted += weights[i] * (float)lineState.strength[i];
  }

  lineState.total = total;
  lineState.mask = mask;

  uint32_t centerTotal =
    lineState.strength[2] +
    lineState.strength[3] +
    lineState.strength[4] +
    lineState.strength[5];

  lineState.detected = centerTotal >= LINE_PRESENT;

  if (lineState.detected && total >= LINE_PRESENT) {
    float denom = max(1.0f, (float)total);
    lineState.error = clampf(weighted / denom, -1.0f, 1.0f);
  }
}

void updateLineDerivative() {
  uint32_t now = micros();
  errorBuf[errorHead] = lineState.error;
  errorTime[errorHead] = now;
  errorHead = (errorHead + 1) % ERROR_BUFFER_SIZE;
  if (errorCount < ERROR_BUFFER_SIZE) ++errorCount;
  if (errorCount < 2) {
    lineState.derivative = 0;
    return;
  }
  uint8_t oldest = (errorHead + ERROR_BUFFER_SIZE - errorCount) % ERROR_BUFFER_SIZE;
  uint32_t dt = now - errorTime[oldest];
  if (dt < 1000) dt = 1000;
  float d = (lineState.error - errorBuf[oldest]) / ((float)dt * 0.000001f);
  lineState.derivative = clampf(d, -35.0f, 35.0f);
}

uint16_t adaptiveLineSpeed(float severity) {
  static const uint16_t day1[11] = {215,214,211,207,201,194,185,174,160,143,122};
  static const uint16_t day2[11] = {225,224,220,216,209,201,191,179,164,145,122};
  const uint16_t *table = DAY_PROFILE == 2 ? day2 : day1;
  severity = clampf(severity, 0.0f, 1.0f);
  float p = severity * 10.0f;
  uint8_t i = (uint8_t)p;
  if (i >= 10) return table[10];
  float t = p - i;
  return (uint16_t)roundf(table[i] + t * (table[i + 1] - table[i]));
}

bool broadDarkFloor() {
  if (!calibrationValid) return false;
  uint8_t count = 0;
  for (uint8_t i = 0; i < QTR_COUNT; ++i) {
    if (normalizedDark(i, lineState.raw[i]) >= LINE_DARK_FLOOR_STRONG) ++count;
  }
  return count >= LINE_DARK_MIN_COUNT;
}

bool allLineStrong() {
  uint8_t count = 0;
  for (uint8_t i = 0; i < QTR_COUNT; ++i) {
    if (lineState.strength[i] >= LINE_STRONG) ++count;
  }
  return count >= 7;
}

bool leftExtremeStrong() {
  return lineState.strength[6] >= LINE_STRONG || lineState.strength[7] >= LINE_STRONG;
}

bool rightExtremeStrong() {
  return lineState.strength[0] >= LINE_STRONG || lineState.strength[1] >= LINE_STRONG;
}

bool centerStrong() {
  return lineState.strength[3] >= LINE_CENTER_STRONG || lineState.strength[4] >= LINE_CENTER_STRONG;
}

bool mazeEntryToFGeometry() {
  bool freshLeft = tofFresh(0);
  bool freshFront = tofFresh(1);
  bool freshRight = tofFresh(2);

  if (freshLeft && freshRight) {
    uint16_t l = tofState.mm[0];
    uint16_t r = tofState.mm[2];
    if (l >= ENTRY_SIDE_MIN_MM && l <= ENTRY_SIDE_MAX_MM &&
        r >= ENTRY_SIDE_MIN_MM && r <= ENTRY_SIDE_MAX_MM &&
        (uint32_t)l + r >= ENTRY_SIDE_SUM_MIN_MM &&
        (uint32_t)l + r <= ENTRY_SIDE_SUM_MAX_MM) {
      return true;
    }
  }

  if (freshFront && (tofFresh(0) || tofFresh(2))) {
    bool frontNear = tofState.mm[1] <= ENTRY_FRONT_MAX_MM;
    bool sideNear = (tofFresh(0) && tofState.mm[0] <= ENTRY_SIDE_MAX_MM) ||
                     (tofFresh(2) && tofState.mm[2] <= ENTRY_SIDE_MAX_MM);
    if (frontNear && sideNear) return true;
  }

  return false;
}

void lineTurnBegin(bool left) {
  lineTurnState = 1;
  lineTurnStart = millis();
  lineTurnSawGap = false;
  lineTurnLeft = left;
}

void lineTurnStep() {
  readLine();
  uint32_t now = millis();
  bool centerSeen = centerStrong();

  if (!lineTurnSawGap) {
    if (!centerSeen && lineState.total < LINE_PRESENT * 2UL) {
      lineTurnSawGap = true;
    }
    int left = lineTurnLeft ? -LINE_SHARP_INNER_PWM : LINE_SHARP_OUTER_PWM;
    int right = lineTurnLeft ? LINE_SHARP_OUTER_PWM : -LINE_SHARP_INNER_PWM;
    setDrive(left, right);
  } else {
    bool recovered = centerSeen && fabsf(lineState.error) <= 0.32f;
    if (recovered) {
      lineTurnState = 0;
      resetLineControl();
      return;
    }
    int left = lineTurnLeft ? -LINE_SHARP_INNER_PWM : LINE_SHARP_OUTER_PWM;
    int right = lineTurnLeft ? LINE_SHARP_OUTER_PWM : -LINE_SHARP_INNER_PWM;
    setDrive(left, right);
  }

  if (now - lineTurnStart > 950) {
    hardFault();
  }
}

void lineControlStep() {
  updateLineDerivative();
  float severity = max(fabsf(lineState.error), clampf(fabsf(lineState.derivative) * 0.035f, 0.0f, 1.0f));
  float correction = LINE_KP * lineState.error + LINE_KI * 0.0f + LINE_KD * lineState.derivative;
  correction = clampf(correction, -LINE_MAX_CORRECTION, LINE_MAX_CORRECTION);

  uint16_t base = adaptiveLineSpeed(severity);

  if (fabsf(lineState.error) > 0.70f) {
    base = min<uint16_t>(base, LINE_SHARP_OUTER_PWM);
  }

  int left = clampi((int)roundf((float)base + correction), -255, 255);
  int right = clampi((int)roundf((float)base - correction), -255, 255);
  setDrive(left, right);
  lineState.previousError = lineState.error;
}

void lineLostStep() {
  uint32_t now = millis();
  if (!lineLostStart) lineLostStart = now;
  uint32_t lostMs = now - lineLostStart;

  if (lostMs <= LINE_LOST_HOLD_MS) {
    int correction = clampi((int)roundf(LINE_KP * lineState.previousError), -35, 35);
    setDrive(TRANSITION_FORWARD_PWM + correction, TRANSITION_FORWARD_PWM - correction);
    return;
  }

  if (lostMs <= LINE_LOST_SEARCH_MS) {
    bool turnRight = lineState.previousError > 0.0f;
    if (turnRight) setDrive(LINE_SEARCH_PWM, -LINE_SEARCH_PWM);
    else setDrive(-LINE_SEARCH_PWM, LINE_SEARCH_PWM);
    return;
  }

  if (lostMs <= LINE_LOST_FAULT_MS) {
    bool turnRight = lineState.previousError > 0.0f;
    if (turnRight) setDrive(50, -95);
    else setDrive(-95, 50);
    return;
  }

  hardFault();
}

void junctionStep() {
  readLine();

  bool left = leftExtremeStrong();
  bool right = rightExtremeStrong();
  bool center = centerStrong();

  if (junctionStage == 0) {
    junctionStart = millis();
    junctionStage = 1;
    setDrive(92, 92);
    return;
  }

  if (millis() - junctionStart < 65) {
    setDrive(92, 92);
    return;
  }

  if (DAY_PROFILE != 2) {
    junctionStage = 0;
    lineControlStep();
    return;
  }

  bool leftChoice = left;
  bool rightChoice = right;
  bool straightChoice = center;
  uint8_t choice = 255;

  if (DAY2_JUNCTION_POLICY == JUNCTION_LEFT_FIRST) {
    if (leftChoice) choice = 1;
    else if (straightChoice) choice = 0;
    else if (rightChoice) choice = 2;
  } else if (DAY2_JUNCTION_POLICY == JUNCTION_RIGHT_FIRST) {
    if (rightChoice) choice = 2;
    else if (straightChoice) choice = 0;
    else if (leftChoice) choice = 1;
  } else {
    if (straightChoice) choice = 0;
    else if (leftChoice) choice = 1;
    else if (rightChoice) choice = 2;
  }

  junctionStage = 0;

  if (choice == 1) {
    lineTurnBegin(true);
    lineTurnStep();
  } else if (choice == 2) {
    lineTurnBegin(false);
    lineTurnStep();
  } else {
    lineControlStep();
  }
}

void lineStep() {
  readLine();
  serviceToFs(false);

  bool darkFloor = broadDarkFloor();

  if (darkFloor) {
    if (!lineDarkStart) lineDarkStart = millis();
    serviceToFs(true);
    if (mazeEntryToFGeometry()) {
      if (entryTofHits < 255) ++entryTofHits;
    } else {
      entryTofHits = 0;
    }

    if (millis() - lineDarkStart >= LINE_ENTRY_DARK_MS && entryTofHits >= LINE_ENTRY_TOF_CONFIRMATIONS) {
      transitionStart = millis();
      transitionLeft = lastLeftCommand > 0 ? lastLeftCommand : TRANSITION_FORWARD_PWM;
      transitionRight = lastRightCommand > 0 ? lastRightCommand : TRANSITION_FORWARD_PWM;
      transitionLeft = clampi(transitionLeft, 70, TRANSITION_FORWARD_PWM + 35);
      transitionRight = clampi(transitionRight, 70, TRANSITION_FORWARD_PWM + 35);
      state = LINE_TO_MAZE;
      return;
    }

    if (millis() - lineDarkStart > LINE_ENTRY_TIMEOUT_MS) {
      hardFault();
      return;
    }

    setDrive(TRANSITION_FORWARD_PWM, TRANSITION_FORWARD_PWM);
    return;
  }

  lineDarkStart = 0;
  entryTofHits = 0;

  if (lineTurnState) {
    lineTurnStep();
    return;
  }

  bool leftExtreme = leftExtremeStrong();
  bool rightExtreme = rightExtremeStrong();
  bool center = centerStrong();
  bool allStrong = allLineStrong();

  if (!lineState.detected) {
    lineLostStep();
    return;
  }

  lineLostStart = 0;

  if (fabsf(lineState.error) > 0.58f && (leftExtreme || rightExtreme) && !allStrong) {
    lineTurnBegin(lineState.error < 0.0f);
    lineTurnStep();
    return;
  }

  if (allStrong) {
    ++junctionCounter;
    junctionStep();
    return;
  }

  if (DAY_PROFILE == 2 && (leftExtreme || rightExtreme) && center) {
    if (DAY2_JUNCTION_POLICY == JUNCTION_LEFT_FIRST && leftExtreme) {
      lineTurnBegin(true);
      lineTurnStep();
      return;
    }
    if (DAY2_JUNCTION_POLICY == JUNCTION_RIGHT_FIRST && rightExtreme) {
      lineTurnBegin(false);
      lineTurnStep();
      return;
    }
  }

  lineControlStep();
}

void configureDay() {
  if (DAY_PROFILE == 2) {
    mazeW = DAY2_W;
    mazeH = DAY2_H;
    goal = {DAY2_GOAL_X0, DAY2_GOAL_X1, DAY2_GOAL_Y0, DAY2_GOAL_Y1};
  } else {
    mazeW = DAY1_W;
    mazeH = DAY1_H;
    goal = {DAY1_GOAL_X0, DAY1_GOAL_X1, DAY1_GOAL_Y0, DAY1_GOAL_Y1};
  }
}

void resetMaze() {
  memset(maze, 0, sizeof(maze));
  memset(parents, 0, sizeof(parents));
  memset(history, 0, sizeof(history));
  memset(flood, 0xFF, sizeof(flood));
  historySize = 0;
  mazeX = START_X;
  mazeY = START_Y;
  mazeHeading = START_HEADING;
  targetHeading = START_HEADING;

  for (uint8_t x = 0; x < mazeW; ++x) {
    maze[0][x].known |= WALL_BIT[DIR_S];
    maze[0][x].blocked |= WALL_BIT[DIR_S];
    maze[mazeH - 1][x].known |= WALL_BIT[DIR_N];
    maze[mazeH - 1][x].blocked |= WALL_BIT[DIR_N];
  }

  for (uint8_t y = 0; y < mazeH; ++y) {
    maze[y][0].known |= WALL_BIT[DIR_W];
    maze[y][0].blocked |= WALL_BIT[DIR_W];
    maze[y][mazeW - 1].known |= WALL_BIT[DIR_E];
    maze[y][mazeW - 1].blocked |= WALL_BIT[DIR_E];
  }

  maze[mazeY][mazeX].flags |= 1;
  history[historySize++] = (uint16_t)(mazeY * mazeW + mazeX);
}

void observeWall(int x, int y, uint8_t dir, bool wallPresent) {
  if (!inMaze(x, y)) return;
  int nx = x + DX[dir];
  int ny = y + DY[dir];

  maze[y][x].known |= WALL_BIT[dir];
  if (wallPresent) maze[y][x].blocked |= WALL_BIT[dir];
  else maze[y][x].blocked &= (uint8_t)~WALL_BIT[dir];

  if (!inMaze(nx, ny)) return;

  uint8_t opp = WALL_OPP[dir];
  maze[ny][nx].known |= opp;
  if (wallPresent) maze[ny][nx].blocked |= opp;
  else maze[ny][nx].blocked &= (uint8_t)~opp;
}

bool cellBlocked(int x, int y, uint8_t dir) {
  if (!inMaze(x, y)) return true;
  return (maze[y][x].blocked & WALL_BIT[dir]) != 0;
}

bool cellKnown(int x, int y, uint8_t dir) {
  if (!inMaze(x, y)) return true;
  return (maze[y][x].known & WALL_BIT[dir]) != 0;
}

void updateMazeMap() {
  uint8_t left = (mazeHeading + 3) & 3;
  uint8_t front = mazeHeading;
  uint8_t right = (mazeHeading + 1) & 3;
  if (tofFresh(0)) observeWall(mazeX, mazeY, left, tofState.wall[0]);
  if (tofFresh(1)) observeWall(mazeX, mazeY, front, tofState.wall[1]);
  if (tofFresh(2)) observeWall(mazeX, mazeY, right, tofState.wall[2]);
}

void recomputeFlood() {
  for (uint8_t y = 0; y < mazeH; ++y) {
    for (uint8_t x = 0; x < mazeW; ++x) {
      flood[y][x] = FLOOD_INF;
    }
  }

  uint16_t head = 0;
  uint16_t tail = 0;

  for (uint8_t y = goal.y0; y <= goal.y1; ++y) {
    for (uint8_t x = goal.x0; x <= goal.x1; ++x) {
      if (!inMaze(x, y)) continue;
      flood[y][x] = 0;
      floodQueue[tail++] = (uint16_t)(y * mazeW + x);
    }
  }

  while (head < tail) {
    uint16_t id = floodQueue[head++];
    uint8_t x = id % mazeW;
    uint8_t y = id / mazeW;
    uint8_t next = (uint8_t)(flood[y][x] + 1);

    for (uint8_t d = 0; d < 4; ++d) {
      if (cellBlocked(x, y, d)) continue;
      int nx = x + DX[d];
      int ny = y + DY[d];
      if (!inMaze(nx, ny)) continue;
      if (flood[ny][nx] <= next) continue;
      flood[ny][nx] = next;
      floodQueue[tail++] = (uint16_t)(ny * mazeW + nx);
    }
  }
}

uint8_t turnDistance(uint8_t a, uint8_t b) {
  uint8_t d = (b + 4 - a) & 3;
  return d > 2 ? 4 - d : d;
}

bool floodNext(uint8_t &best) {
  recomputeFlood();

  uint16_t bestFlood = 65535;
  bool bestUnvisited = false;
  uint8_t bestTurn = 255;
  bool found = false;

  uint8_t parentDir = 255;
  if (parents[mazeY][mazeX].valid) {
    int px = parents[mazeY][mazeX].x;
    int py = parents[mazeY][mazeX].y;
    for (uint8_t d = 0; d < 4; ++d) {
      if (mazeX + DX[d] == px && mazeY + DY[d] == py) {
        parentDir = d;
        break;
      }
    }
  }

  for (uint8_t d = 0; d < 4; ++d) {
    if (cellBlocked(mazeX, mazeY, d)) continue;
    int nx = mazeX + DX[d];
    int ny = mazeY + DY[d];
    if (!inMaze(nx, ny)) continue;

    uint16_t fv = flood[ny][nx];
    bool unvisited = (maze[ny][nx].flags & 1) == 0;
    uint8_t turns = turnDistance(mazeHeading, d);

    if (!found ||
        fv < bestFlood ||
        (fv == bestFlood && unvisited && !bestUnvisited) ||
        (fv == bestFlood && unvisited == bestUnvisited && turns < bestTurn)) {
      found = true;
      best = d;
      bestFlood = fv;
      bestUnvisited = unvisited;
      bestTurn = turns;
    }
  }

  if (found && bestFlood < FLOOD_INF) return true;

  if (parentDir < 4 && !cellBlocked(mazeX, mazeY, parentDir)) {
    best = parentDir;
    return true;
  }

  for (uint8_t d = 0; d < 4; ++d) {
    if (!cellBlocked(mazeX, mazeY, d)) {
      int nx = mazeX + DX[d];
      int ny = mazeY + DY[d];
      if (inMaze(nx, ny)) {
        best = d;
        return true;
      }
    }
  }

  return false;
}

uint16_t heuristic(uint8_t x, uint8_t y) {
  int dx = x < goal.x0 ? goal.x0 - x : (x > goal.x1 ? x - goal.x1 : 0);
  int dy = y < goal.y0 ? goal.y0 - y : (y > goal.y1 ? y - goal.y1 : 0);
  if (USE_CHEBYSHEV) return (uint16_t)(max(dx, dy) * 100);
  return (uint16_t)((dx + dy) * 100);
}

int stateId(uint8_t x, uint8_t y, uint8_t h) {
  return ((y * mazeW + x) * 4 + h);
}

void decodeState(int id, uint8_t &x, uint8_t &y, uint8_t &h) {
  h = id % 4;
  uint16_t cell = id / 4;
  x = cell % mazeW;
  y = cell / mazeW;
}

bool knownOpenEdge(int x, int y, uint8_t dir) {
  if (!inMaze(x, y) || !cellKnown(x, y, dir) || cellBlocked(x, y, dir)) return false;
  int nx = x + DX[dir];
  int ny = y + DY[dir];
  if (!inMaze(nx, ny)) return false;
  uint8_t od = (dir + 2) & 3;
  return cellKnown(nx, ny, od) && !cellBlocked(nx, ny, od);
}

bool heapLess(uint16_t a, uint16_t b) {
  if (astar[a].f != astar[b].f) return astar[a].f < astar[b].f;
  return astar[a].g > astar[b].g;
}

void heapSwap(uint16_t a, uint16_t b) {
  uint16_t t = astarHeap[a];
  astarHeap[a] = astarHeap[b];
  astarHeap[b] = t;
  astar[astarHeap[a]].heapPos = a;
  astar[astarHeap[b]].heapPos = b;
}

void heapUp(uint16_t p) {
  while (p) {
    uint16_t q = (p - 1) / 2;
    if (!heapLess(astarHeap[p], astarHeap[q])) break;
    heapSwap(p, q);
    p = q;
  }
}

void heapDown(uint16_t p) {
  while (true) {
    uint16_t l = p * 2 + 1;
    if (l >= astarHeapSize) break;
    uint16_t r = l + 1;
    uint16_t b = l;
    if (r < astarHeapSize && heapLess(astarHeap[r], astarHeap[l])) b = r;
    if (!heapLess(astarHeap[b], astarHeap[p])) break;
    heapSwap(p, b);
    p = b;
  }
}

void heapPush(uint16_t s) {
  if (astar[s].heapPos < 0) {
    if (astarHeapSize >= MAX_ASTAR_STATES) return;
    uint16_t p = astarHeapSize++;
    astarHeap[p] = s;
    astar[s].heapPos = p;
  }
  heapUp(astar[s].heapPos);
}

int heapPop() {
  if (!astarHeapSize) return -1;
  uint16_t root = astarHeap[0];
  --astarHeapSize;
  if (astarHeapSize) {
    astarHeap[0] = astarHeap[astarHeapSize];
    astar[astarHeap[0]].heapPos = 0;
    heapDown(0);
  }
  astar[root].heapPos = -1;
  return root;
}

uint8_t turnsBetween(uint8_t a, uint8_t b) {
  uint8_t d = (b + 4 - a) & 3;
  return d > 2 ? 4 - d : d;
}

uint16_t aEdgeCost(uint8_t fromH, uint8_t toH) {
  return (uint16_t)(100 + turnsBetween(fromH, toH) * TURN_COST_90);
}

bool buildKnownAStar(uint8_t &firstDir) {
  for (uint16_t i = 0; i < MAX_ASTAR_STATES; ++i) {
    astar[i].g = 65535;
    astar[i].f = 65535;
    astar[i].parent = -1;
    astar[i].heapPos = -1;
    astar[i].closed = false;
  }

  astarHeapSize = 0;
  astarSolutionSize = 0;
  planSize = 0;

  int start = stateId((uint8_t)mazeX, (uint8_t)mazeY, mazeHeading);
  astar[start].g = 0;
  astar[start].f = heuristic((uint8_t)mazeX, (uint8_t)mazeY);
  heapPush((uint16_t)start);

  int goalState = -1;

  while (astarHeapSize) {
    int current = heapPop();
    if (current < 0) break;
    if (astar[current].closed) continue;
    astar[current].closed = true;

    uint8_t x, y, h;
    decodeState(current, x, y, h);

    if (inGoal(x, y)) {
      goalState = current;
      break;
    }

    for (uint8_t d = 0; d < 4; ++d) {
      if (!knownOpenEdge(x, y, d)) continue;

      uint8_t nx = (uint8_t)(x + DX[d]);
      uint8_t ny = (uint8_t)(y + DY[d]);
      uint16_t next = (uint16_t)stateId(nx, ny, d);

      if (astar[next].closed) continue;

      uint32_t ng = (uint32_t)astar[current].g + aEdgeCost(h, d);
      if (ng >= 65535) continue;

      if (ng < astar[next].g) {
        astar[next].g = (uint16_t)ng;
        uint32_t nf = ng + heuristic(nx, ny);
        astar[next].f = nf > 65535 ? 65535 : (uint16_t)nf;
        astar[next].parent = current;
        heapPush(next);
      }
    }
  }

  if (goalState < 0) return false;

  int s = goalState;
  while (s >= 0 && astarSolutionSize < MAX_ASTAR_STATES) {
    astarSolution[astarSolutionSize++] = (int16_t)s;
    s = astar[s].parent;
  }

  if (astarSolutionSize < 2) {
    firstDir = mazeHeading;
    return true;
  }

  for (uint16_t i = 0; i < astarSolutionSize / 2; ++i) {
    int16_t t = astarSolution[i];
    astarSolution[i] = astarSolution[astarSolutionSize - 1 - i];
    astarSolution[astarSolutionSize - 1 - i] = t;
  }

  uint8_t startX, startY, startH;
  decodeState(astarSolution[0], startX, startY, startH);

  uint8_t nextX, nextY, nextH;
  decodeState(astarSolution[1], nextX, nextY, nextH);
  firstDir = nextH;

  uint8_t runDir = startH;
  uint8_t runCount = 0;

  for (uint16_t i = 1; i < astarSolutionSize; ++i) {
    uint8_t x, y, h;
    decodeState(astarSolution[i], x, y, h);

    if (h != runDir) {
      if (runCount && planSize < MAX_PLAN) {
        plan[planSize++] = {0, runCount, 0};
      }
      int delta = (int)h - (int)runDir;
      if (delta > 2) delta -= 4;
      if (delta < -2) delta += 4;
      if (planSize < MAX_PLAN) {
        plan[planSize++] = {1, 1, (int8_t)delta};
      }
      runDir = h;
      runCount = 0;
    }
    ++runCount;
  }

  if (runCount && planSize < MAX_PLAN) {
    plan[planSize++] = {0, runCount, 0};
  }

  return true;
}

void resetYawReference() {
  yawDeg = 0.0f;
  yawTimeUs = micros();
  headingPrevError = 0.0f;
  headingPrevTime = millis();
}

void updateYaw() {
  if (!mpuReady) return;
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  uint32_t now = micros();
  uint32_t dtUs = now - yawTimeUs;
  yawTimeUs = now;
  if (dtUs > 100000) return;
  yawDeg += (gyro.gyro.z - gyroBiasZ) * 57.2957795f * ((float)dtUs * 0.000001f);
}

float headingCorrection() {
  uint32_t now = millis();
  uint32_t dt = now - headingPrevTime;
  if (dt < 1) dt = 1;
  float error = -yawDeg;
  float derivative = (error - headingPrevError) / ((float)dt * 0.001f);
  headingPrevError = error;
  headingPrevTime = now;
  return clampf(HEADING_KP * error + HEADING_KD * derivative, -HEADING_MAX_CORRECTION, HEADING_MAX_CORRECTION);
}

float wallCorrection() {
  if (!tofFresh(0) || !tofFresh(2)) return 0.0f;
  if (!tofState.wall[0] || !tofState.wall[2]) return 0.0f;

  uint32_t now = millis();
  uint32_t dt = now - wallPrevTime;
  if (dt < 1) dt = 1;

  float error = (float)tofState.mm[0] - (float)tofState.mm[2];
  float derivative = (error - wallPrevError) / ((float)dt * 0.001f);
  wallPrevError = error;
  wallPrevTime = now;

  return clampf(WALL_KP * error + WALL_KD * derivative, -WALL_MAX_CORRECTION, WALL_MAX_CORRECTION);
}

void startMazeTurn(uint8_t desiredHeading) {
  uint8_t delta = (desiredHeading + 4 - mazeHeading) & 3;
  if (!delta) {
    resetYawReference();
    moveStartMs = millis();
    state = MAZE_MOVE;
    return;
  }

  targetHeading = desiredHeading;
  turnTargetDeg = delta == 2 ? 180.0f : 90.0f;
  turnSign = (delta == 1 || delta == 2) ? 1.0f : -1.0f;
  turnStartMs = millis();
  resetYawReference();
  state = MAZE_TURN;

  int p = MAZE_TURN_PWM;
  setDrive(turnSign > 0 ? p : -p, turnSign > 0 ? -p : p);
}

void mazeTurnStep() {
  updateYaw();
  float angle = fabsf(yawDeg);
  if (angle >= turnTargetDeg) {
    stopDrive();
    mazeHeading = targetHeading;
    resetYawReference();
    moveStartMs = millis();
    state = MAZE_MOVE;
    return;
  }

  if (millis() - turnStartMs > MAZE_TURN_TIMEOUT_MS) {
    hardFault();
    return;
  }

  float ratio = angle / turnTargetDeg;
  int p = ratio > 0.72f ? MAZE_TURN_PWM_SLOW : MAZE_TURN_PWM;
  setDrive(turnSign > 0 ? p : -p, turnSign > 0 ? -p : p);
}

void finishMazeCell() {
  int nx = mazeX + DX[mazeHeading];
  int ny = mazeY + DY[mazeHeading];

  if (!inMaze(nx, ny)) {
    hardFault();
    return;
  }

  if (!(maze[ny][nx].flags & 1)) {
    parents[ny][nx] = {(int8_t)mazeX, (int8_t)mazeY, true};
  }

  mazeX = nx;
  mazeY = ny;
  maze[mazeY][mazeX].flags |= 1;

  if (historySize < MAX_HISTORY) {
    history[historySize++] = (uint16_t)(mazeY * mazeW + mazeX);
  }

  if (inGoal(mazeX, mazeY)) {
    stopDrive();
    state = FINISHED;
    return;
  }

  serviceToFs(true);
  updateMazeMap();

  uint8_t nextDir = mazeHeading;
  uint8_t astarDir = mazeHeading;

  if (buildKnownAStar(astarDir)) {
    nextDir = astarDir;
  } else if (!floodNext(nextDir)) {
    hardFault();
    return;
  }

  if (nextDir == mazeHeading && !cellBlocked(mazeX, mazeY, mazeHeading)) {
    resetYawReference();
    moveStartMs = millis();
    return;
  }

  stopDrive();
  startMazeTurn(nextDir);
}

void mazeMoveStep() {
  serviceToFs(false);
  updateYaw();

  uint32_t elapsed = millis() - moveStartMs;
  float corr = wallCorrection();

  bool bothSideWalls = tofFresh(0) && tofFresh(2) && tofState.wall[0] && tofState.wall[2];
  if (!bothSideWalls) {
    corr = headingCorrection();
  }

  int base = MAZE_PWM;

  if (tofFresh(1) && tofState.wall[1] && tofState.mm[1] < MAZE_FRONT_BRAKE_MM && elapsed > MAZE_CELL_MIN_MS) {
    float factor = clampf(
      (float)(tofState.mm[1] - MAZE_FRONT_STOP_MM) /
      (float)(MAZE_FRONT_BRAKE_MM - MAZE_FRONT_STOP_MM),
      0.12f,
      1.0f
    );
    base = (int)roundf((float)MAZE_PWM * factor);
  }

  setDrive(
    clampi((int)roundf(base - corr), 0, 255),
    clampi((int)roundf(base + corr), 0, 255)
  );

  bool frontStop = tofFresh(1) && tofState.wall[1] && tofState.mm[1] <= MAZE_FRONT_STOP_MM && elapsed >= MAZE_CELL_MIN_MS;
  bool timeStop = elapsed >= MAZE_CELL_MS;

  if (frontStop || timeStop) {
    finishMazeCell();
  }
}

void transitionStep() {
  serviceToFs(false);

  uint32_t elapsed = millis() - transitionStart;
  float a = clampf((float)elapsed / (float)LINE_TRANSITION_BLEND_MS, 0.0f, 1.0f);
  float corr = wallCorrection();

  int mazeLeft = TRANSITION_FORWARD_PWM - (int)roundf(corr);
  int mazeRight = TRANSITION_FORWARD_PWM + (int)roundf(corr);

  int left = (int)roundf((1.0f - a) * transitionLeft + a * mazeLeft);
  int right = (int)roundf((1.0f - a) * transitionRight + a * mazeRight);

  setDrive(clampi(left, 0, 255), clampi(right, 0, 255));

  if (a >= 1.0f) {
    resetMaze();
    updateMazeMap();
    state = MAZE_SELECT;
  }
}

void mazeSelectStep() {
  if (inGoal(mazeX, mazeY)) {
    stopDrive();
    state = FINISHED;
    return;
  }

  serviceToFs(true);
  updateMazeMap();

  uint8_t next = mazeHeading;
  uint8_t knownNext = mazeHeading;
  bool haveKnownAStar = buildKnownAStar(knownNext);

  if (haveKnownAStar) {
    next = knownNext;
  } else if (!floodNext(next)) {
    hardFault();
    return;
  }

  startMazeTurn(next);
}

void printStatus() {
  if (!DEBUG_SERIAL) return;
  Serial.printf(
    "DAY=%u STATE=%u XY=%d,%d H=%u QTR=0x%02X TOF=%u,%u,%u\n",
    DAY_PROFILE,
    (unsigned)state,
    mazeX,
    mazeY,
    mazeHeading,
    lineState.mask,
    tofState.mm[0],
    tofState.mm[1],
    tofState.mm[2]
  );
}

void printFlood() {
  if (!DEBUG_SERIAL) return;
  recomputeFlood();
  for (int y = mazeH - 1; y >= 0; --y) {
    for (uint8_t x = 0; x < mazeW; ++x) {
      if (inGoal(x, y)) Serial.print(" G ");
      else if (flood[y][x] == FLOOD_INF) Serial.print(" ? ");
      else {
        if (flood[y][x] < 10) Serial.print(' ');
        Serial.print(flood[y][x]);
        Serial.print(' ');
      }
    }
    Serial.println();
  }
}

void onConnectedController(ControllerPtr ctl) {
  for (uint8_t i = 0; i < BP32_MAX_GAMEPADS; ++i) {
    if (controllerSlots[i] == nullptr) {
      controllerSlots[i] = ctl;
      if (primaryController == nullptr) primaryController = ctl;
      return;
    }
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  for (uint8_t i = 0; i < BP32_MAX_GAMEPADS; ++i) {
    if (controllerSlots[i] == ctl) controllerSlots[i] = nullptr;
  }
  if (primaryController == ctl) {
    primaryController = nullptr;
    controllerComboAction = 0;
    controllerComboStart = 0;
    controllerComboConsumed = false;
    for (uint8_t i = 0; i < BP32_MAX_GAMEPADS; ++i) {
      if (controllerSlots[i] != nullptr && controllerSlots[i]->isConnected()) {
        primaryController = controllerSlots[i];
        break;
      }
    }
  }
}

uint8_t controllerAction(ControllerPtr ctl) {
  if (!ctl || !ctl->isConnected() || !ctl->isGamepad()) return 0;
  bool guard = ctl->l1() && ctl->r1();
  if (!guard) return 0;
  if (ctl->a()) return 1;
  if (ctl->y()) return 2;
  if (ctl->x()) return 3;
  if ((ctl->dpad() & DPAD_UP) != 0) return 4;
  return 0;
}

void executeControllerAction(uint8_t action) {
  if (action == 1) {
    stateBeforeOperatorStop = state;
    stopDrive();
    pcaDigital(PCA_STBY_CH, false);
    state = OPERATOR_STOP;
    return;
  }
  if (action == 2) {
    resetMission();
    return;
  }
  if (action == 3) {
    forceMazeStart();
    return;
  }
  if (action == 4 && (state == FAULT || state == OPERATOR_STOP)) {
    state = FAULT;
    stopDrive();
    calibrateQTR();
    resetLineControl();
    if (calibrationValid && mpuReady && tofReady) {
      pcaDigital(PCA_STBY_CH, true);
      startupCalibrationNeeded = false;
      resetMission();
    }
  }
}

void processController() {
  BP32.update();
  ControllerPtr ctl = primaryController;
  if (!ctl || !ctl->isConnected() || !ctl->isGamepad()) return;

  uint8_t action = controllerAction(ctl);
  uint16_t holdMs = 0;
  if (action == 1) holdMs = CONTROLLER_STOP_HOLD_MS;
  else if (action == 2) holdMs = CONTROLLER_RESET_HOLD_MS;
  else if (action == 3) holdMs = CONTROLLER_FORCE_MAZE_HOLD_MS;
  else if (action == 4) holdMs = CONTROLLER_CALIBRATE_HOLD_MS;

  if (action != controllerComboAction) {
    controllerComboAction = action;
    controllerComboStart = action ? millis() : 0;
    controllerComboConsumed = false;
  }

  if (action && !controllerComboConsumed && millis() - controllerComboStart >= holdMs) {
    controllerComboConsumed = true;
    executeControllerAction(action);
  }
}

void processDebug() {
  if (!DEBUG_SERIAL || !Serial.available()) return;
  char c = (char)Serial.read();

  if (c == 'c' || c == 'C') {
    state = FAULT;
    stopDrive();
    calibrateQTR();
    resetLineControl();
    if (calibrationValid) {
      pcaDigital(PCA_STBY_CH, true);
      state = LINE_FOLLOW;
    }
  } else if (c == 'p' || c == 'P') {
    printStatus();
    printFlood();
  } else if (c == 's' || c == 'S') {
    stopDrive();
    state = FINISHED;
  } else if (c == 'r' || c == 'R') {
    stopDrive();
    resetLineControl();
    resetMaze();
    pcaDigital(PCA_STBY_CH, true);
    state = LINE_FOLLOW;
  }
}

bool initMPU() {
  if (!mpu.begin()) return false;
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  delay(400);

  float sum = 0.0f;
  constexpr uint16_t N = 1000;
  for (uint16_t i = 0; i < N; ++i) {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    sum += gyro.gyro.z;
    delay(2);
  }
  gyroBiasZ = sum / (float)N;
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(250);
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableVirtualDevice(false);

  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    pinMode(MOTOR_PWM_PINS[i], OUTPUT);
    if (!ledcAttach(MOTOR_PWM_PINS[i], 20000, 8)) {
      state = FAULT;
      return;
    }
    ledcWrite(MOTOR_PWM_PINS[i], 0);
  }

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  if (!pca.begin()) {
    state = FAULT;
    return;
  }

  pcaReady = true;
  pca.setOutputMode(true);
  pca.setPWMFreq(50);
  pcaDigital(PCA_STBY_CH, false);
  stopDrive();

  qtr.setTypeRC();
  qtr.setSensorPins(QTR_PINS, QTR_COUNT);
  qtr.setTimeout(QTR_TIMEOUT_US);

  configureDay();
  resetMaze();
  resetLineControl();
  loadCalibration();

  if (!calibrationValid) {
    startupCalibrationNeeded = true;
    state = FAULT;
    Serial.println("CALIBRATION NEEDED");
  }

  if (!initMPU()) {
    state = FAULT;
    return;
  }
  mpuReady = true;

  if (!initToFs()) {
    state = FAULT;
    return;
  }
  tofReady = true;

  pcaDigital(PCA_STBY_CH, true);
  if (calibrationValid) {
    state = LINE_FOLLOW;
    Serial.printf("READY DAY %u\n", DAY_PROFILE);
  } else {
    state = FAULT;
  }
}

void loop() {
  processController();
  processDebug();

  if (state == FAULT || state == FINISHED || state == OPERATOR_STOP) {
    stopDrive();
    delay(1);
    return;
  }

  if (state == LINE_FOLLOW) {
    lineStep();
  } else if (state == LINE_TO_MAZE) {
    transitionStep();
  } else if (state == MAZE_SELECT) {
    mazeSelectStep();
  } else if (state == MAZE_TURN) {
    mazeTurnStep();
  } else if (state == MAZE_MOVE) {
    mazeMoveStep();
  }
}
