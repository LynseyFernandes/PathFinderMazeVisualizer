#include <Arduino.h>
#include <Wire.h>
#include <QTRSensors.h>
#include <VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>

constexpr uint8_t DAY_PROFILE = 1;
constexpr uint8_t SDA_PIN = 21;
constexpr uint8_t SCL_PIN = 22;
constexpr uint8_t PCA9685_ADDR = 0x40;
constexpr uint8_t PCA9548A_ADDR = 0x70;

constexpr uint8_t QTR_COUNT = 8;
constexpr uint8_t MOTOR_COUNT = 4;
constexpr uint8_t TOF_COUNT = 3;
constexpr uint8_t MAX_MAZE_W = 10;
constexpr uint8_t MAX_MAZE_H = 10;
constexpr uint16_t MAX_ASTAR_STATES = MAX_MAZE_W * MAX_MAZE_H * 4;
constexpr uint16_t MAX_HISTORY = MAX_MAZE_W * MAX_MAZE_H;
constexpr uint8_t MAX_PLAN = MAX_MAZE_W * MAX_MAZE_H;

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

constexpr uint16_t QTR_TIMEOUT_US = 1800;
constexpr uint16_t QTR_CALIBRATION_MS = 5000;
constexpr uint16_t LINE_PRESENT = 90;
constexpr uint16_t LINE_STRONG = 620;
constexpr uint16_t LINE_ALL_LOST = 180;
constexpr uint16_t LINE_DOT_MAX_MS = 450;
constexpr uint16_t LINE_ENTRY_CONFIRM_MS = 180;
constexpr uint16_t LINE_ENTRY_MAX_MS = 1400;
constexpr uint16_t LINE_TRANSITION_BLEND_MS = 320;
constexpr uint16_t LINE_SEARCH_PWM = 90;
constexpr uint16_t LINE_SHARP_PWM = 105;
constexpr uint16_t LINE_DAY1_BASE = 220;
constexpr uint16_t LINE_DAY2_BASE = 235;
constexpr float LINE_KP = 185.0f;
constexpr float LINE_KD = 120.0f;
constexpr float LINE_MAX_CORRECTION = 205.0f;
constexpr uint8_t ERROR_BUFFER_SIZE = 4;
constexpr uint16_t TRANSITION_PWM = 108;

constexpr uint16_t TOF_TIMING_US = 20000;
constexpr uint16_t TOF_PERIOD_MS = 25;
constexpr uint16_t TOF_TIMEOUT_MS = 30;
constexpr uint16_t TOF_WALL_ON_MM = 145;
constexpr uint16_t TOF_WALL_OFF_MM = 170;
constexpr uint16_t ENTRY_SIDE_MIN_MM = 25;
constexpr uint16_t ENTRY_SIDE_MAX_MM = 155;
constexpr uint16_t ENTRY_SIDE_SUM_MIN_MM = 70;
constexpr uint16_t ENTRY_SIDE_SUM_MAX_MM = 285;

constexpr uint16_t MAZE_PWM = 145;
constexpr uint16_t MAZE_TURN_PWM = 155;
constexpr uint16_t MAZE_TURN_PWM_SLOW = 92;
constexpr uint32_t MAZE_CELL_MS = 470;
constexpr uint32_t MAZE_CELL_MIN_MS = 260;
constexpr uint16_t MAZE_FRONT_BRAKE_MM = 95;
constexpr uint16_t MAZE_FRONT_STOP_MM = 72;
constexpr float MAZE_WALL_KP = 1.45f;
constexpr int16_t MAZE_MAX_WALL_CORRECTION = 55;
constexpr uint32_t MAZE_TURN_TIMEOUT_MS = 1200;
constexpr uint16_t TURN_COST_90 = 7;
constexpr bool USE_CHEBYSHEV = false;
constexpr bool DEBUG_SERIAL = true;
constexpr uint8_t MAX_STRAIGHT_CHAIN = 3;

QTRSensors qtr;
Adafruit_PWMServoDriver pca(PCA9685_ADDR);
Adafruit_MPU6050 mpu;
Preferences prefs;
VL53L0X tof[TOF_COUNT];

struct LineState {
  uint16_t raw[QTR_COUNT];
  uint16_t white[QTR_COUNT];
  uint8_t mask;
  uint32_t total;
  float error;
  float lastError;
  float derivative;
  bool detected;
};

struct ToFState {
  uint16_t mm[TOF_COUNT];
  bool valid[TOF_COUNT];
  bool wall[TOF_COUNT];
};

struct MazeCell {
  uint8_t known;
  uint8_t blocked;
  uint8_t flags;
};

struct ParentCell {
  int8_t x;
  int8_t y;
  bool valid;
};

struct GoalRegion {
  uint8_t x0;
  uint8_t x1;
  uint8_t y0;
  uint8_t y1;
};

struct ANode {
  uint16_t g;
  uint16_t f;
  int16_t parent;
  int16_t heapPos;
  bool closed;
};

struct MotionSegment {
  uint8_t type;
  uint8_t count;
  int8_t turn;
};

LineState lineState{};
ToFState tofState{};
MazeCell maze[MAX_MAZE_H][MAX_MAZE_W]{};
ParentCell parentCell[MAX_MAZE_H][MAX_MAZE_W]{};
uint8_t flood[MAX_MAZE_H][MAX_MAZE_W]{};
uint16_t floodQueue[MAX_MAZE_W * MAX_MAZE_H]{};
uint16_t errorRaw[ERROR_BUFFER_SIZE]{};
float errorBuf[ERROR_BUFFER_SIZE]{};
uint32_t errorTime[ERROR_BUFFER_SIZE]{};
uint8_t errorHead = 0;
uint8_t errorCount = 0;
uint16_t qtrRaw[QTR_COUNT]{};
uint16_t qtrMin[QTR_COUNT]{};
uint16_t qtrMax[QTR_COUNT]{};
bool calibrationValid = false;

ANode astar[MAX_ASTAR_STATES]{};
uint16_t astarHeap[MAX_ASTAR_STATES]{};
uint16_t heapSize = 0;
int16_t astarSolution[MAX_ASTAR_STATES]{};
uint16_t astarSolutionSize = 0;
MotionSegment plan[MAX_PLAN]{};
uint16_t planSize = 0;

uint16_t history[MAX_HISTORY]{};
uint16_t historySize = 0;

uint8_t mazeW = DAY1_W;
uint8_t mazeH = DAY1_H;
GoalRegion goal{DAY1_GOAL_X0, DAY1_GOAL_X1, DAY1_GOAL_Y0, DAY1_GOAL_Y1};

int8_t mazeX = START_X;
int8_t mazeY = START_Y;
uint8_t mazeHeading = START_HEADING;
uint8_t targetHeading = START_HEADING;

uint32_t lineGapStart = 0;
uint32_t transitionStart = 0;
uint32_t lastTofService = 0;
uint8_t tofServiceIndex = 0;
int16_t lastLeftCommand = 0;
int16_t lastRightCommand = 0;

float gyroBiasZ = 0.0f;
float yawDeg = 0.0f;
float turnDeltaDeg = 0.0f;
float turnSign = 0.0f;
uint32_t yawTimeUs = 0;
uint32_t turnStartMs = 0;
uint32_t moveStartMs = 0;

bool pcaReady = false;
bool tofReady = false;
bool mpuReady = false;
bool motorDirValid[MOTOR_COUNT]{};
bool motorDirState[MOTOR_COUNT]{};

uint8_t lineTurnDirection = 0;
uint8_t junctionHold = 0;

enum RobotState : uint8_t {
  LINE_FOLLOW,
  LINE_TO_MAZE,
  MAZE_SELECT,
  MAZE_TURN,
  MAZE_MOVE,
  FINISHED,
  FAULT
};

RobotState state = LINE_FOLLOW;

float fclamp(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

int iclamp(int v, int lo, int hi) {
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

void pcaDigital(uint8_t ch, bool value) {
  if (!pcaReady) return;
  pca.setPin(ch, value ? 4095 : 0, false);
}

void motorDir(uint8_t i, bool forward) {
  if (MOTOR_INVERT[i]) forward = !forward;
  if (!motorDirValid[i] || motorDirState[i] != forward) {
    pcaDigital(PCA_MOTOR_DIR_A[i], forward);
    pcaDigital(PCA_MOTOR_DIR_B[i], !forward);
    motorDirState[i] = forward;
    motorDirValid[i] = true;
  }
}

void setMotor(uint8_t i, int value) {
  value = iclamp(value, -255, 255);
  if (value == 0) {
    ledcWrite(MOTOR_PWM_PINS[i], 0);
    pcaDigital(PCA_MOTOR_DIR_A[i], false);
    pcaDigital(PCA_MOTOR_DIR_B[i], false);
    return;
  }
  motorDir(i, value > 0);
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
  }
  lastLeftCommand = 0;
  lastRightCommand = 0;
}

void hardFault() {
  stopDrive();
  pcaDigital(PCA_STBY_CH, false);
  state = FAULT;
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
    tof[i].setMeasurementTimingBudget(TOF_TIMING_US);
    tof[i].startContinuous(TOF_PERIOD_MS);
  }
  return true;
}

void updateToFOne(uint8_t i) {
  selectMux(TOF_MUX_CHANNEL[i]);
  uint16_t d = tof[i].readRangeContinuousMillimeters();
  if (tof[i].timeoutOccurred() || d == 0 || d == 65535) {
    tofState.valid[i] = false;
    return;
  }
  tofState.valid[i] = true;
  tofState.mm[i] = d;
  if (tofState.wall[i]) {
    if (d >= TOF_WALL_OFF_MM) tofState.wall[i] = false;
  } else {
    if (d <= TOF_WALL_ON_MM) tofState.wall[i] = true;
  }
}

void serviceToFs(bool forceAll = false) {
  uint32_t now = millis();
  if (forceAll) {
    for (uint8_t i = 0; i < TOF_COUNT; ++i) updateToFOne(i);
    tofServiceIndex = 0;
    lastTofService = now;
    return;
  }
  if (now - lastTofService < TOF_PERIOD_MS) return;
  lastTofService = now;
  updateToFOne(tofServiceIndex);
  tofServiceIndex = (tofServiceIndex + 1) % TOF_COUNT;
}

void loadCalibration() {
  prefs.begin("qtr", true);
  calibrationValid = prefs.getBool("valid", false);
  for (uint8_t i = 0; i < QTR_COUNT; ++i) {
    char n[8], x[8];
    snprintf(n, sizeof(n), "n%u", i);
    snprintf(x, sizeof(x), "x%u", i);
    qtrMin[i] = prefs.getUShort(n, 0);
    qtrMax[i] = prefs.getUShort(x, QTR_TIMEOUT_US);
    if (qtrMax[i] <= qtrMin[i] + 30) calibrationValid = false;
  }
  prefs.end();
}

void saveCalibration() {
  prefs.begin("qtr", false);
  prefs.putBool("valid", calibrationValid);
  for (uint8_t i = 0; i < QTR_COUNT; ++i) {
    char n[8], x[8];
    snprintf(n, sizeof(n), "n%u", i);
    snprintf(x, sizeof(x), "x%u", i);
    prefs.putUShort(n, qtrMin[i]);
    prefs.putUShort(x, qtrMax[i]);
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
      if (qtrRaw[i] < lo[i]) lo[i] = qtrRaw[i];
      if (qtrRaw[i] > hi[i]) hi[i] = qtrRaw[i];
    }
  }
  calibrationValid = true;
  for (uint8_t i = 0; i < QTR_COUNT; ++i) {
    qtrMin[i] = lo[i];
    qtrMax[i] = hi[i];
    if (hi[i] <= lo[i] + 30) calibrationValid = false;
  }
  if (calibrationValid) saveCalibration();
}

uint16_t whiteStrength(uint8_t i, uint16_t raw) {
  if (!calibrationValid) return 0;
  uint16_t lo = qtrMin[i];
  uint16_t hi = qtrMax[i];
  if (hi <= lo + 1) return 0;
  raw = (uint16_t)iclamp(raw, lo, hi);
  return (uint16_t)(((uint32_t)(hi - raw) * 1000u) / (hi - lo));
}

void resetLineFilter() {
  memset(errorRaw, 0, sizeof(errorRaw));
  memset(errorBuf, 0, sizeof(errorBuf));
  memset(errorTime, 0, sizeof(errorTime));
  errorHead = 0;
  errorCount = 0;
  lineState.error = 0;
  lineState.lastError = 0;
  lineState.derivative = 0;
}

void readLineSensors() {
  static const int16_t weights[4] = {3500, 1500, -1500, -3500};
  qtr.read(qtrRaw);
  uint32_t total = 0;
  int32_t weighted = 0;
  uint8_t mask = 0;
  for (uint8_t i = 0; i < QTR_COUNT; ++i) {
    lineState.raw[i] = qtrRaw[i];
    lineState.white[i] = whiteStrength(i, qtrRaw[i]);
    if (lineState.white[i] >= LINE_PRESENT) mask |= (uint8_t)(1u << i);
    total += lineState.white[i];
  }
  uint32_t centerTotal = lineState.white[2] + lineState.white[3] + lineState.white[4] + lineState.white[5];
  weighted += (int32_t)lineState.white[2] * weights[0];
  weighted += (int32_t)lineState.white[3] * weights[1];
  weighted += (int32_t)lineState.white[4] * weights[2];
  weighted += (int32_t)lineState.white[5] * weights[3];
  lineState.total = total;
  lineState.mask = mask;
  lineState.detected = centerTotal >= LINE_PRESENT;
  if (lineState.detected) {
    lineState.error = fclamp((float)weighted / (float)max((uint32_t)1, centerTotal), -3500.0f, 3500.0f) / 3500.0f;
  }
}

void updateDerivative() {
  uint32_t now = micros();
  errorBuf[errorHead] = lineState.error;
  errorTime[errorHead] = now;
  errorHead = (errorHead + 1) % ERROR_BUFFER_SIZE;
  if (errorCount < ERROR_BUFFER_SIZE) ++errorCount;
  uint8_t oldest = (errorHead + ERROR_BUFFER_SIZE - errorCount) % ERROR_BUFFER_SIZE;
  lineState.derivative = (lineState.error - errorBuf[oldest]) / (float)(errorCount > 1 ? errorCount - 1 : 1);
}

uint16_t lineSpeed(float severity) {
  static const uint8_t table[11] = {245, 243, 240, 235, 228, 218, 205, 188, 168, 143, 118};
  severity = fclamp(severity, 0.0f, 1.0f);
  float p = severity * 10.0f;
  uint8_t i = (uint8_t)p;
  if (i >= 10) return table[10];
  float t = p - i;
  return (uint16_t)roundf(table[i] + t * (table[i + 1] - table[i]));
}

bool leftExtreme() {
  return lineState.white[6] >= LINE_STRONG || lineState.white[7] >= LINE_STRONG;
}

bool rightExtreme() {
  return lineState.white[0] >= LINE_STRONG || lineState.white[1] >= LINE_STRONG;
}

bool centerStrong() {
  return lineState.white[3] >= LINE_STRONG || lineState.white[4] >= LINE_STRONG;
}

bool broadWhite() {
  uint8_t n = 0;
  for (uint8_t i = 0; i < QTR_COUNT; ++i) if (lineState.white[i] >= LINE_STRONG) ++n;
  return n >= 7;
}

bool possibleEntry() {
  return !lineState.detected && lineState.total <= LINE_ALL_LOST;
}

bool entryConfirmed() {
  if (!tofState.valid[0] || !tofState.valid[2]) return false;
  uint16_t l = tofState.mm[0];
  uint16_t r = tofState.mm[2];
  return l >= ENTRY_SIDE_MIN_MM && l <= ENTRY_SIDE_MAX_MM &&
         r >= ENTRY_SIDE_MIN_MM && r <= ENTRY_SIDE_MAX_MM &&
         (uint32_t)l + r >= ENTRY_SIDE_SUM_MIN_MM &&
         (uint32_t)l + r <= ENTRY_SIDE_SUM_MAX_MM;
}

void lineControl() {
  float severity = fclamp(max(abs(lineState.error), fclamp(abs(lineState.derivative) * 0.035f, 0.0f, 1.0f)), 0.0f, 1.0f);
  uint16_t cap = DAY_PROFILE == 2 ? LINE_DAY2_BASE : LINE_DAY1_BASE;
  uint16_t base = min(cap, lineSpeed(severity));
  float correction = LINE_KP * lineState.error + LINE_KD * lineState.derivative;
  correction = fclamp(correction, -LINE_MAX_CORRECTION, LINE_MAX_CORRECTION);
  setDrive(iclamp((int)roundf(base + correction), -255, 255), iclamp((int)roundf(base - correction), -255, 255));
}

void lineSharpTurn(bool left) {
  if (left) setDrive(-LINE_SHARP_PWM, LINE_SHARP_PWM);
  else setDrive(LINE_SHARP_PWM, -LINE_SHARP_PWM);
}

void lineStep() {
  readLineSensors();
  updateDerivative();

  if (possibleEntry()) {
    if (!lineGapStart) {
      lineGapStart = millis();
      serviceToFs(true);
    } else {
      serviceToFs(false);
    }
    if (millis() - lineGapStart >= LINE_ENTRY_CONFIRM_MS) {
      serviceToFs(true);
      if (entryConfirmed()) {
        transitionStart = millis();
        state = LINE_TO_MAZE;
        return;
      }
    }
    if (millis() - lineGapStart <= LINE_DOT_MAX_MS) {
      int s = lineState.lastError >= 0 ? 1 : -1;
      setDrive(s > 0 ? TRANSITION_PWM + 12 : TRANSITION_PWM - 12,
               s > 0 ? TRANSITION_PWM - 12 : TRANSITION_PWM + 12);
      return;
    }
    setDrive(TRANSITION_PWM, TRANSITION_PWM);
    if (millis() - lineGapStart > LINE_ENTRY_MAX_MS) hardFault();
    return;
  }

  lineGapStart = 0;

  bool l = leftExtreme();
  bool r = rightExtreme();
  bool c = centerStrong();

  if (l && r && !c) {
    lineSharpTurn(true);
    lineState.lastError = lineState.error;
    return;
  }

  if (l && !r && !c) {
    lineSharpTurn(true);
    lineState.lastError = lineState.error;
    return;
  }

  if (r && !l && !c) {
    lineSharpTurn(false);
    lineState.lastError = lineState.error;
    return;
  }

  if (broadWhite()) {
    lineControl();
    lineState.lastError = lineState.error;
    return;
  }

  if (!lineState.detected) {
    int s = lineState.lastError >= 0 ? 1 : -1;
    setDrive(s > 0 ? -LINE_SEARCH_PWM : LINE_SEARCH_PWM,
             s > 0 ? LINE_SEARCH_PWM : -LINE_SEARCH_PWM);
    return;
  }

  lineControl();
  lineState.lastError = lineState.error;
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
  memset(parentCell, 0, sizeof(parentCell));
  memset(history, 0, sizeof(history));
  historySize = 0;
  mazeX = START_X;
  mazeY = START_Y;
  mazeHeading = START_HEADING;
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

void observeWall(int x, int y, uint8_t dir, bool blocked) {
  if (!inMaze(x, y)) return;
  int nx = x + DX[dir];
  int ny = y + DY[dir];
  maze[y][x].known |= WALL_BIT[dir];
  if (blocked) maze[y][x].blocked |= WALL_BIT[dir];
  else maze[y][x].blocked &= (uint8_t)~WALL_BIT[dir];
  if (!inMaze(nx, ny)) return;
  maze[ny][nx].known |= WALL_OPP[dir];
  if (blocked) maze[ny][nx].blocked |= WALL_OPP[dir];
  else maze[ny][nx].blocked &= (uint8_t)~WALL_OPP[dir];
}

void updateMazeMap() {
  uint8_t left = (mazeHeading + 3) & 3;
  uint8_t front = mazeHeading;
  uint8_t right = (mazeHeading + 1) & 3;
  if (tofState.valid[0]) observeWall(mazeX, mazeY, left, tofState.wall[0]);
  if (tofState.valid[1]) observeWall(mazeX, mazeY, front, tofState.wall[1]);
  if (tofState.valid[2]) observeWall(mazeX, mazeY, right, tofState.wall[2]);
}

bool blocked(int x, int y, uint8_t dir) {
  if (!inMaze(x, y)) return true;
  return (maze[y][x].blocked & WALL_BIT[dir]) != 0;
}

bool known(int x, int y, uint8_t dir) {
  if (!inMaze(x, y)) return true;
  return (maze[y][x].known & WALL_BIT[dir]) != 0;
}

bool knownOpenEdge(int x, int y, uint8_t dir) {
  if (!inMaze(x, y) || !known(x, y, dir) || blocked(x, y, dir)) return false;
  int nx = x + DX[dir];
  int ny = y + DY[dir];
  if (!inMaze(nx, ny)) return false;
  uint8_t od = (dir + 2) & 3;
  return known(nx, ny, od) && !blocked(nx, ny, od);
}

void recomputeFlood() {
  for (uint8_t y = 0; y < mazeH; ++y) for (uint8_t x = 0; x < mazeW; ++x) flood[y][x] = 255;
  uint16_t head = 0, tail = 0;
  for (uint8_t y = goal.y0; y <= goal.y1; ++y) {
    for (uint8_t x = goal.x0; x <= goal.x1; ++x) {
      flood[y][x] = 0;
      floodQueue[tail++] = (uint16_t)(y * mazeW + x);
    }
  }
  while (head < tail) {
    uint16_t id = floodQueue[head++];
    uint8_t x = id % mazeW;
    uint8_t y = id / mazeW;
    uint8_t nv = flood[y][x] == 255 ? 255 : flood[y][x] + 1;
    for (uint8_t d = 0; d < 4; ++d) {
      if (blocked(x, y, d)) continue;
      int nx = x + DX[d];
      int ny = y + DY[d];
      if (!inMaze(nx, ny)) continue;
      if (flood[ny][nx] > nv) {
        flood[ny][nx] = nv;
        floodQueue[tail++] = (uint16_t)(ny * mazeW + nx);
      }
    }
  }
}

uint8_t turnsBetween(uint8_t a, uint8_t b) {
  uint8_t d = (b + 4 - a) & 3;
  return d > 2 ? 4 - d : d;
}

bool floodNext(uint8_t &best) {
  recomputeFlood();
  uint16_t bestValue = 65535;
  bool bestUnvisited = false;
  uint8_t bestTurns = 255;
  bool found = false;
  uint8_t parentDir = 255;
  if (parentCell[mazeY][mazeX].valid) {
    int px = parentCell[mazeY][mazeX].x;
    int py = parentCell[mazeY][mazeX].y;
    for (uint8_t d = 0; d < 4; ++d) if (mazeX + DX[d] == px && mazeY + DY[d] == py) parentDir = d;
  }

  for (uint8_t d = 0; d < 4; ++d) {
    if (blocked(mazeX, mazeY, d)) continue;
    int nx = mazeX + DX[d];
    int ny = mazeY + DY[d];
    if (!inMaze(nx, ny)) continue;
    uint16_t v = flood[ny][nx];
    bool uv = (maze[ny][nx].flags & 1) == 0;
    uint8_t tr = turnsBetween(mazeHeading, d);
    uint16_t score = v == 255 ? 60000 : v;
    if (!found || score < bestValue ||
        (score == bestValue && uv && !bestUnvisited) ||
        (score == bestValue && uv == bestUnvisited && tr < bestTurns)) {
      found = true;
      best = d;
      bestValue = score;
      bestUnvisited = uv;
      bestTurns = tr;
    }
  }

  if (found && bestValue < 60000) return true;

  if (parentDir < 4 && !blocked(mazeX, mazeY, parentDir)) {
    best = parentDir;
    return true;
  }

  for (uint8_t d = 0; d < 4; ++d) {
    if (!blocked(mazeX, mazeY, d)) {
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

uint16_t aHeuristic(int x, int y) {
  int dx = x < goal.x0 ? goal.x0 - x : x > goal.x1 ? x - goal.x1 : 0;
  int dy = y < goal.y0 ? goal.y0 - y : y > goal.y1 ? y - goal.y1 : 0;
  if (USE_CHEBYSHEV) return (uint16_t)(max(dx, dy) * 100);
  return (uint16_t)((dx + dy) * 100);
}

int aStateId(uint8_t x, uint8_t y, uint8_t h) {
  return ((y * mazeW + x) * 4 + h);
}

void aDecode(int id, uint8_t &x, uint8_t &y, uint8_t &h) {
  h = id % 4;
  uint16_t c = id / 4;
  x = c % mazeW;
  y = c / mazeW;
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
    if (l >= heapSize) break;
    uint16_t r = l + 1;
    uint16_t b = l;
    if (r < heapSize && heapLess(astarHeap[r], astarHeap[l])) b = r;
    if (!heapLess(astarHeap[b], astarHeap[p])) break;
    heapSwap(p, b);
    p = b;
  }
}

void heapPush(uint16_t s) {
  if (astar[s].heapPos < 0) {
    if (heapSize >= MAX_ASTAR_STATES) return;
    uint16_t p = heapSize++;
    astarHeap[p] = s;
    astar[s].heapPos = p;
  }
  heapUp(astar[s].heapPos);
}

int heapPop() {
  if (!heapSize) return -1;
  uint16_t root = astarHeap[0];
  --heapSize;
  if (heapSize) {
    astarHeap[0] = astarHeap[heapSize];
    astar[astarHeap[0]].heapPos = 0;
    heapDown(0);
  }
  astar[root].heapPos = -1;
  return root;
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
  heapSize = 0;
  astarSolutionSize = 0;
  planSize = 0;

  int start = aStateId((uint8_t)mazeX, (uint8_t)mazeY, mazeHeading);
  astar[start].g = 0;
  astar[start].f = aHeuristic(mazeX, mazeY);
  heapPush(start);

  int goalState = -1;

  while (heapSize) {
    int cur = heapPop();
    if (cur < 0) break;
    if (astar[cur].closed) continue;
    astar[cur].closed = true;
    uint8_t x, y, h;
    aDecode(cur, x, y, h);
    if (inGoal(x, y)) {
      goalState = cur;
      break;
    }
    for (uint8_t d = 0; d < 4; ++d) {
      if (!knownOpenEdge(x, y, d)) continue;
      uint8_t nx = x + DX[d];
      uint8_t ny = y + DY[d];
      int next = aStateId(nx, ny, d);
      if (astar[next].closed) continue;
      uint32_t ng = (uint32_t)astar[cur].g + aEdgeCost(h, d);
      if (ng >= 65535) continue;
      if (ng < astar[next].g) {
        astar[next].g = ng;
        uint32_t nf = ng + aHeuristic(nx, ny);
        astar[next].f = nf > 65535 ? 65535 : (uint16_t)nf;
        astar[next].parent = cur;
        heapPush(next);
      }
    }
  }

  if (goalState < 0) return false;

  int s = goalState;
  while (s >= 0 && astarSolutionSize < MAX_ASTAR_STATES) {
    astarSolution[astarSolutionSize++] = s;
    s = astar[s].parent;
  }
  if (astarSolutionSize < 2) return true;
  for (uint16_t i = 0; i < astarSolutionSize / 2; ++i) {
    int16_t t = astarSolution[i];
    astarSolution[i] = astarSolution[astarSolutionSize - 1 - i];
    astarSolution[astarSolutionSize - 1 - i] = t;
  }
  uint8_t x0, y0, h0;
  aDecode(astarSolution[0], x0, y0, h0);
  uint8_t firstX, firstY, firstH;
  aDecode(astarSolution[1], firstX, firstY, firstH);
  firstDir = firstH;
  uint8_t runDir = h0;
  uint8_t runCount = 0;
  for (uint16_t i = 1; i < astarSolutionSize; ++i) {
    uint8_t x, y, h;
    aDecode(astarSolution[i], x, y, h);
    if (h != runDir) {
      if (runCount && planSize < MAX_PLAN) plan[planSize++] = {0, runCount, 0};
      int delta = (int)h - (int)runDir;
      if (delta > 2) delta -= 4;
      if (delta < -2) delta += 4;
      if (planSize < MAX_PLAN) plan[planSize++] = {1, 1, (int8_t)delta};
      runDir = h;
      runCount = 0;
    }
    ++runCount;
  }
  if (runCount && planSize < MAX_PLAN) plan[planSize++] = {0, runCount, 0};
  return true;
}

void startMazeTurn(uint8_t dir) {
  uint8_t delta = (dir + 4 - mazeHeading) & 3;
  if (!delta) {
    state = MAZE_MOVE;
    moveStartMs = millis();
    return;
  }
  targetHeading = dir;
  turnDeltaDeg = delta == 2 ? 180.0f : 90.0f;
  bool right = delta == 1 || delta == 2;
  turnSign = right ? 1.0f : -1.0f;
  yawDeg = 0;
  yawTimeUs = micros();
  turnStartMs = millis();
  state = MAZE_TURN;
  int p = MAZE_TURN_PWM;
  setDrive(right ? p : -p, right ? -p : p);
}

void updateYaw() {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  uint32_t now = micros();
  uint32_t dtUs = now - yawTimeUs;
  yawTimeUs = now;
  if (dtUs > 100000) return;
  yawDeg += (gyro.gyro.z - gyroBiasZ) * 57.2957795f * ((float)dtUs / 1000000.0f);
}

void mazeTurnStep() {
  updateYaw();
  float a = fabsf(yawDeg);
  if (a >= turnDeltaDeg) {
    stopDrive();
    mazeHeading = targetHeading;
    state = MAZE_MOVE;
    moveStartMs = millis();
    return;
  }
  if (millis() - turnStartMs > MAZE_TURN_TIMEOUT_MS) {
    hardFault();
    return;
  }
  int p = a > turnDeltaDeg * 0.7f ? MAZE_TURN_PWM_SLOW : MAZE_TURN_PWM;
  setDrive(turnSign > 0 ? p : -p, turnSign > 0 ? -p : p);
}

float mazeWallCorrection() {
  if (!tofState.valid[0] || !tofState.valid[2]) return 0;
  if (!tofState.wall[0] || !tofState.wall[2]) return 0;
  return MAZE_WALL_KP * ((float)tofState.mm[0] - (float)tofState.mm[2]);
}

void mazeMoveStep() {
  serviceToFs(false);
  uint32_t elapsed = millis() - moveStartMs;
  float corr = fclamp(mazeWallCorrection(), -MAZE_MAX_WALL_CORRECTION, MAZE_MAX_WALL_CORRECTION);
  int base = MAZE_PWM;

  if (tofState.valid[1] && tofState.wall[1] && tofState.mm[1] < MAZE_FRONT_BRAKE_MM && elapsed > MAZE_CELL_MIN_MS) {
    float t = fclamp((float)(tofState.mm[1] - MAZE_FRONT_STOP_MM) / (float)(MAZE_FRONT_BRAKE_MM - MAZE_FRONT_STOP_MM), 0.15f, 1.0f);
    base = (int)(MAZE_PWM * t);
  }

  setDrive(iclamp((int)(base - corr), 0, 255), iclamp((int)(base + corr), 0, 255));

  bool stopAtFront = tofState.valid[1] && tofState.wall[1] && tofState.mm[1] <= MAZE_FRONT_STOP_MM && elapsed >= (MAZE_CELL_MS * 65UL / 100UL);
  bool stopAtTime = elapsed >= MAZE_CELL_MS;

  if (stopAtFront || stopAtTime) {
    stopDrive();
    int nx = mazeX + DX[mazeHeading];
    int ny = mazeY + DY[mazeHeading];
    if (!inMaze(nx, ny)) {
      hardFault();
      return;
    }
    if (!(maze[ny][nx].flags & 1)) {
      parentCell[ny][nx] = {(int8_t)mazeX, (int8_t)mazeY, true};
    }
    mazeX = nx;
    mazeY = ny;
    maze[mazeY][mazeX].flags |= 1;
    if (historySize < MAX_HISTORY) history[historySize++] = (uint16_t)(mazeY * mazeW + mazeX);
    state = MAZE_SELECT;
  }
}

void transitionStep() {
  serviceToFs(false);
  float a = fclamp((float)(millis() - transitionStart) / (float)LINE_TRANSITION_BLEND_MS, 0.0f, 1.0f);
  float corr = fclamp(mazeWallCorrection(), -30.0f, 30.0f);
  float mazeL = TRANSITION_PWM - corr;
  float mazeR = TRANSITION_PWM + corr;
  int l = (int)roundf((1.0f - a) * lastLeftCommand + a * mazeL);
  int r = (int)roundf((1.0f - a) * lastRightCommand + a * mazeR);
  setDrive(l, r);
  if (a >= 1.0f) {
    resetMaze();
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
  bool aStarSafe = false;
  uint8_t aStarDir = mazeHeading;
  if (buildKnownAStar(aStarDir)) {
    aStarSafe = true;
  }

  if (aStarSafe) next = aStarDir;
  else if (!floodNext(next)) {
    hardFault();
    return;
  }

  startMazeTurn(next);
}

void printStatus() {
  Serial.printf("DAY=%u STATE=%u XY=%d,%d H=%u QTR=0x%02X TOF=%u,%u,%u\n",
                DAY_PROFILE,
                (unsigned)state,
                mazeX,
                mazeY,
                mazeHeading,
                lineState.mask,
                tofState.mm[0],
                tofState.mm[1],
                tofState.mm[2]);
}

void processDebug() {
  if (!DEBUG_SERIAL || !Serial.available()) return;
  char c = (char)Serial.read();
  if (c == 'c' || c == 'C') {
    state = FINISHED;
    calibrateQTR();
    resetLineFilter();
    if (calibrationValid) state = LINE_FOLLOW;
  } else if (c == 's' || c == 'S') {
    stopDrive();
    state = FINISHED;
  } else if (c == 'r' || c == 'R') {
    stopDrive();
    resetMaze();
    resetLineFilter();
    state = LINE_FOLLOW;
  } else if (c == 'p' || c == 'P') {
    recomputeFlood();
    printStatus();
    for (uint8_t y = mazeH; y-- > 0;) {
      for (uint8_t x = 0; x < mazeW; ++x) Serial.printf("%3u ", flood[y][x]);
      Serial.println();
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

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

  qtr.setTypeRC();
  qtr.setSensorPins(QTR_PINS, QTR_COUNT);
  qtr.setTimeout(QTR_TIMEOUT_US);

  if (!mpu.begin()) {
    hardFault();
    return;
  }
  mpuReady = true;
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  gyroBiasZ = 0.0f;
  for (uint16_t i = 0; i < 1200; ++i) {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    gyroBiasZ += gyro.gyro.z;
    delay(2);
  }
  gyroBiasZ /= 1200.0f;

  configureDay();
  loadCalibration();
  resetLineFilter();
  resetMaze();

  if (!calibrationValid) {
    Serial.println("QTR calibration required: send C");
    pcaDigital(PCA_STBY_CH, false);
    state = FAULT;
    return;
  }

  if (!initToFs()) {
    hardFault();
    return;
  }
  tofReady = true;
  pcaDigital(PCA_STBY_CH, true);
  Serial.printf("READY DAY %u\n", DAY_PROFILE);
  Serial.println("LINE -> MAZE is automatic");
  state = LINE_FOLLOW;
}

void loop() {
  processDebug();
  if (state == FAULT || state == FINISHED) {
    stopDrive();
    delay(1);
    return;
  }
  if (state == LINE_FOLLOW) lineStep();
  else if (state == LINE_TO_MAZE) transitionStep();
  else if (state == MAZE_SELECT) mazeSelectStep();
  else if (state == MAZE_TURN) mazeTurnStep();
  else if (state == MAZE_MOVE) mazeMoveStep();
}
