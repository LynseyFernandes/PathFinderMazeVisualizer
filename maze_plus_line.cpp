#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>

constexpr uint8_t SDA_PIN = 21;
constexpr uint8_t SCL_PIN = 22;
constexpr uint8_t PCA_ADDR = 0x40;
constexpr uint8_t LINE_COUNT = 8;
constexpr uint8_t MOTOR_COUNT = 4;
constexpr uint8_t TOF_COUNT = 3;

const uint8_t LINE_PINS[LINE_COUNT] = {4, 16, 17, 18, 19, 27, 32, 33};
const uint8_t MOTOR_PWM[MOTOR_COUNT] = {13, 14, 25, 26};
const uint8_t MOTOR_DIR_A[MOTOR_COUNT] = {3, 5, 7, 9};
const uint8_t MOTOR_DIR_B[MOTOR_COUNT] = {4, 6, 8, 10};
const bool MOTOR_INVERT[MOTOR_COUNT] = {false, true, false, true};
const uint8_t STBY_CH = 11;
const uint8_t TOF_XSHUT_CH[TOF_COUNT] = {12, 13, 14};
const uint8_t TOF_ADDRESS[TOF_COUNT] = {0x30, 0x31, 0x32};
const uint8_t SERVO_CH[3] = {0, 1, 2};

constexpr bool SENSOR_REVERSED = false;
constexpr bool WHITE_IS_LOW_TIME = true;
constexpr uint16_t QTR_CHARGE_US = 10;
constexpr uint16_t QTR_TIMEOUT_US = 2500;
constexpr uint16_t LINE_ON = 220;
constexpr uint16_t LINE_LOST = 80;
constexpr float LINE_KP = 1.15f;
constexpr float LINE_KD = 0.020f;
constexpr float LINE_MAX_CORRECTION = 210.0f;

constexpr uint8_t ERR_HISTORY = 4;
constexpr uint16_t TOF_PERIOD_MS = 20;
constexpr uint16_t TOF_WALL_ON_MM = 140;
constexpr uint16_t TOF_WALL_OFF_MM = 165;

constexpr uint8_t MAZE_W = 16;
constexpr uint8_t MAZE_H = 16;
constexpr uint8_t WALL_N = 1;
constexpr uint8_t WALL_E = 2;
constexpr uint8_t WALL_S = 4;
constexpr uint8_t WALL_W = 8;
constexpr uint8_t CELL_VISITED = 1;
constexpr uint8_t INF8 = 255;

constexpr int16_t MAZE_PWM = 145;
constexpr int16_t MAZE_TURN_PWM = 155;
constexpr uint32_t MAZE_CELL_MS = 360;
constexpr uint32_t MAZE_TURN_90_MS = 185;
constexpr uint16_t FAST_PWM = 220;
constexpr uint32_t FAST_CARDINAL_CELL_MS = 250;
constexpr uint32_t FAST_DIAGONAL_CELL_MS = 290;
constexpr uint32_t FAST_TURN_45_MS = 75;
constexpr float WALL_KP = 1.20f;

constexpr bool USE_CHEBYSHEV = false;
constexpr bool ENABLE_DIAGONAL_ASTAR = true;
constexpr uint16_t CARDINAL_COST = 100;
constexpr uint16_t DIAGONAL_COST = 141;
constexpr uint16_t TURN45_COST = 18;

constexpr uint8_t HCOUNT = 8;
constexpr uint16_t STATE_COUNT = MAZE_W * MAZE_H * HCOUNT;
constexpr uint16_t MAX_PLAN = STATE_COUNT;

Adafruit_PWMServoDriver pca(PCA_ADDR);
VL53L0X tof[TOF_COUNT];
Preferences preferences;

struct LineCalibration {
  uint16_t minValue[LINE_COUNT];
  uint16_t maxValue[LINE_COUNT];
  bool valid;
};

struct LineState {
  uint16_t raw[LINE_COUNT];
  uint16_t strength[LINE_COUNT];
  uint8_t mask;
  int16_t position;
  float error;
  float derivative;
  bool detected;
};

struct ErrorSample {
  float value;
  uint32_t timeUs;
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

struct GoalRegion {
  int8_t xmin;
  int8_t xmax;
  int8_t ymin;
  int8_t ymax;
};

struct AStarNode {
  uint16_t g;
  uint16_t f;
  int16_t parent;
  int16_t heapPos;
  bool closed;
};

struct MotionSegment {
  uint8_t type;
  uint16_t length;
  int8_t turn45;
};

LineCalibration lineCalibration{};
LineState lineState{};
ErrorSample errorHistory[ERR_HISTORY]{};
ToFState tofState{};
MazeCell maze[MAZE_H][MAZE_W]{};
uint8_t flood[MAZE_H][MAZE_W]{};
uint16_t floodQueue[MAZE_W * MAZE_H]{};
AStarNode astar[STATE_COUNT]{};
uint16_t astarHeap[STATE_COUNT]{};
int16_t solutionStates[STATE_COUNT]{};
MotionSegment motionPlan[MAX_PLAN]{};

uint8_t errorHead = 0;
uint8_t errorCount = 0;
float lastLineDirection = 1.0f;
uint32_t lastTofUpdate = 0;
uint16_t astarHeapSize = 0;
uint16_t solutionCount = 0;
uint16_t motionPlanCount = 0;
int8_t mazeX = 0;
int8_t mazeY = 0;
uint8_t mazeHeading = 0;

GoalRegion goal = {7, 8, 7, 8};

float absf(float x) {
  return x < 0 ? -x : x;
}

int clampInt(int x, int lo, int hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

float clampFloat(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

bool inMaze(int x, int y) {
  return x >= 0 && x < MAZE_W && y >= 0 && y < MAZE_H;
}

bool inGoal(int x, int y) {
  return x >= goal.xmin && x <= goal.xmax &&
         y >= goal.ymin && y <= goal.ymax;
}

uint8_t oppositeWall(uint8_t w) {
  if (w == WALL_N) return WALL_S;
  if (w == WALL_E) return WALL_W;
  if (w == WALL_S) return WALL_N;
  return WALL_E;
}

const int8_t DX4[4] = {0, 1, 0, -1};
const int8_t DY4[4] = {1, 0, -1, 0};
const uint8_t WALL4[4] = {WALL_N, WALL_E, WALL_S, WALL_W};

const int8_t DX8[8] = {0, 1, 1, 1, 0, -1, -1, -1};
const int8_t DY8[8] = {1, 1, 0, -1, -1, -1, 0, 1};

void pcaDigital(uint8_t channel, bool high) {
  pca.setPin(channel, high ? 4095 : 0, false);
}

void setMotorDirection(uint8_t index, bool forward) {
  if (index >= MOTOR_COUNT) return;

  if (MOTOR_INVERT[index]) {
    forward = !forward;
  }

  pcaDigital(MOTOR_DIR_A[index], forward);
  pcaDigital(MOTOR_DIR_B[index], !forward);
}

void setMotor(uint8_t index, int speed) {
  if (index >= MOTOR_COUNT) return;

  speed = clampInt(speed, -255, 255);

  if (speed == 0) {
    pcaDigital(MOTOR_DIR_A[index], false);
    pcaDigital(MOTOR_DIR_B[index], false);
    ledcWrite(MOTOR_PWM[index], 0);
    return;
  }

  setMotorDirection(index, speed > 0);
  ledcWrite(MOTOR_PWM[index], abs(speed));
}

void setTank(int left, int right) {
  setMotor(0, left);
  setMotor(2, left);
  setMotor(1, right);
  setMotor(3, right);
}

void stopMotors() {
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    setMotor(i, 0);
  }
}

uint16_t readQTR(uint8_t pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delayMicroseconds(QTR_CHARGE_US);
  pinMode(pin, INPUT);

  uint32_t value = pulseIn(pin, LOW, QTR_TIMEOUT_US);

  if (value == 0) {
    value = QTR_TIMEOUT_US;
  }

  return value > 65535 ? 65535 : (uint16_t)value;
}

void resetLineCalibration() {
  for (uint8_t i = 0; i < LINE_COUNT; i++) {
    lineCalibration.minValue[i] = 0;
    lineCalibration.maxValue[i] = QTR_TIMEOUT_US;
  }

  lineCalibration.valid = false;
}

void saveLineCalibration() {
  preferences.begin("line", false);

  preferences.putBool("valid", lineCalibration.valid);

  for (uint8_t i = 0; i < LINE_COUNT; i++) {
    char a[5];
    char b[5];

    snprintf(a, sizeof(a), "n%d", i);
    snprintf(b, sizeof(b), "x%d", i);

    preferences.putUShort(a, lineCalibration.minValue[i]);
    preferences.putUShort(b, lineCalibration.maxValue[i]);
  }

  preferences.end();
}

void loadLineCalibration() {
  resetLineCalibration();

  preferences.begin("line", true);

  lineCalibration.valid = preferences.getBool("valid", false);

  if (lineCalibration.valid) {
    for (uint8_t i = 0; i < LINE_COUNT; i++) {
      char a[5];
      char b[5];

      snprintf(a, sizeof(a), "n%d", i);
      snprintf(b, sizeof(b), "x%d", i);

      lineCalibration.minValue[i] =
        preferences.getUShort(a, 0);

      lineCalibration.maxValue[i] =
        preferences.getUShort(b, QTR_TIMEOUT_US);

      if (lineCalibration.maxValue[i] <=
          lineCalibration.minValue[i] + 50) {
        lineCalibration.valid = false;
      }
    }
  }

  preferences.end();
}

void calibrateLineSensors() {
  stopMotors();

  uint16_t lo[LINE_COUNT];
  uint16_t hi[LINE_COUNT];

  for (uint8_t i = 0; i < LINE_COUNT; i++) {
    lo[i] = QTR_TIMEOUT_US;
    hi[i] = 0;
  }

  uint32_t start = millis();

  while (millis() - start < 5000) {
    for (uint8_t i = 0; i < LINE_COUNT; i++) {
      uint16_t v = readQTR(LINE_PINS[i]);

      if (v < lo[i]) lo[i] = v;
      if (v > hi[i]) hi[i] = v;
    }
  }

  bool good = true;

  for (uint8_t i = 0; i < LINE_COUNT; i++) {
    lineCalibration.minValue[i] = lo[i];
    lineCalibration.maxValue[i] = hi[i];

    if (hi[i] <= lo[i] + 50) {
      good = false;
    }
  }

  lineCalibration.valid = good;

  if (good) {
    saveLineCalibration();
  }
}

uint16_t normalizeLine(uint16_t raw, uint8_t index) {
  uint16_t lo = lineCalibration.minValue[index];
  uint16_t hi = lineCalibration.maxValue[index];

  if (hi <= lo + 5) {
    return 0;
  }

  raw = clampInt(raw, lo, hi);

  float value;

  if (WHITE_IS_LOW_TIME) {
    value = (float)(hi - raw) / (float)(hi - lo);
  } else {
    value = (float)(raw - lo) / (float)(hi - lo);
  }

  return (uint16_t)(
    clampFloat(value, 0.0f, 1.0f) * 1000.0f
  );
}

void resetLineController() {
  memset(&lineState, 0, sizeof(lineState));
  memset(errorHistory, 0, sizeof(errorHistory));

  errorHead = 0;
  errorCount = 0;
  lastLineDirection = 1.0f;
}

void readLineSensors() {
  static const int16_t weights[8] = {
    -3500, -2500, -1500, -500,
     500, 1500, 2500, 3500
  };

  int32_t weighted = 0;
  uint32_t total = 0;
  uint8_t mask = 0;

  for (uint8_t i = 0; i < LINE_COUNT; i++) {
    lineState.raw[i] = readQTR(LINE_PINS[i]);

    lineState.strength[i] =
      normalizeLine(lineState.raw[i], i);

    if (lineState.strength[i] >= LINE_ON) {
      mask |= (uint8_t)(1u << i);
    }

    uint8_t logical =
      SENSOR_REVERSED ? 7 - i : i;

    weighted +=
      (int32_t)lineState.strength[i] *
      weights[logical];

    total += lineState.strength[i];
  }

  lineState.mask = mask;

  if (total >= LINE_LOST) {
    lineState.position =
      (int16_t)(weighted / (int32_t)total);

    lineState.error =
      clampFloat(
        (float)lineState.position / 3500.0f,
        -1.0f,
        1.0f
      );

    lineState.detected = true;

    if (absf(lineState.error) > 0.03f) {
      lastLineDirection =
        lineState.error > 0 ? 1.0f : -1.0f;
    }
  } else {
    lineState.position = 0;
    lineState.error = 0;
    lineState.detected = false;
  }
}

void updateLineDerivative() {
  uint32_t now = micros();

  errorHistory[errorHead] = {
    lineState.error,
    now
  };

  errorHead =
    (errorHead + 1) % ERR_HISTORY;

  if (errorCount < ERR_HISTORY) {
    errorCount++;
  }

  uint8_t oldest =
    (errorHead + ERR_HISTORY - errorCount)
    % ERR_HISTORY;

  uint32_t dt =
    now - errorHistory[oldest].timeUs;

  if (dt < 1000) {
    dt = 1000;
  }

  lineState.derivative =
    (lineState.error - errorHistory[oldest].value) /
    ((float)dt / 1000000.0f);
}

uint8_t lineSpeed(float severity) {
  static const float x[] = {
    0.00f, 0.10f, 0.20f, 0.30f,
    0.40f, 0.50f, 0.60f, 0.70f,
    0.80f, 0.90f, 1.00f
  };

  static const uint8_t y[] = {
    245, 242, 238, 232,
    222, 210, 195, 175,
    150, 125, 105
  };

  severity =
    clampFloat(absf(severity), 0.0f, 1.0f);

  for (uint8_t i = 1; i < 11; i++) {
    if (severity <= x[i]) {
      float t =
        (severity - x[i - 1]) /
        (x[i] - x[i - 1]);

      return (uint8_t)(
        y[i - 1] +
        t * (y[i] - y[i - 1])
      );
    }
  }

  return y[10];
}

void lineFollowStep() {
  readLineSensors();
  updateLineDerivative();

  if (!lineState.detected) {
    int search =
      lastLineDirection > 0 ? 105 : -105;

    setTank(-search, search);
    return;
  }

  float correction =
      LINE_KP * (lineState.error * 100.0f)
    + LINE_KD * (lineState.derivative * 100.0f);

  correction =
    clampFloat(
      correction,
      -LINE_MAX_CORRECTION,
      LINE_MAX_CORRECTION
    );

  float derivativeSeverity =
    clampFloat(
      absf(lineState.derivative) * 0.06f,
      0.0f,
      1.0f
    );

  float severity =
    absf(lineState.error) > derivativeSeverity
      ? absf(lineState.error)
      : derivativeSeverity;

  int base = lineSpeed(severity);

  int left =
    clampInt(
      (int)(base + correction),
      -255,
      255
    );

  int right =
    clampInt(
      (int)(base - correction),
      -255,
      255
    );

  setTank(left, right);
}

void setToFXshut(uint8_t index, bool high) {
  if (index < TOF_COUNT) {
    pcaDigital(
      TOF_XSHUT_CH[index],
      high
    );
  }
}

bool initToFs() {
  for (uint8_t i = 0; i < TOF_COUNT; i++) {
    setToFXshut(i, false);
  }

  delay(30);

  for (uint8_t i = 0; i < TOF_COUNT; i++) {
    setToFXshut(i, true);
    delay(30);

    if (!tof[i].init()) {
      return false;
    }

    tof[i].setAddress(
      TOF_ADDRESS[i]
    );

    tof[i].setTimeout(50);

    tof[i].startContinuous(
      TOF_PERIOD_MS
    );
  }

  return true;
}

void updateToFs() {
  for (uint8_t i = 0; i < TOF_COUNT; i++) {
    uint16_t d =
      tof[i].readRangeContinuousMillimeters();

    if (
      tof[i].timeoutOccurred() ||
      d == 0 ||
      d == 65535
    ) {
      tofState.valid[i] = false;
      continue;
    }

    tofState.valid[i] = true;
    tofState.mm[i] = d;

    if (!tofState.wall[i] &&
        d <= TOF_WALL_ON_MM) {
      tofState.wall[i] = true;
    } else if (
      tofState.wall[i] &&
      d >= TOF_WALL_OFF_MM
    ) {
      tofState.wall[i] = false;
    }
  }
}

void serviceToFs() {
  uint32_t now = millis();

  if (now - lastTofUpdate >= TOF_PERIOD_MS) {
    lastTofUpdate = now;
    updateToFs();
  }
}

bool hasWall(int x, int y, uint8_t wall) {
  return !inMaze(x, y) ||
         (maze[y][x].blocked & wall);
}

void resetMaze() {
  memset(maze, 0, sizeof(maze));

  for (uint8_t x = 0; x < MAZE_W; x++) {
    maze[0][x].known |= WALL_S;
    maze[0][x].blocked |= WALL_S;

    maze[MAZE_H - 1][x].known |= WALL_N;
    maze[MAZE_H - 1][x].blocked |= WALL_N;
  }

  for (uint8_t y = 0; y < MAZE_H; y++) {
    maze[y][0].known |= WALL_W;
    maze[y][0].blocked |= WALL_W;

    maze[y][MAZE_W - 1].known |= WALL_E;
    maze[y][MAZE_W - 1].blocked |= WALL_E;
  }

  mazeX = 0;
  mazeY = 0;
  mazeHeading = 0;

  maze[0][0].flags |= CELL_VISITED;
}

void observeWall(
  int x,
  int y,
  uint8_t wall,
  bool blocked
) {
  if (!inMaze(x, y)) {
    return;
  }

  int nx = x;
  int ny = y;

  if (wall == WALL_N) {
    ny++;
  } else if (wall == WALL_E) {
    nx++;
  } else if (wall == WALL_S) {
    ny--;
  } else {
    nx--;
  }

  maze[y][x].known |= wall;

  if (blocked) {
    maze[y][x].blocked |= wall;
  } else {
    maze[y][x].blocked &=
      (uint8_t)~wall;
  }

  if (!inMaze(nx, ny)) {
    return;
  }

  uint8_t opposite =
    oppositeWall(wall);

  maze[ny][nx].known |= opposite;

  if (blocked) {
    maze[ny][nx].blocked |= opposite;
  } else {
    maze[ny][nx].blocked &=
      (uint8_t)~opposite;
  }
}

void updateMazeFromToF() {
  uint8_t left =
    (mazeHeading + 3) & 3;

  uint8_t front =
    mazeHeading;

  uint8_t right =
    (mazeHeading + 1) & 3;

  if (tofState.valid[0]) {
    observeWall(
      mazeX,
      mazeY,
      WALL4[left],
      tofState.wall[0]
    );
  }

  if (tofState.valid[1]) {
    observeWall(
      mazeX,
      mazeY,
      WALL4[front],
      tofState.wall[1]
    );
  }

  if (tofState.valid[2]) {
    observeWall(
      mazeX,
      mazeY,
      WALL4[right],
      tofState.wall[2]
    );
  }
}

void recomputeFlood() {
  for (uint8_t y = 0; y < MAZE_H; y++) {
    for (uint8_t x = 0; x < MAZE_W; x++) {
      flood[y][x] = INF8;
    }
  }

  uint16_t head = 0;
  uint16_t tail = 0;

  for (
    int y = goal.ymin;
    y <= goal.ymax;
    y++
  ) {
    for (
      int x = goal.xmin;
      x <= goal.xmax;
      x++
    ) {
      if (!inMaze(x, y)) {
        continue;
      }

      flood[y][x] = 0;

      floodQueue[tail++] =
        (uint16_t)(y * MAZE_W + x);
    }
  }

  while (head < tail) {
    uint16_t id =
      floodQueue[head++];

    int x = id % MAZE_W;
    int y = id / MAZE_W;

    uint8_t next =
      (uint8_t)(flood[y][x] + 1);

    for (uint8_t d = 0; d < 4; d++) {
      if (hasWall(x, y, WALL4[d])) {
        continue;
      }

      int nx = x + DX4[d];
      int ny = y + DY4[d];

      if (!inMaze(nx, ny)) {
        continue;
      }

      if (flood[ny][nx] != INF8) {
        continue;
      }

      flood[ny][nx] = next;

      floodQueue[tail++] =
        (uint16_t)(ny * MAZE_W + nx);
    }
  }
}

uint8_t turnDistance(
  uint8_t a,
  uint8_t b
) {
  uint8_t d =
    (b + 4 - a) & 3;

  return d > 2 ? 4 - d : d;
}

bool chooseFlood(uint8_t &best) {
  recomputeFlood();

  uint8_t bestValue = INF8;
  uint8_t bestTurn = 255;
  bool bestUnvisited = false;
  bool found = false;

  for (uint8_t d = 0; d < 4; d++) {
    if (hasWall(
      mazeX,
      mazeY,
      WALL4[d]
    )) {
      continue;
    }

    int nx =
      mazeX + DX4[d];

    int ny =
      mazeY + DY4[d];

    if (!inMaze(nx, ny)) {
      continue;
    }

    uint8_t v =
      flood[ny][nx];

    bool unvisited =
      !(maze[ny][nx].flags &
         CELL_VISITED);

    uint8_t turns =
      turnDistance(
        mazeHeading,
        d
      );

    if (
      !found ||
      v < bestValue ||
      (
        v == bestValue &&
        unvisited &&
        !bestUnvisited
      ) ||
      (
        v == bestValue &&
        unvisited == bestUnvisited &&
        turns < bestTurn
      )
    ) {
      found = true;
      best = d;
      bestValue = v;
      bestUnvisited = unvisited;
      bestTurn = turns;
    }
  }

  return found;
}

void timedTurnTo(uint8_t target) {
  uint8_t delta =
    (target + 4 - mazeHeading) & 3;

  if (delta == 0) {
    return;
  }

  bool right =
    delta == 1;

  uint8_t quarters =
    delta == 2 ? 2 : 1;

  setTank(
    right ? MAZE_TURN_PWM : -MAZE_TURN_PWM,
    right ? -MAZE_TURN_PWM : MAZE_TURN_PWM
  );

  delay(
    MAZE_TURN_90_MS * quarters
  );

  stopMotors();

  mazeHeading = target;
}

float wallCorrection() {
  if (
    !tofState.valid[0] ||
    !tofState.valid[2]
  ) {
    return 0.0f;
  }

  if (
    !tofState.wall[0] ||
    !tofState.wall[2]
  ) {
    return 0.0f;
  }

  return WALL_KP *
    (
      (float)tofState.mm[0] -
      (float)tofState.mm[2]
    );
}

void driveMazeCell() {
  uint32_t start = millis();

  while (
    millis() - start <
    MAZE_CELL_MS
  ) {
    serviceToFs();

    float correction =
      wallCorrection();

    int left =
      clampInt(
        (int)(MAZE_PWM - correction),
        0,
        255
      );

    int right =
      clampInt(
        (int)(MAZE_PWM + correction),
        0,
        255
      );

    setTank(left, right);
  }

  stopMotors();

  mazeX += DX4[mazeHeading];
  mazeY += DY4[mazeHeading];

  if (inMaze(mazeX, mazeY)) {
    maze[mazeY][mazeX].flags |=
      CELL_VISITED;
  }
}

void mazeExploreStep() {
  if (inGoal(mazeX, mazeY)) {
    stopMotors();
    return;
  }

  serviceToFs();
  updateMazeFromToF();

  uint8_t next;

  if (!chooseFlood(next)) {
    stopMotors();
    return;
  }

  timedTurnTo(next);
  driveMazeCell();
}

int stateId(int x, int y, int h) {
  return (
    ((y * MAZE_W + x) * HCOUNT) +
    h
  );
}

void decodeState(
  int state,
  int &x,
  int &y,
  int &h
) {
  h = state % HCOUNT;

  int cell =
    state / HCOUNT;

  x = cell % MAZE_W;
  y = cell / MAZE_W;
}

uint16_t heuristic(int x, int y) {
  int dx =
    x < goal.xmin
      ? goal.xmin - x
      : (x > goal.xmax
           ? x - goal.xmax
           : 0);

  int dy =
    y < goal.ymin
      ? goal.ymin - y
      : (y > goal.ymax
           ? y - goal.ymax
           : 0);

  int mn =
    dx < dy ? dx : dy;

  int mx =
    dx > dy ? dx : dy;

  if (USE_CHEBYSHEV) {
    return (uint16_t)(
      mx * CARDINAL_COST
    );
  }

  return (uint16_t)(
    mx * CARDINAL_COST +
    mn * (
      DIAGONAL_COST -
      CARDINAL_COST
    )
  );
}

bool cardinalOpen(
  int x,
  int y,
  int h
) {
  if (!inMaze(x, y)) {
    return false;
  }

  int nx =
    x + DX4[h];

  int ny =
    y + DY4[h];

  return (
    inMaze(nx, ny) &&
    !hasWall(
      x,
      y,
      WALL4[h]
    )
  );
}

bool diagonalOpen(
  int x,
  int y,
  int h
) {
  if (!ENABLE_DIAGONAL_ASTAR) {
    return false;
  }

  int nx =
    x + DX8[h];

  int ny =
    y + DY8[h];

  if (!inMaze(nx, ny)) {
    return false;
  }

  uint8_t a;
  uint8_t b;

  if (h == 1) {
    a = 0;
    b = 1;
  } else if (h == 3) {
    a = 2;
    b = 1;
  } else if (h == 5) {
    a = 2;
    b = 3;
  } else {
    a = 0;
    b = 3;
  }

  return (
    cardinalOpen(x, y, a) &&
    cardinalOpen(x, y, b)
  );
}

bool astarLess(
  uint16_t a,
  uint16_t b
) {
  return (
    astar[a].f < astar[b].f ||
    (
      astar[a].f == astar[b].f &&
      astar[a].g > astar[b].g
    )
  );
}

void heapSwap(
  uint16_t a,
  uint16_t b
) {
  uint16_t t =
    astarHeap[a];

  astarHeap[a] =
    astarHeap[b];

  astarHeap[b] =
    t;

  astar[
    astarHeap[a]
  ].heapPos = a;

  astar[
    astarHeap[b]
  ].heapPos = b;
}

void heapUp(uint16_t p) {
  while (p) {
    uint16_t q =
      (p - 1) / 2;

    if (
      !astarLess(
        astarHeap[p],
        astarHeap[q]
      )
    ) {
      break;
    }

    heapSwap(p, q);
    p = q;
  }
}

void heapDown(uint16_t p) {
  while (true) {
    uint16_t left =
      p * 2 + 1;

    uint16_t right =
      left + 1;

    uint16_t best =
      p;

    if (
      left < astarHeapSize &&
      astarLess(
        astarHeap[left],
        astarHeap[best]
      )
    ) {
      best = left;
    }

    if (
      right < astarHeapSize &&
      astarLess(
        astarHeap[right],
        astarHeap[best]
      )
    ) {
      best = right;
    }

    if (best == p) {
      break;
    }

    heapSwap(p, best);
    p = best;
  }
}

void heapPush(uint16_t s) {
  if (astar[s].heapPos < 0) {
    if (astarHeapSize >= STATE_COUNT) {
      return;
    }

    uint16_t p =
      astarHeapSize++;

    astarHeap[p] = s;
    astar[s].heapPos = p;
  }

  heapUp(
    astar[s].heapPos
  );
}

int heapPop() {
  if (!astarHeapSize) {
    return -1;
  }

  uint16_t root =
    astarHeap[0];

  --astarHeapSize;

  if (astarHeapSize) {
    astarHeap[0] =
      astarHeap[astarHeapSize];

    astar[
      astarHeap[0]
    ].heapPos = 0;

    heapDown(0);
  }

  astar[root].heapPos = -1;

  return root;
}

uint16_t moveCost(
  int oldH,
  int newH
) {
  int d =
    abs(newH - oldH);

  if (d > 4) {
    d = 8 - d;
  }

  uint16_t movement =
    (newH & 1)
      ? DIAGONAL_COST
      : CARDINAL_COST;

  return (
    uint16_t)(
      movement +
      TURN45_COST * d
    );
}

bool buildAStar() {
  for (uint16_t i = 0;
       i < STATE_COUNT;
       i++) {
    astar[i].g = 65535;
    astar[i].f = 65535;
    astar[i].parent = -1;
    astar[i].heapPos = -1;
    astar[i].closed = false;
  }

  astarHeapSize = 0;
  solutionCount = 0;
  motionPlanCount = 0;

  int startH =
    mazeHeading * 2;

  int start =
    stateId(
      mazeX,
      mazeY,
      startH
    );

  astar[start].g = 0;

  astar[start].f =
    heuristic(
      mazeX,
      mazeY
    );

  heapPush(start);

  int goalState = -1;

  while (astarHeapSize) {
    int current =
      heapPop();

    if (current < 0) {
      break;
    }

    if (astar[current].closed) {
      continue;
    }

    astar[current].closed = true;

    int x;
    int y;
    int h;

    decodeState(
      current,
      x,
      y,
      h
    );

    if (inGoal(x, y)) {
      goalState = current;
      break;
    }

    for (int nh = 0;
         nh < 8;
         nh++) {
      bool open =
        (nh & 1)
          ? diagonalOpen(x, y, nh)
          : cardinalOpen(x, y, nh);

      if (!open) {
        continue;
      }

      int nx =
        x + DX8[nh];

      int ny =
        y + DY8[nh];

      int next =
        stateId(
          nx,
          ny,
          nh
        );

      if (astar[next].closed) {
        continue;
      }

      uint32_t ng =
        (uint32_t)astar[current].g +
        moveCost(h, nh);

      if (ng >= 65535) {
        continue;
      }

      if (ng < astar[next].g) {
        astar[next].g =
          (uint16_t)ng;

        uint32_t nf =
          ng + heuristic(nx, ny);

        astar[next].f =
          (uint16_t)(
            nf > 65535
              ? 65535
              : nf
          );

        astar[next].parent =
          (int16_t)current;

        heapPush(
          (uint16_t)next
        );
      }
    }
  }

  if (goalState < 0) {
    return false;
  }

  int s = goalState;

  while (
    s >= 0 &&
    solutionCount < STATE_COUNT
  ) {
    solutionStates[
      solutionCount++
    ] = (int16_t)s;

    s =
      astar[s].parent;
  }

  for (
    uint16_t i = 0;
    i < solutionCount / 2;
    i++
  ) {
    int16_t t =
      solutionStates[i];

    solutionStates[i] =
      solutionStates[
        solutionCount - 1 - i
      ];

    solutionStates[
      solutionCount - 1 - i
    ] = t;
  }

  for (
    uint16_t i = 1;
    i < solutionCount;
    i++
  ) {
    int ax;
    int ay;
    int ah;

    int bx;
    int by;
    int bh;

    decodeState(
      solutionStates[i - 1],
      ax,
      ay,
      ah
    );

    decodeState(
      solutionStates[i],
      bx,
      by,
      bh
    );

    int turn =
      bh - ah;

    while (turn > 4) {
      turn -= 8;
    }

    while (turn < -4) {
      turn += 8;
    }

    if (
      turn &&
      motionPlanCount < MAX_PLAN
    ) {
      motionPlan[
        motionPlanCount++
      ] = {
        2,
        0,
        (int8_t)turn
      };
    }

    uint8_t type =
      (bh & 1)
        ? 1
        : 0;

    if (
      motionPlanCount &&
      motionPlan[
        motionPlanCount - 1
      ].type == type
    ) {
      motionPlan[
        motionPlanCount - 1
      ].length++;
    } else if (
      motionPlanCount < MAX_PLAN
    ) {
      motionPlan[
        motionPlanCount++
      ] = {
        type,
        1,
        0
      };
    }
  }

  return true;
}

void executeAStarPlan() {
  uint8_t runHeading =
    mazeHeading * 2;

  for (
    uint16_t i = 0;
    i < motionPlanCount;
    i++
  ) {
    MotionSegment &s =
      motionPlan[i];

    if (s.type == 2) {
      int steps =
        abs(s.turn45);

      bool right =
        s.turn45 > 0;

      setTank(
        right
          ? FAST_PWM
          : -FAST_PWM,

        right
          ? -FAST_PWM
          : FAST_PWM
      );

      delay(
        FAST_TURN_45_MS *
        steps
      );

      stopMotors();

      runHeading =
        (uint8_t)(
          (runHeading +
           s.turn45 +
           8) % 8
        );
    } else if (s.type == 0) {
      for (
        uint16_t n = 0;
        n < s.length;
        n++
      ) {
        driveMazeCell();
      }
    } else {
      setTank(
        FAST_PWM,
        FAST_PWM
      );

      delay(
        FAST_DIAGONAL_CELL_MS *
        s.length
      );

      stopMotors();
    }
  }

  stopMotors();
}

void printPlan() {
  for (
    uint16_t i = 0;
    i < motionPlanCount;
    i++
  ) {
    if (motionPlan[i].type == 2) {
      Serial.printf(
        "TURN %d\n",
        motionPlan[i].turn45 * 45
      );
    } else if (
      motionPlan[i].type == 1
    ) {
      Serial.printf(
        "DIAGONAL %u\n",
        motionPlan[i].length
      );
    } else {
      Serial.printf(
        "STRAIGHT %u\n",
        motionPlan[i].length
      );
    }
  }
}

void printMaze() {
  recomputeFlood();

  for (
    int y = MAZE_H - 1;
    y >= 0;
    y--
  ) {
    for (
      uint8_t x = 0;
      x < MAZE_W;
      x++
    ) {
      Serial.print(
        hasWall(x, y, WALL_N)
          ? "+---"
          : "+   "
      );
    }

    Serial.println("+");

    for (
      uint8_t x = 0;
      x < MAZE_W;
      x++
    ) {
      Serial.print(
        hasWall(x, y, WALL_W)
          ? '|'
          : ' '
      );

      if (
        x == mazeX &&
        y == mazeY
      ) {
        Serial.print(" R ");
      } else if (
        inGoal(x, y)
      ) {
        Serial.print(" G ");
      } else if (
        flood[y][x] < INF8
      ) {
        Serial.printf(
          "%3u",
          flood[y][x]
        );
      } else {
        Serial.print(" ? ");
      }
    }

    Serial.println(
      hasWall(
        MAZE_W - 1,
        y,
        WALL_E
      )
        ? '|'
        : ' '
    );
  }

  for (
    uint8_t x = 0;
    x < MAZE_W;
    x++
  ) {
    Serial.print("+---");
  }

  Serial.println("+");
}

enum Mode {
  STOPPED,
  LINE,
  MAZE
};

Mode mode = STOPPED;

void setup() {
  Serial.begin(115200);

  delay(300);

  Wire.begin(
    SDA_PIN,
    SCL_PIN
  );

  Wire.setClock(400000);

  if (!pca.begin()) {
    while (true) {
      delay(1000);
    }
  }

  pca.setPWMFreq(50);

  pcaDigital(
    STBY_CH,
    true
  );

  for (
    uint8_t i = 0;
    i < MOTOR_COUNT;
    i++
  ) {
    ledcAttach(
      MOTOR_PWM[i],
      20000,
      8
    );

    ledcWrite(
      MOTOR_PWM[i],
      0
    );
  }

  for (
    uint8_t i = 0;
    i < LINE_COUNT;
    i++
  ) {
    pinMode(
      LINE_PINS[i],
      INPUT
    );
  }

  loadLineCalibration();
  resetLineController();
  resetMaze();
  initToFs();
  stopMotors();
}

void processCommand(char c) {
  if (c == 'l' || c == 'L') {
    stopMotors();
    resetLineController();
    mode = LINE;
  } else if (c == 'm' || c == 'M') {
    stopMotors();
    mode = MAZE;
  } else if (c == 's' || c == 'S') {
    stopMotors();
    mode = STOPPED;
  } else if (c == 'c' || c == 'C') {
    mode = STOPPED;
    calibrateLineSensors();
    resetLineController();
  } else if (c == 'r' || c == 'R') {
    stopMotors();
    resetMaze();
  } else if (c == 'p' || c == 'P') {
    stopMotors();
    printMaze();
  } else if (c == 'a' || c == 'A') {
    stopMotors();

    if (buildAStar()) {
      printPlan();
    }
  } else if (c == 'f' || c == 'F') {
    stopMotors();

    if (buildAStar()) {
      executeAStarPlan();
    }

    mode = STOPPED;
  }
}

void loop() {
  while (Serial.available()) {
    processCommand(
      (char)Serial.read()
    );
  }

  if (mode == LINE) {
    lineFollowStep();
  } else if (mode == MAZE) {
    mazeExploreStep();
  } else {
    stopMotors();
    delay(1);
  }
}
