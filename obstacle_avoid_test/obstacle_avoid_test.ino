// ─────────────────────────────────────────
// Task — Obstacle Avoidance on the Line Grid
//
// The robot drives a straight line in the line-zone half of the arena
// toward a target node 6 cells in front of it. A single obstacle blocks
// one node between start and target (placed 2 nodes ahead of start).
// On detecting the obstacle with the forward ultrasonic, the robot:
//   1. stops, marks the forward cell BLOCKED in its 9x9 tagMap
//   2. re-runs A* from current robotPos to target
//   3. turns in place toward the new step direction
//   4. resumes line-following (the perpendicular branch picks up the PID)
//   5. at each subsequent RFID tag, calls A* again to pick the next turn
//   6. naturally swerves around the obstacle and returns to the original
//      heading once clear, then drives the remaining nodes to target
//
// Default geometry (configurable below):
//   START_ROW = 6, START_COL = 0, START_FACING = EAST   (line zone, row 7 1-indexed)
//   TARGET    = (6, 6)                                  (6 nodes east of start)
//   OBSTACLE  = (6, 2)                                  (2 nodes east of start)
// Place the physical obstacle at that node before powering on.
//
// Mirrors main/navigation.ino's NAV_AVOID_OBSTACLE behaviour:
//   - obstacle trigger threshold = OBSTACLE_AVOID_CM (20 cm)
//   - blocked cell = robotPos + one step in robotFacing
//   - in-place turnDegrees, then hand back to the line follower
//
// Same gating model as grid_nav_test:
//   - Power button (pin 17, INPUT_PULLUP): toggles isEnabled
//   - WiFi heartbeat (type=heartbeat enable=1|0) with 1 s timeout
//   - LED on 48/49: solid red enabled, blinking red disabled
//   - Shares IR calibration via kvstore with main/.
// ─────────────────────────────────────────

#include <kvstore_global_api.h>
#include <Wire.h>
#include <Motoron.h>
#include <LSM6.h>
#include <MiniMessenger.h>
#include "MFRC522_I2C.h"
#include "secrets.h"
#include <limits.h>

// ─────────────────────────────────────────
// Grid task config — edit these to relocate the obstacle / target.
// 0-based row/col on the 9x9 RFID grid. Line zone = rows 4..8.
// ─────────────────────────────────────────
enum Facing { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

const int    START_ROW       = 6;
const int    START_COL       = 0;
const Facing START_FACING    = EAST;
const int    TARGET_ROW      = 6;
const int    TARGET_COL      = 6;
const int    OBSTACLE_ROW    = 6;       // single-cell obstacle, must be on the planned path
const int    OBSTACLE_COL    = 2;

// Pre-seed the obstacle into the map at boot so the very first A* already
// routes around it? Leave false for a faithful run-time discovery test —
// the robot only learns about the obstacle when the ultrasonic trips.
const bool   PRESEED_OBSTACLE = false;

// ─────────────────────────────────────────
// IR Sensor
// ─────────────────────────────────────────
const int sensorPins[9]    = {30, 31, 32, 33, 34, 35, 36, 37, 38};
const int ctrlPin          = 12;
const int SensorCount      = 9;
const unsigned int timeout = 2500;

uint16_t minValues[9];
uint16_t maxValues[9];
uint16_t lastPosition = 0;

// ─────────────────────────────────────────
// Motors
// ─────────────────────────────────────────
MotoronI2C motoron(16, &Wire1);
const uint8_t LEFT_MOTOR  = 1;
const uint8_t RIGHT_MOTOR = 2;

const int   BASE_SPEED       = 300;
const float KP               = 0.20f;
const int   LINE_SUM_MIN     = 200;
const int   LINE_CENTER      = 4000;

// ─────────────────────────────────────────
// Encoders (quadrature, one per rear wheel, ISR-driven)
// ─────────────────────────────────────────
const int   ENC_BL_A     = 22;
const int   ENC_BL_B     = 23;
const int   ENC_BR_A     = 14;
const int   ENC_BR_B     = 15;
const float TICKS_PER_CM = 159.97f;

volatile long encBL = 0;
volatile long encBR = 0;
volatile bool lastBL_A = false, lastBL_B = false;
volatile bool lastBR_A = false, lastBR_B = false;

// ─────────────────────────────────────────
// IMU
// ─────────────────────────────────────────
LSM6 imu;
const float GYRO_SENS = 0.00875f;
float gyroZOffset = 0;

// ---- Turn speeds (asymmetric, per-direction — mirrors main/config.h) ----
// The backward-going wheel slips more under weight transfer, so we drive it
// slower than the forward-going wheel. Right- and left-turn speeds are split
// so each direction can be tuned independently.
//   Right turn (direction>0): LEFT wheel forward, RIGHT wheel backward
//   Left  turn (direction<0): LEFT wheel backward, RIGHT wheel forward
const int RIGHT_TURN_FORWARD_SPEED  = 600;
const int RIGHT_TURN_BACKWARD_SPEED = 450;
const int LEFT_TURN_FORWARD_SPEED   = 600;
const int LEFT_TURN_BACKWARD_SPEED  = 450;

// ─────────────────────────────────────────
// Ultrasonic — forward sensor only. Encoder hop-distance is NOT used in
// obstacle decisions (encoders may be miscalibrated). Three thresholds:
//
//   CRASH_STOP_CM   (4 cm)  — absolute safety. Halt motors right now. Back
//                             up a short distance, mark the forward cell
//                             BLOCKED, replan, turn. We're about to hit it.
//   OBSTACLE_STOP_CM (8 cm) — too close to keep going. Mark the forward
//                             cell BLOCKED, replan, turn. No back-up.
//   OBSTACLE_AVOID_CM (20 cm) — soft trigger, only acted on AT an RFID tag
//                               (inside handleRfidNode). At a tag the robot
//                               is on a known grid point, so anything within
//                               20 cm has to be in the next cell. Mid-hop
//                               the same reading is ignored — the robot
//                               keeps going to the next tag and decides
//                               from there.
//
// This sidesteps the "which cell is the obstacle in?" question without
// trusting encoders: at-tag means "obstacle is in next cell", mid-hop CRASH
// or STOP both mean "stop, that obstacle is right here, mark it".
// ─────────────────────────────────────────
const int FORWARD_TRIG_PIN     = 44;
const int FORWARD_ECHO_PIN     = 45;
// 5000 µs = ~85 cm max — anything further doesn't matter for obstacle nav,
// and a shorter timeout keeps the loop responsive. HC-SR04's minimum range
// is ~2 cm; below that, readings may be OOR (handled separately).
const unsigned long ULTRASONIC_TIMEOUT = 5000;
const float OBSTACLE_AVOID_CM  = 20.0f;           // at-tag-only trigger (matches main/config.h)
const float OBSTACLE_STOP_CM   =  8.0f;           // mid-hop stop + mark
const float CRASH_STOP_CM      =  4.0f;           // mid-hop emergency + back-up
const float GRID_SPACING_CM    = 25.0f;           // matches main/config.h
const unsigned long FWD_PING_INTERVAL_MS = 20;    // ~50 Hz ping cadence — fast enough to catch CRASH_STOP_CM at our drive speed
const unsigned long FWD_LOG_INTERVAL_MS  = 500;   // unconditional log cadence — verifies the sensor is alive

// OOR persistence: HC-SR04 readings get unreliable below ~2 cm. If the
// previous reading was inside OBSTACLE_AVOID_CM and the sensor goes OOR
// within this window, assume it's still close (and getting closer) — treat
// as crash-stop. Without this, the robot drives THROUGH an obstacle that's
// too close for the sensor to see.
const unsigned long OOR_PERSISTENCE_MS = 400;

// How long to drive in reverse after a crash-stop. ~5 cm worth at BASE_SPEED.
// Tune empirically — too short leaves us still on top of the obstacle for
// the in-place turn, too long pushes us back past the last RFID tag.
const unsigned long BACKUP_MS  = 400;

float         lastForwardDistanceCm = -1.0f;
unsigned long lastFwdPingMs         = 0;
unsigned long lastFwdLogMs          = 0;
unsigned long lastCloseReadingMs    = 0;   // last time reading was < OBSTACLE_AVOID_CM

// ─────────────────────────────────────────
// RFID
// ─────────────────────────────────────────
MFRC522_I2C rfid(0x28, 255);

// ─────────────────────────────────────────
// WiFi / heartbeat / enable state
// ─────────────────────────────────────────
MiniMessenger        messenger;
const char*          BoardId              = "Master";
bool                 isEnabled            = false;
unsigned long        lastHeartbeatMs      = 0;
unsigned long        lastRegisterMs       = 0;
const unsigned long  HEARTBEAT_TIMEOUT_MS = 1000;
const unsigned long  REGISTER_INTERVAL_MS = 10000;

// ─────────────────────────────────────────
// Status LED + power button
// ─────────────────────────────────────────
const int LED_R = 48;
const int LED_G = 49;
const unsigned long LED_BLINK_INTERVAL_MS = 500;
unsigned long lastBlinkMs = 0;
bool          blinkState  = false;
const int POWER_BUTTON = 17;

// ─────────────────────────────────────────
// Task / nav constants
// ─────────────────────────────────────────
const unsigned long NODE_STOP_MS       = 1000;   // halt this long after each scan
const float         START_FORWARD_NUDGE_CM = 12.0f;
// Pre-turn nudge: per-direction forward distance to centre the wheel axis
// (= pivot) over the tag before the in-place rotation. Right-turn gets a
// longer nudge to compensate for an asymmetric pivot offset on this bot.
const float         RIGHT_PRE_TURN_FORWARD_CM = 6.0f;
const float         LEFT_PRE_TURN_FORWARD_CM  = 5.0f;
const unsigned long JUNCTION_NUDGE_MS  = 150;

// ─────────────────────────────────────────
// 9x9 tagMap + robot pose. Mirrors main/navigation.ino enums.
// ─────────────────────────────────────────
enum TagState { TAG_UNKNOWN = 0, TAG_FERTILE, TAG_INFERTILE, TAG_PLANTED, TAG_BLOCKED };

TagState tagMap[9][9];
int      robotRow      = START_ROW;
int      robotCol      = START_COL;
Facing   robotFacing   = START_FACING;
int      pendingTurnDir = 0;   // -1=left, 0=straight, +1=right, +2=u-turn

bool  runComplete   = false;
bool  startCleared  = false;
char  lastScannedUid[32] = "";
int   scanCount     = 0;       // diagnostic only

// Forward decls — blocking helpers poll the enable gate.
void wifiLoop();
void checkPowerButton();
void pingForwardUltrasonic();

static inline void stopMotors() {
  motoron.setSpeedNow(LEFT_MOTOR,  0);
  motoron.setSpeedNow(RIGHT_MOTOR, 0);
}

// ─────────────────────────────────────────
// IR helpers (same kvstore keys as main/ — calibration is shared)
// ─────────────────────────────────────────
unsigned int readPrivate(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delayMicroseconds(10);
  pinMode(pin, INPUT);
  unsigned long start = micros();
  while (digitalRead(pin) == HIGH) {
    if (micros() - start > timeout) return timeout;
  }
  return micros() - start;
}

void saveCalibration() {
  for (int i = 0; i < SensorCount; i++) {
    char key[20];
    sprintf(key, "/kv/min%d", i);
    kv_set(key, &minValues[i], sizeof(uint16_t), 0);
    sprintf(key, "/kv/max%d", i);
    kv_set(key, &maxValues[i], sizeof(uint16_t), 0);
  }
}

bool loadCalibration() {
  size_t actual_size;
  for (int i = 0; i < SensorCount; i++) {
    char key[20];
    sprintf(key, "/kv/min%d", i);
    if (kv_get(key, &minValues[i], sizeof(uint16_t), &actual_size) != 0) return false;
    sprintf(key, "/kv/max%d", i);
    if (kv_get(key, &maxValues[i], sizeof(uint16_t), &actual_size) != 0) return false;
  }
  return true;
}

void runCalibration() {
  for (int i = 0; i < SensorCount; i++) {
    minValues[i] = timeout;
    maxValues[i] = 0;
  }
  Serial.println("--- CALIBRATION STARTING ---");
  Serial.println("Slide sensors over the black line repeatedly!");
  for (int j = 0; j < 400; j++) {
    for (int i = 0; i < SensorCount; i++) {
      unsigned int val = readPrivate(sensorPins[i]);
      if (val < minValues[i]) minValues[i] = val;
      if (val > maxValues[i]) maxValues[i] = val;
    }
    if (j % 40 == 0) Serial.println("Still calibrating...");
    delay(10);
  }
  saveCalibration();
  Serial.println("--- CALIBRATION COMPLETE ---");
}

// ─────────────────────────────────────────
// IMU
// ─────────────────────────────────────────
void calibrateGyro() {
  Serial.println("Calibrating gyro, keep still...");
  long sum = 0;
  for (int i = 0; i < 500; i++) {
    imu.read();
    sum += imu.g.z;
    delay(2);
  }
  gyroZOffset = sum / 500.0f;
}

// ─────────────────────────────────────────
// Encoders (4× decoding, ISR-driven)
// ─────────────────────────────────────────
void isr_BL_A() { bool a = digitalRead(ENC_BL_A), b = digitalRead(ENC_BL_B); if (a != lastBL_A) { encBL += (a == b) ?  1 : -1; lastBL_A = a; } }
void isr_BL_B() { bool a = digitalRead(ENC_BL_A), b = digitalRead(ENC_BL_B); if (b != lastBL_B) { encBL += (a == b) ? -1 :  1; lastBL_B = b; } }
void isr_BR_A() { bool a = digitalRead(ENC_BR_A), b = digitalRead(ENC_BR_B); if (a != lastBR_A) { encBR += (a == b) ? -1 :  1; lastBR_A = a; } }
void isr_BR_B() { bool a = digitalRead(ENC_BR_A), b = digitalRead(ENC_BR_B); if (b != lastBR_B) { encBR += (a == b) ?  1 : -1; lastBR_B = b; } }

void encoderSetup() {
  pinMode(ENC_BL_A, INPUT_PULLUP); pinMode(ENC_BL_B, INPUT_PULLUP);
  pinMode(ENC_BR_A, INPUT_PULLUP); pinMode(ENC_BR_B, INPUT_PULLUP);
  lastBL_A = digitalRead(ENC_BL_A); lastBL_B = digitalRead(ENC_BL_B);
  lastBR_A = digitalRead(ENC_BR_A); lastBR_B = digitalRead(ENC_BR_B);
  attachInterrupt(digitalPinToInterrupt(ENC_BL_A), isr_BL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_BL_B), isr_BL_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_BR_A), isr_BR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_BR_B), isr_BR_B, CHANGE);
}

long  straightTicks() { return (encBL + encBR) / 2; }

// Hop-distance tracking: ticks since the last RFID tag (or post-turn handoff).
// Separate from forwardCm's internal start-tick so we can measure distance
// since the last tag for the obstacle "is it in the next cell?" check, even
// across multiple forwardCm calls (pre-turn nudges, etc.).
long  hopTickBase = 0;
float hopDistanceCm() { return (float)(straightTicks() - hopTickBase) / TICKS_PER_CM; }
void  resetHop()      { hopTickBase = straightTicks(); }

// Encoder-gated forward drive. Tracks its own start tick locally so the
// hop counter keeps running across nudges (start-clear, pre/post-turn).
// Returns false if disabled mid-drive.
bool forwardCm(float cm) {
  long startTicks = straightTicks();
  motoron.setSpeedNow(LEFT_MOTOR,  BASE_SPEED);
  motoron.setSpeedNow(RIGHT_MOTOR, BASE_SPEED);
  while ((float)(straightTicks() - startTicks) / TICKS_PER_CM < cm) {
    wifiLoop();
    checkPowerButton();
    if (!isEnabled) { stopMotors(); return false; }
    delay(2);
  }
  stopMotors();
  return true;
}

// ─────────────────────────────────────────
// Gyro-tracked in-place turn. Polls enable gate every tick. targetDegrees
// sign: positive = clockwise (right), negative = counter-clockwise (left).
// ─────────────────────────────────────────
bool turnDegrees(float targetDegrees) {
  if (fabsf(targetDegrees) < 1.0f) return true;   // straight: no-op
  motoron.clearMotorFault(LEFT_MOTOR);
  motoron.clearMotorFault(RIGHT_MOTOR);

  float accumulated = 0;
  unsigned long lastTime = micros();
  int direction = (targetDegrees > 0) ? 1 : -1;

  // Per-side asymmetric speed (mirrors main/motors.ino turnDegrees).
  const int leftSpeed  = (direction > 0) ? RIGHT_TURN_FORWARD_SPEED  : LEFT_TURN_BACKWARD_SPEED;
  const int rightSpeed = (direction > 0) ? RIGHT_TURN_BACKWARD_SPEED : LEFT_TURN_FORWARD_SPEED;

  while (true) {
    imu.read();
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0f;
    lastTime = now;
    float gz = -((imu.g.z - gyroZOffset) * GYRO_SENS);
    accumulated += gz * dt;

    if (fabsf(accumulated) >= fabsf(targetDegrees)) break;

    motoron.setSpeedNow(LEFT_MOTOR,   direction * leftSpeed);
    motoron.setSpeedNow(RIGHT_MOTOR, -direction * rightSpeed);

    wifiLoop();
    checkPowerButton();
    if (!isEnabled) { stopMotors(); return false; }
    delay(5);
  }

  stopMotors();
  return true;
}

bool pausedWait(unsigned long ms) {
  stopMotors();
  unsigned long ts = millis();
  while (millis() - ts < ms) {
    wifiLoop();
    checkPowerButton();
    if (!isEnabled) return false;
    delay(5);
  }
  return true;
}

// ─────────────────────────────────────────
// Forward ultrasonic ping — rate-limited so we don't burn the loop on it.
// Updates lastForwardDistanceCm in-place (-1 = out of range / no echo).
// Tracks `lastCloseReadingMs` so a brief OOR after a close reading can be
// treated as "still close" by the caller (HC-SR04 < 2 cm dead-zone).
//
// Also unconditionally logs the reading every FWD_LOG_INTERVAL_MS so you
// can see in the serial monitor whether the sensor is alive at all.
// ─────────────────────────────────────────
void pingForwardUltrasonic() {
  unsigned long now = millis();
  if (now - lastFwdPingMs < FWD_PING_INTERVAL_MS) return;
  lastFwdPingMs = now;

  digitalWrite(FORWARD_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(FORWARD_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(FORWARD_TRIG_PIN, LOW);

  long duration = pulseIn(FORWARD_ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT);
  lastForwardDistanceCm = (duration == 0) ? -1.0f : (duration / 58.0f);

  if (lastForwardDistanceCm >= 0.0f && lastForwardDistanceCm < OBSTACLE_AVOID_CM) {
    lastCloseReadingMs = now;
  }

  if (now - lastFwdLogMs >= FWD_LOG_INTERVAL_MS) {
    lastFwdLogMs = now;
    Serial.print("[FWD] ");
    if (lastForwardDistanceCm < 0.0f) Serial.println("OOR");
    else { Serial.print(lastForwardDistanceCm); Serial.println(" cm"); }
  }
}

// True if (a) the most recent reading is below `cm`, OR (b) the sensor is
// OOR but we saw a reading inside OBSTACLE_AVOID_CM within the last
// OOR_PERSISTENCE_MS — the HC-SR04 dead-zone trick. Use this instead of
// raw `lastForwardDistanceCm < cm` in safety-critical gates.
bool forwardReadingBelow(float cm) {
  if (lastForwardDistanceCm >= 0.0f && lastForwardDistanceCm < cm) return true;
  if (lastForwardDistanceCm < 0.0f &&
      lastCloseReadingMs != 0 &&
      (millis() - lastCloseReadingMs) < OOR_PERSISTENCE_MS) {
    return true;
  }
  return false;
}

// ─────────────────────────────────────────
// RFID one-shot
// ─────────────────────────────────────────
bool readRfidNonBlocking(char* uidOut, size_t uidOutSize) {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) return false;
  uidOut[0] = '\0';
  for (byte i = 0; i < rfid.uid.size; i++) {
    char b[3];
    snprintf(b, sizeof(b), "%02X", rfid.uid.uidByte[i]);
    if (strlen(uidOut) + 2 < uidOutSize) strcat(uidOut, b);
  }
  rfid.PICC_HaltA();
  return true;
}

// ─────────────────────────────────────────
// Grid helpers — facing arithmetic
// ─────────────────────────────────────────
void stepOneCell(int& r, int& c, Facing f) {
  switch (f) {
    case NORTH: r--; break;
    case SOUTH: r++; break;
    case EAST:  c++; break;
    case WEST:  c--; break;
  }
}

Facing facingToward(int fr, int fc, int tr, int tc) {
  if (tr == fr - 1 && tc == fc) return NORTH;
  if (tr == fr + 1 && tc == fc) return SOUTH;
  if (tr == fr && tc == fc + 1) return EAST;
  if (tr == fr && tc == fc - 1) return WEST;
  return robotFacing;   // shouldn't happen for adjacent cells
}

// Returns -1 (left), 0 (straight), +1 (right), +2 (u-turn).
int getTurnDir(Facing cur, Facing want) {
  int diff = ((int)want - (int)cur + 4) % 4;
  if (diff == 0) return 0;
  if (diff == 1) return  1;     // right
  if (diff == 3) return -1;     // left
  return 2;                     // u-turn
}

Facing facingAfterTurn(Facing cur, int turn) {
  return (Facing)(((int)cur + turn + 4) % 4);
}

// ─────────────────────────────────────────
// A* on the 9x9 grid — Manhattan heuristic, uniform unit edges.
// Skips TAG_BLOCKED cells. Returns the first step from `from` via `nextR/C`.
// Lifted from main/navigation.ino with the GridPos struct flattened to
// row/col ints to keep this file self-contained.
// ─────────────────────────────────────────
bool aStarNextStep(int fromR, int fromC, int toR, int toC,
                   int& nextR, int& nextC) {
  if (fromR < 0 || fromR >= 9 || fromC < 0 || fromC >= 9) return false;
  if (toR   < 0 || toR   >= 9 || toC   < 0 || toC   >= 9) return false;
  if (fromR == toR && fromC == toC) return false;

  static int    gCost[81];
  static int    fCost[81];
  static int8_t parent[81];
  static bool   openSet[81];
  static bool   closed[81];

  for (int i = 0; i < 81; i++) {
    gCost[i]   = INT_MAX;
    fCost[i]   = INT_MAX;
    parent[i]  = -1;
    openSet[i] = false;
    closed[i]  = false;
  }

  const int startIdx = fromR * 9 + fromC;
  const int goalIdx  = toR   * 9 + toC;

  gCost[startIdx]   = 0;
  fCost[startIdx]   = abs(fromR - toR) + abs(fromC - toC);
  openSet[startIdx] = true;

  static const int8_t dr[4] = { -1,  1,  0,  0 };
  static const int8_t dc[4] = {  0,  0,  1, -1 };

  while (true) {
    int cur = -1;
    int bestF = INT_MAX;
    for (int i = 0; i < 81; i++) {
      if (openSet[i] && fCost[i] < bestF) { bestF = fCost[i]; cur = i; }
    }
    if (cur == -1) return false;
    if (cur == goalIdx) break;

    openSet[cur] = false;
    closed[cur]  = true;

    const int cr = cur / 9;
    const int cc = cur % 9;

    for (int d = 0; d < 4; d++) {
      const int nr = cr + dr[d];
      const int nc = cc + dc[d];
      if (nr < 0 || nr >= 9 || nc < 0 || nc >= 9) continue;
      if (tagMap[nr][nc] == TAG_BLOCKED) continue;
      const int nIdx = nr * 9 + nc;
      if (closed[nIdx]) continue;

      const int tentativeG = gCost[cur] + 1;
      if (tentativeG < gCost[nIdx]) {
        parent[nIdx]  = (int8_t)cur;
        gCost[nIdx]   = tentativeG;
        fCost[nIdx]   = tentativeG + abs(nr - toR) + abs(nc - toC);
        openSet[nIdx] = true;
      }
    }
  }

  int step = goalIdx;
  while (parent[step] != (int8_t)startIdx) {
    if (parent[step] < 0) return false;
    step = parent[step];
  }
  nextR = step / 9;
  nextC = step % 9;
  return true;
}

// Recompute pendingTurnDir based on current robotPos / robotFacing.
// Returns false if we're already at the target (caller should halt).
bool replanNextDir() {
  if (robotRow == TARGET_ROW && robotCol == TARGET_COL) {
    pendingTurnDir = 0;
    return false;
  }
  int nr, nc;
  if (!aStarNextStep(robotRow, robotCol, TARGET_ROW, TARGET_COL, nr, nc)) {
    Serial.println("[A*] no path to target!");
    pendingTurnDir = 0;
    return false;
  }
  Facing want = facingToward(robotRow, robotCol, nr, nc);
  pendingTurnDir = getTurnDir(robotFacing, want);

  Serial.print("[A*] from (");
  Serial.print(robotRow); Serial.print(","); Serial.print(robotCol);
  Serial.print(") facing "); Serial.print((int)robotFacing);
  Serial.print(" -> step (");
  Serial.print(nr); Serial.print(","); Serial.print(nc);
  Serial.print("), turn "); Serial.println(pendingTurnDir);
  return true;
}

// ─────────────────────────────────────────
// Apply the queued turn after an RFID arrival, mirroring baseTurnBlocking
// in main/navigation.ino:
//   pre-turn 5 cm nudge → in-place rotate → 150 ms post-turn nudge.
// Skips entirely for straight (turn == 0). Resets hop at the end so the
// next hop's distance is measured from the new heading's origin.
// ─────────────────────────────────────────
void executeQueuedTurn() {
  if (pendingTurnDir == 0) return;

  const float preNudgeCm = (pendingTurnDir > 0) ? RIGHT_PRE_TURN_FORWARD_CM
                                                : LEFT_PRE_TURN_FORWARD_CM;
  Serial.print("  pre-turn nudge "); Serial.print(preNudgeCm); Serial.println(" cm");
  if (!forwardCm(preNudgeCm)) return;

  if (!turnDegrees((float)pendingTurnDir * 90.0f)) return;
  robotFacing = facingAfterTurn(robotFacing, pendingTurnDir);

  Serial.print("  post-turn nudge "); Serial.print(JUNCTION_NUDGE_MS); Serial.println(" ms");
  motoron.setSpeedNow(LEFT_MOTOR,  BASE_SPEED);
  motoron.setSpeedNow(RIGHT_MOTOR, BASE_SPEED);
  unsigned long ts = millis();
  while (millis() - ts < JUNCTION_NUDGE_MS) {
    wifiLoop();
    checkPowerButton();
    if (!isEnabled) { stopMotors(); return; }
    delay(5);
  }
  stopMotors();
  pendingTurnDir = 0;
  resetHop();    // new hop begins here, in the new heading
}

// ─────────────────────────────────────────
// Mark the cell directly ahead of robotPos (in robotFacing) as BLOCKED.
// Returns true if anything was marked, false if the cell is off-grid or
// already BLOCKED.
// ─────────────────────────────────────────
bool markForwardCellBlocked(const char* tag) {
  int br = robotRow, bc = robotCol;
  stepOneCell(br, bc, robotFacing);
  if (br < 0 || br >= 9 || bc < 0 || bc >= 9) {
    Serial.print(tag); Serial.println(" forward cell off-grid — skipping mark");
    return false;
  }
  if (tagMap[br][bc] == TAG_BLOCKED) return false;
  tagMap[br][bc] = TAG_BLOCKED;
  Serial.print(tag); Serial.print(" reading ");
  Serial.print(lastForwardDistanceCm); Serial.print(" cm → marked (");
  Serial.print(br); Serial.print(","); Serial.print(bc);
  Serial.println(") BLOCKED");
  return true;
}

// ─────────────────────────────────────────
// Drive backwards for `ms` milliseconds at -BASE_SPEED. Polls the enable
// gate. Returns false if disabled mid-drive.
// ─────────────────────────────────────────
bool reverseFor(unsigned long ms) {
  motoron.setSpeedNow(LEFT_MOTOR,  -BASE_SPEED);
  motoron.setSpeedNow(RIGHT_MOTOR, -BASE_SPEED);
  unsigned long ts = millis();
  while (millis() - ts < ms) {
    wifiLoop();
    checkPowerButton();
    if (!isEnabled) { stopMotors(); return false; }
    delay(5);
  }
  stopMotors();
  return true;
}

// ─────────────────────────────────────────
// Per-tag arrival: advance robotPos, hold NODE_STOP_MS, peek forward for
// an obstacle in the next cell, then replan via A* and execute the queued
// turn. If we just stepped onto the target, halt.
// ─────────────────────────────────────────
void handleRfidNode(const char* uid) {
  stopMotors();
  scanCount++;
  // The tag we just read IS the cell we advanced into.
  stepOneCell(robotRow, robotCol, robotFacing);
  resetHop();    // hop=0 at the moment we landed on this tag

  Serial.print("Tag scan #"); Serial.print(scanCount);
  Serial.print(" uid="); Serial.print(uid);
  Serial.print("  -> robotPos (");
  Serial.print(robotRow); Serial.print(","); Serial.print(robotCol);
  Serial.println(")");

  if (!pausedWait(NODE_STOP_MS)) return;

  // At target? Halt the run.
  if (robotRow == TARGET_ROW && robotCol == TARGET_COL) {
    Serial.println("  TARGET reached. Halting.");
    runComplete = true;
    return;
  }

  // Look ahead before replanning. At a tag, robot is on a known grid point,
  // so any reading inside OBSTACLE_AVOID_CM (20 cm) maps to the next cell.
  // No encoder math needed. A* will route around the blocked cell on this
  // same replan — no second pass required.
  pingForwardUltrasonic();
  if (forwardReadingBelow(OBSTACLE_AVOID_CM)) {
    markForwardCellBlocked("[AT-TAG AVOID]");
  }

  if (!replanNextDir()) {
    Serial.println("  no further path — halting.");
    runComplete = true;
    return;
  }

  executeQueuedTurn();
}

// ─────────────────────────────────────────
// Mid-hop obstacle handler (8 cm > reading >= 4 cm). Stop, mark the next
// cell, replan, turn. No back-up — the obstacle isn't on us yet, and the
// in-place turn lines us up with a perpendicular branch for the line
// follower to pick up.
// ─────────────────────────────────────────
void handleObstacleStopMidHop() {
  stopMotors();
  Serial.print("[STOP] mid-hop reading ");
  Serial.print(lastForwardDistanceCm); Serial.println(" cm");
  markForwardCellBlocked("[STOP]");

  if (!replanNextDir()) {
    Serial.println("[STOP] no detour — halting");
    runComplete = true;
    return;
  }
  if (pendingTurnDir == 0) {
    Serial.println("[STOP] replan still says straight — halting defensively");
    runComplete = true;
    return;
  }
  executeQueuedTurn();
}

// ─────────────────────────────────────────
// Crash-stop handler (reading < 4 cm). Halt motors NOW, back up a short
// distance to give the in-place turn some breathing room, mark the next
// cell BLOCKED, replan, and turn. This is the "don't ram into it" gate —
// runs before any other obstacle logic on every loop tick.
// ─────────────────────────────────────────
void handleObstacleCrashStop() {
  stopMotors();
  Serial.print("[CRASH] reading ");
  Serial.print(lastForwardDistanceCm); Serial.println(" cm — emergency stop");

  markForwardCellBlocked("[CRASH]");

  Serial.print("[CRASH] backing up "); Serial.print(BACKUP_MS); Serial.println(" ms");
  if (!reverseFor(BACKUP_MS)) return;

  if (!replanNextDir()) {
    Serial.println("[CRASH] no detour — halting");
    runComplete = true;
    return;
  }
  if (pendingTurnDir == 0) {
    Serial.println("[CRASH] replan still says straight — halting defensively");
    runComplete = true;
    return;
  }
  executeQueuedTurn();
}

// ─────────────────────────────────────────
// LED / button / WiFi gating — mirrors grid_nav_test
// ─────────────────────────────────────────
void setLED(int r, int g) { digitalWrite(LED_R, r); digitalWrite(LED_G, g); }

void updateLED() {
  if (isEnabled) {
    setLED(HIGH, LOW);
  } else {
    if (millis() - lastBlinkMs > LED_BLINK_INTERVAL_MS) {
      lastBlinkMs = millis();
      blinkState = !blinkState;
      setLED(blinkState ? HIGH : LOW, LOW);
    }
  }
}

void checkPowerButton() {
  static unsigned long lastPressMs = 0;
  static bool wasPressed = false;
  static bool firstCall  = true;
  const bool pressedNow = (digitalRead(POWER_BUTTON) == LOW);

  if (firstCall) { wasPressed = pressedNow; firstCall = false; return; }

  if (pressedNow && !wasPressed && (millis() - lastPressMs > 200)) {
    isEnabled = !isEnabled;
    lastPressMs = millis();
    if (lastHeartbeatMs != 0) lastHeartbeatMs = millis();
    Serial.print(">>> Power button — isEnabled now ");
    Serial.println(isEnabled ? "true" : "false");
  }
  wasPressed = pressedNow;
}

void onMessage(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
  char msg[128];
  size_t copyLen = (length < 127) ? length : 127;
  memcpy(msg, payload, copyLen);
  msg[copyLen] = '\0';
  Serial.print("MSG: ");
  Serial.println(msg);

  if (strstr(msg, "type=heartbeat")) {
    lastHeartbeatMs = millis();
    if (strstr(msg, "enable=1")) {
      if (!isEnabled) Serial.println(">>> ENABLED");
      isEnabled = true;
    } else {
      if (isEnabled) Serial.println(">>> DISABLED (heartbeat)");
      isEnabled = false;
    }
  }

  if (strstr(msg, "type=disable")) {
    if (isEnabled) Serial.println(">>> DISABLED (disable msg)");
    isEnabled = false;
  }
}

void sendRegister() {
  char msg[64];
  snprintf(msg, sizeof(msg), "type=register team_id=%s board_id=%s", GROUP_ID, BoardId);
  messenger.sendToBoard("server", msg);
}

void wifiSetup() {
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  setLED(LOW, LOW);
  messenger.onMessage(onMessage);
  messenger.begin(WIFI_SSID, WIFI_PASSWORD, BROKER_HOST, BROKER_PORT, GROUP_ID, BoardId);
  Serial.println("WiFi connecting...");
}

void wifiLoop() {
  messenger.loop();
  updateLED();

  if (lastHeartbeatMs != 0 && isEnabled &&
      millis() - lastHeartbeatMs > HEARTBEAT_TIMEOUT_MS) {
    isEnabled = false;
    Serial.println(">>> DISABLED (heartbeat timeout)");
  }

  if (millis() - lastRegisterMs > REGISTER_INTERVAL_MS || lastRegisterMs == 0) {
    lastRegisterMs = millis();
    sendRegister();
  }
}

// ─────────────────────────────────────────
// Setup
// ─────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  uint32_t startWait = millis();
  while (!Serial && millis() - startWait < 3000);

  Wire.begin();
  Wire1.begin();
  motoron.reinitialize();
  motoron.disableCommandTimeout();
  motoron.clearResetFlag();

  imu.setBus(&Wire1);
  if (!imu.init()) {
    Serial.println("IMU not found!");
    while (1);
  }
  imu.enableDefault();

  pinMode(ctrlPin, OUTPUT);
  digitalWrite(ctrlPin, HIGH);
  pinMode(POWER_BUTTON, INPUT_PULLUP);

  pinMode(FORWARD_TRIG_PIN, OUTPUT);
  pinMode(FORWARD_ECHO_PIN, INPUT);
  digitalWrite(FORWARD_TRIG_PIN, LOW);

  encoderSetup();
  rfid.PCD_Init();
  Serial.println("RFID ready.");
  wifiSetup();

  if (!loadCalibration()) {
    Serial.println("No saved IR calibration — running now...");
    runCalibration();
  } else {
    Serial.println("IR calibration loaded. Send 'c' within 3s to recalibrate.");
    unsigned long s = millis();
    while (millis() - s < 3000) {
      if (Serial.available() && Serial.read() == 'c') { runCalibration(); break; }
    }
  }

  calibrateGyro();

  // Seed the tagMap. All UNKNOWN (A* treats UNKNOWN as traversable since it
  // only blocks on TAG_BLOCKED). Optionally pre-mark the obstacle.
  for (int r = 0; r < 9; r++)
    for (int c = 0; c < 9; c++)
      tagMap[r][c] = TAG_UNKNOWN;
  if (PRESEED_OBSTACLE) {
    tagMap[OBSTACLE_ROW][OBSTACLE_COL] = TAG_BLOCKED;
    Serial.println("Pre-seeded obstacle into tagMap.");
  }

  Serial.println();
  Serial.println("=== Obstacle Avoidance Test ===");
  Serial.print("Start = ("); Serial.print(START_ROW); Serial.print(",");
  Serial.print(START_COL);  Serial.print("), facing "); Serial.println((int)START_FACING);
  Serial.print("Target = ("); Serial.print(TARGET_ROW); Serial.print(",");
  Serial.print(TARGET_COL); Serial.println(")");
  Serial.print("Obstacle = ("); Serial.print(OBSTACLE_ROW); Serial.print(",");
  Serial.print(OBSTACLE_COL); Serial.print(")  trigger = ");
  Serial.print(OBSTACLE_AVOID_CM); Serial.println(" cm");
  Serial.println("Press power button or send heartbeat enable=1 to start.");
}

// ─────────────────────────────────────────
// Loop
// ─────────────────────────────────────────
void loop() {
  wifiLoop();
  checkPowerButton();

  if (!isEnabled) {
    stopMotors();
    delay(10);
    return;
  }

  if (runComplete) {
    stopMotors();
    return;
  }

  // First-enable: clear the start tag before RFID polling kicks in.
  if (!startCleared) {
    Serial.print("Clearing start tag — forward ");
    Serial.print(START_FORWARD_NUDGE_CM, 1); Serial.println(" cm");
    if (!forwardCm(START_FORWARD_NUDGE_CM)) return;
    startCleared = true;
    resetHop();      // first hop counter starts from where the nudge ended
    Serial.println("Line follow + RFID polling + obstacle watch engaged.");
  }

  // Mid-hop obstacle gate. Encoder-free — pure ultrasonic + RFID.
  //   < CRASH_STOP_CM (4 cm) OR OOR-after-close → emergency halt + back up
  //   < OBSTACLE_STOP_CM (8 cm) → stop + mark + turn (no back-up)
  //   < OBSTACLE_AVOID_CM → ignore mid-hop; handled at next RFID tag
  //
  // The OOR-after-close branch matters because HC-SR04 readings drop to
  // OOR below ~2 cm (echo overlaps trigger). Without it, the robot would
  // see "close, close, OOR, OOR" and drive straight through.
  pingForwardUltrasonic();
  if (forwardReadingBelow(CRASH_STOP_CM)) {
    handleObstacleCrashStop();
    return;
  }
  if (forwardReadingBelow(OBSTACLE_STOP_CM)) {
    handleObstacleStopMidHop();
    return;
  }

  // RFID poll. Dedup so a dwelt-over tag fires once.
  {
    char uid[32];
    if (readRfidNonBlocking(uid, sizeof(uid))) {
      if (strcmp(uid, lastScannedUid) != 0) {
        strncpy(lastScannedUid, uid, sizeof(lastScannedUid) - 1);
        lastScannedUid[sizeof(lastScannedUid) - 1] = '\0';
        handleRfidNode(uid);
      }
      return;
    }
  }

  // Plain PID line-following. Junctions in the IR signal are ignored — the
  // wide crossing averages back near LINE_CENTER. RFID + ultrasonic drive
  // all behaviour changes.
  long avg = 0, sum = 0;
  uint16_t calibratedVals[SensorCount];
  for (int i = 0; i < SensorCount; i++) {
    unsigned int rawVal = readPrivate(sensorPins[i]);
    int v = constrain(map(rawVal, minValues[i], maxValues[i], 0, 1000), 0, 1000);
    calibratedVals[i] = v;
    avg += (long)v * (i * 1000);
    sum += v;
  }

  if (sum < LINE_SUM_MIN) {
    stopMotors();
  } else {
    lastPosition   = avg / sum;
    const int err  = LINE_CENTER - (int)lastPosition;
    const int corr = (int)(KP * (float)err);
    motoron.setSpeedNow(LEFT_MOTOR,  constrain(BASE_SPEED + corr, -800, 800));
    motoron.setSpeedNow(RIGHT_MOTOR, constrain(BASE_SPEED - corr, -800, 800));
  }
}
