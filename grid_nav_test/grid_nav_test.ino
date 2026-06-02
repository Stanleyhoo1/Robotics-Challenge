// ─────────────────────────────────────────
// Task 3 — Solid Grid Navigation (3 pts)
//
// Fixed path on the line-grid half of the arena:
//   forward 2 nodes → RIGHT turn → forward 1 node → LEFT turn → forward 2 nodes
//
// NODES are RFID tags. Line-follower is IR-PID only; junctions in the IR
// signal are NOT used for counting. The sequence of 4 actions consumed
// across the 5 RFID scans:
//   scan 1 (= 1 node forward) → STRAIGHT
//   scan 2 (= 2 nodes fwd)    → RIGHT  (in-place 90°)
//   scan 3 (= 1 past R-turn)  → LEFT   (in-place 90°)
//   scan 4 (= 1 past L-turn)  → STRAIGHT
//   scan 5 (= destination)    → STOP
//
// After every scan the robot stops for NODE_STOP_MS (1 s) before executing
// the action. Same UID can't fire twice — lastScannedUid dedup keeps the
// counter honest while the bot dwells over a tag.
//
// Bot starts ON the start RFID tag. The first time the run is enabled the
// loop does an encoder-gated forward nudge to clear the start tag before
// RFID polling begins — otherwise the start tag would fire as scan 1.
//
// Enable / disable:
//   - Power button (pin 17, INPUT_PULLUP): each press toggles isEnabled.
//   - WiFi heartbeat (type=heartbeat enable=1|0): sets isEnabled, with a
//     1 s timeout that disables motors if heartbeats stop arriving.
//   - LED on pins 48/49: solid red while enabled, blinking red when disabled.
//
// Shares IR calibration via kvstore with main/. Send 'c' within 3 s of boot
// to recalibrate.
// ─────────────────────────────────────────

#include <kvstore_global_api.h>
#include <Wire.h>
#include <Motoron.h>
#include <LSM6.h>
#include <MiniMessenger.h>
#include "MFRC522_I2C.h"
#include "secrets.h"

// ---- IR Sensor ----
const int sensorPins[9]    = {30, 31, 32, 33, 34, 35, 36, 37, 38};
const int ctrlPin          = 12;
const int SensorCount      = 9;
const unsigned int timeout = 2500;

uint16_t minValues[9];
uint16_t maxValues[9];
uint16_t lastPosition = 0;

// ---- Motors ----
MotoronI2C motoron(16, &Wire1);
const uint8_t LEFT_MOTOR  = 1;
const uint8_t RIGHT_MOTOR = 2;

const int   BASE_SPEED       = 300;
const float KP               = 0.20f;
const int   LINE_SUM_MIN     = 200;     // sum below this = line lost
const int   LINE_CENTER      = 4000;    // target position (sensor 4 of 9)

// ---- Encoders (quadrature, one per rear wheel, ISR-driven) ----
const int   ENC_BL_A     = 22;
const int   ENC_BL_B     = 23;
const int   ENC_BR_A     = 14;
const int   ENC_BR_B     = 15;
const float TICKS_PER_CM = 159.97f;

volatile long encBL = 0;
volatile long encBR = 0;
volatile bool lastBL_A = false, lastBL_B = false;
volatile bool lastBR_A = false, lastBR_B = false;

// ---- IMU ----
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

// ---- RFID ----
MFRC522_I2C rfid(0x28, 255);

// ---- WiFi / heartbeat / enable state ----
MiniMessenger        messenger;
const char*          BoardId              = "Master";
bool                 isEnabled            = false;
unsigned long        lastHeartbeatMs      = 0;
unsigned long        lastRegisterMs       = 0;
const unsigned long  HEARTBEAT_TIMEOUT_MS = 1000;
const unsigned long  REGISTER_INTERVAL_MS = 10000;

// ---- Status LED (RGB on pins 48/49, mirroring main/config.h) ----
const int LED_R = 48;
const int LED_G = 49;
const unsigned long LED_BLINK_INTERVAL_MS = 500;
unsigned long lastBlinkMs = 0;
bool          blinkState  = false;

// ---- Power button ----
const int POWER_BUTTON = 17;

// ---- Task sequence ----
// 0 = straight, 1 = right, -1 = left. Length = 4; the 5th scan stops.
const int junctionActions[]      = { 0, 1, -1, 0 };
const int NODE_TOTAL             = 5;       // total RFID scans (incl. destination)
const unsigned long NODE_STOP_MS = 1000;    // halt this long after each scan

int   nodeCount     = 0;
bool  runComplete   = false;
bool  startCleared  = false;                // true after the start-tag clear nudge
char  lastScannedUid[32] = "";              // dedup so a dwelt-over tag fires once

// ~half a grid cell, well past the start tag's RFID read range.
const float START_FORWARD_NUDGE_CM = 12.0f;

// Turn nudges: mirror main/navigation.ino's baseTurnBlocking().
//   PRE: drive forward PRE_TURN_FORWARD_CM (encoder-gated) to centre the
//        wheel axis (= pivot) over the tag before rotating.
//   POST: drive forward JUNCTION_NUDGE_MS (time-gated) after the rotation
//         to re-engage the line on the new branch before PID resumes.
// Skipped entirely for STRAIGHT actions — the bot just keeps going.
const float        PRE_TURN_FORWARD_CM = 5.0f;
const unsigned long JUNCTION_NUDGE_MS  = 150;

// Forward decls — blocking helpers poll the enable gate.
void wifiLoop();
void checkPowerButton();

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
// Encoders (4× decoding, ISR-driven). Pattern copied from main/motors.ino.
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
float distanceCm()    { return (float)straightTicks() / TICKS_PER_CM; }
void  resetEncoders() { noInterrupts(); encBL = encBR = 0; interrupts(); }

// Encoder-gated forward drive that respects the enable gate. Returns false
// if disabled mid-drive so the caller can bail out of its sequence.
bool forwardCm(float cm) {
  resetEncoders();
  motoron.setSpeedNow(LEFT_MOTOR,  BASE_SPEED);
  motoron.setSpeedNow(RIGHT_MOTOR, BASE_SPEED);
  while (distanceCm() < cm) {
    wifiLoop();
    checkPowerButton();
    if (!isEnabled) { stopMotors(); return false; }
    delay(2);
  }
  stopMotors();
  return true;
}

// ─────────────────────────────────────────
// Gyro-tracked in-place turn. Polls enable gate every tick.
// ─────────────────────────────────────────
bool turnDegrees(float targetDegrees) {
  motoron.clearMotorFault(LEFT_MOTOR);
  motoron.clearMotorFault(RIGHT_MOTOR);

  float accumulated = 0;
  unsigned long lastTime = micros();
  int direction = (targetDegrees > 0) ? 1 : -1;

  // Per-side speed picked by which wheel is going forward vs backward, and
  // by which direction we're turning. Mirrors main/motors.ino turnDegrees.
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

// ─────────────────────────────────────────
// Polled idle hold for `ms`. Motors stay off; wifi + button stay live so
// the enable gate, heartbeat watchdog, and LED blink keep updating.
// Returns false if the enable gate flipped during the hold.
// ─────────────────────────────────────────
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
// One-shot RFID read. Fills uidOut and returns true if a tag was present.
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
// Per-node handler: stop, hold for NODE_STOP_MS, then act on the next
// queued action. Increments nodeCount; on NODE_TOTAL latches runComplete.
// ─────────────────────────────────────────
void handleRfidNode(const char* uid) {
  stopMotors();
  nodeCount++;
  Serial.print("Node "); Serial.print(nodeCount);
  Serial.print('/');     Serial.print(NODE_TOTAL);
  Serial.print(" tag="); Serial.println(uid);

  Serial.print("  holding "); Serial.print(NODE_STOP_MS); Serial.println(" ms");
  if (!pausedWait(NODE_STOP_MS)) return;

  if (nodeCount >= NODE_TOTAL) {
    Serial.println("  DESTINATION reached. Halting.");
    runComplete = true;
    return;
  }

  const int action = junctionActions[nodeCount - 1];
  Serial.print("  -> ");
  Serial.println(action == 0 ? "straight" : action == 1 ? "right" : "left");

  if (action == 0) {
    // STRAIGHT: no nudge, no turn — PID resumes on the next loop tick and
    // drives the bot off the tag. UID dedup prevents this tag re-firing.
    return;
  }

  // TURN: mirrors baseTurnBlocking in main/navigation.ino.
  // Phase 1 — pre-turn nudge: 5 cm forward (encoder-gated) so the wheel
  // axis is over the tag when the pivot happens.
  Serial.print("  pre-turn nudge "); Serial.print(PRE_TURN_FORWARD_CM); Serial.println(" cm");
  if (!forwardCm(PRE_TURN_FORWARD_CM)) return;

  // Phase 2 — in-place gyro-tracked rotation.
  if (!turnDegrees(action * 90.0f)) return;

  // Phase 3 — post-turn nudge: 150 ms forward to re-engage the line on the
  // new branch before PID resumes (followLineBase holds motors at 0 until
  // it actually sees a line; the nudge guarantees we present sensors to it).
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
}

// ─────────────────────────────────────────
// LED / button / WiFi gating — mirrors main/wifi.ino + main.ino patterns
// ─────────────────────────────────────────
void setLED(int r, int g) {
  digitalWrite(LED_R, r);
  digitalWrite(LED_G, g);
}

void updateLED() {
  if (isEnabled) {
    setLED(HIGH, LOW);                       // solid red while running
  } else {
    if (millis() - lastBlinkMs > LED_BLINK_INTERVAL_MS) {
      lastBlinkMs = millis();
      blinkState = !blinkState;
      setLED(blinkState ? HIGH : LOW, LOW);  // blink red while disabled
    }
  }
}

void checkPowerButton() {
  static unsigned long lastPressMs = 0;
  static bool wasPressed = false;
  static bool firstCall  = true;
  const bool pressedNow = (digitalRead(POWER_BUTTON) == LOW);

  // Seed from current pin state so holding the button at boot doesn't read
  // as a fresh press on the first poll.
  if (firstCall) {
    wasPressed = pressedNow;
    firstCall  = false;
    return;
  }

  if (pressedNow && !wasPressed && (millis() - lastPressMs > 200)) {
    isEnabled = !isEnabled;
    lastPressMs = millis();
    // Give the heartbeat watchdog a fresh baseline so it doesn't immediately
    // undo the toggle (only relevant once heartbeats have started arriving).
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

  // Heartbeat-timeout safety only kicks in after we've heard at least one
  // heartbeat. Before that (bench-testing with no server), the button is
  // authoritative.
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

  Wire.begin();    // RFID bus
  Wire1.begin();   // Motoron + IMU bus
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

  Serial.println();
  Serial.println("=== Grid Nav Test (RFID-counted) ===");
  Serial.println("Sequence: STRAIGHT, RIGHT, LEFT, STRAIGHT, STOP (5 RFID scans)");
  Serial.println("Press power button or send heartbeat enable=1 to start.");
}

// ─────────────────────────────────────────
// Loop
// ─────────────────────────────────────────
void loop() {
  wifiLoop();
  checkPowerButton();

  // Disabled: motors off, hold state. The startup nudge and node counter
  // both resume cleanly once re-enabled.
  if (!isEnabled) {
    stopMotors();
    delay(10);
    return;
  }

  if (runComplete) {
    stopMotors();
    return;
  }

  // First-enable: clear the starting tag before RFID polling kicks in.
  // forwardCm respects the enable gate, so a disable mid-nudge leaves
  // startCleared false and we restart the nudge on the next enable.
  if (!startCleared) {
    Serial.print("Clearing start tag — forward ");
    Serial.print(START_FORWARD_NUDGE_CM, 1);
    Serial.println(" cm");
    if (!forwardCm(START_FORWARD_NUDGE_CM)) return;
    startCleared = true;
    Serial.println("Line follow + RFID polling engaged.");
  }

  // RFID is the node counter. Dedup against lastScannedUid so a tag the bot
  // dwells over (during the 1 s stop, or just after a turn) only counts once.
  {
    char uid[32];
    if (readRfidNonBlocking(uid, sizeof(uid))) {
      if (strcmp(uid, lastScannedUid) != 0) {
        strncpy(lastScannedUid, uid, sizeof(lastScannedUid) - 1);
        lastScannedUid[sizeof(lastScannedUid) - 1] = '\0';
        handleRfidNode(uid);
      }
      return;   // skip line-follow this tick — handler already stopped/acted
    }
  }

  // Plain PID. Junctions in the IR signal are ignored — the wide-line
  // crossing averages back near LINE_CENTER and the bot drives straight
  // through. RFID is what triggers all behavior changes.
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
