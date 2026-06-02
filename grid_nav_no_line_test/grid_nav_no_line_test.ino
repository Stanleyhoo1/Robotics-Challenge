// ─────────────────────────────────────────
// Task 3 (no-line variant) — Solid Grid Navigation on the top half of the
// arena (rows 0..3, no painted lines connecting holes).
//
// Same fixed path as grid_nav_test (line-zone variant):
//   forward 2 nodes → RIGHT turn → forward 1 node → LEFT turn → forward 2 nodes
//
// NO line follower. Each hop = one cell = GRID_SPACING_CM (25 cm) driven
// straight with gyro heading-lock + encoder distance measurement, mirroring
// main/navigation.ino's no-line-zone branch in navArenaTick.
//
// Node arrival is detected by either:
//   1. RFID scan of a new UID (preferred — ground truth), OR
//   2. Dead-reckon: hopDistanceCm() >= GRID_SPACING_CM * NODE_ARRIVAL_FRACTION
//      (= 25 * 0.85 = 21.25 cm), used as a fallback when the RFID misses.
//
// 4 actions consumed across 5 arrivals (same as line-zone variant):
//   arrival 1 → STRAIGHT
//   arrival 2 → RIGHT (in-place 90°)
//   arrival 3 → LEFT  (in-place 90°)
//   arrival 4 → STRAIGHT
//   arrival 5 → STOP
//
// After every arrival the robot halts for NODE_STOP_MS (1 s) before the
// next action. UID dedup keeps a single tag from firing twice while the
// bot dwells over it during the hold or post-turn.
//
// Bot starts ON the start RFID tag. First-enable does a short heading-locked
// forward nudge to clear the start tag before counting begins.
//
// Turn approach mirrors baseTurnBlocking() in main/navigation.ino:
//   pre-nudge 5 cm forward (heading-locked) → turnDegrees → post-nudge 150 ms.
//
// Enable / disable:
//   - Power button (pin 17, INPUT_PULLUP): each press toggles isEnabled.
//   - WiFi heartbeat (type=heartbeat enable=1|0): sets isEnabled, with a
//     1 s timeout that disables motors if heartbeats stop arriving.
//   - LED on pins 48/49: solid red while enabled, blinking red when disabled.
// ─────────────────────────────────────────

#include <Wire.h>
#include <Motoron.h>
#include <LSM6.h>
#include <MiniMessenger.h>
#include "MFRC522_I2C.h"
#include "secrets.h"

// ---- Motors ----
MotoronI2C motoron(16, &Wire1);
const uint8_t LEFT_MOTOR  = 1;
const uint8_t RIGHT_MOTOR = 2;

const int BASE_SPEED = 300;     // hop drive speed

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

// Per-direction overshoot trim (deg). Subtracted from |target| in
// turnDegrees so the integrator hits its break sooner on the direction
// that tends to coast farther. Mirrors RIGHT_TURN_TRIM_DEG /
// LEFT_TURN_TRIM_DEG in main/config.h — tune these per physical bot.
const float RIGHT_TURN_TRIM_DEG = 5.0f;
const float LEFT_TURN_TRIM_DEG  = 0.0f;

// ---- IMU ----
LSM6 imu;
const float GYRO_SENS  = 0.00875f;
const float HEADING_KP = 3.0f;  // matches main/config.h
float gyroZOffset = 0;

// ---- Encoders (quadrature, one per rear wheel, ISR-driven) ----
const int   ENC_BL_A     = 22;
const int   ENC_BL_B     = 23;
const int   ENC_BR_A     = 14;
const int   ENC_BR_B     = 15;
const float TICKS_PER_CM = 159.97f;     // matches TICKS_PER_CM_FALLBACK in main/

volatile long encBL = 0;
volatile long encBR = 0;
volatile bool lastBL_A = false, lastBL_B = false;
volatile bool lastBR_A = false, lastBR_B = false;

// ---- Grid / hop constants (match main/config.h) ----
const float GRID_SPACING_CM      = 25.0f;
const float NODE_ARRIVAL_FRACTION = 0.85f;   // dead-reckon arrival threshold

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

// ---- Status LED ----
const int LED_R = 48;
const int LED_G = 49;
const unsigned long LED_BLINK_INTERVAL_MS = 500;
unsigned long lastBlinkMs = 0;
bool          blinkState  = false;

// ---- Power button ----
const int POWER_BUTTON = 17;

// ---- Task sequence ----
const int junctionActions[]      = { 0, 1, -1, 0 };
const int NODE_TOTAL             = 5;
const unsigned long NODE_STOP_MS = 1000;

int   nodeCount    = 0;
bool  runComplete  = false;
bool  startCleared = false;
bool  hopActive    = false;
char  lastScannedUid[32] = "";

const float START_FORWARD_NUDGE_CM     = 12.0f;  // clear the start tag
// Pre-turn nudge: per-direction forward distance to centre the wheel axis
// (= pivot) over the tag before the in-place rotation. Split so an
// asymmetric pivot offset can be compensated independently.
const float RIGHT_PRE_TURN_FORWARD_CM  = 6.0f;
const float LEFT_PRE_TURN_FORWARD_CM   = 5.0f;
const unsigned long JUNCTION_NUDGE_MS  = 150;    // re-engage past the tag after pivot

// ---- Hop heading state (no-line dead-reckon heading-lock) ----
float         hopHeadingDeg        = 0.0f;
unsigned long lastHopHeadingMicros = 0;
bool          hopHeadingActive     = false;

// Forward decls — blocking helpers poll the enable gate.
void wifiLoop();
void checkPowerButton();

static inline void stopMotors() {
  motoron.setSpeedNow(LEFT_MOTOR,  0);
  motoron.setSpeedNow(RIGHT_MOTOR, 0);
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

long  straightTicks()  { return (encBL + encBR) / 2; }
float hopDistanceCm()  { return (float)straightTicks() / TICKS_PER_CM; }
void  encoderResetHop(){ noInterrupts(); encBL = encBR = 0; interrupts(); }
bool  nearNextNode()   { return hopDistanceCm() >= GRID_SPACING_CM * NODE_ARRIVAL_FRACTION; }

// ─────────────────────────────────────────
// Hop heading lock — gyro-integrated heading delta over the hop, fed back
// as a proportional speed correction. Mirrors resetHopHeading /
// updateHopHeading / applyHeadingCorrection in main/motors.ino.
// ─────────────────────────────────────────
void resetHopHeading() {
  hopHeadingDeg = 0.0f;
  lastHopHeadingMicros = micros();
  hopHeadingActive = true;
}

void endHopHeading() { hopHeadingActive = false; }

void updateHopHeading() {
  if (!hopHeadingActive) return;
  imu.read();
  unsigned long now = micros();
  float dt = (now - lastHopHeadingMicros) / 1000000.0f;
  lastHopHeadingMicros = now;
  float gz = -((imu.g.z - gyroZOffset) * GYRO_SENS);
  hopHeadingDeg += gz * dt;
}

void applyHeadingCorrection() {
  float correction = HEADING_KP * hopHeadingDeg;
  int leftSpeed  = constrain(BASE_SPEED - (int)correction, 0, 800);
  int rightSpeed = constrain(BASE_SPEED + (int)correction, 0, 800);
  motoron.setSpeedNow(LEFT_MOTOR,  leftSpeed);
  motoron.setSpeedNow(RIGHT_MOTOR, rightSpeed);
}

// ─────────────────────────────────────────
// Fixed-distance heading-locked forward drive. Used for the start-clear
// nudge and the pre-turn centring nudge. Returns false if disabled mid-drive.
// ─────────────────────────────────────────
bool driveForwardCmHeadingLocked(float cm) {
  encoderResetHop();
  resetHopHeading();
  motoron.setSpeedNow(LEFT_MOTOR,  BASE_SPEED);
  motoron.setSpeedNow(RIGHT_MOTOR, BASE_SPEED);
  while (hopDistanceCm() < cm) {
    updateHopHeading();
    applyHeadingCorrection();
    wifiLoop();
    checkPowerButton();
    if (!isEnabled) { stopMotors(); endHopHeading(); return false; }
    delay(5);
  }
  stopMotors();
  endHopHeading();
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

  // Per-side asymmetric speed (mirrors main/motors.ino turnDegrees).
  const int leftSpeed  = (direction > 0) ? RIGHT_TURN_FORWARD_SPEED  : LEFT_TURN_BACKWARD_SPEED;
  const int rightSpeed = (direction > 0) ? RIGHT_TURN_BACKWARD_SPEED : LEFT_TURN_FORWARD_SPEED;

  // Per-direction trim: stop the integrator a few degrees before the nominal
  // target so an over-coasting direction doesn't overshoot.
  const float trimDeg    = (direction > 0) ? RIGHT_TURN_TRIM_DEG : LEFT_TURN_TRIM_DEG;
  const float stopMagDeg = fabsf(targetDegrees) - trimDeg;

  while (true) {
    imu.read();
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0f;
    lastTime = now;
    float gz = -((imu.g.z - gyroZOffset) * GYRO_SENS);
    accumulated += gz * dt;

    if (fabsf(accumulated) >= stopMagDeg) break;

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
// Polled idle hold. Motors stay off; wifi + button stay live.
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
// One-shot RFID read.
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
// Per-node arrival handler: stop the hop, hold NODE_STOP_MS, then act on
// the next queued action. Increments nodeCount; on NODE_TOTAL latches
// runComplete. Same shape as grid_nav_test.ino's handleRfidNode, with the
// turn sequence mirroring main/navigation.ino's baseTurnBlocking.
// ─────────────────────────────────────────
void handleNodeArrival(const char* uid, bool deadReckon) {
  stopMotors();
  endHopHeading();
  nodeCount++;
  Serial.print("Node "); Serial.print(nodeCount);
  Serial.print('/');     Serial.print(NODE_TOTAL);
  Serial.print(deadReckon ? " (dead-reckon) tag=" : " (rfid) tag=");
  Serial.println(uid);

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
    // STRAIGHT: no nudge, no turn. Main loop will start a fresh hop on the
    // next tick; UID dedup prevents this tag re-firing.
    return;
  }

  // TURN: mirrors baseTurnBlocking in main/navigation.ino.
  // Phase 1 — pre-turn nudge: per-direction forward (heading-locked) so the
  // wheel axis is over the tag when the pivot happens.
  const float preNudgeCm = (action > 0) ? RIGHT_PRE_TURN_FORWARD_CM : LEFT_PRE_TURN_FORWARD_CM;
  Serial.print("  pre-turn nudge "); Serial.print(preNudgeCm); Serial.println(" cm");
  if (!driveForwardCmHeadingLocked(preNudgeCm)) return;

  // Phase 2 — in-place gyro-tracked rotation.
  if (!turnDegrees(action * 90.0f)) return;

  // Phase 3 — post-turn nudge: 150 ms forward, so the next hop starts off
  // the tag and the dwell-deduper isn't fighting us.
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

  if (firstCall) {
    wasPressed = pressedNow;
    firstCall  = false;
    return;
  }

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

  pinMode(POWER_BUTTON, INPUT_PULLUP);

  encoderSetup();
  rfid.PCD_Init();
  Serial.println("RFID ready.");
  wifiSetup();

  calibrateGyro();

  Serial.println();
  Serial.println("=== Grid Nav Test (NO-LINE: gyro heading-lock + encoder hops) ===");
  Serial.println("Sequence: STRAIGHT, RIGHT, LEFT, STRAIGHT, STOP (5 arrivals)");
  Serial.print("Hop length = "); Serial.print(GRID_SPACING_CM, 1); Serial.print(" cm; ");
  Serial.print("dead-reckon arrival = ");
  Serial.print(GRID_SPACING_CM * NODE_ARRIVAL_FRACTION, 1); Serial.println(" cm");
  Serial.println("Press power button or send heartbeat enable=1 to start.");
}

// ─────────────────────────────────────────
// Loop
// ─────────────────────────────────────────
void loop() {
  wifiLoop();
  checkPowerButton();

  // Disabled: motors off, hold state.
  if (!isEnabled) {
    stopMotors();
    if (hopActive) {
      endHopHeading();
      hopActive = false;          // hop is rebuilt cleanly on the next enable
    }
    delay(10);
    return;
  }

  if (runComplete) {
    stopMotors();
    return;
  }

  // First-enable: clear the starting tag (heading-locked nudge).
  if (!startCleared) {
    Serial.print("Clearing start tag — heading-locked forward ");
    Serial.print(START_FORWARD_NUDGE_CM, 1);
    Serial.println(" cm");
    if (!driveForwardCmHeadingLocked(START_FORWARD_NUDGE_CM)) return;
    startCleared = true;
    Serial.println("Heading-lock hops engaged.");
    return;
  }

  // Begin a new hop if one isn't running yet.
  if (!hopActive) {
    encoderResetHop();
    resetHopHeading();
    motoron.setSpeedNow(LEFT_MOTOR,  BASE_SPEED);
    motoron.setSpeedNow(RIGHT_MOTOR, BASE_SPEED);
    hopActive = true;
    return;
  }

  // Hop in progress — keep heading locked, then check both arrival paths.
  updateHopHeading();
  applyHeadingCorrection();

  // RFID is the truth. Dedup against lastScannedUid so a tag the bot dwells
  // over during the 1 s stop / post-turn doesn't fire twice.
  {
    char uid[32];
    if (readRfidNonBlocking(uid, sizeof(uid))) {
      if (strcmp(uid, lastScannedUid) != 0) {
        strncpy(lastScannedUid, uid, sizeof(lastScannedUid) - 1);
        lastScannedUid[sizeof(lastScannedUid) - 1] = '\0';
        hopActive = false;
        handleNodeArrival(uid, false);
        return;
      }
      // Same UID as last — keep driving; we're still on the tag we just
      // counted (e.g. just left a node and the reader still sees it).
    }
  }

  // Dead-reckon fallback: the RFID was missed but we've covered ~one cell.
  // Same handling as an RFID arrival, just with a synthetic tag id.
  if (nearNextNode()) {
    hopActive = false;
    handleNodeArrival("DEAD_RECKON", true);
    return;
  }
}
