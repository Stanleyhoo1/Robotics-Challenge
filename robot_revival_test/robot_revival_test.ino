// ─────────────────────────────────────────
// Task 8 — Touch-Based Robot Revival (3 pts)
//
// Conditions recreated from Task 7 (line-zone obstacle scenario): the
// stranded robot sits 2 nodes ahead of the active robot on the line grid.
// Unlike obstacle avoidance, we do NOT route around — we drive INTO it
// gently, hold the contact (which presses the stranded robot's revive
// button) for 1 s, then back off to release the button.
//
// Behavior:
//   APPROACH    — line-follow PID with smooth deceleration based on
//                 forward ultrasonic distance. Speed ramps linearly from
//                 BASE_SPEED (far) down to APPROACH_MIN_SPEED (touching).
//                 The very last cm or two will be ultrasonic-blind (HC-SR04
//                 dead-zone < ~2 cm) — that's fine, the button trigger
//                 catches final contact.
//   REVIVING    — first revive-button press latches: motors stopped, hold
//                 for REVIVE_HOLD_MS (1 s). Button is allowed to bounce
//                 back open during the hold (mechanical contact is rarely
//                 perfectly steady); we just need the timer to elapse.
//   BACKING_OFF — drive backwards at -BASE_SPEED until both revive buttons
//                 read HIGH (released) for BACKOFF_RELEASE_MS continuously,
//                 capped at BACKOFF_MAX_MS as a safety bound.
//   COMPLETE    — stop. Run ends; re-enable to repeat.
//
// Same gating model as the other tests:
//   - Power button (pin 17, INPUT_PULLUP): toggles isEnabled.
//   - WiFi heartbeat (type=heartbeat enable=1|0) with 1 s timeout.
//   - LED on 48/49: solid red enabled, blinking red disabled.
//   - Shares IR calibration via kvstore with main/.
//
// REVIVE_BUTTON_1 = 52, REVIVE_BUTTON_2 = 53. INPUT_PULLUP — pressed = LOW.
// Either button triggers contact.
// ─────────────────────────────────────────

#include <kvstore_global_api.h>
#include <Wire.h>
#include <Motoron.h>
#include <LSM6.h>
#include <MiniMessenger.h>
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

const int   BASE_SPEED   = 300;
const float KP           = 0.20f;
const int   LINE_SUM_MIN = 200;
const int   LINE_CENTER  = 4000;

// ---- IMU (kept for gyro calibration step, not used for the revival task) ----
LSM6 imu;
const float GYRO_SENS = 0.00875f;
float gyroZOffset = 0;

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

// ---- Forward ultrasonic (HC-SR04 on 44/45) ----
const int FORWARD_TRIG_PIN  = 44;
const int FORWARD_ECHO_PIN  = 45;
// 5000 µs = ~85 cm max — anything further doesn't matter for the approach.
const unsigned long ULTRASONIC_TIMEOUT = 5000;
const unsigned long FWD_PING_INTERVAL_MS = 20;   // ~50 Hz ping cadence
const unsigned long FWD_LOG_INTERVAL_MS  = 500;  // print reading this often
float                lastForwardDistanceCm = -1.0f;
unsigned long        lastFwdPingMs         = 0;
unsigned long        lastFwdLogMs          = 0;
unsigned long        lastCloseReadingMs    = 0;  // last time reading < APPROACH_DECEL_START_CM
const unsigned long  OOR_PERSISTENCE_MS    = 400;

// ---- Revive buttons ----
const int REVIVE_BUTTON_1 = 52;
const int REVIVE_BUTTON_2 = 53;

// ---- WiFi / heartbeat / enable state ----
MiniMessenger        messenger;
const char*          BoardId              = "Master";
bool                 isEnabled            = false;
unsigned long        lastHeartbeatMs      = 0;
unsigned long        lastRegisterMs       = 0;
const unsigned long  HEARTBEAT_TIMEOUT_MS = 1000;
const unsigned long  REGISTER_INTERVAL_MS = 10000;

// ---- Status LED + power button ----
const int LED_R = 48;
const int LED_G = 49;
const unsigned long LED_BLINK_INTERVAL_MS = 500;
unsigned long lastBlinkMs = 0;
bool          blinkState  = false;
const int POWER_BUTTON = 17;

// ─────────────────────────────────────────
// Approach / revival tuning
// ─────────────────────────────────────────
// Distance at which deceleration begins. Beyond this, drive at BASE_SPEED.
const float APPROACH_DECEL_START_CM = 30.0f;
// Slowest forward speed used at near-contact. Must be high enough that the
// motors don't stall, low enough that the contact isn't a jolt.
const int   APPROACH_MIN_SPEED      = 120;
// How long the active robot holds against the stranded robot's button.
const unsigned long REVIVE_HOLD_MS  = 1000;
// Back-off phase: drive backwards until both buttons report HIGH for
// BACKOFF_RELEASE_MS continuously. Capped at BACKOFF_MAX_MS so we don't
// reverse forever if a button stays mechanically stuck.
const unsigned long BACKOFF_RELEASE_MS = 150;
const unsigned long BACKOFF_MAX_MS     = 800;
// Clear the start tag/segment before the line follower engages so we're not
// already overlapping with the obstacle / sitting on a tag.
const float START_FORWARD_NUDGE_CM = 12.0f;

// ─────────────────────────────────────────
// Run-state machine
// ─────────────────────────────────────────
enum RunState {
  RUN_APPROACH,
  RUN_REVIVING,
  RUN_BACKING_OFF,
  RUN_COMPLETE
};
RunState runState     = RUN_APPROACH;
bool     startCleared = false;

unsigned long reviveStartMs   = 0;
unsigned long backoffStartMs  = 0;
unsigned long backoffReleaseStartMs = 0;  // tracks "both buttons HIGH" streak

// Forward decls — blocking helpers poll the enable gate.
void wifiLoop();
void checkPowerButton();

static inline void stopMotors() {
  motoron.setSpeedNow(LEFT_MOTOR,  0);
  motoron.setSpeedNow(RIGHT_MOTOR, 0);
}

// ─────────────────────────────────────────
// IR helpers (shared kvstore keys with main/)
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
// IMU (gyro offset captured at boot; not used during the approach but kept
// for parity with the other test sketches in case extensions need it).
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
float ticksDistanceCm(long base) { return (float)(straightTicks() - base) / TICKS_PER_CM; }

// Encoder-gated forward drive. Used only for the start-clear nudge.
bool forwardCm(float cm) {
  long startTicks = straightTicks();
  motoron.setSpeedNow(LEFT_MOTOR,  BASE_SPEED);
  motoron.setSpeedNow(RIGHT_MOTOR, BASE_SPEED);
  while (ticksDistanceCm(startTicks) < cm) {
    wifiLoop();
    checkPowerButton();
    if (!isEnabled) { stopMotors(); return false; }
    delay(2);
  }
  stopMotors();
  return true;
}

// ─────────────────────────────────────────
// Forward ultrasonic. Same pattern as obstacle_avoid_test.
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

  if (lastForwardDistanceCm >= 0.0f && lastForwardDistanceCm < APPROACH_DECEL_START_CM) {
    lastCloseReadingMs = now;
  }

  if (now - lastFwdLogMs >= FWD_LOG_INTERVAL_MS) {
    lastFwdLogMs = now;
    Serial.print("[FWD] ");
    if (lastForwardDistanceCm < 0.0f) Serial.println("OOR");
    else { Serial.print(lastForwardDistanceCm); Serial.println(" cm"); }
  }
}

// ─────────────────────────────────────────
// Map current forward distance → desired forward speed.
// Linear ramp: BASE_SPEED at APPROACH_DECEL_START_CM, APPROACH_MIN_SPEED at
// 0 cm. OOR readings inherit "still close" if we saw < APPROACH_DECEL_START_CM
// recently — otherwise default to BASE_SPEED (sensor genuinely sees nothing
// nearby and the line is what's keeping the bot pointed correctly).
// ─────────────────────────────────────────
int approachSpeedForDistance() {
  float d = lastForwardDistanceCm;
  bool oor = (d < 0.0f);
  if (oor) {
    if (lastCloseReadingMs != 0 &&
        (millis() - lastCloseReadingMs) < OOR_PERSISTENCE_MS) {
      // Recently close — assume we're still near contact, decelerate.
      return APPROACH_MIN_SPEED;
    }
    return BASE_SPEED;
  }
  if (d >= APPROACH_DECEL_START_CM) return BASE_SPEED;
  if (d <= 0.0f)                    return APPROACH_MIN_SPEED;
  // Linear interpolation across [0, APPROACH_DECEL_START_CM] →
  // [APPROACH_MIN_SPEED, BASE_SPEED]. (long) cast keeps map() happy.
  return (int)map((long)(d * 100.0f),
                  0, (long)(APPROACH_DECEL_START_CM * 100.0f),
                  APPROACH_MIN_SPEED, BASE_SPEED);
}

// ─────────────────────────────────────────
// Revive buttons — either one going LOW = mechanical contact made.
// ─────────────────────────────────────────
bool reviveButtonPressed() {
  return (digitalRead(REVIVE_BUTTON_1) == LOW) ||
         (digitalRead(REVIVE_BUTTON_2) == LOW);
}

// ─────────────────────────────────────────
// LED / button / WiFi gating — mirrors the other test sketches.
// ─────────────────────────────────────────
void setLED(int r, int g) { digitalWrite(LED_R, r); digitalWrite(LED_G, g); }

void updateLED() {
  // Override: while a revive button is held, show solid green (matches
  // main/wifi.ino's behaviour — useful visual confirmation of contact).
  if (reviveButtonPressed()) {
    setLED(LOW, HIGH);
    return;
  }
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
  pinMode(POWER_BUTTON,    INPUT_PULLUP);
  pinMode(REVIVE_BUTTON_1, INPUT_PULLUP);
  pinMode(REVIVE_BUTTON_2, INPUT_PULLUP);

  pinMode(FORWARD_TRIG_PIN, OUTPUT);
  pinMode(FORWARD_ECHO_PIN, INPUT);
  digitalWrite(FORWARD_TRIG_PIN, LOW);

  encoderSetup();
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
  Serial.println("=== Touch-Based Robot Revival Test ===");
  Serial.print("Deceleration starts at "); Serial.print(APPROACH_DECEL_START_CM);
  Serial.print(" cm; contact speed = "); Serial.println(APPROACH_MIN_SPEED);
  Serial.print("Revive hold = ");        Serial.print(REVIVE_HOLD_MS); Serial.println(" ms");
  Serial.print("Backoff cap = ");        Serial.print(BACKOFF_MAX_MS); Serial.println(" ms");
  Serial.println("Press power button or send heartbeat enable=1 to start.");
}

// ─────────────────────────────────────────
// State handlers
// ─────────────────────────────────────────
void approachTick() {
  // Stop the instant we feel contact.
  if (reviveButtonPressed()) {
    stopMotors();
    Serial.println("[CONTACT] revive button down — holding");
    reviveStartMs = millis();
    runState = RUN_REVIVING;
    return;
  }

  // Smooth deceleration: base PID forward speed is mapped from forward dist.
  pingForwardUltrasonic();
  const int targetSpeed = approachSpeedForDistance();

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
    // No line visible. Keep approaching at the decelerated speed — the
    // stranded robot might be on a no-line section right ahead.
    motoron.setSpeedNow(LEFT_MOTOR,  targetSpeed);
    motoron.setSpeedNow(RIGHT_MOTOR, targetSpeed);
  } else {
    lastPosition   = avg / sum;
    const int err  = LINE_CENTER - (int)lastPosition;
    const int corr = (int)(KP * (float)err);
    motoron.setSpeedNow(LEFT_MOTOR,  constrain(targetSpeed + corr, -800, 800));
    motoron.setSpeedNow(RIGHT_MOTOR, constrain(targetSpeed - corr, -800, 800));
  }
}

void revivingTick() {
  stopMotors();
  // Wait the full REVIVE_HOLD_MS — even if the button bounces back open
  // mid-hold, we just need the timer to elapse. The point is to keep
  // pressure on the stranded robot's revive pad for a full second.
  if (millis() - reviveStartMs >= REVIVE_HOLD_MS) {
    Serial.println("[REVIVE] 1 s hold complete — backing off");
    backoffStartMs        = millis();
    backoffReleaseStartMs = 0;
    runState              = RUN_BACKING_OFF;
  }
}

void backingOffTick() {
  motoron.setSpeedNow(LEFT_MOTOR,  -BASE_SPEED);
  motoron.setSpeedNow(RIGHT_MOTOR, -BASE_SPEED);

  const bool buttonsReleased = !reviveButtonPressed();
  const unsigned long now    = millis();

  // Track the streak of "both buttons HIGH" so we only declare release
  // after BACKOFF_RELEASE_MS of stable un-press (debounce + mechanical
  // settling).
  if (buttonsReleased) {
    if (backoffReleaseStartMs == 0) backoffReleaseStartMs = now;
    if (now - backoffReleaseStartMs >= BACKOFF_RELEASE_MS) {
      stopMotors();
      Serial.println("[BACKOFF] buttons released cleanly — revival complete");
      runState = RUN_COMPLETE;
      return;
    }
  } else {
    backoffReleaseStartMs = 0;
  }

  // Hard cap: don't reverse forever if a button gets mechanically stuck.
  if (now - backoffStartMs >= BACKOFF_MAX_MS) {
    stopMotors();
    Serial.println("[BACKOFF] cap reached — stopping anyway");
    runState = RUN_COMPLETE;
  }
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

  if (runState == RUN_COMPLETE) {
    stopMotors();
    return;
  }

  // First-enable: nudge forward to clear the start tag / segment so the
  // approach starts cleanly and we're not already touching the obstacle.
  if (!startCleared) {
    Serial.print("Clearing start — forward ");
    Serial.print(START_FORWARD_NUDGE_CM, 1); Serial.println(" cm");
    if (!forwardCm(START_FORWARD_NUDGE_CM)) return;
    startCleared = true;
    Serial.println("Approach engaged.");
  }

  switch (runState) {
    case RUN_APPROACH:    approachTick();    break;
    case RUN_REVIVING:    revivingTick();    break;
    case RUN_BACKING_OFF: backingOffTick();  break;
    case RUN_COMPLETE:    stopMotors();      break;
  }
}
