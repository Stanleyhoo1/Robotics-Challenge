#include <Wire.h>
#include <Servo.h>
#include "MFRC522_I2C.h"
#include <Motoron.h>
#include <LSM6.h>
#include <kvstore_global_api.h>
#include <MiniMessenger.h>
#include "config.h"
#include "globals.h"

// ─────────────────────────────────────────
// TODO:
// 1. Fix turning in place problem, currently wheels slipping/not moving
// 2. Navigation
// 3. Emergency
// 4. Testing planting
// 5. Obstacle detecting and plotting based on other robot pos
// ─────────────────────────────────────────


// ─────────────────────────────────────────
// Global definitions (extern'd in globals.h)
// ─────────────────────────────────────────
bool  showIR               = false;
bool  showDistance         = false;
bool  showEncoders         = false;
bool  showPitch            = false;
bool  showLDR              = false;
bool  isEnabled            = false;
bool  useStateMachine      = true;    // state machine drives behavior; set false to fall back to legacy rfidLoop + applyMotorEnabled
float lastForwardDistanceCm = 0.0f;   // refreshed by the obstacle check at top of loop()

void setup() {
  Serial.begin(SERIAL_BAUD);
  uint32_t startWait = millis();
  while (!Serial && millis() - startWait < SERIAL_WAIT_MS);

  irSetup();
  distanceSetup();
  Wire.begin();
  Wire1.begin();
  rfidSetup();
  servoSetup();
  motorsSetup();
  encoderSetup();
  wifiSetup();

  pinMode(POWER_BUTTON,    INPUT_PULLUP);
  pinMode(REVIVE_BUTTON_1, INPUT_PULLUP);
  pinMode(REVIVE_BUTTON_2, INPUT_PULLUP);

  Serial.println("\nReady. Commands:");
  Serial.println("  <number>            → turn that many degrees");
  Serial.println("  forward [spd]       → move forward 3s");
  Serial.println("  hop                 → one heading-locked hop");
  Serial.println("  ir / distance       → toggle sensor prints");
  Serial.println("  sensors             → toggle all sensor prints");
  Serial.println("  enc / encreset      → encoder print toggle / zero");
  Serial.println("  c / calib           → recalibrate IR / print calib state");
  Serial.println("  nav                 → toggle state machine");
  Serial.println("  state               → dump navState + position");
  Serial.println("  pos <r> <c> <n|e|s|w>  → set robotPos + facing");
  Serial.println("  tag <r> <c> <u|f|i|p>  → set tagMap cell");
  Serial.println("  target              → print selectNextTarget");
  Serial.println("  astar <r1> <c1> <r2> <c2>  → A* next step");
  Serial.println("  selftest            → run logic assertions");
  Serial.println("  <anything else>     → forward to server");
  delay(1000);

  // Change robot state here for testing
  // navState = NAV_WALL_FOLLOW;
}

// Power button: poll the pin every tick, toggle isEnabled on each press
// (HIGH→LOW edge). 200 ms cooldown prevents bounce-induced double-toggle.
// Not static so blocking helpers (turnDegrees, moveForward, sweepTo) can
// also poll it mid-action — otherwise the kill switch is dead while the bot
// is turning or planting.
void checkPowerButton() {
  static unsigned long lastPressMs = 0;
  static bool wasPressed = false;
  static bool firstCall  = true;
  const bool pressedNow = (digitalRead(POWER_BUTTON) == LOW);

  // Seed wasPressed from the real pin state on the first call so that holding
  // the button at boot doesn't get detected as a fresh press the moment we
  // start polling.
  if (firstCall) {
    wasPressed = pressedNow;
    firstCall = false;
    return;
  }

  if (pressedNow && !wasPressed && (millis() - lastPressMs > 200)) {
    isEnabled = !isEnabled;
    lastPressMs = millis();
    // Give the heartbeat timeout a fresh baseline so the wifi safety check
    // doesn't immediately undo the toggle (only relevant once heartbeats
    // have started arriving).
    extern unsigned long lastHeartbeatMs;
    if (lastHeartbeatMs != 0) lastHeartbeatMs = millis();
    Serial.print(">>> Power button — isEnabled now ");
    Serial.println(isEnabled ? "true" : "false");
  }
  wasPressed = pressedNow;
}

void loop() {
  // ── Forward obstacle / door check ─────────────────────────────────────
  // Every tick: if forward sensor sees something within OBSTACLE_STOP_CM,
  // stop motors and skip the body. Auto-resumes when the obstacle clears
  // (door opens → distance jumps above the threshold). Status is sent
  // exactly once per rising edge so the server can run its door handshake.
  // navState is NOT modified — the state machine just pauses for the tick.
  static bool prevObstacle    = false;   // previous tick's stable state
  static bool prevRawObstacle = false;   // previous tick's raw reading
  static bool stableObstacle  = false;   // debounced state used downstream

  const float distFwd     = getDistanceCM(SENSOR_FORWARD);
  lastForwardDistanceCm   = distFwd;
  const bool  rawObstacle = (distFwd >= 0.0f) && (distFwd < (float)OBSTACLE_STOP_CM);

  // 2-consecutive-readings debounce: a single-tick HC-SR04 flicker (OOR
  // glitch, off-axis echo) won't flip the gate. Only when the raw reading
  // disagrees with our stable state for two ticks in a row do we accept it.
  if (rawObstacle != stableObstacle && rawObstacle == prevRawObstacle) {
    stableObstacle = rawObstacle;
  }
  prevRawObstacle = rawObstacle;
  const bool obstacleNow = stableObstacle;

  if (obstacleNow && !prevObstacle) {
    sendStatus("obstacle_stop");
    Serial.println(">>> obstacle in front — holding");
  }
  if (!obstacleNow && prevObstacle) {
    // Door opened / robot moved / object cleared. Body resumes on this same
    // tick — the state machine's current tick re-commands motor speeds.
    sendStatus("obstacle_cleared");
    Serial.println(">>> obstacle cleared — resuming");
  }

  // Door-retry: if we're stuck at a closed door in either wall-follow phase,
  // resend the open-airlock request every DOOR_RETRY_INTERVAL_MS. The
  // baseline is set on the first paused tick so the server gets a quiet
  // window after the original request before any resend. Airlock A = exit
  // (NAV_WALL_FOLLOW), Airlock B = return (NAV_TUNNEL_B_WALL_FOLLOW).
  static unsigned long doorRetryDeadlineMs = 0;
  const bool atClosedDoor = obstacleNow &&
    (navState == NAV_WALL_FOLLOW || navState == NAV_TUNNEL_B_WALL_FOLLOW);
  if (atClosedDoor) {
    if (!prevObstacle) {
      doorRetryDeadlineMs = millis() + DOOR_RETRY_INTERVAL_MS;
    }
    if (millis() >= doorRetryDeadlineMs) {
      if (navState == NAV_WALL_FOLLOW) {
        Serial.println("[BASE] door still closed — resending openAirlockA");
        sendOpenAirlockA();
      } else {
        Serial.println("[BASE] door still closed — resending openAirlockB");
        sendOpenAirlockB();
      }
      doorRetryDeadlineMs = millis() + DOOR_RETRY_INTERVAL_MS;
    }
  }

  prevObstacle = obstacleNow;

  // ── Always-run pass ───────────────────────────────────────────────────
  // WiFi must run so the server can re-enable us. Serial commands must run
  // so debug tools work while disabled. Encoders and diagnostic prints are
  // passive (no motor activity) and useful for bench-testing without the
  // server.
  wifiLoop();
  checkPowerButton();
  handleSerialCommands();
  updateEncoders();
  updateHopHeading();           // no-ops when no hop is active
  readAndPrintIR();
  readAndPrintDistance();
  readAndPrintEncoders();
  readAndPrintPitch();
  readAndPrintLDR();

  // ── Safety gate ───────────────────────────────────────────────────────
  // Skip the body when disabled OR while an obstacle is in front. The
  // disabled path additionally collapses nav state via handleNavDisable
  // so it requires explicit re-engagement. The obstacle path just pauses
  // — navState is preserved and the body resumes as soon as the obstacle
  // clears, which is the door-opening case.
  if (!isEnabled) {
    motoron.setSpeedNow(LEFT_MOTOR,  0);
    motoron.setSpeedNow(RIGHT_MOTOR, 0);
    if (useStateMachine) handleNavDisable();
    delay(10);
    return;
  }
  // If obstacle also stop robot
  if (obstacleNow) {
    motoron.setSpeedNow(LEFT_MOTOR,  0);
    motoron.setSpeedNow(RIGHT_MOTOR, 0);
    // Base-return termination: hitting something in front while line-following
    // in the base ends the run (per the spec: "follow it until no more line or
    // we detect something in front and can't move").
    if (useStateMachine && navState == NAV_BASE_RETURN) {
      sendStatus("parked_obstacle");
      navState = NAV_PARKED;
    }
    delay(10);
    return;
  }

  // ── Enabled body ──────────────────────────────────────────────────────
  if (useStateMachine) {
    navigationUpdate();
  } else {
    // For testing purposes, in practice not used
    rfidLoop();
    applyMotorEnabled();
  }

  delay(10);
}
