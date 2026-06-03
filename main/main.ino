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
  // Three-tier detection (mirrors obstacle_avoid_test):
  //   < CRASH_STOP_CM (4 cm) or OOR-after-close → immediate: enter
  //     NAV_AVOID_OBSTACLE; navAvoidObstacleTick backs up first.
  //   < OBSTACLE_STOP_CM (8 cm), debounced → enter NAV_AVOID_OBSTACLE
  //     in NAV_ARENA_NAV, or stop-and-wait in door / base states.
  //   < OBSTACLE_AVOID_CM (20 cm) → handled inside navArenaTick (soft
  //     mid-hop trigger) and at-tag peek in navAtTagTick.
  // Status is sent exactly once per rising edge.
  static bool prevObstacle    = false;
  static bool prevRawObstacle = false;
  static bool stableObstacle  = false;

  const float distFwd   = getDistanceCM(SENSOR_FORWARD);
  lastForwardDistanceCm = distFwd;

  // OOR persistence: HC-SR04 can return -1 (OOR) below ~2 cm because the
  // echo arrives before the trigger window closes. If we saw something within
  // OBSTACLE_AVOID_CM recently and the sensor just went OOR, treat it as
  // still close (the obstacle is getting closer, not disappearing).
  static unsigned long lastCloseReadingMs = 0;
  if (distFwd >= 0.0f && distFwd < (float)OBSTACLE_AVOID_CM) {
    lastCloseReadingMs = millis();
  }
  const bool oor_persisting = (distFwd < 0.0f &&
                               lastCloseReadingMs != 0 &&
                               millis() - lastCloseReadingMs < (unsigned long)OOR_PERSISTENCE_MS);

  // Crash: reading below CRASH_STOP_CM, or OOR-after-close. No debounce —
  // react on the first tick.
  const bool crashNow = (distFwd >= 0.0f && distFwd < (float)CRASH_STOP_CM) ||
                        oor_persisting;

  // Stop: debounced signal for the 8 cm tier (also catches OOR persistence).
  const bool rawObstacle = (distFwd >= 0.0f && distFwd < (float)OBSTACLE_STOP_CM) ||
                           oor_persisting;

  // 2-consecutive-readings debounce for the stop tier.
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
    sendStatus("obstacle_cleared");
    Serial.println(">>> obstacle cleared — resuming");
  }

  // Door-retry: resend the open-airlock request while paused at a closed door.
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
  // ── Obstacle / avoidance gate ────────────────────────────────────────
  // Bypass entirely during revival (intentional contact) and while
  // navAvoidObstacleTick is running (it manages its own motors).
  const bool revivingNow      = useStateMachine &&
    (navState == NAV_REVIVING || navState == NAV_WAIT_REVIVE_REPLY);
  const bool avoidingObstacle = useStateMachine && navState == NAV_AVOID_OBSTACLE;

  if (!revivingNow && !avoidingObstacle) {
    // ── Crash tier (< CRASH_STOP_CM or OOR-after-close): no debounce ──
    // Immediately enter NAV_AVOID_OBSTACLE. The tick function checks
    // lastForwardDistanceCm and backs up before replanning.
    if (crashNow && useStateMachine && navState == NAV_ARENA_NAV) {
      motoron.setSpeedNow(LEFT_MOTOR,  0);
      motoron.setSpeedNow(RIGHT_MOTOR, 0);
      Serial.print("[CRASH] ");
      if (distFwd < 0.0f) Serial.print("OOR-persist");
      else { Serial.print(distFwd, 1); Serial.print(" cm"); }
      Serial.println(" — entering NAV_AVOID_OBSTACLE (crash)");
      lastCloseReadingMs = 0;   // stale after replan+turn; reset so it can't retrigger
      navState = NAV_AVOID_OBSTACLE;
      // Fall through — let navigationUpdate run navAvoidObstacleTick this tick.

    // ── Stop tier (< OBSTACLE_STOP_CM, debounced) ─────────────────────
    } else if (obstacleNow) {
      // In NAV_ARENA_NAV: trigger avoidance rather than a bare stop.
      if (useStateMachine && navState == NAV_ARENA_NAV) {
        motoron.setSpeedNow(LEFT_MOTOR,  0);
        motoron.setSpeedNow(RIGHT_MOTOR, 0);
        Serial.print("[OBSTACLE] "); Serial.print(distFwd, 1);
        Serial.println(" cm — entering NAV_AVOID_OBSTACLE");
        lastCloseReadingMs = 0;
        navState = NAV_AVOID_OBSTACLE;
        // Fall through — let navigationUpdate run navAvoidObstacleTick this tick.

      } else {
        // Door / tunnel / base states: stop-and-wait (existing behaviour).
        motoron.setSpeedNow(LEFT_MOTOR,  0);
        motoron.setSpeedNow(RIGHT_MOTOR, 0);
        if (useStateMachine && navState == NAV_BASE_RETURN) {
          sendStatus("parked_obstacle");
          navState = NAV_PARKED;
        }
        delay(10);
        return;
      }
    }
  }

  //useStateMachine = false;

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
