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
bool  isEnabled            = false;
bool  useStateMachine      = false;   // false = legacy junctionActions[] + rfidLoop path
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

void loop() {
  // ── Forward obstacle / door check ─────────────────────────────────────
  // Every tick: if forward sensor sees something within OBSTACLE_STOP_CM,
  // stop motors and skip the body. Auto-resumes when the obstacle clears
  // (door opens → distance jumps above the threshold). Status is sent
  // exactly once per rising edge so the server can run its door handshake.
  // navState is NOT modified — the state machine just pauses for the tick.
  static bool prevObstacle = false;

  const float distFwd     = getDistanceCM(SENSOR_FORWARD);
  lastForwardDistanceCm   = distFwd;
  const bool  obstacleNow = (distFwd >= 0.0f) && (distFwd < (float)OBSTACLE_STOP_CM);

  if (obstacleNow && !prevObstacle) {
    sendStatus("obstacle_stop");
  }
  prevObstacle = obstacleNow;

  // ── Always-run pass ───────────────────────────────────────────────────
  // WiFi must run so the server can re-enable us. Serial commands must run
  // so debug tools work while disabled. Encoders and diagnostic prints are
  // passive (no motor activity) and useful for bench-testing without the
  // server.
  wifiLoop();
  handleSerialCommands();
  updateEncoders();
  updateHopHeading();           // no-ops when no hop is active
  readAndPrintIR();
  readAndPrintDistance();
  readAndPrintEncoders();

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
  if (obstacleNow) {
    motoron.setSpeedNow(LEFT_MOTOR,  0);
    motoron.setSpeedNow(RIGHT_MOTOR, 0);
    delay(10);
    return;
  }

  // ── Enabled body ──────────────────────────────────────────────────────
  if (useStateMachine) {
    navigationUpdate();
  } else {
    rfidLoop();
    applyMotorEnabled();
  }

  delay(10);
}
