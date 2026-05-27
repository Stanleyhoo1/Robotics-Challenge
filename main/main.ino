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
bool showIR          = false;
bool showDistance    = false;
bool showEncoders    = false;
bool isEnabled       = false;
bool useStateMachine = false;   // false = legacy junctionActions[] + rfidLoop path

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
  // When disabled, everything below is skipped. Motors are forced to 0 and
  // navigation state (if any) is collapsed once on the disable transition.
  if (!isEnabled) {
    motoron.setSpeedNow(LEFT_MOTOR,  0);
    motoron.setSpeedNow(RIGHT_MOTOR, 0);
    if (useStateMachine) handleNavDisable();
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
