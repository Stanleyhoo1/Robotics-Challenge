#include <Wire.h>
#include <Servo.h>
#include "MFRC522_I2C.h"
#include <Motoron.h>
#include <LSM6.h>
#include <kvstore_global_api.h>
#include "config.h"

// ─────────────────────────────────────────
// Display toggles (used across modules)
// ─────────────────────────────────────────
bool showIR       = true;
bool showDistance = true;

// =========================================
// Setup
// =========================================
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
  wifiSetup();

  Serial.println("\nReady. Commands:");
  Serial.println("  <number>      → turn that many degrees (e.g. 90 or -90)");
  Serial.println("  forward       → move forward 3s at default speed");
  Serial.println("  forward <spd> → move forward 3s at given speed");
  Serial.println("  ir            → toggle IR readings");
  Serial.println("  distance      → toggle distance readings");
  Serial.println("  sensors       → toggle all sensor readings");
  Serial.println("  c             → recalibrate IR sensors");
  Serial.println("  <anything>    → forward to server");
  delay(1000);
}

// =========================================
// Loop
// =========================================
void loop() {
  wifiLoop();

  readAndPrintIR();
  readAndPrintDistance();

  if (rfidLoop()) {
    sweepTo(MAX_ANGLE, MIN_ANGLE);
    sweepTo(MIN_ANGLE, MAX_ANGLE);
  }

  handleSerialCommands();

  // Motor output gated on server enable signal
  if (isEnabled) {
    motoron.setSpeedNow(LEFT_MOTOR,  scaleSpeed(FORWARD_SPEED));
    motoron.setSpeedNow(RIGHT_MOTOR, scaleSpeed(FORWARD_SPEED));
  } else {
    motoron.setSpeedNow(LEFT_MOTOR,  0);
    motoron.setSpeedNow(RIGHT_MOTOR, 0);
  }

  delay(20);
}
