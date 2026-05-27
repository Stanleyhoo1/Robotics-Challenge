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

void loop() {
  // Encoder + heading updates first — they're cheap and the rest of the
  // loop expects fresh tick counts and an integrated hop heading.
  updateEncoders();
  updateHopHeading();

  wifiLoop();
  readAndPrintIR();
  readAndPrintDistance();

  if (useStateMachine) {
    // State machine owns motor + RFID control; legacy helpers stay out of the way.
    navigationUpdate();
  } else {
    rfidLoop();
    applyMotorEnabled();
  }

  handleSerialCommands();
  delay(10);
}
