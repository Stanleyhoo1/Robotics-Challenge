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
  Serial.begin(115200);
  uint32_t startWait = millis();
  while (millis() - startWait < 3000);

  // IR
  irSetup();

  // Ultrasonic
  distanceSetup();

  // I2C
  Wire.begin();
  Wire1.begin();

  // RFID
  rfidSetup();

  // Servo
  servoSetup();

  // IMU + Motors
  motorsSetup();

  Serial.println("\nReady. Commands:");
  Serial.println("  <number>      → turn that many degrees (e.g. 90 or -90)");
  Serial.println("  forward       → move forward 3s at default speed");
  Serial.println("  forward <spd> → move forward 3s at given speed (e.g. forward 500)");
  Serial.println("  ir            → toggle IR readings");
  Serial.println("  distance      → toggle distance readings");
  Serial.println("  c             → recalibrate IR sensors");
  delay(1000);
}

// =========================================
// Loop
// =========================================
void loop() {
  readAndPrintIR();
  readAndPrintDistance();

  if (rfidLoop()) {
    sweepTo(MAX_ANGLE, MIN_ANGLE);
    sweepTo(MIN_ANGLE, MAX_ANGLE);
  }

  handleSerialCommands();
  delay(20);
}
