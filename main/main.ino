#include <Wire.h>
#include <Servo.h>
#include "MFRC522_I2C.h"
#include <Motoron.h>
#include <LSM6.h>
#include <kvstore_global_api.h>

// ─────────────────────────────────────────
// RFID
// ─────────────────────────────────────────
MFRC522_I2C rfid(0x28, 255);

// ─────────────────────────────────────────
// Servo
// ─────────────────────────────────────────
Servo myServo;
const int SERVO_PIN  = 9;
const int MIN_ANGLE  = 60;
const int MAX_ANGLE  = 160;
const int STEP_DELAY = 5;

// ─────────────────────────────────────────
// Motors / IMU
// ─────────────────────────────────────────
LSM6 imu;
MotoronI2C motoron(16, &Wire1);
const uint8_t LEFT_MOTOR  = 1;
const uint8_t RIGHT_MOTOR = 2;
const int TURN_SPEED    = 500;
const int FORWARD_SPEED = 400;
const float GYRO_SENS   = 0.00875;
float gyroZOffset       = 0;

// ─────────────────────────────────────────
// IR Sensor Array
// ─────────────────────────────────────────
const int sensorPins[9]    = {30, 31, 32, 33, 34, 35, 36, 37, 38};
const int ctrlPin          = 12;
const int SensorCount      = 9;
const unsigned int timeout = 2500;
uint16_t minValues[9];
uint16_t maxValues[9];
uint16_t lastPosition      = 0;

// ─────────────────────────────────────────
// Ultrasonic
// ─────────────────────────────────────────
const int TRIG_PIN = 40;
const int ECHO_PIN = 41;

// ─────────────────────────────────────────
// Display toggles
// ─────────────────────────────────────────
bool showIR       = true;
bool showDistance = true;

// =========================================
// IR Functions
// =========================================
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
  Serial.println("Calibration saved.");
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
  Serial.println("Calibration loaded.");
  return true;
}

void runCalibration() {
  for (int i = 0; i < SensorCount; i++) {
    minValues[i] = timeout;
    maxValues[i] = 0;
  }
  Serial.println("--- CALIBRATION STARTING (10 SECONDS) ---");
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

void readAndPrintIR() {
  if (!showIR) return;
  long avg = 0, sum = 0;
  for (int i = 0; i < SensorCount; i++) {
    unsigned int rawVal = readPrivate(sensorPins[i]);
    int calibratedVal = map(rawVal, minValues[i], maxValues[i], 0, 1000);
    calibratedVal = constrain(calibratedVal, 0, 1000);
    avg += (long)calibratedVal * (i * 1000);
    sum += calibratedVal;
    Serial.print(calibratedVal);
    Serial.print("\t");
  }
  if (sum > 200) {
    lastPosition = avg / sum;
  } else {
    lastPosition = (lastPosition < (SensorCount - 1) * 1000 / 2) ? 0 : (SensorCount - 1) * 1000;
  }
  Serial.print("| Pos: ");
  Serial.println(lastPosition);
}

// =========================================
// Ultrasonic Functions
// =========================================
float getDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return -1;
  return duration / 58.0;
}

// =========================================
// Servo Functions
// =========================================
void sweepTo(int from, int to) {
  int step = (to > from) ? 1 : -1;
  for (int angle = from; angle != to; angle += step) {
    myServo.write(angle);
    delay(STEP_DELAY);
  }
  myServo.write(to);
}

// =========================================
// Motor Functions
// =========================================
void calibrateGyro() {
  Serial.println("Calibrating gyro, keep still...");
  long sum = 0;
  for (int i = 0; i < 500; i++) {
    imu.read();
    sum += imu.g.z;
    delay(2);
  }
  gyroZOffset = sum / 500.0;
  Serial.println("Gyro calibrated.");
}

void turnDegrees(float targetDegrees) {
  motoron.clearMotorFault(LEFT_MOTOR);
  motoron.clearMotorFault(RIGHT_MOTOR);

  float accumulated = 0;
  unsigned long lastTime = micros();
  int direction = (targetDegrees > 0) ? 1 : -1;
  const float SLOW_ZONE = min(20.0f, abs(targetDegrees) * 0.3f);
  const int MIN_SPEED = 150;

  while (true) {
    imu.read();
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0;
    lastTime = now;
    float gz = -((imu.g.z - gyroZOffset) * GYRO_SENS);
    accumulated += gz * dt;

    float remaining = abs(targetDegrees) - abs(accumulated);
    if (remaining <= 0) break;

    int speed = (remaining < SLOW_ZONE)
      ? map(remaining, 0, SLOW_ZONE, MIN_SPEED, TURN_SPEED)
      : TURN_SPEED;

    motoron.setSpeedNow(LEFT_MOTOR,   direction * speed);
    motoron.setSpeedNow(RIGHT_MOTOR, -direction * speed);
    delay(5);
  }

  motoron.setSpeedNow(LEFT_MOTOR,  0);
  motoron.setSpeedNow(RIGHT_MOTOR, 0);
  Serial.println("Turn complete.");
}

void moveForward(int ms, int speed = FORWARD_SPEED) {
  Serial.print("Moving forward for ");
  Serial.print(ms / 1000);
  Serial.print("s at speed ");
  Serial.println(speed);
  motoron.setSpeedNow(LEFT_MOTOR,  speed);
  motoron.setSpeedNow(RIGHT_MOTOR, speed);
  delay(ms);
  motoron.setSpeedNow(LEFT_MOTOR,  0);
  motoron.setSpeedNow(RIGHT_MOTOR, 0);
  Serial.println("Stopped.");
}

// =========================================
// Setup
// =========================================
void setup() {
  Serial.begin(115200);
  uint32_t startWait = millis();
  while (!Serial && millis() - startWait < 3000);

  // IR
  pinMode(ctrlPin, OUTPUT);
  digitalWrite(ctrlPin, HIGH);

  if (!loadCalibration()) {
    Serial.println("No saved calibration, running now...");
    runCalibration();
  } else {
    Serial.println("Send 'c' within 3 seconds to recalibrate...");
    unsigned long start = millis();
    while (millis() - start < 3000) {
      if (Serial.available() && Serial.read() == 'c') {
        runCalibration();
        break;
      }
    }
  }

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.println("Ultrasonic ready.");

  // I2C
  Wire.begin();
  Wire1.begin();

  // RFID
  rfid.PCD_Init();
  Serial.println("RFID ready.");

  // Servo
  myServo.attach(SERVO_PIN, 750, 2250);
  sweepTo(myServo.read(), MAX_ANGLE);
  Serial.println("Servo at 160.");

  // IMU
  imu.setBus(&Wire1);
  if (!imu.init()) {
    Serial.println("IMU not found!");
    while (1);
  }
  imu.enableDefault();

  // Motoron
  motoron.reinitialize();
  motoron.disableCommandTimeout();
  motoron.clearResetFlag();

  calibrateGyro();

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
  // --- IR Sensors ---
  readAndPrintIR();

  // --- Ultrasonic ---
  if (showDistance) {
    float distance = getDistanceCM();
    Serial.print("Dist: ");
    if (distance < 0) Serial.println("OOR");
    else { Serial.print(distance); Serial.println(" cm"); }
  }

  // --- RFID ---
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    Serial.print("Card UID: ");
    for (byte i = 0; i < rfid.uid.size; i++) {
      Serial.print(rfid.uid.uidByte[i] < 0x10 ? "0" : "");
      Serial.print(rfid.uid.uidByte[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    rfid.PICC_HaltA();
    sweepTo(MAX_ANGLE, MIN_ANGLE);
    sweepTo(MIN_ANGLE, MAX_ANGLE);
  }

  // --- Serial Commands ---
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.equalsIgnoreCase("ir")) {
      showIR = !showIR;
      Serial.print("IR readings ");
      Serial.println(showIR ? "ON" : "OFF");

    } else if (input.equalsIgnoreCase("distance")) {
      showDistance = !showDistance;
      Serial.print("Distance readings ");
      Serial.println(showDistance ? "ON" : "OFF");

    } else if (input.equalsIgnoreCase("c")) {
      runCalibration();

    } else if (input.startsWith("forward")) {
      int speed = FORWARD_SPEED;
      int spaceIdx = input.indexOf(' ');
      if (spaceIdx != -1) {
        speed = input.substring(spaceIdx + 1).toInt();
      }
      moveForward(3000, speed);

    } else {
      float degrees = input.toFloat();
      if (degrees != 0) {
        Serial.print("Turning ");
        Serial.print(degrees);
        Serial.println(" degrees...");
        turnDegrees(degrees);
      } else {
        Serial.println("Unknown command.");
      }
    }
  }

  delay(20);
}