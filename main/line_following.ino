#include <kvstore_global_api.h>
#include <Wire.h>
#include <Motoron.h>

// ---- IR Sensor ----
const int sensorPins[9] = {2, 3, 4, 5, 6, 7, 8, 9, 10};
const int ctrlPin = 12;
const int SensorCount = 9;
const unsigned int timeout = 2500;

uint16_t minValues[9];
uint16_t maxValues[9];
uint16_t lastPosition = 0;

// ---- Motors ----
MotoronI2C motoron(16, &Wire1);
const uint8_t LEFT_MOTOR  = 1;
const uint8_t RIGHT_MOTOR = 2;

const int BASE_SPEED = 300;
const float KP = 0.05;

// ---- Encoders ----
const int ENC_A_LEFT  = 18;
const int ENC_B_LEFT  = 19;
const int ENC_A_RIGHT = 20;
const int ENC_B_RIGHT = 21;

volatile long encoderLeft  = 0;
volatile long encoderRight = 0;
volatile bool lastALeft  = false;
volatile bool lastBLeft  = false;
volatile bool lastARight = false;
volatile bool lastBRight = false;

void encoderISR_A_Left() {
  bool a = digitalRead(ENC_A_LEFT);
  bool b = digitalRead(ENC_B_LEFT);
  if (a != lastALeft) {
    encoderLeft += (a == b) ? -1 : 1;
    lastALeft = a;
  }
}

void encoderISR_B_Left() {
  bool a = digitalRead(ENC_A_LEFT);
  bool b = digitalRead(ENC_B_LEFT);
  if (b != lastBLeft) {
    encoderLeft += (a == b) ? 1 : -1;
    lastBLeft = b;
  }
}

void encoderISR_A_Right() {
  bool a = digitalRead(ENC_A_RIGHT);
  bool b = digitalRead(ENC_B_RIGHT);
  if (a != lastARight) {
    encoderRight += (a == b) ? -1 : 1;
    lastARight = a;
  }
}

void encoderISR_B_Right() {
  bool a = digitalRead(ENC_A_RIGHT);
  bool b = digitalRead(ENC_B_RIGHT);
  if (b != lastBRight) {
    encoderRight += (a == b) ? 1 : -1;
    lastBRight = b;
  }
}

// ---- IR Calibration ----
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

// ---- Setup ----
void setup() {
  Serial.begin(115200);
  uint32_t startWait = millis();
  while (!Serial && millis() - startWait < 3000);

  // Motors
  Wire1.begin();
  motoron.reinitialize();
  motoron.disableCommandTimeout();
  motoron.clearResetFlag();

  // Encoders
  pinMode(ENC_A_LEFT,  INPUT_PULLUP);
  pinMode(ENC_B_LEFT,  INPUT_PULLUP);
  pinMode(ENC_A_RIGHT, INPUT_PULLUP);
  pinMode(ENC_B_RIGHT, INPUT_PULLUP);
  lastALeft  = digitalRead(ENC_A_LEFT);
  lastBLeft  = digitalRead(ENC_B_LEFT);
  lastARight = digitalRead(ENC_A_RIGHT);
  lastBRight = digitalRead(ENC_B_RIGHT);
  attachInterrupt(digitalPinToInterrupt(ENC_A_LEFT),  encoderISR_A_Left,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_LEFT),  encoderISR_B_Left,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A_RIGHT), encoderISR_A_Right, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_RIGHT), encoderISR_B_Right, CHANGE);

  // IR sensors
  pinMode(ctrlPin, OUTPUT);
  digitalWrite(ctrlPin, HIGH);

  if (!loadCalibration()) {
    Serial.println("No saved calibration found, running calibration...");
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

  Serial.println("Starting line following...");
  delay(1000);
}

// ---- Loop ----
void loop() {
  long avg = 0;
  long sum = 0;

  for (int i = 0; i < SensorCount; i++) {
    unsigned int rawVal = readPrivate(sensorPins[i]);
    int calibratedVal = map(rawVal, minValues[i], maxValues[i], 0, 1000);
    calibratedVal = constrain(calibratedVal, 0, 1000);
    avg += (long)calibratedVal * (i * 1000);
    sum += calibratedVal;
  }

  if (sum > 200) {
    lastPosition = avg / sum;
  } else {
    if (lastPosition < (SensorCount - 1) * 1000 / 2) {
      lastPosition = 0;
    } else {
      lastPosition = (SensorCount - 1) * 1000;
    }
  }

  // Error: 0 = far left, 4000 = centre, 8000 = far right
  int error = lastPosition - 4000;
  int correction = (int)(KP * error);

  int leftSpeed  = constrain(BASE_SPEED + correction, -800, 800);
  int rightSpeed = constrain(BASE_SPEED - correction, -800, 800);

  motoron.setSpeedNow(LEFT_MOTOR,  leftSpeed);
  motoron.setSpeedNow(RIGHT_MOTOR, rightSpeed);

  // Debug
  noInterrupts();
  long leftCount  = encoderLeft  / 2;
  long rightCount = encoderRight / 2;
  interrupts();

  Serial.print("Pos: ");   Serial.print(lastPosition);
  Serial.print("  Err: "); Serial.print(error);
  Serial.print("  L: ");   Serial.print(leftSpeed);
  Serial.print("  R: ");   Serial.print(rightSpeed);
  Serial.print("  EncL: "); Serial.print(leftCount);
  Serial.print("  EncR: "); Serial.println(rightCount);
}