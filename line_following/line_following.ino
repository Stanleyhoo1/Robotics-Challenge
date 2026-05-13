#include <kvstore_global_api.h>
#include <Wire.h>
#include <Motoron.h>
#include <LSM6.h>

// ---- IR Sensor ----
const int sensorPins[9] = {30, 31, 32, 33, 34, 35, 36, 37, 38};
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

const int BASE_SPEED = 500;
const float KP = 0.1;
const float KP_AGGRESSIVE = 0.15;
const int AGGRESSIVE_DURATION = 2000;

float currentKP = KP;
unsigned long junctionExitTime = 0;

// ---- IMU ----
LSM6 imu;
const float GYRO_SENS = 0.00875;
const int TURN_SPEED = 500;
float gyroZOffset = 0;

// ---- Junction ----
// 0 = straight, 1 = turn right, -1 = turn left
const int junctionActions[] = {-1, 1, -1};
int junctionCount = 0;
bool inJunction = false;

// ---- Line State ----
enum LineState {
  LINE_NORMAL,
  LINE_LOST,
  LINE_JUNCTION_LEFT,
  LINE_JUNCTION_RIGHT,
  LINE_JUNCTION_BOTH,
};

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

// ---- IMU ----
void calibrateGyro() {
  Serial.println("Calibrating gyro, keep still...");
  long sum = 0;
  for (int i = 0; i < 500; i++) {
    imu.read();
    sum += imu.g.z;
    delay(2);
  }
  gyroZOffset = sum / 500.0;
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

    int speed;
    if (remaining < SLOW_ZONE) {
      speed = map(remaining, 0, SLOW_ZONE, MIN_SPEED, TURN_SPEED);
    } else {
      speed = TURN_SPEED;
    }

    motoron.setSpeedNow(LEFT_MOTOR,   direction * speed);
    motoron.setSpeedNow(RIGHT_MOTOR, -direction * speed);

    delay(5);
  }

  motoron.setSpeedNow(LEFT_MOTOR,  0);
  motoron.setSpeedNow(RIGHT_MOTOR, 0);
}

// ---- Line State Detection ----
LineState getLineState(uint16_t* calibratedVals, long sum) {
  bool leftActive  = false;
  bool rightActive = false;
  int activeCount  = 0;

  for (int i = 0; i < SensorCount; i++) {
    if (calibratedVals[i] > 500) {
      activeCount++;
      if (i <= 2) leftActive = true;
      if (i >= 6) rightActive = true;
    }
  }

  if (sum < 200) return LINE_LOST;
  if (leftActive && rightActive) return LINE_JUNCTION_BOTH;
  if (leftActive && activeCount >= 7) return LINE_JUNCTION_LEFT;
  if (rightActive && activeCount >= 7) return LINE_JUNCTION_RIGHT;
  return LINE_NORMAL;
}

// ---- Sensor Sum ----
long getSensorSum() {
  long sum = 0;
  for (int i = 0; i < SensorCount; i++) {
    unsigned int rawVal = readPrivate(sensorPins[i]);
    int calibratedVal = map(rawVal, minValues[i], maxValues[i], 0, 1000);
    sum += constrain(calibratedVal, 0, 1000);
  }
  return sum;
}

// ---- Spin Until Line ----
void spinUntilLine(int direction) {
  int spinLeft  = (direction == -1) ? -BASE_SPEED : BASE_SPEED;
  int spinRight = (direction == -1) ?  BASE_SPEED : -BASE_SPEED;

  motoron.setSpeedNow(LEFT_MOTOR,  spinLeft);
  motoron.setSpeedNow(RIGHT_MOTOR, spinRight);

  float accumulated = 0;
  unsigned long lastTime = micros();
  const float MIN_ROTATION = 20.0;
  const float MAX_ROTATION = 180.0;

  while (abs(accumulated) < MAX_ROTATION) {
    imu.read();
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0;
    lastTime = now;
    float gz = -((imu.g.z - gyroZOffset) * GYRO_SENS);
    accumulated += gz * dt;

    if (abs(accumulated) > MIN_ROTATION) {
      bool lowIndexActive  = false; // sensors 0-2 (pin 1 side = right)
      bool highIndexActive = false; // sensors 6-8 (pin 9 side = left)

      for (int i = 0; i < SensorCount; i++) {
        unsigned int rawVal = readPrivate(sensorPins[i]);
        int calibratedVal = map(rawVal, minValues[i], maxValues[i], 0, 1000);
        calibratedVal = constrain(calibratedVal, 0, 1000);
        if (calibratedVal > 500) {
          if (i <= 2) lowIndexActive = true;  // pin 1 side
          if (i >= 6) highIndexActive = true; // pin 9 side
        }
      }

      // Turning right: stop when only low index sensors (pin 1 side) see line
      // Turning left:  stop when only high index sensors (pin 9 side) see line
      if (direction == 1  && lowIndexActive  && !highIndexActive) break;
      if (direction == -1 && highIndexActive && !lowIndexActive)  break;
    }

    delay(5);
  }

  motoron.setSpeedNow(LEFT_MOTOR,  0);
  motoron.setSpeedNow(RIGHT_MOTOR, 0);

  motoron.setSpeedNow(LEFT_MOTOR,  BASE_SPEED);
  motoron.setSpeedNow(RIGHT_MOTOR, BASE_SPEED);
  delay(150);
  motoron.setSpeedNow(LEFT_MOTOR,  0);
  motoron.setSpeedNow(RIGHT_MOTOR, 0);
}

// ---- Junction Handler ----
void handleJunction() {
  if (inJunction) return;
  inJunction = true;

  if (junctionCount >= 3) {
    motoron.setSpeedNow(LEFT_MOTOR,  0);
    motoron.setSpeedNow(RIGHT_MOTOR, 0);
    return;
  }

  int action = junctionActions[junctionCount];
  junctionCount++;

  Serial.print("Junction encountered: ");
  Serial.println(junctionCount);

  // Drive forward to centre on junction
  motoron.setSpeedNow(LEFT_MOTOR,  BASE_SPEED);
  motoron.setSpeedNow(RIGHT_MOTOR, BASE_SPEED);
  delay(400);

  if (action != 0) {
    spinUntilLine(action);
  }

  delay(100);
  currentKP = KP_AGGRESSIVE;
  junctionExitTime = millis();
  inJunction = false;
}

// ---- Setup ----
void setup() {
  Serial.begin(115200);
  uint32_t startWait = millis();
  while (!Serial && millis() - startWait < 3000);

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

  if (!loadCalibration()) {
    Serial.println("No saved calibration, running calibration...");
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

  calibrateGyro();
  Serial.println("Setup done.");
  delay(1000);
}

// ---- Loop ----
void loop() {
  long avg = 0;
  long sum = 0;
  uint16_t calibratedVals[SensorCount];

  for (int i = 0; i < SensorCount; i++) {
    unsigned int rawVal = readPrivate(sensorPins[i]);
    int calibratedVal = map(rawVal, minValues[i], maxValues[i], 0, 1000);
    calibratedVal = constrain(calibratedVal, 0, 1000);
    calibratedVals[i] = calibratedVal;
    avg += (long)calibratedVal * (i * 1000);
    sum += calibratedVal;
  }

  // Ramp KP back to normal after junction
  if (currentKP == KP_AGGRESSIVE && millis() - junctionExitTime > AGGRESSIVE_DURATION) {
    currentKP = KP;
  }

  LineState state = getLineState(calibratedVals, sum);

  switch (state) {
    case LINE_NORMAL:
      if (sum > 200) {
        lastPosition = avg / sum;
        int error = lastPosition - 4000;
        int correction = (int)(currentKP * error);
        motoron.setSpeedNow(LEFT_MOTOR,  constrain(BASE_SPEED + correction, -800, 800));
        motoron.setSpeedNow(RIGHT_MOTOR, constrain(BASE_SPEED - correction, -800, 800));
      }
      break;

    case LINE_LOST:
      motoron.setSpeedNow(LEFT_MOTOR,  0);
      motoron.setSpeedNow(RIGHT_MOTOR, 0);
      break;

    case LINE_JUNCTION_LEFT:
    case LINE_JUNCTION_RIGHT:
    case LINE_JUNCTION_BOTH:
      handleJunction();
      break;
  }
}