#include <Wire.h>
#include <Motoron.h>
#include <LSM6.h>

LSM6 imu;
MotoronI2C motoron(16, &Wire1);

const uint8_t LEFT_MOTOR  = 1;
const uint8_t RIGHT_MOTOR = 2;
const int TURN_SPEED = 500;
const float GYRO_SENS = 0.00875;
float gyroZOffset = 0;

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

void setup() {
  Serial.begin(115200);
  uint32_t startWait = millis();
  while (!Serial && millis() - startWait < 3000);

  Wire1.begin();

  imu.setBus(&Wire1);
  if (!imu.init()) {
    Serial.println("IMU not found!");
    while (1);
  }
  imu.enableDefault();

  motoron.reinitialize();
  motoron.disableCommandTimeout();
  motoron.clearResetFlag();

  calibrateGyro();
  Serial.println("Setup done.");
}

void loop() {
  delay(1000);
  turnDegrees(90);
  delay(1000);
  turnDegrees(-90);
}