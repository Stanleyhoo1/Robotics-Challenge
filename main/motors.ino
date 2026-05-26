// ─────────────────────────────────────────
// Motors / IMU
// ─────────────────────────────────────────
LSM6 imu;
MotoronI2C motoron(MOTORON_ADDRESS, &Wire1);
float gyroZOffset = 0;

void motorsSetup() {
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
}

void calibrateGyro() {
  Serial.println("Calibrating gyro, keep still...");
  long sum = 0;
  for (int i = 0; i < GYRO_CALIB_SAMPLES; i++) {
    imu.read();
    sum += imu.g.z;
    delay(2);
  }
  gyroZOffset = sum / (float)GYRO_CALIB_SAMPLES;
  Serial.println("Gyro calibrated.");
}

void turnDegrees(float targetDegrees) {
  motoron.clearMotorFault(LEFT_MOTOR);
  motoron.clearMotorFault(RIGHT_MOTOR);

  float accumulated = 0;
  unsigned long lastTime = micros();
  int direction = (targetDegrees > 0) ? 1 : -1;
  const float SLOW_ZONE = min(TURN_SLOW_ZONE_DEG, abs(targetDegrees) * 0.3f);

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
      ? map(remaining, 0, SLOW_ZONE, MIN_TURN_SPEED, TURN_SPEED)
      : TURN_SPEED;

    motoron.setSpeedNow(LEFT_MOTOR,   scaleSpeed(direction * speed));
    motoron.setSpeedNow(RIGHT_MOTOR,  scaleSpeed(-direction * speed));
    delay(5);
  }

  motoron.setSpeedNow(LEFT_MOTOR,  0);
  motoron.setSpeedNow(RIGHT_MOTOR, 0);
  Serial.println("Turn complete.");
}

// Two overloads avoids default-parameter cross-file prototype issues
void moveForward(int ms) {
  moveForward(ms, FORWARD_SPEED);
}

void moveForward(int ms, int speed) {
  Serial.print("Moving forward for ");
  Serial.print(ms / 1000);
  Serial.print("s at speed ");
  Serial.println(speed);
  motoron.setSpeedNow(LEFT_MOTOR,  scaleSpeed(speed));
  motoron.setSpeedNow(RIGHT_MOTOR, scaleSpeed(speed));
  delay(ms);
  motoron.setSpeedNow(LEFT_MOTOR,  0);
  motoron.setSpeedNow(RIGHT_MOTOR, 0);
  Serial.println("Stopped.");
}

int scaleSpeed(int speed) {
  return (int)(speed * ((float)MOTOR_VOLTAGE / INPUT_VOLTAGE));
}