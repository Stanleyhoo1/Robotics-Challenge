// ─────────────────────────────────────────
// Motors / IMU
// ─────────────────────────────────────────
LSM6 imu;
MotoronI2C motoron(MOTORON_ADDRESS, &Wire1);
float gyroZOffset = 0;

// ─────────────────────────────────────────
// Encoders (quadrature, one per rear wheel, interrupt-driven on Giga).
// Back-left  uses pins 22/23. Back-right uses pins 24/25.
// ─────────────────────────────────────────
volatile long encBL = 0;
volatile long encBR = 0;

static volatile bool lastBL_A = false, lastBL_B = false;
static volatile bool lastBR_A = false, lastBR_B = false;

// Quadrature decoders: one ISR per channel. A-edge sign is the inverse of
// B-edge sign for the same direction — that's what gives 4x decoding.
static void isr_BL_A() { bool a = digitalRead(ENC_BL_A), b = digitalRead(ENC_BL_B); if (a != lastBL_A) { encBL += (a == b) ?  1 : -1; lastBL_A = a; } }
static void isr_BL_B() { bool a = digitalRead(ENC_BL_A), b = digitalRead(ENC_BL_B); if (b != lastBL_B) { encBL += (a == b) ? -1 :  1; lastBL_B = b; } }
static void isr_BR_A() { bool a = digitalRead(ENC_BR_A), b = digitalRead(ENC_BR_B); if (a != lastBR_A) { encBR += (a == b) ? -1 :  1; lastBR_A = a; } }
static void isr_BR_B() { bool a = digitalRead(ENC_BR_A), b = digitalRead(ENC_BR_B); if (b != lastBR_B) { encBR += (a == b) ?  1 : -1; lastBR_B = b; } }

void encoderSetup() {
  pinMode(ENC_BL_A, INPUT_PULLUP); pinMode(ENC_BL_B, INPUT_PULLUP);
  pinMode(ENC_BR_A, INPUT_PULLUP); pinMode(ENC_BR_B, INPUT_PULLUP);

  // Seed last-state from current pin reads so the first edge isn't misread.
  lastBL_A = digitalRead(ENC_BL_A); lastBL_B = digitalRead(ENC_BL_B);
  lastBR_A = digitalRead(ENC_BR_A); lastBR_B = digitalRead(ENC_BR_B);

  attachInterrupt(digitalPinToInterrupt(ENC_BL_A), isr_BL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_BL_B), isr_BL_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_BR_A), isr_BR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_BR_B), isr_BR_B, CHANGE);

  Serial.println("Encoders ready (interrupt-driven, 4x quadrature, 1 per side).");

  // Restore previously-locked ticks/cm if we have one saved. If not, the
  // in-run calibration path takes over and saves on lock.
  if (!loadCalibFromKV()) {
    Serial.print("No saved encoder calib; using fallback ticks_per_cm=");
    Serial.println(calibTicksPerCm);
  }
}

// No-op under the current interrupt-driven encoder design — counters are
// updated by ISRs. Kept so the main-loop call site is uniform and so a
// future swap to polling only needs a body change here.
void updateEncoders() { }

// Pitch debug print — used to identify which accelerometer axis points
// forward on this build so the ramp feed-forward picks the right axis & sign.
// Prints both possible pitch computations; whichever swings most on a ramp
// is the forward axis. ±sign on the ramp tells us the sign convention.
void readAndPrintPitch() {
  if (!showPitch) return;
  static unsigned long lastPrintMs = 0;
  if (millis() - lastPrintMs < 200) return;
  lastPrintMs = millis();
  imu.read();
  float pitchX = atan2((float)imu.a.x, (float)imu.a.z) * RAD_TO_DEG;
  float pitchY = atan2((float)imu.a.y, (float)imu.a.z) * RAD_TO_DEG;
  Serial.print("accel ax="); Serial.print(imu.a.x);
  Serial.print(" ay=");      Serial.print(imu.a.y);
  Serial.print(" az=");      Serial.print(imu.a.z);
  Serial.print("  pitchX="); Serial.print(pitchX, 1);
  Serial.print("deg pitchY="); Serial.print(pitchY, 1);
  Serial.println("deg");
}

// Diagnostic print, rate-limited to ~5 Hz so the serial monitor stays readable.
void readAndPrintEncoders() {
  if (!showEncoders) return;
  static unsigned long lastPrintMs = 0;
  if (millis() - lastPrintMs < 200) return;
  lastPrintMs = millis();
  Serial.print("enc BL="); Serial.print(encBL);
  Serial.print(" BR=");    Serial.println(encBR);
}

long leftTicks()     { return encBL; }
long rightTicks()    { return encBR; }
long straightTicks() { return (leftTicks() + rightTicks()) / 2; }

void encoderResetHop() {
  noInterrupts();
  encBL = encBR = 0;
  interrupts();
}

// ─────────────────────────────────────────
// Self-calibrating ticks/cm
// Locks after CALIB_MIN_SAMPLES accepted straight-hop samples.
// ─────────────────────────────────────────
float calibTicksPerCm = TICKS_PER_CM_FALLBACK;
bool  calibLocked     = false;
int   calibSamples    = 0;
float calibSum        = 0;

static const char* CALIB_KV_KEY = "/kv/calib_tpc";

// Persist the locked ticks/cm so the robot doesn't recalibrate from scratch
// on every boot. Mirrors the IR sensor's kvstore pattern in ir_sensor.ino.
void saveCalibToKV() {
  int rc = kv_set(CALIB_KV_KEY, &calibTicksPerCm, sizeof(calibTicksPerCm), 0);
  if (rc == 0) {
    Serial.print("Encoder calib saved to KV: ticks_per_cm=");
    Serial.println(calibTicksPerCm);
  } else {
    Serial.print("Encoder calib KV save FAILED, rc=");
    Serial.println(rc);
  }
}

// Try to restore a previously-locked value. Returns true on success and sets
// calibTicksPerCm + calibLocked. Sanity-checks the loaded value so a corrupt
// KV entry falls back to the in-run calibration path instead of poisoning
// dead reckoning with garbage.
bool loadCalibFromKV() {
  float    saved;
  size_t   actualSize = 0;
  int rc = kv_get(CALIB_KV_KEY, &saved, sizeof(saved), &actualSize);
  if (rc != 0)                    return false;
  if (actualSize != sizeof(float)) return false;
  if (saved < 0.1f || saved > 1000.0f) {
    Serial.print("Encoder calib KV entry out of range (");
    Serial.print(saved);
    Serial.println("); ignoring.");
    return false;
  }
  calibTicksPerCm = saved;
  calibLocked     = true;
  Serial.print("Encoder calib loaded from KV: ticks_per_cm=");
  Serial.println(calibTicksPerCm);
  return true;
}

// Erase the saved value and reset the accumulator so the next run
// recalibrates from scratch. Used by the `recalib` serial command.
void clearCalibKV() {
  int rc = kv_remove(CALIB_KV_KEY);
  calibLocked     = false;
  calibSamples    = 0;
  calibSum        = 0.0f;
  calibTicksPerCm = TICKS_PER_CM_FALLBACK;
  Serial.print("Encoder calib reset (KV rc=");
  Serial.print(rc);
  Serial.print(", in-memory ticks_per_cm=");
  Serial.print(calibTicksPerCm);
  Serial.println(")");
}

float hopDistanceCm() {
  return (float)straightTicks() / calibTicksPerCm;
}

bool nearNextNode() {
  return hopDistanceCm() >= GRID_SPACING_CM * NODE_ARRIVAL_FRACTION;
}

void calibRecordHop(GridPos prev, GridPos curr) {
  if (calibLocked) return;

  int manhattan = abs(prev.row - curr.row) + abs(prev.col - curr.col);
  if (manhattan != 1) return;

  long ticks = straightTicks();
  if (ticks <= 0) return;

  float candidate = (float)ticks / GRID_SPACING_CM;

  if (calibSamples > 0) {
    float mean = calibSum / calibSamples;
    float dev  = fabsf(candidate - mean) / mean;
    if (dev > CALIB_OUTLIER_PCT) return;
  }

  calibSum += candidate;
  calibSamples++;

  if (calibSamples >= CALIB_MIN_SAMPLES) {
    calibTicksPerCm = calibSum / calibSamples;
    calibLocked = true;
    char msg[64];
    snprintf(msg, sizeof(msg), "calib_locked ticks_per_cm=%.3f", calibTicksPerCm);
    sendStatus(msg);
    saveCalibToKV();
  }
}

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

    motoron.setSpeedNow(LEFT_MOTOR,   direction * speed);
    motoron.setSpeedNow(RIGHT_MOTOR, -direction * speed);
    delay(5);
  }

  motoron.setSpeedNow(LEFT_MOTOR,  0);
  motoron.setSpeedNow(RIGHT_MOTOR, 0);
  Serial.println("Turn complete.");
}

// ─────────────────────────────────────────
// Hop-heading correction for no-line-zone dead-reckoning.
// Mirrors the gyro-integration math in turnDegrees() but accumulates
// into a hop-relative heading and feeds a proportional motor correction.
// ─────────────────────────────────────────
float hopHeadingDeg = 0.0f;
static unsigned long lastHopHeadingMicros = 0;
static bool hopHeadingActive = false;

void resetHopHeading() {
  hopHeadingDeg = 0.0f;
  lastHopHeadingMicros = micros();
  hopHeadingActive = true;
}

void endHopHeading() {
  hopHeadingActive = false;
}

void updateHopHeading() {
  if (!hopHeadingActive) return;
  imu.read();
  unsigned long now = micros();
  float dt = (now - lastHopHeadingMicros) / 1000000.0f;
  lastHopHeadingMicros = now;
  float gz = -((imu.g.z - gyroZOffset) * GYRO_SENS);
  hopHeadingDeg += gz * dt;
}

void applyHeadingCorrection() {
  float correction = HEADING_KP * hopHeadingDeg;
  int leftSpeed  = FORWARD_SPEED - (int)correction;
  int rightSpeed = FORWARD_SPEED + (int)correction;
  leftSpeed  = constrain(leftSpeed,  0, 800);
  rightSpeed = constrain(rightSpeed, 0, 800);
  motoron.setSpeedNow(LEFT_MOTOR,  scaleSpeed(leftSpeed));
  motoron.setSpeedNow(RIGHT_MOTOR, scaleSpeed(rightSpeed));
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
  motoron.setSpeedNow(LEFT_MOTOR,  speed);
  motoron.setSpeedNow(RIGHT_MOTOR, speed);
  delay(ms);
  motoron.setSpeedNow(LEFT_MOTOR,  0);
  motoron.setSpeedNow(RIGHT_MOTOR, 0);
  Serial.println("Stopped.");
}

// ─────────────────────────────────────────
// Gate motor output on server enable signal
// Called from main loop() to avoid main.ino
// referencing `motoron` before it's declared
// ─────────────────────────────────────────
void applyMotorEnabled() {
  if (isEnabled) {
    motoron.setSpeedNow(LEFT_MOTOR,  scaleSpeed(FORWARD_SPEED));
    motoron.setSpeedNow(RIGHT_MOTOR, scaleSpeed(FORWARD_SPEED));
  } else {
    motoron.setSpeedNow(LEFT_MOTOR,  0);
    motoron.setSpeedNow(RIGHT_MOTOR, 0);
  }
}

int scaleSpeed(int speed) {
  return (int)(speed * ((float)MOTOR_VOLTAGE / INPUT_VOLTAGE));
}
