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

// ─────────────────────────────────────────
// Arena-absolute heading.
// Cardinal frame: 0°=NORTH, 90°=EAST, 180°=SOUTH, 270°=WEST. Seeded on the
// WALL_FOLLOW → ARENA_NAV transition (see wallFollow in navigation.ino) and
// integrated each main-loop tick via updateArenaHeading(). turnDegrees()
// bakes its measured rotation in on completion, so absolute heading stays
// correct across the blocking turn.
// Used by the pre-plant square-up to detect drift between the line PID's
// actual heading and robotFacing's expected cardinal — line-follow keeps us
// laterally on the line but can leave us a few degrees skewed.
// Defined ahead of turnDegrees() so it can reference the static state below.
// ─────────────────────────────────────────
float arenaHeadingDeg = 0.0f;
static unsigned long lastArenaHeadingMicros = 0;
static bool arenaHeadingActive = false;

static inline void wrapHeading360(float& h) {
  while (h >= 360.0f) h -= 360.0f;
  while (h <    0.0f) h += 360.0f;
}

void startArenaHeading(float initialDeg) {
  arenaHeadingDeg = initialDeg;
  wrapHeading360(arenaHeadingDeg);
  lastArenaHeadingMicros = micros();
  arenaHeadingActive = true;
}

void updateArenaHeading() {
  if (!arenaHeadingActive) return;
  imu.read();
  unsigned long now = micros();
  float dt = (now - lastArenaHeadingMicros) / 1000000.0f;
  lastArenaHeadingMicros = now;
  // Discard the first interval after a long pause (disable, blocking turn
  // that didn't bake in, etc.) — we'd otherwise integrate a ~0 gz over a
  // huge dt; harmless but pollutes the trace.
  if (dt > 1.0f) return;
  float gz = -((imu.g.z - gyroZOffset) * GYRO_SENS);
  arenaHeadingDeg += gz * dt;
  wrapHeading360(arenaHeadingDeg);
}

// Signed delta in (-180, 180] from current heading to the expected cardinal
// for `expected`. Positive = robot has rotated CW past the cardinal, so
// turnDegrees(-error) squares it back up.
float arenaHeadingError(Facing expected) {
  const float expectedDeg = ((int)expected) * 90.0f;
  float err = arenaHeadingDeg - expectedDeg;
  while (err >  180.0f) err -= 360.0f;
  while (err <= -180.0f) err += 360.0f;
  return err;
}

void turnDegrees(float targetDegrees) {
  motoron.clearMotorFault(LEFT_MOTOR);
  motoron.clearMotorFault(RIGHT_MOTOR);

  float accumulated = 0;
  unsigned long lastTime = micros();
  int direction = (targetDegrees > 0) ? 1 : -1;

  // Per-side speed picked by which wheel is going forward vs backward, and
  // by which direction we're turning. Right and left turns use separate
  // constants so motor-strength asymmetry can be compensated per direction.
  // Right turn (direction>0): LEFT forward, RIGHT backward.
  // Left turn  (direction<0): LEFT backward, RIGHT forward.
  int leftSpeed  = (direction > 0) ? RIGHT_TURN_FORWARD_SPEED  : LEFT_TURN_BACKWARD_SPEED;
  int rightSpeed = (direction > 0) ? RIGHT_TURN_BACKWARD_SPEED : LEFT_TURN_FORWARD_SPEED;

  // Per-direction overshoot trim — subtract a few degrees from the target so
  // the integrator hits its break sooner when one direction tends to coast
  // farther. Tune via {RIGHT,LEFT}_TURN_TRIM_DEG in config.h.
  const float trimDeg   = (direction > 0) ? RIGHT_TURN_TRIM_DEG : LEFT_TURN_TRIM_DEG;
  const float stopMagDeg = fabsf(targetDegrees) - trimDeg;

  while (true) {
    imu.read();
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0;
    lastTime = now;
    float gz = -((imu.g.z - gyroZOffset) * GYRO_SENS);
    accumulated += gz * dt;

    if (fabsf(accumulated) >= stopMagDeg) break;

    // Stop the turn early once a line is centred under the IR array. We
    // require both (a) enough total signal that a line is actually present
    // and (b) the weighted position is within ±1 sensor of centre — that way
    // we don't bail when the line is only grazing one edge of the array,
    // which would dump the line follower a huge initial correction.
    // TURN_LINE_CHECK_MIN_DEG (e.g. 75°) grace period prevents re-locking on
    // the original line or stopping while we're still sweeping through it.
    // In the no-line zone the sum stays well below IR_MIN_LINE_SUM so this
    // is a no-op there.
    if (fabsf(accumulated) > TURN_LINE_CHECK_MIN_DEG) {
      uint16_t cv[IR_SENSOR_COUNT];
      long avg, sum;
      readSensors(cv, avg, sum);
      if (sum >= IR_MIN_LINE_SUM) {
        long position = avg / sum;
        if (labs(position - LINE_CENTER) < 1000) {
          Serial.print("Turn: line centred at pos=");
          Serial.print(position);
          Serial.println(", stopping early.");
          break;
        }
      }
    }

    // Kill-switch responsiveness: service WiFi (heartbeat / emergency) and
    // poll the power button so either can stop the turn mid-action.
    wifiLoop();
    checkPowerButton();
    if (!isEnabled) {
      motoron.setSpeedNow(LEFT_MOTOR,  0);
      motoron.setSpeedNow(RIGHT_MOTOR, 0);
      Serial.println("Turn aborted: !isEnabled");
      return;
    }

    motoron.setSpeedNow(LEFT_MOTOR,   direction * leftSpeed);
    motoron.setSpeedNow(RIGHT_MOTOR, -direction * rightSpeed);
    delay(5);
  }

  motoron.setSpeedNow(LEFT_MOTOR,  0);
  motoron.setSpeedNow(RIGHT_MOTOR, 0);

  // Bake the measured rotation into the arena-absolute heading so the next
  // updateArenaHeading() doesn't double-count the turn. Resync the timestamp
  // since the main loop didn't tick during this blocking call.
  if (arenaHeadingActive) {
    arenaHeadingDeg += accumulated;
    wrapHeading360(arenaHeadingDeg);
    lastArenaHeadingMicros = micros();
  }
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
  int leftSpeed  = constrain(BASE_SPEED - (int)correction, 0, 800);
  int rightSpeed = constrain(BASE_SPEED + (int)correction, 0, 800);
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

  // Poll for kill switch instead of one big delay(ms) — otherwise the bot
  // can't be stopped mid-move.
  unsigned long start = millis();
  while (millis() - start < (unsigned long)ms) {
    wifiLoop();
    checkPowerButton();
    if (!isEnabled) {
      motoron.setSpeedNow(LEFT_MOTOR,  0);
      motoron.setSpeedNow(RIGHT_MOTOR, 0);
      Serial.println("moveForward aborted: !isEnabled");
      return;
    }
    delay(5);
  }

  motoron.setSpeedNow(LEFT_MOTOR,  0);
  motoron.setSpeedNow(RIGHT_MOTOR, 0);
  Serial.println("Stopped.");
}

// ─────────────────────────────────────────
// Gate motor output on server enable signal
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
