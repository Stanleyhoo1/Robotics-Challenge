// ─────────────────────────────────────────
// Motors / IMU
// ─────────────────────────────────────────
LSM6 imu;
MotoronI2C motoron(MOTORON_ADDRESS, &Wire1);
float gyroZOffset = 0;

// ─────────────────────────────────────────
// Encoders (quadrature, interrupt-driven on Giga)
// ─────────────────────────────────────────
volatile long encBL = 0;
volatile long encFL = 0;
volatile long encBR = 0;
volatile long encFR = 0;

static volatile bool lastBL_A = false, lastBL_B = false;
static volatile bool lastFL_A = false, lastFL_B = false;
static volatile bool lastBR_A = false, lastBR_B = false;
static volatile bool lastFR_A = false, lastFR_B = false;

// Quadrature decoders: one ISR per channel. A-edge sign is the inverse of
// B-edge sign for the same direction — that's what gives 4x decoding.
static void isr_BL_A() { bool a = digitalRead(ENC_BL_A), b = digitalRead(ENC_BL_B); if (a != lastBL_A) { encBL += (a == b) ? -1 :  1; lastBL_A = a; } }
static void isr_BL_B() { bool a = digitalRead(ENC_BL_A), b = digitalRead(ENC_BL_B); if (b != lastBL_B) { encBL += (a == b) ?  1 : -1; lastBL_B = b; } }
static void isr_FL_A() { bool a = digitalRead(ENC_FL_A), b = digitalRead(ENC_FL_B); if (a != lastFL_A) { encFL += (a == b) ? -1 :  1; lastFL_A = a; } }
static void isr_FL_B() { bool a = digitalRead(ENC_FL_A), b = digitalRead(ENC_FL_B); if (b != lastFL_B) { encFL += (a == b) ?  1 : -1; lastFL_B = b; } }
static void isr_BR_A() { bool a = digitalRead(ENC_BR_A), b = digitalRead(ENC_BR_B); if (a != lastBR_A) { encBR += (a == b) ? -1 :  1; lastBR_A = a; } }
static void isr_BR_B() { bool a = digitalRead(ENC_BR_A), b = digitalRead(ENC_BR_B); if (b != lastBR_B) { encBR += (a == b) ?  1 : -1; lastBR_B = b; } }
static void isr_FR_A() { bool a = digitalRead(ENC_FR_A), b = digitalRead(ENC_FR_B); if (a != lastFR_A) { encFR += (a == b) ? -1 :  1; lastFR_A = a; } }
static void isr_FR_B() { bool a = digitalRead(ENC_FR_A), b = digitalRead(ENC_FR_B); if (b != lastFR_B) { encFR += (a == b) ?  1 : -1; lastFR_B = b; } }

void encoderSetup() {
  pinMode(ENC_BL_A, INPUT_PULLUP); pinMode(ENC_BL_B, INPUT_PULLUP);
  pinMode(ENC_FL_A, INPUT_PULLUP); pinMode(ENC_FL_B, INPUT_PULLUP);
  pinMode(ENC_BR_A, INPUT_PULLUP); pinMode(ENC_BR_B, INPUT_PULLUP);
  pinMode(ENC_FR_A, INPUT_PULLUP); pinMode(ENC_FR_B, INPUT_PULLUP);

  // Seed last-state from current pin reads so the first edge isn't misread.
  lastBL_A = digitalRead(ENC_BL_A); lastBL_B = digitalRead(ENC_BL_B);
  lastFL_A = digitalRead(ENC_FL_A); lastFL_B = digitalRead(ENC_FL_B);
  lastBR_A = digitalRead(ENC_BR_A); lastBR_B = digitalRead(ENC_BR_B);
  lastFR_A = digitalRead(ENC_FR_A); lastFR_B = digitalRead(ENC_FR_B);

  attachInterrupt(digitalPinToInterrupt(ENC_BL_A), isr_BL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_BL_B), isr_BL_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_FL_A), isr_FL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_FL_B), isr_FL_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_BR_A), isr_BR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_BR_B), isr_BR_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_FR_A), isr_FR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_FR_B), isr_FR_B, CHANGE);

  Serial.println("Encoders ready (interrupt-driven, 4x quadrature).");
}

// No-op under the current interrupt-driven encoder design — counters are
// updated by ISRs. Kept so the main-loop call site is uniform and so a
// future swap to polling only needs a body change here.
void updateEncoders() { }

// Diagnostic print, rate-limited to ~5 Hz so the serial monitor stays readable.
void readAndPrintEncoders() {
  if (!showEncoders) return;
  static unsigned long lastPrintMs = 0;
  if (millis() - lastPrintMs < 200) return;
  lastPrintMs = millis();
  Serial.print("enc BL="); Serial.print(encBL);
  Serial.print(" FL=");     Serial.print(encFL);
  Serial.print(" BR=");     Serial.print(encBR);
  Serial.print(" FR=");     Serial.println(encFR);
}

long leftTicks()     { return (encBL + encFL) / 2; }
long rightTicks()    { return (encBR + encFR) / 2; }
long straightTicks() { return (leftTicks() + rightTicks()) / 2; }

void encoderResetHop() {
  noInterrupts();
  encBL = encFL = encBR = encFR = 0;
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
// Hop-heading correction for right-half dead-reckoning.
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
