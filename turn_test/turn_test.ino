// Standalone turn-in-place test sketch.
// Mirrors turnDegrees() from main/motors.ino so any speed / slow-zone tuning
// here transfers 1:1 to the production firmware. Use to find a TURN_SPEED
// that keeps the wheels from slipping (= bot drifting off centre during a turn).

#include <Wire.h>
#include <Motoron.h>
#include <LSM6.h>

MotoronI2C motoron(16, &Wire1);
const uint8_t LEFT_MOTOR  = 1;
const uint8_t RIGHT_MOTOR = 2;

LSM6 imu;
const float GYRO_SENS = 0.00875f;
float gyroZOffset = 0;

// ---- Encoders (one quadrature encoder per rear wheel). Pins match config.h. ----
const int ENC_BL_A = 22;
const int ENC_BL_B = 23;
const int ENC_BR_A = 14;
const int ENC_BR_B = 15;

volatile long encBL = 0;
volatile long encBR = 0;
volatile bool lastBL_A = false, lastBL_B = false;
volatile bool lastBR_A = false, lastBR_B = false;

// Same ISR pattern as main/motors.ino — see that file for the (a==b) derivation.
void isr_BL_A() { bool a = digitalRead(ENC_BL_A), b = digitalRead(ENC_BL_B); if (a != lastBL_A) { encBL += (a == b) ?  1 : -1; lastBL_A = a; } }
void isr_BL_B() { bool a = digitalRead(ENC_BL_A), b = digitalRead(ENC_BL_B); if (b != lastBL_B) { encBL += (a == b) ? -1 :  1; lastBL_B = b; } }
void isr_BR_A() { bool a = digitalRead(ENC_BR_A), b = digitalRead(ENC_BR_B); if (a != lastBR_A) { encBR += (a == b) ? -1 :  1; lastBR_A = a; } }
void isr_BR_B() { bool a = digitalRead(ENC_BR_A), b = digitalRead(ENC_BR_B); if (b != lastBR_B) { encBR += (a == b) ?  1 : -1; lastBR_B = b; } }

void encoderSetup() {
  pinMode(ENC_BL_A, INPUT_PULLUP); pinMode(ENC_BL_B, INPUT_PULLUP);
  pinMode(ENC_BR_A, INPUT_PULLUP); pinMode(ENC_BR_B, INPUT_PULLUP);
  lastBL_A = digitalRead(ENC_BL_A); lastBL_B = digitalRead(ENC_BL_B);
  lastBR_A = digitalRead(ENC_BR_A); lastBR_B = digitalRead(ENC_BR_B);
  attachInterrupt(digitalPinToInterrupt(ENC_BL_A), isr_BL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_BL_B), isr_BL_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_BR_A), isr_BR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_BR_B), isr_BR_B, CHANGE);
}

void resetEncoders() {
  noInterrupts();
  encBL = 0;
  encBR = 0;
  interrupts();
}

// Pure-rotation diagnostic. For a clean in-place turn |BL| should equal |BR|
// and their signs should be opposite. Sum != 0 = translation drift. Magnitude
// gap = one side scrubbed more / stalled more than the other.
void printEncoderResult() {
  long bl = encBL, br = encBR;
  Serial.print(F("  enc BL=")); Serial.print(bl);
  Serial.print(F(" BR="));      Serial.print(br);
  Serial.print(F(" |BL|-|BR|=")); Serial.print(abs(bl) - abs(br));
  Serial.print(F(" sum="));     Serial.print(bl + br);
  Serial.println(bl + br == 0 ? F(" (pure rotation)") : F(" (translation drift)"));
}

// Runtime-tunable. Defaults match main/config.h.
int   turnSpeed    = 800;
int   minTurnSpeed = 400;
float slowZoneDeg  = 20.0f;

void calibrateGyro() {
  Serial.println(F("Calibrating gyro, keep still..."));
  long sum = 0;
  for (int i = 0; i < 500; i++) { imu.read(); sum += imu.g.z; delay(2); }
  gyroZOffset = sum / 500.0f;
  Serial.println(F("Gyro calibrated."));
}

// Identical control loop to main/motors.ino turnDegrees(). Returns the
// gyro-integrated rotation actually achieved (signed) for over/undershoot
// diagnostics.
float turnDegrees(float targetDegrees) {
  motoron.clearMotorFault(LEFT_MOTOR);
  motoron.clearMotorFault(RIGHT_MOTOR);
  resetEncoders();

  float accumulated = 0;
  unsigned long lastTime = micros();
  int direction = (targetDegrees > 0) ? 1 : -1;
  const float SLOW_ZONE = min(slowZoneDeg, abs(targetDegrees) * 0.3f);

  unsigned long startMs = millis();
  while (true) {
    imu.read();
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0f;
    lastTime = now;
    float gz = -((imu.g.z - gyroZOffset) * GYRO_SENS);
    accumulated += gz * dt;

    float remaining = abs(targetDegrees) - abs(accumulated);
    if (remaining <= 0) break;

    int speed = (remaining < SLOW_ZONE)
      ? map((long)remaining, 0, (long)SLOW_ZONE, minTurnSpeed, turnSpeed)
      : turnSpeed;

    motoron.setSpeedNow(LEFT_MOTOR,   direction * speed);
    motoron.setSpeedNow(RIGHT_MOTOR, -direction * speed);
    delay(5);

    // Safety: if a stall or wild gyro drift keeps `remaining` positive forever,
    // bail rather than spinning the wheels indefinitely.
    if (millis() - startMs > 8000) {
      Serial.println(F("turnDegrees TIMEOUT"));
      break;
    }
  }
  motoron.setSpeedNow(LEFT_MOTOR,  0);
  motoron.setSpeedNow(RIGHT_MOTOR, 0);
  return accumulated;
}

// Constant-power variant: holds turnSpeed for the entire turn, no slow-zone
// ramp. Dodges the mid-turn stall caused by power dropping below the level
// needed to overcome wheel stiction. Expect some overshoot since there's no
// deceleration before the target — that's the trade.
float turnDegreesConstant(float targetDegrees) {
  motoron.clearMotorFault(LEFT_MOTOR);
  motoron.clearMotorFault(RIGHT_MOTOR);
  resetEncoders();

  float accumulated = 0;
  unsigned long lastTime = micros();
  int direction = (targetDegrees > 0) ? 1 : -1;

  unsigned long startMs = millis();
  while (true) {
    imu.read();
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0f;
    lastTime = now;
    float gz = -((imu.g.z - gyroZOffset) * GYRO_SENS);
    accumulated += gz * dt;

    if (abs(accumulated) >= abs(targetDegrees)) break;

    motoron.setSpeedNow(LEFT_MOTOR,   direction * turnSpeed);
    motoron.setSpeedNow(RIGHT_MOTOR, -direction * turnSpeed);
    delay(5);

    if (millis() - startMs > 8000) {
      Serial.println(F("turnDegreesConstant TIMEOUT"));
      break;
    }
  }
  motoron.setSpeedNow(LEFT_MOTOR,  0);
  motoron.setSpeedNow(RIGHT_MOTOR, 0);
  return accumulated;
}

// Asymmetric per-side power. Use this to compensate for asymmetric stiction:
// give the sticky side more power than the free side until |BL| == |BR|.
// Direction follows targetDegrees sign; speeds are magnitudes for each side.
float turnDegreesAsymmetric(float targetDegrees, int leftSpd, int rightSpd) {
  motoron.clearMotorFault(LEFT_MOTOR);
  motoron.clearMotorFault(RIGHT_MOTOR);
  resetEncoders();

  float accumulated = 0;
  unsigned long lastTime = micros();
  int direction = (targetDegrees > 0) ? 1 : -1;

  unsigned long startMs = millis();
  while (true) {
    imu.read();
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0f;
    lastTime = now;
    float gz = -((imu.g.z - gyroZOffset) * GYRO_SENS);
    accumulated += gz * dt;

    if (abs(accumulated) >= abs(targetDegrees)) break;

    motoron.setSpeedNow(LEFT_MOTOR,   direction * leftSpd);
    motoron.setSpeedNow(RIGHT_MOTOR, -direction * rightSpd);
    delay(5);

    if (millis() - startMs > 8000) {
      Serial.println(F("turnDegreesAsymmetric TIMEOUT"));
      break;
    }
  }
  motoron.setSpeedNow(LEFT_MOTOR,  0);
  motoron.setSpeedNow(RIGHT_MOTOR, 0);
  return accumulated;
}

// Four equal turns in sequence. After all four the bot should face its
// original direction AND sit in the same spot — visual check on the floor
// for translation drift, printed total for gyro over/undershoot.
void squareTest(float perTurnDeg) {
  Serial.print(F("--- SQUARE TEST: 4 x "));
  Serial.print(perTurnDeg, 1);
  Serial.println(F("°. Mark start pose on the floor for drift check ---"));
  float total = 0;
  for (int i = 0; i < 4; i++) {
    Serial.print(F("  turn "));
    Serial.print(i + 1);
    Serial.print(F(": commanded "));
    Serial.print(perTurnDeg, 1);
    Serial.print(F("° -> measured "));
    float actual = turnDegrees(perTurnDeg);
    total += actual;
    Serial.print(actual, 2);
    Serial.println(F("°"));
    delay(500);
  }
  float ideal = (perTurnDeg > 0) ? 360.0f : -360.0f;
  Serial.print(F("--- TOTAL measured = "));
  Serial.print(total, 2);
  Serial.print(F("° (ideal "));
  Serial.print(ideal, 0);
  Serial.print(F("°, error = "));
  Serial.print(total - ideal, 2);
  Serial.println(F("°) ---"));
}

void printHelp() {
  Serial.println(F("--- TURN TEST ---"));
  Serial.println(F("  <deg>      turn that many degrees, ramped (positive=right, negative=left)"));
  Serial.println(F("  c <deg>    constant-power turn (no slow zone) — best for stall resistance"));
  Serial.println(F("  a <L> <R>  asymmetric 90° right turn at left/right speeds (e.g. 'a 500 800')"));
  Serial.println(F("  A <L> <R>  asymmetric 90° left turn at left/right speeds"));
  Serial.println(F("  s          square test: 4 x 90° right"));
  Serial.println(F("  S          square test: 4 x -90° (left)"));
  Serial.println(F("  t <spd>    set TURN_SPEED (e.g. 't 400')"));
  Serial.println(F("  m <spd>    set MIN_TURN_SPEED (slow-zone floor)"));
  Serial.println(F("  z <deg>    set slow-zone width in degrees"));
  Serial.println(F("  e          print last encoder result"));
  Serial.println(F("  g          recalibrate gyro"));
  Serial.println(F("  ?          this help"));
  Serial.print  (F("  current: turn=")); Serial.print(turnSpeed);
  Serial.print  (F(" min="));            Serial.print(minTurnSpeed);
  Serial.print  (F(" slow="));           Serial.print(slowZoneDeg, 1);
  Serial.println(F("°"));
}

void serviceSerial() {
  while (Serial.available()) {
    int peek = Serial.peek();

    // Bare number on its own = turn command. Lets the user just type "90".
    if (peek == '-' || (peek >= '0' && peek <= '9')) {
      float deg = Serial.parseFloat();
      if (deg != 0.0f) {
        Serial.print(F("turning "));
        Serial.print(deg, 1);
        Serial.println(F("°..."));
        float actual = turnDegrees(deg);
        Serial.print(F("done. measured "));
        Serial.print(actual, 2);
        Serial.println(F("°"));
        printEncoderResult();
      }
      continue;
    }

    char ch = Serial.read();
    switch (ch) {
      case 's': squareTest( 90.0f); break;
      case 'S': squareTest(-90.0f); break;
      case 'c': {
        float deg = Serial.parseFloat();
        if (deg != 0.0f) {
          Serial.print(F("constant-power turning "));
          Serial.print(deg, 1);
          Serial.println(F("°..."));
          float actual = turnDegreesConstant(deg);
          Serial.print(F("done. measured "));
          Serial.print(actual, 2);
          Serial.print(F("° (overshoot = "));
          Serial.print(actual - deg, 2);
          Serial.println(F("°)"));
          printEncoderResult();
        } else {
          Serial.println(F("usage: c <deg>  (e.g. 'c 90' or 'c -45')"));
        }
        break;
      }
      case 'a':
      case 'A': {
        // a <leftSpd> <rightSpd> -> +90° turn with asymmetric per-side power
        // A <leftSpd> <rightSpd> -> -90° turn with same speeds
        // Iterate until printEncoderResult shows |BL| ~= |BR|.
        float deg = (ch == 'A') ? -90.0f : 90.0f;
        int leftSpd  = Serial.parseInt();
        int rightSpd = Serial.parseInt();
        if (leftSpd > 0 && leftSpd <= 800 && rightSpd > 0 && rightSpd <= 800) {
          Serial.print(F("asymmetric turn: L=")); Serial.print(leftSpd);
          Serial.print(F(" R="));                 Serial.print(rightSpd);
          Serial.print(F(" deg="));               Serial.println(deg, 1);
          float actual = turnDegreesAsymmetric(deg, leftSpd, rightSpd);
          Serial.print(F("done. measured "));
          Serial.print(actual, 2);
          Serial.println(F("°"));
          printEncoderResult();
        } else {
          Serial.println(F("usage: a <leftSpd> <rightSpd>   (90° right with asymmetric power)"));
          Serial.println(F("       A <leftSpd> <rightSpd>   (90° left with asymmetric power)"));
        }
        break;
      }
      case 'e': printEncoderResult(); break;
      case 't': {
        int v = Serial.parseInt();
        if (v > 0 && v <= 800) {
          turnSpeed = v;
          Serial.print(F("TURN_SPEED = ")); Serial.println(turnSpeed);
        } else {
          Serial.println(F("usage: t <speed>  (0 < speed <= 800)"));
        }
        break;
      }
      case 'm': {
        int v = Serial.parseInt();
        if (v > 0 && v <= 800) {
          minTurnSpeed = v;
          Serial.print(F("MIN_TURN_SPEED = ")); Serial.println(minTurnSpeed);
        } else {
          Serial.println(F("usage: m <speed>  (0 < speed <= 800)"));
        }
        break;
      }
      case 'z': {
        float v = Serial.parseFloat();
        if (v > 0.0f && v < 90.0f) {
          slowZoneDeg = v;
          Serial.print(F("slow zone = ")); Serial.print(slowZoneDeg, 1); Serial.println(F("°"));
        } else {
          Serial.println(F("usage: z <deg>  (0 < deg < 90)"));
        }
        break;
      }
      case 'g': calibrateGyro(); break;
      case '?': printHelp();     break;
      case '\n': case '\r': case ' ': break;
      default: break;
    }
  }
}

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
    Serial.println(F("IMU not found!"));
    while (1);
  }
  imu.enableDefault();

  encoderSetup();
  calibrateGyro();
  Serial.println(F("Setup done."));
  printHelp();
}

void loop() {
  serviceSerial();
}
