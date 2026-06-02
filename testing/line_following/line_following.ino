#include <kvstore_global_api.h>
#include <Wire.h>
#include <Motoron.h>
#include <LSM6.h>
#include <Servo.h>
#include "MFRC522_I2C.h"

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

const int BASE_SPEED = 300;
const float KP = 0.2;
const float KP_AGGRESSIVE = 0.2;
const int AGGRESSIVE_DURATION = 2000;

float currentKP = KP;
unsigned long junctionExitTime = 0;

// ---- Debug ----
bool          debugDiag       = false;
bool          paused          = false;
bool          searchFailed    = false;     // true after a lost-line search came up empty
float         runtimeKP       = -1.0f;     // <0 = use currentKP, >=0 = override
unsigned long lastDiagMs      = 0;
const unsigned long DIAG_INTERVAL_MS = 200; // ~5 Hz dump rate
const int     SEARCH_SPIN_SPEED      = 800; // slow enough for reliable line detection mid-spin
const float   SEARCH_SWEEP_DEG       = 90.0f;

// ---- Encoders (quadrature, one per rear wheel, ISR-driven) ----
const int   ENC_BL_A      = 22;
const int   ENC_BL_B      = 23;
const int   ENC_BR_A      = 14;
const int   ENC_BR_B      = 15;
const float TICKS_PER_CM  = 159.97f;  // measured: 2438 ticks over 6 in (15.24 cm)

volatile long encBL = 0;
volatile long encBR = 0;
volatile bool lastBL_A = false, lastBL_B = false;
volatile bool lastBR_A = false, lastBR_B = false;
bool showEnc = false;

// ---- Servo / RFID ----
const int    SERVO_PIN          = 9;
const int    SERVO_MIN_US       = 750;
const int    SERVO_MAX_US       = 2250;
const int    SERVO_MIN_ANGLE    = 60;
const int    SERVO_MAX_ANGLE    = 180;
const int    SERVO_STEP_DELAY_MS = 5;
const float  POST_TAG_FORWARD_CM = 4.0f;  // post-tag nudge AND lost-line forward nudge — both 5 cm
const uint8_t RFID_I2C_ADDR     = 0x28;
const uint8_t RFID_RESET_PIN    = 255;
const unsigned long RFID_SCAN_COOLDOWN_MS = 2000;

MFRC522_I2C   rfid(RFID_I2C_ADDR, RFID_RESET_PIN);
Servo         seedServo;
unsigned long lastRfidScanMs = 0;

// ---- IMU ----
LSM6 imu;
const float GYRO_SENS = 0.00875;
const int TURN_SPEED = 400;   // matches MIN_SPEED — slow zone becomes flat; powered wheels need extra torque to drag the two unpowered ones
float gyroZOffset = 0;

// ---- Junction ----
// 0 = straight, 1 = turn right, -1 = turn left
const int junctionActions[] = {-1, 0, 1};
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

// ---- Encoders (quadrature, ISR-driven, 4x decoding) ----
// One ISR per channel; sign uses (a == b) for A-edges, inverted for B-edges.
// Pattern copied from main/motors.ino — see that file for the derivation.
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
  Serial.println("Encoders ready.");
}

long  straightTicks() { return (encBL + encBR) / 2; }
float distanceCm()    { return (float)straightTicks() / TICKS_PER_CM; }
void  resetEncoders() { noInterrupts(); encBL = encBR = 0; interrupts(); }

// Plain encoder-gated forward drive. Blocks until cm covered.
void forwardCm(float cm) {
  resetEncoders();
  motoron.setSpeedNow(LEFT_MOTOR,  BASE_SPEED);
  motoron.setSpeedNow(RIGHT_MOTOR, BASE_SPEED);
  while (distanceCm() < cm) { delay(2); }
  motoron.setSpeedNow(LEFT_MOTOR,  0);
  motoron.setSpeedNow(RIGHT_MOTOR, 0);
}

// Forward-while-watching variant for the lost-line recovery: stops early if
// the IR array picks up a line during the nudge. Returns true on early line
// detection so the caller knows the search succeeded.
bool forwardCmCheckLine(float cm);  // forward decl — definition below uses lineSeen()

void turnDegrees(float targetDegrees) {
  motoron.clearMotorFault(LEFT_MOTOR);
  motoron.clearMotorFault(RIGHT_MOTOR);

  float accumulated = 0;
  unsigned long lastTime = micros();
  int direction = (targetDegrees > 0) ? 1 : -1;
  const float SLOW_ZONE = min(20.0f, abs(targetDegrees) * 0.3f);
  const int MIN_SPEED = 800;

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

// ---- Lost-line recovery ----
// Returns true if any sensor sees the line (sum threshold matches LINE_LOST).
bool lineSeen() {
  long sum = 0;
  for (int i = 0; i < SensorCount; i++) {
    unsigned int rawVal = readPrivate(sensorPins[i]);
    int v = constrain(map(rawVal, minValues[i], maxValues[i], 0, 1000), 0, 1000);
    sum += v;
  }
  return sum > 200;
}

// Forward-while-watching-for-line. Encoder-gated distance, polls IR every tick.
bool forwardCmCheckLine(float cm) {
  resetEncoders();
  motoron.setSpeedNow(LEFT_MOTOR,  BASE_SPEED);
  motoron.setSpeedNow(RIGHT_MOTOR, BASE_SPEED);
  while (distanceCm() < cm) {
    if (lineSeen()) {
      motoron.setSpeedNow(LEFT_MOTOR,  0);
      motoron.setSpeedNow(RIGHT_MOTOR, 0);
      return true;
    }
    delay(2);
  }
  motoron.setSpeedNow(LEFT_MOTOR,  0);
  motoron.setSpeedNow(RIGHT_MOTOR, 0);
  return false;
}

// Nudge forward ~5cm, then sweep 90° right and 90° left from centre looking
// for the line. Bails the instant a sensor sees it. If the full sweep fails,
// returns to centre, stops, and reports false. Blocking — fine because the
// only thing we do while the line is lost is this search.
bool searchForLine() {
  Serial.println("LINE LOST — nudging forward then sweeping R/L...");
  motoron.clearMotorFault(LEFT_MOTOR);
  motoron.clearMotorFault(RIGHT_MOTOR);

  // Phase 0: encoder-gated forward nudge POST_TAG_FORWARD_CM (~5 cm). The
  // line often disappears right at the start of a gap — pushing forward first
  // puts the array over the next segment without having to spin to find it.
  // Bails early if a sensor catches the line mid-nudge.
  if (forwardCmCheckLine(POST_TAG_FORWARD_CM)) {
    Serial.println("  found during forward nudge");
    return true;
  }

  float accumulated = 0;
  unsigned long lastTime = micros();

  // Phase 1: spin right (positive accumulation in our convention) up to +90°.
  motoron.setSpeedNow(LEFT_MOTOR,   SEARCH_SPIN_SPEED);
  motoron.setSpeedNow(RIGHT_MOTOR, -SEARCH_SPIN_SPEED);
  while (accumulated < SEARCH_SWEEP_DEG) {
    imu.read();
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0f;
    lastTime = now;
    accumulated += -((imu.g.z - gyroZOffset) * GYRO_SENS) * dt;
    if (lineSeen()) {
      motoron.setSpeedNow(LEFT_MOTOR,  0);
      motoron.setSpeedNow(RIGHT_MOTOR, 0);
      Serial.print("  found at +"); Serial.print(accumulated); Serial.println("°");
      return true;
    }
    delay(5);
  }

  // Phase 2: spin left through centre down to -90° — single continuous
  // sweep, covers the whole arc on the other side without re-traversing 0°.
  motoron.setSpeedNow(LEFT_MOTOR,  -SEARCH_SPIN_SPEED);
  motoron.setSpeedNow(RIGHT_MOTOR,  SEARCH_SPIN_SPEED);
  while (accumulated > -SEARCH_SWEEP_DEG) {
    imu.read();
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0f;
    lastTime = now;
    accumulated += -((imu.g.z - gyroZOffset) * GYRO_SENS) * dt;
    if (lineSeen()) {
      motoron.setSpeedNow(LEFT_MOTOR,  0);
      motoron.setSpeedNow(RIGHT_MOTOR, 0);
      Serial.print("  found at "); Serial.print(accumulated); Serial.println("°");
      return true;
    }
    delay(5);
  }

  // Phase 3: return to centre (no line check — we already swept this arc).
  motoron.setSpeedNow(LEFT_MOTOR,   SEARCH_SPIN_SPEED);
  motoron.setSpeedNow(RIGHT_MOTOR, -SEARCH_SPIN_SPEED);
  while (accumulated < 0.0f) {
    imu.read();
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0f;
    lastTime = now;
    accumulated += -((imu.g.z - gyroZOffset) * GYRO_SENS) * dt;
    delay(5);
  }

  motoron.setSpeedNow(LEFT_MOTOR,  0);
  motoron.setSpeedNow(RIGHT_MOTOR, 0);
  Serial.println("  no line found — stopped at centre");
  return false;
}

// ---- Servo / RFID plant ----
void servoSetup() {
  seedServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  seedServo.write(SERVO_MAX_ANGLE);
}

void sweepTo(int from, int to) {
  int step = (to > from) ? 1 : -1;
  for (int a = from; a != to; a += step) {
    seedServo.write(a);
    delay(SERVO_STEP_DELAY_MS);
  }
  seedServo.write(to);
}

void rfidSetup() {
  rfid.PCD_Init();
  Serial.println("RFID ready.");
}

// On hit: stop, nudge POST_TAG_FORWARD_CM forward (encoder-gated, no line
// check — we want the dispenser over the hole regardless of line state),
// sweep servo, set cooldown so the same tag doesn't immediately re-trigger.
// Returns true if a plant happened — caller skips the line-follow tick.
bool rfidPollAndPlant() {
  if (millis() - lastRfidScanMs < RFID_SCAN_COOLDOWN_MS) return false;
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) return false;

  char uid[32] = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
    char b[3];
    snprintf(b, sizeof(b), "%02X", rfid.uid.uidByte[i]);
    if (strlen(uid) + 2 < sizeof(uid)) strcat(uid, b);
  }
  rfid.PICC_HaltA();

  Serial.print("RFID tag: ");
  Serial.println(uid);

  motoron.setSpeedNow(LEFT_MOTOR,  0);
  motoron.setSpeedNow(RIGHT_MOTOR, 0);

  Serial.print("  nudging "); Serial.print(POST_TAG_FORWARD_CM, 1); Serial.println(" cm forward");
  forwardCm(POST_TAG_FORWARD_CM);

  Serial.println("  planting (servo sweep)");
  sweepTo(SERVO_MAX_ANGLE, SERVO_MIN_ANGLE);
  sweepTo(SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

  lastRfidScanMs = millis();
  return true;
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
  delay(300);

  if (action != 0) {
    spinUntilLine(action);
  }

  delay(100);
  currentKP = KP_AGGRESSIVE;
  junctionExitTime = millis();
  inJunction = false;
}

// ---- Debug helpers ----
void printDebugHelp() {
  Serial.println(F("--- DEBUG COMMANDS ---"));
  Serial.println(F("  d        toggle live diagnostic dump"));
  Serial.println(F("  m        motor balance test (3s straight, no line input)"));
  Serial.println(F("  k <val>  set KP at runtime (e.g. 'k 0.15')"));
  Serial.println(F("  s        pause (motors off, controller idle)"));
  Serial.println(F("  g        resume from pause"));
  Serial.println(F("  c        recalibrate IR"));
  Serial.println(F("  r        clear search-failed latch (retry lost-line search)"));
  Serial.println(F("  e        toggle encoder print"));
  Serial.println(F("  z        zero encoder counters"));
  Serial.println(F("  f <cm>   forward N cm (encoder-gated, no line check)"));
  Serial.println(F("  t        test servo sweep (max → min → max)"));
  Serial.println(F("  ?        print this help"));
}

void motorBalanceTest() {
  Serial.println(F("--- MOTOR BALANCE: 3s at BASE_SPEED, equal both sides ---"));
  Serial.println(F("Watch the bot. Straight = motors balanced. Curving = hardware bias."));
  motoron.setSpeedNow(LEFT_MOTOR,  BASE_SPEED);
  motoron.setSpeedNow(RIGHT_MOTOR, BASE_SPEED);
  delay(3000);
  motoron.setSpeedNow(LEFT_MOTOR,  0);
  motoron.setSpeedNow(RIGHT_MOTOR, 0);
  Serial.println(F("--- BALANCE TEST DONE ---"));
}

void serviceSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'd':
        debugDiag = !debugDiag;
        Serial.print(F("diag: "));
        Serial.println(debugDiag ? F("ON") : F("OFF"));
        break;
      case 'm':
        motorBalanceTest();
        break;
      case 'k': {
        float v = Serial.parseFloat();
        if (v > 0.0f && v < 5.0f) {
          runtimeKP = v;
          Serial.print(F("KP override = "));
          Serial.println(v, 4);
        } else {
          Serial.println(F("KP value out of range (0, 5)"));
        }
        break;
      }
      case 's':
        paused = true;
        motoron.setSpeedNow(LEFT_MOTOR,  0);
        motoron.setSpeedNow(RIGHT_MOTOR, 0);
        Serial.println(F("paused"));
        break;
      case 'g':
        paused = false;
        Serial.println(F("resumed"));
        break;
      case 'c':
        motoron.setSpeedNow(LEFT_MOTOR,  0);
        motoron.setSpeedNow(RIGHT_MOTOR, 0);
        runCalibration();
        break;
      case 'e':
        showEnc = !showEnc;
        Serial.print(F("enc print: "));
        Serial.println(showEnc ? F("ON") : F("OFF"));
        break;
      case 'z':
        resetEncoders();
        Serial.println(F("encoders zeroed"));
        break;
      case 'f': {
        float fcm = Serial.parseFloat();
        if (fcm > 0.0f && fcm < 200.0f) {
          Serial.print(F("forward "));
          Serial.print(fcm, 2);
          Serial.println(F(" cm..."));
          forwardCm(fcm);
          Serial.print(F("done. actual="));
          Serial.print(distanceCm(), 2);
          Serial.println(F(" cm"));
        } else {
          Serial.println(F("usage: f <cm>  (0 < cm < 200)"));
        }
        break;
      }
      case 't':
        Serial.println(F("servo sweep..."));
        sweepTo(SERVO_MAX_ANGLE, SERVO_MIN_ANGLE);
        sweepTo(SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
        Serial.println(F("done"));
        break;
      case 'r':
        searchFailed = false;
        Serial.println(F("search-failed latch cleared"));
        break;
      case '?':
        printDebugHelp();
        break;
      case '\n':
      case '\r':
      case ' ':
        break;
      default:
        break;
    }
  }
}

// Single tab-separated diagnostic row. Header is printed once on first dump
// so a serial-plotter or copy-into-spreadsheet workflow lines up.
void dumpDiag(uint16_t* vals, long sum, int pos, int error, float kpUsed,
              int correction, int leftCmd, int rightCmd, const char* stateStr) {
  static bool headerPrinted = false;
  if (!headerPrinted) {
    Serial.println(F("ms\ts0\ts1\ts2\ts3\ts4\ts5\ts6\ts7\ts8\tsum\tpos\terr\tkp\tcorr\tL\tR\tstate"));
    headerPrinted = true;
  }
  Serial.print(millis());
  for (int i = 0; i < SensorCount; i++) {
    Serial.print('\t');
    Serial.print(vals[i]);
  }
  Serial.print('\t'); Serial.print(sum);
  Serial.print('\t'); Serial.print(pos);
  Serial.print('\t'); Serial.print(error);
  Serial.print('\t'); Serial.print(kpUsed, 4);
  Serial.print('\t'); Serial.print(correction);
  Serial.print('\t'); Serial.print(leftCmd);
  Serial.print('\t'); Serial.print(rightCmd);
  Serial.print('\t'); Serial.println(stateStr);
}

// ---- Setup ----
void setup() {
  Serial.begin(115200);
  uint32_t startWait = millis();
  while (!Serial && millis() - startWait < 3000);

  Wire.begin();    // RFID bus
  Wire1.begin();   // Motoron + IMU bus
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

  encoderSetup();
  servoSetup();
  rfidSetup();

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
  printDebugHelp();
  delay(1000);
}

// ---- Loop ----
void loop() {
  serviceSerial();

  if (paused) {
    motoron.setSpeedNow(LEFT_MOTOR,  0);
    motoron.setSpeedNow(RIGHT_MOTOR, 0);
    return;
  }

  // RFID has priority over line follow: any tag → stop, nudge forward, plant.
  // Next tick resumes line following from wherever the bot ended up.
  if (rfidPollAndPlant()) return;

  if (showEnc) {
    static unsigned long lastEncPrintMs = 0;
    if (millis() - lastEncPrintMs > 200) {
      lastEncPrintMs = millis();
      Serial.print("enc BL="); Serial.print(encBL);
      Serial.print(" BR=");    Serial.print(encBR);
      Serial.print(" cm=");    Serial.println(distanceCm(), 2);
    }
  }

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

  int   error      = 0;
  int   correction = 0;
  int   leftCmd    = 0;
  int   rightCmd   = 0;
  float kpUsed     = (runtimeKP >= 0.0f) ? runtimeKP : currentKP;
  const char* stateStr = "NORMAL";

  switch (state) {
    case LINE_NORMAL:         stateStr = "NORMAL"; break;
    case LINE_LOST:           stateStr = "LOST";   break;
    case LINE_JUNCTION_LEFT:  stateStr = "JCT_L";  break;
    case LINE_JUNCTION_RIGHT: stateStr = "JCT_R";  break;
    case LINE_JUNCTION_BOTH:  stateStr = "JCT_B";  break;
  }

  if (state == LINE_LOST) {
    motoron.setSpeedNow(LEFT_MOTOR,  0);
    motoron.setSpeedNow(RIGHT_MOTOR, 0);
    // One-shot search: nudge forward + 90° R/L sweep. After a failure we
    // latch searchFailed so we don't infinite-loop — clears as soon as a
    // line reappears (e.g. user repositions the bot) or via 'r' on serial.
    if (!searchFailed) {
      if (!searchForLine()) {
        searchFailed = true;
        Serial.println("search exhausted — stopped. Reposition bot or send 'r' to retry.");
      }
    }
  } else if (sum > 200) {
    searchFailed = false;
    // Junctions fall through to plain PID — the wide sensor coverage at a
    // junction averages back near centre, so the bot drives straight through.
    // Sign convention: sensor 0 is mounted on the bot's RIGHT, sensor 8 on
    // the LEFT (see spinUntilLine comments). Defining error so that
    // line-on-right is POSITIVE means left-wheel-fast / right-wheel-slow
    // steers toward the line.
    lastPosition = avg / sum;
    error      = 4000 - (int)lastPosition;
    correction = (int)(kpUsed * (float)error);
    leftCmd    = constrain(BASE_SPEED + correction, -800, 800);
    rightCmd   = constrain(BASE_SPEED - correction, -800, 800);
    motoron.setSpeedNow(LEFT_MOTOR,  leftCmd);
    motoron.setSpeedNow(RIGHT_MOTOR, rightCmd);
  }

  if (debugDiag && millis() - lastDiagMs >= DIAG_INTERVAL_MS) {
    lastDiagMs = millis();
    dumpDiag(calibratedVals, sum, (int)lastPosition, error, kpUsed,
             correction, leftCmd, rightCmd, stateStr);
  }
}