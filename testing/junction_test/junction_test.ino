// ─────────────────────────────────────────
// Junction-detection bench test.
// Slide the robot manually over different line patterns and watch the
// classifier output on serial. Uses the same IR pins and kvstore calibration
// keys as main/, so any calibration saved by the main sketch carries over.
// Send 'c' on serial within 3s of boot to recalibrate.
// ─────────────────────────────────────────

#include <kvstore_global_api.h>

// Must match main/config.h to share calibration with the main sketch.
#define IR_CTRL_PIN                    12
#define IR_SENSOR_COUNT                9
#define IR_TIMEOUT_US                  2500
#define IR_MIN_LINE_SUM                200
#define LINE_CENTER                    4000
#define JUNCTION_ZONE_ACTIVE_THRESHOLD 500

static const int IR_SENSOR_PINS[IR_SENSOR_COUNT] = {30, 31, 32, 33, 34, 35, 36, 37, 38};

uint16_t minValues[IR_SENSOR_COUNT];
uint16_t maxValues[IR_SENSOR_COUNT];

enum LineState {
  LINE_NORMAL,
  LINE_LOST,
  LINE_JUNCTION_LEFT,
  LINE_JUNCTION_RIGHT,
  LINE_JUNCTION_BOTH
};

const char* lineStateStr(LineState s) {
  switch (s) {
    case LINE_NORMAL:         return "NORMAL";
    case LINE_LOST:           return "LOST";
    case LINE_JUNCTION_LEFT:  return "JUNCTION_LEFT  (fork left)";
    case LINE_JUNCTION_RIGHT: return "JUNCTION_RIGHT (fork right)";
    case LINE_JUNCTION_BOTH:  return "JUNCTION_BOTH  (T)";
  }
  return "?";
}

unsigned int readPrivate(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delayMicroseconds(10);
  pinMode(pin, INPUT);
  unsigned long start = micros();
  while (digitalRead(pin) == HIGH) {
    if (micros() - start > IR_TIMEOUT_US) return IR_TIMEOUT_US;
  }
  return micros() - start;
}

bool loadCalibration() {
  size_t actual_size;
  for (int i = 0; i < IR_SENSOR_COUNT; i++) {
    char key[20];
    sprintf(key, "/kv/min%d", i);
    if (kv_get(key, &minValues[i], sizeof(uint16_t), &actual_size) != 0) return false;
    sprintf(key, "/kv/max%d", i);
    if (kv_get(key, &maxValues[i], sizeof(uint16_t), &actual_size) != 0) return false;
  }
  return true;
}

void saveCalibration() {
  for (int i = 0; i < IR_SENSOR_COUNT; i++) {
    char key[20];
    sprintf(key, "/kv/min%d", i);
    kv_set(key, &minValues[i], sizeof(uint16_t), 0);
    sprintf(key, "/kv/max%d", i);
    kv_set(key, &maxValues[i], sizeof(uint16_t), 0);
  }
  Serial.println("Calibration saved.");
}

void runCalibration() {
  for (int i = 0; i < IR_SENSOR_COUNT; i++) {
    minValues[i] = IR_TIMEOUT_US;
    maxValues[i] = 0;
  }
  Serial.println("--- CALIBRATION (10 s) ---");
  Serial.println("Slide sensors over the black line repeatedly!");
  for (int j = 0; j < 400; j++) {
    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
      unsigned int val = readPrivate(IR_SENSOR_PINS[i]);
      if (val < minValues[i]) minValues[i] = val;
      if (val > maxValues[i]) maxValues[i] = val;
    }
    if (j % 40 == 0) Serial.println("Still calibrating...");
    delay(10);
  }
  saveCalibration();
  Serial.println("--- CALIBRATION COMPLETE ---");
}

void readSensors(uint16_t* calibratedVals, long& avg, long& sum) {
  avg = 0;
  sum = 0;
  for (int i = 0; i < IR_SENSOR_COUNT; i++) {
    unsigned int rawVal = readPrivate(IR_SENSOR_PINS[i]);
    int val = constrain(map(rawVal, minValues[i], maxValues[i], 0, 1000), 0, 1000);
    calibratedVals[i] = val;
    avg += (long)val * (i * 1000);
    sum += val;
  }
}

// IR array is mounted with sensor 0 on the PHYSICAL RIGHT side of the robot
// and sensor 8 on the PHYSICAL LEFT. Zone definitions match physical
// orientation. The middle-zone requirement is intentionally dropped so an
// L-turn / off-centre fork (line entirely on one side, no middle activity)
// still classifies as JUNCTION_LEFT/RIGHT.
LineState getLineState(uint16_t* calibratedVals, long sum) {
  if (sum < IR_MIN_LINE_SUM) return LINE_LOST;

  const int T = JUNCTION_ZONE_ACTIVE_THRESHOLD;
  const bool rightActive = (calibratedVals[0] > T) && (calibratedVals[1] > T);
  const bool leftActive  = (calibratedVals[7] > T) && (calibratedVals[8] > T);

  if (leftActive && rightActive)   return LINE_JUNCTION_BOTH;
  if (leftActive  && !rightActive) return LINE_JUNCTION_LEFT;
  if (rightActive && !leftActive)  return LINE_JUNCTION_RIGHT;
  return LINE_NORMAL;
}

void setup() {
  Serial.begin(115200);
  uint32_t startWait = millis();
  while (!Serial && millis() - startWait < 3000);

  pinMode(IR_CTRL_PIN, OUTPUT);
  digitalWrite(IR_CTRL_PIN, HIGH);

  if (!loadCalibration()) {
    Serial.println("No saved calibration, running now...");
    runCalibration();
  } else {
    Serial.println("Calibration loaded. Send 'c' within 3 s to recalibrate.");
    unsigned long s = millis();
    while (millis() - s < 3000) {
      if (Serial.available() && Serial.read() == 'c') { runCalibration(); break; }
    }
  }

  Serial.println("\nReady. Slide the robot over different patterns.");
  Serial.println("Columns: s0..s8 calibrated (0-1000), sum, pos, state");
  Serial.print  ("JUNCTION_ZONE_ACTIVE_THRESHOLD = ");
  Serial.println(JUNCTION_ZONE_ACTIVE_THRESHOLD);
}

void loop() {
  uint16_t cv[IR_SENSOR_COUNT];
  long avg = 0, sum = 0;
  readSensors(cv, avg, sum);
  LineState state = getLineState(cv, sum);

  // Print every tick on transitions; rate-limit the rest to ~5 Hz so the
  // serial monitor stays readable while still being live.
  static LineState lastState   = LINE_NORMAL;
  static unsigned long lastMs  = 0;
  const  unsigned long now     = millis();
  const  bool transitioned     = (state != lastState);
  if (transitioned || (now - lastMs >= 200)) {
    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
      Serial.print(cv[i]);
      Serial.print('\t');
    }
    const int T = JUNCTION_ZONE_ACTIVE_THRESHOLD;
    const bool R = (cv[0] > T) && (cv[1] > T);
    const bool L = (cv[7] > T) && (cv[8] > T);
    long pos = (sum > 0) ? (avg / sum) : 0;
    Serial.print("sum="); Serial.print(sum);
    Serial.print(" pos="); Serial.print(pos);
    Serial.print(" zones[L="); Serial.print(L ? '1' : '0');
    Serial.print(" R=");      Serial.print(R ? '1' : '0');
    Serial.print("] -> ");
    Serial.print(lineStateStr(state));
    if (transitioned) Serial.print("   *** CHANGE ***");
    Serial.println();
    lastMs    = now;
    lastState = state;
  }

  delay(20);
}
