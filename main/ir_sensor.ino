// ─────────────────────────────────────────
// IR Sensor Array
// ─────────────────────────────────────────
uint16_t minValues[IR_SENSOR_COUNT];
uint16_t maxValues[IR_SENSOR_COUNT];
uint16_t lastPosition = 0;

void irSetup() {
  pinMode(IR_CTRL_PIN, OUTPUT);
  digitalWrite(IR_CTRL_PIN, HIGH);

  if (!loadCalibration()) {
    Serial.println("No saved calibration, running now...");
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

bool loadCalibration() {
  size_t actual_size;
  for (int i = 0; i < IR_SENSOR_COUNT; i++) {
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
  for (int i = 0; i < IR_SENSOR_COUNT; i++) {
    minValues[i] = IR_TIMEOUT_US;
    maxValues[i] = 0;
  }
  Serial.println("--- CALIBRATION STARTING (10 SECONDS) ---");
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

// LDR diagnostic print, rate-limited to ~5 Hz.
void readAndPrintLDR() {
  if (!showLDR) return;
  static unsigned long lastPrintMs = 0;
  if (millis() - lastPrintMs < 200) return;
  lastPrintMs = millis();
  Serial.print("LDR=");
  Serial.println(analogRead(LDR_PIN));
}

void readAndPrintIR() {
  if (!showIR) return;
  long avg = 0, sum = 0;
  for (int i = 0; i < IR_SENSOR_COUNT; i++) {
    unsigned int rawVal = readPrivate(IR_SENSOR_PINS[i]);
    int calibratedVal = map(rawVal, minValues[i], maxValues[i], 0, 1000);
    calibratedVal = constrain(calibratedVal, 0, 1000);
    avg += (long)calibratedVal * (i * 1000);
    sum += calibratedVal;
    Serial.print(calibratedVal);
    Serial.print("\t");
  }
  if (sum > IR_MIN_LINE_SUM) {
    lastPosition = avg / sum;
  } else {
    lastPosition = (lastPosition < (IR_SENSOR_COUNT - 1) * 1000 / 2) ? 0 : (IR_SENSOR_COUNT - 1) * 1000;
  }
  Serial.print("| Pos: ");
  Serial.print(lastPosition);
  Serial.print(" sum=");
  Serial.print(sum);
  Serial.print(" state=");
  Serial.println(useStateMachine ? navStateStr(navState) : "(legacy)");
}
