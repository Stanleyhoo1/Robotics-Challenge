// ─────────────────────────────────────────
// Ultrasonic Distance Sensors
// ─────────────────────────────────────────

// Pin lookup tables indexed by DistanceSensor enum
const int TRIG_PINS[SENSOR_COUNT] = {
  LEFT_TRIG_PIN,
  RIGHT_TRIG_PIN,
  FORWARD_TRIG_PIN
};

const int ECHO_PINS[SENSOR_COUNT] = {
  LEFT_ECHO_PIN,
  RIGHT_ECHO_PIN,
  FORWARD_ECHO_PIN
};

const char* SENSOR_NAMES[SENSOR_COUNT] = {
  "Left",
  "Right",
  "Forward"
};

void distanceSetup() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    pinMode(TRIG_PINS[i], OUTPUT);
    pinMode(ECHO_PINS[i], INPUT);
  }
  Serial.println("Ultrasonic sensors ready.");
}

// ─────────────────────────────────────────
// Get distance from a single sensor.
// Returns distance in cm, or -1 if out of range.
// Usage: float d = getDistanceCM(SENSOR_FORWARD);
// ─────────────────────────────────────────
float getDistanceCM(DistanceSensor sensor) {
  int trig = TRIG_PINS[sensor];
  int echo = ECHO_PINS[sensor];

  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH, ULTRASONIC_TIMEOUT);
  if (duration == 0) return -1;
  return duration / 58.0;
}

// ─────────────────────────────────────────
// Print a single sensor reading to Serial.
// Usage: printDistance(SENSOR_LEFT);
// ─────────────────────────────────────────
void printDistance(DistanceSensor sensor) {
  float d = getDistanceCM(sensor);
  Serial.print(SENSOR_NAMES[sensor]);
  Serial.print(": ");
  if (d < 0) Serial.println("OOR");
  else { Serial.print(d); Serial.println(" cm"); }
}

// ─────────────────────────────────────────
// Print all sensor readings (called from loop
// when showDistance is enabled)
// ─────────────────────────────────────────
void readAndPrintDistance() {
  if (!showDistance) return;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    printDistance((DistanceSensor)i);
  }
}
