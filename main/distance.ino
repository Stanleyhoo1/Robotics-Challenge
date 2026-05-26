// ─────────────────────────────────────────
// Ultrasonic
// ─────────────────────────────────────────
void distanceSetup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.println("Ultrasonic ready.");
}

float getDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT);
  if (duration == 0) return -1;
  return duration / 58.0;
}

void readAndPrintDistance() {
  if (!showDistance) return;
  float distance = getDistanceCM();
  Serial.print("Dist: ");
  if (distance < 0) Serial.println("OOR");
  else { Serial.print(distance); Serial.println(" cm"); }
}
