// ─────────────────────────────────────────
// Servo
// ─────────────────────────────────────────
Servo myServo;

void servoSetup() {
  myServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  sweepTo(myServo.read(), MAX_ANGLE);
  Serial.println("Servo at MAX_ANGLE.");
}

void sweepTo(int from, int to) {
  int step = (to > from) ? 1 : -1;
  for (int angle = from; angle != to; angle += step) {
    myServo.write(angle);
    delay(STEP_DELAY);
  }
  myServo.write(to);
}
