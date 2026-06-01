// ─────────────────────────────────────────
// Servo
// ─────────────────────────────────────────
Servo myServo;

void servoSetup() {
  myServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  // Snap directly to MAX_ANGLE — using sweepTo here would no-op at boot since
  // isEnabled starts false and the kill-switch check bails immediately.
  myServo.write(MAX_ANGLE);
  Serial.println("Servo at MAX_ANGLE.");
}

void sweepTo(int from, int to) {
  int step = (to > from) ? 1 : -1;
  for (int angle = from; angle != to; angle += step) {
    // Kill-switch responsiveness mid-sweep. Servo holds its current position
    // when we bail; the next sweepTo call will pick up from wherever it is.
    wifiLoop();
    checkPowerButton();
    if (!isEnabled) {
      Serial.println("sweepTo aborted: !isEnabled");
      return;
    }
    myServo.write(angle);
    delay(STEP_DELAY);
  }
  myServo.write(to);
}
