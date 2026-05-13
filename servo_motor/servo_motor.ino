#include <Servo.h>

Servo myServo;

const int SERVO_PIN = 9;
const int MIN_ANGLE = 60;
const int MAX_ANGLE = 160;
const int STEP_DELAY_MS = 5;  // Lower = faster sweep

void setup() {
  myServo.attach(SERVO_PIN, 750, 2250);
  int current = myServo.read();
  int step = (current < MAX_ANGLE) ? 1 : -1;
  for (int i = current; i != MAX_ANGLE; i += step) {
    myServo.write(i);
    delay(15);
  }
}

void loop() {
  // Sweep 60 → 160
  for (int angle = MIN_ANGLE; angle <= MAX_ANGLE; angle++) {
    myServo.write(angle);
    delay(STEP_DELAY_MS);
  }

  // Sweep 160 → 60
  for (int angle = MAX_ANGLE; angle >= MIN_ANGLE; angle--) {
    myServo.write(angle);
    delay(STEP_DELAY_MS);
  }
}