#include <Servo.h>

Servo myServo;

const int SERVO_PIN = 9;

// Pulse widths (in microseconds)
const int STOP       = 1500;  // Servo holds still (after calibration)
const int FULL_CW    = 1300;  // Full speed clockwise
const int FULL_CCW   = 1700;  // Full speed counter-clockwise
const int HALF_CW    = 1400;  // Half speed clockwise
const int HALF_CCW   = 1600;  // Half speed counter-clockwise

void setup() {
  myServo.attach(SERVO_PIN);
  myServo.writeMicroseconds(STOP); // Start at rest
  delay(1000);
}

void loop() {
  // Full speed clockwise for 2 seconds
  myServo.writeMicroseconds(FULL_CW);
  delay(2000);

  // Stop for 1 second
  myServo.writeMicroseconds(STOP);
  delay(1000);

  // Full speed counter-clockwise for 2 seconds
  myServo.writeMicroseconds(FULL_CCW);
  delay(2000);

  // Stop for 1 second
  myServo.writeMicroseconds(STOP);
  delay(1000);
}