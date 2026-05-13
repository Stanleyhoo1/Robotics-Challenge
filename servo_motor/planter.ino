#include <Servo.h>

// Define pins
const int servoPin = 9;
const int feedbackPin = A0;

Servo myservo;

void setup() {
  Serial.begin(9600);                // Initialize serial monitor
  myservo.attach(servoPin);          // Attach servo
  Serial.println("Servo Feedback Test");
}

void loop() {
  moveAndRead(0);
  delay(600);
  moveAndRead(270);
  delay(600);
}

void moveAndRead(int angle) {
  myservo.write(angle);              // Set position
    delay(500);                        // Wait fo\\feedback
}
