#include <Wire.h>
#include <Motoron.h>

MotoronI2C motoron;

const uint8_t TEST_MOTOR = 1; // change to 2 or 3 for other channels

void setup() {
  Serial.begin(115200);
  Wire1.begin();

  motoron.reinitialize();
  motoron.disableCommandTimeout();
  motoron.clearResetFlag();

  Serial.println("Motoron ready.");
}

void loop() {
  Serial.println("Forward...");
  motoron.setSpeed(TEST_MOTOR, 400);
  delay(2000);

  Serial.println("Stop...");
  motoron.setSpeed(TEST_MOTOR, 0);
  delay(1000);

  Serial.println("Backward...");
  motoron.setSpeed(TEST_MOTOR, -400);
  delay(2000);

  Serial.println("Stop...");
  motoron.setSpeed(TEST_MOTOR, 0);
  delay(1000);
}