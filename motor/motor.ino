#include <Wire.h>
#include <Motoron.h>

MotoronI2C motoron;

const int ENC_A = 18;
const int ENC_B = 19;

volatile long encoderCount = 0;
volatile bool lastA = false;
volatile bool lastB = false;

void encoderISR_A() {
  bool a = digitalRead(ENC_A);
  bool b = digitalRead(ENC_B);
  if (a != lastA) {
    encoderCount += (a == b) ? -1 : 1;
    lastA = a;
  }
}

void encoderISR_B() {
  bool a = digitalRead(ENC_A);
  bool b = digitalRead(ENC_B);
  if (b != lastB) {
    encoderCount += (a == b) ? 1 : -1;
    lastB = b;
  }
}

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