#include <Wire.h>
#include <Motoron.h>

MotoronI2C motoron(16, &Wire1);
const uint8_t TEST_MOTOR = 1;

const int ENC_A = 2;
const int ENC_B = 3;

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

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Wire1.begin();

  motoron.reinitialize();
  motoron.disableCommandTimeout();
  motoron.clearResetFlag();

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  lastA = digitalRead(ENC_A);
  lastB = digitalRead(ENC_B);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), encoderISR_B, CHANGE);

  Serial.println("Setup done.");
}

void loop() {
  motoron.setSpeedNow(TEST_MOTOR, 0);

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    noInterrupts();
    long rawCount = encoderCount;
    interrupts();
    long count = rawCount / 2; // correct for double counting
    Serial.print("Count: ");
    Serial.println(count);
    lastPrint = millis();
  }
}