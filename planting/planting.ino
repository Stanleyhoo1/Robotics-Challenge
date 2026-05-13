#include <Wire.h>
#include <Servo.h>
#include "MFRC522_I2C.h"

MFRC522_I2C rfid(0x28, 255);
Servo myServo;

const int SERVO_PIN = 9;
const int MIN_ANGLE = 60;
const int MAX_ANGLE = 160;
const int STEP_DELAY_MS = 5;  // Fast sweep

void sweepTo(int from, int to) {
  int step = (to > from) ? 1 : -1;
  for (int angle = from; angle != to; angle += step) {
    myServo.write(angle);
    delay(STEP_DELAY_MS);
  }
  myServo.write(to);
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Wire.begin();
  rfid.PCD_Init();
  Serial.println("Hold an RFID card near the reader...");

  myServo.attach(SERVO_PIN, 750, 2250);
  sweepTo(myServo.read(), MAX_ANGLE);  // Move to 160 on startup
}

void loop() {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
    delay(50);
    return;
  }

  Serial.print("Card UID: ");
  for (byte i = 0; i < rfid.uid.size; i++) {
    Serial.print(rfid.uid.uidByte[i] < 0x10 ? "0" : "");
    Serial.print(rfid.uid.uidByte[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  rfid.PICC_HaltA();

  // Triggered: sweep to 60 then back to 160
  sweepTo(MAX_ANGLE, MIN_ANGLE);
  sweepTo(MIN_ANGLE, MAX_ANGLE);
}