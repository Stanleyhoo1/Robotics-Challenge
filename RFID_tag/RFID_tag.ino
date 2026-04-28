#include <Wire.h>
#include "MFRC522_I2C.h"

MFRC522_I2C rfid(0x28, 255);

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Wire.begin();
  rfid.PCD_Init();
  Serial.println("Hold an RFID card near the reader...");
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
}