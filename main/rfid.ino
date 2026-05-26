// ─────────────────────────────────────────
// RFID
// ─────────────────────────────────────────
MFRC522_I2C rfid(RFID_I2C_ADDRESS, RFID_RESET_PIN);

void rfidSetup() {
  rfid.PCD_Init();
  Serial.println("RFID ready.");
}

bool rfidLoop() {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) return false;

  Serial.print("Card UID: ");
  for (byte i = 0; i < rfid.uid.size; i++) {
    Serial.print(rfid.uid.uidByte[i] < 0x10 ? "0" : "");
    Serial.print(rfid.uid.uidByte[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  rfid.PICC_HaltA();
  return true;
}
