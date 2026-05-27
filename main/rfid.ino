// ─────────────────────────────────────────
// RFID
// ─────────────────────────────────────────
MFRC522_I2C rfid(RFID_I2C_ADDRESS, RFID_RESET_PIN);
int seedCount = SEED_COUNT;
unsigned long lastScanMs = 0;

void rfidSetup() {
  rfid.PCD_Init();
  Serial.println("RFID ready.");
}

void rfidLoop() {
  if (millis() - lastScanMs < 2000) return;
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) return;

  // Build UID string e.g. "A3F2019C"
  char uidStr[32] = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
    char byteStr[3];
    snprintf(byteStr, sizeof(byteStr), "%02X", rfid.uid.uidByte[i]);
    strcat(uidStr, byteStr);
  }

  Serial.print("Card UID: ");
  Serial.println(uidStr);

  rfid.PICC_HaltA();

  clearFertileResult();
  sendIsFertile(uidStr);

  // Wait for reply, keeping messenger alive
  unsigned long start = millis();
  while (!fertileResult.received) {
    wifiPoll();
    if (millis() - start > FERTILE_REPLY_TIMEOUT_MS) {
      Serial.println("RFID: no reply from server, timed out.");
      return;
    }
    delay(10);
  }

  if (fertileResult.fertile && !fertileResult.planted) {
    if (seedCount <= 0) {
      Serial.println("RFID: no seeds remaining.");
    } else {
      Serial.println("RFID: fertile and unplanted — planting!");
      sweepTo(MAX_ANGLE, MIN_ANGLE);
      sweepTo(MIN_ANGLE, MAX_ANGLE);
      sendPlanted(fertileResult.tagId);
      seedCount--;
      Serial.print("RFID: seeds remaining: ");
      Serial.println(seedCount);
    }
  } else {
    Serial.println("RFID: infertile or seed already planted");
  }

  lastScanMs = millis();

  return;
}
