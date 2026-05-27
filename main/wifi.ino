// ─────────────────────────────────────────
// WiFi / Messaging
// (MiniMessenger.h and types.h included in main.ino)
// ─────────────────────────────────────────
#include "secrets.h"

// FertileResult struct defined in types.h
FertileResult fertileResult = {false, false, false, "", 0, 0};

MiniMessenger messenger;
const char* BoardId = BOARD_ID;
unsigned long lastHeartbeatMs = 0;
unsigned long lastRegisterMs  = 0;

void clearFertileResult() {
  memset(&fertileResult, 0, sizeof(fertileResult));
}

// ─────────────────────────────────────────
// Status LED
// ─────────────────────────────────────────
unsigned long lastBlinkMs = 0;
bool blinkState           = false;

void setLED(int r, int g, int b) {
  digitalWrite(LED_R, r);
  digitalWrite(LED_G, g);
  digitalWrite(LED_B, b);
}

void updateLED() {
  if (isEnabled) {
    setLED(HIGH, LOW, LOW);
  } else {
    if (millis() - lastBlinkMs > LED_BLINK_INTERVAL_MS) {
      lastBlinkMs = millis();
      blinkState = !blinkState;
      setLED(blinkState ? HIGH : LOW, LOW, LOW);
    }
  }
}

// ─────────────────────────────────────────
// Key=value parser
// ─────────────────────────────────────────
String parseValue(const char* msg, const char* key) {
  char searchKey[32];
  snprintf(searchKey, sizeof(searchKey), "%s=", key);
  const char* found = strstr(msg, searchKey);
  if (!found) return "";
  found += strlen(searchKey);
  char value[64] = "";
  int i = 0;
  while (found[i] && found[i] != ' ' && i < 63) {
    value[i] = found[i];
    i++;
  }
  value[i] = '\0';
  return String(value);
}

// ─────────────────────────────────────────
// Incoming message handler
// ─────────────────────────────────────────
void onMessage(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
  char msg[128];
  size_t copyLen = (length < 127) ? length : 127;
  memcpy(msg, payload, copyLen);
  msg[copyLen] = '\0';

  Serial.print("MSG: ");
  Serial.println(msg);

  if (strstr(msg, "type=heartbeat")) {
    lastHeartbeatMs = millis();
    if (strstr(msg, "enable=1")) {
      if (!isEnabled) Serial.println(">>> ENABLED");
      isEnabled = true;
    } else {
      if (isEnabled) Serial.println(">>> DISABLED (heartbeat)");
      isEnabled = false;
    }
  }

  if (strstr(msg, "type=emergency") || strstr(msg, "type=disable")) {
    if (isEnabled) Serial.println(">>> DISABLED (emergency/disable)");
    isEnabled = false;
  }

  if (strstr(msg, "type=isFertileReply")) {
    fertileResult.received = true;
    fertileResult.fertile  = parseValue(msg, "fertile") == "true";
    fertileResult.planted  = parseValue(msg, "planted") == "true";
    strncpy(fertileResult.tagId, parseValue(msg, "tag_id").c_str(), sizeof(fertileResult.tagId) - 1);
    fertileResult.x        = parseValue(msg, "x").toInt();
    fertileResult.y        = parseValue(msg, "y").toInt();

    Serial.println("=== isFertileReply ===");
    Serial.print("  Tag:     "); Serial.println(fertileResult.tagId);
    Serial.print("  Fertile: "); Serial.println(fertileResult.fertile ? "true" : "false");
    Serial.print("  Planted: "); Serial.println(fertileResult.planted ? "true" : "false");
    Serial.print("  Pos:     x="); Serial.print(fertileResult.x);
    Serial.print(" y=");          Serial.println(fertileResult.y);
  }
}

// ─────────────────────────────────────────
// Outgoing message functions
// ─────────────────────────────────────────

void wifiSend(const char* msg) {
  messenger.sendToBoard("server", msg);
  Serial.print("[SENT] ");
  Serial.println(msg);
}

void sendRegister() {
  char msg[64];
  snprintf(msg, sizeof(msg), "type=register team_id=%s board_id=%s", GROUP_ID, BoardId);
  wifiSend(msg);
}

void sendIsFertile(const char* rfid_hex) {
  char msg[64];
  snprintf(msg, sizeof(msg), "type=isFertile tag_id=%s", rfid_hex);
  wifiSend(msg);
}

void sendPlanted(const char* rfid_hex) {
  char msg[64];
  snprintf(msg, sizeof(msg), "type=planted tag_id=%s", rfid_hex);
  wifiSend(msg);
}

void sendPosition(int x, int y) {
  char msg[64];
  snprintf(msg, sizeof(msg), "type=position board_id=%s x=%d y=%d", BoardId, x, y);
  wifiSend(msg);
}

void sendStatus(const char* status) {
  char msg[96];
  snprintf(msg, sizeof(msg), "type=status board_id=%s msg=%s", BoardId, status);
  wifiSend(msg);
}

// ─────────────────────────────────────────
// Setup / Loop
// ─────────────────────────────────────────
void wifiSetup() {
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  setLED(LOW, LOW, LOW);

  messenger.onMessage(onMessage);
  messenger.begin(WIFI_SSID, WIFI_PASSWORD, BROKER_HOST, BROKER_PORT, GROUP_ID, BoardId);
  Serial.println("WiFi connecting...");
}

void wifiPoll() {
  messenger.loop();
}

void wifiLoop() {
  messenger.loop();
  updateLED();

  if (isEnabled && millis() - lastHeartbeatMs > HEARTBEAT_TIMEOUT_MS) {
    isEnabled = false;
    Serial.println(">>> DISABLED (heartbeat timeout)");
  }

  if (millis() - lastRegisterMs > REGISTER_INTERVAL_MS || lastRegisterMs == 0) {
    lastRegisterMs = millis();
    sendRegister();
    Serial.println("Registered.");
  }
}
