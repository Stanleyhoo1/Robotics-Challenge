// ─────────────────────────────────────────
// WiFi / Messaging
// (MiniMessenger.h and types.h included in main.ino)
// ─────────────────────────────────────────
#include "secrets.h"

// FertileResult struct defined in types.h
FertileResult fertileResult = {false, false, false, "", 0, 0};

// Server-side clearance acks. Reset on each open-request send, set true on
// receipt of the corresponding clearance message. Currently used for
// verification / debugging only — the door-open signal that actually drives
// the state machine is the forward ultrasonic going back above OBSTACLE_STOP_CM.
bool exitClearanceReceived  = false;
bool enterClearanceReceived = false;

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

void setLED(int r, int g) {
  digitalWrite(LED_R, r);
  digitalWrite(LED_G, g);
}

void updateLED() {
  // Revive button held: override everything else, show solid green.
  // Released → falls through to the red logic below.
  if (digitalRead(REVIVE_BUTTON_1) == LOW || digitalRead(REVIVE_BUTTON_2) == LOW) {
    setLED(LOW, HIGH);
    return;
  }

  if (isEnabled) {
    setLED(HIGH, LOW);                      // solid red while running
  } else {
    if (millis() - lastBlinkMs > LED_BLINK_INTERVAL_MS) {
      lastBlinkMs = millis();
      blinkState = !blinkState;
      setLED(blinkState ? HIGH : LOW, LOW); // blink red while disabled
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

  // Updated server protocol: a single openAirlockReply carries airlock=A|B and
  // accepted=true|false plus queue counters. Branch on airlock to set the
  // matching clearance flag. accepted=false logs the reply but leaves the flag
  // false so the wait state's retry timer resends the open request.
  if (strstr(msg, "type=openAirlockReply")) {
    const String airlock  = parseValue(msg, "airlock");
    const bool   accepted = parseValue(msg, "accepted") == "true";
    if (accepted) {
      if (airlock == "A") {
        exitClearanceReceived = true;
        Serial.println(">>> openAirlockReply A accepted");
      } else if (airlock == "B") {
        enterClearanceReceived = true;
        Serial.println(">>> openAirlockReply B accepted");
      } else {
        Serial.print(">>> openAirlockReply accepted, unknown airlock=");
        Serial.println(airlock);
      }
    } else {
      Serial.print(">>> openAirlockReply ");
      Serial.print(airlock);
      Serial.print(" REJECTED queue_enter=");
      Serial.print(parseValue(msg, "queue_enter"));
      Serial.print(" queue_exit=");
      Serial.println(parseValue(msg, "queue_exit"));
    }
  }

  // Back-compat: keep the old single-purpose clearance messages working in
  // case any server endpoint still emits them.
  if (strstr(msg, "type=exitClearance")) {
    exitClearanceReceived = true;
    Serial.println(">>> exitClearance");
  }

  if (strstr(msg, "type=enterClearance")) {
    enterClearanceReceived = true;
    Serial.println(">>> enterClearance");
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

// Airlock open requests. A = exit base, B = enter base.
// Wire format: type=openAirlock airlock=A|B tag_id=<UID> board_id=<id>.
// The server rejects requests without tag_id (reason=missing_tag), so the
// UID of the airlock RFID tag must be passed in on the first send. Each
// function stores the UID in a per-airlock buffer so retries via the no-arg
// overloads (called from the wait-state retry timer and main.ino's
// door-retry logic) can resend without re-reading the tag.
static char lastAirlockAUid[32] = "";
static char lastAirlockBUid[32] = "";

static void sendOpenAirlockMsg(char which, const char* uid) {
  char msg[96];
  snprintf(msg, sizeof(msg),
           "type=openAirlock airlock=%c tag_id=%s board_id=%s",
           which, uid, BoardId);
  wifiSend(msg);
}

void sendOpenAirlockA(const char* tagId) {
  exitClearanceReceived = false;
  strncpy(lastAirlockAUid, tagId, sizeof(lastAirlockAUid) - 1);
  lastAirlockAUid[sizeof(lastAirlockAUid) - 1] = '\0';
  sendOpenAirlockMsg('A', lastAirlockAUid);
}

void sendOpenAirlockA() {
  if (lastAirlockAUid[0] == '\0') {
    Serial.println("sendOpenAirlockA: no stored tag UID, skipping resend");
    return;
  }
  exitClearanceReceived = false;
  sendOpenAirlockMsg('A', lastAirlockAUid);
}

void sendOpenAirlockB(const char* tagId) {
  enterClearanceReceived = false;
  strncpy(lastAirlockBUid, tagId, sizeof(lastAirlockBUid) - 1);
  lastAirlockBUid[sizeof(lastAirlockBUid) - 1] = '\0';
  sendOpenAirlockMsg('B', lastAirlockBUid);
}

void sendOpenAirlockB() {
  if (lastAirlockBUid[0] == '\0') {
    Serial.println("sendOpenAirlockB: no stored tag UID, skipping resend");
    return;
  }
  enterClearanceReceived = false;
  sendOpenAirlockMsg('B', lastAirlockBUid);
}

// ─────────────────────────────────────────
// Setup / Loop
// ─────────────────────────────────────────
void wifiSetup() {
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  setLED(LOW, LOW);   // off at boot; first wifiLoop tick will pick up blinking-disabled

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

  // Heartbeat-timeout safety only kicks in after we've heard at least one
  // heartbeat. Before that (e.g. bench-testing without a server), the button
  // / serial commands are authoritative; if the server has never talked to
  // us, we have no "lost contact" condition to react to.
  if (lastHeartbeatMs != 0 && isEnabled &&
      millis() - lastHeartbeatMs > HEARTBEAT_TIMEOUT_MS) {
    isEnabled = false;
    Serial.println(">>> DISABLED (heartbeat timeout)");
  }

  if (millis() - lastRegisterMs > REGISTER_INTERVAL_MS || lastRegisterMs == 0) {
    lastRegisterMs = millis();
    sendRegister();
    Serial.println("Registered.");
  }
}
