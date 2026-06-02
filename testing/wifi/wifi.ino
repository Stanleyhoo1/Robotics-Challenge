#include <MiniMessenger.h>
#include <Wire.h>
#include <Motoron.h>
#include "secrets.h"


MotoronI2C motoron(16, &Wire1);
MiniMessenger messenger;

const char* BoardId = "Master";
bool isEnabled = false;
unsigned long lastRegisterMs = 0;
unsigned long lastHeartbeatMs = 0;
const unsigned long HEARTBEAT_TIMEOUT_MS = 1000;

// --- RGB LED ---
const int LED_R = 50;  // change pins to match your wiring
const int LED_G = 51;
const int LED_B = 52;

unsigned long lastBlinkMs = 0;
bool blinkState = false;

void setLED(int r, int g, int b) {
    digitalWrite(LED_R, r);
    digitalWrite(LED_G, g);
    digitalWrite(LED_B, b);
}

void updateLED() {
    if (isEnabled) {
        setLED(HIGH, LOW, LOW); // solid red
    } else {
        // blink red non-blocking
        if (millis() - lastBlinkMs > 500) {
            lastBlinkMs = millis();
            blinkState = !blinkState;
            setLED(blinkState ? HIGH : LOW, LOW, LOW);
        }
    }
}

// --- KEY=VALUE PARSER ---
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
        String tagId   = parseValue(msg, "tag_id");
        String fertile = parseValue(msg, "fertile");
        String planted = parseValue(msg, "planted");
        String x       = parseValue(msg, "x");
        String y       = parseValue(msg, "y");

        Serial.println("=== isFertileReply ===");
        Serial.print("  Tag:     "); Serial.println(tagId);
        Serial.print("  Fertile: "); Serial.println(fertile);
        Serial.print("  Planted: "); Serial.println(planted);
        Serial.print("  Pos:     x="); Serial.print(x); Serial.print(" y="); Serial.println(y);

        if (fertile == "true" && planted == "false") {
            Serial.println("  >> Can plant here!");
        }
    }
}

void handleSerialInput() {
    if (!Serial.available()) return;

    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    messenger.sendToBoard("server", input.c_str());
    Serial.print("[SENT] ");
    Serial.println(input);
}

void setup() {
    Serial.begin(115200);
    Wire1.begin();
    motoron.reinitialize();
    motoron.disableCommandTimeout();
    motoron.clearResetFlag();

    pinMode(LED_R, OUTPUT);
    pinMode(LED_G, OUTPUT);
    pinMode(LED_B, OUTPUT);
    setLED(LOW, LOW, LOW);

    Serial.println("Starting...");
    messenger.onMessage(onMessage);
    messenger.begin(WIFI_SSID, WIFI_PASSWORD, BROKER_HOST, BROKER_PORT, GROUP_ID, BoardId);
    Serial.println("Connecting...");
}

void loop() {
    messenger.loop();
    handleSerialInput();
    updateLED();

    if (isEnabled && millis() - lastHeartbeatMs > HEARTBEAT_TIMEOUT_MS) {
        isEnabled = false;
        Serial.println(">>> DISABLED (heartbeat timeout)");
    }

    if (millis() - lastRegisterMs > 10000 || lastRegisterMs == 0) {
        lastRegisterMs = millis();
        char reg[64];
        snprintf(reg, sizeof(reg), "type=register team_id=%s board_id=%s", GROUP_ID, BoardId);
        messenger.sendToBoard("server", reg);
        Serial.println("Registered.");
    }

    if (isEnabled) {
        motoron.setSpeedNow(1, 500);
        motoron.setSpeedNow(2, 500);
    } else {
        motoron.setSpeedNow(1, 0);
        motoron.setSpeedNow(2, 0);
    }
}