#include <MiniMessenger.h>
#include "secrets.h"

MiniMessenger messenger;
const char* BoardId = "Leonard";  // Remember to choose a name for your robot :)

// Callback function that runs whenever a new message arrives
void onMessage(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
  Serial.print("Message from Board ");
  Serial.print(metadata.fromBoardId);
  Serial.print(": ");
  
  // Print the payload as text
  for (size_t i = 0; i < length; i++) {
    Serial.write(payload[i]);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  
  // Register the callback function
  messenger.onMessage(onMessage);

  // Initialize messenger
  messenger.begin(WIFI_SSID, WIFI_PASSWORD, BROKER_HOST, BROKER_PORT, GROUP_ID, BoardId);

  Serial.println("Messenger Ready!");
}

void loop() {
  // Critical: Keep the messenger connection alive
  messenger.loop();

  static unsigned long lastSend = 0;
  if (messenger.isConnected() && millis() - lastSend > 5000) {
    lastSend = millis();
    
    // Send a message to Board "2"
    messenger.sendToBoard("2", "Hello from Terminator!");
    Serial.println("Message sent!");
  }
}