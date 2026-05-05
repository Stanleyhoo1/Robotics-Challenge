#include <WiFi.h>

const char* SSID     = "PhaseSpaceNetwork_2.4G";
const char* PASSWORD = "8igMacNet";

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.print("Connecting to WiFi");
  WiFi.begin(SSID, PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nFailed to connect!");
    Serial.print("WiFi status code: ");
    Serial.println(WiFi.status());
  } else {
    Serial.println("\nConnected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("RSSI: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  }
}

void loop() {}