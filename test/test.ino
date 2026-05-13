#include <Wire.h>

TwoWire* buses[] = { &Wire, &Wire1, &Wire2 };
const char* busNames[] = { "Wire (pins 20/21)", "Wire1 (pins 11/12)", "Wire2 (ESLOV)" };

void scanBus(TwoWire* bus, const char* name) {
  Serial.print("\nScanning ");
  Serial.println(name);

  bus->begin();
  int found = 0;

  for (byte addr = 1; addr < 127; addr++) {
    bus->beginTransmission(addr);
    byte error = bus->endTransmission();

    if (error == 0) {
      Serial.print("  Device found at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      found++;
    }
  }

  if (found == 0) Serial.println("  Nothing found.");
}

void setup() {
  Serial.begin(9600);
  uint32_t startWait = millis();
  while (!Serial && millis() - startWait < 3000);
  
  Serial.println("=== I2C Multi-Bus Scan ===");
  for (int i = 0; i < 3; i++) {
    scanBus(buses[i], busNames[i]);
  }
  Serial.println("\nDone.");
}

void loop() {}