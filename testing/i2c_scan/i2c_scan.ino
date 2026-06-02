#include <Wire.h>

void scanBus(TwoWire &bus, const char *name) {
  Serial.print("Scanning ");
  Serial.print(name);
  Serial.println("...");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    bus.beginTransmission(addr);
    uint8_t err = bus.endTransmission();
    if (err == 0) {
      Serial.print("  Found device at 0x");
      if (addr < 16) Serial.print("0");
      Serial.print(addr, HEX);
      Serial.print(" (");
      Serial.print(addr);
      Serial.println(")");
      found++;
    }
  }
  if (found == 0) Serial.println("  No devices found.");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();
  Wire1.begin();

  scanBus(Wire,  "Wire  (pins 20/21)");
  scanBus(Wire1, "Wire1 (SDA1/SCL1) ");
}

void loop() {}
