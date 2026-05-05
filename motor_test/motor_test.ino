#include <Wire.h>
#include <Motoron.h>

MotoronI2C motoron(16, &Wire1);

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Wire1.begin();

  motoron.reinitialize();
  motoron.disableCommandTimeout();
  motoron.clearResetFlag();

  Serial.println("Going forward...");
  motoron.setSpeedNow(1, 700);
  motoron.setSpeedNow(2, 700);
}

void loop() {}