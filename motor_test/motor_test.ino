#include <Wire.h>
#include <Motoron.h>

MotoronI2C motoron(16, &Wire1);

void setup() {
  Serial.begin(115200);
  // while (!Serial);

  Wire1.begin();

  motoron.reinitialize();
  motoron.disableCommandTimeout();
  motoron.clearResetFlag();

  // Check status flags
  uint16_t status = motoron.getStatusFlags();
  Serial.print("Status flags: 0x");
  Serial.println(status, HEX);

  Serial.println("Going forward...");

  motoron.setSpeedNow(1, 700);
  motoron.setSpeedNow(2, 700);

  delay(3000);

  // Check again after running
  status = motoron.getStatusFlags();
  Serial.print("Status flags after running: 0x");
  Serial.println(status, HEX);
}

void loop() {
  motoron.setSpeedNow(1, 700);
  motoron.setSpeedNow(2, 700);
}