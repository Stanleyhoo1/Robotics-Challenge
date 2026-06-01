#include <Wire.h>
#include <Motoron.h>

MotoronI2C motoron(16, &Wire1);

void initMotor() {
  motoron.reinitialize();
  motoron.disableCommandTimeout();
  motoron.clearResetFlag();
}

void setup() {
  Serial.begin(115200);
  Wire1.begin();
  initMotor();
  Serial.println("Setup done");
}

void loop() {
  motoron.clearResetFlag();
  motoron.setSpeedNow(1, 800);
  motoron.setSpeedNow(2, 800);
}
