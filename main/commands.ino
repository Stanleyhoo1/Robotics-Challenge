// ─────────────────────────────────────────
// Serial Command Handler
// ─────────────────────────────────────────
void handleSerialCommands() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() == 0) return;

  if (input.equalsIgnoreCase("ir")) {
    showIR = !showIR;
    Serial.print("IR readings ");
    Serial.println(showIR ? "ON" : "OFF");

  } else if (input.equalsIgnoreCase("distance")) {
    showDistance = !showDistance;
    Serial.print("Distance readings ");
    Serial.println(showDistance ? "ON" : "OFF");

  } else if (input.equalsIgnoreCase("sensors")) {
    bool newState = !(showIR && showDistance);
    showIR = newState;
    showDistance = newState;
    Serial.print("All sensor readings ");
    Serial.println(newState ? "ON" : "OFF");

  } else if (input.equalsIgnoreCase("c")) {
    runCalibration();

  } else if (input.startsWith("forward")) {
    int speed = FORWARD_SPEED;
    int spaceIdx = input.indexOf(' ');
    if (spaceIdx != -1) speed = input.substring(spaceIdx + 1).toInt();
    moveForward(3000, speed);

  } else {
    float degrees = input.toFloat();
    if (degrees != 0) {
      Serial.print("Turning ");
      Serial.print(degrees);
      Serial.println(" degrees...");
      turnDegrees(degrees);
    } else {
      // Forward unrecognised commands to the server
      wifiSend(input.c_str());
    }
  }
}
