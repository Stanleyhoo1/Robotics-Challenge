const int TRIG_PIN = 40;
const int ECHO_PIN = 41;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.println("HC-SR04 ready.");
}

float getDistanceCM() {
  // Send 10us trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure echo pulse duration
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout

  if (duration == 0) return -1; // no echo received

  // Convert to cm
  return duration / 58.0;
}

void loop() {
  float distance = getDistanceCM();

  if (distance < 0) {
    Serial.println("Out of range");
  } else {
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  }

  delay(60); // datasheet recommends at least 60ms between readings
}