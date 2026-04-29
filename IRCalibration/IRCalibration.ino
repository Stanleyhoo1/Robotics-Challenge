const int sensorPins[9] = {2, 3, 4, 5, 6, 7, 8, 9, 10};
const int ctrlPin = 12;
const int SensorCount = 9;
const unsigned int timeout = 2500;

uint16_t minValues[9];
uint16_t maxValues[9];
uint16_t lastPosition = 0; // This provides the "Memory" the library uses

unsigned int readPrivate(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delayMicroseconds(10);
  pinMode(pin, INPUT);
  unsigned long start = micros();
  while (digitalRead(pin) == HIGH) {
    if (micros() - start > timeout) return timeout;
  }
  return micros() - start;
}

void setup() {
  Serial.begin(115200);
  uint32_t startWait = millis();
  while (!Serial && millis() - startWait < 3000);

  pinMode(ctrlPin, OUTPUT);
  digitalWrite(ctrlPin, HIGH);

  for (int i = 0; i < SensorCount; i++) {
    minValues[i] = timeout;
    maxValues[i] = 0;
  }

  Serial.println("--- CALIBRATION STARTING (10 SECONDS) ---");
  Serial.println("Slide sensors over the black line repeatedly!");

  for (int j = 0; j < 400; j++) {
    for (int i = 0; i < SensorCount; i++) {
      unsigned int val = readPrivate(sensorPins[i]);
      if (val < minValues[i]) minValues[i] = val;
      if (val > maxValues[i]) maxValues[i] = val;
    }
    if (j % 40 == 0) Serial.println("Still calibrating...");
    delay(10); 
  }

  Serial.println("--- CALIBRATION COMPLETE ---");
  Serial.println("Starting Live Data...");
  delay(1000);
}

void loop() {
  long avg = 0;
  long sum = 0;

  for (int i = 0; i < SensorCount; i++) {
    unsigned int rawVal = readPrivate(sensorPins[i]);
    
    // Map raw data to 0-1000 range based on calibration
    int calibratedVal = map(rawVal, minValues[i], maxValues[i], 0, 1000);
    calibratedVal = constrain(calibratedVal, 0, 1000);
    
    // Weighted average logic (index * 1000)
    avg += (long)calibratedVal * (i * 1000);
    sum += calibratedVal;

    Serial.print(calibratedVal);
    Serial.print("\t");
  }

  /* Line Position Math:
     If the line is seen, calculate new position.
     If the line is LOST (sum is small), use the last known position.
     This is exactly what qtr.readLineBlack() does.
  */
  if (sum > 200) { 
    lastPosition = avg / sum;
  } else {
    // If we lost the line, keep it at the extreme 0 or 8000
    if (lastPosition < (SensorCount - 1) * 1000 / 2) {
      lastPosition = 0;
    } else {
      lastPosition = (SensorCount - 1) * 1000;
    }
  }

  Serial.print("| Pos: ");
  Serial.println(lastPosition);

  delay(20); // Faster for smoother line following
}