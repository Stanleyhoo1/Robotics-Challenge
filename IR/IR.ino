const int sensorPins[9] = {2, 3, 4, 5, 6, 7, 8, 9, 10}; // 1 to D23, 2 to D24, etc.
const int ctrlPin = 12; // CTRL blue pin

const unsigned int timeout = 3000; // microseconds

unsigned int readQTR(int pin) {
// Charge capacitor
pinMode(pin, OUTPUT);
digitalWrite(pin, HIGH);
delayMicroseconds(10);

// Let it discharge
pinMode(pin, INPUT);

unsigned long start = micros();

while (digitalRead(pin) == HIGH) {
if (micros() - start > timeout) {
return timeout;
}
}

return micros() - start;
}

void setup() {
Serial.begin(115200);
delay(2000);

pinMode(ctrlPin, OUTPUT);
digitalWrite(ctrlPin, HIGH); // turn IR LEDs ON

Serial.println("QTR-5RC Sensor Test");
Serial.println("S1\tS2\tS3\tS4\tS5\tS6\tS7\tS8\tS9");
}

void loop() {
for (int i = 0; i < 9; i++) {
unsigned int value = readQTR(sensorPins[i]);
Serial.print(value);
Serial.print("\t");
}

Serial.println();
delay(1000);
}