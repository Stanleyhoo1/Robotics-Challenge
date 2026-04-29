#include <Servo.h>
#include <QTRSensors.h>

// --- IR Sensor setup ---
QTRSensors qtr;
const uint8_t SENSOR_COUNT = 5;
uint16_t sensorValues[SENSOR_COUNT];

// --- Servo setup ---
Servo leftServo, rightServo;
const int LEFT_PIN  = 9;
const int RIGHT_PIN = 10;

// Tune these after calibration
const int BASE_SPEED = 50;   // how fast to drive (maps to pulse offset from 1500)
const float KP = 0.1;        // proportional gain — tune this

void setup() {
  Serial.begin(115200);

  // Configure QTR sensor
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4}, SENSOR_COUNT);
  qtr.setEmitterPins(2, 3); // CTRL ODD, CTRL EVEN

  // Calibrate — move robot slowly over the line for ~5 seconds
  Serial.println("Calibrating... move over line now");
  for (int i = 0; i < 200; i++) {
    qtr.calibrate();
    delay(20);
  }
  Serial.println("Calibration done.");

  leftServo.attach(LEFT_PIN);
  rightServo.attach(RIGHT_PIN);
}

void loop() {
  // Returns 0–4000 for a 5-sensor array (2000 = line centred)
  int16_t position = qtr.readLineBlack(sensorValues);

  // Error: how far from centre (2000)
  int error = position - 2000;

  // P-controller: adjust servo speeds based on error
  int correction = (int)(KP * error);

  // For continuous rotation servos: 1500 = stop, offset = speed
  // Left and right servos spin in opposite directions for forward motion
  leftServo.writeMicroseconds(1500 + BASE_SPEED + correction);
  rightServo.writeMicroseconds(1500 - BASE_SPEED + correction);

  Serial.print("Pos: "); Serial.print(position);
  Serial.print("  Err: "); Serial.println(error);
}