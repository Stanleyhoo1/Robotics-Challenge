#include <Wire.h>
#include <Motoron.h>

// ── Hardware ────────────────────────────────────────────────────────────────
const int TRIG_PIN = 40;
const int ECHO_PIN = 41;

MotoronI2C motoron(16, &Wire1);

// ── Tuning ──────────────────────────────────────────────────────────────────
const float TARGET_CM       = 20.0;   // desired gap to wall
const float Kp              = 18.0;  // proportional gain
const float Kd              = 25.0;  // derivative gain
const int   BASE_SPEED      = 300;   // straight-line motor speed (–800 … 800)
const int   MAX_CORRECTION  = 400;   // clamp so we never stall a motor
const int   SENSOR_DELAY_MS = 60;    // HC-SR04 minimum cycle time
const float EMA_ALPHA       = 0.2;   // EMA smoothing (0.1 = smooth, 0.4 = responsive)

// ── State ───────────────────────────────────────────────────────────────────
float previousDistance = TARGET_CM;
float smoothedDistance = TARGET_CM;
unsigned long previousTime = 0;

// ── Sensor ──────────────────────────────────────────────────────────────────
float singleRead() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);  // 30ms timeout
  if (duration == 0) return -1.0;
  return duration / 58.0;
}

float medianOfThree(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

float getDistanceCM() {
  float a = singleRead(); delay(10);
  float b = singleRead(); delay(10);
  float c = singleRead();

  // If all failed, return invalid
  if (a < 0 && b < 0 && c < 0) return -1.0;

  // Replace any failed reading with a valid neighbour
  if (a < 0) a = (b > 0) ? b : c;
  if (b < 0) b = (a > 0) ? a : c;
  if (c < 0) c = (a > 0) ? a : b;

  return medianOfThree(a, b, c);
}

// ── Motor helpers ────────────────────────────────────────────────────────────
int clampSpeed(int speed) {
  return constrain(speed, -800, 800);
}

void setMotors(int left, int right) {
  motoron.setSpeedNow(1, clampSpeed(left));
  motoron.setSpeedNow(2, clampSpeed(right));
}

// ── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Wire1.begin();
  motoron.reinitialize();
  motoron.disableCommandTimeout();
  motoron.clearResetFlag();

  previousTime = millis();
  Serial.println("Wall follower ready.");
}

// ── Main loop ────────────────────────────────────────────────────────────────
void loop() {
  float raw = getDistanceCM();

  // Skip bad readings — hold previous correction rather than jerking
  if (raw < 0) {
    Serial.println("Sensor: out of range, holding course");
    delay(SENSOR_DELAY_MS);
    return;
  }

  // EMA: smoothed = α·raw + (1−α)·smoothed
  smoothedDistance = EMA_ALPHA * raw + (1.0 - EMA_ALPHA) * smoothedDistance;

  unsigned long now = millis();
  float dt = (now - previousTime) / 1000.0;
  if (dt <= 0) dt = SENSOR_DELAY_MS / 1000.0;  // guard against zero

  // ── PD controller ───────────────────────────────────────────────────────
  float error      = smoothedDistance - TARGET_CM;           // +ve → too far, –ve → too close
  float dError     = (smoothedDistance - previousDistance) / dt; // +ve → drifting away, –ve → closing
  float correction = (Kp * error) + (Kd * dError);
  correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

  // ── Differential steering ───────────────────────────────────────────────
  // Sensor on LEFT side: positive error → steer left (boost left, reduce right)
  // Flip signs if sensor faces right
  int leftSpeed  = BASE_SPEED + (int)correction;
  int rightSpeed = BASE_SPEED - (int)correction;
  setMotors(leftSpeed, rightSpeed);

  // ── Debug ────────────────────────────────────────────────────────────────
  Serial.print("raw=");      Serial.print(raw, 1);
  Serial.print("  smooth="); Serial.print(smoothedDistance, 1);
  Serial.print("  err=");    Serial.print(error, 1);
  Serial.print("  dErr=");   Serial.print(dError, 1);
  Serial.print("  corr=");   Serial.print((int)correction);
  Serial.print("  L=");      Serial.print(leftSpeed);
  Serial.print("  R=");      Serial.println(rightSpeed);

  previousDistance = smoothedDistance;
  previousTime     = now;

  delay(SENSOR_DELAY_MS);
}