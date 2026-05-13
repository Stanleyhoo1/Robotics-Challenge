#include <Wire.h>
#include <Motoron.h>

// ── Hardware ────────────────────────────────────────────────────────────────
const int TRIG_PIN = 8;
const int ECHO_PIN = 9;

MotoronI2C motoron(16, &Wire1);

// ── Tuning ──────────────────────────────────────────────────────────────────
const float TARGET_CM      = 7.0;  // desired gap to wall
const float Kp             = 18.0;  // proportional gain
const float Kd             = 25.0;  // derivative gain  (damps oscillation)
const int   BASE_SPEED     = 300;   // straight-line motor speed  (–800 … 800)
const int   MAX_CORRECTION = 400;   // clamp so we never stall a motor
const int   SENSOR_DELAY_MS = 60;   // HC-SR04 minimum cycle time

// ── State ───────────────────────────────────────────────────────────────────
float previousDistance = TARGET_CM;  // seed with target so first D-term = 0
unsigned long previousTime = 0;

// ── Sensor ──────────────────────────────────────────────────────────────────
float getDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);  // 30 ms timeout
  if (duration == 0) return -1.0;
  return duration / 58.0;
}

// ── Motor helper ─────────────────────────────────────────────────────────────
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

// ── Main loop ─────────────────────────────────────────────────────────────────
void loop() {
  float current = getDistanceCM();

  // Skip bad readings — keep previous correction rather than jerking
  if (current < 0) {
    Serial.println("Sensor: out of range, holding course");
    delay(SENSOR_DELAY_MS);
    return;
  }

  unsigned long now = millis();
  float dt = (now - previousTime) / 1000.0;  // seconds
  if (dt <= 0) dt = SENSOR_DELAY_MS / 1000.0; // guard against zero

  // ── PD terms ────────────────────────────────────────────────────────────
  float error      = current - TARGET_CM;           // +ve → too far, –ve → too close
  float dError     = (current - previousDistance) / dt; // +ve → moving away, –ve → closing in
  float correction = (Kp * error) + (Kd * dError);
  correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

  // ── Differential steering ───────────────────────────────────────────────
  // Sensor on LEFT side → positive error means steer left (increase left, decrease right)
  // Flip signs if your sensor faces right.
  int leftSpeed  = BASE_SPEED + (int)correction;
  int rightSpeed = BASE_SPEED - (int)correction;
  setMotors(leftSpeed, rightSpeed);

  // ── Debug ────────────────────────────────────────────────────────────────
  Serial.print("dist="); Serial.print(current, 1);
  Serial.print("  err="); Serial.print(error, 1);
  Serial.print("  dErr="); Serial.print(dError, 1);
  Serial.print("  corr="); Serial.print((int)correction);
  Serial.print("  L="); Serial.print(leftSpeed);
  Serial.print("  R="); Serial.println(rightSpeed);

  previousDistance = current;
  previousTime     = now;

  delay(SENSOR_DELAY_MS);
}