// Minimal encoder test sketch — push the bot by hand and watch the counts.
// Same pin layout + decoding pattern as the production firmware in
// main/motors.ino. Motoron is initialised only to force both motors to 0
// at boot so wheels are free to push manually.

#include <Wire.h>
#include <Motoron.h>

MotoronI2C motoron(16, &Wire1);
const uint8_t LEFT_MOTOR  = 1;
const uint8_t RIGHT_MOTOR = 2;

// ---- Encoder pins (match main/config.h) ----
const int ENC_BL_A = 22;
const int ENC_BL_B = 23;
const int ENC_BR_A = 14;
const int ENC_BR_B = 15;

// Measured empirically: 2438 ticks over 15.24 cm (6 in) on this build.
// Use the `c <value>` serial command to override at runtime.
float ticksPerCm = 159.97f;

// ---- ISR state ----
volatile long encBL = 0;
volatile long encBR = 0;
volatile bool lastBL_A = false, lastBL_B = false;
volatile bool lastBR_A = false, lastBR_B = false;

// One ISR per channel. (a == b) gives 4× decoding + direction. Sign of the
// right-side ISRs is inverted because the right wheel is mirrored — if your
// "forward push" gives BL positive but BR negative, swap the +1/-1 on the
// BR ISRs below.
void isr_BL_A() { bool a = digitalRead(ENC_BL_A), b = digitalRead(ENC_BL_B); if (a != lastBL_A) { encBL += (a == b) ?  1 : -1; lastBL_A = a; } }
void isr_BL_B() { bool a = digitalRead(ENC_BL_A), b = digitalRead(ENC_BL_B); if (b != lastBL_B) { encBL += (a == b) ? -1 :  1; lastBL_B = b; } }
void isr_BR_A() { bool a = digitalRead(ENC_BR_A), b = digitalRead(ENC_BR_B); if (a != lastBR_A) { encBR += (a == b) ? -1 :  1; lastBR_A = a; } }
void isr_BR_B() { bool a = digitalRead(ENC_BR_A), b = digitalRead(ENC_BR_B); if (b != lastBR_B) { encBR += (a == b) ?  1 : -1; lastBR_B = b; } }

// ---- Print state ----
bool printOn = true;
unsigned long lastPrintMs = 0;
const unsigned long PRINT_INTERVAL_MS = 200;  // 5 Hz

void resetEncoders() {
  noInterrupts();
  encBL = 0;
  encBR = 0;
  interrupts();
}

void printHelp() {
  Serial.println(F("--- ENCODER TEST ---"));
  Serial.println(F("  z          zero both counts"));
  Serial.println(F("  p          toggle printing"));
  Serial.println(F("  c <ticks>  set ticks-per-cm (e.g. 'c 120.5')"));
  Serial.println(F("  s          one-shot snapshot (prints once even if 'p' is off)"));
  Serial.println(F("  ?          this help"));
  Serial.print  (F("  current ticks_per_cm = ")); Serial.println(ticksPerCm, 3);
}

void printSnapshot() {
  Serial.print(F("BL=")); Serial.print(encBL);
  Serial.print(F("\tBR=")); Serial.print(encBR);
  Serial.print(F("\tavg=")); Serial.print((encBL + encBR) / 2);
  Serial.print(F("\tcm=")); Serial.println(((encBL + encBR) / 2.0f) / ticksPerCm, 2);
}

void serviceSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    switch (ch) {
      case 'z':
        resetEncoders();
        Serial.println(F("zeroed"));
        break;
      case 'p':
        printOn = !printOn;
        Serial.print(F("print: "));
        Serial.println(printOn ? F("ON") : F("OFF"));
        break;
      case 'c': {
        float v = Serial.parseFloat();
        if (v > 0.1f && v < 10000.0f) {
          ticksPerCm = v;
          Serial.print(F("ticks_per_cm = "));
          Serial.println(ticksPerCm, 3);
        } else {
          Serial.println(F("usage: c <ticks>  (0.1 < ticks < 10000)"));
        }
        break;
      }
      case 's':
        printSnapshot();
        break;
      case '?':
        printHelp();
        break;
      case '\n': case '\r': case ' ':
        break;
      default:
        break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  uint32_t startWait = millis();
  while (!Serial && millis() - startWait < 3000);

  Wire1.begin();
  motoron.reinitialize();
  motoron.disableCommandTimeout();
  motoron.clearResetFlag();
  motoron.setSpeedNow(LEFT_MOTOR,  0);
  motoron.setSpeedNow(RIGHT_MOTOR, 0);

  pinMode(ENC_BL_A, INPUT_PULLUP); pinMode(ENC_BL_B, INPUT_PULLUP);
  pinMode(ENC_BR_A, INPUT_PULLUP); pinMode(ENC_BR_B, INPUT_PULLUP);

  // Seed last-state so the first edge isn't misread.
  lastBL_A = digitalRead(ENC_BL_A); lastBL_B = digitalRead(ENC_BL_B);
  lastBR_A = digitalRead(ENC_BR_A); lastBR_B = digitalRead(ENC_BR_B);

  attachInterrupt(digitalPinToInterrupt(ENC_BL_A), isr_BL_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_BL_B), isr_BL_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_BR_A), isr_BR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_BR_B), isr_BR_B, CHANGE);

  Serial.println(F("Encoder test ready. Push the bot by hand."));
  printHelp();
}

void loop() {
  serviceSerial();
  if (printOn && millis() - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = millis();
    printSnapshot();
  }
}
