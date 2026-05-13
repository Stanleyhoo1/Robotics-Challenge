// ============================================================
//  ROBOT POSITION TRACKING
//  Arduino Giga R1
//
//  Hardware:
//    - M5Stack RFID Unit 2 (WS1850S) — I2C
//    - Pololu AltIMU-10 v6 (LSM6DSO gyro) — I2C
//    - 2x Pololu 150:1 Micro Metal Gearmotor w/ encoders
//
//  Libraries required:
//    - LSM6 by Pololu  (install via Library Manager)
//    - Wire (built-in)
//
//  RFID note:
//    The WS1850S library from M5Stack is for their own
//    platform. On Arduino Giga use the raw I2C approach
//    provided here, or adapt the M5Stack RFID2 library.
// ============================================================

#include <Wire.h>
#include <LSM6.h>

// ============================================================
//  PIN DEFINITIONS
//  All encoder pins must be interrupt-capable on the Giga.
//  The Giga supports interrupts on all digital pins.
// ============================================================

#define ENC_LEFT_A   2
#define ENC_LEFT_B   3
#define ENC_RIGHT_A  4
#define ENC_RIGHT_B  5

// ============================================================
//  PHYSICAL CONSTANTS
//  Measure these on your actual robot — even small errors
//  in wheel diameter or wheelbase affect odometry accuracy.
// ============================================================

#define CELL_SIZE_MM          300.0f   // distance between intersections (mm)
#define WHEEL_DIAMETER_MM      32.0f   // measure your actual wheels
#define WHEELBASE_MM          100.0f   // centre-to-centre of drive wheels

#define TICKS_PER_MOTOR_REV    12      // encoder CPR on motor shaft
#define GEAR_RATIO            150      // 150:1 gearbox
#define TICKS_PER_WHEEL_REV   (TICKS_PER_MOTOR_REV * GEAR_RATIO)  // 1800
#define MM_PER_TICK           (PI * WHEEL_DIAMETER_MM / TICKS_PER_WHEEL_REV)

// ============================================================
//  GRID CONFIGURATION
// ============================================================

#define GRID_WIDTH   9
#define GRID_HEIGHT  18

// Entry point — robot always starts here facing this direction
#define START_X        0
#define START_Y        0
#define START_HEADING  NORTH

// ============================================================
//  I2C ADDRESSES
// ============================================================

#define RFID_I2C_ADDR  0x28   // WS1850S default
// LSM6DSO is handled by the Pololu LSM6 library (0x6B default)

// ============================================================
//  TYPES
// ============================================================

enum Direction { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

struct GridPos {
    int x;
    int y;
};

struct RobotState {
    int       grid_x;
    int       grid_y;
    Direction heading;
    float     progress;       // 0.0 → 1.0 between current and next cell
    bool      positionValid;  // false until first RFID fix (after start)
};

// ============================================================
//  TAG MAP
//  Maps each RFID UID to a grid coordinate.
//  Populate this by driving to each intersection and logging
//  the UID printed to Serial — then fill in the coordinates.
//
//  9x18 grid = 162 tags total.
//  A few example entries are shown; add the rest.
// ============================================================

struct TagEntry {
    const char* uid;
    int x;
    int y;
};

TagEntry tagMap[] = {
    // Row y=0
    {"A1B2C3D4", 0,  0},
    {"A1B2C3D5", 1,  0},
    {"A1B2C3D6", 2,  0},
    {"A1B2C3D7", 3,  0},
    {"A1B2C3D8", 4,  0},
    {"A1B2C3D9", 5,  0},
    {"A1B2C3DA", 6,  0},
    {"A1B2C3DB", 7,  0},
    {"A1B2C3DC", 8,  0},
    // Row y=1
    {"B1B2C3D4", 0,  1},
    // ... add all 162 tags
};
const int TAG_COUNT = sizeof(tagMap) / sizeof(TagEntry);

// ============================================================
//  GLOBALS
// ============================================================

RobotState robot;
LSM6       imu;

// Encoder counts — updated atomically by ISRs
volatile long encLeftCount  = 0;
volatile long encRightCount = 0;

// Snapshot of encoder counts at the last RFID fix
// Progress is always measured relative to this
long refLeftCount  = 0;
long refRightCount = 0;

// Gyro state for turn tracking
float   gyroHeading    = 0.0f;   // integrated heading in degrees
unsigned long lastGyroTime = 0;

// RFID debounce — ignore the same tag for this long (ms)
#define RFID_COOLDOWN_MS  1000
String         lastTagUID      = "";
unsigned long  lastTagTime     = 0;

// Serial debug rate limiter
unsigned long lastPrintTime = 0;
#define PRINT_INTERVAL_MS  200

// ============================================================
//  ENCODER ISRs
//  Quadrature decoding on channel A interrupt.
//  Direction determined by comparing A and B at moment of
//  A's rising/falling edge.
// ============================================================

void encoderLeftISR() {
    if (digitalRead(ENC_LEFT_A) == digitalRead(ENC_LEFT_B))
        encLeftCount++;
    else
        encLeftCount--;
}

void encoderRightISR() {
    // Right motor is typically mirrored — invert so both
    // count positive when driving forward
    if (digitalRead(ENC_RIGHT_A) != digitalRead(ENC_RIGHT_B))
        encRightCount++;
    else
        encRightCount--;
}

// ============================================================
//  RFID
//  Raw I2C communication with WS1850S.
//  Returns the UID as an uppercase hex string, e.g. "A1B2C3D4"
//  Returns "" if no card is present.
// ============================================================

String rfidReadUID() {
    // Step 1: Check if a card is present
    Wire.beginTransmission(RFID_I2C_ADDR);
    Wire.write(0x01);   // request card detect
    if (Wire.endTransmission() != 0) return "";

    Wire.requestFrom(RFID_I2C_ADDR, 1);
    if (!Wire.available()) return "";
    uint8_t status = Wire.read();
    if (status == 0) return "";   // no card

    // Step 2: Read UID
    Wire.beginTransmission(RFID_I2C_ADDR);
    Wire.write(0x02);   // request UID
    Wire.endTransmission();
    delayMicroseconds(500);

    Wire.requestFrom(RFID_I2C_ADDR, 5);   // 1 length + up to 4 UID bytes
    if (!Wire.available()) return "";

    uint8_t uidLen = Wire.read();
    if (uidLen == 0 || uidLen > 4) return "";

    String uid = "";
    for (uint8_t i = 0; i < uidLen && Wire.available(); i++) {
        uint8_t b = Wire.read();
        if (b < 0x10) uid += "0";
        uid += String(b, HEX);
    }
    uid.toUpperCase();
    return uid;
}

// ============================================================
//  TAG LOOKUP
// ============================================================

GridPos lookupTag(const String& uid) {
    for (int i = 0; i < TAG_COUNT; i++) {
        if (uid == tagMap[i].uid)
            return {tagMap[i].x, tagMap[i].y};
    }
    return {-1, -1};   // not in map
}

// ============================================================
//  HEADING HELPERS
// ============================================================

Direction deriveHeading(GridPos from, GridPos to) {
    int dx = to.x - from.x;
    int dy = to.y - from.y;

    if (dx ==  1) return EAST;
    if (dx == -1) return WEST;
    if (dy ==  1) return NORTH;
    if (dy == -1) return SOUTH;

    // No movement — shouldn't happen between different tags
    return robot.heading;
}

Direction applyTurnRight(Direction d) { return (Direction)((d + 1) % 4); }
Direction applyTurnLeft (Direction d) { return (Direction)((d + 3) % 4); }

const char* headingName(Direction d) {
    switch (d) {
        case NORTH: return "NORTH";
        case EAST:  return "EAST";
        case SOUTH: return "SOUTH";
        case WEST:  return "WEST";
        default:    return "?";
    }
}

// ============================================================
//  POSITION UPDATES
// ============================================================

void onRFIDFix(const String& uid) {
    GridPos curr = lookupTag(uid);

    if (curr.x == -1) {
        Serial.print("WARNING: Unknown tag UID: ");
        Serial.println(uid);
        return;
    }

    GridPos prev = {robot.grid_x, robot.grid_y};

    // Derive heading from movement — only once we have a previous fix
    // On the very first scan after startup we trust START_HEADING
    if (robot.positionValid && (curr.x != prev.x || curr.y != prev.y)) {
        robot.heading = deriveHeading(prev, curr);
    }

    robot.grid_x       = curr.x;
    robot.grid_y       = curr.y;
    robot.progress     = 0.0f;
    robot.positionValid = true;

    // Reset odometry reference to this intersection
    noInterrupts();
    refLeftCount  = encLeftCount;
    refRightCount = encRightCount;
    interrupts();

    Serial.print("[RFID] Fix at (");
    Serial.print(curr.x);
    Serial.print(", ");
    Serial.print(curr.y);
    Serial.print(") heading=");
    Serial.println(headingName(robot.heading));
}

void updateProgress() {
    // Take an atomic snapshot of encoder counts
    long leftSnap, rightSnap;
    noInterrupts();
    leftSnap  = encLeftCount;
    rightSnap = encRightCount;
    interrupts();

    long leftDelta  = leftSnap  - refLeftCount;
    long rightDelta = rightSnap - refRightCount;

    float leftMM  = (float)leftDelta  * MM_PER_TICK;
    float rightMM = (float)rightDelta * MM_PER_TICK;
    float distMM  = (leftMM + rightMM) / 2.0f;

    robot.progress = constrain(distMM / CELL_SIZE_MM, 0.0f, 1.0f);
}

// ============================================================
//  GYRO — used only for turn verification, not long-term
//  integration. Drift over a single 90° turn is negligible.
// ============================================================

void updateGyro() {
    imu.read();
    unsigned long now = micros();
    float dt = (now - lastGyroTime) / 1e6f;
    lastGyroTime = now;

    // LSM6 library returns raw counts — convert to degrees/sec
    // Default sensitivity at ±245 dps = 8.75 mdps/LSB
    float gyroDPS = imu.g.z * 8.75f / 1000.0f;
    gyroHeading  += gyroDPS * dt;
}

// Call this when commanding a right turn.
// Blocks until the gyro confirms ~90 degrees of rotation.
// Your motor drive code should be running concurrently or
// you integrate this into a non-blocking state machine.
void executeTurnRight() {
    float startHeading = gyroHeading;
    float rotated = 0.0f;

    // TODO: start right turn motors here

    while (rotated < 88.0f) {   // stop 2° early; momentum carries the rest
        updateGyro();
        rotated = abs(gyroHeading - startHeading);
        delay(2);
    }

    // TODO: stop motors here

    robot.heading = applyTurnRight(robot.heading);

    Serial.print("[TURN] Right → now facing ");
    Serial.println(headingName(robot.heading));
}

void executeTurnLeft() {
    float startHeading = gyroHeading;
    float rotated = 0.0f;

    // TODO: start left turn motors here

    while (rotated < 88.0f) {
        updateGyro();
        rotated = abs(gyroHeading - startHeading);
        delay(2);
    }

    // TODO: stop motors here

    robot.heading = applyTurnLeft(robot.heading);

    Serial.print("[TURN] Left → now facing ");
    Serial.println(headingName(robot.heading));
}

// ============================================================
//  REAL-WORLD POSITION ESTIMATE
//  Returns position in mm from grid origin (0,0).
//  Exact at intersections; interpolated between them.
// ============================================================

float estimatedRealX() {
    float base = robot.grid_x * CELL_SIZE_MM;
    if (robot.heading == EAST)  return base + robot.progress * CELL_SIZE_MM;
    if (robot.heading == WEST)  return base - robot.progress * CELL_SIZE_MM;
    return base;
}

float estimatedRealY() {
    float base = robot.grid_y * CELL_SIZE_MM;
    if (robot.heading == NORTH) return base + robot.progress * CELL_SIZE_MM;
    if (robot.heading == SOUTH) return base - robot.progress * CELL_SIZE_MM;
    return base;
}

// ============================================================
//  PUBLIC ACCESSORS
//  Call these from your navigation / motor control code
// ============================================================

int       getGridX()    { return robot.grid_x; }
int       getGridY()    { return robot.grid_y; }
Direction getHeading()  { return robot.heading; }
float     getProgress() { return robot.progress; }
bool      isPositionValid() { return robot.positionValid; }

// ============================================================
//  SETUP
// ============================================================

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);   // wait for Serial on Giga

    Wire.begin();
    Wire1.begin();   // Giga has two I2C buses — use whichever your hardware is on

    // --- Encoders ---
    pinMode(ENC_LEFT_A,  INPUT_PULLUP);
    pinMode(ENC_LEFT_B,  INPUT_PULLUP);
    pinMode(ENC_RIGHT_A, INPUT_PULLUP);
    pinMode(ENC_RIGHT_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A),  encoderLeftISR,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), encoderRightISR, CHANGE);

    // --- IMU ---
    if (!imu.init()) {
        Serial.println("ERROR: IMU (LSM6) not found. Check wiring.");
        while (1);
    }
    imu.enableDefault();
    lastGyroTime = micros();

    // --- Initial robot state ---
    // We always enter from the same position so this is hardcoded
    robot.grid_x        = START_X;
    robot.grid_y        = START_Y;
    robot.heading       = START_HEADING;
    robot.progress      = 0.0f;
    robot.positionValid = true;

    Serial.println("=== Position Tracker Ready ===");
    Serial.print("Start: (");
    Serial.print(START_X);
    Serial.print(", ");
    Serial.print(START_Y);
    Serial.print(") facing ");
    Serial.println(headingName(START_HEADING));
}

// ============================================================
//  MAIN LOOP
// ============================================================

void loop() {
    unsigned long now = millis();

    // --- 1. Gyro update (run every loop for accuracy) ---
    updateGyro();

    // --- 2. RFID poll ---
    String uid = rfidReadUID();
    if (uid.length() > 0) {
        // Debounce — ignore same tag within cooldown window
        if (uid != lastTagUID || (now - lastTagTime) > RFID_COOLDOWN_MS) {
            lastTagUID  = uid;
            lastTagTime = now;
            onRFIDFix(uid);
        }
    } else {
        // No tag under reader — allow same tag to trigger again once clear
        lastTagUID = "";
    }

    // --- 3. Update progress between intersections ---
    updateProgress();

    // --- 4. Debug output (rate limited) ---
    if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
        lastPrintTime = now;

        Serial.print("Grid:(");
        Serial.print(robot.grid_x);
        Serial.print(",");
        Serial.print(robot.grid_y);
        Serial.print(")  ");

        Serial.print("Progress:");
        Serial.print(robot.progress * 100.0f, 1);
        Serial.print("%  ");

        Serial.print("Heading:");
        Serial.print(headingName(robot.heading));
        Serial.print("  ");

        Serial.print("RealPos:(");
        Serial.print(estimatedRealX(), 0);
        Serial.print("mm, ");
        Serial.print(estimatedRealY(), 0);
        Serial.println("mm)");
    }
}
