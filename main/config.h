#pragma once

// ─────────────────────────────────────────
// Robot State
// ─────────────────────────────────────────
#define SEED_COUNT  5

// ─────────────────────────────────────────
// Servo
// ─────────────────────────────────────────
#define SERVO_PIN       9
#define SERVO_MIN_US    750
#define SERVO_MAX_US    2250
#define MIN_ANGLE       60
#define MAX_ANGLE       160
#define STEP_DELAY      5     // ms per degree during sweep

// ─────────────────────────────────────────
// Motors
// ─────────────────────────────────────────
#define LEFT_MOTOR      1
#define RIGHT_MOTOR     2
#define MOTORON_ADDRESS 16
#define MOTOR_VOLTAGE   6     // rated motor voltage
#define INPUT_VOLTAGE   6     // actual supply voltage
#define TURN_SPEED      500
#define FORWARD_SPEED   400
#define MIN_TURN_SPEED  150   // minimum speed at end of turn slow-zone

// ─────────────────────────────────────────
// IMU
// ─────────────────────────────────────────
#define GYRO_CALIB_SAMPLES  500
#define GYRO_SENS           0.00875f  // dps/LSB for ±250 dps range
#define TURN_SLOW_ZONE_DEG  20.0f     // degrees before target to start slowing
#define HEADING_KP          3.0f      // proportional gain for hop-heading correction

// ─────────────────────────────────────────
// IR Sensor Array
// ─────────────────────────────────────────
#define IR_CTRL_PIN         12
#define IR_SENSOR_COUNT     9
#define IR_TIMEOUT_US       2500      // max reflection time (µs)
#define IR_MIN_LINE_SUM     200       // minimum sum to trust position reading

static const int IR_SENSOR_PINS[IR_SENSOR_COUNT] = {30, 31, 32, 33, 34, 35, 36, 37, 38};

// ─────────────────────────────────────────
// Ultrasonic
// ─────────────────────────────────────────
#define RIGHT_TRIG_PIN      40
#define RIGHT_ECHO_PIN      41
#define LEFT_TRIG_PIN       42
#define LEFT_ECHO_PIN       43
#define FORWARD_TRIG_PIN    44
#define FORWARD_ECHO_PIN    45
#define ULTRASONIC_TIMEOUT  30000     // pulseIn timeout (µs), ~5m max range

// ─────────────────────────────────────────
// RFID
// ─────────────────────────────────────────
#define RFID_I2C_ADDRESS    0x28
#define RFID_RESET_PIN      255       // not used (I2C mode)

// ─────────────────────────────────────────
// Serial
// ─────────────────────────────────────────
#define SERIAL_BAUD         115200
#define SERIAL_WAIT_MS      3000      // max wait for Serial to connect on startup

// ─────────────────────────────────────────
// WiFi / Messaging
// ─────────────────────────────────────────
#define HEARTBEAT_TIMEOUT_MS    1000    // disable motors if no heartbeat within this window
#define REGISTER_INTERVAL_MS    10000   // how often to re-register with server

// ─────────────────────────────────────────
// Status LED (RGB)
// ─────────────────────────────────────────
#define LED_R   50
#define LED_G   51
#define LED_B   52
#define LED_BLINK_INTERVAL_MS   500     // blink rate when disabled

// ─────────────────────────────────────────
// Line Following
// ─────────────────────────────────────────
#define BASE_SPEED              500
#define KP                      0.10f     // proportional gain, normal
#define KP_AGGRESSIVE           0.15f     // proportional gain after junction
#define AGGRESSIVE_DURATION_MS  2000      // how long to stay on aggressive KP
#define LINE_CENTER             4000      // target position (sensor 4 of 9, 0-indexed)
#define JUNCTION_MIN_ROT_DEG    20.0f     // min rotation before checking for line on spin
#define JUNCTION_MAX_ROT_DEG    180.0f    // abort spin if exceeded
#define JUNCTION_NUDGE_MS       150       // forward nudge duration after spin
#define JUNCTION_FORWARD_MS     300       // forward drive to centre on junction

// ─────────────────────────────────────────
// RFID
// ─────────────────────────────────────────
#define FERTILE_REPLY_TIMEOUT_MS  5000    // max ms to wait for isFertileReply

// ─────────────────────────────────────────
// Board identity
// ─────────────────────────────────────────
#define BOARD_ID  "Master"

// ─────────────────────────────────────────
// Junction sequence
// -1 = left, 0 = straight, 1 = right
// ─────────────────────────────────────────
#define JUNCTION_SEQUENCE  { -1, 0, 1 }

// ─────────────────────────────────────────
// Encoders (quadrature, 2 per side, all EXTI-capable on Giga)
// ─────────────────────────────────────────
#define ENC_BL_A  22
#define ENC_BL_B  23
#define ENC_FL_A  24
#define ENC_FL_B  25
#define ENC_BR_A  26
#define ENC_BR_B  27
#define ENC_FR_A  28
#define ENC_FR_B  29

// ─────────────────────────────────────────
// Grid / encoder calibration
// ─────────────────────────────────────────
#define GRID_SPACING_CM         31.25f
#define TICKS_PER_CM_FALLBACK   8.0f
#define NODE_ARRIVAL_FRACTION   0.85f
#define CALIB_MIN_SAMPLES       4
#define CALIB_OUTLIER_PCT       0.20f

// ─────────────────────────────────────────
// Arena halves: left half has black guidance lines, right half has only holes.
// LEFT_HALF_MAX_COL is the highest column index (0-based) where line-following works.
// Cols 0..4 ≡ A..E (left half), 5..8 ≡ F..I (right half).
// ─────────────────────────────────────────
#define LEFT_HALF_MAX_COL       4

// ─────────────────────────────────────────
// Forward obstacle threshold (cm). A forward ultrasonic reading in
// [0, OBSTACLE_STOP_CM) latches a stop. -1 (out of range) does NOT trigger.
// Latch only clears on an isEnabled false→true cycle from the server.
// ─────────────────────────────────────────
#define OBSTACLE_STOP_CM        8

// ─────────────────────────────────────────
// Obstacle avoidance (NAV_AVOID_OBSTACLE)
// Trigger threshold sits above the door/emergency threshold so the state
// machine sidesteps non-door obstacles before they're close enough to latch.
// ─────────────────────────────────────────
#define OBSTACLE_AVOID_CM       20
#define OBSTACLE_SIDESTEP_MS    400
#define OBSTACLE_FORWARD_MS     600

// ─────────────────────────────────────────
// Tunnel wall-following
// ─────────────────────────────────────────
#define WALL_FOLLOW_TARGET_CM   15
#define WALL_KP                 4.0f
#define WALL_FORWARD_CLEAR_CM   40
