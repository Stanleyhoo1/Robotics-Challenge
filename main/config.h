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
#define MIN_ANGLE       60    // get seed
#define MAX_ANGLE       180   // release seed
#define STEP_DELAY      5     // ms per degree during sweep

// ─────────────────────────────────────────
// Motors
// ─────────────────────────────────────────
#define LEFT_MOTOR      1
#define RIGHT_MOTOR     2
#define MOTORON_ADDRESS 16
#define MOTOR_VOLTAGE   6     // rated motor voltage
#define INPUT_VOLTAGE   7.2     // actual supply voltage
// Pivot turn speeds. The "forward" wheel goes forward, the "backward" wheel
// goes backward; we keep the backward wheel slower because the backward-going
// side slips more due to weight transfer. Split by direction so left- and
// right-turn behaviour can be tuned independently — the two motors aren't
// always equally strong, so symmetric speed commands can produce asymmetric
// physical turns. RIGHT_* tunes a clockwise pivot, LEFT_* tunes CCW.
#define RIGHT_TURN_FORWARD_SPEED    600     // left wheel during right turn
#define RIGHT_TURN_BACKWARD_SPEED   450     // right wheel during right turn
#define LEFT_TURN_FORWARD_SPEED     400     // right wheel during left turn — tune to match right turn
#define LEFT_TURN_BACKWARD_SPEED    400     // left wheel during left turn — tune to match right turn
// Per-direction "stop early" trim, in degrees, subtracted from the target
// magnitude inside turnDegrees(). If one direction over-rotates, raise its
// trim so the gyro integrator hits the break point sooner. Default 0 = no trim.
#define RIGHT_TURN_TRIM_DEG         0.0f
#define LEFT_TURN_TRIM_DEG          5.0f
#define FORWARD_SPEED         400     // Default forward speed

// ─────────────────────────────────────────
// IMU
// ─────────────────────────────────────────
#define GYRO_CALIB_SAMPLES  500
#define GYRO_SENS           0.00875f  // dps/LSB for ±250 dps range
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
// LDR (light-dependent resistor, analog read 0..1023)
// ─────────────────────────────────────────
#define LDR_PIN             A7

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
#define LED_R   48
#define LED_G   49
#define LED_BLINK_INTERVAL_MS   500     // blink rate when disabled

// ─────────────────────────────────────────
// Buttons
// ─────────────────────────────────────────
#define POWER_BUTTON      17
#define REVIVE_BUTTON_1   52
#define REVIVE_BUTTON_2   53

// ─────────────────────────────────────────
// Line Following
// ─────────────────────────────────────────
#define BASE_SPEED              200
#define KP                      0.10f     // proportional gain, normal
#define KP_AGGRESSIVE           0.15f     // proportional gain after junction
#define AGGRESSIVE_DURATION_MS  2000      // how long to stay on aggressive KP
#define LINE_CENTER             4000      // target position (sensor 4 of 9, 0-indexed)
#define JUNCTION_MIN_ROT_DEG    20.0f     // min rotation before checking for line on spin
#define JUNCTION_MAX_ROT_DEG    180.0f    // abort spin if exceeded
#define TURN_LINE_CHECK_MIN_DEG 80.0f     // turnDegrees() ignores line detection until past this rotation — keeps it from latching on the line we're turning off of (or the perpendicular fork branch still visible mid-turn)
#define JUNCTION_NUDGE_MS       150       // forward nudge duration after spin
#define JUNCTION_FORWARD_MS     300       // forward drive to centre on junction
// Per-sensor calibrated value (0..1000) above which a sensor is considered
// "on the line" for junction classification in getLineState(). Higher than a
// generic line-present threshold so PID drift onto an outer sensor doesn't
// register as a side branch — a real branch saturates the IR signal.
#define JUNCTION_ZONE_ACTIVE_THRESHOLD 500
#define LINE_FOLLOW_MIN_SPEED   300       // floor for the slow-wheel side of the line-follow PID; below this the motor stalls
#define LINE_SEARCH_SPIN_SPEED  400       // wheel speed used by spinUntilLine / sweepForLine — independent of BASE_SPEED so slow line-follow doesn't make the search too slow to spin the wheels

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
#define JUNCTION_SEQUENCE  { -1, 0, 1 } // For testing, not currently used

// ─────────────────────────────────────────
// Encoders (quadrature, 1 per side on the rear wheels; EXTI-capable on Giga).
// Back-right uses pins 24/25 (what the 4-encoder layout previously assigned
// to front-left); pins 26/27/28/29 are free.
// ─────────────────────────────────────────
#define ENC_BL_A  22
#define ENC_BL_B  23
#define ENC_BR_A  14
#define ENC_BR_B  15

// ─────────────────────────────────────────
// Grid / encoder calibration
// ─────────────────────────────────────────
#define GRID_SPACING_CM         25.0f
#define TICKS_PER_CM_FALLBACK   159.97f   // measured: 2438 ticks over 6 in (15.24 cm)
#define NODE_ARRIVAL_FRACTION   0.85f // stops robot early so it doesn't go past hole
#define CALIB_MIN_SAMPLES       4
#define CALIB_OUTLIER_PCT       0.20f

// Forward distance to drive after an RFID hit before turning or dispensing
// a seed. Same value for both intents: it offsets the robot so the wheel
// axis (= turn pivot) and the seed dispenser sit over the tag/hole.
#define POST_TAG_FORWARD_CM     3.0f  // still needs tuning

// ─────────────────────────────────────────
// Arena zones: the line grid covers the BOTTOM half of the arena across ALL
// columns. 1-based competition rows 5..9 (= 0-based rows 4..8) have black
// guidance lines on every column. 0-based rows 0..3 (1-based 1..4) are
// hole-only — navigate by gyro heading-lock + encoder dead-reckoning.
// LINE_ZONE_MIN_ROW is the lowest 0-based row index where line-following works.
// ─────────────────────────────────────────
#define LINE_ZONE_MIN_ROW       4

// ─────────────────────────────────────────
// Airlocks (arena-side positions, 0-based row/col).
// Competition 1-based coords: A = (3, 9), B = (7, 9).
// Exit base via Tunnel A → robot arrives at Airlock A.
// Return to base via Tunnel B → robot leaves arena from Airlock B.
// ─────────────────────────────────────────
#define AIRLOCK_A_ROW           2
#define AIRLOCK_A_COL           8
#define AIRLOCK_B_ROW           6
#define AIRLOCK_B_COL           8

// Direction the robot is facing right after popping out of Tunnel A into
// the arena. Used as the seed for robotFacing on the WALL_FOLLOW → ARENA_NAV
// transition, and to compute the "face base" direction at Airlock B (=
// (ARENA_ENTRY_FACING + 2) mod 4). Integer value matches the Facing enum in
// types.h: NORTH=0, EAST=1, SOUTH=2, WEST=3. Default WEST since the tunnel
// is on the east edge of the arena.
#define ARENA_ENTRY_FACING_INT  3

// ─────────────────────────────────────────
// Forward obstacle threshold (cm). A forward ultrasonic reading in
// [0, OBSTACLE_STOP_CM) latches a stop. -1 (out of range) does NOT trigger.
// ─────────────────────────────────────────
#define OBSTACLE_STOP_CM        8

// ─────────────────────────────────────────
// Obstacle avoidance (NAV_AVOID_OBSTACLE)
// Trigger threshold sits above the door/emergency threshold so the state
// machine sidesteps non-door obstacles before they're close enough to latch.
// ─────────────────────────────────────────
#define OBSTACLE_AVOID_CM       20
// #define OBSTACLE_SIDESTEP_MS    400 // change to mark obstacle on grid and replan route
// #define OBSTACLE_FORWARD_MS     600

// ─────────────────────────────────────────
// Tunnel wall-following (PD on left/right balance — no target distance,
// just minimize |leftDist − rightDist| to stay centered)
// ─────────────────────────────────────────
#define WALL_KP                 25.0f
#define WALL_KD                 120.0f
#define WALL_BASE_SPEED         350
#define WALL_MAX_CORRECTION     600
#define WALL_EMA_ALPHA          0.4f

// ─────────────────────────────────────────
// Base exit sequence (base → airlock → arena).
// Airlock A = exit (base → arena), Airlock B = entry (arena → base).
// Turn convention: positive = CW (right) per turnDegrees.
// ─────────────────────────────────────────
#define BASE_FIRST_TURN_DEG     90.0f      // T-junction: positive = right (exit), flip sign to let another robot in
#define BASE_SECOND_TURN_DEG    -90.0f     // second junction (fork): turn left or right, never straight
#define BASE_LINE_LOST_PAUSE_MS 1500       // momentary stop after line lost in base before tunnel approach
#define BASE_FORWARD_NUDGE_MS   500        // drive forward this long after losing line
#define DOOR_RETRY_INTERVAL_MS  3000       // resend openAirlockX this often while paused at a closed door
