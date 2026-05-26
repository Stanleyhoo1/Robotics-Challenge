#pragma once

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
#define MOTOR_VOLTAGE   6
#define INPUT_VOLTAGE   6
#define TURN_SPEED      500
#define FORWARD_SPEED   400
#define MIN_TURN_SPEED  150   // minimum speed at end of turn slow-zone

// ─────────────────────────────────────────
// IMU
// ─────────────────────────────────────────
#define GYRO_CALIB_SAMPLES  500
#define GYRO_SENS           0.00875f  // dps/LSB for ±250 dps range
#define TURN_SLOW_ZONE_DEG  20.0f     // degrees before target to start slowing

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
#define TRIG_PIN            40
#define ECHO_PIN            41
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