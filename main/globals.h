#pragma once
#include "types.h"   // FertileResult, LineState etc. must be defined before extern declarations

// ─────────────────────────────────────────
// globals.h
// extern declarations for variables defined
// in .ino files but used across multiple files.
//
// Definitions live in:
//   main.ino        → showIR, showDistance, isEnabled, useStateMachine
//   motors.ino      → motoron, imu, gyroZOffset,
//                     encBL, encFL, encBR, encFR,
//                     calibTicksPerCm, calibLocked, calibSamples, calibSum,
//                     hopHeadingDeg
//   ir_sensors.ino  → minValues, maxValues, lastPosition
//   rfid.ino        → rfid
//   wifi.ino        → fertileResult, messenger
//   navigation.ino  → navState, tagMap, robotPos, targetPos,
//                     robotFacing, seedsRemaining, pendingJunctionDir
// ─────────────────────────────────────────

// ── main.ino ─────────────────────────────
extern bool  showIR;
extern bool  showDistance;
extern bool  showEncoders;
extern bool  isEnabled;
extern bool  useStateMachine;
extern float lastForwardDistanceCm;   // updated by the obstacle check each tick

// ── motors.ino ───────────────────────────
extern MotoronI2C motoron;
extern LSM6       imu;
extern float      gyroZOffset;
extern volatile long encBL;
extern volatile long encFL;
extern volatile long encBR;
extern volatile long encFR;
extern float calibTicksPerCm;
extern bool  calibLocked;
extern int   calibSamples;
extern float calibSum;
extern float hopHeadingDeg;

// ── ir_sensors.ino ───────────────────────
extern uint16_t minValues[];
extern uint16_t maxValues[];
extern uint16_t lastPosition;

// ── rfid.ino ─────────────────────────────
extern MFRC522_I2C rfid;

// ── wifi.ino ─────────────────────────────
extern FertileResult fertileResult;
extern MiniMessenger messenger;

// ── navigation.ino ───────────────────────
extern NavState  navState;
extern TagState  tagMap[9][9];
extern GridPos   robotPos;
extern GridPos   targetPos;
extern Facing    robotFacing;
extern int       seedsRemaining;
extern int       pendingJunctionDir;
