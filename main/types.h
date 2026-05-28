#pragma once

// ─────────────────────────────────────────
// Line Following States
// ─────────────────────────────────────────
enum LineState {
  LINE_NORMAL,
  LINE_LOST,
  LINE_JUNCTION_LEFT,
  LINE_JUNCTION_RIGHT,
  LINE_JUNCTION_BOTH
};

// ─────────────────────────────────────────
// Distance Sensors
// ─────────────────────────────────────────
enum DistanceSensor {
  SENSOR_LEFT,
  SENSOR_RIGHT,
  SENSOR_FORWARD,
  SENSOR_COUNT
};

// ─────────────────────────────────────────
// Fertile Reply Result
// Uses plain char[] instead of String so
// this struct is safe to use in headers
// before Arduino.h is fully processed.
// ─────────────────────────────────────────
struct FertileResult {
  bool received;
  bool fertile;
  bool planted;
  char tagId[32];
  int  x;
  int  y;
};

// ─────────────────────────────────────────
// Grid Position on the 9x9 arena.
// Row increases southward, col increases eastward — see Facing below.
// ─────────────────────────────────────────
struct GridPos {
  int8_t row;
  int8_t col;
  bool valid() const  { return row >= 0 && row < 9 && col >= 0 && col < 9; }
  bool equals(const GridPos& o) const { return row == o.row && col == o.col; }
};

// ─────────────────────────────────────────
// Compass facing. Values 0..3 are deliberate so (newFacing - oldFacing)
// modulo 4 gives a signed turn count for routing.
// Row increases southward, col increases eastward.
// ─────────────────────────────────────────
enum Facing : uint8_t {
  NORTH = 0,
  EAST  = 1,
  SOUTH = 2,
  WEST  = 3
};

// ─────────────────────────────────────────
// Per-tag map state. UNKNOWN means we haven't queried the server for
// this cell yet; the rest are post-query terminal states.
// ─────────────────────────────────────────
enum TagState : uint8_t {
  TAG_UNKNOWN,
  TAG_FERTILE,
  TAG_INFERTILE,
  TAG_PLANTED
};

// ─────────────────────────────────────────
// Top-level navigation state. NAV_WALL_FOLLOW is a placeholder for the
// tunnel-traversal phase; not yet implemented.
// ─────────────────────────────────────────
enum NavState : uint8_t {
  NAV_DISABLED,
  NAV_LINE_FOLLOW,
  NAV_ARENA_NAV,
  NAV_AT_TAG,
  NAV_PLANTING,
  NAV_WALL_FOLLOW,
  NAV_AVOID_OBSTACLE,
  NAV_PARKED
};
