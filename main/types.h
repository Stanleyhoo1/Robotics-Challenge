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
  TAG_PLANTED,
  TAG_BLOCKED      // cell contains an obstacle / other robot — A* routes around it
};

// ─────────────────────────────────────────
// Top-level navigation state. NAV_WALL_FOLLOW is a placeholder for the
// tunnel-traversal phase; not yet implemented.
// ─────────────────────────────────────────
enum NavState : uint8_t {
  NAV_DISABLED,
  NAV_LINE_FOLLOW,
  // Base-exit sequence (in order)
  NAV_BASE_TO_FIRST_JUNCTION,    // line-follow until first intersection
  NAV_BASE_FIRST_TURN,           // turn right (exit case)
  NAV_BASE_TO_TAG,               // line-follow until RFID tag detected
  NAV_WAIT_EXIT_CLEARANCE,       // exitRequest sent at base tag, wait for server clearance
  NAV_BASE_TO_SECOND_JUNCTION,   // exitClearance received, line-follow to T-junction
  NAV_BASE_SECOND_TURN,          // turn opposite direction of first turn
  NAV_BASE_TO_LINE_LOST,         // line-follow until LINE_LOST
  NAV_BASE_LINE_LOST_PAUSE,      // momentary stop with yellow LED after losing line
  NAV_BASE_FORWARD_NUDGE,        // brief forward drive, then wall-follow tunnel
  // Arena states
  NAV_ARENA_NAV,
  NAV_AT_TAG,
  NAV_POST_TAG_NUDGE,    // forward nudge after RFID hit, before turn or plant
  NAV_PLANTING,
  NAV_WALL_FOLLOW,
  NAV_AVOID_OBSTACLE,
  // Return sequence (arena → base via Airlock B)
  NAV_AT_AIRLOCK_B,              // rotate to face base, send openAirlockB
  NAV_WAIT_ENTER_CLEARANCE,      // enterRequest sent at airlock B, wait for server clearance
  NAV_TUNNEL_B_WALL_FOLLOW,      // wall-follow down tunnel B (mirrors NAV_WALL_FOLLOW)
  NAV_BASE_RETURN,               // line-follow inside base until LINE_LOST or obstacle
  NAV_PARKED
};
