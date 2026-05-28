// ─────────────────────────────────────────
// Line Following / Navigation
// NOTE: This file is named navigation.ino
// (n > m) so it sorts after motors.ino and
// can access motoron, imu, gyroZOffset etc.
// ─────────────────────────────────────────

// Junction action sequence: -1 = left, 0 = straight, 1 = right
const int junctionActions[] = JUNCTION_SEQUENCE;
int   junctionCount    = 0;
bool  inJunction       = false;
float currentKP        = KP;
unsigned long junctionExitTime = 0;

// ─────────────────────────────────────────
// Read all sensors into calibratedVals,
// populate avg and sum by reference
// ─────────────────────────────────────────
void readSensors(uint16_t* calibratedVals, long& avg, long& sum) {
  avg = 0;
  sum = 0;
  for (int i = 0; i < IR_SENSOR_COUNT; i++) {
    unsigned int rawVal = readPrivate(IR_SENSOR_PINS[i]);
    int val = constrain(map(rawVal, minValues[i], maxValues[i], 0, 1000), 0, 1000);
    calibratedVals[i] = val;
    avg += (long)val * (i * 1000);
    sum += val;
  }
}

// ─────────────────────────────────────────
// Classify what the sensors are seeing
// ─────────────────────────────────────────
LineState getLineState(uint16_t* calibratedVals, long sum) {
  bool leftActive  = false;
  bool rightActive = false;
  int  activeCount = 0;

  for (int i = 0; i < IR_SENSOR_COUNT; i++) {
    if (calibratedVals[i] > 500) {
      activeCount++;
      if (i <= 2) leftActive  = true;
      if (i >= 6) rightActive = true;
    }
  }

  if (sum < IR_MIN_LINE_SUM)           return LINE_LOST;
  if (leftActive && rightActive)       return LINE_JUNCTION_BOTH;
  if (leftActive  && activeCount >= 7) return LINE_JUNCTION_LEFT;
  if (rightActive && activeCount >= 7) return LINE_JUNCTION_RIGHT;
  return LINE_NORMAL;
}

// ─────────────────────────────────────────
// Spin in direction (-1 left, 1 right)
// until the line reappears on the correct
// side, then nudge forward to align
// ─────────────────────────────────────────
void spinUntilLine(int direction) {
  int spinLeft  = (direction == -1) ? -BASE_SPEED :  BASE_SPEED;
  int spinRight = (direction == -1) ?  BASE_SPEED : -BASE_SPEED;
  motoron.setSpeedNow(LEFT_MOTOR,  scaleSpeed(spinLeft));
  motoron.setSpeedNow(RIGHT_MOTOR, scaleSpeed(spinRight));

  float accumulated = 0;
  unsigned long lastTime = micros();

  while (abs(accumulated) < JUNCTION_MAX_ROT_DEG) {
    imu.read();
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0;
    lastTime = now;
    accumulated += -((imu.g.z - gyroZOffset) * GYRO_SENS) * dt;

    if (abs(accumulated) > JUNCTION_MIN_ROT_DEG) {
      bool lowActive  = false;   // sensors 0-2
      bool highActive = false;   // sensors 6-8

      for (int i = 0; i < IR_SENSOR_COUNT; i++) {
        int val = constrain(
          map(readPrivate(IR_SENSOR_PINS[i]), minValues[i], maxValues[i], 0, 1000),
          0, 1000
        );
        if (val > 500) {
          if (i <= 2) lowActive  = true;
          if (i >= 6) highActive = true;
        }
      }

      if (direction ==  1 && lowActive  && !highActive) break;
      if (direction == -1 && highActive && !lowActive)  break;
    }

    delay(5);
  }

  motoron.setSpeedNow(LEFT_MOTOR,  0);
  motoron.setSpeedNow(RIGHT_MOTOR, 0);
  motoron.setSpeedNow(LEFT_MOTOR,  scaleSpeed(BASE_SPEED));
  motoron.setSpeedNow(RIGHT_MOTOR, scaleSpeed(BASE_SPEED));
  delay(JUNCTION_NUDGE_MS);
  motoron.setSpeedNow(LEFT_MOTOR,  0);
  motoron.setSpeedNow(RIGHT_MOTOR, 0);
}

// ─────────────────────────────────────────
// State-machine local flag — set true by handleJunction() after a turn
// completes when useStateMachine is on, consumed by navigationUpdate().
// ─────────────────────────────────────────
static bool junctionJustHandled = false;

// ─────────────────────────────────────────
// Junction handler. When useStateMachine is true, the turn direction
// comes from pendingJunctionDir (planner output) instead of the static
// junctionActions[] sequence. The legacy path is preserved verbatim.
// ─────────────────────────────────────────
void handleJunction() {
  if (inJunction) return;
  inJunction = true;

  int action;
  if (useStateMachine) {
    action = pendingJunctionDir;
    Serial.print("[SM] Junction → ");
    Serial.println(action == 0 ? "straight" :
                   action == 1 ? "right"    :
                   action ==-1 ? "left"     : "uturn");
  } else {
    if (junctionCount >= (int)(sizeof(junctionActions) / sizeof(junctionActions[0]))) {
      motoron.setSpeedNow(LEFT_MOTOR,  0);
      motoron.setSpeedNow(RIGHT_MOTOR, 0);
      Serial.println("All junctions complete. Stopped.");
      return;
    }
    action = junctionActions[junctionCount++];
    Serial.print("Junction "); Serial.print(junctionCount);
    Serial.print(" → "); Serial.println(action == 0 ? "straight" : action == 1 ? "right" : "left");
  }

  motoron.setSpeedNow(LEFT_MOTOR,  scaleSpeed(BASE_SPEED));
  motoron.setSpeedNow(RIGHT_MOTOR, scaleSpeed(BASE_SPEED));
  delay(JUNCTION_FORWARD_MS);

  if (action != 0) spinUntilLine(action);

  delay(100);
  currentKP = KP_AGGRESSIVE;
  junctionExitTime = millis();
  inJunction = false;

  if (useStateMachine) {
    robotFacing = facingAfterTurn(robotFacing, action);
    junctionJustHandled = true;
  }
}

// ─────────────────────────────────────────
// Main line follow tick — call each loop
// ─────────────────────────────────────────
void followLine() {
  uint16_t calibratedVals[IR_SENSOR_COUNT];
  long avg, sum;
  readSensors(calibratedVals, avg, sum);

  if (currentKP == KP_AGGRESSIVE && millis() - junctionExitTime > AGGRESSIVE_DURATION_MS) {
    currentKP = KP;
  }

  LineState state = getLineState(calibratedVals, sum);

  switch (state) {
    case LINE_NORMAL: {
      lastPosition   = avg / sum;
      int error      = lastPosition - LINE_CENTER;
      int correction = (int)(currentKP * error);
      motoron.setSpeedNow(LEFT_MOTOR,  scaleSpeed(constrain(BASE_SPEED + correction, -800, 800)));
      motoron.setSpeedNow(RIGHT_MOTOR, scaleSpeed(constrain(BASE_SPEED - correction, -800, 800)));
      break;
    }
    case LINE_LOST:
      motoron.setSpeedNow(LEFT_MOTOR,  0);
      motoron.setSpeedNow(RIGHT_MOTOR, 0);
      break;

    case LINE_JUNCTION_LEFT:
    case LINE_JUNCTION_RIGHT:
    case LINE_JUNCTION_BOTH:
      handleJunction();
      break;
  }
}

// ─────────────────────────────────────────
// Reset junction state (e.g. new run)
// ─────────────────────────────────────────
void resetJunctions() {
  junctionCount    = 0;
  inJunction       = false;
  currentKP        = KP;
  junctionExitTime = 0;
}

// ─────────────────────────────────────────
// Tag map. Zero-init means every cell starts as TAG_UNKNOWN.
// Owned here, extern in globals.h.
// ─────────────────────────────────────────
TagState tagMap[9][9];

// ─────────────────────────────────────────
// Facing / direction helpers
// Encoding: NORTH=0, EAST=1, SOUTH=2, WEST=3 → CW around the compass.
// Return codes for turn direction: -1=left, 0=straight, 1=right, 2=uturn.
// ─────────────────────────────────────────
int getTurnDir(Facing current, Facing desired) {
  int diff = ((int)desired - (int)current + 4) % 4;
  switch (diff) {
    case 0: return  0;
    case 1: return  1;
    case 2: return  2;
    case 3: return -1;
  }
  return 0;
}

Facing facingAfterTurn(Facing current, int dir) {
  return (Facing)(((int)current + dir + 4) % 4);
}

// Assumes `from` and `to` are adjacent (Manhattan distance == 1).
// If they aren't, row deltas take precedence; falls back to NORTH on equal cells.
Facing facingToward(GridPos from, GridPos to) {
  if (to.row < from.row) return NORTH;
  if (to.row > from.row) return SOUTH;
  if (to.col > from.col) return EAST;
  if (to.col < from.col) return WEST;
  return NORTH;
}

// ─────────────────────────────────────────
// A* on the 9x9 grid.
// Manhattan heuristic, uniform unit edge costs, no obstacles modeled yet.
// Returns the first step along the optimal path via `next`.
// All scratch arrays are static so we never touch the heap.
// ─────────────────────────────────────────
bool aStarNextStep(GridPos from, GridPos to, GridPos& next) {
  if (!from.valid() || !to.valid()) return false;
  if (from.equals(to)) return false;

  static int    gCost[81];
  static int    fCost[81];
  static int8_t parent[81];
  static bool   openSet[81];
  static bool   closed[81];

  for (int i = 0; i < 81; i++) {
    gCost[i]   = INT_MAX;
    fCost[i]   = INT_MAX;
    parent[i]  = -1;
    openSet[i] = false;
    closed[i]  = false;
  }

  const int startIdx = from.row * 9 + from.col;
  const int goalIdx  = to.row   * 9 + to.col;

  gCost[startIdx]   = 0;
  fCost[startIdx]   = abs(from.row - to.row) + abs(from.col - to.col);
  openSet[startIdx] = true;

  // Neighbour offsets: N, S, E, W.
  static const int8_t dr[4] = { -1,  1,  0,  0 };
  static const int8_t dc[4] = {  0,  0,  1, -1 };

  while (true) {
    // Linear-scan min-f extraction — 81 nodes, no need for a heap.
    int cur = -1;
    int bestF = INT_MAX;
    for (int i = 0; i < 81; i++) {
      if (openSet[i] && fCost[i] < bestF) {
        bestF = fCost[i];
        cur = i;
      }
    }
    if (cur == -1) return false;     // open set drained → no path
    if (cur == goalIdx) break;

    openSet[cur] = false;
    closed[cur]  = true;

    const int cr = cur / 9;
    const int cc = cur % 9;

    for (int d = 0; d < 4; d++) {
      const int nr = cr + dr[d];
      const int nc = cc + dc[d];
      if (nr < 0 || nr >= 9 || nc < 0 || nc >= 9) continue;
      // Future obstacle hook: when a TAG_BLOCKED state is added, skip the
      // neighbour here (and clear it when the block lifts).
      const int nIdx = nr * 9 + nc;
      if (closed[nIdx]) continue;

      const int tentativeG = gCost[cur] + 1;
      if (tentativeG < gCost[nIdx]) {
        parent[nIdx]  = (int8_t)cur;
        gCost[nIdx]   = tentativeG;
        fCost[nIdx]   = tentativeG + abs(nr - to.row) + abs(nc - to.col);
        openSet[nIdx] = true;
      }
    }
  }

  // Walk back from the goal until we find the cell whose parent is the start.
  int step = goalIdx;
  while (parent[step] != (int8_t)startIdx) {
    if (parent[step] < 0) return false;   // defensive — shouldn't trigger
    step = parent[step];
  }
  next.row = (int8_t)(step / 9);
  next.col = (int8_t)(step % 9);
  return true;
}

// ─────────────────────────────────────────
// Pick the next planting target.
// Priority: TAG_FERTILE > TAG_UNKNOWN; TAG_INFERTILE / TAG_PLANTED ignored.
// Within a tier, lowest Manhattan distance wins. Returns false if no
// candidates remain (run is finished — caller should head to Tunnel A).
// ─────────────────────────────────────────
bool selectNextTarget(GridPos from, GridPos& target) {
  if (!from.valid()) return false;

  int  bestPriority = INT_MAX;
  int  bestDist     = INT_MAX;
  bool found        = false;
  GridPos best{-1, -1};

  for (int r = 0; r < 9; r++) {
    for (int c = 0; c < 9; c++) {
      const TagState s = tagMap[r][c];
      int priority;
      if      (s == TAG_FERTILE) priority = 0;
      else if (s == TAG_UNKNOWN) priority = 1;
      else continue;

      const int dist = abs(from.row - r) + abs(from.col - c);
      if (priority < bestPriority ||
         (priority == bestPriority && dist < bestDist)) {
        bestPriority = priority;
        bestDist     = dist;
        best.row     = (int8_t)r;
        best.col     = (int8_t)c;
        found        = true;
      }
    }
  }

  if (!found) return false;
  target = best;
  return true;
}

// ─────────────────────────────────────────
// State-machine globals (definitions; externs in globals.h).
// `tagMap` is defined further up alongside the helpers — keeping it adjacent
// to selectNextTarget() since that's where it's primarily read.
// ─────────────────────────────────────────
NavState navState           = NAV_DISABLED;
GridPos  robotPos           = { -1, -1 };
GridPos  targetPos          = { -1, -1 };
Facing   robotFacing        = NORTH;
int      seedsRemaining     = SEED_COUNT;
int      pendingJunctionDir = 0;

// File-local bookkeeping for the state machine.
static GridPos       lastConfirmedPos = { -1, -1 };   // last RFID-confirmed cell, for calib
static unsigned long atTagEnteredMs   = 0;            // for FERTILE_REPLY_TIMEOUT_MS
static bool          rightHalfDriving = false;        // right-half hop in progress

// ─────────────────────────────────────────
// Step robotPos one cell in robotFacing's direction.
// ─────────────────────────────────────────
static void advancePosOneCell() {
  switch (robotFacing) {
    case NORTH: robotPos.row--; break;
    case SOUTH: robotPos.row++; break;
    case EAST:  robotPos.col++; break;
    case WEST:  robotPos.col--; break;
  }
}

// ─────────────────────────────────────────
// Re-plan the next turn direction based on current tagMap state.
// Sets targetPos and pendingJunctionDir as side effects.
// ─────────────────────────────────────────
static void replanNextDir() {
  GridPos next;
  if (!selectNextTarget(robotPos, targetPos)) {
    // No candidates remain — caller decides what to do (RETURN/PARKED).
    return;
  }
  if (aStarNextStep(robotPos, targetPos, next)) {
    const Facing want = facingToward(robotPos, next);
    pendingJunctionDir = getTurnDir(robotFacing, want);
  }
}

// ─────────────────────────────────────────
// Try to read an RFID tag at the current node. On success, kick off the
// fertility query and transition to NAV_AT_TAG. On miss, re-plan.
// ─────────────────────────────────────────
static void tryRfidAtNode() {
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    char uidStr[32] = "";
    for (byte i = 0; i < rfid.uid.size; i++) {
      char byteStr[3];
      snprintf(byteStr, sizeof(byteStr), "%02X", rfid.uid.uidByte[i]);
      strcat(uidStr, byteStr);
    }
    rfid.PICC_HaltA();

    clearFertileResult();
    sendIsFertile(uidStr);
    navState       = NAV_AT_TAG;
    atTagEnteredMs = millis();
  } else {
    // Tag miss — continue planning toward the current target.
    replanNextDir();
  }
}

// ─────────────────────────────────────────
// NAV_ARENA_NAV tick: left half uses line-following + junction detection,
// right half uses gyro heading-lock + encoder dead-reckoning.
// ─────────────────────────────────────────
static void navArenaTick() {
  // Obstacle-avoidance trigger. Uses the cached forward reading (refreshed
  // at the top of main loop() each tick) so we don't double-ping the sensor.
  // The top-of-loop check handles forward < OBSTACLE_STOP_CM (8cm) by
  // pausing motion entirely; this trigger handles the 8..20cm band by
  // diverting to a sidestep instead of stopping.
  if (lastForwardDistanceCm >= 0.0f &&
      lastForwardDistanceCm < (float)OBSTACLE_AVOID_CM) {
    motoron.setSpeedNow(LEFT_MOTOR,  0);
    motoron.setSpeedNow(RIGHT_MOTOR, 0);
    rightHalfDriving = false;
    endHopHeading();
    Serial.print("[NAV] obstacle at ");
    Serial.print(lastForwardDistanceCm);
    Serial.println(" cm — entering NAV_AVOID_OBSTACLE");
    navState = NAV_AVOID_OBSTACLE;
    return;
  }

  if (robotPos.col <= LEFT_HALF_MAX_COL) {
    // Left half — followLine() drives, handleJunction() sets junctionJustHandled.
    followLine();
    if (junctionJustHandled) {
      junctionJustHandled = false;
      advancePosOneCell();
      tryRfidAtNode();
    }
  } else {
    // Right half — no line, so dead-reckon one cell at a time.
    if (!rightHalfDriving) {
      encoderResetHop();
      resetHopHeading();
      motoron.setSpeedNow(LEFT_MOTOR,  scaleSpeed(BASE_SPEED));
      motoron.setSpeedNow(RIGHT_MOTOR, scaleSpeed(BASE_SPEED));
      rightHalfDriving = true;
    } else {
      // updateHopHeading() is now called from main loop(); just consume its result.
      applyHeadingCorrection();
      if (nearNextNode()) {
        motoron.setSpeedNow(LEFT_MOTOR,  0);
        motoron.setSpeedNow(RIGHT_MOTOR, 0);
        rightHalfDriving = false;
        endHopHeading();
        advancePosOneCell();
        tryRfidAtNode();
      }
    }
  }
}

// ─────────────────────────────────────────
// NAV_AT_TAG tick: wait for isFertileReply, update map + position,
// decide between planting and continuing.
// ─────────────────────────────────────────
static void navAtTagTick() {
  wifiPoll();

  if (fertileResult.received) {
    const GridPos newPos = { (int8_t)fertileResult.y, (int8_t)fertileResult.x };

    if (newPos.valid()) {
      TagState s;
      if      (fertileResult.planted) s = TAG_PLANTED;
      else if (fertileResult.fertile) s = TAG_FERTILE;
      else                            s = TAG_INFERTILE;
      tagMap[newPos.row][newPos.col] = s;

      // Calibration: previous RFID fix → this one. calibRecordHop itself
      // filters by Manhattan distance, so non-adjacent hops are silently
      // dropped (e.g. when we re-acquire after a miss or a teleport).
      if (lastConfirmedPos.valid()) {
        calibRecordHop(lastConfirmedPos, newPos);
      }
      robotPos         = newPos;
      lastConfirmedPos = newPos;
    }

    if (fertileResult.fertile && !fertileResult.planted && seedsRemaining > 0) {
      navState = NAV_PLANTING;
    } else {
      replanNextDir();
      navState = NAV_ARENA_NAV;
    }
  } else if (millis() - atTagEnteredMs > FERTILE_REPLY_TIMEOUT_MS) {
    Serial.println("[SM] isFertileReply timed out — resuming arena nav.");
    navState = NAV_ARENA_NAV;
  }
}

// ─────────────────────────────────────────
// NAV_PLANTING tick: blocking sweep, report, decrement, transition.
// ─────────────────────────────────────────
static void navPlantingTick() {
  sweepTo(MAX_ANGLE, MIN_ANGLE);
  sweepTo(MIN_ANGLE, MAX_ANGLE);
  sendPlanted(fertileResult.tagId);

  if (robotPos.valid()) {
    tagMap[robotPos.row][robotPos.col] = TAG_PLANTED;
  }
  seedsRemaining--;
  Serial.print("[SM] Planted. Seeds remaining: ");
  Serial.println(seedsRemaining);

  if (seedsRemaining == 0) {
    // TODO: extend with return-to-base routing (tunnel A entrance, etc.).
    navState = NAV_PARKED;
  } else {
    replanNextDir();
    navState = NAV_ARENA_NAV;
  }
}

// ─────────────────────────────────────────
// Disable-side cleanup. main.ino calls this when it gates the loop on
// !isEnabled; we touch file-local state that main.ino can't reach directly.
// Idempotent — safe to call every tick while disabled.
// ─────────────────────────────────────────
void handleNavDisable() {
  if (navState != NAV_DISABLED) {
    Serial.println("[NAV] disabled (isEnabled=false) — holding");
    navState         = NAV_DISABLED;
    rightHalfDriving = false;
    endHopHeading();
  }
}

// ─────────────────────────────────────────
// String helpers for `state`, `tag`, `pos` debug commands.
// ─────────────────────────────────────────
const char* navStateStr(NavState s) {
  switch (s) {
    case NAV_DISABLED:       return "NAV_DISABLED";
    case NAV_LINE_FOLLOW:    return "NAV_LINE_FOLLOW";
    case NAV_ARENA_NAV:      return "NAV_ARENA_NAV";
    case NAV_AT_TAG:         return "NAV_AT_TAG";
    case NAV_PLANTING:       return "NAV_PLANTING";
    case NAV_WALL_FOLLOW:    return "NAV_WALL_FOLLOW";
    case NAV_AVOID_OBSTACLE: return "NAV_AVOID_OBSTACLE";
    case NAV_PARKED:         return "NAV_PARKED";
  }
  return "?";
}

const char* facingStr(Facing f) {
  switch (f) {
    case NORTH: return "NORTH";
    case EAST:  return "EAST";
    case SOUTH: return "SOUTH";
    case WEST:  return "WEST";
  }
  return "?";
}

const char* tagStateStr(TagState s) {
  switch (s) {
    case TAG_UNKNOWN:   return "UNKNOWN";
    case TAG_FERTILE:   return "FERTILE";
    case TAG_INFERTILE: return "INFERTILE";
    case TAG_PLANTED:   return "PLANTED";
  }
  return "?";
}

// ─────────────────────────────────────────
// Between-step abort check for NAV_AVOID_OBSTACLE. Called between blocking
// turn/drive calls. Services wifi so the heartbeat survives the ~3s
// maneuver, then aborts to NAV_DISABLED on either disable or a real
// collision-range forward reading.
// Returns true when the caller should bail out of the maneuver.
// ─────────────────────────────────────────
static bool avoidStepAbort() {
  wifiLoop();   // keep heartbeat / LED alive during the blocking sequence
  if (!isEnabled) {
    motoron.setSpeedNow(LEFT_MOTOR,  0);
    motoron.setSpeedNow(RIGHT_MOTOR, 0);
    handleNavDisable();
    Serial.println("[AVOID] aborting — robot disabled mid-maneuver");
    return true;
  }
  const float fwd = getDistanceCM(SENSOR_FORWARD);
  if (fwd >= 0.0f && fwd < (float)OBSTACLE_STOP_CM) {
    motoron.setSpeedNow(LEFT_MOTOR,  0);
    motoron.setSpeedNow(RIGHT_MOTOR, 0);
    navState = NAV_DISABLED;
    Serial.print("[AVOID] aborting — forward at ");
    Serial.print(fwd);
    Serial.println(" cm");
    return true;
  }
  return false;
}

// ─────────────────────────────────────────
// NAV_AVOID_OBSTACLE: simple fixed sidestep around a forward obstacle.
// Runs the whole sequence in one tick (turnDegrees + moveForward are
// blocking by design). Step 5 is a literal interpretation of the spec —
// after step 3 the robot is already at original heading, so step 5 is a
// turnDegrees(0) no-op; this keeps cardinal orientation correct for the
// state machine's heading tracking. If you want a true U-shape detour
// (return to original lane), add a sidestep-back drive between steps 4
// and 5 and change step 5 to turn opposite direction of step 1.
// ─────────────────────────────────────────
static void navAvoidObstacleTick() {
  const float leftDist  = getDistanceCM(SENSOR_LEFT);
  const float rightDist = getDistanceCM(SENSOR_RIGHT);

  // Direction: +1 = right (CW), -1 = left (CCW). Prefer the side that's
  // out of range (open space); on ties or both valid, pick the larger.
  int dir;
  if      (leftDist  < 0 && rightDist < 0) dir = +1;
  else if (leftDist  < 0)                  dir = -1;
  else if (rightDist < 0)                  dir = +1;
  else                                     dir = (rightDist > leftDist) ? +1 : -1;

  Serial.print("[AVOID] sidestepping ");
  Serial.print(dir > 0 ? "right" : "left");
  Serial.print(" (L="); Serial.print(leftDist);
  Serial.print(" R=");  Serial.print(rightDist);
  Serial.println(")");

  // Step 1: turn 90° toward the more-clear side.
  turnDegrees(dir * 90.0f);
  if (avoidStepAbort()) return;

  // Step 2: sidestep.
  moveForward(OBSTACLE_SIDESTEP_MS);
  if (avoidStepAbort()) return;

  // Step 3: turn 90° back toward original heading.
  turnDegrees(-dir * 90.0f);
  if (avoidStepAbort()) return;

  // Step 4: drive past the obstacle.
  moveForward(OBSTACLE_FORWARD_MS);
  if (avoidStepAbort()) return;

  // Step 5: per spec — "turn 90° to restore original heading". Robot is
  // already at original heading after step 3, so this is a no-op. Keeping
  // the call (with 0°) documents the spec literally.
  turnDegrees(0);
  if (avoidStepAbort()) return;

  Serial.println("[AVOID] done — returning to NAV_ARENA_NAV");
  navState = NAV_ARENA_NAV;
}

// ─────────────────────────────────────────
// Tunnel wall-following.
// Non-blocking — one tick of sampling + proportional correction per call.
// Uses a 5-sample rolling window per side; out-of-range readings (-1) are
// excluded from the running average. Exits to NAV_ARENA_NAV when the
// forward ultrasonic indicates the tunnel has opened up.
// ─────────────────────────────────────────
void wallFollow() {
  static float leftBuf[5]  = { -1, -1, -1, -1, -1 };
  static float rightBuf[5] = { -1, -1, -1, -1, -1 };
  static int   wallIdx     = 0;

  // Exit check first — uses the cached forward reading from this tick's
  // top-of-loop obstacle check, so no extra ultrasonic ping here.
  if (lastForwardDistanceCm >= 0.0f &&
      lastForwardDistanceCm < (float)WALL_FORWARD_CLEAR_CM) {
    motoron.setSpeedNow(LEFT_MOTOR,  0);
    motoron.setSpeedNow(RIGHT_MOTOR, 0);
    Serial.println("tunnel exit detected");
    navState = NAV_ARENA_NAV;
    return;
  }

  // Sample LEFT and RIGHT once per tick into the circular buffer.
  leftBuf[wallIdx]  = getDistanceCM(SENSOR_LEFT);
  rightBuf[wallIdx] = getDistanceCM(SENSOR_RIGHT);
  wallIdx = (wallIdx + 1) % 5;

  // Average only the valid (>=0) samples on each side.
  float leftSum  = 0.0f, rightSum  = 0.0f;
  int   leftN    = 0,    rightN    = 0;
  for (int i = 0; i < 5; i++) {
    if (leftBuf[i]  >= 0.0f) { leftSum  += leftBuf[i];  leftN++; }
    if (rightBuf[i] >= 0.0f) { rightSum += rightBuf[i]; rightN++; }
  }

  // Sign convention: positive `error` means "turn left" (left wheel slower,
  // right wheel faster). Both half-errors below resolve to that convention.
  float error    = 0.0f;
  bool  haveError = false;
  if (leftN > 0 && rightN > 0) {
    const float leftErr  = (leftSum  / leftN)  - WALL_FOLLOW_TARGET_CM; // too far left → +
    const float rightErr = WALL_FOLLOW_TARGET_CM - (rightSum / rightN); // too close right → +
    error    = (leftErr + rightErr) * 0.5f;
    haveError = true;
  } else if (leftN > 0) {
    error    = (leftSum / leftN) - WALL_FOLLOW_TARGET_CM;
    haveError = true;
  } else if (rightN > 0) {
    error    = WALL_FOLLOW_TARGET_CM - (rightSum / rightN);
    haveError = true;
  }

  const int correction = haveError ? (int)(WALL_KP * error) : 0;
  const int leftSpeed  = constrain(BASE_SPEED - correction, 0, 800);
  const int rightSpeed = constrain(BASE_SPEED + correction, 0, 800);
  motoron.setSpeedNow(LEFT_MOTOR,  scaleSpeed(leftSpeed));
  motoron.setSpeedNow(RIGHT_MOTOR, scaleSpeed(rightSpeed));
}

// ─────────────────────────────────────────
// Top-level dispatcher. Called from main loop() when useStateMachine is true
// AND isEnabled is true — main.ino owns the disable side now.
// ─────────────────────────────────────────
void navigationUpdate() {
  // Defense-in-depth: if main.ino somehow calls us while disabled, hold.
  if (!isEnabled) {
    motoron.setSpeedNow(LEFT_MOTOR,  0);
    motoron.setSpeedNow(RIGHT_MOTOR, 0);
    handleNavDisable();
    return;
  }

  // Re-enable: drop back into arena nav so the planner re-evaluates from the
  // last known robotPos. If you want manual re-engagement instead, remove
  // this block — `nav` from the serial console can still wake the robot up.
  if (navState == NAV_DISABLED) {
    Serial.println("[NAV] re-enabled → NAV_ARENA_NAV");
    navState = NAV_ARENA_NAV;
  }

  // Stub-state prints fire once per entry, not every tick.
  static NavState lastTickState = (NavState)255;
  const bool justEntered = (lastTickState != navState);
  lastTickState = navState;

  switch (navState) {
    case NAV_DISABLED:
      // Unreachable below the gate, but kept defensively.
      motoron.setSpeedNow(LEFT_MOTOR,  0);
      motoron.setSpeedNow(RIGHT_MOTOR, 0);
      return;

    case NAV_LINE_FOLLOW:
      // Legacy test mode — line follow only, no state transitions.
      followLine();
      return;

    case NAV_ARENA_NAV:
      navArenaTick();
      return;

    case NAV_AT_TAG:
      navAtTagTick();
      return;

    case NAV_PLANTING:
      navPlantingTick();
      return;

    case NAV_AVOID_OBSTACLE:
      if (justEntered) Serial.println("[NAV] avoid-obstacle active");
      navAvoidObstacleTick();
      return;

    case NAV_WALL_FOLLOW:
      if (justEntered) Serial.println("[NAV] wall-follow active");
      wallFollow();
      return;

    case NAV_PARKED:
      motoron.setSpeedNow(LEFT_MOTOR,  0);
      motoron.setSpeedNow(RIGHT_MOTOR, 0);
      if (justEntered) Serial.println("[NAV] parked");
      return;
  }
}
