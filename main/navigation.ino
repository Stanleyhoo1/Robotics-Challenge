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
      // Sensor 0 is mounted on the bot's RIGHT, sensor 8 on the LEFT (see
      // spinUntilLine). Error is defined so line-on-right is POSITIVE → left
      // wheel faster, right wheel slower → steers toward the line.
      int error      = LINE_CENTER - (int)lastPosition;
      int correction = (int)(currentKP * error);
      motoron.setSpeedNow(LEFT_MOTOR,  scaleSpeed(constrain(BASE_SPEED + correction, LINE_FOLLOW_MIN_SPEED, 800)));
      motoron.setSpeedNow(RIGHT_MOTOR, scaleSpeed(constrain(BASE_SPEED - correction, LINE_FOLLOW_MIN_SPEED, 800)));
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
      if (tagMap[nr][nc] == TAG_BLOCKED) continue;   // obstacle — route around
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
NavState navState           = NAV_BASE_TO_FIRST_JUNCTION;   // boot at start of base-exit; button press just enables it
GridPos  robotPos           = { -1, -1 };
GridPos  targetPos          = { -1, -1 };
Facing   robotFacing        = NORTH;
int      seedsRemaining     = SEED_COUNT;
int      pendingJunctionDir = 0;

// File-local bookkeeping for the state machine.
static GridPos       lastConfirmedPos = { -1, -1 };   // last RFID-confirmed cell, for calib
static unsigned long atTagEnteredMs   = 0;            // for FERTILE_REPLY_TIMEOUT_MS
static bool          deadReckonDriving = false;       // no-line-zone hop in progress
static bool          postTagIntentPlanting = false;   // NAV_POST_TAG_NUDGE: plant vs turn
static bool          nudgeStarted          = false;   // NAV_POST_TAG_NUDGE: motors armed
static bool          returning             = false;   // seeds exhausted → route to AIRLOCK_B
static unsigned long clearanceRetryDeadlineMs = 0;     // resend open-airlock-X while waiting for clearance

// Airlock B position as a GridPos — used in several arrival-check sites.
static inline GridPos airlockBPos() {
  return { (int8_t)AIRLOCK_B_ROW, (int8_t)AIRLOCK_B_COL };
}

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
// In return mode (seeds exhausted), the target is fixed to AIRLOCK_B.
// ─────────────────────────────────────────
static void replanNextDir() {
  GridPos next;
  if (returning) {
    targetPos = airlockBPos();
  } else if (!selectNextTarget(robotPos, targetPos)) {
    // No candidates remain — caller decides what to do (RETURN/PARKED).
    return;
  }
  if (aStarNextStep(robotPos, targetPos, next)) {
    const Facing want = facingToward(robotPos, next);
    pendingJunctionDir = getTurnDir(robotFacing, want);
  } else {
    // from == to: we're at the target. Caller checks for return-arrival.
    pendingJunctionDir = 0;
  }
}

// ─────────────────────────────────────────
// Non-blocking RFID poll. Returns true and transitions to NAV_AT_TAG (after
// kicking off the fertility query) if a tag was read. Returns false on miss
// — caller keeps driving forward.
// ─────────────────────────────────────────
static bool pollRfidAndQueue() {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) return false;

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
  return true;
}

// Forward declaration — followLineBase is defined later in this file but
// used here. Static functions in .ino files aren't reliably picked up by
// Arduino's auto-prototype generator, so we declare it explicitly.
static LineState followLineBase();

// ─────────────────────────────────────────
// NAV_ARENA_NAV tick: drive forward and poll RFID every tick. The RFID is
// the position truth and the trigger for all decisions — line junctions are
// not acted on here, they're just absorbed by the PID. On the line zone the
// PID keeps us centred on the lane; in the no-line zone gyro heading-lock +
// encoders carry us between cells with nearNextNode() as a miss-fallback.
// ─────────────────────────────────────────
static void navArenaTick() {
  // Return-mode short-circuit: if we re-enter arena nav while already on
  // Airlock B (e.g. post-plant at the airlock cell, or post-AT_TAG replan
  // with no further step needed), hand off to the door handshake directly.
  if (returning && robotPos.equals(airlockBPos())) {
    motoron.setSpeedNow(LEFT_MOTOR,  0);
    motoron.setSpeedNow(RIGHT_MOTOR, 0);
    deadReckonDriving = false;
    endHopHeading();
    Serial.println("[NAV] return-mode at Airlock B — handing off to NAV_AT_AIRLOCK_B");
    navState = NAV_AT_AIRLOCK_B;
    return;
  }

  // Obstacle-avoidance trigger. Uses the cached forward reading (refreshed
  // at the top of main loop() each tick) so we don't double-ping the sensor.
  // The top-of-loop check handles forward < OBSTACLE_STOP_CM (8cm) by
  // pausing motion entirely; this trigger handles the 8..20cm band by
  // diverting to a sidestep instead of stopping.
  if (lastForwardDistanceCm >= 0.0f &&
      lastForwardDistanceCm < (float)OBSTACLE_AVOID_CM) {
    motoron.setSpeedNow(LEFT_MOTOR,  0);
    motoron.setSpeedNow(RIGHT_MOTOR, 0);
    deadReckonDriving = false;
    endHopHeading();
    Serial.print("[NAV] obstacle at ");
    Serial.print(lastForwardDistanceCm);
    Serial.println(" cm — entering NAV_AVOID_OBSTACLE");
    navState = NAV_AVOID_OBSTACLE;
    return;
  }

  // RFID is the truth — poll every tick. On hit, stop and hand off to
  // NAV_AT_TAG. The fertility reply will overwrite robotPos with ground truth.
  if (pollRfidAndQueue()) {
    motoron.setSpeedNow(LEFT_MOTOR,  0);
    motoron.setSpeedNow(RIGHT_MOTOR, 0);
    deadReckonDriving = false;
    endHopHeading();
    return;
  }

  if (robotPos.row >= LINE_ZONE_MIN_ROW) {
    // Line zone — PID only. Junction classification is ignored; the wide
    // sensor coverage of a junction crossing averages back near LINE_CENTER
    // so the robot drives straight through. RFID detection ends the hop.
    (void)followLineBase();
  } else {
    // No-line zone — gyro heading-lock + encoders. nearNextNode() is a
    // miss-fallback: if we've travelled a full cell without an RFID hit,
    // stop, advance the dead-reckoned position, log it, and start the next
    // hop. The next tick's RFID poll will pick up the next tag.
    if (!deadReckonDriving) {
      encoderResetHop();
      resetHopHeading();
      motoron.setSpeedNow(LEFT_MOTOR,  scaleSpeed(BASE_SPEED));
      motoron.setSpeedNow(RIGHT_MOTOR, scaleSpeed(BASE_SPEED));
      deadReckonDriving = true;
    } else {
      applyHeadingCorrection();
      if (nearNextNode()) {
        motoron.setSpeedNow(LEFT_MOTOR,  0);
        motoron.setSpeedNow(RIGHT_MOTOR, 0);
        deadReckonDriving = false;
        endHopHeading();
        advancePosOneCell();
        sendStatus("rfid_miss_dead_reckon_advance");
        // Return-mode dead-reckon arrival at Airlock B (RFID never read).
        if (returning && robotPos.equals(airlockBPos())) {
          Serial.println("[NAV] dead-reckon arrival at Airlock B");
          navState = NAV_AT_AIRLOCK_B;
          return;
        }
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

    // Return-mode arrival check: if we're at Airlock B with seeds exhausted,
    // skip the planting decision entirely and start the door handshake.
    if (returning && robotPos.equals(airlockBPos())) {
      motoron.setSpeedNow(LEFT_MOTOR,  0);
      motoron.setSpeedNow(RIGHT_MOTOR, 0);
      Serial.println("[SM] return-mode arrival at Airlock B");
      navState = NAV_AT_AIRLOCK_B;
      return;
    }

    if (fertileResult.fertile && !fertileResult.planted && seedsRemaining > 0) {
      // Nudge forward so the seed dispenser sits over the hole, then plant.
      postTagIntentPlanting = true;
      nudgeStarted          = false;
      navState              = NAV_POST_TAG_NUDGE;
    } else {
      // No plant — ask the planner where we're going next. Straight means
      // keep driving forward without a nudge; a turn means nudge forward to
      // centre the pivot, then rotate.
      replanNextDir();
      if (pendingJunctionDir == 0) {
        navState = NAV_ARENA_NAV;
      } else {
        postTagIntentPlanting = false;
        nudgeStarted          = false;
        navState              = NAV_POST_TAG_NUDGE;
      }
    }
  } else if (millis() - atTagEnteredMs > FERTILE_REPLY_TIMEOUT_MS) {
    Serial.println("[SM] isFertileReply timed out — resuming arena nav.");
    navState = NAV_ARENA_NAV;
  }
}

// ─────────────────────────────────────────
// NAV_POST_TAG_NUDGE tick: drive forward POST_TAG_FORWARD_CM using the
// encoder hop counter, then dispatch by intent. Planting → NAV_PLANTING;
// turning → in-place turnDegrees + back to NAV_ARENA_NAV. The nudge uses
// the same encoder/ticksPerCm path as a full hop, so it falls back to
// TICKS_PER_CM_FALLBACK before calibration locks.
// ─────────────────────────────────────────
static void navPostTagNudgeTick() {
  if (!nudgeStarted) {
    encoderResetHop();
    motoron.setSpeedNow(LEFT_MOTOR,  scaleSpeed(BASE_SPEED));
    motoron.setSpeedNow(RIGHT_MOTOR, scaleSpeed(BASE_SPEED));
    nudgeStarted = true;
    return;
  }

  if (hopDistanceCm() < POST_TAG_FORWARD_CM) return;

  motoron.setSpeedNow(LEFT_MOTOR,  0);
  motoron.setSpeedNow(RIGHT_MOTOR, 0);
  nudgeStarted = false;

  if (postTagIntentPlanting) {
    navState = NAV_PLANTING;
    return;
  }

  // Turn intent — rotate in place to face the replanned step direction,
  // then hand off to NAV_ARENA_NAV so the next hop begins immediately.
  if (pendingJunctionDir != 0) {
    Serial.print("[NUDGE] turning ");
    Serial.print(pendingJunctionDir * 90);
    Serial.println(" degrees");
    turnDegrees((float)pendingJunctionDir * 90.0f);
    wifiLoop();
    if (!isEnabled) { handleNavDisable(); return; }
    robotFacing        = facingAfterTurn(robotFacing, pendingJunctionDir);
    pendingJunctionDir = 0;
  }
  navState = NAV_ARENA_NAV;
}

// ─────────────────────────────────────────
// NAV_PLANTING tick: blocking sweep, report, decrement, transition.
// Robot is already nudged POST_TAG_FORWARD_CM past the tag when we land
// here, so any turn the planner requests next can happen in place — no
// second nudge needed before rotating.
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
    Serial.println("[PLANT] seeds exhausted — entering return mode, target = Airlock B");
    returning = true;
    // Fall through to the replan/turn path below; replanNextDir routes to
    // AIRLOCK_B when `returning` is true.
  }

  replanNextDir();
  if (pendingJunctionDir != 0) {
    Serial.print("[PLANT] turning ");
    Serial.print(pendingJunctionDir * 90);
    Serial.println(" degrees toward next target");
    turnDegrees((float)pendingJunctionDir * 90.0f);
    wifiLoop();
    if (!isEnabled) { handleNavDisable(); return; }
    robotFacing        = facingAfterTurn(robotFacing, pendingJunctionDir);
    pendingJunctionDir = 0;
  }
  navState = NAV_ARENA_NAV;
}

// ─────────────────────────────────────────
// Disable-side cleanup. main.ino calls this when it gates the loop on
// !isEnabled; we touch file-local state that main.ino can't reach directly.
// Idempotent — safe to call every tick while disabled.
// ─────────────────────────────────────────
void handleNavDisable() {
  // navState is intentionally preserved — re-enable resumes whatever state
  // was active before. Only mid-hop / mid-turn bookkeeping that would go
  // stale gets cleared, so the state's next tick starts cleanly.
  static NavState printedFor = (NavState)255;
  if (printedFor != navState) {
    Serial.print("[NAV] disabled (isEnabled=false) — holding state=");
    Serial.println(navStateStr(navState));
    printedFor = navState;
  }
  deadReckonDriving = false;
  endHopHeading();
}

// ─────────────────────────────────────────
// String helpers for `state`, `tag`, `pos` debug commands.
// ─────────────────────────────────────────
const char* navStateStr(NavState s) {
  switch (s) {
    case NAV_DISABLED:                  return "NAV_DISABLED";
    case NAV_LINE_FOLLOW:               return "NAV_LINE_FOLLOW";
    case NAV_BASE_TO_FIRST_JUNCTION:    return "NAV_BASE_TO_FIRST_JUNCTION";
    case NAV_BASE_FIRST_TURN:           return "NAV_BASE_FIRST_TURN";
    case NAV_BASE_TO_TAG:               return "NAV_BASE_TO_TAG";
    case NAV_WAIT_EXIT_CLEARANCE:       return "NAV_WAIT_EXIT_CLEARANCE";
    case NAV_BASE_TO_SECOND_JUNCTION:   return "NAV_BASE_TO_SECOND_JUNCTION";
    case NAV_BASE_SECOND_TURN:          return "NAV_BASE_SECOND_TURN";
    case NAV_BASE_TO_LINE_LOST:         return "NAV_BASE_TO_LINE_LOST";
    case NAV_BASE_LINE_LOST_PAUSE:      return "NAV_BASE_LINE_LOST_PAUSE";
    case NAV_BASE_FORWARD_NUDGE:        return "NAV_BASE_FORWARD_NUDGE";
    case NAV_ARENA_NAV:                 return "NAV_ARENA_NAV";
    case NAV_AT_TAG:                    return "NAV_AT_TAG";
    case NAV_POST_TAG_NUDGE:            return "NAV_POST_TAG_NUDGE";
    case NAV_PLANTING:                  return "NAV_PLANTING";
    case NAV_WALL_FOLLOW:               return "NAV_WALL_FOLLOW";
    case NAV_AVOID_OBSTACLE:            return "NAV_AVOID_OBSTACLE";
    case NAV_AT_AIRLOCK_B:              return "NAV_AT_AIRLOCK_B";
    case NAV_WAIT_ENTER_CLEARANCE:      return "NAV_WAIT_ENTER_CLEARANCE";
    case NAV_TUNNEL_B_WALL_FOLLOW:      return "NAV_TUNNEL_B_WALL_FOLLOW";
    case NAV_BASE_RETURN:               return "NAV_BASE_RETURN";
    case NAV_PARKED:                    return "NAV_PARKED";
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

// Short human-readable description of what the robot is currently *doing* in
// each navState. Printed once per state entry by the dispatcher so the serial
// log reads as a play-by-play instead of just a list of enum names.
const char* navActivityStr(NavState s) {
  switch (s) {
    case NAV_DISABLED:                  return "stopped (disabled)";
    case NAV_LINE_FOLLOW:               return "line-following (legacy test mode)";
    case NAV_BASE_TO_FIRST_JUNCTION:    return "following line toward first junction";
    case NAV_BASE_FIRST_TURN:           return "turning at first junction";
    case NAV_BASE_TO_TAG:               return "following line toward RFID tag";
    case NAV_WAIT_EXIT_CLEARANCE:       return "holding for exitClearance";
    case NAV_BASE_TO_SECOND_JUNCTION:   return "following line toward T-junction";
    case NAV_BASE_SECOND_TURN:          return "turning at T-junction";
    case NAV_BASE_TO_LINE_LOST:         return "following line until it ends";
    case NAV_BASE_LINE_LOST_PAUSE:      return "paused — line lost in base";
    case NAV_BASE_FORWARD_NUDGE:        return "driving forward into tunnel mouth";
    case NAV_ARENA_NAV:                 return "arena nav — driving toward next tag";
    case NAV_AT_TAG:                    return "at RFID tag — waiting for fertility reply";
    case NAV_POST_TAG_NUDGE:            return "slight forward to centre on hole";
    case NAV_PLANTING:                  return "planting seed";
    case NAV_WALL_FOLLOW:               return "wall-following through tunnel A";
    case NAV_AVOID_OBSTACLE:            return "marking obstacle + replanning";
    case NAV_AT_AIRLOCK_B:              return "at Airlock B — rotating + opening door";
    case NAV_WAIT_ENTER_CLEARANCE:      return "holding for enterClearance";
    case NAV_TUNNEL_B_WALL_FOLLOW:      return "wall-following through tunnel B";
    case NAV_BASE_RETURN:               return "line-following back into base";
    case NAV_PARKED:                    return "parked";
  }
  return "?";
}

const char* tagStateStr(TagState s) {
  switch (s) {
    case TAG_UNKNOWN:   return "UNKNOWN";
    case TAG_FERTILE:   return "FERTILE";
    case TAG_INFERTILE: return "INFERTILE";
    case TAG_PLANTED:   return "PLANTED";
    case TAG_BLOCKED:   return "BLOCKED";
  }
  return "?";
}

// ─────────────────────────────────────────
// NAV_AVOID_OBSTACLE: mark the obstructed cell, replan, turn in place to
// face the new step direction, hand control back to NAV_ARENA_NAV. The
// A* planner already skips TAG_BLOCKED cells, so the replan naturally
// detours around the obstacle.
//
// This is a one-tick maneuver: one blocking turnDegrees call + transition.
// Heartbeat survives because the turn takes ~1s (well under the 1s
// HEARTBEAT_TIMEOUT_MS budget once you allow for the post-turn wifiLoop).
// ─────────────────────────────────────────
static void navAvoidObstacleTick() {
  // The blocked cell is the one immediately forward of robotPos in the
  // current heading. If we're off-grid (robotPos at the edge facing out),
  // skip the mark — there's no in-grid cell to flag.
  GridPos blocked = robotPos;
  switch (robotFacing) {
    case NORTH: blocked.row--; break;
    case SOUTH: blocked.row++; break;
    case EAST:  blocked.col++; break;
    case WEST:  blocked.col--; break;
  }

  if (blocked.valid()) {
    tagMap[blocked.row][blocked.col] = TAG_BLOCKED;
    Serial.print("[AVOID] marked (");
    Serial.print(blocked.row);
    Serial.print(",");
    Serial.print(blocked.col);
    Serial.println(") as TAG_BLOCKED");
  } else {
    Serial.println("[AVOID] obstacle off-grid — skipping tagMap update");
  }

  // Replan from current robotPos. selectNextTarget skips TAG_BLOCKED
  // automatically (falls into the default `continue` branch); aStarNextStep
  // skips it as a neighbour. So the new pendingJunctionDir naturally detours.
  replanNextDir();

  if (pendingJunctionDir == 0) {
    // Either replan failed (no candidates left) or the new step happens to
    // be the same direction we're facing — which shouldn't happen now that
    // the cell directly ahead is marked blocked. Fall through to ARENA_NAV
    // and let the state machine handle "no targets" later.
    Serial.println("[AVOID] no detour direction available — handing back to NAV_ARENA_NAV");
    navState = NAV_ARENA_NAV;
    return;
  }

  // Turn in place to face the replanned direction. pendingJunctionDir is
  // -1 (left), +1 (right), or +2 (uturn) — turnDegrees handles all three.
  Serial.print("[AVOID] turning ");
  Serial.print(pendingJunctionDir * 90);
  Serial.println(" degrees toward new step");
  turnDegrees((float)pendingJunctionDir * 90.0f);

  // Service wifi after the blocking turn so the heartbeat refreshes; bail
  // out cleanly if the operator killed us during the turn.
  wifiLoop();
  if (!isEnabled) {
    motoron.setSpeedNow(LEFT_MOTOR,  0);
    motoron.setSpeedNow(RIGHT_MOTOR, 0);
    handleNavDisable();
    return;
  }

  robotFacing = facingAfterTurn(robotFacing, pendingJunctionDir);
  pendingJunctionDir = 0;   // consumed in-place; the line-follower has nothing to do

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

  // Exit check: IR sees a line — we're in the arena. The door-wait case
  // (forward < OBSTACLE_STOP_CM) is handled by the top-of-loop pause in
  // main.ino, which simply skips this function until the door opens; from
  // wallFollow's perspective, nothing special needs to be done.
  {
    uint16_t calibratedVals[IR_SENSOR_COUNT];
    long avg, sum;
    readSensors(calibratedVals, avg, sum);
    if (sum >= IR_MIN_LINE_SUM) {
      motoron.setSpeedNow(LEFT_MOTOR,  0);
      motoron.setSpeedNow(RIGHT_MOTOR, 0);
      if (navState == NAV_TUNNEL_B_WALL_FOLLOW) {
        // Return path: just popped into base — hand off to line-follow.
        Serial.println("[WALL_B] line detected — entering NAV_BASE_RETURN");
        navState = NAV_BASE_RETURN;
      } else {
        // Exit path (NAV_WALL_FOLLOW): Tunnel A drops us at Airlock A. Seed
        // robotPos + robotFacing so the planner has a starting cell on arena
        // entry — the first RFID read (via the server's isFertileReply in
        // navAtTagTick) will overwrite robotPos with ground truth.
        if (!robotPos.valid()) {
          robotPos    = { (int8_t)AIRLOCK_A_ROW, (int8_t)AIRLOCK_A_COL };
          robotFacing = (Facing)ARENA_ENTRY_FACING_INT;
          Serial.print("[WALL] seeded robotPos to Airlock A (");
          Serial.print(robotPos.row); Serial.print(", ");
          Serial.print(robotPos.col); Serial.print("), facing ");
          Serial.println(facingStr(robotFacing));
        }
        Serial.println("[WALL] line detected — entering NAV_ARENA_NAV");
        navState = NAV_ARENA_NAV;
      }
      return;
    }
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
// Base-exit helpers + state ticks.
//
// Flow (set in motion by the `nav` serial command):
//   BASE_TO_FIRST_JUNCTION  → follow line, watch for any junction classification
//   BASE_FIRST_TURN         → blocking turnDegrees, exit-side turn
//   BASE_TO_TAG             → follow line, watch for RFID; tag triggers
//                             sendOpenAirlockA + transition
//   BASE_TO_SECOND_JUNCTION → follow line, watch for any junction
//   BASE_SECOND_TURN        → blocking turnDegrees, opposite of first turn
//   BASE_TO_LINE_LOST       → follow line, watch for LINE_LOST
//   BASE_FORWARD_NUDGE      → drive forward BASE_FORWARD_NUDGE_MS into tunnel
//   NAV_WALL_FOLLOW         → wall-follow until IR detects a line (arena)
//
// 90° line corners are tolerated by skipping handleJunction here — the
// inline PID below corrects through gentle curves; sharp 90° corners still
// risk losing the line and may need a dedicated handler later.
// ─────────────────────────────────────────

// PID-only line follow with no junction handling. Caller inspects the
// returned LineState to decide whether to keep going, turn, or stop.
static LineState followLineBase() {
  uint16_t calibratedVals[IR_SENSOR_COUNT];
  long avg = 0, sum = 0;
  readSensors(calibratedVals, avg, sum);

  if (sum >= IR_MIN_LINE_SUM) {
    lastPosition       = avg / sum;
    // Sensor 0 is mounted on the bot's RIGHT, sensor 8 on the LEFT (see
    // spinUntilLine). Error is defined so line-on-right is POSITIVE → left
    // wheel faster, right wheel slower → steers toward the line.
    const int error      = LINE_CENTER - (int)lastPosition;
    const int correction = (int)(KP * (float)error);
    motoron.setSpeedNow(LEFT_MOTOR,
      scaleSpeed(constrain(BASE_SPEED + correction, LINE_FOLLOW_MIN_SPEED, 800)));
    motoron.setSpeedNow(RIGHT_MOTOR,
      scaleSpeed(constrain(BASE_SPEED - correction, LINE_FOLLOW_MIN_SPEED, 800)));
  } else {
    // No line under the array — stop instead of drifting on the last
    // commanded PWM. The caller decides what comes next (pause state,
    // tunnel transition, replan, etc.) by inspecting the returned LineState.
    motoron.setSpeedNow(LEFT_MOTOR,  0);
    motoron.setSpeedNow(RIGHT_MOTOR, 0);
  }
  return getLineState(calibratedVals, sum);
}

// One-shot RFID read. Fills uidOut and returns true if a tag was present.
static bool readRfidNonBlocking(char* uidOut, size_t uidOutSize) {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) return false;
  uidOut[0] = '\0';
  for (byte i = 0; i < rfid.uid.size; i++) {
    char b[3];
    snprintf(b, sizeof(b), "%02X", rfid.uid.uidByte[i]);
    if (strlen(uidOut) + strlen(b) < uidOutSize) strcat(uidOut, b);
  }
  rfid.PICC_HaltA();
  return true;
}

static bool isJunctionState(LineState ls) {
  return ls == LINE_JUNCTION_LEFT
      || ls == LINE_JUNCTION_RIGHT
      || ls == LINE_JUNCTION_BOTH;
}

// Blocking pre-turn nudge + turn + brief post-turn forward nudge. Keeps the
// heartbeat alive across both motions and bails cleanly if the operator
// disables mid-sequence.
//
// Pre-turn nudge mirrors NAV_POST_TAG_NUDGE in the arena: drive forward
// POST_TAG_FORWARD_CM via the encoder hop counter so the wheel axis (= turn
// pivot) sits over the junction crossing before rotating. Post-turn nudge
// re-engages the line on the new branch since followLineBase holds motors at
// zero until it actually sees a line.
static bool baseTurnBlocking(float deg) {
  Serial.print("[BASE] pre-turn nudge ");
  Serial.print(POST_TAG_FORWARD_CM);
  Serial.println(" cm to centre on junction");
  encoderResetHop();
  motoron.setSpeedNow(LEFT_MOTOR,  scaleSpeed(BASE_SPEED));
  motoron.setSpeedNow(RIGHT_MOTOR, scaleSpeed(BASE_SPEED));
  while (hopDistanceCm() < POST_TAG_FORWARD_CM) {
    wifiLoop();
    if (!isEnabled) {
      motoron.setSpeedNow(LEFT_MOTOR,  0);
      motoron.setSpeedNow(RIGHT_MOTOR, 0);
      handleNavDisable();
      return false;
    }
    delay(5);
  }
  motoron.setSpeedNow(LEFT_MOTOR,  0);
  motoron.setSpeedNow(RIGHT_MOTOR, 0);

  turnDegrees(deg);
  wifiLoop();
  if (!isEnabled) {
    motoron.setSpeedNow(LEFT_MOTOR,  0);
    motoron.setSpeedNow(RIGHT_MOTOR, 0);
    handleNavDisable();
    return false;
  }
  motoron.setSpeedNow(LEFT_MOTOR,  scaleSpeed(BASE_SPEED));
  motoron.setSpeedNow(RIGHT_MOTOR, scaleSpeed(BASE_SPEED));
  delay(JUNCTION_NUDGE_MS);
  motoron.setSpeedNow(LEFT_MOTOR,  0);
  motoron.setSpeedNow(RIGHT_MOTOR, 0);
  return true;
}

// Spin in `direction` (-1=left, +1=right) up to `maxDeg` watching for line.
// Returns true if the line is acquired (motors stopped on the line), false
// on max rotation or operator disable. Heartbeat stays alive via wifiLoop.
static bool sweepForLine(int direction, float maxDeg) {
  int spinLeft  = (direction == -1) ? -BASE_SPEED :  BASE_SPEED;
  int spinRight = (direction == -1) ?  BASE_SPEED : -BASE_SPEED;
  motoron.setSpeedNow(LEFT_MOTOR,  scaleSpeed(spinLeft));
  motoron.setSpeedNow(RIGHT_MOTOR, scaleSpeed(spinRight));

  float accumulated = 0.0f;
  unsigned long lastTime = micros();

  while (fabsf(accumulated) < maxDeg) {
    imu.read();
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0f;
    lastTime = now;
    accumulated += -((imu.g.z - gyroZOffset) * GYRO_SENS) * dt;

    uint16_t cv[IR_SENSOR_COUNT];
    long avg, sum;
    readSensors(cv, avg, sum);
    if (sum >= IR_MIN_LINE_SUM) {
      motoron.setSpeedNow(LEFT_MOTOR,  0);
      motoron.setSpeedNow(RIGHT_MOTOR, 0);
      return true;
    }

    wifiLoop();
    if (!isEnabled) {
      motoron.setSpeedNow(LEFT_MOTOR,  0);
      motoron.setSpeedNow(RIGHT_MOTOR, 0);
      handleNavDisable();
      return false;
    }
    delay(5);
  }

  motoron.setSpeedNow(LEFT_MOTOR,  0);
  motoron.setSpeedNow(RIGHT_MOTOR, 0);
  return false;
}

// Blocking recovery for unexpected line loss in the base. Nudges forward
// POST_TAG_FORWARD_CM (same distance as junction pre-turn and planting), then
// sweeps left 90°, then right 180° (so ±90° from start), watching for the
// line. Returns true if the line is acquired at any point; the caller's
// followLineBase will resume PID on the next tick.
static bool baseLineLostRecovery() {
  Serial.print("[RECOVER] forward ");
  Serial.print(POST_TAG_FORWARD_CM);
  Serial.println(" cm then ±90° sweep");

  encoderResetHop();
  motoron.setSpeedNow(LEFT_MOTOR,  scaleSpeed(BASE_SPEED));
  motoron.setSpeedNow(RIGHT_MOTOR, scaleSpeed(BASE_SPEED));
  while (hopDistanceCm() < POST_TAG_FORWARD_CM) {
    wifiLoop();
    if (!isEnabled) {
      motoron.setSpeedNow(LEFT_MOTOR,  0);
      motoron.setSpeedNow(RIGHT_MOTOR, 0);
      handleNavDisable();
      return false;
    }
    delay(5);
  }
  motoron.setSpeedNow(LEFT_MOTOR,  0);
  motoron.setSpeedNow(RIGHT_MOTOR, 0);

  // Did the nudge alone land us back on the line?
  {
    uint16_t cv[IR_SENSOR_COUNT];
    long avg, sum;
    readSensors(cv, avg, sum);
    if (sum >= IR_MIN_LINE_SUM) {
      Serial.println("[RECOVER] line re-acquired after forward nudge");
      return true;
    }
  }

  if (sweepForLine(-1, 90.0f)) {
    Serial.println("[RECOVER] line acquired on left sweep");
    return true;
  }
  if (!isEnabled) return false;
  if (sweepForLine(+1, 180.0f)) {
    Serial.println("[RECOVER] line acquired on right sweep");
    return true;
  }
  if (!isEnabled) return false;
  // Re-centre to roughly the original heading so the bot's mental model and
  // the visible orientation match. Best-effort — return value is ignored.
  sweepForLine(-1, 90.0f);

  Serial.println("[RECOVER] line not found in ±90° sweep");
  return false;
}

// Common LINE_LOST handler for unexpected-loss states in the base: run the
// recovery routine; on failure, send status + park so the operator can
// intervene rather than thrashing forever.
static void handleBaseUnexpectedLineLoss() {
  if (!baseLineLostRecovery()) {
    sendStatus("base_recovery_failed");
    navState = NAV_PARKED;
  }
}

static void navBaseToFirstJunctionTick() {
  LineState ls = followLineBase();
  if (isJunctionState(ls)) {
    motoron.setSpeedNow(LEFT_MOTOR,  0);
    motoron.setSpeedNow(RIGHT_MOTOR, 0);
    Serial.println("[BASE] first junction reached");
    navState = NAV_BASE_FIRST_TURN;
  } else if (ls == LINE_LOST) {
    handleBaseUnexpectedLineLoss();
  }
}

static void navBaseFirstTurnTick() {
  Serial.print("[BASE] first turn ");
  Serial.print(BASE_FIRST_TURN_DEG);
  Serial.println(" deg");
  if (!baseTurnBlocking(BASE_FIRST_TURN_DEG)) return;
  navState = NAV_BASE_TO_TAG;
}

static void navBaseToTagTick() {
  // Keep the line PID running (line may have 90° bends here).
  LineState ls = followLineBase();

  char uid[32];
  if (readRfidNonBlocking(uid, sizeof(uid))) {
    motoron.setSpeedNow(LEFT_MOTOR,  0);
    motoron.setSpeedNow(RIGHT_MOTOR, 0);
    Serial.print("[BASE] tag detected: ");
    Serial.println(uid);
    Serial.println("[BASE] sending openAirlockA — waiting for exitClearance before continuing");
    sendOpenAirlockA();
    clearanceRetryDeadlineMs = millis() + DOOR_RETRY_INTERVAL_MS;
    navState = NAV_WAIT_EXIT_CLEARANCE;
    return;
  }

  if (ls == LINE_LOST) {
    handleBaseUnexpectedLineLoss();
  }
}

static void navBaseToSecondJunctionTick() {
  LineState ls = followLineBase();
  if (isJunctionState(ls)) {
    motoron.setSpeedNow(LEFT_MOTOR,  0);
    motoron.setSpeedNow(RIGHT_MOTOR, 0);
    Serial.println("[BASE] second (T) junction reached");
    navState = NAV_BASE_SECOND_TURN;
  } else if (ls == LINE_LOST) {
    handleBaseUnexpectedLineLoss();
  }
}

static void navBaseSecondTurnTick() {
  Serial.print("[BASE] second turn ");
  Serial.print(BASE_SECOND_TURN_DEG);
  Serial.println(" deg");
  if (!baseTurnBlocking(BASE_SECOND_TURN_DEG)) return;
  navState = NAV_BASE_TO_LINE_LOST;
}

static void navBaseToLineLostTick() {
  LineState ls = followLineBase();
  if (ls == LINE_LOST) {
    motoron.setSpeedNow(LEFT_MOTOR,  0);
    motoron.setSpeedNow(RIGHT_MOTOR, 0);
    Serial.println("[BASE] line lost — pausing momentarily");
    navState = NAV_BASE_LINE_LOST_PAUSE;
  }
}

// Stop in place for BASE_LINE_LOST_PAUSE_MS, then hand off to the forward
// nudge + wall-follow that probes for the next line / tunnel mouth.
static void navBaseLineLostPauseTick() {
  static unsigned long pauseStartMs = 0;
  if (pauseStartMs == 0) {
    motoron.setSpeedNow(LEFT_MOTOR,  0);
    motoron.setSpeedNow(RIGHT_MOTOR, 0);
    pauseStartMs = millis();
    return;
  }
  if (millis() - pauseStartMs >= BASE_LINE_LOST_PAUSE_MS) {
    pauseStartMs = 0;
    navState = NAV_BASE_FORWARD_NUDGE;
  }
}

static void navBaseForwardNudgeTick() {
  Serial.print("[BASE] forward nudge ");
  Serial.print(BASE_FORWARD_NUDGE_MS);
  Serial.println(" ms then wall-follow");
  motoron.setSpeedNow(LEFT_MOTOR,  scaleSpeed(BASE_SPEED));
  motoron.setSpeedNow(RIGHT_MOTOR, scaleSpeed(BASE_SPEED));
  delay(BASE_FORWARD_NUDGE_MS);
  motoron.setSpeedNow(LEFT_MOTOR,  0);
  motoron.setSpeedNow(RIGHT_MOTOR, 0);
  wifiLoop();
  if (!isEnabled) { handleNavDisable(); return; }
  navState = NAV_WALL_FOLLOW;
}

// ─────────────────────────────────────────
// NAV_AT_AIRLOCK_B: arrived at the return airlock. Rotate to face the base
// (180° from arena-entry direction), send openAirlockB, hand off to
// NAV_TUNNEL_B_WALL_FOLLOW. The door is initially closed: forward ultrasonic
// will read < OBSTACLE_STOP_CM and main.ino's top-of-loop will pause motors
// + retry the open request until the door opens.
// ─────────────────────────────────────────
static void navAtAirlockBTick() {
  const Facing baseDir = (Facing)((ARENA_ENTRY_FACING_INT + 2) % 4);
  const int turnDir = getTurnDir(robotFacing, baseDir);
  if (turnDir != 0) {
    Serial.print("[AIRLOCK_B] rotating ");
    Serial.print(turnDir * 90);
    Serial.print(" deg to face base (");
    Serial.print(facingStr(baseDir));
    Serial.println(")");
    turnDegrees((float)turnDir * 90.0f);
    wifiLoop();
    if (!isEnabled) { handleNavDisable(); return; }
    robotFacing = baseDir;
  }
  Serial.println("[AIRLOCK_B] sending openAirlockB — waiting for enterClearance before wall-follow");
  sendOpenAirlockB();
  clearanceRetryDeadlineMs = millis() + DOOR_RETRY_INTERVAL_MS;
  navState = NAV_WAIT_ENTER_CLEARANCE;
}

// ─────────────────────────────────────────
// Wait-for-clearance ticks. After sending an open-airlock-X request the robot
// holds still until the server replies with the matching clearance message
// (set by onMessage in wifi.ino). If no clearance arrives within
// DOOR_RETRY_INTERVAL_MS the request is resent — covers a dropped first
// request. Motors are held at zero throughout.
// ─────────────────────────────────────────
static void navWaitExitClearanceTick() {
  motoron.setSpeedNow(LEFT_MOTOR,  0);
  motoron.setSpeedNow(RIGHT_MOTOR, 0);
  wifiPoll();
  if (exitClearanceReceived) {
    Serial.println("[BASE] exitClearance received — continuing to second junction");
    navState = NAV_BASE_TO_SECOND_JUNCTION;
    return;
  }
  if (millis() >= clearanceRetryDeadlineMs) {
    Serial.println("[BASE] no exitClearance yet — resending openAirlockA");
    sendOpenAirlockA();
    clearanceRetryDeadlineMs = millis() + DOOR_RETRY_INTERVAL_MS;
  }
}

static void navWaitEnterClearanceTick() {
  motoron.setSpeedNow(LEFT_MOTOR,  0);
  motoron.setSpeedNow(RIGHT_MOTOR, 0);
  wifiPoll();
  if (enterClearanceReceived) {
    Serial.println("[AIRLOCK_B] enterClearance received — entering tunnel wall-follow");
    navState = NAV_TUNNEL_B_WALL_FOLLOW;
    return;
  }
  if (millis() >= clearanceRetryDeadlineMs) {
    Serial.println("[AIRLOCK_B] no enterClearance yet — resending openAirlockB");
    sendOpenAirlockB();
    clearanceRetryDeadlineMs = millis() + DOOR_RETRY_INTERVAL_MS;
  }
}

// ─────────────────────────────────────────
// NAV_BASE_RETURN: line-follow back into the base from the tunnel mouth.
// Run ends on LINE_LOST (line ran out) or a forward obstacle (handled by
// main.ino's obstacle gate which flips us to NAV_PARKED).
// ─────────────────────────────────────────
static void navBaseReturnTick() {
  LineState ls = followLineBase();
  if (ls == LINE_LOST) {
    motoron.setSpeedNow(LEFT_MOTOR,  0);
    motoron.setSpeedNow(RIGHT_MOTOR, 0);
    Serial.println("[BASE_RETURN] line ended — parking");
    sendStatus("parked_line_end");
    navState = NAV_PARKED;
  }
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

  // Re-enable: navState is whatever it was before the pause — the dispatcher
  // below will run that state's tick directly. To bootstrap from scratch use
  // `nav` (sets NAV_BASE_TO_FIRST_JUNCTION) or `arena` (sets NAV_ARENA_NAV)
  // over serial.

  // Stub-state prints fire once per entry, not every tick.
  static NavState lastTickState = (NavState)255;
  const bool justEntered = (lastTickState != navState);
  if (justEntered) {
    Serial.print("[NAV] -> ");
    Serial.print(navStateStr(navState));
    Serial.print(" :: ");
    Serial.println(navActivityStr(navState));
  }
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

    case NAV_BASE_TO_FIRST_JUNCTION:
      navBaseToFirstJunctionTick();
      return;

    case NAV_BASE_FIRST_TURN:
      navBaseFirstTurnTick();
      return;

    case NAV_BASE_TO_TAG:
      navBaseToTagTick();
      return;

    case NAV_WAIT_EXIT_CLEARANCE:
      navWaitExitClearanceTick();
      return;

    case NAV_BASE_TO_SECOND_JUNCTION:
      navBaseToSecondJunctionTick();
      return;

    case NAV_BASE_SECOND_TURN:
      navBaseSecondTurnTick();
      return;

    case NAV_BASE_TO_LINE_LOST:
      navBaseToLineLostTick();
      return;

    case NAV_BASE_LINE_LOST_PAUSE:
      navBaseLineLostPauseTick();
      return;

    case NAV_BASE_FORWARD_NUDGE:
      navBaseForwardNudgeTick();
      return;

    case NAV_ARENA_NAV:
      navArenaTick();
      return;

    case NAV_AT_TAG:
      navAtTagTick();
      return;

    case NAV_POST_TAG_NUDGE:
      navPostTagNudgeTick();
      return;

    case NAV_PLANTING:
      navPlantingTick();
      return;

    case NAV_AVOID_OBSTACLE:
      navAvoidObstacleTick();
      return;

    case NAV_WALL_FOLLOW:
      wallFollow();
      return;

    case NAV_AT_AIRLOCK_B:
      navAtAirlockBTick();
      return;

    case NAV_WAIT_ENTER_CLEARANCE:
      navWaitEnterClearanceTick();
      return;

    case NAV_TUNNEL_B_WALL_FOLLOW:
      wallFollow();
      return;

    case NAV_BASE_RETURN:
      navBaseReturnTick();
      return;

    case NAV_PARKED:
      motoron.setSpeedNow(LEFT_MOTOR,  0);
      motoron.setSpeedNow(RIGHT_MOTOR, 0);
      return;
  }
}
