// ─────────────────────────────────────────
// Self-test: hardcoded assertions on pure-logic subsystems.
// Saves and restores tagMap / robotPos / robotFacing so it's safe to invoke
// any time. Prints pass/fail counts; intentionally no hardware involvement.
// ─────────────────────────────────────────
static int testPass = 0;
static int testFail = 0;

static void assertEqInt(long actual, long expected, const char* name) {
  if (actual == expected) {
    testPass++;
  } else {
    testFail++;
    Serial.print("FAIL: "); Serial.print(name);
    Serial.print(" — got ");    Serial.print(actual);
    Serial.print(", expected "); Serial.println(expected);
  }
}

static void runSelfTest() {
  // Save state we'll clobber.
  const GridPos savedPos     = robotPos;
  const Facing  savedFacing  = robotFacing;
  static TagState savedTagMap[9][9];
  memcpy(savedTagMap, tagMap, sizeof(tagMap));

  testPass = 0;
  testFail = 0;

  // getTurnDir
  assertEqInt(getTurnDir(NORTH, NORTH),  0, "getTurnDir N->N");
  assertEqInt(getTurnDir(NORTH, EAST),   1, "getTurnDir N->E");
  assertEqInt(getTurnDir(NORTH, SOUTH),  2, "getTurnDir N->S");
  assertEqInt(getTurnDir(NORTH, WEST),  -1, "getTurnDir N->W");
  assertEqInt(getTurnDir(WEST,  NORTH),  1, "getTurnDir W->N");
  assertEqInt(getTurnDir(SOUTH, NORTH),  2, "getTurnDir S->N");

  // facingAfterTurn
  assertEqInt(facingAfterTurn(NORTH,  1), EAST,  "facingAfterTurn N+1");
  assertEqInt(facingAfterTurn(NORTH, -1), WEST,  "facingAfterTurn N-1");
  assertEqInt(facingAfterTurn(NORTH,  2), SOUTH, "facingAfterTurn N+2");
  assertEqInt(facingAfterTurn(WEST,   1), NORTH, "facingAfterTurn W+1");

  // facingToward
  assertEqInt(facingToward({0,0}, {0,1}), EAST,  "facingToward (0,0)->(0,1)");
  assertEqInt(facingToward({0,0}, {1,0}), SOUTH, "facingToward (0,0)->(1,0)");
  assertEqInt(facingToward({1,1}, {0,1}), NORTH, "facingToward (1,1)->(0,1)");
  assertEqInt(facingToward({1,1}, {1,0}), WEST,  "facingToward (1,1)->(1,0)");

  // A*
  GridPos next;
  assertEqInt(aStarNextStep({0,0}, {0,1}, next), 1, "A* adjacent found");
  assertEqInt(next.row, 0, "A* adjacent next.row");
  assertEqInt(next.col, 1, "A* adjacent next.col");

  assertEqInt(aStarNextStep({0,0}, {0,0}, next), 0, "A* same cell -> false");
  assertEqInt(aStarNextStep({-1,0}, {3,3}, next), 0, "A* invalid from -> false");

  assertEqInt(aStarNextStep({0,0}, {3,3}, next), 1, "A* (0,0)->(3,3) found");
  bool firstStepOk = (next.row == 0 && next.col == 1) || (next.row == 1 && next.col == 0);
  assertEqInt(firstStepOk, 1, "A* (0,0)->(3,3) first step is N-neighbour");

  // selectNextTarget — mark everything PLANTED then introduce candidates one at a time.
  memset(tagMap, TAG_PLANTED, sizeof(tagMap));
  GridPos target;
  assertEqInt(selectNextTarget({0,0}, target), 0, "selectNext nothing left");

  tagMap[5][5] = TAG_FERTILE;
  assertEqInt(selectNextTarget({0,0}, target), 1, "selectNext one FERTILE");
  assertEqInt(target.row, 5, "selectNext FERTILE row");
  assertEqInt(target.col, 5, "selectNext FERTILE col");

  tagMap[1][1] = TAG_UNKNOWN;
  // FERTILE should win regardless of distance (tier 0 > tier 1).
  assertEqInt(selectNextTarget({0,0}, target), 1, "selectNext F+U");
  assertEqInt(target.row, 5, "selectNext F-over-U row");
  assertEqInt(target.col, 5, "selectNext F-over-U col");

  tagMap[2][2] = TAG_FERTILE;
  // Two FERTILE: closer one (2,2) wins on distance.
  assertEqInt(selectNextTarget({0,0}, target), 1, "selectNext two F");
  assertEqInt(target.row, 2, "selectNext closer-F row");
  assertEqInt(target.col, 2, "selectNext closer-F col");

  // Restore
  robotPos    = savedPos;
  robotFacing = savedFacing;
  memcpy(tagMap, savedTagMap, sizeof(tagMap));

  Serial.print("Self-test: "); Serial.print(testPass);
  Serial.print(" passed, ");    Serial.print(testFail);
  Serial.println(" failed.");
}

// ─────────────────────────────────────────
// Run one heading-locked forward hop, blocking until nearNextNode()
// or 10s timeout. Used to verify encoders + heading correction.
// ─────────────────────────────────────────
static void runHopTest() {
  if (!isEnabled) {
    Serial.println("hop: refusing — robot is disabled.");
    return;
  }
  Serial.println("hop: starting...");
  encoderResetHop();
  resetHopHeading();
  motoron.setSpeedNow(LEFT_MOTOR,  scaleSpeed(BASE_SPEED));
  motoron.setSpeedNow(RIGHT_MOTOR, scaleSpeed(BASE_SPEED));

  const unsigned long start = millis();
  while (!nearNextNode() && (millis() - start) < 10000) {
    wifiLoop();              // keep heartbeat alive; will set isEnabled=false on timeout
    if (!isEnabled) {
      Serial.println("hop: aborted — disabled mid-run.");
      break;
    }
    updateHopHeading();
    applyHeadingCorrection();
    delay(10);
  }
  motoron.setSpeedNow(LEFT_MOTOR,  0);
  motoron.setSpeedNow(RIGHT_MOTOR, 0);
  endHopHeading();

  Serial.print("hop: done. straightTicks="); Serial.print(straightTicks());
  Serial.print("  hopDistanceCm=");           Serial.print(hopDistanceCm());
  Serial.print("  finalHeadingDeg=");         Serial.println(hopHeadingDeg);
}

// ─────────────────────────────────────────
// Serial Command Handler
// ─────────────────────────────────────────
void handleSerialCommands() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() == 0) return;

  if (input.equalsIgnoreCase("ir")) {
    showIR = !showIR;
    Serial.print("IR readings ");
    Serial.println(showIR ? "ON" : "OFF");

  } else if (input.equalsIgnoreCase("distance")) {
    showDistance = !showDistance;
    Serial.print("Distance readings ");
    Serial.println(showDistance ? "ON" : "OFF");

  } else if (input.equalsIgnoreCase("sensors")) {
    bool newState = !(showIR && showDistance);
    showIR = newState;
    showDistance = newState;
    Serial.print("All sensor readings ");
    Serial.println(newState ? "ON" : "OFF");

  } else if (input.equalsIgnoreCase("enc")) {
    showEncoders = !showEncoders;
    Serial.print("Encoder readings ");
    Serial.println(showEncoders ? "ON" : "OFF");

  } else if (input.equalsIgnoreCase("encreset")) {
    encoderResetHop();
    Serial.println("Encoders zeroed.");

  } else if (input.equalsIgnoreCase("c")) {
    runCalibration();

  } else if (input.equalsIgnoreCase("calib")) {
    Serial.println("── Calibration ──");
    Serial.print("  calibTicksPerCm = "); Serial.println(calibTicksPerCm);
    Serial.print("  calibLocked     = "); Serial.println(calibLocked ? "true" : "false");
    Serial.print("  calibSamples    = "); Serial.println(calibSamples);
    Serial.print("  calibSum        = "); Serial.println(calibSum);

  } else if (input.equalsIgnoreCase("nav")) {
    useStateMachine = !useStateMachine;
    if (useStateMachine) {
      navState = NAV_ARENA_NAV;
      Serial.println("State machine ENABLED -> navState=NAV_ARENA_NAV");
    } else {
      navState = NAV_DISABLED;
      Serial.println("State machine DISABLED -> navState=NAV_DISABLED");
    }

  } else if (input.equalsIgnoreCase("state")) {
    Serial.println("── State dump ──");
    Serial.print("  isEnabled          = "); Serial.println(isEnabled ? "true" : "false");
    Serial.print("  useStateMachine    = "); Serial.println(useStateMachine ? "true" : "false");
    Serial.print("  navState           = "); Serial.println(navStateStr(navState));
    Serial.print("  robotPos           = ("); Serial.print(robotPos.row);
    Serial.print(", "); Serial.print(robotPos.col); Serial.println(")");
    Serial.print("  targetPos          = ("); Serial.print(targetPos.row);
    Serial.print(", "); Serial.print(targetPos.col); Serial.println(")");
    Serial.print("  robotFacing        = "); Serial.println(facingStr(robotFacing));
    Serial.print("  seedsRemaining     = "); Serial.println(seedsRemaining);
    Serial.print("  pendingJunctionDir = "); Serial.println(pendingJunctionDir);

  } else if (input.equalsIgnoreCase("target")) {
    GridPos t;
    if (selectNextTarget(robotPos, t)) {
      Serial.print("Next target: ("); Serial.print(t.row);
      Serial.print(", "); Serial.print(t.col); Serial.println(")");
    } else {
      Serial.println("No targets remaining.");
    }

  } else if (input.equalsIgnoreCase("astar") || input.startsWith("astar ")) {
    int r1, c1, r2, c2;
    if (sscanf(input.c_str(), "astar %d %d %d %d", &r1, &c1, &r2, &c2) != 4) {
      Serial.println("Usage: astar <fromRow> <fromCol> <toRow> <toCol>");
    } else {
      GridPos from = { (int8_t)r1, (int8_t)c1 };
      GridPos to   = { (int8_t)r2, (int8_t)c2 };
      GridPos next;
      if (aStarNextStep(from, to, next)) {
        Serial.print("Next step: ("); Serial.print(next.row);
        Serial.print(", "); Serial.print(next.col); Serial.println(")");
      } else {
        Serial.println("No path (or already at target / invalid input).");
      }
    }

  } else if (input.equalsIgnoreCase("tag") || input.startsWith("tag ")) {
    int r, c;
    char stateChar;
    if (sscanf(input.c_str(), "tag %d %d %c", &r, &c, &stateChar) != 3) {
      Serial.println("Usage: tag <row> <col> <u|f|i|p>");
    } else if (r < 0 || r >= 9 || c < 0 || c >= 9) {
      Serial.println("Row/col must be 0..8");
    } else {
      TagState s;
      bool ok = true;
      switch (stateChar) {
        case 'u': case 'U': s = TAG_UNKNOWN;   break;
        case 'f': case 'F': s = TAG_FERTILE;   break;
        case 'i': case 'I': s = TAG_INFERTILE; break;
        case 'p': case 'P': s = TAG_PLANTED;   break;
        default: Serial.println("State must be u/f/i/p"); ok = false; break;
      }
      if (ok) {
        tagMap[r][c] = s;
        Serial.print("tagMap["); Serial.print(r);
        Serial.print("]["); Serial.print(c); Serial.print("] = ");
        Serial.println(tagStateStr(s));
      }
    }

  } else if (input.equalsIgnoreCase("pos") || input.startsWith("pos ")) {
    int r, c;
    char facingChar;
    if (sscanf(input.c_str(), "pos %d %d %c", &r, &c, &facingChar) != 3) {
      Serial.println("Usage: pos <row> <col> <n|e|s|w>");
    } else if (r < 0 || r >= 9 || c < 0 || c >= 9) {
      Serial.println("Row/col must be 0..8");
    } else {
      Facing f;
      bool ok = true;
      switch (facingChar) {
        case 'n': case 'N': f = NORTH; break;
        case 'e': case 'E': f = EAST;  break;
        case 's': case 'S': f = SOUTH; break;
        case 'w': case 'W': f = WEST;  break;
        default: Serial.println("Facing must be n/e/s/w"); ok = false; break;
      }
      if (ok) {
        robotPos    = { (int8_t)r, (int8_t)c };
        robotFacing = f;
        Serial.print("robotPos=("); Serial.print(r);
        Serial.print(","); Serial.print(c);
        Serial.print(") robotFacing="); Serial.println(facingStr(f));
      }
    }

  } else if (input.equalsIgnoreCase("hop")) {
    runHopTest();

  } else if (input.equalsIgnoreCase("selftest")) {
    runSelfTest();

  } else if (input.startsWith("forward")) {
    int speed = FORWARD_SPEED;
    int spaceIdx = input.indexOf(' ');
    if (spaceIdx != -1) speed = input.substring(spaceIdx + 1).toInt();
    moveForward(3000, speed);

  } else {
    float degrees = input.toFloat();
    if (degrees != 0) {
      Serial.print("Turning ");
      Serial.print(degrees);
      Serial.println(" degrees...");
      turnDegrees(degrees);
    } else {
      // Forward unrecognised commands to the server
      wifiSend(input.c_str());
    }
  }
}
