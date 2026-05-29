# Software overview

Text-based block diagram of the firmware. Arrows mean "calls" or "delegates to". Italics in the boxes are the file each function lives in.

## Top-level loop

`main.ino`'s `loop()` always runs the housekeeping pass, then runs the safety gate, then dispatches to the state machine (or legacy path when `useStateMachine` is false). `useStateMachine` defaults to **true** on boot.

```
                                main.ino — loop()
                                       │
                                       ▼
                  ── top-of-loop obstacle gate ──
                  getDistanceCM(SENSOR_FORWARD)
                  2-consecutive-readings debounce
                  rising edge → sendStatus("obstacle_stop")
                  falling edge → sendStatus("obstacle_cleared")
                  if NAV_WALL_FOLLOW / TUNNEL_B_WALL_FOLLOW
                    and door still closed:
                    every DOOR_RETRY_INTERVAL_MS resend
                    openAirlockA/B
                                       │
                                       ▼
        ┌──────────────────────────────┼──────────────────────────────┐
        │                              │                              │
   wifiLoop()                  checkPowerButton()             handleSerialCommands()
   (wifi.ino)                  (main.ino)                     (commands.ino)
        │                              │                              │
        │                  HIGH→LOW edge on pin 17                    │
        │                  toggles isEnabled                          │
        │                  (200 ms debounce)                          │
        │                                                             │
   ┌────┴───────┐                                                     │
   │            │                                                     │
onMessage   updateLED                                                 │
parser     priority stack:                                            │
   │       1. revive btn → green                                      │
   │       2. isEnabled  → solid red                                  │
isEnabled  3. else       → blink red                                  │
heartbeat                                                             │
fertileResult                                                         │
exit/enterClearance                                                   │
   │                                                                  │
   ▼                                                                  ▼
heartbeat watchdog:                                              test commands
if lastHeartbeatMs != 0                                          (nav, arena, stop,
   and millis - last > 1 s:                                       enc, hop, selftest,
   isEnabled = false                                              state, ldr, pitch,
                                                                  recalib, etc.)
                                       │
                          updateEncoders + updateHopHeading
                          readAndPrintIR / Distance / Encoders /
                                       Pitch / LDR
                                       │
                          ┌────────────┴────────────────┐
                          ▼                             ▼
              ── safety gate ──             ── enabled body ──
              if !isEnabled:                if useStateMachine:
                motors = 0                    navigationUpdate()
                handleNavDisable()          else:
                  (preserves navState!)       rfidLoop() +
                return                        applyMotorEnabled()
              if obstacleNow:                  [legacy path]
                motors = 0
                if NAV_BASE_RETURN:
                  → NAV_PARKED
                  sendStatus("parked_obstacle")
                return
```

Key invariants:
- `handleNavDisable` does **not** mutate `navState`. Re-enable resumes whatever state was active when the bot was disabled — the operator can disable mid-run with the button and resume from the same spot.
- The forward-obstacle gate only pauses motion for the tick; it doesn't change state. Auto-resume on the same tick that `stableObstacle` flips back to false (door opens, robot moves, object cleared).
- Heartbeat timeout only fires once at least one heartbeat has been received — bench-testing without a server doesn't trip it.

## State-machine dispatcher

`navigationUpdate()` switches on `navState` and dispatches to a per-state tick. Transitions happen inside each tick. The dispatcher prints `[NAV] -> NAME :: activity` once per state entry so the serial log reads as a play-by-play.

```
                navigationUpdate()  (navigation.ino)
                        │
                        ▼
              defense-in-depth: if !isEnabled, hold + return
                        │
                        ▼
              if state changed since last tick:
                Serial.print "[NAV] -> NAME :: activity"
                        │
                        ▼
                    switch(navState)
                        │
   ┌─── BASE-EXIT SEQUENCE ───────────────────────────────────────────────────┐
   │                                                                         │
   │  NAV_BASE_TO_FIRST_JUNCTION  →  followLineBase()                        │
   │     LINE_JUNCTION_*  → NAV_BASE_FIRST_TURN                              │
   │     LINE_LOST        → baseLineLostRecovery (nudge + ±90° sweep)        │
   │                                                                         │
   │  NAV_BASE_FIRST_TURN         →  baseTurnBlocking(+90°)                  │
   │     pre-turn nudge POST_TAG_FORWARD_CM (encoder)                        │
   │     turnDegrees → post-turn JUNCTION_NUDGE_MS forward kick              │
   │     → NAV_BASE_TO_TAG                                                   │
   │                                                                         │
   │  NAV_BASE_TO_TAG             →  followLineBase + readRfidNonBlocking    │
   │     RFID hit → sendOpenAirlockA → NAV_WAIT_EXIT_CLEARANCE               │
   │     LINE_LOST → recovery                                                │
   │                                                                         │
   │  NAV_WAIT_EXIT_CLEARANCE     →  hold + wifiPoll                         │
   │     exitClearance recvd → NAV_BASE_TO_SECOND_JUNCTION                   │
   │     timeout → resend openAirlockA every DOOR_RETRY_INTERVAL_MS          │
   │                                                                         │
   │  NAV_BASE_TO_SECOND_JUNCTION →  followLineBase()                        │
   │     LINE_JUNCTION_* → NAV_BASE_SECOND_TURN                              │
   │     LINE_LOST       → recovery                                          │
   │                                                                         │
   │  NAV_BASE_SECOND_TURN        →  baseTurnBlocking(-90°)                  │
   │     → NAV_BASE_TO_LINE_LOST                                             │
   │                                                                         │
   │  NAV_BASE_TO_LINE_LOST       →  followLineBase()                        │
   │     LINE_LOST       → NAV_BASE_LINE_LOST_PAUSE                          │
   │                                                                         │
   │  NAV_BASE_LINE_LOST_PAUSE    →  motors = 0, momentary hold              │
   │     hold BASE_LINE_LOST_PAUSE_MS → NAV_BASE_FORWARD_NUDGE               │
   │                                                                         │
   │  NAV_BASE_FORWARD_NUDGE      →  delay BASE_FORWARD_NUDGE_MS forward     │
   │     → NAV_WALL_FOLLOW                                                   │
   │                                                                         │
   │  NAV_WALL_FOLLOW             →  wallFollow() through Tunnel A           │
   │     IR sees line → seed robotPos = Airlock A → NAV_ARENA_NAV            │
   │                                                                         │
   └─────────────────────────────────────────────────────────────────────────┘

   ┌─── ARENA NAVIGATION ────────────────────────────────────────────────────┐
   │                                                                         │
   │  NAV_ARENA_NAV               →  navArenaTick()                          │
   │     RFID hit         → NAV_AT_TAG                                       │
   │     obstacle < 20cm  → NAV_AVOID_OBSTACLE                               │
   │     in line zone (row ≥ LINE_ZONE_MIN_ROW=4) → followLineBase           │
   │     in no-line zone → encoder hop + heading lock; nearNextNode → step   │
   │                                                                         │
   │  NAV_AT_TAG                  →  wait for isFertileReply                 │
   │     reply → update tagMap, robotPos                                     │
   │           if fertile + unplanted + seeds>0 → NAV_POST_TAG_NUDGE (plant) │
   │           else → replan; turn-needed → NAV_POST_TAG_NUDGE; else ARENA   │
   │                                                                         │
   │  NAV_POST_TAG_NUDGE          →  encoder-driven 5cm nudge                │
   │     intent=plant → NAV_PLANTING                                         │
   │     intent=turn  → turnDegrees(pendingJunctionDir × 90°), → ARENA_NAV   │
   │                                                                         │
   │  NAV_PLANTING                →  sweepTo MAX→MIN + MIN→MAX               │
   │     sendPlanted, seedsRemaining--                                       │
   │     seedsRemaining==0 → returning=true (target=Airlock B)               │
   │     replan + turn → NAV_ARENA_NAV                                       │
   │                                                                         │
   │  NAV_AVOID_OBSTACLE          →  mark cell TAG_BLOCKED, replan, turn     │
   │     → NAV_ARENA_NAV (A* now routes around the marked cell)              │
   │                                                                         │
   └─────────────────────────────────────────────────────────────────────────┘

   ┌─── RETURN SEQUENCE ─────────────────────────────────────────────────────┐
   │                                                                         │
   │  NAV_AT_AIRLOCK_B            →  rotate to face base, sendOpenAirlockB   │
   │     → NAV_WAIT_ENTER_CLEARANCE                                          │
   │                                                                         │
   │  NAV_WAIT_ENTER_CLEARANCE    →  hold + wifiPoll                         │
   │     enterClearance → NAV_TUNNEL_B_WALL_FOLLOW                           │
   │     timeout → resend openAirlockB                                       │
   │                                                                         │
   │  NAV_TUNNEL_B_WALL_FOLLOW    →  wallFollow() through Tunnel B           │
   │     IR sees line → NAV_BASE_RETURN                                      │
   │                                                                         │
   │  NAV_BASE_RETURN             →  followLineBase()                        │
   │     LINE_LOST → sendStatus("parked_line_end") → NAV_PARKED              │
   │     obstacle (top-of-loop) → sendStatus("parked_obstacle") → NAV_PARKED │
   │                                                                         │
   │  NAV_PARKED                  →  motors = 0, hold                        │
   │                                                                         │
   └─────────────────────────────────────────────────────────────────────────┘
```

## Arena navigation tick

`navArenaTick` runs different sub-logic for the line-zone (bottom rows 4–8, lines connect holes) vs. the no-line zone (top rows 0–3, holes only). **Note this is a ROW split**, not a column split (an earlier draft of the docs had it wrong).

```
                          navArenaTick()
                                │
                  ┌─────────────┴───────────────────────────────────────┐
                  │                                                     │
        return-mode short-circuit:                          obstacle check:
        if returning && robotPos == AIRLOCK_B               lastForwardDistanceCm
        → stop, → NAV_AT_AIRLOCK_B                          < OBSTACLE_AVOID_CM (20 cm)?
                                                            yes → stop, → NAV_AVOID_OBSTACLE
                                │
                                ▼
                  pollRfidAndQueue()
                  PICC_IsNewCardPresent + ReadCardSerial
                                │
                  ┌─────────────┴─────────────┐
                  │                           │
              tag detected               no tag
                  │                           │
                  ▼                           ▼
        build UID + halt           if (robotPos.row >= LINE_ZONE_MIN_ROW):
        clearFertileResult           // line zone
        sendIsFertile                followLineBase()
        stop motors                  (PID lane-keeping; junction
        navState = NAV_AT_TAG         classifications are ignored —
                                      sensor averages back to centre)
                                   else:
                                     // no-line zone
                                     if (!deadReckonDriving):
                                       encoderResetHop, resetHopHeading
                                       motors = BASE_SPEED
                                       deadReckonDriving = true
                                     else:
                                       applyHeadingCorrection()
                                       if nearNextNode():
                                         stop, endHopHeading
                                         advancePosOneCell
                                         sendStatus("rfid_miss_dead_reckon_advance")
                                         if returning && AIRLOCK_B → NAV_AT_AIRLOCK_B
```

## Line-lost recovery (base only)

When `followLineBase()` returns `LINE_LOST` in `NAV_BASE_TO_FIRST_JUNCTION`, `NAV_BASE_TO_TAG`, or `NAV_BASE_TO_SECOND_JUNCTION`, the state's tick calls `baseLineLostRecovery()`:

```
        baseLineLostRecovery()  (blocking)
              │
              ▼
        1. Forward POST_TAG_FORWARD_CM via encoder hop
              │
              ▼
        2. readSensors → line under array? → yes, return true
              │
              ▼ no
        3. sweepForLine(-1, 90°)   ← sweep left up to 90°
              │
              ▼ no line
        4. sweepForLine(+1, 180°)  ← sweep right through centre to +90°
              │
              ▼ no line
        5. sweepForLine(-1, 90°)   ← re-centre, best-effort
              │
              ▼
        return false  → caller: sendStatus("base_recovery_failed"), → NAV_PARKED
```

Each sweep stops the instant `readSensors` shows `sum >= IR_MIN_LINE_SUM`. `wifiLoop()` runs every ~5 ms inside the blocking loops so the heartbeat survives.

`NAV_BASE_TO_LINE_LOST` (line ending at tunnel mouth) and `NAV_BASE_RETURN` (line ending at parking) do **not** use this recovery — losing the line in those states is the expected exit signal.

## Planner

```
        navAtTagTick fertile reply / navPlantingTick done / navAvoidObstacleTick done
              │
              ▼
        replanNextDir()
              │
       ┌──────┴──────┐
       │             │
   returning?     normal
       │             │
       ▼             ▼
   targetPos =    selectNextTarget(robotPos, targetPos)
   AIRLOCK_B      prioritise TAG_FERTILE > TAG_UNKNOWN
                  skip TAG_INFERTILE / TAG_PLANTED / TAG_BLOCKED
                  within tier: lowest Manhattan distance
                  │
                  ▼
              aStarNextStep(robotPos, targetPos, next)
              A* on 9×9 grid, Manhattan heuristic, unit edge costs
              skips TAG_BLOCKED cells, static scratch arrays
                  │
                  ▼
              facingToward(robotPos, next)
              getTurnDir(robotFacing, want)
                  │
                  ▼
              pendingJunctionDir ∈ {-1, 0, +1, 2}
              (consumed by NAV_POST_TAG_NUDGE / NAV_AVOID_OBSTACLE)
```

`returning` flips true once `seedsRemaining` hits 0 inside `navPlantingTick`. From then on, every replan targets Airlock B. `TAG_BLOCKED` cells (set by `NAV_AVOID_OBSTACLE`) are skipped by both `selectNextTarget` and the A* neighbour iteration, so a marked obstacle reroutes automatically.

## WiFi / messaging

```
        MiniMessenger (wifi.ino)
                │
   ┌────────────┴────────────────┐
   │                             │
incoming                     outgoing
   │                             │
   ▼                             ▼
onMessage(payload):       wifiSend(msg)
parses key=value           ↑
strings                    │
   │              ┌────────┴────────────────────────────────────┐
   │              │       │      │        │        │        │
   ▼          sendRegister sendIsFertile sendPlanted          │
matches:        sendPosition sendStatus sendOpenAirlockA      │
 type=heartbeat                          sendOpenAirlockB     │
   enable=1|0 → isEnabled
   refreshes lastHeartbeatMs
 type=emergency / disable → isEnabled = false
 type=exitClearance       → exitClearanceReceived = true
 type=enterClearance      → enterClearanceReceived = true
 type=isFertileReply      → fertileResult populated (received, fertile,
                            planted, tagId, x, y)
```

`wifiLoop()` runs:
1. `messenger.loop()`
2. `updateLED()` — applies the priority stack (revive → red enabled → blink red disabled)
3. Heartbeat watchdog: `lastHeartbeatMs != 0 && now - last > HEARTBEAT_TIMEOUT_MS` → `isEnabled = false`
4. Re-registration every `REGISTER_INTERVAL_MS` (10 s)

Door-open signal: the state machine watches the forward ultrasonic going back above `OBSTACLE_STOP_CM`, **not** the clearance messages. `exit/enterClearanceReceived` flags exist for verification/debug only — the gate retry loop in `main.ino` resends `openAirlockA/B` while the obstacle is still latched at a closed door.

## Encoder + hop-heading subsystem

Two quadrature encoders only — back left (BL) and back right (BR). Front encoders aren't wired. Each rear encoder uses two ISRs (one per channel) with full 4× quadrature decoding via `CHANGE` on both A and B.

```
   Hardware ISRs              Always-run in loop()              Consumers
   (motors.ino)               (main.ino)                        (navigation.ino,
                                                                 commands.ino)
   isr_BL_A  ─┐
   isr_BL_B   │  → encBL  → leftTicks()  ──┐                    │
   isr_BR_A   │                            │── straightTicks() ─→ hopDistanceCm()
   isr_BR_B  ─┘  → encBR  → rightTicks() ──┘                    │   nearNextNode()
                                                                 ─→ calibRecordHop()
                                                                    ticksPerCm lock
                                                                 │  (KV-persisted)
                          updateHopHeading()                     │
                          (no-op unless                          │
                           hopHeadingActive)                     │
                                  │                              │
                                  ▼                              │
                              hopHeadingDeg ─→ applyHeadingCorrection()
                                              motor differential
                                              while deadReckonDriving
```

`leftTicks() = encBL`, `rightTicks() = encBR` directly — no averaging needed with one encoder per side. `straightTicks() = (leftTicks() + rightTicks()) / 2`. The BL ISRs are sign-flipped so both sides report positive ticks for forward motion.

Self-calibrating `ticksPerCm`: every confirmed straight tag-to-tag hop in the arena feeds `calibRecordHop(prevPos, newPos)`. Manhattan-distance-1 samples are accepted; outliers (>20 % from running mean) are rejected. After `CALIB_MIN_SAMPLES` (4) accepted samples, the value locks and is persisted to KV. `recalib` clears it.

## Self-test (`selftest` command)

`runSelftest()` asserts on the pure-logic code (no hardware needed):
- `getTurnDir` / `facingAfterTurn` round-trips
- `facingToward` for cardinal neighbours
- `aStarNextStep` straight-line, around obstacles, no-path
- `selectNextTarget` priority order with mixed tag states
- `calibRecordHop` accept/reject paths
- LineState classification on synthetic IR buffers

Run after any change to the planner, calibration, or facing-math code.
