# Software overview

Text-based block diagram of the firmware. Arrows mean "calls" or "delegates to". Italics in the boxes are the file each function lives in.

## Top-level loop

`main.ino`'s `loop()` always runs the housekeeping pass, then either the state machine or the legacy path based on `useStateMachine`:

```
                                main.ino — loop()
                                       │
        ┌──────────────────────────────┼──────────────────────────────┐
        │                              │                              │
   updateEncoders()                wifiLoop()                  handleSerialCommands()
   updateHopHeading()              (wifi.ino)                  (commands.ino)
   (motors.ino)                         │                              │
        │                  ┌────────────┴────────────┐                 │
        │                  │                         │                 │
        │           onMessage()                heartbeat               │
        │           parser                     watchdog                │
        │                  │                         │                 │
        │       isEnabled / fertileResult       isEnabled              │
        │       (globals)                       false on               │
        │                                       timeout                │
        ▼                                                              ▼
  obstacle check         ┌──────────── safety gate ──────────┐     test commands
  (top of loop):         │   if (!isEnabled) → stop motors,  │     (enc, hop,
  forward < 8 cm         │      handleNavDisable(), return   │      selftest,
  → motors off,          │   if (obstacleNow) → stop motors, │      state, etc.)
  sendStatus once        │      return (auto-resume on clear)│
                         └─────────────┬─────────────────────┘
                                       │
                          ┌────────────┴───────────┐
                          │                        │
                  useStateMachine = true    useStateMachine = false
                          │                        │
                  navigationUpdate()         rfidLoop() + applyMotorEnabled()
                  (navigation.ino)           (rfid.ino + motors.ino)
                                             [legacy path]
```

## State-machine dispatcher

`navigationUpdate()` switches on `navState` and dispatches to a per-state tick function. Transitions happen inside the tick functions.

```
                navigationUpdate()  (navigation.ino)
                        │
       ┌────────────────┼─────────────────────────────────────────────┐
       │ defense-in-depth: if !isEnabled, stop motors + return        │
       │ if navState == NAV_DISABLED, auto-resume to NAV_ARENA_NAV    │
       └────────────────┬─────────────────────────────────────────────┘
                        │
                        ▼
                    switch(navState)
                        │
   ┌─────────────┬──────┴──────┬──────────────┬──────────────┬──────────────┬──────────────┐
   │             │             │              │              │              │              │
NAV_LINE_     NAV_ARENA_   NAV_AT_TAG   NAV_PLANTING   NAV_AVOID_     NAV_WALL_     NAV_PARKED /
FOLLOW        NAV               │             │         OBSTACLE       FOLLOW       NAV_DISABLED
   │             │              │             │              │              │              │
   ▼             ▼              ▼             ▼              ▼              ▼              ▼
followLine()  navArenaTick  navAtTagTick   navPlantingTick  navAvoid    wallFollow()   motors off
              ()                                            ObstacleTick                (stub print
                                                            ()                          on entry)
```

## Arena navigation tick

`navArenaTick` is where the real planning happens. It runs different sub-logic for the line-followed left half vs. the dead-reckoned right half.

```
                          navArenaTick()
                                │
                  ┌─────────────┴───────────────┐
                  │                             │
        obstacle check:                    proceed
        lastForwardDistanceCm
        < OBSTACLE_AVOID_CM
        (20 cm)?
                  │
                yes
                  │
                  ▼
        stop, navState =
        NAV_AVOID_OBSTACLE
                                              │
                  ┌───────────────────────────┴────────────────┐
                  │                                            │
         robotPos.col <= LEFT_HALF_MAX_COL         robotPos.col > LEFT_HALF_MAX_COL
         (left half — has lines)                  (right half — holes only)
                  │                                            │
                  ▼                                            ▼
         followLine()  (line-follow PID)         rightHalfDriving?
         readSensors → getLineState                │
         → LINE_NORMAL / LOST /                    ├── false → encoderResetHop,
            JUNCTION                               │           resetHopHeading,
                  │                                │           motors=BASE_SPEED,
         if junctionJustHandled:                   │           rightHalfDriving=true
           advancePosOneCell                       │
           tryRfidAtNode                           └── true  → applyHeadingCorrection
              (RFID scan,                                      (consumes hopHeadingDeg
               sendIsFertile,                                   updated by main loop)
               → NAV_AT_TAG                                    if nearNextNode():
               or replan)                                        motors=0,
                                                                 endHopHeading,
                                                                 advancePosOneCell,
                                                                 tryRfidAtNode
```

## Planner

```
        tryRfidAtNode()
        (navigation.ino)
              │
       ┌──────┴──────┐
       │             │
   RFID hit       miss
       │             │
       ▼             ▼
   clearFertileResult   replanNextDir()
   sendIsFertile()             │
   navState = NAV_AT_TAG       ▼
                       ┌──── selectNextTarget(robotPos, targetPos)
                       │     prioritise TAG_FERTILE > TAG_UNKNOWN
                       │     skip TAG_INFERTILE / TAG_PLANTED
                       │     within tier: lowest Manhattan distance
                       │
                       └──── aStarNextStep(robotPos, targetPos, next)
                             A* on 9×9 grid, Manhattan heuristic,
                             unit edge costs, static scratch arrays
                                  │
                                  ▼
                             facingToward(robotPos, next)
                             getTurnDir(robotFacing, want)
                                  │
                                  ▼
                             pendingJunctionDir =
                             {-1, 0, +1, 2}
                             (consumed by handleJunction)
```

## WiFi / messaging

```
        MiniMessenger (wifi.ino)
                │
   ┌────────────┴───────────────┐
   │                            │
incoming                     outgoing
   │                            │
   ▼                            ▼
onMessage(payload):       wifiSend(msg)
parses key=value           ↑
strings                    │
   │              ┌────────┴──────────────┐
   │              │       │      │        │
   ▼          sendRegister sendIsFertile sendPlanted ...
matches:        sendPosition sendStatus
 type=heartbeat
   enable=1|0 → isEnabled
   refreshes lastHeartbeatMs
 type=emergency / disable
   → isEnabled = false
 type=isFertileReply
   → fertileResult.received = true,
     fertile/planted/tagId/x/y populated
```

The heartbeat watchdog in `wifiLoop()` flips `isEnabled` to false if `millis() - lastHeartbeatMs > HEARTBEAT_TIMEOUT_MS` (1 s). That false is what the safety gate at the top of `main.ino`'s `loop()` reads to stop motors.

## Encoder + hop-heading subsystem

```
   Hardware ISRs              Always-run in loop()              Consumers
   (motors.ino)               (main.ino)                        (navigation.ino)
                                                                 commands.ino
   encBL_A_ISR  ─┐
   encBL_B_ISR   │  → encBL                                     │
   encFL_A_ISR   │  → encFL  → leftTicks() ────┐                │
   encFL_B_ISR   │                              │── straightTicks() ─→ hopDistanceCm()
   encBR_A_ISR   │  → encBR  → rightTicks() ───┘                │   nearNextNode()
   encBR_B_ISR   │                                                ─→ calibRecordHop()
   encFR_A_ISR   │  → encFR                                     │   ticksPerCm lock
   encFR_B_ISR  ─┘
                                                                │
                          updateHopHeading()                     │
                          (no-op unless                          │
                           hopHeadingActive)                     │
                                  │                              │
                                  ▼                              │
                              hopHeadingDeg ─→ applyHeadingCorrection()
                                              motor differential
                                              while rightHalfDriving
```
