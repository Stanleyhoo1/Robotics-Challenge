# Flowcharts

Mermaid diagrams for the key control flows. They render on GitHub, GitLab, and most markdown viewers with Mermaid support; the raw source is still readable as plain text otherwise.

## 1. Line following

`followLineBase()` is the state-machine path's line-follow tick. Called from every base line-follow state (`NAV_BASE_TO_FIRST_JUNCTION`, `NAV_BASE_TO_TAG`, `NAV_BASE_TO_SECOND_JUNCTION`, `NAV_BASE_TO_LINE_LOST`, `NAV_BASE_RETURN`) and from `NAV_ARENA_NAV` while in the line zone (`robotPos.row >= LINE_ZONE_MIN_ROW`).

```mermaid
flowchart TD
    A[followLineBase called] --> B[readSensors<br/>read 9-element IR array<br/>compute avg, sum]
    B --> C{sum >= IR_MIN_LINE_SUM?}
    C -- yes --> D[lastPosition = avg/sum<br/>error = LINE_CENTER - lastPosition<br/>correction = KP * error]
    D --> E[left = constrain BASE_SPEED + correction,<br/>     LINE_FOLLOW_MIN_SPEED, 800<br/>right = constrain BASE_SPEED - correction,<br/>     LINE_FOLLOW_MIN_SPEED, 800<br/>scaleSpeed and apply via Motoron]
    C -- no --> F[motors = 0<br/>do NOT keep last commanded PWM]
    E --> G[return LineState classification]
    F --> G
    G --> H{caller inspects LineState}
    H -- JUNCTION_*<br/>in junction-aware state --> I[transition to TURN state]
    H -- LINE_LOST<br/>in unexpected-loss state --> J[baseLineLostRecovery]
    H -- LINE_LOST<br/>in NAV_BASE_TO_LINE_LOST --> K[transition to NAV_BASE_LINE_LOST_PAUSE]
    H -- LINE_LOST<br/>in NAV_BASE_RETURN --> L[sendStatus parked_line_end<br/>NAV_PARKED]
    H -- LINE_NORMAL --> M[continue]
```

Key behaviors:
- The `else` branch on `sum < IR_MIN_LINE_SUM` stops motors instead of letting the last PWM keep the wheels turning. This prevents drift past the line end.
- `LINE_FOLLOW_MIN_SPEED` (300) is the floor for the slow-wheel side of the PID. Below this the motor stalls and the wheel doesn't actually rotate, just buzzes.
- The legacy `followLine()` (only used in `NAV_LINE_FOLLOW` test mode) still has the older switch-on-LineState shape with `handleJunction()` inline.

## 2. RFID + planting decision

State-machine version: `pollRfidAndQueue → NAV_AT_TAG → NAV_POST_TAG_NUDGE → NAV_PLANTING` (or `→ NAV_ARENA_NAV`).

```mermaid
flowchart TD
    A[navArenaTick<br/>poll RFID every tick] --> B{tag detected?}
    B -- no --> C[continue driving<br/>line-zone PID or no-line-zone hop]
    B -- yes --> D[build UID, PICC_HaltA<br/>clearFertileResult<br/>sendIsFertile uid<br/>stop motors<br/>navState = NAV_AT_TAG<br/>atTagEnteredMs = now]
    D --> E[navAtTagTick wifiPoll]
    E --> F{fertileResult.received?}
    F -- no --> G{now - atTagEnteredMs<br/>> FERTILE_REPLY_TIMEOUT_MS?}
    G -- no --> E
    G -- yes --> H[print timeout<br/>navState = NAV_ARENA_NAV]
    F -- yes --> I[update tagMap with TAG_FERTILE /<br/>TAG_INFERTILE / TAG_PLANTED<br/>calibRecordHop lastConfirmedPos newPos<br/>robotPos = newPos]
    I --> J{returning and<br/>robotPos == AIRLOCK_B?}
    J -- yes --> K[stop, navState = NAV_AT_AIRLOCK_B]
    J -- no --> L{fertile and not planted<br/>and seedsRemaining > 0?}
    L -- yes --> M[postTagIntentPlanting = true<br/>navState = NAV_POST_TAG_NUDGE]
    L -- no --> N[replanNextDir]
    N --> O{pendingJunctionDir == 0?}
    O -- yes --> P[navState = NAV_ARENA_NAV]
    O -- no --> Q[postTagIntentPlanting = false<br/>navState = NAV_POST_TAG_NUDGE]
    M --> R[NAV_POST_TAG_NUDGE:<br/>encoder-driven 5 cm forward<br/>centres wheel axis on hole/tag]
    Q --> R
    R --> S{postTagIntentPlanting?}
    S -- yes --> T[navState = NAV_PLANTING]
    S -- no --> U[turnDegrees pendingJunctionDir × 90°<br/>navState = NAV_ARENA_NAV]
    T --> V[navPlantingTick<br/>sweepTo MAX, MIN<br/>sweepTo MIN, MAX<br/>sendPlanted]
    V --> W[tagMap robotPos = TAG_PLANTED<br/>seedsRemaining--]
    W --> X{seedsRemaining == 0?}
    X -- yes --> Y[returning = true<br/>replanNextDir targets AIRLOCK_B<br/>turn if needed → NAV_ARENA_NAV]
    X -- no --> Z[replanNextDir<br/>turn if needed → NAV_ARENA_NAV]
```

Key invariants:
- `fertileResult` is *not* cleared in `NAV_AT_TAG`. `NAV_PLANTING` still reads `fertileResult.tagId` to send `sendPlanted`. Clearing happens at the start of the next `pollRfidAndQueue` via `clearFertileResult`.
- The `POST_TAG_FORWARD_CM` (5 cm) nudge is used in three places with identical semantics: post-tag nudge before plant/turn, base junction pre-turn, and base line-lost recovery. Same encoder-based path each time.

## 3. Emergency / kill switch

Four independent triggers can pause or stop the robot.

```mermaid
flowchart TD
    subgraph TRIGGERS
        T1[heartbeat timeout<br/>lastHeartbeatMs != 0<br/>and now - last > 1 s<br/>checked in wifiLoop]
        T2[server message:<br/>type=emergency<br/>type=disable<br/>type=heartbeat enable=0]
        T3[forward ultrasonic<br/>0 less-or-equal d less than OBSTACLE_STOP_CM<br/>2-tick debounce]
        T4[power button pin 17<br/>HIGH→LOW edge<br/>200 ms debounce]
    end
    T1 --> U[isEnabled = false]
    T2 --> U
    T4 --> U
    T3 --> V[motors = 0<br/>skip body this tick<br/>sendStatus obstacle_stop on rising edge<br/>sendStatus obstacle_cleared on falling edge]
    U --> W[main loop safety gate:<br/>motors = 0<br/>if useStateMachine: handleNavDisable<br/>return]
    W --> X[navState PRESERVED<br/>deadReckonDriving = false<br/>endHopHeading<br/>hold here]
    X --> Y{isEnabled<br/>true again?}
    Y -- no --> X
    Y -- yes --> Z[next tick: navigationUpdate runs<br/>SAME navState as before<br/>tick re-engages motors]
    V --> AA{forward distance<br/>cleared above<br/>OBSTACLE_STOP_CM?}
    AA -- no --> AB{NAV_WALL_FOLLOW or<br/>NAV_TUNNEL_B_WALL_FOLLOW?}
    AB -- yes --> AC[every DOOR_RETRY_INTERVAL_MS<br/>resend openAirlockA/B]
    AC --> V
    AB -- no --> V
    AA -- yes --> AD[next tick: body runs<br/>state machine resumes<br/>in same state]
```

Behavioral split:
- An `isEnabled` drop **does not change `navState`**. Re-enable resumes whatever state was active before the pause. This is a deliberate change from earlier versions that collapsed to `NAV_DISABLED` + auto-resumed to `NAV_ARENA_NAV`.
- A forward obstacle auto-resumes the moment the obstacle clears (door opens / object moved). This is the door-open path. Same-tick resumption.
- `NAV_BASE_RETURN` is special-cased in the obstacle gate: hitting something in front while returning ends the run (`parked_obstacle` → `NAV_PARKED`).

## 4. Base exit sequence

From button press in the base to arena nav. All the new state-machine work since the original docs.

```mermaid
flowchart TD
    A[boot: navState = NAV_BASE_TO_FIRST_JUNCTION<br/>useStateMachine = true<br/>isEnabled = false] --> B[power button press<br/>isEnabled = true]
    B --> C[NAV_BASE_TO_FIRST_JUNCTION<br/>followLineBase]
    C --> D{LineState?}
    D -- JUNCTION_* --> E[stop, NAV_BASE_FIRST_TURN]
    D -- LINE_LOST --> F[baseLineLostRecovery<br/>see flowchart 5]
    F -- found --> C
    F -- not found --> G[sendStatus base_recovery_failed<br/>NAV_PARKED]
    E --> H[baseTurnBlocking +90°:<br/>pre-turn 5 cm nudge<br/>turnDegrees<br/>post-turn 150 ms forward kick]
    H --> I[NAV_BASE_TO_TAG<br/>followLineBase + RFID poll]
    I --> J{RFID hit?}
    J -- yes --> K[sendOpenAirlockA<br/>NAV_WAIT_EXIT_CLEARANCE]
    J -- no --> L{LINE_LOST?}
    L -- yes --> F
    L -- no --> I
    K --> M[hold, wifiPoll<br/>resend openAirlockA every<br/>DOOR_RETRY_INTERVAL_MS]
    M --> N{exitClearanceReceived?}
    N -- no --> M
    N -- yes --> O[NAV_BASE_TO_SECOND_JUNCTION<br/>followLineBase]
    O --> P{LineState?}
    P -- JUNCTION_* --> Q[stop, NAV_BASE_SECOND_TURN]
    P -- LINE_LOST --> F
    Q --> R[baseTurnBlocking -90°:<br/>pre-turn 5 cm nudge<br/>turnDegrees<br/>post-turn forward kick]
    R --> S[NAV_BASE_TO_LINE_LOST<br/>followLineBase]
    S --> T{LINE_LOST?<br/>tunnel mouth reached}
    T -- no --> S
    T -- yes --> U[stop motors<br/>NAV_BASE_LINE_LOST_PAUSE<br/>momentary hold]
    U --> V[hold BASE_LINE_LOST_PAUSE_MS<br/>1500 ms]
    V --> W[NAV_BASE_FORWARD_NUDGE<br/>delay BASE_FORWARD_NUDGE_MS<br/>at BASE_SPEED]
    W --> X[NAV_WALL_FOLLOW<br/>through Tunnel A]
    X --> Y{IR array<br/>sees line?}
    Y -- no --> X
    Y -- yes --> Z[seed robotPos = AIRLOCK_A<br/>robotFacing = ARENA_ENTRY_FACING<br/>NAV_ARENA_NAV]
```

## 5. Line-lost recovery (base only)

Triggered from `NAV_BASE_TO_FIRST_JUNCTION`, `NAV_BASE_TO_TAG`, `NAV_BASE_TO_SECOND_JUNCTION` when `followLineBase()` returns `LINE_LOST`. **Not** used in `NAV_BASE_TO_LINE_LOST` (line ending is expected) or `NAV_BASE_RETURN` (line ending = parking).

```mermaid
flowchart TD
    A[baseLineLostRecovery<br/>blocking] --> B[encoderResetHop<br/>motors = BASE_SPEED forward]
    B --> C{hopDistanceCm<br/>>= POST_TAG_FORWARD_CM?}
    C -- no --> D[wifiLoop, delay 5 ms]
    D --> E{isEnabled?}
    E -- no --> F[stop, handleNavDisable<br/>return false]
    E -- yes --> C
    C -- yes --> G[stop motors<br/>readSensors]
    G --> H{sum >= IR_MIN_LINE_SUM?<br/>line under array}
    H -- yes --> I[print line re-acquired after nudge<br/>return true]
    H -- no --> J[sweepForLine -1, 90°<br/>spin left up to 90°<br/>poll readSensors every iter]
    J --> K{line found<br/>during sweep?}
    K -- yes --> L[stop on line<br/>return true]
    K -- no --> M[sweepForLine +1, 180°<br/>spin right through centre<br/>to +90° relative to start]
    M --> N{line found?}
    N -- yes --> L
    N -- no --> O[sweepForLine -1, 90°<br/>re-centre best-effort<br/>return value ignored]
    O --> P[return false]
    P --> Q[caller: sendStatus<br/>base_recovery_failed<br/>NAV_PARKED]
    L --> R[caller: stay in same state<br/>next tick: followLineBase<br/>resumes PID on the line]
    I --> R
```

The total angular search range is ±90° from start. Each sweep stops the instant a line appears. `wifiLoop()` is called every iteration so the heartbeat survives a multi-second recovery.

## 6. Status LED priority stack

`updateLED()` runs every tick inside `wifiLoop()`.

```mermaid
flowchart TD
    A[updateLED] --> B{REVIVE_BUTTON_1 LOW or<br/>REVIVE_BUTTON_2 LOW?}
    B -- yes --> C[setLED LOW, HIGH<br/>solid green]
    B -- no --> F{isEnabled?}
    F -- yes --> G[setLED HIGH, LOW<br/>solid red]
    F -- no --> H{millis - lastBlinkMs<br/>> LED_BLINK_INTERVAL_MS?}
    H -- no --> I[no change<br/>previous blink state holds]
    H -- yes --> J[blinkState = !blinkState<br/>setLED blinkState ? HIGH : LOW, LOW]
    C --> K[return]
    G --> K
    I --> K
    J --> K
```

Revive buttons (52/53) are the only override. Otherwise the LED is solid red while `isEnabled` is true and blinks red at `LED_BLINK_INTERVAL_MS` while disabled.

## 7. Return-to-base sequence

After the last seed is planted, `returning = true` and `replanNextDir` targets `AIRLOCK_B` (row 6, col 8).

```mermaid
flowchart TD
    A[navPlantingTick<br/>seedsRemaining-- → 0] --> B[returning = true<br/>replanNextDir → targetPos = AIRLOCK_B<br/>turn if needed]
    B --> C[NAV_ARENA_NAV<br/>standard hop + A* + RFID]
    C --> D{robotPos == AIRLOCK_B?}
    D -- no --> C
    D -- yes --> E[NAV_AT_AIRLOCK_B<br/>compute baseDir = ARENA_ENTRY + 180°<br/>rotate in place to face base]
    E --> F[sendOpenAirlockB<br/>NAV_WAIT_ENTER_CLEARANCE]
    F --> G[hold, wifiPoll<br/>resend openAirlockB every<br/>DOOR_RETRY_INTERVAL_MS]
    G --> H{enterClearanceReceived?}
    H -- no --> G
    H -- yes --> I[NAV_TUNNEL_B_WALL_FOLLOW<br/>wallFollow through Tunnel B]
    I --> J{IR array sees line?}
    J -- no --> I
    J -- yes --> K[NAV_BASE_RETURN<br/>followLineBase]
    K --> L{LineState?}
    L -- LINE_NORMAL --> K
    L -- LINE_LOST --> M[sendStatus parked_line_end<br/>NAV_PARKED]
    L -- obstacle in front<br/>top-of-loop --> N[sendStatus parked_obstacle<br/>NAV_PARKED]
```

Notes:
- The door-open detection in both tunnel transitions watches the forward ultrasonic, not the clearance message. The clearance flag is set by `onMessage` for verification only.
- `BASE_RETURN` ends on the first of (a) the line ending naturally, or (b) hitting something in front (per the spec: "follow it until no more line or we detect something in front and can't move").

## 8. Junction handling (legacy)

`handleJunction()` is invoked from `followLine()` (legacy `NAV_LINE_FOLLOW` test mode only). The state-machine path uses `baseTurnBlocking()` for base junctions and `NAV_POST_TAG_NUDGE` for arena turns. Both are simpler and don't go through this function.

```mermaid
flowchart TD
    A[handleJunction] --> B{inJunction?}
    B -- yes --> Z[return]
    B -- no --> C[inJunction = true]
    C --> D{useStateMachine?}
    D -- yes --> E[action = pendingJunctionDir]
    D -- no --> F{junctionCount<br/>more-or-equal<br/>length junctionActions?}
    F -- yes --> G[motors = 0<br/>all junctions complete<br/>return]
    F -- no --> H[action = junctionActions junctionCount++]
    E --> I[motors = BASE_SPEED both<br/>delay JUNCTION_FORWARD_MS<br/>drive past junction centre]
    H --> I
    I --> J{action == 0?<br/>straight}
    J -- yes --> K[skip spin]
    J -- no --> L[spinUntilLine action<br/>spin until line reappears<br/>on the correct side<br/>then nudge JUNCTION_NUDGE_MS]
    K --> M[delay 100 ms<br/>currentKP = KP_AGGRESSIVE<br/>junctionExitTime = now<br/>inJunction = false]
    L --> M
    M --> N{useStateMachine?}
    N -- yes --> O[robotFacing = facingAfterTurn action<br/>junctionJustHandled = true]
    N -- no --> P[return]
    O --> P
```

Kept for the test-mode path and for reference, but the production base-exit and arena flows do not use it.

flowchart TD
    A[handleJunction] --> B{inJunction?}
    B -- yes --> Z[return]
    B -- no --> C[inJunction = true]
    C --> D{useStateMachine?}
    D -- yes --> E[action = pendingJunctionDir]
    D -- no --> F{junctionCount<br/>more-or-equal<br/>length junctionActions?}
    F -- yes --> G[motors = 0<br/>all junctions complete<br/>return]
    F -- no --> H[action = junctionActions junctionCount++]
    E --> I[motors = BASE_SPEED both<br/>delay JUNCTION_FORWARD_MS<br/>drive past junction centre]
    H --> I
    I --> J{action == 0?<br/>straight}
    J -- yes --> K[skip spin]
    J -- no --> L[spinUntilLine action<br/>spin until line reappears<br/>on the correct side<br/>then nudge JUNCTION_NUDGE_MS]
    K --> M[delay 100 ms<br/>currentKP = KP_AGGRESSIVE<br/>junctionExitTime = now<br/>inJunction = false]
    L --> M
    M --> N{useStateMachine?}
    N -- yes --> O[robotFacing = facingAfterTurn action<br/>junctionJustHandled = true]
    N -- no --> P[return]
    O --> P

##9. Path planning for second half of arena (Prospective Flowchart)
```mermaid
    flowchart TD
    A[Scan RFID] --> B{Is RFID_pos = row5?}
    B -- Yes -->C[readSensors read 9-element IR array<br/>compute avg, sum<br/>update lineCurrentlyDetected]
    B --No --> D[followLineBase called]
    C --> E{sum >= IR_MIN_LINE_SUM?}
    E -- No --> F[choose direction + A* algorithm]
    E -- Yes --> G[baseLineLostRecovery blocking]
    F --> H[IMU steer to direction]
    H --> I[Encoders rotate motors 25cm]
    I --> J{RFID Scanned?}
    J -- Yes --> K[navArenaTick<br/>poll RFID every tick]
    K --> L[Planting function called]
    J -- No --> L[Check orientation]
    L --> I
    K --> M[Planting function called]
```
