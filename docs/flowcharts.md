# Flowcharts

Mermaid diagrams for the four key control flows. They render on GitHub, GitLab, and most markdown viewers with Mermaid support; the raw source is still readable as plain text otherwise.

## 1. Line following

`followLine()` is called every tick while in `NAV_LINE_FOLLOW` (legacy test mode) or in `NAV_ARENA_NAV` on the left half of the arena.

```mermaid
flowchart TD
    A[followLine called] --> B[readSensors<br/>read 9-element IR array<br/>compute avg, sum]
    B --> C{currentKP == KP_AGGRESSIVE<br/>and aggressive window<br/>expired?}
    C -- yes --> D[currentKP = KP]
    C -- no --> E[getLineState<br/>classify reading]
    D --> E
    E --> F{LineState?}
    F -- LINE_NORMAL --> G[lastPosition = avg/sum<br/>error = lastPosition - LINE_CENTER<br/>correction = currentKP * error]
    G --> H[left  = BASE_SPEED + correction<br/>right = BASE_SPEED - correction<br/>scaleSpeed and apply via Motoron]
    F -- LINE_LOST --> I[motors = 0<br/>wait for line to reappear]
    F -- LINE_JUNCTION_LEFT --> J[handleJunction]
    F -- LINE_JUNCTION_RIGHT --> J
    F -- LINE_JUNCTION_BOTH --> J
    J --> K[currentKP = KP_AGGRESSIVE<br/>junctionExitTime = now]
    H --> L[return]
    I --> L
    K --> L
```

Key behavior: after every junction, the PID switches to a more aggressive `KP_AGGRESSIVE` gain for `AGGRESSIVE_DURATION_MS` to recapture the line, then falls back to the normal `KP`.

## 2. RFID + planting decision

State-machine version: the `tryRfidAtNode → NAV_AT_TAG → NAV_PLANTING` chain. Legacy `rfidLoop()` follows the same logic but is blocking inside one function.

```mermaid
flowchart TD
    A[navArenaTick arrives at a node] --> B[tryRfidAtNode<br/>PICC_IsNewCardPresent + ReadCardSerial]
    B --> C{tag detected?}
    C -- no --> D[replanNextDir<br/>selectNextTarget then aStarNextStep<br/>continue NAV_ARENA_NAV]
    C -- yes --> E[build UID string<br/>PICC_HaltA<br/>clearFertileResult<br/>sendIsFertile uid]
    E --> F[navState = NAV_AT_TAG<br/>atTagEnteredMs = now]
    F --> G[navAtTagTick: wifiPoll]
    G --> H{fertileResult.received?}
    H -- no --> I{now - atTagEnteredMs<br/>> FERTILE_REPLY_TIMEOUT_MS?}
    I -- no --> G
    I -- yes --> J[navState = NAV_ARENA_NAV<br/>print timeout]
    H -- yes --> K[update tagMap with TAG_FERTILE /<br/>TAG_INFERTILE / TAG_PLANTED<br/>calibRecordHop lastConfirmedPos, newPos<br/>robotPos = newPos<br/>lastConfirmedPos = newPos]
    K --> L{fertile and not planted<br/>and seedsRemaining > 0?}
    L -- no --> M[replanNextDir<br/>navState = NAV_ARENA_NAV]
    L -- yes --> N[navState = NAV_PLANTING]
    N --> O[navPlantingTick:<br/>sweepTo MAX, MIN<br/>sweepTo MIN, MAX<br/>blocking servo]
    O --> P[sendPlanted fertileResult.tagId<br/>tagMap robotPos = TAG_PLANTED<br/>seedsRemaining--]
    P --> Q{seedsRemaining == 0?}
    Q -- yes --> R[navState = NAV_PARKED]
    Q -- no --> S[replanNextDir<br/>navState = NAV_ARENA_NAV]
```

Key invariant: `fertileResult` is *not* cleared in `NAV_AT_TAG` — `NAV_PLANTING` still reads `fertileResult.tagId` to send `sendPlanted`. Clearing happens at the start of the next `tryRfidAtNode` via the `clearFertileResult` call inside it.

## 3. Emergency / kill switch

Three independent triggers can stop the robot. Forward-obstacle is the new entry; the others are existing safety paths.

```mermaid
flowchart TD
    subgraph TRIGGERS
        T1[heartbeat timeout<br/>more than 1 s since last heartbeat<br/>checked in wifiLoop]
        T2[server message:<br/>type=emergency<br/>type=disable<br/>type=heartbeat enable=0]
        T3[forward ultrasonic<br/>0 less-or-equal d less than OBSTACLE_STOP_CM<br/>checked at top of loop]
    end
    T1 --> U[isEnabled = false]
    T2 --> U
    T3 --> V[motors = 0<br/>skip body this tick<br/>sendStatus obstacle_stop once]
    U --> W[main loop safety gate:<br/>motors = 0<br/>handleNavDisable<br/>return]
    W --> X[navState = NAV_DISABLED<br/>rightHalfDriving = false<br/>endHopHeading<br/>holds here]
    X --> Y{isEnabled<br/>true again?}
    Y -- no --> X
    Y -- yes --> Z[navigationUpdate:<br/>navState = NAV_ARENA_NAV<br/>state machine resumes]
    V --> AA{forward distance<br/>cleared above<br/>OBSTACLE_STOP_CM?}
    AA -- no --> V
    AA -- yes --> AB[next tick: body<br/>runs normally<br/>state machine<br/>resumes in same state]
```

Behavioral split: an `isEnabled` drop forces `NAV_DISABLED` and needs the server to re-enable. A forward obstacle (within 8 cm) auto-resumes the moment the obstacle clears — this is the door-open path. The two paths use different code routes deliberately.

## 4. Junction handling

`handleJunction()` is invoked from `followLine()` when the IR array classifies a junction. The legacy path uses `junctionActions[]`; the state-machine path uses `pendingJunctionDir` from the planner.

```mermaid
flowchart TD
    A[handleJunction] --> B{inJunction?}
    B -- yes --> Z[return]
    B -- no --> C[inJunction = true]
    C --> D{useStateMachine?}
    D -- yes --> E[action = pendingJunctionDir<br/>print SM Junction action]
    D -- no --> F{junctionCount<br/>more-or-equal<br/>length junctionActions?}
    F -- yes --> G[motors = 0<br/>print all junctions complete<br/>return]
    F -- no --> H[action = junctionActions junctionCount++<br/>print Junction n action]
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

Key bits:
- `inJunction` is a re-entrancy guard — `followLine` can fire multiple `LINE_JUNCTION_*` states in a row as the IR array crosses the intersection.
- The `JUNCTION_FORWARD_MS` drive centres the chassis on the junction *before* spinning, so the spin pivots around the intersection rather than the IR array's leading edge.
- `spinUntilLine` is bang-bang with a gyro-tracked max angle (`JUNCTION_MAX_ROT_DEG`, 180°). Uturn (`action == 2`) doesn't have a dedicated break condition — it spins to max and bails.
