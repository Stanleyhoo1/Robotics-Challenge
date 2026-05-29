# Flowcharts

Mermaid diagrams for the key control flows. They render on GitHub, GitLab, and most markdown viewers with Mermaid support; the raw source is still readable as plain text otherwise.

## 1. Line following

`followLineBase()` is the line-follow tick used inside the base and in the line zone of the arena (rows 4–8).

```mermaid
flowchart TD
    A["followLineBase called"] --> B["read IR array"]
    B --> C{"line under array?"}
    C -->|yes| D["PID correction<br/>drive motors"]
    C -->|no| E["stop motors"]
    D --> F["return LineState"]
    E --> F
    F --> G{"LineState?"}
    G -->|JUNCTION| H["transition to turn state"]
    G -->|LINE_LOST<br/>unexpected| I["baseLineLostRecovery"]
    G -->|LINE_LOST<br/>expected end| J["next state<br/>(pause / park)"]
    G -->|LINE_NORMAL| K["continue"]
```

Key behaviors:
- When the line is lost, motors are zeroed instead of holding the last PWM. Prevents drift past the line end.
- `LINE_FOLLOW_MIN_SPEED` is the floor for the slow-wheel side of the PID — below it the motor stalls and just buzzes.

## 2. RFID + planting decision

Runs on every arena tick. Scan the tag, ask the server what it is, act on the answer.

```mermaid
flowchart TD
    A["scan RFID"] --> B{"tag found?"}
    B -->|no| C["keep driving"]
    B -->|yes| D{"in base?"}
    D -->|yes| E["request airlock A (outbound)<br/>or B (returning)"]
    D -->|no| F["request fertility"]
    F --> G["update map:<br/>fertile / infertile / planted"]
    G --> H{"fertile and not planted<br/>and seeds left?"}
    H -->|yes| I["plant seed<br/>mark planted on map"]
    H -->|no| J["pick next target<br/>from map"]
    I --> J
    J --> C
```

Notes:
- "Pick next target" prioritizes fertile-unplanted, then unknown, then everything else, by Manhattan distance.
- Once seeds are exhausted, the next target becomes airlock B (return path).

## 3. Emergency / kill switch

Four independent triggers can pause or stop the robot.

```mermaid
flowchart TD
    T1["heartbeat timeout"] --> U["isEnabled = false"]
    T2["server: emergency / disable"] --> U
    T3["power button"] --> U
    T4["forward obstacle"] --> V["stop motors this tick"]
    U --> W["safety gate:<br/>stop motors<br/>preserve navState"]
    W --> X{"isEnabled again?"}
    X -->|no| W
    X -->|yes| Y["resume same state"]
    V --> Z{"obstacle cleared?"}
    Z -->|no| V
    Z -->|yes| AA["resume same state"]
```

Behavioral split:
- An `isEnabled` drop **does not change `navState`**. Re-enable resumes whatever state was active before the pause.
- A forward obstacle auto-resumes the moment the obstacle clears (door opens / object moved).
- `NAV_BASE_RETURN` is special-cased in the obstacle gate: hitting something in front while returning ends the run.

## 4. Base exit sequence

From boot to arena nav.

```mermaid
flowchart TD
    A["boot: disabled"] --> B["power button press"]
    B --> C["follow line"]
    C --> D{"junction?"}
    D -->|no| C
    D -->|yes| E["turn +90°"]
    E --> F["follow line + RFID poll"]
    F --> G{"RFID hit?"}
    G -->|no| F
    G -->|yes| H["request airlock A"]
    H --> I{"clearance?"}
    I -->|no| I
    I -->|yes| J["follow line"]
    J --> K{"junction?"}
    K -->|no| J
    K -->|yes| L["turn -90°"]
    L --> M["follow line to tunnel mouth"]
    M --> N{"line lost?"}
    N -->|no| M
    N -->|yes| O["nudge forward into tunnel"]
    O --> P["wall-follow through Tunnel A"]
    P --> Q{"IR sees line?"}
    Q -->|no| P
    Q -->|yes| R["enter arena nav"]
```

## 5. Line-lost recovery (base only)

Triggered when `followLineBase()` returns `LINE_LOST` in a state where the line is *not* expected to end.

```mermaid
flowchart TD
    A["nudge forward 5 cm"] --> B{"line under array?"}
    B -->|yes| C["return success"]
    B -->|no| D["sweep left 90°"]
    D --> E{"line found?"}
    E -->|yes| C
    E -->|no| F["sweep right 180°"]
    F --> G{"line found?"}
    G -->|yes| C
    G -->|no| H["recenter<br/>return failure"]
    H --> I["caller: park"]
```

Total angular search range is ±90° from start. Each sweep stops the instant a line appears.

## 6. Status LED priority stack

`updateLED()` runs every tick.

```mermaid
flowchart TD
    A["updateLED"] --> B{"revive button pressed?"}
    B -->|yes| C["solid green"]
    B -->|no| D{"isEnabled?"}
    D -->|yes| E["solid red"]
    D -->|no| F["blink red"]
```

Revive buttons are the only override. Otherwise the LED is solid red while enabled and blinks red while disabled.

## 7. Return-to-base sequence

After the last seed is planted, the planner targets airlock B and the run reverses.

```mermaid
flowchart TD
    A["last seed planted"] --> B["target = airlock B"]
    B --> C["arena nav"]
    C --> D{"at airlock B?"}
    D -->|no| C
    D -->|yes| E["rotate to face base"]
    E --> F["request airlock B"]
    F --> G{"clearance?"}
    G -->|no| G
    G -->|yes| H["wall-follow through Tunnel B"]
    H --> I{"IR sees line?"}
    I -->|no| H
    I -->|yes| J["follow line"]
    J --> K{"line ended<br/>or obstacle?"}
    K -->|no| J
    K -->|yes| L["parked"]
```

Notes:
- Door-open detection in both tunnels watches the forward ultrasonic, not the clearance message. Clearance is for verification only.
- `BASE_RETURN` ends on whichever happens first: the line ending naturally, or an obstacle in front.

## 8. Junction handling (legacy)

`handleJunction()` is only used by the legacy `NAV_LINE_FOLLOW` test path. The state-machine path uses `baseTurnBlocking()` for base junctions and `NAV_POST_TAG_NUDGE` for arena turns instead — both are simpler and don't go through this function.

```mermaid
flowchart TD
    A["handleJunction"] --> B{"already in junction?"}
    B -->|yes| C["return"]
    B -->|no| D["drive past junction centre"]
    D --> E{"action = straight?"}
    E -->|yes| F["skip spin"]
    E -->|no| G["spin until line reappears"]
    F --> H["bump KP to aggressive<br/>exit junction"]
    G --> H
```

Kept for the test-mode path and for reference; production base-exit and arena flows do not use it.
