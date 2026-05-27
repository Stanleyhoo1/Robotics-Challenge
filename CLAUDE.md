# Robotics-Challenge

UCL robotics competition robot. Starts in an underground **Base** (1.2 × 2.4 m), exits via an airlock tunnel to the **Main Arena** (2.5 × 2.5 m surface), plants seeds at fertile RFID-tagged holes on a 9×9 grid, then returns to base. Begins each run with **5 seeds max**.

## Working directory

All firmware lives in `main/` — a multi-file Arduino sketch (`.ino` + `.h`). Work inside `main/` for code changes; the repo root is for git/docs/scripts.

---

## Competition layout

### Base (1.2 × 2.4 m, underground)
- **Tunnel A** (red): entrance *into* base from arena, ≤20° ramp, ~1.2 m
- **Tunnel B** (green): exit *from* base to arena
- Each tunnel is an **airlock** with two sets of doors — robot must wait for sequencing, cannot force them
- Two RFID tags inside the base:
  - **A** — driven over by *another robot* to open Airlock A's surface door (multi-robot coordination, handled server-side)
  - **B** — robot sends its ID to request exit clearance via Tunnel B

### Main Arena (9×9 grid of RFID tags A1–I9)
- **Left half (~columns A–E):** solid black lines connect holes → line-following works
- **Right half (~columns F–I):** holes only, no lines → navigate by gyro heading + encoder ticks
- Grid spacing: **2.5 m across 9 tags → ~31.25 cm per cell** (8 gaps). Tune via physical measurement.
- Tag fertile/infertile status is randomized at run start; robot must query the server per tag

### Planting rules
- A "seed" is a ½-inch sphere dropped into a 25 mm × ~12 mm hole
- Only plant if server confirms `fertile=true` AND `planted=false`
- Max 5 seeds per robot per run

---

## Navigation strategy

### Unified grid movement
Robot always moves along grid lines (cardinal N/S/E/W between adjacent tags), on **both** halves. The *sensor* used differs by half, but path-planning and state-machine logic stay identical:

- **Left half:** line sensor tracks the lane; RFID gives position fix on arrival
- **Right half:** gyro heading-lock keeps a straight line; encoder ticks measure ~31 cm of travel; RFID gives position fix on arrival

Free/diagonal movement is a future extension once grid nav is reliable.

### Position tracking
- **Primary fix:** RFID coordinate on every successful scan resets estimated position to ground truth — kills accumulated drift
- **Between-tag dead reckoning:** gyro (LSM6, already calibrated at boot) for heading lock + encoder ticks for distance. Best-effort only; the next RFID fix corrects any error
- **Encoders:** ⚠️ hardware present (2 per side, 4 total), **software TODO** — no encoder code in `motors.ino` yet. Calibration plan below; until wired, right-half dead-reckoning falls back to timed runs at known speed
- **Missed reads:** if RFID fails on arrival, retry once in place, then continue. Log misses via `sendStatus()` for post-run debugging — don't stall

### Pathfinding (two separate concerns)

**1. Target selection — which tag next:**
- Maintain a 9×9 map of tag states: `UNKNOWN | FERTILE | INFERTILE | PLANTED`
- Priority: `FERTILE+unplanted` > `UNKNOWN` > `INFERTILE/PLANTED`
- After each plant or skip, pick nearest highest-priority tag by Manhattan distance
- Once 5 seeds planted (or exhausted), target = Tunnel A entrance node

**2. Routing — how to get there:**
- A* on the 9×9 grid with Manhattan-distance heuristic
- All edges cost 1 (uniform grid, no obstacles modeled yet)
- Output = waypoint sequence → turn commands at each junction via `turnDegrees()`
- Left half: detect junctions via line sensor and turn. Right half: turn in place, then dead-reckon one cell forward.

⚠️ **`JUNCTION_SEQUENCE` in `config.h` is test/debug only** — real runs use the A* planner.

---

## State machine

Lives in `navigation.ino`. `loop()` calls `navigationUpdate()` which dispatches on `navState`. `DISABLED` is a global override — any state transitions to it immediately on heartbeat timeout or emergency (set by `wifi.ino` clearing `isEnabled`).

```
BOOT
 └─► BASE_LINE_FOLLOW       (line-follow inside base to tag B)
      └─► WAIT_EXIT_CLEARANCE   (send exitRequest, block for exitClearance)
           └─► TUNNEL_EXIT      (wall-follow through Tunnel B to arena)
                └─► ARENA_NAV   (A* + grid movement, main loop)
                     ├─► AT_TAG     (at node, read RFID + query server)
                     │    └─► PLANTING   (servo sweep, sendPlanted)
                     │         └─► ARENA_NAV   (pick next target)
                     └─► RETURNING  (seeds exhausted → Tunnel A entrance)
                          └─► WAIT_ENTER_CLEARANCE (send enterRequest, wait)
                               └─► TUNNEL_ENTER   (wall-follow through Tunnel A)
                                    └─► BASE_RETURN  (line-follow to parking)
                                         └─► PARKED
```

| State | Entry action | Update action | Exit condition |
|---|---|---|---|
| `BASE_LINE_FOLLOW` | — | Line-follow toward tag B | RFID reads tag B |
| `WAIT_EXIT_CLEARANCE` | `sendExitRequest(tagB_uid)` | Idle, LED blink | `exitClearance` received |
| `TUNNEL_EXIT` | Start wall-follow | Wall-follow forward | FORWARD ultrasonic detects open space |
| `ARENA_NAV` | Compute next waypoint via A* | Move one cell (line or dead-reckon) | Arrived at target node |
| `AT_TAG` | Read RFID, `sendIsFertile(uid)` | Wait for `isFertileReply` | Reply received |
| `PLANTING` | Start servo `sweepTo()` | Wait for sweep complete | Sweep done + `sendPlanted()` |
| `RETURNING` | Set target = Tunnel A node | Same as `ARENA_NAV` | Arrived at Tunnel A entrance |
| `WAIT_ENTER_CLEARANCE` | `sendEnterRequest()` | Idle | `enterClearance` received |
| `TUNNEL_ENTER` | Start wall-follow | Wall-follow forward | FORWARD ultrasonic detects base interior |
| `BASE_RETURN` | — | Line-follow toward parking | RFID reads parking tag (or line ends) |
| `PARKED` | Stop motors, send status | Heartbeat only | — |
| `DISABLED` | Stop motors immediately | Await re-enable heartbeat | `isEnabled` true again |

### Encoder calibration (self-calibrating CPR)

Datasheet CPR is inaccurate — don't hard-code it. Estimate empirically from RFID ground-truth fixes during the first few arena traversals.

**Key insight:** CPR and wheel diameter don't need to be known separately. The only value needed is:
```
ticksPerCm = ticks_counted_on_straight_hop / GRID_SPACING_CM
```
This single float absorbs all systematic hardware error.

**Algorithm:**
- On every confirmed straight tag-to-tag hop (Manhattan distance == 1), record `straightTicks() / GRID_SPACING_CM` as one sample
- Reject outliers >20% from running mean (catches stalls, nudges, bad fixes)
- After `CALIB_MIN_SAMPLES` (default 4) accepted samples, lock the estimate
- Before lock: use `TICKS_PER_CM_FALLBACK` — robot navigates, just less accurately
- Log the lock event via `sendStatus()`

**4-motor encoder averaging** (two encoders per side):
- `leftTicks()  = (enc_left_front  + enc_left_rear)  / 2`
- `rightTicks() = (enc_right_front + enc_right_rear) / 2`
- `straightTicks() = (leftTicks() + rightTicks()) / 2`

**Node arrival trigger:** `hopDistanceCm() >= GRID_SPACING_CM * 0.85` — the 0.85 threshold gives deceleration time before the hole.

**Where things live:**
- Reference sketch: `encoder_calibration_sketch.ino` (TODO: doesn't exist yet)
- Constants in `config.h`: `GRID_SPACING_CM`, `TICKS_PER_CM_FALLBACK`, `CALIB_MIN_SAMPLES`, `CALIB_OUTLIER_PCT`
- Globals in `globals.h` (defined in `motors.ino`): `calibTicksPerCm`, `calibLocked`, `calibSampleCount`

### Encoder wiring plan (not yet implemented)

Quadrature encoders, 2 channels each (A, B). Planned pin assignments:

| Wheel | A pin | B pin |
|---|---|---|
| Back left   | 22 | 23 |
| Front left  | 24 | 25 |
| Back right  | 26 | 27 |
| Front right | 28 | 29 |

Motoron channel mapping: **left motor = channel 1, right motor = channel 2**.

**ISR pattern (from user's reference sketch):** quadrature decode that fires on both A and B edges. Direction comes from `(a == b)`:

```cpp
void encoderISR_A() {
  bool a = digitalRead(ENC_A);
  bool b = digitalRead(ENC_B);
  if (a != lastA) {
    encoderCount += (a == b) ? -1 : 1;
    lastA = a;
  }
}
// encoderISR_B mirrors it; sign flips because B leads A in the opposite direction
```

**Interrupts on the Giga:** every digital pin supports EXTI, so `attachInterrupt(digitalPinToInterrupt(pin), isr, CHANGE)` works on all 8 encoder lines. One ISR per channel (A and B both fire on CHANGE) gives 4× decoding resolution and correct direction even at high RPM. Keep ISRs short — read both pins, update count, return.

### Tunnel wall-following (used by `TUNNEL_EXIT` / `TUNNEL_ENTER`)
- Rolling window of LEFT + RIGHT ultrasonic readings
- PID (or bang-bang) on wall-distance difference to correct heading while driving forward
- **Watch the ramp** — IMU pitch changes; don't confuse tilt for a turn

---

## Hardware (from `main/config.h`)

- **MCU:** Arduino Giga R1 WiFi (STM32H747 dual-core M7+M4, Mega form factor). Two I2C buses: `Wire` on the Mega-row pins 20/21, `Wire1` on the dedicated SDA1/SCL1 pads. mbed-based — `kvstore_global_api` is available for persistent storage. **All digital pins are EXTI-capable** (no Mega-style interrupt restrictions).
- **Drive:** Motoron I2C motor controller on `Wire1` (addr 16), 2 motors
- **IMU:** LSM6 on `Wire1`, gyro-only for turn control (calibrated at boot, ±250 dps)
- **Line sensor:** 9-element IR array on pins 30–38, control pin 12, calibration persisted via `kvstore_global_api`
- **Encoders:** hardware on the robot (4× quadrature, 2 per side), **not yet wired in firmware**. Planned pins 22–29 — all EXTI-capable on the Giga, so `attachInterrupt(digitalPinToInterrupt(pin), isr, CHANGE)` works directly. See "Encoder wiring plan".
- **Ultrasonics:** 3× HC-SR04 — LEFT (40/41), RIGHT (42/43), FORWARD (44/45)
- **RFID:** MFRC522 on default `Wire` at 0x28
- **Servo:** pin 9, sweeps MIN_ANGLE (60°) ↔ MAX_ANGLE (160°) to plant
- **Status LED:** RGB on 50/51/52. Solid red = enabled, blinking red = disabled
- **WiFi:** `MiniMessenger` library, broker-based pub/sub. Board id is `"Master"`

---

## File layout (`main/`)

Arduino concatenates `.ino` files alphabetically with `main.ino` first. Filenames are deliberately ordered so cross-file references resolve without forward declarations.

| File | Owns |
|---|---|
| `main.ino` | `setup()`/`loop()`, defines `showIR` / `showDistance` / `isEnabled` |
| `config.h` | **All tunable constants** — pins, speeds, KP gains, junction sequence, seed count, board id |
| `globals.h` | `extern` declarations for cross-`.ino` variables |
| `types.h` | `LineState`, `DistanceSensor`, `FertileResult` (`char[]` not `String` so it's safe in headers) |
| `secrets.h` | WiFi creds, broker host/port, team id. **Do not commit.** (gitignored) |
| `motors.ino` | Motoron + LSM6, `turnDegrees()` (gyro-tracked with slow zone), `moveForward()`, `scaleSpeed()` voltage compensation, `applyMotorEnabled()` |
| `navigation.ino` | Line-follow + junction handling + state machine. **Named `n*` so it sorts after `motors.ino`** and can use its globals |
| `ir_sensor.ino` | 9-sensor read loop, kvstore-backed min/max calibration |
| `distance.ino` | HC-SR04 wrappers, `getDistanceCM(SENSOR_LEFT\|RIGHT\|FORWARD)` |
| `rfid.ino` | Tag scan → `sendIsFertile` → block for reply → servo-plant + `sendPlanted` |
| `servo.ino` | `sweepTo(from, to)` |
| `wifi.ino` | `MiniMessenger` setup, `onMessage` parser, send helpers, heartbeat watchdog, LED status |
| `commands.ino` | Serial debug commands |

---

## Conventions

- **Constants → `config.h`.** Don't sprinkle magic numbers in `.ino` files.
- **Cross-file globals:** declare `extern` in `globals.h`, define in exactly one `.ino`. The header comment block tracks which file owns each — keep it accurate.
- **No `String` in shared headers.** `FertileResult` uses `char[]` because Arduino's `String` needs `Arduino.h` to be fully processed first.
- **Voltage compensation:** wrap PWM speeds in `scaleSpeed()` when input voltage differs from `MOTOR_VOLTAGE`. Raw `setSpeedNow()` calls inside `turnDegrees()` skip it deliberately — they're already in calibrated gyro-feedback territory.

---

## Server protocol

Key=value space-separated strings over `MiniMessenger`. Robot is `board_id=Master`; team is `GROUP_ID` from `secrets.h`.

### Implemented

**Incoming (`onMessage` in `wifi.ino`):**
- `type=heartbeat enable=1|0` — gates motors. Must arrive within `HEARTBEAT_TIMEOUT_MS` (1s) or motors cut.
- `type=emergency` / `type=disable` — immediate disable
- `type=isFertileReply tag_id=... fertile=true|false planted=true|false x=N y=N`

**Outgoing (helpers in `wifi.ino`):**
- `sendRegister()` — re-sent every 10s
- `sendIsFertile(uid)` / `sendPlanted(uid)`
- `sendPosition(x, y)` / `sendStatus(msg)`

### Required (not yet implemented — design alongside state machine)

| Direction | Type | Purpose |
|---|---|---|
| Outgoing | `type=exitRequest tag_id=<B_uid>` | Request exit via Tunnel B |
| Incoming | `type=exitClearance` | Server grants exit |
| Outgoing | `type=enterRequest board_id=Master` | Re-entry request for Airlock A |
| Incoming | `type=enterClearance` | Server signals door is opening |

Multi-robot airlock coordination (another robot driving over tag A) is handled server-side. Robot sends `enterRequest` and waits — no direct robot-to-robot comms needed.

---

## Safety / coordination contract

Don't bypass these — the server is the safety supervisor:

- Motors only run when `isEnabled == true`
- `isEnabled` is set by server heartbeats and cleared on timeout, emergency, or disable
- Any state transitions to `DISABLED` immediately when `isEnabled` goes false
- RGB LED reflects state — useful for visual debugging during runs

---

## Game-logic knobs (likely to change per-run)

- `JUNCTION_SEQUENCE` in `config.h` — **debug/test only**, real runs use the A* planner
- `SEED_COUNT` — max plants per run (5 per competition rules)
- `KP` / `KP_AGGRESSIVE` / `AGGRESSIVE_DURATION_MS` — line-follower tuning
- `BASE_SPEED` / `FORWARD_SPEED` / `TURN_SPEED` — drive speeds

---

## Serial debug commands (`commands.ino`)

Over USB once `setup()` runs:

- `ir` — toggle IR readings printout
- `distance` — toggle ultrasonic printout
- `sensors` — toggle both
- `c` — recalibrate IR (10s window; slide the array over the line)
- `forward [speed]` — drive forward 3s, optional speed
- `<number>` — turn that many degrees via gyro (e.g. `90`, `-45`)
- Anything else → forwarded to the server over WiFi

---

## Open design questions

- **Right-half node arrival detection** (decide before implementing `ARENA_NAV` right-half logic): encoder tick count from last known tag, timed interval at known speed, or IR dip sensor. Encoder-based is most accurate once encoders are wired.
- **Missed RFID reads:** retry once in place, then continue. Don't stall.
- **Obstacle avoidance:** not in scope yet. If another robot blocks a cell, current A* has no obstacle model — adding a `BLOCKED` tag state and re-routing on forward-ultrasonic trigger is the planned extension.
- **Free movement (future):** once grid nav is reliable, diagonal moves on the right half would reduce path length. Swap Manhattan heuristic for Euclidean and allow 8-directional movement.
- **Seed target selection refinement:** greedy nearest-unvisited is simple but may backtrack heavily — consider TSP-lite once enough fertile tags are discovered.
