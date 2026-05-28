# Robotics-Challenge

Firmware for a UCL robotics competition robot. The robot starts in an underground base, exits via an airlock tunnel into a 9×9 arena, plants seeds at fertile RFID-tagged holes, then returns to base.

All active firmware lives in `main/`. Other top-level folders (`Saviour`, `WallFollowing`, `line_following`, `motor_test`, `servo_motor`, `wifi`) are earlier standalone sketches kept for reference.

See `CLAUDE.md` for the deep design notes — competition layout, state machine, server protocol, calibration approach.

## Repository structure

```
main/
├── main.ino           setup() / loop(); owns showIR, showDistance, showEncoders,
│                      isEnabled, useStateMachine, lastForwardDistanceCm
├── config.h           tunable constants — pins, speeds, gains, thresholds,
│                      calibration values, command keys
├── globals.h          extern declarations for cross-.ino globals (one block per
│                      owning file)
├── types.h            enums (NavState, TagState, Facing, LineState,
│                      DistanceSensor) and structs (GridPos, FertileResult)
├── secrets.h          WiFi credentials, broker host/port, team id (gitignored)
├── commands.ino       handleSerialCommands; debug/test commands (selftest, hop,
│                      enc, state, pos, tag, astar, target, calib)
├── distance.ino       HC-SR04 wrappers — getDistanceCM, readAndPrintDistance
├── ir_sensor.ino      9-element IR reflectance array, kvstore-backed calibration
├── motors.ino         Motoron I2C drive, LSM6 IMU, turnDegrees, moveForward,
│                      encoder ISRs + counters + averaging, heading-lock,
│                      self-calibrating ticks/cm
├── navigation.ino     Line-following, junction handling, A* planner, target
│                      selection, state-machine dispatcher (navigationUpdate)
│                      with tick functions, wall-follow, obstacle-avoid
├── rfid.ino           MFRC522 setup; legacy rfidLoop (only runs when state
│                      machine is off)
├── servo.ino          servo sweep (planting actuator)
└── wifi.ino           MiniMessenger pub/sub, onMessage parser, send helpers,
                       heartbeat watchdog, RGB status LED
```

## Required libraries

Install via Arduino IDE → Tools → Manage Libraries, or manually drop them into your Arduino libraries folder:

| Library              | Purpose                            | Source                          |
|----------------------|------------------------------------|---------------------------------|
| Motoron              | I2C motor controller driver        | Library Manager ("Motoron")     |
| LSM6                 | 6-DOF IMU (gyro-only used)         | Library Manager ("LSM6")        |
| MFRC522_I2C          | RFID reader over I2C               | Manual install — repo: arozcan/MFRC522-I2C-Library |
| MiniMessenger        | WiFi pub/sub messaging             | Manual install (project-supplied) |
| Servo                | Servo driver                       | Bundled with Arduino IDE        |
| kvstore_global_api   | Persistent KV storage (mbed)       | Bundled with Arduino mbed core  |

`kvstore_global_api` is part of the Arduino mbed core that's installed with the Giga board package — installing the board (see below) brings it in automatically. `Servo` ships with the IDE. The other four need explicit installs.

## Setup

### 1. Install the board package
Arduino IDE → Tools → Board → Boards Manager → search "Giga" → install **Arduino Mbed OS Giga Boards**.

### 2. Create `main/secrets.h`
This file is gitignored and must be created locally. Template:

```cpp
#pragma once
#define WIFI_SSID      "your-ssid"
#define WIFI_PASSWORD  "your-password"
#define BROKER_HOST    "broker.local"
#define BROKER_PORT    1883
#define GROUP_ID       "your-team-id"
```

### 3. First-boot calibration
Two things calibrate at boot:

- **Gyro bias** — robot must be motionless for ~2 seconds during boot. The Serial monitor will print "Calibrating gyro, keep still..." followed by "Gyro calibrated." Don't bump the chassis during this window.
- **IR sensor min/max** — only needed once. Hold the robot over the line, open the Serial monitor, send the `c` command. You have 10 seconds to slide the array fully across the line. Values are persisted in flash via `kvstore_global_api`, so subsequent boots reuse them.

### 4. I2C bus order
`setup()` calls `Wire.begin()` then `Wire1.begin()`. `Wire` carries the RFID reader (default I2C pins, address 0x28); `Wire1` carries the Motoron motor controller (address 16) and the LSM6 IMU. Both buses must be initialized before any device on them is touched.

## Upload

1. Connect the Giga over USB.
2. Arduino IDE → Tools → Board → **Arduino Giga R1 WiFi**.
3. Arduino IDE → Tools → Port → select the device (e.g. `COM5` on Windows, `/dev/ttyACM0` on Linux, `/dev/cu.usbmodem...` on macOS).
4. Open `main/main.ino` — the IDE will load the whole multi-file sketch from that directory.
5. Click Upload.

The first compile pulls in all the libraries above; expect ~30 s. Subsequent builds are fast.

## Serial commands

Open the Serial monitor at **115200 baud**, line ending = Newline. Commands are case-insensitive.

### Movement
| Command           | Effect                                                 |
|-------------------|--------------------------------------------------------|
| `<number>`        | Turn that many degrees via gyro (e.g. `90`, `-45`)     |
| `forward`         | Drive forward 3 s at `FORWARD_SPEED`                   |
| `forward <speed>` | Drive forward 3 s at the given Motoron speed           |
| `hop`             | One heading-locked hop until `nearNextNode()` (≤10 s)  |

### Sensors / calibration
| Command    | Effect                                              |
|------------|-----------------------------------------------------|
| `ir`       | Toggle IR readings printout                         |
| `distance` | Toggle ultrasonic readings printout                 |
| `sensors`  | Toggle both `ir` and `distance` together            |
| `enc`      | Toggle 5 Hz encoder counter printout                |
| `encreset` | Zero all encoder counters                           |
| `c`        | Recalibrate IR sensors (10 s window)                |
| `calib`    | Print current encoder calibration state             |

### State machine
| Command                       | Effect                                          |
|-------------------------------|-------------------------------------------------|
| `nav`                         | Toggle state machine (sets `NAV_ARENA_NAV` on)  |
| `state`                       | Dump `navState`, `robotPos`, `robotFacing` etc. |
| `pos <r> <c> <n\|e\|s\|w>`    | Force `robotPos` and `robotFacing`              |
| `tag <r> <c> <u\|f\|i\|p>`    | Set `tagMap[r][c]` cell state                   |
| `target`                      | Print `selectNextTarget` result                 |
| `astar <r1> <c1> <r2> <c2>`   | Print A* first step for the given pair          |
| `selftest`                    | Run hardcoded assertions on the pure-logic code |

### Server passthrough
Anything not matched above is forwarded verbatim to the server via WiFi — useful for debugging the broker side.

## Status LED

The on-board RGB LED reflects the safety-gate state:

- **Solid red** — `isEnabled` true, motors live
- **Blinking red** — `isEnabled` false; awaiting server re-enable

## Further reading

- `CLAUDE.md` — full project context: competition layout, state-machine table, server protocol, calibration plan, safety contract.
- `docs/software_overview.md` — block diagram of the firmware.
- `docs/flowcharts.md` — flowcharts for line-following, RFID/planting, kill-switch, junction handling.
