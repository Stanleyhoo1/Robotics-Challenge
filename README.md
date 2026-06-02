# Robotics-Challenge

All active firmware lives in `main/`. Other top-level folders (`Saviour`, `WallFollowing`, `line_following`, `motor_test`, `servo_motor`, `wifi`) are earlier standalone sketches kept for reference.

## Repository structure

```
main/
├── main.ino           setup() / loop(); owns showIR, showDistance, showEncoders,
│                      showPitch, showLDR, isEnabled, useStateMachine,
│                      lastForwardDistanceCm; power-button + obstacle gates
├── config.h           tunable constants — pins, speeds, gains, thresholds,
│                      calibration values, airlock coords, line-zone split
├── globals.h          extern declarations for cross-.ino globals (one block per
│                      owning file)
├── types.h            enums (NavState, TagState, Facing, LineState,
│                      DistanceSensor) and structs (GridPos, FertileResult)
├── secrets.h          WiFi credentials, broker host/port, team id (gitignored)
├── commands.ino       handleSerialCommands; debug/test commands (nav, arena,
│                      stop, selftest, hop, enc, state, pos, tag, astar, target,
│                      calib, recalib, ldr, pitch)
├── distance.ino       HC-SR04 wrappers — getDistanceCM, readAndPrintDistance
├── ir_sensor.ino      9-element IR reflectance array, kvstore-backed calibration,
│                      LDR analog read
├── motors.ino         Motoron I2C drive, LSM6 IMU, turnDegrees, moveForward,
│                      2 quadrature encoders (BL + BR) with ISRs + counters,
│                      hop-heading lock, self-calibrating ticks/cm with KV persist
├── navigation.ino     Line-following, junction handling, A* planner, target
│                      selection, state-machine dispatcher (navigationUpdate)
│                      with tick functions, wall-follow, obstacle-avoid, base
│                      exit + return sequences, line-lost recovery
├── rfid.ino           MFRC522 setup; legacy rfidLoop (only runs when state
│                      machine is off)
├── servo.ino          servo sweep (planting actuator)
└── wifi.ino           MiniMessenger pub/sub, onMessage parser, send helpers,
                       heartbeat watchdog, status LED
```

## Required libraries

Install via Arduino IDE → Library Manager:

| Library              | Purpose                            | 
|----------------------|------------------------------------|
| Motoron              | I2C motor controller driver        |
| LSM6                 | 6-DOF IMU (gyro-only used)         |
| MFRC522_I2C          | RFID reader over I2C               |
| MiniMessenger        | WiFi pub/sub messaging             |
| Servo                | Servo driver                       |
| kvstore_global_api   | Persistent KV storage (mbed)       |

`kvstore_global_api` is part of the Arduino mbed core that's installed with the Giga board package. Installing the board (see below) brings it in automatically. `Servo` ships with the IDE. The other four need explicit installs.

## Setup

### 1. Install the board package
Arduino IDE → Boards Manager → search "Giga" → install **Arduino Mbed OS Giga Boards**.

### 2. Patch the Motoron library for mbed I2C

The stock Motoron library uses `TwoWire*` for its I2C bus type, which is incompatible with the Arduino Giga's mbed-based I2C stack. After installing the library, replace the `MotoronI2C` class in:

```
Documents/Arduino/libraries/Motoron/src/Motoron.h  (lines ~1923–2010)
```

with the mbed-compatible version below:

```cpp
class MotoronI2C : public MotoronBase
{
public:
  MotoronI2C(uint8_t address = 16, arduino::MbedI2C* wire = &Wire) : bus(wire), address(address) {}

  void setBus(arduino::MbedI2C * bus) { this->bus = bus; }
  arduino::MbedI2C * getBus() { return this->bus; }
  void setAddress(uint8_t address) { this->address = address; }
  uint8_t getAddress() { return address; }

private:
  arduino::MbedI2C * bus;
  uint8_t address;

  void sendCommandCore(uint8_t length, const uint8_t * cmd, bool sendCrc) override
  {
    bus->beginTransmission(address);
    for (uint8_t i = 0; i < length; i++) { bus->write(cmd[i]); }
    if (sendCrc) { bus->write(calculateCrc(length, cmd)); }
    lastError = bus->endTransmission();
  }

  void flushTransmission() { }

  void readResponse(uint8_t length, uint8_t * response) override
  {
    bool crcEnabled = protocolOptions & (1 << MOTORON_PROTOCOL_OPTION_CRC_FOR_RESPONSES);
    uint8_t byteCount = bus->requestFrom(address, (uint8_t)(length + crcEnabled));
    if (byteCount != length + crcEnabled)
    {
      memset(response, 0, length);
      lastError = 50;
      return;
    }
    lastError = 0;
    uint8_t * ptr = response;
    for (uint8_t i = 0; i < length; i++) { *ptr = bus->read(); ptr++; }
    if (crcEnabled && bus->read() != calculateCrc(length, response))
    {
      lastError = 51;
      return;
    }
  }
};
```

This patch is required to pass `Wire1` (a `arduino::MbedI2C` object) to the Motoron constructor. Without it, the firmware will not compile on the Giga.

### 3. Create `main/secrets.h`
This file is gitignored and must be created locally. Template:

```cpp
#pragma once
#define WIFI_SSID      "your-ssid"
#define WIFI_PASSWORD  "your-password"
#define BROKER_HOST    "broker.local"
#define BROKER_PORT    1883
#define GROUP_ID       "your-team-id"
```

### 4. First-boot calibration
Two things calibrate at boot:

- **Gyro bias** — robot must be motionless for ~2 seconds during boot. The Serial monitor will print "Calibrating gyro, keep still..." followed by "Gyro calibrated." Don't bump the chassis during this window.
- **IR sensor min/max** — only needed once. Hold the robot over the line, open the Serial monitor, send the `c` command. You have 10 seconds to slide the array fully across the line. Values are persisted in flash via `kvstore_global_api`, so subsequent boots reuse them.
- **Encoder ticks/cm** — self-calibrates at run time from confirmed straight tag-to-tag hops once the bot is in the arena. Until `CALIB_MIN_SAMPLES` (4) accepted samples land, `TICKS_PER_CM_FALLBACK` is used. Calibration is persisted in KV — use the `recalib` serial command to wipe and re-acquire.

### 5. I2C bus order
`setup()` calls `Wire.begin()` then `Wire1.begin()`. `Wire` carries the RFID reader (default I2C pins, address 0x28); `Wire1` carries the Motoron motor controller (address 16) and the LSM6 IMU. Both buses must be initialized before any device on them is touched.

## Upload

1. Connect the Giga over USB.
2. Arduino IDE → Select Board → **Arduino Giga R1 WiFi**.
3. Check the port to make sure it is correct (e.g. `COM5` on Windows, `/dev/ttyACM0` on Linux, `/dev/cu.usbmodem...` on macOS).
4. Open `main/main.ino` — the IDE will load the whole multi-file sketch from that directory.
5. Click Upload.

The first compile pulls in all the libraries above; expect ~30 s. Subsequent builds are fast.

## Starting a run

1. Power on. Boot prints land on Serial at 115200 baud.
2. Wait for `Gyro calibrated.` and `WiFi connecting...`.
3. The LED blinks red while disabled (no heartbeat / button off).
4. Press the **power button** (pin 17) — the LED goes solid red and the state machine starts executing whatever state it was last in (default `NAV_BASE_TO_FIRST_JUNCTION`). The state is preserved across disable/re-enable cycles, so if the bot gets disabled mid-run it resumes from where it stopped.
5. Holding either **revive button** (52 or 53) overrides the LED to green for the duration. Useful as a visual sanity check that the green channel works.

## Serial commands

Open the Serial monitor at **115200 baud**, line ending = Newline. Commands are case-insensitive. These were used for debugging and isolating features to test.

### Movement
| Command           | Effect                                                 |
|-------------------|--------------------------------------------------------|
| `<number>`        | Turn that many degrees via gyro (e.g. `90`, `-45`)     |
| `forward`         | Drive forward 3 s at `FORWARD_SPEED`                   |
| `forward <speed>` | Drive forward 3 s at the given Motoron speed           |
| `hop`             | One heading-locked hop until `nearNextNode()` (≤10 s)  |
| `stop`            | Zero motors + `isEnabled = false`                      |

### Sensors / calibration
| Command    | Effect                                              |
|------------|-----------------------------------------------------|
| `ir`       | Toggle IR readings printout                         |
| `distance` | Toggle forward-ultrasonic readings printout         |
| `sensors`  | Toggle both `ir` and `distance` together            |
| `enc`      | Toggle 5 Hz encoder counter printout                |
| `encreset` | Zero both encoder counters                          |
| `ldr`      | Toggle 5 Hz LDR (light sensor) printout             |
| `pitch`    | Toggle 5 Hz IMU pitch printout (ramp detection)     |
| `c`        | Recalibrate IR sensors (10 s window)                |
| `calib`    | Print current encoder calibration state             |
| `recalib`  | Wipe persisted encoder calibration from KV          |

### State machine
| Command                       | Effect                                                          |
|-------------------------------|-----------------------------------------------------------------|
| `nav`                         | Toggle state machine (sets `NAV_BASE_TO_FIRST_JUNCTION` on)     |
| `arena`                       | Jump straight to `NAV_ARENA_NAV` (skip base exit, bench tests)  |
| `state`                       | Dump `navState`, `robotPos`, `robotFacing`, seedsRemaining etc. |
| `pos <r> <c> <n\|e\|s\|w>`    | Force `robotPos` and `robotFacing`                              |
| `tag <r> <c> <u\|f\|i\|p\|b>` | Set `tagMap[r][c]` cell state (b = BLOCKED for A\* obstacle)    |
| `target`                      | Print `selectNextTarget` result                                 |
| `astar <r1> <c1> <r2> <c2>`   | Print A\* first step for the given pair                         |
| `selftest`                    | Run hardcoded assertions on the pure-logic code                 |

### Server passthrough
Anything not matched above is forwarded verbatim to the server via WiFi (to test robot -> server requests and responses).

## Status LED

The RGB status LED on pins 48 (R) and 49 (G) reflects a small priority stack:

| Color              | Condition                                                                                                     |
|--------------------|---------------------------------------------------------------------------------------------------------------|
| **Green**          | Either revive button (52/53) held — overrides everything else |
| **Solid red**      | `isEnabled` true                                              |
| **Blinking red**   | `isEnabled` false; awaiting button press / server heartbeat   |

## Further reading

- `docs/software_overview.md` — block diagram of the firmware.
- `docs/flowcharts.md` — flowcharts for line-following, RFID/planting, kill-switch, base exit, line-lost recovery, LED state.
