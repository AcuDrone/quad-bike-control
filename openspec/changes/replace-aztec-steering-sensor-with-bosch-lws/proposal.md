# Change: Bosch LWS CAN steering-angle sensor replaces the dual AS5600 on the aztec buggy

## Why
The dual-AS5600 design (`add-aztec-dual-as5600-steering`, firmware 1.2.0) needs a second sensor on a
geared-down shaft, turn tracking, turn recovery, a mismatch latch and a coordinated recovery of the
shared I2C2 bus. All of that exists only because an AS5600 is absolute within one turn.

A Bosch LWS steering-wheel-angle sensor 0 265 005 486 (PSA 9650236180) has now been characterised
on the bench (2026-10-24, `~/Documents/SourceCode/Arduino/CAN-TEST/README.md`). It is absolute
over several turns, has 0.1° resolution, sends its angle by itself over CAN every 10 ms after 12 V
power-up, and stores its own zero. Its working range was measured at about ±600°. The buggy's
real lock-to-lock travel is less than that: the "±1.7 turns" figure in the dual-AS5600 change was
an early estimate and is wrong. So the sensor can sit **directly on the steering shaft**, with no
gearing, and one sensor gives the absolute position that two AS5600s gave before.

## What Changes
- **New steering feedback: the Bosch LWS over CAN.** It is on the existing MCP2515 bus (500 kbit/s),
  shared with the engine ECU (OBD-II at `0x7DF`/`0x7E8`). There is one bus and no new GPIO.
  - The sensor sends frame `0x305`, DLC 6, at 100 Hz. It carries an int16 angle in 0.1° units,
    rotation speed, status bits, a nibble checksum and a rolling counter. The decoder reuses the
    bench-verified logic of `CAN-TEST/src/lws305.h`.
  - The sensor's sign convention is positive = left (counter-clockwise). The firmware converts it
    with a **fixed negation**, `pos = −angle`, so that positive = right as everywhere else in the
    firmware. The old derived `invert` flag is removed.
- **Fault model**:
  - These are sensor faults: no valid frame for `STEER_LWS_TIMEOUT_MS` (100 ms), which covers a
    bad checksum or a frozen counter (such frames are rejected); angle `0x7FFF`; status OK bit
    clear; a nonzero fault code; the `calibrated` bit clear; the CAN controller down; or a zero-set
    sequence in progress.
  - The out-of-range **latched** fault (fault code 9, cleared only by a 12 V power cycle) gets its
    own token, so the operator knows to restart the vehicle. The firmware does nothing else about it.
  - Lost frames are harmless, because each frame is absolute. They are only counted.
- **CAN controller**:
  - RXB1 gets its own acceptance path for `0x305` (mask `0x7FF`, filters 2–5 = `0x305`). RXB0 keeps
    the OBD window `0x7E8–0x7EF`. This way 100 Hz LWS frames and OBD replies do not evict each other.
  - Every receive path (IDLE drain, OBD response wait, ECU probe) hands `0x305` frames to the
    steering decoder, in any CAN state.
  - A new, internal zero-set sequence pauses OBD polling, sends exactly `0x7C0 [05 00]`, waits
    ~100 ms, sends exactly `0x7C0 [53 30]`, and resumes polling. No other `0x7C0` payload can ever
    be sent. In particular, `03 00` would move the sensor to ID `0x000`.
- **Calibration, BREAKING**:
  - The steering centre is the **sensor's own stored zero** (angle 0.0). There is no centre
    capture and no firmware centre offset.
  - NVS keeps only the left and right limits, in 0.1°, firmware sign convention, under new keys
    and `cal_ver` = 3. Every dual-sensor key and every older single-sensor key is erased one by one
    (`steer_sca_base` is kept). **Every vehicle needs recalibrating.**
  - Limits are captured with the existing Left/Right buttons. Each one is saved immediately. They
    must be on the correct side of zero and at least `STEER_CAL_MIN_SPAN` away from it. The ±550°
    safe range does **not** restrict them: a limit beyond it is accepted and saved, and the firmware
    only warns (`steer_range_warning` in telemetry, a note next to that limit in the UI) that the
    sensor's latched-fault threshold (~600°) is near.
- **"Set sensor zero" web button**: runs the zero-set sequence with **no user-facing
  preconditions** (no ignition, RPM or standstill guard). The firmware reports success only after
  the sensor reports `calibrated` = 1 with an angle of about 0. Once the reset frame has been sent,
  the stored limits are erased, because they referred to the old zero.
- **Sensorless manual steering**: jog and nudge work whenever the VESC link is up, whatever the
  sensor state. While the sensor is faulted, absent or invalid they run open loop, with no stall
  backstop. Only the existing 500 ms jog-refresh timeout, the nudge pulse length and the VESC's own
  current limit and fault stop apply.
  - Percent position commands still need a healthy sensor and both limits. These are the MAVLink
    `SERVO_OUTPUT_RAW` path, the web slider and test buttons, boot centring and the failsafe centre.
  - This makes the sensorless centring flow possible: jog to the mechanical centre → restart the
    vehicle (power-cycles the sensor) → press "Set sensor zero" → capture Left and Right.
- **Range guard** (only with a healthy sensor, closed-loop moves only): a percent-driven position
  move (MAVLink `SERVO_OUTPUT_RAW`, web slider and test buttons, boot centring, failsafe centre)
  has its target clamped to ±`STEER_LWS_SAFE_RANGE` (±550°) after the limit clamp, and is stopped
  if it would drive further outward at or beyond ±550°. This keeps automatic steering out of the
  sensor's latched fault. Jog, nudge and the Left/Right captures are **not** limited by the guard;
  manual driving past 550° is the operator's decision.
- **Units**: every steering position, limit, tolerance and gain is in 0.1° (int32), not AS5600
  counts. The tolerances and the gain are re-expressed and marked bench-tune (see `design.md`).
- **Removed**:
  - Both AS5600 sensors and `include/AS5600Sensor.h` / `src/AS5600Sensor.cpp`.
  - Fine/coarse sampling, turn tracking, turn recovery, the seam and ratio, the mismatch latch, and
    the session-slot calibration.
  - The coordinated `Wire1` recovery in `main.cpp` and its constants.
  - The fine/coarse/turn/ratio/residual telemetry and the centre capture.
  - I2C1 (GPIO1/2, header X14) becomes free, and `Wire` is no longer started. I2C2 keeps the
    on-board expanders and the relay board only.
- **Telemetry and UI**:
  - New fields: angle in degrees, signed rotation rate, the LWS status bits and fault code, a new
    fault-token set, frame rate, checksum and counter error counts, and zero-set progress.
  - The dual-sensor fields are removed.
  - The calibration card shows Left/Right capture plus "Set sensor zero". Jog and nudge are never
    disabled because of the sensor.
  - MAVLink `STEER_POS` keeps its meaning and gate: percent of calibrated travel, valid only while
    the sensor is healthy and both limits are set.
- **Docs and version**: the pinout document, the `Constants.h` comments, the `MAVLINK_SETUP.md`
  fault table, firmware version 1.3.0, and a `Changelog.h` entry (uk + en).
- **Web portal address**: the WiFi AP IP moves from 192.168.4.1 to 192.168.10.1 (bundled with
  1.3.0; `WIFI_AP_IP` / `WIFI_AP_GATEWAY` in `Constants.h`, `WEB_PORTAL_SETUP.md`).

## Out of scope
- A very slow, pulsed jog mode for fine positioning without a sensor. It is deferred; the existing
  nudge pulse is the fine control for now.
- Switching the sensor's 12 V from a relay or MOSFET to clear the latched fault automatically. The
  sensor is fed from the ignition key, so restarting the vehicle is the clearing action.
- Any other LWS configuration (changing the output ID, PSA/UDS calibration).

## Relationship to other pending changes (archive order matters)
- This change **supersedes the steering-sensor parts of `add-aztec-dual-as5600-steering`**: the
  dual-sensor position, the dual-sensor calibration, the dual-sensor telemetry, the coarse sensor on
  I2C2 and the coordinated I2C2 recovery. It does not edit that change. That change is already
  implemented (firmware 1.2.0, commit `de4ab55`), and its requirements exist **only as deltas in
  that change, not in `openspec/specs/`**.
- **Recommended archive order**:
  1. `add-aztec-dual-as5600-steering`
  2. `update-aztec-vesc-uart0`
  3. this change

  Reasons:
  - This change **REMOVES** `Dual-Sensor Multi-Turn Steering Position` and `Dual-Sensor Steering
    Calibration` (vehicle-systems), `Steering Position Sensor Telemetry` (web-telemetry) and
    `Coordinated I2C2 Bus Recovery` (gpio-expander). Those requirements exist only after the
    dual-AS5600 change is archived, so archiving this change first would fail.
  - `Steering Control System` (vehicle-systems) and `Dual I2C Bus Topology and Address Map`
    (gpio-expander) are full replacements and supersede both the current spec text and the
    dual-AS5600 version.
  - `Hardware Configuration` (vehicle-actuators) is modified by three pending changes. Following
    the precedent of `update-aztec-vesc-uart0`, this change's version is **based on the
    `update-aztec-vesc-uart0` delta**, which already contains the dual-AS5600 edits plus the VESC
    UART0 pins. It must therefore be archived after both, or the VESC pin change would be reverted.
- The other unarchived aztec changes (`update-aztec-gear-code-inputs`,
  `update-aztec-relay-assignment`, `update-throttle-inverted-calibration`) touch no requirement this
  change modifies, so their order relative to this change does not matter.

## Impact
- Affected specs:
  - `vehicle-systems`:
    - MODIFIED: Steering Control System
    - ADDED: Steering Limit Calibration Around the Sensor Zero
    - REMOVED: Dual-Sensor Multi-Turn Steering Position, Dual-Sensor Steering Calibration
  - `vehicle-actuators`:
    - MODIFIED: Hardware Configuration, Steering Motor Driver Abstraction, Steering Re-command
      Guard and Stall Latch, VESC Fault Monitoring and Communication Failsafe
    - ADDED: Bosch LWS Steering Angle Sensor, Sensorless Manual Steering
  - `can-controller`:
    - MODIFIED: CAN Communication Error Handling (drain and dispatch)
    - ADDED: Shared-Bus Steering Angle Frame Reception, Steering Sensor Zero-Set Sequence
  - `gpio-expander`:
    - MODIFIED: Dual I2C Bus Topology and Address Map
    - REMOVED: Coordinated I2C2 Bus Recovery
  - `web-control`:
    - ADDED: Steering Calibration and Sensor Zero via Web Interface
  - `web-telemetry`:
    - REMOVED: Steering Position Sensor Telemetry
    - ADDED: Steering Angle Sensor Telemetry
  - `mavlink-interface`:
    - MODIFIED: Steering VESC Telemetry via NAMED_VALUE_FLOAT. Only the `STEER_POS` wording and the
      gate's sensor definition change; the meaning is unchanged.
  - `web-server`:
    - MODIFIED: WiFi Access Point Operation (AP IP 192.168.10.1)
- Affected code:
  - New: `include/Lws305.h` (Arduino-free decoder), `include/LwsSensor.h` / `src/LwsSensor.cpp`
    (frame validation, health, counters)
  - `include/SteeringController.h`, `src/SteeringController.cpp`
  - `include/CANController.h`, `src/CANController.cpp`
  - `include/Constants.h`
  - `src/main.cpp`: no `Wire` start, no I2C2 recovery, CAN-to-steering wiring, boot centring
  - `include/VehicleController.h`, `src/VehicleController.cpp`: commands
  - `src/TelemetryManager.cpp`, `include/WebPortal.h`, `src/WebPortal.cpp`
  - `data/index.html`
  - `include/MavlinkInterface.h` (comment only)
  - `include/Changelog.h`
  - `MAVLINK_SETUP.md`, `GPIO_PINOUT_CUSTOM_BOARD_S3.md`
  - `platformio.ini`: a `native` test environment
  - `test/test_lws305/`
- Deleted code: `include/AS5600Sensor.h`, `src/AS5600Sensor.cpp`.
- Hardware:
  - The LWS is mounted directly on the steering shaft.
  - CAN-H/CAN-L go to the existing MCP2515 bus, next to the ECU. Check the termination: exactly
    two 120 Ω terminators on the bus.
  - 12 V comes from the ignition key (switched), with ground in common with the board.
  - Both AS5600s and the geared-down shaft are removed. X14 is left empty.
