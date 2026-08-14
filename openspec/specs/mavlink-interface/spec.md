# mavlink-interface Specification

## Purpose
TBD - created by archiving change replace-sbus-with-mavlink. Update Purpose after archive.
## Requirements
### Requirement: MAVLink 2 Serial Transport
The system SHALL communicate with the Pixhawk over a single bidirectional MAVLink 2
connection on a dedicated UART (Pixhawk TELEM2 ↔ ESP32), using the `MavlinkInterface`
class wrapping the MAVLink C library.

#### Scenario: Initialize MAVLink transport
- **WHEN** `MavlinkInterface.begin(rxPin, txPin, uartNum, baud)` is called with valid
  configuration
- **THEN** the UART is configured non-inverted, 8N1, at `MAVLINK_BAUD_RATE` (default 115200)
- **AND** both RX and TX pins are configured
- **AND** the MAVLink parser state is reset and signal-tracking variables are cleared
- **AND** true is returned if initialization succeeds

#### Scenario: Parse inbound bytes into MAVLink messages
- **WHEN** `update()` is called and bytes are available on the UART
- **THEN** each byte is fed to the MAVLink 2 parser (`mavlink_parse_char`)
- **AND** completed messages are dispatched by message id
- **AND** malformed or partially received frames are discarded without blocking the loop
- **AND** the parser tolerates interleaved message types on the link

#### Scenario: Learn autopilot addressing from HEARTBEAT
- **WHEN** a `HEARTBEAT` message is received from the autopilot
- **THEN** the sender's `system_id` and `component_id` are recorded as the command target
- **AND** the heartbeat timestamp is updated for link-liveness tracking

### Requirement: Servo Command Decoding
The system SHALL receive vehicle actuator commands by decoding the MAVLink
`SERVO_OUTPUT_RAW` message and SHALL request it from the autopilot at the design rate.

#### Scenario: Request command stream on link establishment
- **WHEN** the first autopilot `HEARTBEAT` is received after startup
- **THEN** the ESP32 sends `MAV_CMD_SET_MESSAGE_INTERVAL` requesting `SERVO_OUTPUT_RAW`
  at `MAVLINK_SERVO_OUTPUT_RATE_HZ` (default 50 Hz)
- **AND** the request is addressed to the learned autopilot system/component id

#### Scenario: Decode servo output values
- **WHEN** a `SERVO_OUTPUT_RAW` message is received
- **THEN** the `servoN_raw` PWM values (microseconds) are stored as the latest command frame
- **AND** the command timestamp is recorded
- **AND** the total command-frame counter is incremented
- **AND** values are clamped to the valid microsecond range before use

#### Scenario: Consume commands regardless of exact stream rate
- **WHEN** `SERVO_OUTPUT_RAW` messages arrive at a rate other than the requested 50 Hz
- **THEN** the decoder uses the most recent frame
- **AND** command validity is governed solely by the command-staleness timeout
- **AND** no command is fabricated when no frame has been received

### Requirement: Servo Channel to Vehicle Command Mapping
The system SHALL map decoded servo-output channels to typed vehicle commands using a
compile-time `ServoChannelConfig` and the transport-neutral range constants in `Constants.h`,
preserving the function assignments previously used for SBUS.

#### Scenario: Define servo channel assignments
- **WHEN** `ServoChannelConfig` is defined
- **THEN** the following function→servo-output indices are available:
  - STEERING = 1
  - THROTTLE = 2 (combined throttle/brake)
  - TRANSMISSION = 3
  - IGNITION = 4
  - FRONT_LIGHT = 6 (channel 5 is intentionally unused)

#### Scenario: Map steering channel to percentage
- **WHEN** `getSteering()` is called
- **THEN** the steering servo-output value is read and converted using bidirectional mapping
  (center = 0%, min = -100%, max = +100%)
- **AND** the steering deadband is applied around center

#### Scenario: Map throttle and brake from the combined channel
- **WHEN** `getThrottle()` is called
- **THEN** only the upper half (above center µs) of the throttle channel maps to 0–100%
- **AND** values at or below center return 0%
- **WHEN** `getBrake()` is called
- **THEN** only the lower half (below center µs) of the same channel maps to 0–100%
- **AND** values at or above center return 0%

#### Scenario: Map transmission, ignition, and light channels
- **WHEN** `getGear()`, `getIgnitionState()`, or `getFrontLight()` is called
- **THEN** the corresponding channel µs value is mapped to the `Gear` enum, the
  `IgnitionState` enum (OFF/ACC/IGNITION), or a boolean respectively, using the configured
  microsecond ranges/threshold

### Requirement: Command Link-Loss Detection and Fail-Safe
The system SHALL detect loss of the MAVLink command stream and the MAVLink link, and SHALL
report invalid signal so the vehicle controller activates fail-safe.

#### Scenario: Detect command staleness
- **WHEN** no `SERVO_OUTPUT_RAW` has been received for `MAVLINK_CMD_TIMEOUT_MS`
  (default 500 ms)
- **THEN** `isSignalValid()` returns false
- **AND** `getSignalAge()` reports the time since the last command frame

#### Scenario: Detect heartbeat loss
- **WHEN** no autopilot `HEARTBEAT` has been received for `MAVLINK_HEARTBEAT_TIMEOUT_MS`
  (default 3000 ms)
- **THEN** the link is reported as down
- **AND** `isSignalValid()` returns false

#### Scenario: Recover on stream resumption
- **WHEN** valid `SERVO_OUTPUT_RAW` frames resume after a loss
- **THEN** `isSignalValid()` returns true once a fresh frame is within the command timeout
- **AND** normal command decoding resumes

### Requirement: Vehicle State Reporting via Standard MAVLink Messages
The system SHALL report vehicle state back to the MAVLink network using standard MAVLink
messages sourced from the CAN `VehicleData`, on a fixed schedule.

#### Scenario: Emit heartbeat
- **WHEN** the heartbeat interval elapses (1 Hz)
- **THEN** a `HEARTBEAT` is sent with `type = MAV_TYPE_GROUND_ROVER` and the ESP32's own
  system/component id
- **AND** `system_status` reflects fail-safe vs healthy operation

#### Scenario: Report engine telemetry
- **WHEN** the engine-telemetry interval elapses (default 5 Hz)
- **THEN** an `EFI_STATUS` message is sent carrying engine RPM, coolant temperature,
  throttle position, and a health flag (set from CAN validity)
- **AND** vehicle speed and oil temperature are NOT reported (the ECU does not provide
  them; reporting zeros would be misleading)

#### Scenario: Report state changes as status text
- **WHEN** the current gear, ignition state, or fail-safe state changes
- **THEN** a `STATUSTEXT` message describing the new state is sent (rate-limited)

#### Scenario: Report current gear as a live named value
- **WHEN** the engine-telemetry interval elapses
- **THEN** a `NAMED_VALUE_FLOAT` named `GEAR` is sent so the GCS shows it as a live,
  graphable value (Mission Planner Status tab)
- **AND** when settled the value is the gear's physical-sequence position
  `[R, N, H, L] = [-1, 0, 1, 2]` (exact in float)
- **AND** the value reflects the controller's ASSUMED (commanded) gear — the transmission is
  sensorless and time-based, so it represents intent, not a measured position

#### Scenario: Show a shift as the midpoint between gears
- **WHEN** the servo is actively moving between two gears (a gear-change phase is active)
- **THEN** the `GEAR` value is the midpoint of the outgoing and incoming gear values (e.g.
  moving N→H reports `0.5`, moving R→N reports `-0.5`)
- **AND** when settled or dwelling at a gear the value is that gear's integer position
- **AND** a multi-step change (e.g. R→L) therefore renders as an even 0.5 staircase through
  the sequence: `-1.0 → -0.5 → 0.0 → 0.5 → 1.0 → 1.5 → 2.0`

#### Scenario: Suppress engine telemetry when CAN data is invalid
- **WHEN** CAN `VehicleData` is invalid or stale
- **THEN** engine-telemetry messages are either skipped or marked unhealthy
- **AND** `HEARTBEAT` continues to be sent for link liveness

### Requirement: MavlinkInterface Class API
The system SHALL provide a `MavlinkInterface` class exposing the same typed command and
signal-monitoring API previously provided by `SBusInput`, so the vehicle/actuator layer is
unchanged.

#### Scenario: Provide typed command accessors
- **WHEN** the application needs command data
- **THEN** `getSteering()` returns -100..+100, `getThrottle()`/`getBrake()` return 0..100,
  `getGear()` returns the `Gear` enum, `getIgnitionState()` returns OFF/ACC/IGNITION, and
  `getFrontLight()` returns a boolean

#### Scenario: Provide link monitoring
- **WHEN** the application queries link status
- **THEN** `isSignalValid()`, `getSignalAge()`, and `getLinkQuality()` report command
  freshness, age in milliseconds, and link metrics (command rate, frames received, time
  since last heartbeat)

#### Scenario: Update in the main loop
- **WHEN** `update()` is called each loop iteration
- **THEN** inbound bytes are parsed, command/heartbeat timestamps are refreshed, the command
  stream is requested if not yet active, and scheduled outbound telemetry is sent

### Requirement: Input Source Priority Integration
The system SHALL report MAVLink command priority so the input-source arbiter can select it
as the primary control source.

#### Scenario: Report priority status
- **WHEN** the arbiter queries the MAVLink interface
- **THEN** MAVLink is PRIMARY when command data is valid (within the command timeout)
- **AND** MAVLink is INACTIVE when command data is stale or the link is down
- **AND** time since the last valid command frame is provided

### Requirement: Relay Controller for Ignition and Lights
The system SHALL provide a `RelayController` class to manage relay outputs for ignition
states, front light and front-wheel lock control, driven by commands decoded from the MAVLink
interface or the web portal. The relays SHALL be driven through the external relay board's PCA9557
expander at address `0x1F` rather than through direct ESP32 GPIO, with the public API unchanged for
callers. The bus is supplied by the owner at construction: `Wire1` (I2C2), the relay board's home on
header X15.

The relay board's PCA9557 IO routing is NOT 1:1 with the relay numbers (`Relay_1=IO4,
Relay_2=IO5, Relay_3=IO6, Relay_4=IO7, Relay_5=IO3, Relay_6=IO2, Relay_7=IO0, Relay_8=IO1`,
per the `Relay.PrjPcb` netlist), and two functions drive a PAIR of relays that always energize
together. Each function is therefore addressed as a whole-port MASK, not a single bit:

| Function | Mask constant | Value | Relays (IO / relay-board connector) |
|----------|---------------|-------|--------------------------------------|
| Ignition / ECU line | `RELAY_MASK_IGNITION` | `0b00110000` | Relay_1 (IO4, X4) + Relay_2 (IO5, X5) |
| Starter | `RELAY_MASK_STARTER` | `0b01000000` | Relay_3 (IO6, X6) |
| Front light | `RELAY_MASK_FRONT_LIGHT` | `0b10000000` | Relay_4 (IO7, X7) |
| Front-wheel lock | `RELAY_MASK_WHEEL_LOCK` | `0b00001100` | Relay_5 (IO3, X8) + Relay_6 (IO2, X9) |
| Spare | — | `0b00000011` | Relay_7 (IO0, X10) + Relay_8 (IO1, X11) — always off |

The duplicated relays are intentional: both relays of a pair are set and cleared as one mask, so a
pair can never be left half-energized by firmware.

#### Scenario: Initialize relay controller
- **WHEN** `RelayController.begin()` is called with the I2C bus the relay board is wired to
- **THEN** the PCA9557 at `0x1F` SHALL be detected and all eight pins configured as outputs
- **AND** the output port SHALL be written to `0x00` (all relays OFF) and verified by read-back
- **AND** internal state tracking is initialized to OFF / light off / lock off
- **AND** true is returned if initialization succeeds, false if the expander does not answer or the
  all-off write does not verify

#### Scenario: Set ignition state
- **WHEN** `setIgnitionState(OFF)` is called, `RELAY_MASK_IGNITION` (Relay_1 + Relay_2) and
  `RELAY_MASK_STARTER` (Relay_3) are LOW
- **WHEN** `setIgnitionState(ACC)` is called, `RELAY_MASK_IGNITION` is HIGH and `RELAY_MASK_STARTER`
  is LOW
- **WHEN** `setIgnitionState(IGNITION)` is called, `RELAY_MASK_IGNITION` is HIGH and
  `RELAY_MASK_STARTER` is LOW
- **WHEN** `setIgnitionState(CRANKING)` is called, `RELAY_MASK_IGNITION` and `RELAY_MASK_STARTER` are
  both HIGH
- **AND** every bit of a mask moves together, so Relay_1 and Relay_2 are always in the same state
- **AND** internal state is updated to match in each case
- **AND** each change is applied as a single whole-port write from the output shadow byte

#### Scenario: Control front light
- **WHEN** `setFrontLight(true)` is called, `RELAY_MASK_FRONT_LIGHT` (Relay_4, IO7) is set HIGH and
  state is updated
- **WHEN** `setFrontLight(false)` is called, `RELAY_MASK_FRONT_LIGHT` is set LOW and state is updated

#### Scenario: Control front-wheel lock
- **WHEN** `setWheelLock(true)` is called, `RELAY_MASK_WHEEL_LOCK` (Relay_5 + Relay_6) is set HIGH
  and state is updated
- **WHEN** `setWheelLock(false)` is called, `RELAY_MASK_WHEEL_LOCK` is set LOW and state is updated
- **AND** both relays of the pair change together in the same whole-port write

#### Scenario: Spare relays are never energized
- **WHEN** any relay state changes
- **THEN** `Relay_7` (IO0) and `Relay_8` (IO1) SHALL be written LOW in every port write

#### Scenario: Fail-safe all relays off
- **WHEN** `allOff()` is called (fail-safe trigger)
- **THEN** `RELAY_MASK_IGNITION`, `RELAY_MASK_STARTER` and `RELAY_MASK_FRONT_LIGHT` are set LOW
- **AND** internal state is reset to safe defaults (OFF, lights off)
- **AND** the front-wheel lock pair (`RELAY_MASK_WHEEL_LOCK`) is intentionally left in its last state
- **AND** the fail-safe write is verified by read-back like any other relay write

#### Scenario: Relay commands are no-ops when the expander is unavailable
- **WHEN** the expander failed to initialize or the driver is faulted
- **AND** a MAVLink- or web-decoded relay command arrives
- **THEN** the command SHALL be logged and ignored rather than silently assumed applied
- **AND** the reported ignition state SHALL be `OFF`

