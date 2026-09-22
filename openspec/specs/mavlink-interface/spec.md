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
The system SHALL report vehicle state back to the MAVLink network using standard MAVLink messages, on
a fixed schedule. Engine data is sourced from the CAN `VehicleData`; vehicle ground speed is sourced
from the hall-effect speed sensor and reported via `VFR_HUD`. Additional ECU sensor values SHALL be
carried only in `EFI_STATUS` fields that are semantically appropriate and not already assigned to
another value. Where a value can be either measured by the ECU or commanded by the controller, the
reported value SHALL be unambiguous as to which of the two it is.

#### Scenario: Emit heartbeat
- **WHEN** the heartbeat interval elapses (1 Hz)
- **THEN** a `HEARTBEAT` is sent with `type = MAV_TYPE_GROUND_ROVER` and the ESP32's own
  system/component id
- **AND** `system_status` reflects fail-safe vs healthy operation

#### Scenario: Report engine telemetry
- **WHEN** the engine-telemetry interval elapses (default 5 Hz)
- **THEN** an `EFI_STATUS` message is sent carrying engine RPM, coolant temperature,
  throttle position, and a health flag (set from CAN validity)
- **AND** oil temperature is NOT reported (the ECU does not provide it; reporting zeros would be
  misleading)
- **AND** vehicle speed is NOT carried in `EFI_STATUS`; it is reported separately via `VFR_HUD` from
  the hall sensor

#### Scenario: Report intake air temperature and module voltage in EFI_STATUS
- **WHEN** the engine-telemetry interval elapses
- **AND** CAN `VehicleData` is valid
- **THEN** the `EFI_STATUS` message SHALL carry the intake air temperature (OBD-II PID `0x0F`) in the
  `intake_manifold_temperature` field, in °C
- **AND** SHALL carry the control module supply voltage (OBD-II PID `0x42`) in the
  `ignition_voltage` field, in volts
- **AND** both fields were previously transmitted as unused zeros, so no existing value is displaced

#### Scenario: Report measured throttle position in EFI_STATUS
- **WHEN** the engine-telemetry interval elapses
- **AND** CAN `VehicleData` is valid
- **THEN** the `EFI_STATUS` message SHALL carry the ECU-measured throttle position (OBD-II PID
  `0x11`) in the `throttle_out` field, as a percentage
- **AND** the field was previously transmitted as an unused zero, so no existing value is displaced
- **AND** the value SHALL be the measured position only — a commanded throttle value SHALL NOT be
  substituted into `EFI_STATUS`, which reports ECU measurements

#### Scenario: Report throttle percent in VFR_HUD
- **WHEN** the engine-telemetry interval elapses
- **AND** CAN `VehicleData` is valid
- **THEN** the `VFR_HUD` message SHALL carry the ECU-measured throttle position in its `throttle`
  field, as an integer percentage in the range 0-100
- **AND** the GCS SHALL be able to display it on the standard HUD throttle indicator without any
  ground-station plugin change

#### Scenario: Fall back to commanded throttle in VFR_HUD when CAN data is invalid
- **WHEN** the engine-telemetry interval elapses
- **AND** CAN `VehicleData` is invalid or stale, so no measured throttle position is available
- **THEN** `VFR_HUD.throttle` SHALL carry the commanded throttle percentage — the value the throttle
  servo is actually being driven with — rather than 0, because the field is an unsigned integer
  percentage with no `NaN` or "unknown" encoding and a HUD reading 0 % while throttle is applied is
  misleading
- **AND** the commanded value SHALL be the arbitrated output actually applied to the servo
  (autopilot command, web command, gear-change boost override, speed-limit cap, or fail-safe idle),
  not the raw command of any single input source
- **AND** `EFI_STATUS.throttle_out` SHALL read `NaN` in the same reporting tick, so a consumer can
  tell unambiguously that the `VFR_HUD` value is commanded rather than measured
- **AND** `VFR_HUD` SHALL NOT be suppressed for that tick, since its ground-speed field is governed
  by the hall sensor's own validity, independent of CAN validity

#### Scenario: Preserve existing EFI_STATUS field assignments
- **WHEN** additional ECU values are mapped into `EFI_STATUS`
- **THEN** the `engine_load` field SHALL continue to carry the encoded GEAR value and SHALL NOT be
  reused for the ECU's calculated engine load
- **AND** the `pt_compensation` field SHALL continue to carry the digital-output bitmask
- **AND** any ECU value without a free, semantically appropriate field SHALL be omitted from MAVLink
  rather than mapped onto a mismatched field
- **AND** a value already carried in one `EFI_STATUS` field SHALL NOT be duplicated into a second
  field of the same message (the measured throttle position is carried by `throttle_out` only;
  `throttle_position` remains unused)

#### Scenario: Report vehicle speed via VFR_HUD
- **WHEN** the engine-telemetry interval elapses
- **AND** the hall-effect speed sensor reading is valid
- **THEN** a `VFR_HUD` message SHALL be sent with `groundspeed` set to the sensor speed converted to
  metres per second
- **AND** the GCS SHALL be able to display and graph it as ground speed for the ground rover
- **WHEN** the hall sensor reading is invalid or stale
- **THEN** `groundspeed` SHALL be reported as 0 (or the `VFR_HUD` message suppressed for that tick) so
  a stale reading is not presented as genuine motion

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
- **AND** the `intake_manifold_temperature` and `ignition_voltage` fields SHALL be reported as `NaN`
  (matching the existing `rpm` / `cylinder_head_temperature` convention) so a ground station shows
  "no data" rather than a misleading 0 °C / 0 V
- **AND** the `throttle_out` field SHALL likewise be reported as `NaN`, while `VFR_HUD.throttle`
  falls back to the commanded throttle percentage
- **AND** `VFR_HUD` ground-speed reporting is governed by the hall sensor's own validity, independent
  of CAN validity

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

### Requirement: Autopilot Maximum-Speed Parameter Subscription
The system SHALL subscribe to the autopilot's `SPEED_MAX` parameter over MAVLink, which is the
ONLY source of the vehicle's maximum-speed limit. The subscription SHALL be read-only: the system
SHALL NOT send `PARAM_SET` or otherwise write any autopilot parameter. A value that cannot be
trusted SHALL be rejected rather than stored, and an absent, zero, silent or stale value SHALL be
reported as unavailable so the vehicle layer stops limiting altogether.

#### Scenario: Poll the parameter periodically
- **WHEN** the autopilot's system and component ids have been learned from its `HEARTBEAT`
- **AND** the link is up
- **THEN** the system SHALL send a `PARAM_REQUEST_READ` for `SPEED_MAX` with a parameter index of
  `-1` (look up by name), addressed to the learned autopilot
- **AND** the first request SHALL be sent `MAVLINK_PARAM_FIRST_DELAY_MS` after the autopilot was
  learned, so the request does not race the autopilot's own boot
- **AND** subsequent requests SHALL be sent every `MAVLINK_PARAM_POLL_MS`
- **AND** polling SHALL NOT stop once a value has been received, because the poll — not any
  broadcast — is what detects a change made from the ground station

#### Scenario: Accept an unsolicited parameter broadcast
- **WHEN** a `PARAM_VALUE` message arrives that was not requested by this system
- **AND** it carries the parameter id `SPEED_MAX`
- **THEN** it SHALL be accepted on the same terms as a polled reply
- **AND** this SHALL be treated as an optimisation only, because whether the autopilot broadcasts
  after a `PARAM_SET` is firmware-version and routing dependent

#### Scenario: Reject a parameter from any sender other than the autopilot
- **WHEN** a `PARAM_VALUE` for `SPEED_MAX` arrives
- **AND** its sender system id or component id does not match the learned autopilot
- **THEN** the value SHALL be discarded without being stored
- **AND** the stored value and its timestamp SHALL be left unchanged, so that a ground station or
  any other component sharing the link cannot move the vehicle's speed ceiling

#### Scenario: Compare the parameter id safely
- **WHEN** a `PARAM_VALUE` is decoded
- **THEN** the 16-byte `param_id` field SHALL be copied into a 17-byte buffer and terminated
  before any string comparison
- **AND** the field SHALL NOT be compared in place, because MAVLink does not NUL-terminate a
  `param_id` that fills all 16 bytes

#### Scenario: Reject a value that is not a usable speed
- **WHEN** a `PARAM_VALUE` for `SPEED_MAX` arrives from the autopilot
- **AND** its value is not-a-number, infinite, negative, or greater than
  `MAVLINK_PARAM_SPEED_MAX_MS`
- **THEN** the value SHALL be rejected and SHALL NOT be stored
- **AND** the rejection SHALL be logged
- **AND** the previously stored value and its timestamp SHALL be left unchanged, so one corrupt
  frame cannot invalidate a good subscription

#### Scenario: Store and expose an accepted value
- **WHEN** a `PARAM_VALUE` for `SPEED_MAX` passes every check
- **THEN** the value SHALL be stored in metres per second together with the time of receipt
- **AND** the system SHALL expose it in metres per second — the firmware's internal speed unit —
  together with the age of the reading, and SHALL NOT expose a km/h accessor
- **AND** a change SHALL be logged only when the new value differs from the previous one by more
  than `MAVLINK_PARAM_EPSILON_MS`, so a 5-second poll of an unchanged parameter does not fill the
  console
- **AND** the log line MAY show km/h in parentheses alongside the m/s value, because a log is a
  presentation surface

#### Scenario: Report the parameter as unavailable when it cannot be trusted
- **WHEN** the vehicle layer asks whether a `SPEED_MAX` value is available
- **THEN** the answer SHALL be true only if a value has been received, the value is greater than
  zero, the link is up, and the reading is younger than `MAVLINK_PARAM_STALE_MS`
- **AND** a value of zero SHALL be reported as unavailable, because `SPEED_MAX = 0` means "no
  limit" both to the autopilot and to this vehicle
- **AND** both the link gate and the age gate SHALL apply independently, so a disconnected cable
  is reported as unavailable within the heartbeat timeout while an autopilot that is alive but has
  stopped answering this parameter is reported as unavailable within the staleness timeout
- **AND** "unavailable" SHALL result in no limiting at all, not in a fallback ceiling

#### Scenario: Reset the subscription on re-initialisation
- **WHEN** the MAVLink interface is initialised
- **THEN** the stored value SHALL be cleared to "never received"
- **AND** the receipt timestamp and the poll timestamp SHALL be cleared, so the first poll after a
  restart is scheduled from the moment the autopilot is next learned

#### Scenario: Surface the subscription state on the diagnostic line
- **WHEN** the periodic (1 Hz) MAVLink debug line is emitted
- **THEN** it SHALL include the last received `SPEED_MAX` in m/s, the age of that reading, and
  whether the value is currently considered available
- **AND** this SHALL be sufficient to distinguish "never received", "received and fresh" and
  "received but stale" on the serial console alone

### Requirement: Outbound Speed Reporting in Metres per Second
The interface SHALL receive the vehicle's ground speed from the vehicle layer already in metres
per second, which is both the firmware's internal unit and the unit every MAVLink speed field
uses. It SHALL NOT perform a km/h conversion on any outbound speed path.

#### Scenario: VFR_HUD groundspeed needs no conversion
- **WHEN** a `VFR_HUD` message is packed
- **THEN** the `groundspeed` field SHALL carry the reported sensor speed directly
- **AND** no division by 3.6 SHALL be applied, because the reported speed is already m/s
- **AND** an invalid or non-positive reading SHALL still be reported as zero rather than as
  genuine motion

#### Scenario: The odometry delta needs no conversion
- **WHEN** a `VISION_POSITION_DELTA` message is packed
- **THEN** the signed longitudinal speed SHALL be the reported sensor speed multiplied by the
  travel direction, with no unit conversion
- **AND** the neutral-rolling suppression threshold SHALL be expressed in metres per second
  (`MAVLINK_VISO_NEUTRAL_ZERO_MS`)

