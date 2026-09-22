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
from the hall-effect speed sensor and reported via `VFR_HUD`, and the distance counters derived from
that same sensor are reported in `EFI_STATUS`. Every ECU value carried in
`EFI_STATUS` SHALL occupy the field that names that quantity, so the message is self-describing to a
consumer reading the MAVLink field names alone. A field MAY be repurposed for a value it does not
name ONLY where the named quantity is confirmed to be permanently unavailable on this vehicle, and
every such repurposing SHALL be documented. Where a value can be either measured by the ECU or
commanded by the controller, the reported value SHALL be unambiguous as to which of the two it is.

#### Scenario: Emit heartbeat
- **WHEN** the heartbeat interval elapses (1 Hz)
- **THEN** a `HEARTBEAT` is sent with `type = MAV_TYPE_GROUND_ROVER` and the ESP32's own
  system/component id
- **AND** `system_status` reflects fail-safe vs healthy operation

#### Scenario: Report engine telemetry
- **WHEN** the engine-telemetry interval elapses (default 5 Hz)
- **THEN** a single `EFI_STATUS` message is sent from this component carrying engine RPM, coolant
  temperature, intake air temperature, manifold absolute pressure, ECU calculated engine load,
  measured and commanded throttle, module supply voltage, both gear values, the total odometer and
  trip distance, the digital-output bitmask and a health flag
- **AND** all of those values SHALL be carried in distinct fields of that one message, rather than
  as per-value `NAMED_VALUE_FLOAT` messages, which share a single message id and therefore collide
  in any name-agnostic store
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

#### Scenario: Report manifold pressure and ECU engine load in EFI_STATUS
- **WHEN** the engine-telemetry interval elapses
- **AND** CAN `VehicleData` is valid
- **THEN** the `EFI_STATUS` message SHALL carry the manifold absolute pressure (OBD-II PID `0x0B`)
  in the `intake_manifold_pressure` field, in kPa
- **AND** SHALL carry the ECU's own calculated engine load (OBD-II PID `0x04`) in the `engine_load`
  field, as a percentage
- **AND** `engine_load` SHALL NOT carry any gear encoding — the gear values are reported in the fuel
  fields instead, which is what frees this field for its named quantity

#### Scenario: Report measured throttle position in EFI_STATUS
- **WHEN** the engine-telemetry interval elapses
- **THEN** the `EFI_STATUS` message SHALL carry the ECU-measured throttle position (OBD-II PID
  `0x11`) in the `throttle_position` field, as a percentage, and `NaN` when CAN `VehicleData` is
  invalid
- **AND** SHALL carry the commanded/arbitrated throttle percentage — the value the throttle servo is
  actually being driven with — in the `throttle_out` field
- **AND** the commanded value SHALL be the arbitrated output actually applied to the servo
  (autopilot command, web command, gear-change boost override, speed-limit cap, or fail-safe idle),
  not the raw command of any single input source
- **AND** `throttle_out` SHALL always be valid and SHALL NEVER be reported as `NaN`, because the
  commanded value is known locally and does not depend on CAN health
- **AND** a measured value SHALL NOT be substituted into `throttle_out`, nor a commanded value into
  `throttle_position`, so a consumer can compare the two directly

#### Scenario: Report current gear as a live named value
- **WHEN** the engine-telemetry interval elapses
- **THEN** the controller's ASSUMED (commanded) gear SHALL be sent as a live, graphable value in the
  `fuel_consumed` field of the same `EFI_STATUS` message, encoded by physical-sequence position
  `[R, N, H, L] = [-1, 0, 1, 2]` (exact in float)
- **AND** it SHALL NOT be sent as a `NAMED_VALUE_FLOAT`: all `NAMED_VALUE_FLOAT` messages share one
  message id and are distinguished only by a `name` field, so any name-agnostic store keeps only the
  last one received and the values appear to overwrite each other — which is why the value lives in
  a dedicated `EFI_STATUS` field instead
- **AND** the value SHALL always be valid and SHALL NEVER be reported as `NaN` — the transmission is
  commanded time-based, so the assumed gear is always known, independent of both CAN health and
  gear-switch health
- **AND** the value reflects intent, not a measured position; the measured position is reported
  separately in `fuel_flow`

#### Scenario: Show a shift as the midpoint between gears
- **WHEN** the servo is actively moving between two gears (a gear-change phase is active)
- **THEN** the `fuel_consumed` value SHALL be the midpoint of the outgoing and incoming gear values
  (e.g. moving N→H reports `0.5`, moving R→N reports `-0.5`)
- **AND** when settled or dwelling at a gear the value SHALL be that gear's integer position
- **AND** a multi-step change (e.g. R→L) SHALL therefore render as an even 0.5 staircase through
  the sequence: `-1.0 → -0.5 → 0.0 → 0.5 → 1.0 → 1.5 → 2.0`
- **AND** the midpoint SHALL apply to the assumed gear only — `fuel_flow` (the physically sensed
  gear) SHALL NEVER be interpolated, reading `NaN` while the mechanism is between detents

#### Scenario: Report the physically sensed gear in EFI_STATUS fuel_flow
- **WHEN** the engine-telemetry interval elapses
- **AND** the transmission's gear opto-switches report an unambiguous gear
- **THEN** the `EFI_STATUS` message SHALL carry that PHYSICALLY SENSED gear in the `fuel_flow`
  field, using the same `[R, N, H, L] = [-1, 0, 1, 2]` encoding
- **AND** the value SHALL be a measurement, never a midpoint or an interpolation
- **AND** a consumer SHALL be able to compare it against `fuel_consumed` to distinguish what the
  controller intended from what the transmission actually did

#### Scenario: Report an unknown physical gear as NaN
- **WHEN** the transmission's physical gear reads UNKNOWN — the mechanism is between detents during
  a shift, the switch pattern is ambiguous, or the input GPIO expander is faulted
- **THEN** `EFI_STATUS.fuel_flow` SHALL be reported as `NaN` rather than as any numeric gear value,
  because every plausible sentinel collides with a real gear or with the midpoint staircase
- **AND** `NaN` on this field during a shift SHALL be understood as the EXPECTED signature of a gear
  in motion, not as a fault
- **AND** `fuel_consumed`, and every other `EFI_STATUS` field, SHALL remain unaffected — the
  physical-gear reading is the only value gated on gear-switch validity
- **AND** the firmware SHALL NOT attempt to distinguish mid-shift from a sensor fault on the wire;
  that inference belongs to the consumer, which has both gear values and a clock

#### Scenario: Report the odometer and the trip distance in EFI_STATUS
- **WHEN** the engine-telemetry interval elapses
- **THEN** the `EFI_STATUS` message SHALL carry the TOTAL odometer in the `barometric_pressure`
  field, in kilometres as a float
- **AND** SHALL carry the resettable TRIP distance in the `fuel_pressure` field, in kilometres as a
  float
- **AND** both values SHALL be the vehicle layer's exact millimetre counters scaled by `1e-6`,
  reported to the precision float32 allows — the millimetre counter, not this float, SHALL remain
  the authoritative value
- **AND** both SHALL ALWAYS be valid and SHALL NEVER be reported as `NaN`, because distance already
  driven depends on neither CAN health nor the current speed reading's validity
- **AND** neither value SHALL be reported as a `NAMED_VALUE_FLOAT`, for the same message-id
  collision reason that keeps the gear values in `EFI_STATUS`
- **AND** the odometer SHALL NEVER decrease between two messages, and a trip reset SHALL be
  observable as `fuel_pressure` falling to zero while `barometric_pressure` is unchanged

#### Scenario: Preserve existing EFI_STATUS field assignments
- **WHEN** ECU values are mapped into `EFI_STATUS` after this remap
- **THEN** the `rpm`, `cylinder_head_temperature`, `intake_manifold_temperature`,
  `intake_manifold_pressure` and `ignition_voltage` fields SHALL continue to carry the quantities
  they already carried — engine RPM, coolant temperature, intake air temperature, manifold absolute
  pressure and control module supply voltage respectively
- **AND** the `pt_compensation` field SHALL continue to carry the digital-output bitmask
- **AND** the ONLY assignments that move SHALL be the gear value, which vacates `engine_load` for
  `fuel_consumed` (assumed gear) and `fuel_flow` (physically sensed gear), and the ECU-measured
  throttle position, which vacates `throttle_out` for `throttle_position`
- **AND** `engine_load` SHALL therefore carry the ECU's own calculated engine load, and
  `throttle_out` the commanded/arbitrated throttle, each being the quantity that field names
- **AND** any ECU value without a free, semantically appropriate field SHALL be omitted from MAVLink
  rather than mapped onto a mismatched field
- **AND** a value already carried in one `EFI_STATUS` field SHALL NOT be duplicated into a second
  field of the same message

#### Scenario: Repurpose only permanently-free fields
- **WHEN** a value is carried in an `EFI_STATUS` field that does not name it
- **THEN** the only such fields SHALL be `fuel_consumed` and `fuel_flow` (carrying the two gear
  values), `pt_compensation` (carrying the digital-output bitmask), `barometric_pressure` (carrying
  the total odometer in km) and `fuel_pressure` (carrying the trip distance in km)
- **AND** the fuel fields SHALL be justified by the bench-confirmed permanent absence of the
  corresponding ECU signals on this vehicle (PIDs `0x2F` fuel level and `0x5C` oil temperature do
  not answer and are absent from the supported-PID bitmaps), so no genuine fuel quantity or flow can
  ever be displaced
- **AND** `barometric_pressure` SHALL be justified on the same terms: this ECU has no barometric or
  ambient-pressure sensor, no such PID answers the probe, and the manifold pressure it does measure
  is already carried in `intake_manifold_pressure`, the field that names it
- **AND** `fuel_pressure` reading zero SHALL be understood as a genuine zero trip distance, even
  though the MAVLink field definition assigns zero the meaning "unknown"; the sentinel value that
  definition suggests SHALL NOT be substituted, because reporting a non-zero distance when the
  operator has just reset the trip is the worse error
- **AND** fields naming engine quantities this ECU could plausibly expose later —
  `spark_dwell_time`, `ignition_timing`, `injection_time`, `exhaust_gas_temperature` — SHALL be left
  unused rather than repurposed
- **AND** any ECU value without a free, semantically appropriate field SHALL be omitted from MAVLink
  rather than mapped onto a mismatched field
- **AND** a value already carried in one `EFI_STATUS` field SHALL NOT be duplicated into a second
  field of the same message

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
- **AND** `EFI_STATUS.throttle_position` SHALL read `NaN` in the same reporting tick, so a consumer
  can tell unambiguously that the `VFR_HUD` value is commanded rather than measured
- **AND** in that same tick `VFR_HUD.throttle` SHALL equal `EFI_STATUS.throttle_out`, which carries
  the same commanded value
- **AND** `VFR_HUD` SHALL NOT be suppressed for that tick, since its ground-speed field is governed
  by the hall sensor's own validity, independent of CAN validity

#### Scenario: Report vehicle speed via VFR_HUD
- **WHEN** the engine-telemetry interval elapses
- **AND** the hall-effect speed sensor reading is valid
- **THEN** a `VFR_HUD` message SHALL be sent with `groundspeed` set to the sensor speed in metres
  per second, INCLUDING a genuine `0.0` when the vehicle is measurably stopped
- **AND** the GCS SHALL be able to display and graph it as ground speed for the ground rover
- **WHEN** the hall sensor reading is invalid — never pulsed since boot, or latched suspicious
- **THEN** `groundspeed` SHALL be reported as `NaN` rather than as `0`, so that "no reading" is
  distinguishable on the wire from "stopped"
- **AND** the `VFR_HUD` message SHALL still be sent for that tick, because its `throttle` field is
  governed by CAN validity and not by the speed sensor

#### Scenario: Report state changes as status text
- **WHEN** the current gear, ignition state, or fail-safe state changes
- **THEN** a `STATUSTEXT` message describing the new state is sent (rate-limited)

#### Scenario: Suppress engine telemetry when CAN data is invalid
- **WHEN** CAN `VehicleData` is invalid or stale
- **THEN** engine-telemetry messages are either skipped or marked unhealthy
- **AND** `HEARTBEAT` continues to be sent for link liveness
- **AND** the `rpm`, `cylinder_head_temperature`, `intake_manifold_temperature`,
  `intake_manifold_pressure`, `engine_load`, `throttle_position` and `ignition_voltage` fields SHALL
  ALL be reported as `NaN`, so a ground station shows "no data" rather than misleading zeros
- **AND** the `fuel_consumed` (assumed gear), `throttle_out` (commanded throttle),
  `pt_compensation` (digital-output bitmask), `barometric_pressure` (odometer) and `fuel_pressure`
  (trip distance) fields SHALL remain valid, because none of them depends on CAN health
- **AND** `fuel_flow` (physical gear) SHALL be governed by the gear-switch validity alone and SHALL
  be unaffected by CAN validity, so a lone `fuel_flow` `NaN` alongside otherwise-populated fields
  means "gear sensor unsure" and says nothing about CAN
- **AND** `VFR_HUD.throttle` falls back to the commanded throttle percentage
- **AND** `VFR_HUD` ground-speed reporting is governed by the hall sensor's own validity, independent
  of CAN validity

#### Scenario: Breaking field move for existing EFI_STATUS consumers
- **WHEN** the `EFI_STATUS` field mapping moves gear from `engine_load` to `fuel_consumed` and the
  measured throttle position from `throttle_out` to `throttle_position`
- **THEN** this SHALL be treated as a BREAKING change for any consumer that decodes `EFI_STATUS` by
  packet subscription, and the firmware flash and the ground-station plugin update SHALL be deployed
  in lockstep
- **AND** the repository documentation SHALL carry a migration table complete enough to update an
  out-of-repo consumer from that table alone
- **AND** the documentation SHALL state the failure mode of an un-updated consumer: reading gear
  from `engine_load` now yields the ECU's calculated load (0-100 %), an implausible "gear" that
  tracks engine effort
- **AND** a consumer SHOULD render any gear value outside the range `-1..2` as "no data" so the
  mismatch is self-evident rather than displayed as a wrong gear

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
uses. It SHALL NOT perform a km/h conversion on any outbound speed path. Where the reading is
unavailable, the interface SHALL say so in the field's own encoding rather than substituting a
plausible number.

#### Scenario: VFR_HUD groundspeed needs no conversion
- **WHEN** a `VFR_HUD` message is packed
- **THEN** the `groundspeed` field SHALL carry the reported sensor speed directly
- **AND** no division by 3.6 SHALL be applied, because the reported speed is already m/s
- **AND** a valid reading of `0.0` SHALL be carried through as `0.0`, because a measurably stopped
  vehicle is a genuine measurement

#### Scenario: Report an invalid ground speed as NaN
- **WHEN** a `VFR_HUD` message is packed
- **AND** the hall-effect speed sensor reading is invalid — it has never pulsed since boot, or it
  is latched suspicious after an implausible pulse loss
- **THEN** `groundspeed` SHALL be packed as `NaN`, NOT as `0`
- **AND** a consumer SHALL therefore be able to distinguish "no reading" from "stopped", which a
  zero cannot express
- **AND** the substitution SHALL NOT be applied in the other direction: a valid reading SHALL NEVER
  be replaced by `NaN`, and `NaN` SHALL NEVER be used to mean "stopped"
- **AND** `VFR_HUD.throttle` SHALL be unaffected — it is a `uint16_t` percentage with no `NaN`
  encoding and keeps its measured-TPS-with-commanded-fallback behaviour
- **AND** `VISION_POSITION_DELTA` SHALL be unaffected — it is a fusable measurement, not a display
  value, and already goes silent rather than reporting an invalid reading at all

#### Scenario: The odometry delta needs no conversion
- **WHEN** a `VISION_POSITION_DELTA` message is packed
- **THEN** the signed longitudinal speed SHALL be the reported sensor speed multiplied by the
  travel direction, with no unit conversion
- **AND** the neutral-rolling suppression threshold SHALL be expressed in metres per second
  (`MAVLINK_VISO_NEUTRAL_ZERO_MS`)

### Requirement: Inbound Command Handling and Trip Reset
The system SHALL accept inbound `COMMAND_LONG` (#76) messages addressed to THIS component and
SHALL answer each one with a `COMMAND_ACK`. Because this component deliberately shares the
autopilot's system id (`MAVLINK_SYSTEM_ID`) and is distinguished only by `MAVLINK_COMPONENT_ID`,
the addressing check SHALL be strict: a command SHALL be handled only when BOTH
`target_system == MAVLINK_SYSTEM_ID` AND `target_component == MAVLINK_COMPONENT_ID`. A broadcast
(`target_component == 0`) or a command addressed to any other component SHALL be ignored silently,
with NO acknowledgement, so that this peripheral can never answer for the autopilot.

The only command this system implements is a **trip reset**, carried as `MAV_CMD_USER_1` (31010)
with a magic `param1`. The odometer SHALL NOT be resettable by any command.

The handler SHALL follow the existing transport policy: `include/MavlinkInterface.h` stays free of
mavlink headers, so the handler SHALL take decoded scalars (the `handleServoOutputRaw` /
`handleParamValue` pattern), and the transport SHALL NOT hold a reference to the speed sensor —
it SHALL latch a pending request that the vehicle layer consumes and performs.

#### Scenario: Accept a trip reset addressed to this component
- **WHEN** a `COMMAND_LONG` arrives with `target_system == MAVLINK_SYSTEM_ID`,
  `target_component == MAVLINK_COMPONENT_ID`, `command == MAV_CMD_USER_1` (31010) and `param1`
  equal to `MAVLINK_CMD_TRIP_RESET_MAGIC`
- **THEN** a trip-reset request SHALL be latched for the vehicle layer to perform
- **AND** the vehicle layer SHALL zero the trip counter and persist it on its next control
  iteration
- **AND** a `COMMAND_ACK` with `MAV_RESULT_ACCEPTED` SHALL be sent
- **AND** one line SHALL be logged on the serial console naming the requester's system and
  component (for example `[MAV] TRIP reset accepted from 255/190`)
- **AND** the odometer SHALL be left unchanged

#### Scenario: Compare the magic parameter with a tolerance
- **WHEN** `param1` is evaluated against `MAVLINK_CMD_TRIP_RESET_MAGIC`
- **THEN** the comparison SHALL use a tolerance rather than float equality, in line with the
  existing rule against comparing floats with `==`
- **AND** a `param1` of `NaN` SHALL fail the comparison and SHALL therefore be denied, not accepted

#### Scenario: Deny a trip reset with the wrong parameter
- **WHEN** a `COMMAND_LONG` for `MAV_CMD_USER_1` is addressed to this component with any `param1`
  other than the magic value
- **THEN** a `COMMAND_ACK` with `MAV_RESULT_DENIED` SHALL be sent
- **AND** the rejection SHALL be logged with the offending `param1` value and the requester
- **AND** the trip counter SHALL be left unchanged, so a stray, replayed or mis-scripted
  `MAV_CMD_USER_1` cannot silently destroy the operator's reading

#### Scenario: Report any other command as unsupported
- **WHEN** a `COMMAND_LONG` carrying any command other than `MAV_CMD_USER_1` is addressed
  specifically to this system AND this component
- **THEN** a `COMMAND_ACK` with `MAV_RESULT_UNSUPPORTED` SHALL be sent
- **AND** the command id and the requester SHALL be logged
- **AND** no vehicle state SHALL change

#### Scenario: Ignore broadcasts and commands for other components
- **WHEN** a `COMMAND_LONG` arrives with `target_component == 0` (broadcast), with a
  `target_component` that is not `MAVLINK_COMPONENT_ID`, or with a `target_system` that is not
  `MAVLINK_SYSTEM_ID`
- **THEN** the message SHALL be discarded with NO `COMMAND_ACK` and no log line
- **AND** a normal ground-station session against the autopilot (arming, mode changes, mission
  upload) SHALL therefore produce no acknowledgement from this component at all

#### Scenario: Acknowledge back to the requester, not to the autopilot
- **WHEN** a `COMMAND_ACK` is sent
- **THEN** its `target_system` and `target_component` SHALL be the `sysid` and `compid` of the
  message that requested the command, not the learned autopilot's
- **AND** the ACK SHALL be sent from this component's own `MAVLINK_SYSTEM_ID` /
  `MAVLINK_COMPONENT_ID`
- **AND** delivery SHALL rely on the autopilot's existing routing, which has already learned this
  component's address from its outbound `HEARTBEAT` and `EFI_STATUS` traffic — no additional
  routing configuration SHALL be required

#### Scenario: The odometer is not resettable over MAVLink
- **WHEN** any inbound command is processed
- **THEN** there SHALL be NO command, parameter or magic value that zeroes or decreases the
  odometer
- **AND** only the trip counter SHALL be clearable

