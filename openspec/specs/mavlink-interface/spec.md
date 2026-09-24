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
that same sensor, together with the total and trip engine hour meters, are reported in `EFI_STATUS`.
Every ECU value
carried in
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
  measured and commanded throttle, module supply voltage, both gear values, the total odometer, the
  trip distance, the total engine hour meter, the trip engine hours, the digital-output bitmask and
  a health flag
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
- **AND** the transmission's gear switches report an unambiguous gear
- **THEN** the `EFI_STATUS` message SHALL carry that PHYSICALLY SENSED gear in the `fuel_flow`
  field, using the same `[R, N, H, L] = [-1, 0, 1, 2]` encoding
- **AND** the value SHALL be a measurement, never a midpoint or an interpolation
- **AND** a consumer SHALL be able to compare it against `fuel_consumed` to distinguish what the
  controller intended from what the transmission actually did

#### Scenario: Report an unknown physical gear as NaN
- **WHEN** the transmission's physical gear reads UNKNOWN — the mechanism is between detents during
  a shift, the switch pattern is ambiguous, or no switch is active
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
  observable as `fuel_pressure` falling to zero — together with `injection_time`, the trip engine
  hours — while `barometric_pressure` and `spark_dwell_time` are unchanged

#### Scenario: Report the engine hour meter in EFI_STATUS
- **WHEN** the engine-telemetry interval elapses
- **THEN** the `EFI_STATUS` message SHALL carry the total engine hour meter in the
  `spark_dwell_time` field, in HOURS as a float
- **AND** SHALL carry the resettable TRIP engine hours in the `injection_time` field, in HOURS as a
  float
- **AND** both values SHALL be the vehicle layer's exact whole-second counters divided by 3600 —
  the second counters, not these floats, SHALL remain the authoritative values
- **AND** both SHALL ALWAYS be valid and SHALL NEVER be reported as `NaN`, because accumulated
  running time is history and depends on neither the CURRENT CAN health nor the current engine
  state, even though only valid CAN data can make them grow
- **AND** neither SHALL be reported as a `NAMED_VALUE_FLOAT`, for the same message-id collision
  reason that keeps the gear values and the distance counters in `EFI_STATUS`
- **AND** the TOTAL SHALL NEVER decrease between two messages, and no command, parameter, magic
  value or web control SHALL exist that zeroes it
- **AND** the TRIP value SHALL decrease only to zero and only on an accepted trip reset, and
  `injection_time` reading zero SHALL be understood as a GENUINE zero trip-hours reading rather
  than as "unknown", on the same terms as `fuel_pressure`
- **AND** a trip reset SHALL therefore be observable on the wire as `fuel_pressure` AND
  `injection_time` both falling to zero in the same message, while `barometric_pressure` and
  `spark_dwell_time` are unchanged

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
  the total odometer in km), `fuel_pressure` (carrying the trip distance in km),
  `spark_dwell_time` (carrying the total engine hour meter in hours) and `injection_time`
  (carrying the trip engine hours in hours)
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
- **AND** `spark_dwell_time` SHALL be justified by it being a reserved engine-quantity field with
  no route onto this bus at all: spark dwell has no standard OBD-II Mode 01 PID, so this ECU can
  never surface it, and the field can therefore be treated as permanently free rather than merely
  unused today
- **AND** `injection_time` SHALL be justified on the same test: it too has NO PID of its own, being
  only DERIVABLE from the fuel-trim PIDs together with engine load, so it can never arrive on this
  bus as a reported field the way a standard PID could
- **AND** `injection_time` reading zero SHALL be understood as a genuine zero trip-hours reading,
  not as "unknown", for the identical reason `fuel_pressure` zero is a genuine zero: reporting a
  non-zero figure when the operator has just reset the trip is the worse error
- **AND** the fields naming engine quantities this ECU could plausibly expose later over a standard
  Mode 01 PID — `ignition_timing` (PID `0x0E`) and `exhaust_gas_temperature` (PID `0x78`) — SHALL
  be left unused rather than repurposed, and a consumer SHALL treat `0.0` in those two as "not
  reported"
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
  `pt_compensation` (digital-output bitmask), `barometric_pressure` (odometer), `fuel_pressure`
  (trip distance), `spark_dwell_time` (total engine hours) and `injection_time` (trip engine hours)
  fields SHALL remain valid, because none of them depends on CAN health
- **AND** BOTH engine hour counters SHALL simply STOP GROWING while CAN is invalid, rather than
  being reported as `NaN` or continuing to accumulate on an assumed engine state — an unknown
  engine state SHALL NOT invent running time
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
interface or the web portal. The relays SHALL be driven DIRECTLY from ESP32 GPIO — there is no
I2C port expander in the path and therefore no write verification and no expander fault state.

| Function | Pin | GPIO |
|----------|-----|------|
| Ignition / ECU line | `PIN_RELAY1` | GPIO36 |
| Starter | `PIN_RELAY2` | GPIO37 |
| Front light | `PIN_RELAY3` | GPIO38 |
| Front-wheel lock | `PIN_WHEEL_LOCK` | GPIO39 |

#### Scenario: Initialize relay controller
- **WHEN** `RelayController.begin()` is called
- **THEN** `PIN_RELAY1`, `PIN_RELAY2`, `PIN_RELAY3` and `PIN_WHEEL_LOCK` SHALL be configured as
  outputs
- **AND** all four SHALL be driven LOW (all relays OFF, safe state)
- **AND** internal state tracking is initialized to OFF / light off / lock off
- **AND** true is returned

#### Scenario: Set ignition state
- **WHEN** `setIgnitionState(OFF)` is called, `PIN_RELAY1` and `PIN_RELAY2` are LOW
- **WHEN** `setIgnitionState(ACC)` is called, `PIN_RELAY1` is HIGH and `PIN_RELAY2` is LOW
- **WHEN** `setIgnitionState(IGNITION)` is called, `PIN_RELAY1` is HIGH and `PIN_RELAY2` is LOW
- **WHEN** `setIgnitionState(CRANKING)` is called, `PIN_RELAY1` and `PIN_RELAY2` are both HIGH
- **AND** internal state is updated to match in each case

#### Scenario: Control front light
- **WHEN** `setFrontLight(true)` is called, `PIN_RELAY3` is set HIGH and state is updated
- **WHEN** `setFrontLight(false)` is called, `PIN_RELAY3` is set LOW and state is updated

#### Scenario: Control front-wheel lock
- **WHEN** `setWheelLock(true)` is called, `PIN_WHEEL_LOCK` is set HIGH and state is updated
- **WHEN** `setWheelLock(false)` is called, `PIN_WHEEL_LOCK` is set LOW and state is updated

#### Scenario: Fail-safe all relays off
- **WHEN** `allOff()` is called (fail-safe trigger)
- **THEN** `PIN_RELAY1`, `PIN_RELAY2` and `PIN_RELAY3` are set LOW
- **AND** internal state is reset to safe defaults (OFF, lights off)
- **AND** the front-wheel lock (`PIN_WHEEL_LOCK`) is intentionally left in its last state

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
with a magic `param1`. That ONE command SHALL clear BOTH resettable trip counters — the trip
DISTANCE and the trip ENGINE HOURS — because they describe the same "since the operator last
reset" interval in different units and SHALL NOT be allowed to diverge. There SHALL be NO second
command, NO additional magic `param1` value and NO separate web control for either of them. The
odometer and the TOTAL engine hour meter SHALL NOT be resettable by any command.

The handler SHALL follow the existing transport policy: `include/MavlinkInterface.h` stays free of
mavlink headers, so the handler SHALL take decoded scalars (the `handleServoOutputRaw` /
`handleParamValue` pattern), and the transport SHALL NOT hold a reference to the speed sensor —
it SHALL latch a pending request that the vehicle layer consumes and performs.

#### Scenario: Accept a trip reset addressed to this component
- **WHEN** a `COMMAND_LONG` arrives with `target_system == MAVLINK_SYSTEM_ID`,
  `target_component == MAVLINK_COMPONENT_ID`, `command == MAV_CMD_USER_1` (31010) and `param1`
  equal to `MAVLINK_CMD_TRIP_RESET_MAGIC`
- **THEN** a trip-reset request SHALL be latched for the vehicle layer to perform
- **AND** the vehicle layer SHALL zero the trip DISTANCE counter and persist it on its next control
  iteration
- **AND** in the SAME operation the vehicle layer SHALL zero the trip ENGINE HOURS counter and
  persist it, so the two trip readings can never disagree about which interval they describe
- **AND** a `COMMAND_ACK` with `MAV_RESULT_ACCEPTED` SHALL be sent
- **AND** one line SHALL be logged on the serial console naming the requester's system and
  component (for example `[MAV] TRIP reset accepted from 255/190`)
- **AND** the odometer AND the TOTAL engine hour meter SHALL be left unchanged
- **AND** the inbound command handler SHALL NOT be changed to accommodate the second counter: it
  SHALL still latch exactly one request, carry no reference to the vehicle's counters, and know
  nothing of which counters the vehicle layer clears

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
- **AND** BOTH trip counters SHALL be left unchanged, so a stray, replayed or mis-scripted
  `MAV_CMD_USER_1` cannot silently destroy the operator's readings

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
  odometer, and none that zeroes or decreases the TOTAL engine hour meter
- **AND** only the two trip counters — the trip distance and the trip engine hours — SHALL be
  clearable, and only together

### Requirement: External Navigation Odometry Reporting
The system SHALL report the hall-effect wheel speed to the autopilot as a fusable body-frame
distance increment using the `VISION_POSITION_DELTA` message, so the EKF3 can use it as a
body-odometry aiding source on a vehicle without reliable GPS. The measurement SHALL be suppressed
entirely rather than reported as zero whenever its validity or its sign is in doubt, and a
suppressed interval SHALL NOT be folded into a later increment.

#### Scenario: Report wheel travel as a body-frame position delta
- **WHEN** the report interval elapses (`MAVLINK_REPORT_TX_MS`, default 200 ms / 5 Hz)
- **AND** the link is up and the speed reading is valid
- **AND** the travel direction is known, or the vehicle is not rolling in neutral
- **AND** a valid integration baseline exists (see the re-baseline scenario)
- **THEN** a `VISION_POSITION_DELTA` message SHALL be sent from the ESP32's component id
- **AND** the signed longitudinal speed SHALL be the sensor speed in metres per second multiplied
  by the travel direction (`speedMs · travelDirection`)
- **AND** `position_delta` SHALL be that signed speed multiplied by the integration interval,
  placed on the body-frame forward axis, with the lateral and vertical components exactly zero
- **AND** the velocity SHALL NOT be rotated into the earth frame, and no heading SHALL be required
  to send the message, because the autopilot rotates the body-frame increment with its own
  attitude

#### Scenario: Declare no measured rotation
- **WHEN** a `VISION_POSITION_DELTA` is packed
- **THEN** `angle_delta` SHALL be `{0, 0, 0}`
- **AND** a rotation SHALL NOT be fabricated from any other source, because the wheel sensor
  measures no rotation and the autopilot's own gyros own that channel

#### Scenario: Report the actually measured integration interval
- **WHEN** a `VISION_POSITION_DELTA` is packed
- **THEN** `time_delta_usec` SHALL be the measured elapsed time since the previous transmitted
  delta, taken from the same monotonic microsecond clock
- **AND** a nominal or assumed interval SHALL NOT be substituted, because the autopilot divides
  the position delta by exactly this value to recover a velocity, so an assumed interval would
  inject a proportional velocity error
- **AND** `time_usec` SHALL be taken from a 64-bit monotonic microsecond clock
  (`esp_timer_get_time()`), never a 32-bit microsecond source whose wrap (about every 71 minutes)
  would present a fresh measurement as a very old one
- **AND** no attempt SHALL be made to convert to autopilot time — the autopilot corrects the boot
  offset itself

#### Scenario: Re-baseline the interval after a break
- **WHEN** the message has been suppressed by any gate, or no delta has ever been sent, or the
  elapsed time since the previous delta exceeds `MAVLINK_VISO_MAX_DT_US`
- **THEN** the next opportunity SHALL NOT send a message
- **AND** it SHALL instead re-establish the integration baseline from the current clock, so that
  exactly one interval is discarded and the following message is integrated over an interval that
  was actually observed
- **AND** the current speed SHALL NOT be multiplied by the elapsed time across the break, because
  a delta is an integral and reporting travel over an unobserved interval would inject a large
  spurious position step

#### Scenario: Declare full confidence when the message is sent at all
- **WHEN** a `VISION_POSITION_DELTA` is packed
- **THEN** `confidence` SHALL be `MAVLINK_VISO_CONFIDENCE` (100)
- **AND** a reduced confidence SHALL NOT be used to signal degraded health, because the health
  policy is suppression: a message that is sent has passed every gate
- **AND** this SHALL keep the autopilot's derived velocity error at the tight end of its
  configured range, satisfying the autopilot's requirement that the error be below 1 m/s before
  body-odometry aiding may begin

#### Scenario: Derive travel direction from the physical gear
- **WHEN** the travel direction is determined
- **THEN** it SHALL be derived from the physically sensed gear (the opto gear switches), mapping
  REVERSE to `-1`, LOW and HIGH to `+1`, and NEUTRAL or UNKNOWN to `0`
- **AND** it SHALL NOT be derived from the assumed or commanded gear, because the transmission
  command path is sensorless and an assumption must never sign a fused measurement
- **AND** a gear reading that is briefly UNKNOWN during a shift SHALL yield direction `0`, which
  suppresses the sample rather than guessing its sign

#### Scenario: Suppress the message when the link is down or the speed is invalid
- **WHEN** the report interval elapses
- **AND** the autopilot link is down, **OR** the hall speed reading is invalid or stale
- **THEN** no `VISION_POSITION_DELTA` SHALL be sent for that tick
- **AND** the integration baseline SHALL be invalidated
- **AND** a zero delta SHALL NOT be substituted, because an invalid reading includes the sensor's
  wire-fault (suspicious) latch, where a disconnected sensor decays to zero while the vehicle is
  still moving — fusing that as "stopped" would corrupt the state estimate
- **AND** the other outbound messages (`HEARTBEAT`, `EFI_STATUS`, `VFR_HUD`) SHALL continue to be
  sent unaffected

#### Scenario: Suppress the message when rolling in neutral
- **WHEN** the report interval elapses
- **AND** the travel direction is `0` (neutral, or gear unknown)
- **AND** the speed reading exceeds `MAVLINK_VISO_NEUTRAL_ZERO_MS`
- **THEN** no `VISION_POSITION_DELTA` SHALL be sent for that tick, because the vehicle is moving
  with no recoverable direction sign and a wrong sign would inject an error of twice the speed
- **AND** the integration baseline SHALL be invalidated

#### Scenario: Send healthy zero-motion updates
- **WHEN** the report interval elapses
- **AND** the link is up and the speed reading is valid
- **AND** the vehicle is stationary — including stationary in neutral, with a speed at or below
  `MAVLINK_VISO_NEUTRAL_ZERO_MS`
- **THEN** a `VISION_POSITION_DELTA` carrying a zero position delta SHALL be sent
- **AND** it SHALL NOT be suppressed, because with no reliable GPS a zero-motion update is the
  strongest available constraint on estimator drift while the vehicle is parked or idling

#### Scenario: Require no inbound subscription
- **WHEN** the external navigation odometry feature is active
- **THEN** the interface SHALL NOT subscribe to, request, or depend upon the autopilot's
  `ATTITUDE` message or any other inbound message beyond the existing `HEARTBEAT` and
  `SERVO_OUTPUT_RAW`
- **AND** the stream re-request logic SHALL therefore gate on the command stream rate alone

#### Scenario: Expose the gating state on the debug log
- **WHEN** the periodic MAVLink debug line is emitted (1 Hz, `DebugFeature::MAVLINK`)
- **THEN** it SHALL include the count of `VISION_POSITION_DELTA` messages sent and the last
  transmitted integration interval
- **AND** an operator SHALL therefore be able to tell, from serial output alone and without a
  ground station, whether the message is flowing and whether the baseline is being repeatedly
  re-established by a flapping gate

### Requirement: Steering VESC Telemetry via NAMED_VALUE_FLOAT
The system SHALL report the steering VESC's electrical and thermal telemetry, its link health, the
measured position of the steering axis that VESC drives, and the steering speed-scale being applied
to that axis's commands, to the MAVLink network as **exactly six** `NAMED_VALUE_FLOAT` messages
(id 251) sent from this component. These six named values SHALL
be the ONLY `NAMED_VALUE_FLOAT` this component ever sends: they are a **scoped exception**, granted
because the standard ESC telemetry messages are absent from the dialect the ground station decodes
with and are therefore discarded before any consumer can read them. The exception SHALL NOT extend
to any engine, gear, digital-flag, odometer or trip value, all of which SHALL continue to be carried
in distinct fields of the single `EFI_STATUS` message. Every reported value SHALL be gated by the
validity of its OWN source, SHALL be reported as `NaN` rather than as a stale or zero reading when
that source is unhealthy, and SHALL keep being sent regardless of source health — except the link
flag and the steering scale, which SHALL never be `NaN`. None of these values SHALL be a control
input.

#### Scenario: Five named values from this component
- **WHEN** the interface is reporting vehicle state
- **THEN** it SHALL send exactly six `NAMED_VALUE_FLOAT` names from the ESP32's own system and
  component id (system 1, component 25): `STEER_POS`, `STEER_A`, `VESC_V`, `VESC_TEMP`,
  `VESC_OK` and `STEER_SCA`
- **AND** `STEER_POS` SHALL carry the MEASURED steering position as a percentage of the CALIBRATED
  lock-to-lock range, −100 at the left limit, 0 at centre, +100 at the right limit — percent of
  calibrated travel, NOT degrees, because the firmware holds no counts-to-degrees calibration
- **AND** a negative `STEER_POS` SHALL mean LEFT of centre and a positive value SHALL mean RIGHT of
  centre, matching the sign convention of the steering-percent accessor
- **AND** `STEER_A` SHALL carry the VESC's average MOTOR current in amperes, which is the steering
  load and stall diagnostic; the VESC's average INPUT current SHALL NOT be reported under this name
- **AND** `VESC_V` SHALL carry the input voltage measured by the VESC itself — the 24 V boost rail
  that powers the steering driver — in volts
- **AND** `VESC_TEMP` SHALL carry the VESC's power-stage (FET) temperature in degrees Celsius
- **AND** `VESC_OK` SHALL carry the steering driver's link health as `1.0` or `0.0`
- **AND** `STEER_SCA` SHALL carry the steering speed-scale the vehicle layer is applying to
  autopilot steering commands after rate limiting, in the range 0 to 1 inclusive, `1.0` meaning
  "not scaling"; it is sourced from the vehicle layer rather than from the VESC and SHALL NOT be
  gated by the steering driver's link health
- **AND** the VESC's raw fault code and any boot-cumulative reply or fault counters SHALL NOT be
  reported on this link; they remain available on the web portal and the serial console

#### Scenario: Three names on the report tick and two on the slow timer
- **WHEN** the engine/state report interval elapses (`MAVLINK_REPORT_TX_MS`, default 200 ms / 5 Hz)
- **THEN** `STEER_POS`, `STEER_A` and `VESC_V` SHALL be sent in that same tick, after `VFR_HUD`, so
  a ground-station log lines the live values up with `EFI_STATUS` without interpolation
- **AND** `VESC_TEMP`, `VESC_OK` and `STEER_SCA` SHALL be sent on their own slower interval
  (`MAVLINK_STEER_SLOW_TX_MS`, default 1000 ms / 1 Hz), because a power stage's thermal time
  constant is seconds, a link flag needs no faster rate, and the steering scale moves at the pace
  the vehicle accelerates
- **AND** the `time_boot_ms` field of all six SHALL carry the milliseconds-since-boot value already
  computed for that report tick; a microsecond clock SHALL NOT be used, and the 49.7-day wrap of
  that field is its defined behaviour and SHALL NOT be worked around

#### Scenario: Electrical and thermal values are NaN while the VESC is silent
- **WHEN** no valid VESC reply has arrived within the steering driver's communication timeout, or
  no reply has ever arrived since boot
- **THEN** `STEER_A`, `VESC_V` and `VESC_TEMP` SHALL each be reported as `NaN`
- **AND** the last value seen SHALL NOT be repeated, and zero SHALL NOT be substituted, because a
  stale or zero electrical reading is indistinguishable from a real one at the ground station
- **AND** a consumer SHALL render `NaN` as "no data" rather than as a number

#### Scenario: VESC_OK is the validity flag and is never NaN
- **WHEN** any `VESC_OK` message is sent
- **THEN** its value SHALL be `1.0` when a valid VESC reply arrived within the steering driver's
  communication timeout and `0.0` when it did not
- **AND** it SHALL NEVER be `NaN`, because it is the value that TELLS a consumer why the other
  VESC-sourced names went `NaN`, and an "unknown" there would defeat its only purpose
- **AND** `VESC_OK` = `0.0` SHALL be read as "the steering driver is offline", not as "the
  peripheral is offline"
- **AND** `STEER_SCA` SHALL likewise NEVER be `NaN`: every condition that disables the scaling
  resolves to `1.0`, because "not scaling" is a definite answer and a ground station should plot it
  as a flat line at one rather than as a gap in the record

#### Scenario: Steering position validity is the sensor's, independent of the driver link
- **WHEN** `STEER_POS` is sent
- **THEN** its validity gate SHALL be the position sensor's own — the sensor reading healthy AND the
  steering calibrated — and SHALL NOT be the steering driver's link health
- **AND** `STEER_POS` SHALL carry a live position while `STEER_A`, `VESC_V` and `VESC_TEMP` are
  `NaN` and `VESC_OK` is `0.0`, because a silent driver does not blind the shaft sensor
- **AND** `STEER_POS` SHALL be `NaN` while those same values are live, whenever the sensor is
  unhealthy or the steering is uncalibrated
- **AND** `0.0` SHALL be reported as a REAL reading meaning "straight ahead"; an unknown position
  SHALL NEVER be reported as `0.0`
- **AND** a consumer SHALL NOT infer the state of either subsystem from the other

#### Scenario: Keep sending while the VESC is down
- **WHEN** the steering driver is offline for any length of time, including from boot
- **THEN** all six names SHALL keep being sent at their normal rates, carrying `NaN` and
  `VESC_OK` = `0.0` as appropriate, with `STEER_SCA` unaffected because its source is the vehicle
  layer and not the VESC
- **AND** transmission SHALL NOT be suppressed, because silence would make "this component is alive
  and its driver is down" indistinguishable from "this component is gone", and the two require
  different operator responses
- **AND** this SHALL remain the deliberate opposite of the external-navigation policy, where a
  fusable measurement goes silent rather than sending a doubtful value

#### Scenario: Names are unique within the ten-character field
- **WHEN** a named value is packed
- **THEN** the name SHALL be zero-padded into a ten-byte buffer before packing, because the wire
  field is a fixed ten bytes and is copied in full
- **AND** every name SHALL be a compile-time literal of at most ten characters, checked at compile
  time; names SHALL NOT be constructed at runtime, because truncation of an over-long name would be
  silent
- **AND** all six names SHALL be distinct within their first ten characters
- **AND** a consumer SHALL trim the name by length rather than by a terminator, because a
  ten-character name carries no terminator

#### Scenario: Scoped exception to the single-message rule
- **WHEN** vehicle telemetry is reported
- **THEN** engine RPM, coolant temperature, intake air temperature, manifold pressure, ECU load,
  measured and commanded throttle, module voltage, both gear values, the digital-output bitmask, the
  total odometer and the trip distance SHALL continue to be carried in distinct fields of the single
  `EFI_STATUS` message and SHALL NOT be sent as `NAMED_VALUE_FLOAT`, because those messages share one
  message id and collide by name in any name-agnostic store
- **AND** the six steering VESC, steering-position and steering-scale values named above SHALL be
  the ONLY exception, and the ONLY `NAMED_VALUE_FLOAT` this component sends
- **AND** the exception SHALL be justified by the standard ESC telemetry messages being undecodable
  at the ground station rather than by preference, and SHALL be documented at the point where the
  messages are packed
- **AND** a consumer SHALL filter on this component's id, because message id 251 may be sent by
  other components on the same link

#### Scenario: The reported voltage is not the ECU module voltage
- **WHEN** both `VESC_V` and `EFI_STATUS` are sent in the same interval tick
- **THEN** `VESC_V` SHALL be the VESC-measured steering supply rail
- **AND** `EFI_STATUS.ignition_voltage` SHALL remain the ECU control module supply voltage
  (OBD-II PID `0x42`)
- **AND** the two SHALL NOT be treated as the same measurement, because they are different physical
  quantities on different rails

#### Scenario: Telemetry is a reporting path only
- **WHEN** any of the six named values is at any value, including `NaN` or `VESC_OK` = `0.0`
- **THEN** no steering stop, stall latch, interlock, fail-safe or other control action SHALL be
  triggered by the act of reporting it
- **AND** the existing fault-code stop and communication fail-safe inside the steering controller
  SHALL remain the only consumers of that data for control purposes
- **AND** removing or changing this reporting path SHALL NOT alter any control behaviour

### Requirement: Autopilot Flight-Mode Awareness
The system SHALL decode the learned autopilot's `HEARTBEAT` and expose its flight mode, so that
behaviour which is only correct in MANUAL can be gated on the mode the autopilot is actually in.
The mode SHALL be observed only; the system SHALL NOT command, request or change it.

#### Scenario: Store the mode from the autopilot's heartbeat
- **WHEN** a `HEARTBEAT` arrives from the learned autopilot's system and component id
- **THEN** its `base_mode` and `custom_mode` fields SHALL be decoded and stored together with the
  time of receipt
- **AND** the heartbeat SHALL continue to serve its existing purposes — learning the autopilot and
  driving the link-loss timer — unchanged

#### Scenario: Ignore a heartbeat from any other component
- **WHEN** a `HEARTBEAT` arrives whose system or component id is not the learned autopilot's
- **THEN** the stored mode SHALL be left unchanged
- **AND** a ground station or any other component sharing the link SHALL therefore be unable to
  make the vehicle believe it is in a different mode

#### Scenario: Expose the mode and its availability
- **WHEN** a consumer asks for the autopilot's flight mode
- **THEN** the system SHALL report the stored `custom_mode`, and separately whether that value is
  currently trustworthy
- **AND** the value SHALL be reported as untrustworthy when no heartbeat has been received since
  initialisation, or when the link is down
- **AND** for an ArduPilot Rover a `custom_mode` of zero SHALL mean MANUAL

#### Scenario: Reset the mode on re-initialisation
- **WHEN** the MAVLink interface is initialised
- **THEN** the stored mode SHALL be cleared to "never received"
- **AND** consumers SHALL see the mode as unavailable until the next heartbeat from the autopilot

### Requirement: Autopilot Steering Speed-Scaling Base Parameter Subscription
The system SHALL subscribe to the autopilot's `MOT_SPD_SCA_BASE` parameter over MAVLink, as the
fallback source of the steering speed-scaling base. The subscription SHALL reuse the existing
read-only parameter-subscription mechanism rather than duplicating it, and SHALL NOT change the
behaviour of the `SPEED_MAX` subscription that mechanism already serves. The subscription SHALL be
read-only: the system SHALL NOT send `PARAM_SET` or otherwise write any autopilot parameter.

#### Scenario: Two parameters share one subscription mechanism
- **WHEN** the parameter subscription runs
- **THEN** `SPEED_MAX` and `MOT_SPD_SCA_BASE` SHALL both be subscribed through the same poll,
  the same `PARAM_VALUE` handler and the same value-hygiene rules
- **AND** each parameter SHALL keep its own stored value, its own receipt time, its own poll
  timestamp and its own upper bound
- **AND** the existing `SPEED_MAX` accessors SHALL keep their current semantics exactly, so no
  existing consumer of the speed limit changes behaviour

#### Scenario: Poll the parameter periodically
- **WHEN** the autopilot's system and component ids have been learned and the link is up
- **THEN** a `PARAM_REQUEST_READ` for `MOT_SPD_SCA_BASE` SHALL be sent with a parameter index of
  `-1` (look up by name), addressed to the learned autopilot
- **AND** the first request SHALL be sent `MAVLINK_PARAM_FIRST_DELAY_MS` after the autopilot was
  learned and subsequent requests every `MAVLINK_PARAM_POLL_MS`, as for `SPEED_MAX`
- **AND** the requests for the two parameters SHALL NOT be emitted in the same loop iteration, so
  two `PARAM_REQUEST_READ` frames do not contend for the same window on a 115200-baud link
- **AND** polling SHALL NOT stop once a value has been received, because the poll — not any
  broadcast — is what detects a change made from the ground station

#### Scenario: Apply the same value hygiene
- **WHEN** a `PARAM_VALUE` for `MOT_SPD_SCA_BASE` arrives
- **THEN** it SHALL be accepted only from the learned autopilot's system and component id
- **AND** the 16-byte `param_id` SHALL be copied into a 17-byte buffer and terminated before any
  string comparison
- **AND** a value that is not-a-number, infinite, negative, or greater than
  `MAVLINK_PARAM_SPD_SCA_BASE_MAX` SHALL be rejected with a log and SHALL NOT be stored
- **AND** a rejected value SHALL leave the previously stored value and its timestamp unchanged, so
  one corrupt frame cannot invalidate a good subscription

#### Scenario: Store and expose an accepted value
- **WHEN** a `PARAM_VALUE` for `MOT_SPD_SCA_BASE` passes every check
- **THEN** the value SHALL be stored in metres per second together with the time of receipt
- **AND** the system SHALL expose it in metres per second together with the age of the reading
- **AND** a change SHALL be logged only when the new value differs from the previous one by more
  than `MAVLINK_PARAM_EPSILON_MS`, so a five-second poll of an unchanged parameter does not fill
  the console

#### Scenario: Report the parameter as unavailable when it cannot be trusted
- **WHEN** the vehicle layer asks whether a `MOT_SPD_SCA_BASE` value is available
- **THEN** the answer SHALL be true only if a value has been received, the value is greater than
  zero, the link is up, and the reading is younger than `MAVLINK_PARAM_STALE_MS`
- **AND** a value of zero SHALL be reported as unavailable, because a non-positive base means "no
  scaling" both to ArduPilot and to this vehicle
- **AND** "unavailable" SHALL result in the vehicle layer falling back to its own stored base, or
  to no scaling at all when it has none

#### Scenario: Surface the subscription state on the diagnostic line
- **WHEN** the periodic (1 Hz) MAVLink debug line is emitted
- **THEN** it SHALL include the last received `MOT_SPD_SCA_BASE`, the age of that reading and
  whether the value is currently considered available
- **AND** it SHALL also include the autopilot's stored flight mode, so "not scaling because the
  autopilot is not in MANUAL" can be distinguished from "not scaling because there is no base" on
  the serial console alone

### Requirement: Steering Scale Telemetry via NAMED_VALUE_FLOAT
The system SHALL report the steering speed-scaling factor it is currently applying to the MAVLink
network as a `NAMED_VALUE_FLOAT` (id 251) named `STEER_SCA`, sent from this component. This name
SHALL be admitted under the same scoped exception that admits the steering VESC values, and the
exception SHALL remain closed to every other value: engine, gear, digital-flag, odometer and trip
values SHALL continue to be carried in distinct fields of the single `EFI_STATUS` message.
`STEER_SCA` SHALL NOT be a control input.

#### Scenario: Report the applied scale at 1 Hz
- **WHEN** the slow steering telemetry interval elapses (`MAVLINK_STEER_SLOW_TX_MS`, default
  1000 ms)
- **THEN** a `NAMED_VALUE_FLOAT` named `STEER_SCA` SHALL be sent from the ESP32's own system and
  component id (system 1, component 25), alongside `VESC_TEMP` and `VESC_OK`
- **AND** its value SHALL be the scale actually applied to the steering command after rate
  limiting, in the range 0 to 1 inclusive
- **AND** its `time_boot_ms` SHALL carry the milliseconds-since-boot value already computed for
  that tick

#### Scenario: One means "not scaling", and it is never NaN
- **WHEN** the steering scaling is inactive for any reason — no base value, an invalid speed
  reading, the autopilot not in MANUAL, or a speed at or below the base
- **THEN** `STEER_SCA` SHALL be reported as `1.0`
- **AND** it SHALL NEVER be `NaN`, because "not scaling" is a definite answer and a ground station
  should plot it as a flat line at one rather than as a gap in the record

#### Scenario: The value reflects the command path that is actually in use
- **WHEN** the active command source is not the autopilot
- **THEN** `STEER_SCA` SHALL be `1.0`, matching the fact that the web steering path is never scaled
- **AND** the message SHALL keep being sent regardless of the command source, so the record has no
  holes

