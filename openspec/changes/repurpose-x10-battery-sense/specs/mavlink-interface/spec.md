## MODIFIED Requirements
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
  measured and commanded throttle, the 12 V battery voltage, both gear values, the total odometer, the
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
- **WHEN** the engine-telemetry interval elapses, whatever the CAN health
- **THEN** the `ignition_voltage` field SHALL carry the calibrated 12 V battery voltage measured on
  X10 / GPIO9, in volts, while the battery sense is valid (a battery is seen on X10), and `NaN`
  otherwise
- **AND** it SHALL NOT be sourced from OBD-II PID `0x42`, which the Delphi MT05 ECU never answers

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
  pressure and the 12 V supply voltage respectively (the latter now measured by the board on X10
  rather than requested from the ECU)
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
  `intake_manifold_pressure`, `engine_load` and `throttle_position` fields SHALL ALL be reported as
  `NaN`, so a ground station shows "no data" rather than misleading zeros
- **AND** `ignition_voltage` SHALL NOT depend on CAN health: it follows the battery sense's own
  validity (calibrated volts while a battery is seen on X10, `NaN` otherwise)
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
  lock-to-lock range, −100 at the left limit, 0 at centre (the steering sensor's zero), +100 at the
  right limit — percent of calibrated travel, NOT degrees, so that the autopilot sees the same
  scale it commands on `SERVO_OUTPUT_RAW` even though the sensor itself reports degrees
- **AND** a negative `STEER_POS` SHALL mean LEFT of centre and a positive value SHALL mean RIGHT of
  centre, matching the sign convention of the steering-percent accessor
- **AND** `STEER_A` SHALL carry the VESC's average MOTOR current in amperes, which is the steering
  load and stall diagnostic; the VESC's average INPUT current SHALL NOT be reported under this name
- **AND** `VESC_V` SHALL carry the input voltage measured by the VESC itself — the 24 V VESC supply
  (the 7s LiFePO4 pack on aztec) that powers the steering driver — in volts
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
- **THEN** its validity gate SHALL be the position sensor's own — the CAN steering-angle sensor
  healthy (fault token `none`) AND both steering limits stored — and SHALL NOT be the steering
  driver's link health
- **AND** `STEER_POS` SHALL be `NaN` while the sensor reports `range_latched`, `timeout`, `zeroing`
  or any other non-`none` token, even if a jog is driving the steering sensorless
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
- **THEN** `VESC_V` SHALL be the VESC-measured steering supply
- **AND** `EFI_STATUS.ignition_voltage` SHALL be the 12 V battery voltage measured on X10 / GPIO9
- **AND** the two SHALL NOT be treated as the same measurement, because they are different physical
  quantities on different rails

#### Scenario: Telemetry is a reporting path only
- **WHEN** any of the six named values is at any value, including `NaN` or `VESC_OK` = `0.0`
- **THEN** no steering stop, stall latch, interlock, fail-safe or other control action SHALL be
  triggered by the act of reporting it
- **AND** the existing fault-code stop and communication fail-safe inside the steering controller
  SHALL remain the only consumers of that data for control purposes
- **AND** removing or changing this reporting path SHALL NOT alter any control behaviour
