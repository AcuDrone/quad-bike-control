## MODIFIED Requirements

### Requirement: Vehicle State Reporting via Standard MAVLink Messages
The system SHALL report vehicle state back to the MAVLink network using standard MAVLink messages, on
a fixed schedule. Engine data is sourced from the CAN `VehicleData`; vehicle ground speed is sourced
from the hall-effect speed sensor and reported via `VFR_HUD`. Every ECU value carried in
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
  measured and commanded throttle, module supply voltage, both gear values, the digital-output
  bitmask and a health flag
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

#### Scenario: Report measured and commanded throttle in EFI_STATUS
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

#### Scenario: Repurpose only permanently-free fields
- **WHEN** a value is carried in an `EFI_STATUS` field that does not name it
- **THEN** the only such fields SHALL be `fuel_consumed` and `fuel_flow` (carrying the two gear
  values) and `pt_compensation` (carrying the digital-output bitmask)
- **AND** the fuel fields SHALL be justified by the bench-confirmed permanent absence of the
  corresponding ECU signals on this vehicle (PIDs `0x2F` fuel level and `0x5C` oil temperature do
  not answer and are absent from the supported-PID bitmaps), so no genuine fuel quantity or flow can
  ever be displaced
- **AND** fields naming engine quantities this ECU could plausibly expose later — `spark_dwell_time`,
  `barometric_pressure`, `ignition_timing`, `injection_time`, `exhaust_gas_temperature`,
  `fuel_pressure` — SHALL be left unused rather than repurposed
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
- **THEN** a `VFR_HUD` message SHALL be sent with `groundspeed` set to the sensor speed converted to
  metres per second
- **AND** the GCS SHALL be able to display and graph it as ground speed for the ground rover
- **WHEN** the hall sensor reading is invalid or stale
- **THEN** `groundspeed` SHALL be reported as 0 (or the `VFR_HUD` message suppressed for that tick) so
  a stale reading is not presented as genuine motion

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
- **AND** the `fuel_consumed` (assumed gear), `throttle_out` (commanded throttle) and
  `pt_compensation` (digital-output bitmask) fields SHALL remain valid, because none of them depends
  on CAN health
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
