## MODIFIED Requirements

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
