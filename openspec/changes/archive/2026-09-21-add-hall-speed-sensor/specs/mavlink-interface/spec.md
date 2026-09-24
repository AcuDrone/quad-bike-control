## MODIFIED Requirements

### Requirement: Vehicle State Reporting via Standard MAVLink Messages
The system SHALL report vehicle state back to the MAVLink network using standard MAVLink messages, on
a fixed schedule. Engine data is sourced from the CAN `VehicleData`; vehicle ground speed is sourced
from the hall-effect speed sensor and reported via `VFR_HUD`.

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
- **AND** `VFR_HUD` ground-speed reporting is governed by the hall sensor's own validity, independent
  of CAN validity
