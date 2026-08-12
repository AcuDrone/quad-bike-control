## MODIFIED Requirements

### Requirement: Speed-Based Gear Change Prevention
The transmission system SHALL use vehicle speed data from the hall-effect speed sensor (not the CAN
bus) to prevent unsafe gear changes while the vehicle is in motion. When the sensor reading is not
valid, the system SHALL fall back to a fail-safe policy that mirrors the previous CAN-timeout
behavior.

#### Scenario: Block gear change when speed exceeds threshold
- **WHEN** the hall sensor reports a valid speed of 10 km/h
- **AND** a gear change to LOW is requested
- **THEN** the gear change shall be blocked
- **AND** a warning message shall be logged: "Gear change blocked: vehicle moving"
- **AND** the current gear shall remain unchanged

#### Scenario: Allow gear change when vehicle is stopped
- **WHEN** the hall sensor reports a valid speed of 0 km/h
- **AND** a gear change to LOW is requested
- **THEN** the gear change shall be allowed
- **AND** the transmission shall move to LOW gear

#### Scenario: Allow gear change to NEUTRAL regardless of speed
- **WHEN** the hall sensor reports a valid speed of 20 km/h
- **AND** a gear change to NEUTRAL is requested
- **THEN** the gear change shall be allowed (safety override)
- **AND** the transmission shall move to NEUTRAL

#### Scenario: Fail-safe fallback when speed reading is invalid
- **WHEN** the hall sensor reading is invalid, uninitialized, or flagged suspicious
- **AND** a gear change is requested
- **THEN** the system SHALL fall back to the existing fail-safe policy and allow the gear change
- **AND** a warning shall be logged indicating the speed reading was unavailable

#### Scenario: Timeout fallback when CAN data is unavailable
**Given** CAN data was last updated 6000ms ago
**And** the hall sensor reading is also unavailable
**And** a gear change is requested
**When** the timeout threshold (5000ms) is exceeded
**Then** the gear change shall be allowed (fail-safe override)
**And** a warning shall be logged: "CAN timeout, allowing gear change"

### Requirement: CAN Data Telemetry Broadcasting
The web portal SHALL display real-time vehicle data from the CAN bus to provide visibility into
engine status. Vehicle speed SHALL NOT be part of the CAN telemetry payload because speed is now
sourced from the hall-effect speed sensor and published independently (see the `speed-sensor`
capability).

#### Scenario: Include vehicle data in WebSocket telemetry
- **WHEN** CAN data is valid
- **AND** a WebSocket client is connected
- **AND** the telemetry broadcast interval elapses
- **THEN** the telemetry JSON shall include CAN-sourced engine fields, for example:
```json
{
    "engineRPM": 2500,
    "coolantTemp": 85,
    "oilTemp": 90,
    "throttlePosition": 25,
    "canStatus": "connected"
}
```
- **AND** `vehicleSpeed` SHALL NOT be emitted inside the CAN-connected block
- **AND** the hall-sensor `vehicle_speed` field SHALL instead be emitted independently of `can_status`

#### Scenario: Indicate CAN disconnected status in telemetry
- **WHEN** CAN data is invalid
- **AND** a WebSocket client is connected
- **When** the telemetry broadcast interval elapses
- **Then** the telemetry JSON shall include:
```json
{
    "canStatus": "disconnected",
    "canDataAge": 5234
}
```
- **AND** CAN vehicle data fields shall be omitted or set to null
- **AND** hall-sensor `vehicle_speed` SHALL still be emitted, unaffected by the CAN state
