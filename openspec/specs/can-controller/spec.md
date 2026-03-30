# can-controller Specification

## Purpose
TBD - created by archiving change add-can-controller-class. Update Purpose after archive.
## Requirements
### Requirement: CAN Controller OBD-II Data Reading
The system SHALL interface with vehicle CAN bus using MCP2515 SPI controller to read standard OBD-II diagnostic data using a non-blocking state machine.

#### Scenario: Initialize MCP2515 on startup
**Given** the ESP32 is powered on
**When** `CANController::begin()` is called with SPI pins
**Then** the MCP2515 shall be initialized at 500 kbps
**And** SPI communication shall be verified
**And** the PID scheduling table shall be initialized with polling intervals (RPM/speed: 100ms, temperatures/fuel: 1000ms)
**And** the state machine shall be set to IDLE
**And** return `true` if successful, `false` otherwise

#### Scenario: Non-blocking OBD-II polling via state machine
**Given** the CAN controller is initialized
**When** `update()` is called
**Then** if state is IDLE and a PID is due for polling, an OBD-II request SHALL be sent and state transitions to WAITING_RESPONSE
**And** if state is WAITING_RESPONSE, a single non-blocking `checkReceive()` SHALL be performed
**And** if a matching response is available, it SHALL be parsed and stored, and state transitions to IDLE
**And** if the response timeout (200ms) elapses, the request SHALL be marked as failed and state transitions to IDLE
**And** `update()` SHALL return immediately without blocking the main loop

#### Scenario: Read engine RPM via OBD-II PID 0x0C
**Given** the CAN controller is initialized
**When** the RPM PID becomes due (100ms interval) and is selected by the scheduler
**Then** an OBD-II Mode 01 request for PID 0x0C shall be sent
**And** the response shall be parsed to extract RPM value
**And** RPM shall be calculated as `((A*256)+B)/4`
**And** the value shall be stored in `VehicleData.engineRPM`

#### Scenario: Read vehicle speed via OBD-II PID 0x0D
**Given** the CAN controller is initialized
**When** the speed PID becomes due (100ms interval) and is selected by the scheduler
**Then** an OBD-II Mode 01 request for PID 0x0D shall be sent
**And** the response shall be parsed to extract speed value
**And** speed shall be stored in `VehicleData.vehicleSpeed` (km/h)

#### Scenario: Read engine coolant temperature via OBD-II PID 0x05
**Given** the CAN controller is initialized
**When** the coolant temp PID becomes due (1000ms interval) and is selected by the scheduler
**Then** an OBD-II Mode 01 request for PID 0x05 shall be sent
**And** the response shall be parsed to extract temperature value
**And** temperature shall be calculated as `A - 40` (°C)
**And** the value shall be stored in `VehicleData.coolantTemp`

#### Scenario: Read fuel tank level via OBD-II PID 0x2F
**Given** the CAN controller is initialized
**When** the fuel level PID becomes due (1000ms interval) and is selected by the scheduler
**Then** an OBD-II Mode 01 request for PID 0x2F shall be sent
**And** the response shall be parsed to extract fuel level
**And** fuel level shall be calculated as `A * 100 / 255` (percentage, 0–100%)
**And** the value shall be stored in `VehicleData.fuelLevel`

#### Scenario: PID scheduling prioritizes most overdue requests
**Given** multiple PIDs are due for polling
**When** the state machine enters IDLE
**Then** the PID with the largest overdue time SHALL be selected first
**And** high-frequency PIDs (RPM, speed at 100ms) naturally receive priority over low-frequency PIDs (temps, fuel at 1000ms)

### Requirement: CAN Communication Error Handling
The system SHALL detect and recover from CAN communication errors without blocking the main control loop.

#### Scenario: Handle OBD-II response timeout
**Given** an OBD-II request has been sent
**When** 200ms elapses without a matching response
**Then** the per-PID retry counter shall be incremented
**And** the state machine shall transition to IDLE to process the next PID
**And** the main loop SHALL NOT be blocked during the timeout period
**And** a warning shall be logged to Serial

#### Scenario: Detect stale CAN data
**Given** CAN data was previously valid
**When** 5000ms elapses without successful updates
**Then** `VehicleData.dataValid` shall be set to `false`
**And** dependent systems shall use fallback behavior

#### Scenario: Recover from MCP2515 hardware error
**Given** the MCP2515 reports a hardware error
**When** the error is detected
**Then** the controller shall attempt to reset the MCP2515
**And** retry up to 3 times before marking CAN as failed
**And** log error details to Serial

#### Scenario: Drain MCP2515 receive buffers
**Given** the state machine is in WAITING_RESPONSE
**When** CAN messages are available
**Then** up to 5 messages SHALL be read per `update()` call to prevent MCP2515 RX buffer overflow
**And** non-matching messages SHALL be discarded
**And** the first matching response SHALL be accepted and parsed

### Requirement: Speed-Based Gear Change Prevention
The transmission system SHALL use vehicle speed data to prevent unsafe gear changes while the vehicle is in motion.

#### Scenario: Block gear change when speed exceeds threshold
**Given** the vehicle speed is 10 km/h
**And** CAN data is valid
**When** a gear change to LOW is requested
**Then** the gear change shall be blocked
**And** a warning message shall be logged: "Gear change blocked: vehicle moving"
**And** the current gear shall remain unchanged

#### Scenario: Allow gear change when vehicle is stopped
**Given** the vehicle speed is 0 km/h
**And** CAN data is valid
**When** a gear change to LOW is requested
**Then** the gear change shall be allowed
**And** the transmission shall move to LOW gear

#### Scenario: Allow gear change to NEUTRAL regardless of speed
**Given** the vehicle speed is 20 km/h
**And** CAN data is valid
**When** a gear change to NEUTRAL is requested
**Then** the gear change shall be allowed (safety override)
**And** the transmission shall move to NEUTRAL

#### Scenario: Timeout fallback when CAN data is unavailable
**Given** CAN data was last updated 6000ms ago
**And** a gear change is requested
**When** the timeout threshold (5000ms) is exceeded
**Then** the gear change shall be allowed (fail-safe override)
**And** a warning shall be logged: "CAN timeout, allowing gear change"

---

### Requirement: Throttle Boost During Gear Changes
The system SHALL temporarily increase engine throttle during gear changes to maintain RPM and enable smoother shifts.

#### Scenario: Apply throttle boost during gear transition
**Given** a gear change from NEUTRAL to LOW is in progress
**When** the transmission actuator is moving
**Then** throttle shall be increased to 20%
**And** the boost shall be maintained for the duration of the gear change
**And** the boost shall not exceed 500ms maximum

#### Scenario: Release throttle boost after gear change completes
**Given** throttle boost is active at 20%
**When** the gear change completes (actuator reaches target position)
**Then** throttle shall return to commanded value from SBUS or web input
**And** the boost shall be disabled

#### Scenario: Disable throttle boost if brake is applied
**Given** throttle boost is active at 20%
**When** brake is applied > 10%
**Then** throttle boost shall be immediately disabled
**And** throttle shall return to 0% (idle)
**And** the gear change shall continue normally

---

### Requirement: CAN Data Telemetry Broadcasting
The web portal SHALL display real-time vehicle data from the CAN bus to provide visibility into engine status.

#### Scenario: Include vehicle data in WebSocket telemetry
**Given** CAN data is valid
**And** a WebSocket client is connected
**When** the telemetry broadcast interval (200ms) elapses
**Then** the telemetry JSON shall include:
```json
{
    "engineRPM": 2500,
    "vehicleSpeed": 15,
    "coolantTemp": 85,
    "oilTemp": 90,
    "throttlePosition": 25,
    "canStatus": "connected"
}
```

#### Scenario: Indicate CAN disconnected status in telemetry
**Given** CAN data is invalid
**And** a WebSocket client is connected
**When** the telemetry broadcast interval elapses
**Then** the telemetry JSON shall include:
```json
{
    "canStatus": "disconnected",
    "canDataAge": 5234
}
```
**And** vehicle data fields shall be omitted or set to null

---

### Requirement: CAN Status and Diagnostics
The system SHALL expose CAN controller status for debugging and monitoring.

#### Scenario: Report CAN connection status
**Given** the CAN controller is initialized
**When** `isConnected()` is called
**Then** it shall return `true` if MCP2515 is responding
**And** return `false` if hardware is not responding

#### Scenario: Provide human-readable status string
**Given** the CAN controller is running
**When** `getStatusString()` is called
**Then** it shall return a status string like:
- "Connected - 10 Hz" (when healthy)
- "Disconnected - timeout" (when failing)
- "Error - MCP2515 not responding" (when hardware fails)

#### Scenario: Log CAN initialization on startup
**Given** the ESP32 is booting
**When** `CANController::begin()` is called
**Then** initialization status shall be logged:
```
[CAN] Initializing MCP2515 on SPI...
[CAN] MCP2515 initialized at 500 kbps
[CAN] Ready to read vehicle data
```
**Or** if failed:
```
[CAN] ERROR: MCP2515 initialization failed
```

