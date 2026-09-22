# can-controller Specification

## Purpose
TBD - created by archiving change add-can-controller-class. Update Purpose after archive.
## Requirements
### Requirement: CAN Controller OBD-II Data Reading
The system SHALL interface with vehicle CAN bus using MCP2515 SPI controller to read standard OBD-II diagnostic data using a non-blocking state machine.
The RPM poll interval SHALL be adjustable at runtime via `setRPMPollInterval(uint32_t ms)` to allow higher-frequency polling during gear changes.
The active poll table SHALL contain seven PIDs: engine RPM (`0x0C`) in the fast class, and coolant temperature (`0x05`), throttle position (`0x11`), manifold absolute pressure (`0x0B`), control module voltage (`0x42`), intake air temperature (`0x0F`) and calculated engine load (`0x04`) in the `CAN_POLL_INTERVAL_TEMP` class. Exactly one OBD-II request SHALL be in flight at any time, and the RPM poll cadence SHALL remain unchanged by the additional PIDs.

#### Scenario: Initialize MCP2515 on startup
**Given** the ESP32 is powered on
**When** `CANController::begin()` is called with SPI pins
**Then** the MCP2515 shall be initialized at 500 kbps
**And** SPI communication shall be verified
**And** the PID scheduling table shall be initialized with polling intervals (RPM: `CAN_POLL_INTERVAL_RPM` ms; coolant, throttle position, MAP, module voltage, intake air temperature and engine load: `CAN_POLL_INTERVAL_TEMP` ms)
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
**When** the RPM PID becomes due (current interval) and is selected by the scheduler
**Then** an OBD-II Mode 01 request for PID 0x0C shall be sent
**And** the response shall be parsed to extract RPM value
**And** RPM shall be calculated as `((A*256)+B)/4`
**And** the value shall be stored in `VehicleData.engineRPM`

#### Scenario: Read engine coolant temperature via OBD-II PID 0x05
**Given** the CAN controller is initialized
**When** the coolant temp PID becomes due (1000ms interval) and is selected by the scheduler
**Then** an OBD-II Mode 01 request for PID 0x05 shall be sent
**And** the response shall be parsed to extract temperature value
**And** temperature shall be calculated as `A - 40` (°C)
**And** the value shall be stored in `VehicleData.coolantTemp`

#### Scenario: Read throttle position via OBD-II PID 0x11
**Given** the CAN controller is initialized
**When** the throttle position PID becomes due (1000ms interval) and is selected by the scheduler
**Then** an OBD-II Mode 01 request for PID 0x11 shall be sent
**And** throttle position shall be calculated as `A * 100 / 255` (percentage, 0–100%)
**And** the value shall be stored in `VehicleData.throttlePosition`

#### Scenario: Read manifold absolute pressure via OBD-II PID 0x0B
**Given** the CAN controller is initialized
**When** the MAP PID becomes due (`CAN_POLL_INTERVAL_TEMP`, 2000ms interval) and is selected by the scheduler
**Then** an OBD-II Mode 01 request for PID 0x0B shall be sent
**And** the response shall be parsed to extract manifold absolute pressure
**And** pressure shall be calculated as `A` (kPa absolute, 0–255 kPa)
**And** the value shall be stored in `VehicleData.mapKpa`

#### Scenario: Read control module voltage via OBD-II PID 0x42
**Given** the CAN controller is initialized
**When** the module voltage PID becomes due (`CAN_POLL_INTERVAL_TEMP`, 2000ms interval) and is selected by the scheduler
**Then** an OBD-II Mode 01 request for PID 0x42 shall be sent
**And** the response shall be parsed to extract the control module supply voltage
**And** the raw value `((A*256)+B)` SHALL be stored verbatim as millivolts in `VehicleData.moduleVoltageMv`
**And** consumers SHALL convert to volts as `((A*256)+B)/1000` (0.000–65.535 V) at the presentation edge

#### Scenario: Read intake air temperature via OBD-II PID 0x0F
**Given** the CAN controller is initialized
**When** the intake air temperature PID becomes due (`CAN_POLL_INTERVAL_TEMP`, 2000ms interval) and is selected by the scheduler
**Then** an OBD-II Mode 01 request for PID 0x0F shall be sent
**And** temperature shall be calculated as `A - 40` (°C, -40 to +215)
**And** the value shall be stored in `VehicleData.intakeTemp`

#### Scenario: Read calculated engine load via OBD-II PID 0x04
**Given** the CAN controller is initialized
**When** the engine load PID becomes due (`CAN_POLL_INTERVAL_TEMP`, 2000ms interval) and is selected by the scheduler
**Then** an OBD-II Mode 01 request for PID 0x04 shall be sent
**And** engine load shall be calculated as `A * 100 / 255` (percentage, 0–100%)
**And** the value shall be stored in `VehicleData.engineLoad`

#### Scenario: PIDs confirmed unsupported by the ECU are not polled
**Given** the 2026-08-14 ECU capability probe confirmed that fuel tank level (`0x2F`) and engine oil temperature (`0x5C`) produce no response
**When** the poll table is constructed
**Then** neither `0x2F` nor `0x5C` SHALL be present in the poll table
**And** no request slot SHALL be spent on them
**And** PIDs that the probe reported as supported but that have no consumer (for example `0x0E` timing advance, `0x14` O2 sensor B1S1, and the fuel-trim PIDs) SHALL also be omitted, to keep the request budget small

#### Scenario: Additional PIDs do not disturb the RPM poll cadence
**Given** the poll table contains seven PIDs with RPM in the fast class and six PIDs at `CAN_POLL_INTERVAL_TEMP`
**When** the scheduler runs in steady state
**Then** the RPM PID SHALL continue to be polled at its configured interval (`CAN_POLL_INTERVAL_RPM`, or `CAN_POLL_INTERVAL_RPM_BOOST` during a gear change)
**And** an RPM request SHALL be delayed by at most one in-flight transaction (bounded by the 200 ms response timeout)
**And** the added PIDs SHALL NOT cause `VehicleData.dataValid` to be lost through the stale-data timeout

#### Scenario: PID scheduling prioritizes most overdue requests
**Given** multiple PIDs are due for polling
**When** the state machine enters IDLE
**Then** the PID with the largest overdue time SHALL be selected first
**And** high-frequency PIDs (RPM at reduced interval) naturally receive priority over low-frequency PIDs (temps at 1000ms)

#### Scenario: Change RPM poll interval at runtime
**Given** the CAN controller is initialized
**When** `setRPMPollInterval(50)` is called
**Then** the RPM PID entry interval SHALL be updated to 50ms
**And** subsequent RPM polls SHALL occur at the new rate
**When** `setRPMPollInterval(CAN_POLL_INTERVAL_RPM)` is called to restore normal rate
**Then** the RPM PID entry interval SHALL revert to the default value

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
The transmission system SHALL use vehicle speed data from the hall-effect speed sensor (not the CAN
bus) to prevent unsafe gear changes while the vehicle is in motion. When the sensor has never
produced a reading (uninitialized), the system SHALL fall back to a fail-safe policy that mirrors
the previous CAN-timeout behavior. While the sensor is flagged suspicious, its decaying reading is
a physical upper bound on speed and the interlock SHALL keep using it.

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
- **WHEN** the hall sensor reading is invalid or uninitialized
- **AND** a gear change is requested
- **THEN** the system SHALL fall back to the existing fail-safe policy and allow the gear change
- **AND** a warning shall be logged indicating the speed reading was unavailable

#### Scenario: Interlock holds on a suspicious reading
- **WHEN** the hall sensor is flagged suspicious after pulses vanished mid-motion
- **AND** a gear change is requested
- **THEN** the interlock SHALL compare the decaying reading against the threshold as if the sensor
  were valid
- **AND** the change SHALL be blocked until the reading falls below the threshold, pulses resume, or
  the stale timeout zeroes it

#### Scenario: Timeout fallback when CAN data is unavailable
**Given** CAN data was last updated 6000ms ago
**And** the hall sensor reading is also unavailable
**And** a gear change is requested
**When** the timeout threshold (5000ms) is exceeded
**Then** the gear change shall be allowed (fail-safe override)
**And** a warning shall be logged: "CAN timeout, allowing gear change"

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

### Requirement: Dynamic RPM Poll Rate for Gear Changes
The system SHALL support a high-frequency RPM polling mode during active gear changes to provide the PID controller with timely feedback.

#### Scenario: Increase RPM poll rate when gear change starts
**Given** a gear change is initiated (`transmission_.needsThrottleBoost()` becomes true)
**When** the gear boost PID is activated in `VehicleController`
**Then** `setRPMPollInterval(CAN_POLL_INTERVAL_RPM_BOOST)` SHALL be called on the CAN controller
**And** RPM values SHALL be read at the boost poll rate (e.g. 50ms)

#### Scenario: Restore default RPM poll rate when gear change ends
**Given** a gear change has completed or timed out
**When** the gear boost PID is deactivated
**Then** `setRPMPollInterval(CAN_POLL_INTERVAL_RPM)` SHALL be called on the CAN controller
**And** RPM polling SHALL revert to the default rate (200ms)

### Requirement: ECU Capability Probe
The system SHALL provide an on-demand, web-triggered ECU capability probe that discovers which
OBD-II PIDs the ECU answers and reads stored diagnostic trouble codes, without permanently
altering normal polling behavior. The probe SHALL reuse the existing non-blocking OBD-II send and
receive helpers, keep at most one request in flight at a time, and complete within a bounded
duration so that engine RPM does not become stale during the probe.

#### Scenario: Web-triggered probe walks bitmaps, candidate PIDs, and reads DTCs
- **WHEN** a `can_probe` request is received and CAN data is valid and no gear change is active
- **THEN** the controller SHALL enter probe mode and suspend normal PID polling
- **AND** SHALL send a Mode 01 request for PID 0x00 and record the 4-byte supported-PID bitmap
- **AND** SHALL request the next bitmap group (0x20, then 0x40, then 0x60) only when the previous
  group's continuation bit indicates more PIDs are supported
- **AND** SHALL send one Mode 01 test request per candidate PID (0x04, 0x0B, 0x0E, 0x0F, 0x42,
  0x0D, 0x2F, 0x5C, 0x14) regardless of what the bitmaps report
- **AND** SHALL send one Mode 03 request and record the reported DTC count and codes
- **AND** each request SHALL use the 200 ms response timeout with a single retry
- **AND** SHALL store, per candidate PID, whether it was reported supported by a bitmap, whether
  it answered, its raw response bytes, and its decoded value when a known formula exists

#### Scenario: Probe suspends then resumes normal polling
- **WHEN** the probe sequence completes (all bitmap groups, candidate PIDs, and the Mode 03
  request have been answered or timed out)
- **THEN** the controller SHALL mark the results as complete with a completion timestamp
- **AND** SHALL return to normal mode and resume the existing IDLE/WAITING_RESPONSE polling
  machine from where it left off
- **AND** normal `VehicleData` values SHALL not have been overwritten by the probe

#### Scenario: Disconnected ECU short-circuits the probe
- **WHEN** a `can_probe` request is received and `VehicleData.dataValid` is false
- **THEN** the controller SHALL NOT generate any bus traffic
- **AND** SHALL immediately produce a completed result indicating "no ECU / disconnected"
- **AND** SHALL remain in normal mode

#### Scenario: Probe deferred while a gear change is active
- **WHEN** a `can_probe` request is received while a gear change is active (throttle boost /
  position control in progress)
- **THEN** the probe SHALL be rejected/deferred rather than suspending polling mid-shift
- **AND** the result SHALL indicate a busy/deferred status
- **AND** high-frequency RPM polling for the gear change SHALL be unaffected

#### Scenario: Partial responses and multi-frame DTCs are recorded, never block
- **WHEN** a bitmap group, candidate PID, or the Mode 03 request receives no response within the
  timeout and its retry
- **THEN** that entry SHALL be recorded as not answered and the sequencer SHALL advance
- **AND** if the Mode 03 count implies more DTCs than fit in a single classic frame, the results
  SHALL set a truncation flag and only the single-frame codes SHALL be reported (multi-frame
  ISO-TP reassembly is a documented limitation)

#### Scenario: Probe progress and results mirrored to serial log
- **WHEN** the probe runs and completes
- **THEN** per-PID answered/raw/decoded results and the DTC summary SHALL be printed to the serial
  console via `Debug::printfFeature(DebugFeature::CAN, ...)`

