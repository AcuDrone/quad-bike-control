## ADDED Requirements

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

## MODIFIED Requirements

### Requirement: CAN Controller OBD-II Data Reading
The system SHALL interface with vehicle CAN bus using MCP2515 SPI controller to read standard OBD-II diagnostic data using a non-blocking state machine.
The RPM poll interval SHALL be adjustable at runtime via `setRPMPollInterval(uint32_t ms)` to allow higher-frequency polling during gear changes.

#### Scenario: Initialize MCP2515 on startup
**Given** the ESP32 is powered on
**When** `CANController::begin()` is called with SPI pins
**Then** the MCP2515 shall be initialized at 500 kbps
**And** SPI communication shall be verified
**And** the PID scheduling table shall be initialized with polling intervals (RPM: `CAN_POLL_INTERVAL_RPM` ms, temperatures: 1000ms, MAP: `CAN_POLL_INTERVAL_TEMP` ms)
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
