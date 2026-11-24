# can-controller Specification

## Purpose
TBD - created by archiving change add-can-controller-class. Update Purpose after archive.
## Requirements
### Requirement: CAN Controller OBD-II Data Reading
The system SHALL interface with vehicle CAN bus using MCP2515 SPI controller to read standard OBD-II diagnostic data using a non-blocking state machine.
The RPM poll interval SHALL be adjustable at runtime via `setRPMPollInterval(uint32_t ms)` to allow higher-frequency polling during gear changes.
The active poll table SHALL contain seven PIDs: engine RPM (`0x0C`) in the fast class, and coolant temperature (`0x05`), throttle position (`0x11`), manifold absolute pressure (`0x0B`), control module voltage (`0x42`), intake air temperature (`0x0F`) and calculated engine load (`0x04`) in the `CAN_POLL_INTERVAL_TEMP` class. Exactly one OBD-II request SHALL be in flight at any time, and the RPM poll cadence SHALL remain unchanged by the additional PIDs.
The OBD-II state machine SHALL run in the main loop and SHALL receive its replies from the CAN receive queue and send its requests through the queued CAN transmit, never by accessing the MCP2515 directly.

#### Scenario: Initialize MCP2515 on startup
**Given** the ESP32 is powered on
**When** `CANController::begin()` is called with SPI pins
**Then** the CAN task SHALL be created and SHALL initialize the MCP2515 at 500 kbps
**And** SPI communication shall be verified
**And** the PID scheduling table shall be initialized with polling intervals (RPM: `CAN_POLL_INTERVAL_RPM` ms; coolant, throttle position, MAP, module voltage, intake air temperature and engine load: `CAN_POLL_INTERVAL_TEMP` ms)
**And** the state machine shall be set to IDLE
**And** `begin()` SHALL wait for the result of that first initialization for at most `CAN_BOOT_INIT_WAIT_MS` (1000 ms) and return `true` if it succeeded, `false` otherwise
**And** a `false` result SHALL NOT stop later initialization attempts (see "MCP2515 Initialization Retry")

#### Scenario: Non-blocking OBD-II polling via state machine
**Given** the CAN controller is initialized
**When** `update()` is called
**Then** if state is IDLE and a PID is due for polling, an OBD-II request SHALL be queued for transmission and state transitions to WAITING_RESPONSE
**And** if state is WAITING_RESPONSE, the frames taken from the receive queue in this call SHALL be checked for a matching response
**And** if a matching response is available, it SHALL be parsed and stored, and state transitions to IDLE
**And** if the response timeout (200ms) elapses, the request SHALL be marked as failed and state transitions to IDLE
**And** `update()` SHALL return immediately without blocking the main loop, and SHALL NOT access the MCP2515 over SPI

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
**And** every reset or re-initialization SHALL re-apply both acceptance paths (OBD-II window and steering-angle frame)
**And** the reset or re-initialization SHALL be executed by the CAN task, never inside `loop()`, and the main loop SHALL continue to run while it is in progress

#### Scenario: Error flags monitored in every state
**Given** the CAN controller is initialized, in any OBD-II state, in probe mode, during an LWS zero-set or while sniffing
**When** `CAN_EFLG_POLL_MS` (10 ms) elapses
**Then** the CAN task SHALL read EFLG, count and clear RX0OVR / RX1OVR, and keep a TEC / REC / EFLG snapshot for the health log
**And** a set TXBO SHALL be reported to the main loop once per bus-off episode, which SHALL journal `bus_off` and request the re-initialization as before

#### Scenario: Drain MCP2515 receive buffers
**Given** the CAN controller is initialized, in any OBD-II state (IDLE or WAITING_RESPONSE), in probe mode, during an LWS zero-set or while sniffing
**When** the CAN task has placed received frames in the receive queue
**Then** every `update()` call SHALL take all queued frames, at most `CAN_RX_RING` (64) per call, before running the OBD-II, probe, zero-set or sniffer step
**And** every standard-ID `0x305` message SHALL be dispatched to the steering-angle decoder and SHALL NOT be logged as unmatched
**And** while waiting for an OBD-II or probe response, the first matching response SHALL be accepted and parsed
**And** other non-matching messages SHALL be discarded and logged as unmatched at most once per `CAN_RX_LOG_INTERVAL`
**And** the MCP2515 receive buffers SHALL be read only by the CAN task

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

### Requirement: Shared-Bus Steering Angle Frame Reception
The MCP2515 SHALL accept the Bosch LWS steering-angle frame (`0x305`) through its own receive buffer alongside the OBD-II response window, so that the 100 Hz angle stream and engine replies on the one shared bus do not evict each other.

#### Scenario: Two acceptance paths
- **WHEN** the receive filters are applied (at `begin()` and after any re-initialization)
- **THEN** RXB0 SHALL use mask `0x7F8` with filters 0 and 1 = `0x7E8` (OBD-II replies `0x7E8`–`0x7EF`)
- **AND** RXB1 SHALL use mask `0x7FF` with filters 2–5 = `0x305` (steering angle only)
- **AND** extended-ID frames SHALL match no filter
- **AND** RXB0-to-RXB1 rollover SHALL stay enabled so an OBD-II reply arriving while RXB0 is full is not lost

#### Scenario: Lost steering frames are tolerated
- **WHEN** a `0x305` frame is lost (RXB1 overflow or a slow loop iteration)
- **THEN** the loss SHALL NOT be treated as a CAN error, because every frame carries the absolute angle
- **AND** only the steering-angle frame timeout SHALL turn missing frames into a steering sensor fault

#### Scenario: OBD-II polling unaffected
- **WHEN** the steering-angle sensor transmits at 100 Hz on the shared bus
- **THEN** the RPM poll cadence (`CAN_POLL_INTERVAL_RPM`, and `CAN_POLL_INTERVAL_RPM_BOOST` during a gear change) and the other PIDs SHALL be polled as without the sensor

### Requirement: Steering Sensor Zero-Set Sequence
The CAN controller SHALL provide a non-blocking sequence that sets the Bosch LWS sensor's zero at the current position while keeping its output ID `0x305`, pausing OBD-II polling for its duration. It SHALL be the only code that transmits on ID `0x7C0`, and it SHALL transmit only the two fixed payloads below.

#### Scenario: Zero-set frames
- **WHEN** a zero-set is started
- **THEN** OBD-II polling SHALL be paused: no new request SHALL be sent, and a request already in flight SHALL be allowed to complete or time out first
- **AND** the frame `0x7C0`, DLC 2, `05 00` (reset calibration) SHALL be sent
- **AND** after `STEER_LWS_ZERO_GAP_MS` (about 100 ms), measured without blocking, the frame `0x7C0`, DLC 2, `53 30` (zero here, output ID `0x305`) SHALL be sent
- **AND** OBD-II polling SHALL then resume where it left off

#### Scenario: No other configuration payload
- **WHEN** any code path transmits on ID `0x7C0`
- **THEN** the payload SHALL be exactly `05 00` or `53 30`
- **AND** no interface SHALL accept a `0x7C0` payload from outside the CAN controller, so that a payload such as `03 00` (which moves the sensor's output to ID `0x000`) can never be sent

#### Scenario: Start refused or aborted
- **WHEN** a zero-set is requested while the CAN controller is not initialized, an ECU capability probe is running, or a zero-set is already running
- **THEN** it SHALL be refused with a reason and no frame SHALL be sent
- **WHEN** a zero-set frame fails to transmit
- **THEN** the sequence SHALL abort with a `tx_failed` reason and OBD-II polling SHALL resume

### Requirement: CAN Bus Sniffer
The CAN controller SHALL provide a one-shot sniffer that, for a bounded window, accepts every
standard and extended CAN ID and records a per-ID summary, then restores the normal acceptance
filters and resumes OBD-II polling. The filter changes SHALL be made by the CAN task; the recording
SHALL run in the main loop on the frames taken from the receive queue.

#### Scenario: Start the sniffer
- **WHEN** `startSniff(ms)` is called while the controller is initialised, no ECU probe and no LWS
  zero-set is running, and no gear change is active
- **THEN** OBD-II polling SHALL pause (an in-flight request is abandoned)
- **AND** the CAN task SHALL set both receive-buffer masks to 0 with one standard and one extended
  filter per buffer, so every standard and extended ID is accepted, with RXB0→RXB1 rollover kept
- **AND** the previous results SHALL be cleared and the window SHALL start when the task reports
  the filters open
- **AND** if the task cannot open the filters, it SHALL put the normal filters back (a full chip
  re-initialisation if that fails), and the sniff SHALL complete at once with an "open failed"
  flag and no frames

#### Scenario: Refuse while busy
- **WHEN** `startSniff()` is called while the ECU probe, the LWS zero-set or a sniff is running, or
  a gear change is active
- **THEN** it SHALL be refused and the bus configuration SHALL not change
- **WHEN** an ECU probe or an LWS zero-set is requested while the sniffer runs
- **THEN** that request SHALL be refused as busy

#### Scenario: Record frames
- **WHEN** a frame is received during the window
- **THEN** every frame taken from the receive queue in an `update()` call SHALL be recorded
- **AND** the total frame count SHALL increase
- **AND** for an ID already in the table its count, DLC and last 8 data bytes SHALL be updated, and
  each data byte that differs from the previous frame of that ID SHALL set its bit in the ID's
  changed mask (the bit stays set)
- **AND** a new ID SHALL get a row while fewer than 32 rows exist
- **AND** a new ID that does not fit SHALL be counted in a distinct "IDs beyond 32" counter
- **AND** the RX0OVR / RX1OVR overflow flags counted by the CAN task during the window SHALL be
  reported as the window's overflow count, and the frames dropped from the full receive queue
  during the window SHALL be reported separately
- **AND** a standard 0x305 frame SHALL still be handed to the LWS decoder

#### Scenario: Window ends
- **WHEN** the window elapses
- **THEN** the CAN task SHALL re-program the normal masks and filters with the same routine used at
  start-up (a full chip re-initialisation if that fails)
- **AND** OBD-II polling SHALL resume from IDLE without an immediate forced re-initialisation
- **AND** the results SHALL be marked complete with the completion time

#### Scenario: Steering sensor during the window
- **WHEN** LWS frames are lost during the window because the receive buffers overflow
- **THEN** the steering controller SHALL take its normal sensor-unhealthy path
- **AND** it SHALL recover by itself once valid frames arrive again, without a reboot

### Requirement: Steering Angle Frame Silence Watchdog
The CAN controller SHALL re-initialize the MCP2515 when the Bosch LWS `0x305` stream stops while the
rest of the bus demonstrably works, because the existing OBD-II no-data watchdog
(`CAN_REINIT_NO_DATA_MS`, 15 s) never fires on a vehicle whose ECU keeps answering. This change
leaves the OBD-II watchdog's trigger unchanged (its ignition gating is specified by change
`gate-obd-watchdog-and-pull-up-vesc-rx`). Because the LWS is powered from the ACC line (ignition Relay_1), the
watchdog SHALL run only while the relay controller's ignition state is `ACC` or `IGNITION`, SHALL arm
only after the first `0x305` frame received since the ignition was switched on, and SHALL use a fixed
re-init spacing of `CAN_LWS_REINIT_MIN_MS` (5 s) without any back-off.

#### Scenario: LWS silent, ECU answering
- **WHEN** the ignition state is `ACC` or `IGNITION`
- **AND** at least one `0x305` frame has been received since the ignition was switched on
- **AND** no `0x305` frame has been received for more than `CAN_LWS_SILENT_MS` (1 s)
- **AND** an OBD-II reply was received within `CAN_LWS_WD_OBD_ALIVE_MS` (2 s)
- **AND** no LWS zero-set sequence is running
- **AND** at least `CAN_LWS_REINIT_MIN_MS` (5 s) have passed since the last re-initialization of any kind
- **THEN** the controller SHALL re-initialize the MCP2515 (reset, both acceptance paths, NORMAL mode)
- **AND** journal `CAN W lws_silent ms=<silence> obd_alive=1 -> reinit`

#### Scenario: Whole bus silent
- **WHEN** neither `0x305` frames nor OBD-II replies are being received
- **THEN** the LWS watchdog SHALL NOT re-initialize the chip
- **AND** recovery SHALL be left to the OBD-II no-data watchdog

#### Scenario: Sensor never seen
- **WHEN** no `0x305` frame has been received since the ignition was switched on
- **THEN** the LWS watchdog SHALL stay disarmed

#### Scenario: Ignition off
- **WHEN** the ignition state is `OFF` (or `CRANKING`)
- **THEN** the LWS watchdog SHALL NOT journal `lws_silent` and SHALL NOT re-initialize the chip
- **AND** switching the ignition to `ACC` or `IGNITION` again SHALL leave the watchdog disarmed until
  the next `0x305` frame

#### Scenario: Sensor stays dead with the ignition on
- **WHEN** the armed watchdog has re-initialized the chip and the LWS still sends nothing
- **THEN** further re-inits SHALL follow at the fixed `CAN_LWS_REINIT_MIN_MS` (5 s) spacing, with no
  back-off
- **AND** within that silence episode the `lws_silent … -> reinit` line SHALL be journaled at its
  start and then at most every `CAN_WD_REPEAT_LOG_MS` (10 min) with `repeat=<re-inits since the last
  line>`; the successful re-inits in between SHALL be counted (`can_reinit`) but not journaled
  one by one (the same rule applies to the OBD-II no-data watchdog while the ECU stays silent)

#### Scenario: First frame after a re-initialization
- **WHEN** a re-initialization completed
- **THEN** the times to the first `0x305` frame and the first OBD-II reply SHALL be journaled as
  `CAN I first_frame`

### Requirement: MCP2515 Re-initialization Verification
Every MCP2515 re-initialization (bus-off, OBD-II no-data watchdog, LWS silence watchdog, sniffer filter restore) SHALL check the results of the chip reset (`begin()`), of `applyRxFilters()` and of `setMode(MCP_NORMAL)`, SHALL retry a failed attempt on later `update()` calls without looping inside one call, at most `CAN_REINIT_MAX_ATTEMPTS` (3) times per trigger, and SHALL journal every outcome. The main loop SHALL decide when to re-initialize and SHALL handle the result; each attempt SHALL be executed by the CAN task and SHALL be bounded.

#### Scenario: Re-initialization succeeds
- **WHEN** all three steps succeed
- **THEN** `CAN I reinit reason=<trigger> result=ok attempt=<n>` SHALL be journaled (except for a
  watchdog repeat inside one silence episode, see "Sensor stays dead with the ignition on")
- **AND** the session counter `can_reinit` SHALL be incremented

#### Scenario: Re-initialization fails
- **WHEN** any of the three steps fails
- **THEN** the attempt SHALL be repeated on a later `update()` call, up to 3 attempts in total
- **AND** after the third failure `CAN E reinit result=fail` SHALL be journaled and the next watchdog
  interval SHALL try again

#### Scenario: Bus-off is journaled
- **WHEN** the controller detects bus-off
- **THEN** `CAN E bus_off tec=<TEC> rec=<REC> eflg=<EFLG>` SHALL be journaled before the recovery

#### Scenario: Attempt executed by the CAN task
- **WHEN** the main loop requests a re-initialization attempt
- **THEN** the CAN task SHALL run it at `CAN_TASK_CONFIG_PRIORITY` (the loop's priority) and restore its own priority afterwards
- **AND** the task SHALL first reset the chip and require CANSTAT to show CONFIG mode and CANCTRL to read `0x87`, and SHALL otherwise fail the attempt with `begin=0` without any library mode change
- **AND** filter programming SHALL stop at the first failing call
- **AND** after `setMode(MCP_NORMAL)` the task SHALL read back CANSTAT and CANINTE (`0x03`), and a mismatch SHALL fail the attempt
- **AND** the main loop SHALL NOT wait for the attempt; its result SHALL be handled on a later `update()` call
- **AND** no second command SHALL be posted to the task while an attempt is in flight
- **AND** frames already in the receive queue SHALL be kept

#### Scenario: Attempt duration bounded
- **WHEN** the chip does not answer the reset
- **THEN** the attempt SHALL fail within about 10 ms
- **WHEN** the chip answers the reset but refuses a mode change
- **THEN** the attempt SHALL end within about 0.6 s, in the CAN task

### Requirement: OBD-II No-Data Watchdog Ignition Gating
The CAN controller's OBD-II no-data watchdog (no valid OBD-II reply for `CAN_REINIT_NO_DATA_MS`,
15 s, while polling → MCP2515 re-initialization, at least `CAN_REINIT_MIN_INTERVAL_MS`, 15 s, after
the last re-initialization of any kind) SHALL run only while the relay controller's ignition state is
`ACC` or `IGNITION`, using the same ignition flag as the steering-angle frame silence watchdog,
because the ECU is powered from the ACC line (ignition Relay_1). The silence SHALL be counted from
the later of the last OBD-II reply (before any reply: `begin()` or the last re-initialization) and
the last ignition switch-on, so that the window starts fresh on every switch to `ACC` / `IGNITION`.
Bus-off and send-failure recovery SHALL NOT be gated.

#### Scenario: Ignition off, bus silent
- **WHEN** the ignition state is `OFF` (or `CRANKING`)
- **AND** no OBD-II reply has been received for any length of time
- **THEN** the OBD-II watchdog SHALL NOT re-initialize the MCP2515
- **AND** SHALL NOT journal `obd_silent`
- **AND** the `can_reinit` counter SHALL NOT be incremented by it

#### Scenario: Fresh window after switch-on
- **WHEN** the ignition is switched from `OFF` to `ACC` or `IGNITION`
- **AND** the last OBD-II reply (or the last re-initialization) is older than 15 s
- **THEN** the watchdog SHALL NOT fire at the switch-on
- **AND** SHALL fire only if no OBD-II reply arrives within 15 s after the switch-on

#### Scenario: Ignition on, ECU silent
- **WHEN** the ignition state is `ACC` or `IGNITION`
- **AND** no OBD-II reply has been received for 15 s counted from the later of the last reply / re-init
  and the switch-on
- **AND** at least 15 s have passed since the last re-initialization of any kind
- **THEN** the controller SHALL re-initialize the MCP2515 (reason `obd_silent`)
- **AND** journal `CAN W obd_silent ms=<silence> obd_alive=0 repeat=<n> -> reinit` under the existing
  per-episode journal economy

#### Scenario: New power cycle starts a new journal episode
- **WHEN** an `obd_silent` silence episode was running and the ignition goes `OFF` and back to `ACC`
  or `IGNITION`
- **THEN** the next `obd_silent` trigger SHALL be journaled (not counted as a quiet repeat)

#### Scenario: LWS watchdog re-init is not blocked
- **WHEN** the ignition state is `ACC` or `IGNITION` and the steering-angle frame silence watchdog
  requests a re-initialization
- **THEN** that re-initialization SHALL proceed exactly as before
- **AND** the OBD-II watchdog's minimum spacing SHALL be measured from it, as from any re-init

### Requirement: MCP2515 Single-Owner CAN Task
A dedicated FreeRTOS task (`can_io`) SHALL be the only code that accesses the MCP2515 over SPI: initialization, re-initialization, filter programming, reception, transmission and error-flag reads. The main loop SHALL communicate with it only through bounded, non-blocking queues and a command slot, and SHALL never wait on the chip.

#### Scenario: Task configuration
- **WHEN** `begin()` runs
- **THEN** the task SHALL be created before the first initialization, pinned to `CAN_TASK_CORE` (1, the Arduino loop's core), at `CAN_TASK_PRIORITY` (12, above the AsyncTCP task and below the WiFi, lwIP, esp_timer and IPC tasks), with a `CAN_TASK_STACK` (4096-byte) stack
- **AND** the task SHALL never be deleted, SHALL never log, and SHALL end every cycle in a blocking wait

#### Scenario: Interrupt wakes the task without SPI
- **WHEN** the MCP2515 pulls `INT` (GPIO38, `PIN_CAN_INT`) low
- **THEN** a FALLING-edge ISR SHALL only count the interrupt and notify the task
- **AND** the ISR SHALL NOT access SPI, the MCP2515 library or any queue

#### Scenario: Task drains the chip
- **WHEN** the task wakes on a notification or after `CAN_POLL_MS` (2 ms) without one
- **THEN** it SHALL read frames while READ STATUS reports one, at most `CAN_RX_DRAIN_CAP` (16) per wake, and put each into the receive queue without blocking
- **AND** it SHALL service the transmit queue and, every `CAN_EFLG_POLL_MS`, the error flags

#### Scenario: Board without the INT line
- **WHEN** `PIN_CAN_INT` is not defined (the `old-board` branch) or the build defines `CAN_TEST_NO_INT`
- **THEN** no interrupt SHALL be attached and the task SHALL receive on its `CAN_POLL_MS` poll alone
- **AND** steering-angle frames SHALL stay fresh

#### Scenario: Back-to-back burst received
- **WHEN** three accepted frames arrive about 260 µs apart while the main loop is busy for tens of milliseconds and the INT line works
- **THEN** all three SHALL reach the receive queue and the RX overflow count SHALL NOT increase

#### Scenario: No SPI from the main loop
- **WHEN** any `CANController` method runs in the main loop
- **THEN** it SHALL NOT perform SPI transfers or call the MCP2515 library, and SHALL NOT wait for the task

### Requirement: CAN Receive Queue
Received frames SHALL pass from the CAN task to the main loop through a bounded ring of `CAN_RX_RING` (64) frames, each holding the ID in the `readMsgBuf()` encoding (bit 31 extended, bit 30 remote), the DLC and up to 8 data bytes.

#### Scenario: Ring full
- **WHEN** the task has a frame and the ring is full
- **THEN** the oldest frame in the ring SHALL be discarded, the new one stored, and the drop counted
- **AND** the task SHALL NOT block

#### Scenario: Consumers stay in the main loop
- **WHEN** `update()` takes frames from the ring
- **THEN** steering-angle frames SHALL be handed to the LWS decoder with the main-loop time of dispatch
- **AND** the LWS decoder, the OBD-II and probe state machines, the sniffer table and the CAN watchdogs SHALL run only in the main loop

#### Scenario: Extended frames pass through
- **WHEN** a frame with an extended ID is accepted by the acceptance filters
- **THEN** it SHALL reach the main loop with bit 31 of its ID set

### Requirement: Queued CAN Transmit
Every frame the firmware sends on the CAN bus SHALL be queued by the main loop in a transmit ring of `CAN_TX_RING` (8) frames and loaded into the MCP2515 by the CAN task, without any busy-wait in the main loop. General frames SHALL use transmit buffer TXB0 only, in FIFO order; TXB1 and TXB2 SHALL NOT be used by this change.

#### Scenario: Queue a frame
- **WHEN** the main loop queues a frame
- **THEN** the call SHALL return at once, `true` if the frame was queued and `false` if the ring is full (counted)
- **AND** the task SHALL be notified

#### Scenario: Frame sent
- **WHEN** TXB0 is free and a frame is queued
- **THEN** the task SHALL write it into TXB0 and request transmission
- **AND** when TXREQ clears, the frame SHALL count as sent

#### Scenario: Frame not sent in time
- **WHEN** a frame has not left TXB0 within `CAN_TX_TIMEOUT_MS` (20 ms)
- **THEN** the task SHALL abort that buffer only, by clearing its TXREQ
- **AND** once TXREQ reads clear, the frame SHALL count as a transmit failure if TXB0CTRL.ABTF is set, and as sent otherwise (an attempt already on the wire completed)
- **AND** if TXREQ is still set `CAN_TX_TIMEOUT_MS` after the abort, the frame SHALL count as a transmit failure
- **AND** no other buffer and no other sender's frame SHALL be aborted
- **AND** the CANCTRL abort-all bit SHALL NOT be used

#### Scenario: Sender asks for the result
- **WHEN** a frame was queued with a result request (the LWS zero-set frames)
- **THEN** the task SHALL report sent or failed to the main loop
- **AND** a frame queued while the chip is down (a failed initialization or re-initialization) SHALL be reported as failed at once
- **AND** a frame still pending in the transmit buffer when a re-initialization or filter change starts SHALL be aborted in that buffer only and reported as sent or failed by the same ABTF rule
- **AND** a frame whose requester has stopped waiting for its result SHALL NOT be loaded into the chip afterwards

#### Scenario: Steering sensor zero-set frames
- **WHEN** the zero-set queues `[05 00]`
- **THEN** the reset frame SHALL be reported as possibly sent from that moment, so the steering erases its Left / Right limits whatever the transmit result (only a frame that could not be queued leaves them)
- **AND** the sequence SHALL continue only after a confirmed send of `[05 00]`, and SHALL complete only after a confirmed send of `[53 30]`
- **AND** a failed result, or no result within `CAN_TX_RESULT_WAIT_MS` (1000 ms), for either frame SHALL abort the sequence with `tx_failed`, and a frame without a result SHALL be withdrawn so that it is never sent later

#### Scenario: OBD-II send failure
- **WHEN** an OBD-II request cannot be queued
- **THEN** the PID SHALL be rescheduled and its retry counter incremented, as for any send failure
- **AND** a request that was queued but never sent SHALL be handled by the 200 ms response timeout

### Requirement: MCP2515 Initialization Retry
A failed MCP2515 initialization at boot, or a re-initialization sequence that failed all its attempts, SHALL NOT leave the CAN bus down until a reboot or a later watchdog trigger. While the chip is down the CAN task SHALL retry the initialization every `CAN_INIT_RETRY_MS` (5 s) after the last attempt of any kind, until it succeeds; a command from the main loop SHALL take precedence over a retry.

#### Scenario: Chip absent at boot, present later
- **WHEN** the first initialization fails
- **THEN** `CAN E init result=fail begin=<0|1> filt=<0|1> mode=<0|1>` SHALL be journaled once
- **AND** the task SHALL retry every `CAN_INIT_RETRY_MS`, each retry first checking that the chip answers a reset
- **WHEN** a retry succeeds
- **THEN** the controller SHALL become initialized as if `begin()` had succeeded (PID stagger, no-data watchdog reference)
- **AND** `CAN I init result=ok attempt=<n> after_s=<s>` SHALL be journaled
- **AND** the steering-angle sensor SHALL leave `can_down` without a reboot

#### Scenario: Chip down after a failed re-initialization
- **WHEN** `CAN E reinit result=fail` has been journaled and the chip stays down
- **THEN** the task SHALL retry every `CAN_INIT_RETRY_MS`, gate first, without journaling the failed retries
- **WHEN** a retry succeeds
- **THEN** `CAN I reinit reason=task_retry result=ok attempt=<n> after_s=<s>` SHALL be journaled, `can_reinit` SHALL be incremented and the first-frame timing SHALL run as after any re-initialization

#### Scenario: Retries are cheap and quiet
- **WHEN** the chip keeps failing
- **THEN** each retry SHALL take about 10 ms or less while the chip does not answer the reset, and SHALL run in the CAN task
- **AND** the failed retries SHALL be counted but not journaled one by one

### Requirement: CAN Driver Diagnostics
The CAN controller SHALL count and report the health of the CAN task, its queues and its INT wake-up, through the event journal and the telemetry.

#### Scenario: Periodic journal line
- **WHEN** `JOURNAL_SNAP_MS` (60 s) elapses while the CAN task exists
- **THEN** one line `CAN I stats rx=<n> drop=<n> hwm=<n> ovr=<n> int=<n> poll=<n> tx=<n> txf=<n> bmax=<us> miss=<n>` SHALL be journaled
- **AND** `rx`, `drop`, `ovr`, `int`, `poll`, `tx`, `txf` and `miss` SHALL count the window (frames read, receive-queue drops, RX overflow flags, INT wakes, frames found on poll wakes, frames sent, transmit failures, LWS frames missed by the decoder), and `hwm` and `bmax` SHALL be the window's receive-queue high-water mark and longest receive / transmit / error-flag service pass of the task in microseconds, excluding initialization, re-initialization and filter-change time

#### Scenario: INT state
- **WHEN** a `CAN_INT_WINDOW_MS` (5 s) window ends
- **THEN** the INT state SHALL be `none` without an attached interrupt, `dead` if frames were read and no interrupt was counted, `ok` if interrupts were counted, and unchanged if no frame was read
- **AND** a change to `dead` SHALL be journaled once as `CAN W int state=dead`, and a change back as `CAN I int state=ok`

#### Scenario: Console lines kept
- **WHEN** RX overflows occur or the health log interval elapses without valid data
- **THEN** the existing rate-limited `[CAN] RX overflow` and `[CAN] health:` lines SHALL still be printed, from the task's counters and snapshot, in every controller mode

