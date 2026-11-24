# debug-logging Specification

## Purpose
TBD - created by archiving change add-debug-utility. Update Purpose after archive.
## Requirements
### Requirement: Runtime Debug Toggle
The system SHALL provide a Debug utility class that conditionally outputs serial messages based on a runtime-toggleable flag.

#### Scenario: Debug enabled - messages printed
- **WHEN** DEBUG_ENABLED is true
- **AND** Debug::print() or Debug::println() is called
- **THEN** the message SHALL be output to Serial

#### Scenario: Debug disabled - messages suppressed
- **WHEN** DEBUG_ENABLED is false
- **AND** Debug::print() or Debug::println() is called
- **THEN** no output SHALL be sent to Serial

#### Scenario: Backward compatibility with existing code
- **WHEN** existing code uses Serial.print/println directly
- **THEN** those calls SHALL continue to work without modification
- **AND** SHALL be refactored to use Debug utility for consistency

### Requirement: Debug Utility API
The Debug utility SHALL provide methods compatible with Arduino Serial API for drop-in replacement.

#### Scenario: Print without newline
- **WHEN** Debug::print(value) is called with any printable type
- **AND** DEBUG_ENABLED is true
- **THEN** the value SHALL be printed to Serial without trailing newline

#### Scenario: Print with newline
- **WHEN** Debug::println(value) is called with any printable type
- **AND** DEBUG_ENABLED is true
- **THEN** the value SHALL be printed to Serial with trailing newline

#### Scenario: Printf-style formatted output
- **WHEN** Debug::printf(format, args...) is called
- **AND** DEBUG_ENABLED is true
- **THEN** the formatted string SHALL be printed to Serial

#### Scenario: Support all Arduino printable types
- **WHEN** Debug methods are called with String, char*, int, float, bool, or other Serial-compatible types
- **THEN** the output SHALL match Serial.print behavior exactly

### Requirement: Web Portal Debug Control
The web portal SHALL provide an API endpoint to toggle DEBUG_ENABLED at runtime.

#### Scenario: Enable debug via web portal
- **WHEN** a POST request to /api/debug with {"enabled": true} is received
- **THEN** DEBUG_ENABLED SHALL be set to true
- **AND** a success response SHALL be returned
- **AND** Debug output SHALL begin appearing immediately

#### Scenario: Disable debug via web portal
- **WHEN** a POST request to /api/debug with {"enabled": false} is received
- **THEN** DEBUG_ENABLED SHALL be set to false
- **AND** a success response SHALL be returned
- **AND** Debug output SHALL stop immediately

#### Scenario: Query debug status
- **WHEN** a GET request to /api/debug is received
- **THEN** the current DEBUG_ENABLED state SHALL be returned as JSON
- **AND** response format SHALL be {"enabled": <boolean>}

### Requirement: Debug State Persistence
The debug enabled state SHALL persist across device reboots for consistent troubleshooting.

#### Scenario: Save debug state to NVS
- **WHEN** DEBUG_ENABLED is changed via web portal
- **THEN** the new state SHALL be saved to NVS (Non-Volatile Storage)
- **AND** the state SHALL survive power loss and reboots

#### Scenario: Restore debug state on startup
- **WHEN** the device boots up
- **THEN** DEBUG_ENABLED SHALL be initialized from NVS if available
- **AND** SHALL default to the Constants.h value if no saved state exists

### Requirement: Existing Code Migration
All existing Serial.print/println calls SHALL be systematically migrated to Debug utility.

#### Scenario: Replace Serial.print with Debug::print
- **WHEN** existing code contains Serial.print(msg)
- **THEN** it SHALL be refactored to Debug::print(msg)
- **AND** functionality SHALL remain identical

#### Scenario: Replace Serial.println with Debug::println
- **WHEN** existing code contains Serial.println(msg)
- **THEN** it SHALL be refactored to Debug::println(msg)
- **AND** functionality SHALL remain identical

#### Scenario: Replace Serial.printf with Debug::printf
- **WHEN** existing code contains Serial.printf(format, args...)
- **THEN** it SHALL be refactored to Debug::printf(format, args...)
- **AND** functionality SHALL remain identical

#### Scenario: Preserve critical Serial output
- **WHEN** Serial output is critical for bootloader or emergency diagnostics
- **THEN** those calls MAY remain as direct Serial calls
- **AND** SHALL be documented with a comment explaining why

### Requirement: USB-CDC Serial Console
The debug console SHALL be carried over the ESP32-S3 native USB CDC device on the board's USB-C
connector, not over UART0, because UART0 (GPIO43/44, Prog header X1) carries the steering VESC link.

#### Scenario: Console routed to native USB
- **WHEN** the firmware is built
- **THEN** `platformio.ini` SHALL define the build flag `ARDUINO_USB_CDC_ON_BOOT=1`
- **AND** `Serial` SHALL resolve to the native USB CDC device
- **AND** all `Debug::print` / `printf` / `println` output SHALL appear on the USB-C serial monitor

#### Scenario: Boot never waits for a USB host
- **WHEN** the board is powered with no USB host attached
- **THEN** the firmware SHALL complete `setup()` and enter `loop()` normally
- **AND** no `while (!Serial)` or equivalent host-wait SHALL be present anywhere in the firmware
- **AND** vehicle operation SHALL be unaffected by the absence of a console

#### Scenario: Early-boot output may be lost
- **WHEN** log output is produced before the host enumerates the CDC endpoint
- **THEN** that output MAY be lost and SHALL NOT be relied upon as a boot diagnostic
- **AND** no safety-relevant decision SHALL depend on early-boot console output being observed

#### Scenario: UART0 remains reserved
- **WHEN** the pin map is read
- **THEN** GPIO43 and GPIO44 SHALL be reserved for the steering VESC driver and used by nothing else
- **AND** no `Serial0` / UART0 console SHALL be initialized by the firmware
- **AND** ESP-IDF `esp_log` output SHALL be redirected to the USB-CDC console before the VESC UART is
  opened, and the ROM putc output on UART0 SHALL be closed
- **AND** this SHALL be documented in `platformio.ini` and `Constants.h`

### Requirement: USB Steering Diagnostics Console
The firmware SHALL accept line commands on the USB-CDC `Serial` console (never UART0 or UART2) to measure the steering actuator: `diag on`, `diag off`, `status`, `stop` / `s`, `help`, `ramp L|R [max= step= stepms= start=]`, `pulse L|R <duty> <ms>`, `hold L|R <duty> <ms>` and `goto <deg>`. Lines SHALL be accepted with CR, LF or CRLF endings, case-insensitively. Output SHALL be machine-readable with fixed prefixes: `OK <cmd>`, `E,<cmd>,<reason>`, `E,abort,<reason>`, `I,...`, `D,<t_ms>,<duty>,<pos>,<rate>,<cur>,<vin>,<state>,<ref>` at 50 Hz while an experiment runs, and `S,<cmd>,key=value,...` summaries. Reading and writing SHALL never block the main loop; sample lines that do not fit the USB transmit buffer SHALL be dropped and counted.

#### Scenario: Standstill only
- **WHEN** `diag on` or an experiment is requested while the trusted vehicle speed exceeds `STEER_DIAG_MAX_SPEED_KMH` (0.5 km/h)
- **THEN** it SHALL be refused with `E,<cmd>,speed`
- **AND** a running experiment SHALL be aborted with the motor cut when speed appears

#### Scenario: Experiment preconditions and aborts
- **WHEN** an experiment is requested
- **THEN** it SHALL require diag mode, both limits captured, a valid LWS angle and the VESC link with no fault, and SHALL NOT start toward a limit closer than `STEER_DIAG_LIMIT_MARGIN` (2°)
- **AND** a running experiment SHALL be aborted (motor cut, `E,abort,<reason>`) on angle loss, VESC link loss or fault, driving to within `STEER_DIAG_LIMIT_MARGIN` of a captured limit, `stop`, or a new command

#### Scenario: Break-away ramp
- **WHEN** `ramp R` is run
- **THEN** the duty SHALL start at `start` and rise by `step` every `stepms` up to `max`
- **AND** when the wheel has moved 0.5° in the drive direction (or turns at ≥ `STEER_DIAG_MOVE_RATE` °/s that way) the duty and the latest motor current SHALL be recorded, the duty cut to 0, and the coast logged for `STEER_DIAG_SETTLE_MS`
- **AND** an `S,ramp` line SHALL report the break-away duty, current and coast; the ramp SHALL end after 1 s at `max` without movement and SHALL NOT exceed 15 s

#### Scenario: Pulse and hold
- **WHEN** `pulse` or `hold` is run
- **THEN** the duty SHALL be applied open loop for the given time (pulse ≤ 1000 ms, hold ≤ 1500 ms) and logged until `STEER_DIAG_SETTLE_MS` after the cut
- **AND** the summary SHALL report the displacement at the cut and after settling, the coast and the peak current; a hold SHALL also report the mean rate and mean current over its second half

#### Scenario: Closed-loop goto trace
- **WHEN** `goto <deg>` is run in diag mode
- **THEN** the normal closed loop SHALL drive to the target while D lines carry the loop state (`drive`, `park`, `idle`) and the target
- **AND** after the loop has stopped driving for `STEER_DIAG_SETTLE_MS` (or after 12 s) an `S,goto` line SHALL report the overshoot past the target and the residual error, and the loop SHALL be dropped to idle

#### Scenario: Automatic release
- **WHEN** diag mode is on and no command line arrives for `STEER_DIAG_IDLE_TIMEOUT_MS` (60 s), or the USB host disappears (`Serial` reads not connected continuously for `STEER_DIAG_HOST_LOST_MS` (500 ms), counted only after it has read connected once since `diag on`)
- **THEN** any experiment SHALL be aborted, the motor stopped and the steering released (`I,diag,off,reason=...`)

