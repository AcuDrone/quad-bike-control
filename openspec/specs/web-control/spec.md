# web-control Specification

## Purpose
TBD - created by archiving change add-web-portal. Update Purpose after archive.
## Requirements
### Requirement: Web-Based Manual Control
The system SHALL accept manual control commands from the web interface when the MAVLink
command stream is inactive.

#### Scenario: Receive brake control command from web
- **WHEN** set_brake command is received via WebSocket with value 0 to 100
- **AND** input source is WEB (MAVLink inactive)
- **THEN** brake value is validated (0 to 100 range, integer or float)
- **AND** BrakeController or BTS7960Controller is called with brake percentage
- **AND** success response is sent to web client
- **AND** brake position updates in real-time

#### Scenario: Reject brake control when MAVLink active
- **WHEN** set_brake command is received via WebSocket
- **AND** input source is MAVLINK (MAVLink command stream valid)
- **THEN** command is rejected
- **AND** error response is sent with reason "MAVLink control active"
- **AND** no brake actuator state change occurs

#### Scenario: Validate brake command value range
- **WHEN** set_brake command is received with value <0 or >100
- **THEN** command is rejected
- **AND** error response is sent with validation error message "Value must be 0-100"
- **AND** no brake actuator state change occurs

### Requirement: Input Source Priority Management
The system SHALL enforce input source priority to prevent control conflicts.

#### Scenario: Activate web control when MAVLink inactive
- **WHEN** the MAVLink command stream is lost or times out (>500ms)
- **AND** no recent `SERVO_OUTPUT_RAW` commands received
- **THEN** input source switches to WEB (if web control commands present) or FAILSAFE (if no commands)
- **AND** web control commands are accepted
- **AND** web interface enables manual control UI

#### Scenario: Deactivate web control when MAVLink reconnects
- **WHEN** the MAVLink command stream becomes valid after being inactive
- **THEN** input source switches to MAVLINK immediately
- **AND** web control commands are rejected
- **AND** web interface disables manual control UI (read-only telemetry only)
- **AND** MAVLink commands take control of vehicle

#### Scenario: Apply fail-safe when both sources inactive
- **WHEN** the MAVLink command stream is inactive (>500ms timeout)
- **AND** no web control commands received for >1 second
- **THEN** input source switches to FAILSAFE
- **AND** fail-safe commands are applied (brakes on, neutral, center steering, idle throttle)
- **AND** telemetry shows FAILSAFE input source

### Requirement: Web Control User Interface
The system SHALL provide web interface controls for manual operation.

#### Scenario: Display brake control slider
- **WHEN** web page loads
- **THEN** manual control section includes brake slider
- **AND** brake slider range is 0% to 100% (released to fully applied)
- **AND** slider default position is 0% (released)
- **AND** slider is styled consistently with steering/throttle sliders

#### Scenario: Enable brake control when web control active
- **WHEN** input source is WEB
- **THEN** brake slider is enabled
- **AND** brake slider position reflects telemetry feedback
- **AND** brake percentage value is displayed next to slider

#### Scenario: Disable brake control when MAVLink active
- **WHEN** input source is MAVLINK
- **THEN** brake slider is disabled (greyed out)
- **AND** brake slider shows current MAVLink-controlled position (read-only)
- **AND** slider cannot be moved by user

#### Scenario: Send brake command from web UI
- **WHEN** user moves brake slider
- **THEN** set_brake command is sent via WebSocket with slider value (0-100)
- **AND** commands are rate-limited (max 10 Hz, one per 100ms) to avoid flooding
- **AND** slider position shows visual feedback during command transmission

### Requirement: Web Control Safety
The system SHALL enforce safety constraints on web-based control commands.

#### Scenario: Apply rate limiting to brake commands
- **WHEN** web client moves brake slider rapidly
- **THEN** brake commands are sent at maximum 10 Hz (one command per 100ms)
- **AND** intermediate slider values are debounced
- **AND** final position is always sent after user stops sliding

#### Scenario: Reset brake on client disconnect
- **WHEN** WebSocket client disconnects
- **AND** brake was being controlled via web
- **THEN** brake is released to 0% (fail-safe)
- **AND** system switches to FAILSAFE input source
- **AND** fail-safe brake state is applied (fully released unless emergency)

### Requirement: Ignition Control via Web Interface
The system SHALL accept ignition control commands from the web interface to manage vehicle ignition states.

#### Scenario: Set ignition state via WebSocket
- **WHEN** set_ignition command is received via WebSocket with value OFF/ACC/IGNITION/START
- **THEN** ignition state value is validated (must be one of: OFF, ACC, IGNITION, START)
- **AND** VehicleController.setIgnitionState() is called with the requested state
- **AND** success response is sent to web client
- **AND** ignition state updates in real-time telemetry

#### Scenario: Reject invalid ignition state command
- **WHEN** set_ignition command is received with invalid state value
- **THEN** command is rejected
- **AND** error response is sent with validation error message "Invalid ignition state"
- **AND** no ignition state change occurs

#### Scenario: Display ignition control buttons in web UI
- **WHEN** web page loads
- **THEN** "Vehicle Ignition & Lighting" card is displayed
- **AND** card includes four ignition buttons: OFF, ACC, IGNITION, START
- **AND** active ignition state button is visually highlighted
- **AND** buttons are always enabled regardless of S-bus status (ignition control available in all modes)

#### Scenario: Highlight active ignition state
- **WHEN** telemetry updates ignition state
- **THEN** corresponding button is highlighted with active styling
- **AND** other ignition buttons show inactive styling
- **AND** START button returns to inactive after cranking completes

#### Scenario: Send ignition command on button click
- **WHEN** user clicks ignition button
- **THEN** set_ignition command is sent via WebSocket with button value (OFF/ACC/IGNITION/START)
- **AND** button shows visual feedback during command transmission

### Requirement: Front Light Control via Web Interface
The system SHALL accept front light control commands from the web interface.

#### Scenario: Set front light state via WebSocket
- **WHEN** set_light command is received via WebSocket with boolean value
- **THEN** light state is validated (boolean true/false)
- **AND** VehicleController.setFrontLight() is called with on/off state
- **AND** success response is sent to web client
- **AND** light state updates in real-time telemetry

#### Scenario: Reject invalid light command
- **WHEN** set_light command is received with non-boolean value
- **THEN** command is rejected
- **AND** error response is sent with validation error message "Invalid light state"
- **AND** no light state change occurs

#### Scenario: Display front light toggle in web UI
- **WHEN** web page loads
- **THEN** "Vehicle Ignition & Lighting" card includes front light toggle switch
- **AND** toggle switch reflects current light state from telemetry
- **AND** toggle is always enabled regardless of S-bus status

#### Scenario: Toggle light state on user interaction
- **WHEN** user clicks light toggle switch
- **THEN** set_light command is sent via WebSocket with new state (true/false)
- **AND** toggle shows visual feedback during command transmission

### Requirement: Ignition and Light Telemetry Broadcasting
The system SHALL broadcast current ignition state and front light status in telemetry updates.

#### Scenario: Include ignition state in WebSocket telemetry
- **WHEN** telemetry update is broadcast via WebSocket
- **THEN** telemetry includes ignition_state field with current state (OFF/ACC/IGNITION/CRANKING)
- **AND** telemetry includes is_cranking boolean field (true during starter motor operation)

#### Scenario: Include light state in WebSocket telemetry
- **WHEN** telemetry update is broadcast via WebSocket
- **THEN** telemetry includes front_light_on boolean field with current light state
- **AND** light state reflects actual relay output

#### Scenario: Update ignition display in telemetry section
- **WHEN** web page receives telemetry update
- **THEN** ignition state indicator updates to show current state
- **AND** cranking indicator shows animated/highlighted state during START operation
- **AND** front light indicator updates to show ON/OFF state

#### Scenario: Report the front-wheel lock as not fitted
- **WHEN** telemetry update is broadcast via WebSocket
- **THEN** telemetry includes `wheel_lock_fitted` boolean field, `false` on the aztec buggy
- **AND** `wheel_lock_on` SHALL be `false`

#### Scenario: Wheel-lock control disabled when not fitted
- **WHEN** web page receives telemetry with `wheel_lock_fitted` equal to `false`
- **THEN** the wheel-lock button SHALL be disabled and its status SHALL read "not fitted" (localized)
- **AND** clicking it SHALL NOT send a `set_wheel_lock` command
- **AND** telemetry without the `wheel_lock_fitted` field SHALL be treated as fitted

#### Scenario: Wheel-lock command rejected when not fitted
- **WHEN** a `set_wheel_lock` command is received via WebSocket and the lock is not fitted
- **THEN** an error response "Front-wheel lock not fitted on this vehicle" SHALL be sent
- **AND** no relay SHALL change state

### Requirement: Gear Position Default Configuration via Web Interface
The system SHALL allow the user to view and update the default transmission servo position
(0.0–100.0 %) for each gear through the web portal, without requiring a firmware recompile.

#### Scenario: Set a gear's default position via WebSocket
- **WHEN** a `set_gear_default` WebSocket command is received with a valid gear string (`"R"`, `"N"`, `"L"`, `"H"`) and a position value
- **THEN** the system SHALL call `setDefaultPosition()` on the transmission controller
- **AND** SHALL respond with success naming the gear and the stored position
- **AND** this command SHALL be accepted regardless of the active input source (same privilege level as calibration commands)

#### Scenario: Reject invalid gear or out-of-range position
- **WHEN** a `set_gear_default` command is received with an unrecognised gear string or a position outside 0.0–100.0 %
- **THEN** the system SHALL respond with `{"ok": false, "error": "<reason>"}` and leave NVS unchanged

#### Scenario: Display gear position defaults editor in web UI
- **WHEN** the Calibration section is shown
- **THEN** a "Gear position defaults" card SHALL show one "Положення серво" / "Servo position"
  slider (0–100 %, 0.1 % steps) that follows `hall_position` from telemetry while it is not being
  dragged
- **AND** four large buttons "Зберегти як R/N/H/L" / "Save as R/N/H/L" in a 2×2 grid (order R, N,
  H, L), each with the saved position from telemetry on a second line (`--` when unknown)
- **AND** a compact row of "▶ R", "▶ N", "▶ H", "▶ L" buttons
- **AND** the boost RPM target input with its Save button
- **AND** there SHALL be no per-gear number inputs, no 📍 capture button and no per-row Move/Save
  buttons

#### Scenario: Start and finish positioning
- **WHEN** the engineer presses "Почати" / "Start"
- **THEN** the slider and the ▶ row SHALL become enabled and "Готово" / "Done" SHALL replace "Почати"
- **WHEN** the engineer presses "Готово"
- **THEN** the slider and the ▶ row SHALL be disabled again
- **AND** this active state SHALL exist only in the page; no WebSocket command SHALL be sent for
  Почати or Готово, and a reloaded page SHALL start locked
- **WHEN** the WebSocket disconnects while the card is active
- **THEN** the slider and the ▶ row SHALL be disabled and "Почати" shown again, and reconnecting
  SHALL NOT unlock them by itself

#### Scenario: Drag the slider to move the servo
- **WHEN** the slider is enabled and the engineer drags it
- **THEN** the existing `move_to_position` command SHALL be sent with the slider value
- **AND** commands SHALL be rate-limited to at most one per 100 ms, and the final value SHALL
  always be sent when the drag ends

#### Scenario: Save a default from the web UI
- **WHEN** the engineer presses "Зберегти як <gear>" (enabled whether or not "Почати" was pressed,
  because saving does not move the servo)
- **THEN** the web UI SHALL send a `set_gear_default` WebSocket command with that gear and the
  current slider value
- **AND** on the next telemetry the button's second line SHALL show the saved value

#### Scenario: Highlight the gear at the current position
- **WHEN** the current slider value is within ±0.3 % of a gear's saved position
- **THEN** that gear's "Зберегти як" button SHALL be highlighted with the active-gear style
- **AND** the other buttons SHALL not be highlighted

#### Scenario: Move to a saved position
- **WHEN** the card is active and the engineer presses "▶ <gear>"
- **THEN** `move_to_position` SHALL be sent with that gear's saved position

#### Scenario: Show live encoder count near gear controls
- **WHEN** the Manual Control card is displayed
- **THEN** the current transmission servo position in percent (from `hall_position` telemetry; formerly an encoder count) SHALL be visible adjacent to the gear buttons
- **AND** it SHALL update in real time

### Requirement: 24V Boost Rail Control via Web Interface
The web portal SHALL provide a guarded control for the 24V boost rail, following the same command
pattern as the existing front light and wheel lock controls, so the rail can be power-cycled on the
bench without removing steering authority from a live vehicle.

#### Scenario: Send a boost command
- **WHEN** the operator toggles the 24V boost control in the web UI
- **THEN** a `set_boost` command with a boolean value SHALL be sent over the WebSocket
- **AND** it SHALL be parsed into `WebPortal::WebCommand` and dispatched by
  `VehicleController::processWebCommand` in the same manner as `set_light` and `set_wheel_lock`
- **AND** it SHALL be accepted regardless of the active input source (MAVLINK / WEB / FAILSAFE)

#### Scenario: Enable the boost rail
- **WHEN** a `set_boost` command with value `true` is received
- **THEN** `PIN_BOOST_EN` SHALL be driven HIGH
- **AND** the startup grace period for the low-rail warning SHALL be restarted
- **AND** a success response SHALL be sent to the client
- **AND** enabling SHALL always be permitted

#### Scenario: Refuse to disable the boost while ignition is not OFF
- **WHEN** a `set_boost` command with value `false` is received
- **AND** the reported ignition state is not `OFF`
- **THEN** the command SHALL be rejected and `PIN_BOOST_EN` SHALL remain HIGH
- **AND** an error response SHALL be sent to the client explaining that the rail powers the steering
  driver and that ignition must be OFF first
- **AND** the refusal SHALL be logged

#### Scenario: Disable the boost when ignition is OFF
- **WHEN** a `set_boost` command with value `false` is received
- **AND** the reported ignition state is `OFF`
- **THEN** `PIN_BOOST_EN` SHALL be driven LOW
- **AND** the low-rail warning SHALL be suppressed while the boost is commanded off
- **AND** a success response SHALL be sent to the client

#### Scenario: Reflect boost state in the UI
- **WHEN** telemetry reporting `boost_on` is received
- **THEN** the boost control SHALL reflect the actual commanded state rather than the last click
- **AND** its label SHALL have i18n keys present in both the `en` and `uk` dictionaries

### Requirement: ECU Capability Probe Command via Web Interface
The system SHALL accept a `can_probe` command from the web interface to trigger a one-shot ECU
capability probe, and SHALL surface the probe's progress and outcome back to the web client.

#### Scenario: Trigger probe via WebSocket
- **WHEN** a `can_probe` command is received via WebSocket
- **AND** CAN data is valid and no gear change is active
- **THEN** the command SHALL be routed through `VehicleController::processWebCommand` to
  `CANController::startProbe()`
- **AND** the controller SHALL enter probe mode and begin the capability sweep
- **AND** the command SHALL be accepted regardless of the active input source (diagnostic
  privilege level, same as calibration commands)

#### Scenario: Reject or defer probe while busy or unavailable
- **WHEN** a `can_probe` command is received while a probe is already running, or while a gear
  change is active
- **THEN** the command SHALL be rejected/deferred without disrupting the in-progress probe or gear
  change
- **AND** the response SHALL indicate a busy/deferred status
- **WHEN** a `can_probe` command is received while CAN data is invalid (ECU disconnected)
- **THEN** the probe SHALL short-circuit with a "no ECU / disconnected" result and no bus traffic

#### Scenario: Deliver probe results to the web client
- **WHEN** the probe completes
- **THEN** the probe results (supported-PID bitmaps, per-PID supported/answered/raw/decoded, and
  the DTC summary) SHALL be delivered to connected web clients via the existing telemetry
  WebSocket broadcast while the results are fresh
- **AND** the results SHALL also be mirrored to the serial log via the CAN debug feature

#### Scenario: Display probe control and results in web UI
- **WHEN** the web page loads
- **THEN** a "Probe ECU" button SHALL be available to send the `can_probe` command
- **AND** the button SHALL show an in-progress state and reject further clicks while a probe is
  running
- **AND** a results panel SHALL display the returned bitmaps, per-PID results, and DTC summary
  when results are received

### Requirement: Steering Speed-Scaling Configuration via Web Interface
The web portal SHALL allow the operator to set the steering speed-scaling base without a reflash,
and to inject a test speed for bench verification, following the same command pattern as the
existing speed calibration setters. Both commands SHALL be routed through
`WebPortal::WebCommand` → `VehicleController::processWebCommand`.

#### Scenario: Set the scaling base from the portal
- **WHEN** the operator enters a base speed in metres per second and saves it
- **THEN** a `set_steer_sca_base` WebSocket command carrying that value SHALL be sent
- **AND** on success the system SHALL respond with an acknowledgement naming the stored value
- **AND** the command SHALL be accepted regardless of the active input source, at the same
  privilege level as the existing `speed_cal_ppr` and `speed_cal_circ` commands

#### Scenario: Zero clears the local base
- **WHEN** a `set_steer_sca_base` command carries zero
- **THEN** the stored base SHALL be cleared to "not set" and persisted as such
- **AND** the portal SHALL show that the base now comes from the autopilot's `MOT_SPD_SCA_BASE`,
  or that there is none

#### Scenario: Reject an out-of-range base from the portal
- **WHEN** a `set_steer_sca_base` command carries a value that is not-a-number, negative, or above
  `STEER_SCALE_BASE_MAX_MS`
- **THEN** the system SHALL respond with an error naming the accepted range
- **AND** the stored base SHALL be left unchanged

#### Scenario: Inject a bench test speed
- **WHEN** the operator enters a test speed and sends it
- **THEN** a `set_test_speed` WebSocket command carrying that value in metres per second SHALL be
  sent
- **AND** the system SHALL apply it to the steering speed-scaling calculation only
- **AND** the response SHALL state that the override is temporary and when it will expire
- **AND** the command SHALL be accepted regardless of the active input source

#### Scenario: Reject an out-of-range test speed
- **WHEN** a `set_test_speed` command carries a value that is not-a-number, negative, or above
  `STEER_SCALE_TEST_SPEED_MAX_MS`
- **THEN** the system SHALL respond with an error and no override SHALL be applied

#### Scenario: Show the scaling state in the portal
- **WHEN** the web portal renders a telemetry frame
- **THEN** it SHALL show the base currently in use, which source that base came from, and the
  scale being applied
- **AND** a live test-speed override SHALL be shown prominently, with its value and the fact that
  it expires by itself
- **AND** every label SHALL be present in both the English and the Ukrainian dictionary

### Requirement: Gear Code Calibration via Web Interface
The system SHALL let the operator assign the current live 3-bit gear code to a gear from the web
interface, so the code table can be matched to the actual gear selector on the bench.

The runtime table SHALL start from the `GEAR_CODE_*` defaults in `Constants.h` and SHALL be
persisted in the NVS namespace `"transmission"` under the keys `code_r`, `code_n`, `code_h` and
`code_l`. Commands SHALL go through the queued web-command path (`WebCommand` →
`VehicleController::processWebCommand()` → `TransmissionController`) and SHALL be refused while the
engine is running, like throttle calibration.

#### Scenario: Show the live bits
- **WHEN** telemetry arrives with `gear_code`
- **THEN** the web interface SHALL show bits A (port bit 0), B (port bit 1) and C (port bit 6) as
  individual indicators and the packed code as `0bABC` with its decoded gear
- **AND** the indicators SHALL show "no data" while `gear_code` is `-1`
- **AND** labels SHALL have i18n keys present in both the `en` and `uk` dictionaries

#### Scenario: Capture the current code for a gear
- **WHEN** the operator presses "Save as R" (or N, L, H), sending `{"cmd":"set_gear_code","value":"R"}`
- **AND** the engine is not running, no gear change is in progress and the opto-input snapshot is valid
- **THEN** the live code SHALL be stored as that gear's code in NVS
- **AND** it SHALL take effect immediately for `getPhysicalGear()` and `gear_sensed`
- **AND** the response SHALL be success with the message `Gear code for R saved: XYZ`
- **AND** a TRANSMISSION debug line SHALL be logged

#### Scenario: Reject the reserved codes 000 and 111
- **WHEN** a capture is requested while the live code is `000` (all lines open, mid-shift) or `111`
  (opto feed lost)
- **THEN** the capture SHALL be rejected with an error naming the code as reserved for UNKNOWN
- **AND** the table SHALL be unchanged

#### Scenario: Reject a stale or missing input snapshot
- **WHEN** a capture is requested while the opto-input snapshot is stale or the reader is not bound
- **THEN** the capture SHALL be rejected with an error that the opto inputs are unavailable

#### Scenario: Take over a code assigned to another gear
- **WHEN** a capture for a gear is requested and the live code is the active code of a different gear
- **THEN** the capture SHALL succeed and the code SHALL be assigned to the requested gear
- **AND** the other gear SHALL become UNSET (sentinel `0xFF` in the runtime table and in NVS)
- **AND** the response SHALL name the other gear, e.g. `Gear code for R saved: 011 (was N; N is now unset)`
- **AND** an UNSET gear SHALL never decode, so the physical gear SHALL read `GEAR_UNKNOWN` while the
  lever is in that gear until it is captured again
- **AND** the assigned codes SHALL stay distinct

#### Scenario: Reject while the engine runs or the servo moves
- **WHEN** a capture or reset is requested while the engine is running
- **THEN** it SHALL be rejected with an error
- **AND** a capture SHALL also be rejected while a gear change is in progress

#### Scenario: Reset to defaults
- **WHEN** the operator presses "Reset to defaults", sending `{"cmd":"reset_gear_codes"}`
- **THEN** all four entries SHALL be restored to the `GEAR_CODE_*` defaults immediately, including UNSET ones
- **AND** the `code_r`/`code_n`/`code_h`/`code_l` NVS keys SHALL be removed
- **AND** a TRANSMISSION debug line SHALL be logged

#### Scenario: Codes persist across reboot
- **WHEN** a code was captured and the controller reboots
- **THEN** `TransmissionController::begin()` SHALL load the stored codes from NVS
- **AND** a gear with no stored key SHALL keep its default code

#### Scenario: Invalid NVS content falls back to defaults
- **WHEN** the loaded table assigns `000`, `111` or a value above 7 (other than the UNSET sentinel
  `0xFF`), or assigns the same code to two gears
- **THEN** the whole table SHALL fall back to the `GEAR_CODE_*` defaults
- **AND** an error SHALL be logged on the TRANSMISSION debug feature

#### Scenario: Show the active table
- **WHEN** telemetry arrives with `gear_codes`
- **THEN** the four "Зберегти як" / "Save as" buttons SHALL be shown as large buttons in a 2×2 grid
  in the order R, N, H, L, styled identically to the gear position default buttons
- **AND** each button's second line SHALL show that gear's active code as three binary digits,
  followed by "(типовий)" / "(default)" when its `gear_codes_custom` bit is clear
- **AND** an UNSET entry (`-1`) SHALL be shown as "не задано" / "not set" in a warning colour
- **AND** the button whose code equals the live `gear_code` (when `gear_code` ≥ 0) SHALL be
  highlighted with the active-gear style
- **AND** no separate text table of the codes SHALL be shown

#### Scenario: Help text for take-over
- **WHEN** the engineer opens the card's ⓘ help
- **THEN** the help text SHALL tell the operator that after a take-over the other gear must be
  captured, and that the engine must be off

### Requirement: Web Relay Test Control
The web portal SHALL provide a relay test for wiring checks: one on/off button per relay
(Relay_1..Relay_8), sent as the WebSocket command
`{"cmd":"set_relay_test","relay":<1..8>,"value":<true|false>}` and handled in
`VehicleController::processWebCommand()` through the queued web-command path into
`RelayController::setRelayTest()`. The command SHALL NOT be gated by input source; its gate is the
relay-test gate (ignition `OFF`, no function relay energized).

#### Scenario: Toggle a relay on from the web UI
- **WHEN** the operator clicks a relay's button while it is off and the ignition is OFF
- **THEN** a `set_relay_test` command with that relay number and `value: true` SHALL be sent
- **AND** the response SHALL be success ("Relay test: Relay_N ON") and the relay SHALL energize

#### Scenario: Toggle a relay off from the web UI
- **WHEN** the operator clicks an energized test relay's button
- **THEN** `set_relay_test` with `value: false` SHALL be sent and the relay SHALL release
- **AND** switching OFF SHALL remain possible whatever the ignition state

#### Scenario: Relay outside 1..8 is rejected
- **WHEN** a `set_relay_test` command carries a missing relay number or one outside 1..8
- **THEN** an error response "Relay test: Relay number must be 1-8" SHALL be sent
- **AND** no relay SHALL change state

#### Scenario: Relay test rejected while the ignition is on
- **WHEN** a `set_relay_test` ON command arrives while the ignition is not OFF or the front light is on
- **THEN** an error response starting with "Relay test:" SHALL be sent and no relay SHALL change

#### Scenario: Relay test card in the UI
- **WHEN** the Tests section is shown
- **THEN** a "Relay Test" card SHALL show eight equal-size buttons in a 2-column grid (1 column at
  768 px or less), styled like the gear buttons
- **AND** each button SHALL show "Реле N" / "Relay N" on its first line and the relay's function on
  a smaller second line — 1 ignition, 2 starter (max 2 s), 3 light, 4–6 lock (unassigned), 7–8
  spare — in English and Ukrainian
- **AND** an energized relay SHALL be shown with the same highlight as the active gear button and
  `aria-pressed="true"`; the button SHALL NOT carry ON/OFF text, and its tooltip SHALL name the
  relay, its function and its ON/OFF state
- **AND** each button's state SHALL follow the telemetry `relay_test` bitfield, not the last click
- **AND** a button that is off SHALL be disabled while the ignition is not OFF, the front light is
  on, the relay board is faulted or the WebSocket is disconnected; a button that is on SHALL stay
  clickable while connected so it can be switched off
- **AND** the response to a relay test command SHALL be shown in the card

### Requirement: Steering Calibration and Sensor Zero via Web Interface
The web portal SHALL provide steering calibration controls for the Bosch LWS sensor: jog and nudge that are always available, a "Set sensor zero" action, and Left/Right limit capture. These commands SHALL be accepted regardless of the active input source (same privilege level as the other calibration commands). The calibration card SHALL hold the actions and a compact readout only; the full sensor diagnostics SHALL be shown in the Diagnostics section ("Кермо: стан").

#### Scenario: Set sensor zero
- **WHEN** the operator presses "Set sensor zero" and confirms the dialog explaining that the current position becomes the steering center and that the stored limits will be erased
- **THEN** a `steer_lws_zero` WebSocket command SHALL be sent
- **AND** the firmware SHALL start the zero-set sequence with no ignition, engine-RPM or standstill precondition
- **AND** the firmware SHALL report success only after an accepted sensor frame shows sensor OK, no fault code, the calibrated bit set and an angle within `STEER_LWS_ZERO_TOLERANCE` of zero, within `STEER_LWS_ZERO_CONFIRM_MS` of the second frame
- **AND** otherwise it SHALL report failure with a reason: `busy`, `can_unavailable`, `tx_failed`, `range_latched` (restart the vehicle and retry), `no_frames` or `not_confirmed`
- **AND** the "Кермо: стан" card SHALL show the progress (`steer_zero_state`) and the reason (`steer_zero_error`), and the calibration card's status line SHALL show the command response

#### Scenario: Capture limits only
- **WHEN** the operator presses "Set Left" or "Set Right"
- **THEN** a `steer_cal_left` or `steer_cal_right` command SHALL be sent
- **AND** the response SHALL report the captured angle in degrees on success, or name the reason on failure (sensor fault, too close to zero, or a reading that contradicts the stored opposite limit together with the "Set sensor zero" remedy)
- **AND** when the capture toggled the sensor-direction flag, the success response SHALL say so (e.g. "sensor direction detected: mirrored — capture the right limit again") and SHALL NOT claim the calibration is complete
- **AND** a capture beyond ±550° (`STEER_LWS_SAFE_RANGE`) SHALL succeed, and its response SHALL carry a warning that the sensor's latched-fault threshold is near
- **AND** no center capture button SHALL be shown
- **AND** a `steer_cal_center` command SHALL be answered with a failure that points to "Set sensor zero"

#### Scenario: Jog and nudge always enabled
- **WHEN** the calibration card is shown
- **THEN** the jog and nudge controls SHALL NOT be disabled because of the steering sensor state or the calibration state
- **AND** while `steer_sensor_ok` is false the card SHALL show a note that jog is running without a sensor and without stall protection

#### Scenario: Compact readout in the calibration card
- **WHEN** the calibration card is shown
- **THEN** it SHALL show the jog speed slider, the current angle, the Left/Right limits (with the ⚠ beyond-550° note when `steer_range_warning` is true), and one status line with the sensor state and the calibration state
- **AND** it SHALL NOT show the LWS status bits, fault code, frame rate, error counters, zero-set state, motor current, FET temperature or driver state, which are shown in "Кермо: стан"
- **AND** readouts shown both here and in "Кермо: стан" SHALL be rendered from the same telemetry frame so the two never disagree

#### Scenario: Calibration help text
- **WHEN** the engineer opens the calibration card's ⓘ help
- **THEN** its help text (en/uk) SHALL describe the flow: jog to straight-ahead, restart the vehicle, press "Set sensor zero", then jog to each lock and capture Left and Right
- **AND** it SHALL state that the sensor direction (normal or mirrored mount) is detected automatically from the first limit capture, and that after re-mounting the sensor the operator sets the sensor zero again and captures both limits
- **AND** it SHALL tell the operator to check, right after the first capture, that Jog ▶ makes the angle grow, and that a shrinking angle means Left and Right were captured at the wrong locks
- **AND** it SHALL state that a sensor reporting `range_latched` is cleared only by restarting the vehicle, and that jogging past about 550° risks that latch

### Requirement: Service Console Navigation
The web portal SHALL be organised as a service console for the engineer, with a left sidebar
navigation and exactly four sections, of which exactly one is visible at a time: Діагностика /
Diagnostics, Калібрування / Calibration, Тести / Tests and Система / System. The sidebar SHALL
show the brand "Сервісна консоль" / "Service console", one button per section, and the WebSocket
and firmware status items at its bottom. The portal SHALL NOT include an operator dashboard or
overview tiles. All cards SHALL stay in the page while hidden, so switching sections never shows
stale data.

#### Scenario: Default section on first visit
- **WHEN** the page loads and no section has been stored
- **THEN** the Diagnostics section SHALL be shown and its sidebar button SHALL be highlighted
- **AND** the other three sections SHALL be hidden

#### Scenario: Switching sections
- **WHEN** the engineer clicks a section button in the sidebar
- **THEN** that section SHALL be shown, the previously shown one hidden, and the button highlighted
- **AND** the section title in the content header SHALL show the localized section name
- **AND** the page SHALL NOT reload and the WebSocket SHALL stay connected

#### Scenario: Last section is remembered
- **WHEN** the engineer selects a section and later reloads the page
- **THEN** the same section SHALL be shown, restored from `localStorage` key `ui_section`
- **AND** a stored value that is not one of the four sections SHALL fall back to Diagnostics

#### Scenario: Storage unavailable
- **WHEN** `localStorage` throws or is unavailable (private mode, blocked storage)
- **THEN** every read and write SHALL be caught, no JavaScript error SHALL escape
- **AND** the page SHALL start on Diagnostics and section switching SHALL still work

#### Scenario: Narrow screen
- **WHEN** the viewport is 768 px wide or less
- **THEN** the sidebar SHALL be replaced by a sticky top bar with a ☰ button and the current
  section title
- **AND** ☰ SHALL open the navigation (including the WebSocket and firmware status items) and
  choosing a section SHALL close it
- **AND** a small round connection dot next to ☰ SHALL show the WebSocket state: green while
  connected, red while disconnected, with a localized tooltip
- **AND** cards SHALL be laid out in one column with no horizontal scrolling at 375 px

### Requirement: Service Console Section Layout
Each card SHALL belong to exactly one section. The Diagnostics section SHALL be read-only: apart
from the existing "Probe ECU" diagnostic button in the CAN card it SHALL contain no control that
sends a WebSocket command. Every command-sending control SHALL be in Calibration, Tests or System.

#### Scenario: Diagnostics content
- **WHEN** the Diagnostics section is shown
- **THEN** it SHALL show, in order, the attention box (when not empty), Telemetry, "Кермо: стан" /
  "Steering: state", Vehicle Data (CAN), MAVLink Command Channels and the read-only "Швидкість" /
  "Speed" card

#### Scenario: Calibration content
- **WHEN** the Calibration section is shown
- **THEN** it SHALL show Steering calibration, Throttle calibration, Gear position defaults, Gear
  sensor codes, the "Датчик швидкості" / "Speed sensor" card (pulses per revolution, the
  pulses-per-revolution measurement, wheel circumference) and, directly after it, the "Обмеження
  швидкості та кермо" / "Speed limit and steering" card (limiter floor, speed-limit readout with
  the limiter row, steering authority, steering scaling base, bench test speed)
- **AND** each of the two speed cards SHALL have its own status line, and the reply to a save SHALL
  be shown in the status line of the card the save came from

#### Scenario: Tests content
- **WHEN** the Tests section is shown
- **THEN** it SHALL show a one-line note "Ручне керування виконавчими механізмами для перевірки на
  стенді" / "Manual actuator control for bench checks", then Manual control, Ignition & lighting
  and Relay test

#### Scenario: System content
- **WHEN** the System section is shown
- **THEN** it SHALL show System maintenance (debug toggle, boost test, firmware upload, and a "Що
  нового" / "What's new" button that opens the existing changelog modal)
- **AND** a read-only "Мережа" / "Network" card showing the access point SSID and IP address from
  telemetry (`wifi_ssid`, `wifi_ip`) with a note that the settings will come later, offering no
  input or command
- **AND** there SHALL be no separate "What's new" card

### Requirement: Card Help Modal
Cards whose long instruction paragraph was shown inline SHALL instead offer a small ⓘ help button
in their title that opens one shared help modal. The modal SHALL reuse the changelog modal styles,
use the card title as its title and render the card's existing i18n help string(s) as HTML. The
long paragraphs SHALL be removed from the cards; one-line field hints and state-dependent warning
boxes SHALL stay in the cards.

#### Scenario: Cards with a help button
- **WHEN** the page is rendered
- **THEN** an ⓘ button SHALL be present in the titles of Steering calibration, Throttle
  calibration, Gear position defaults, Gear sensor codes, Speed sensor (the sensor help, including
  the pulses-per-revolution measuring procedure), Speed limit and steering (one modal with an
  introduction that this card uses the speed from the sensor card, followed by the limiter,
  limiter-floor, steering-scaling and test-speed help texts), Manual control, Ignition & lighting
  (24 V rail note), Relay test and System maintenance (firmware update warning)
- **AND** each ⓘ button SHALL have a localized accessible label

#### Scenario: Open and close the help
- **WHEN** the engineer clicks an ⓘ button
- **THEN** the shared modal SHALL open with the card title and that card's help text
- **AND** it SHALL close on ✕, on a click on the overlay outside the dialog, and on Escape

#### Scenario: Language switch with the help open
- **WHEN** the language is switched while the help modal is open
- **THEN** the modal title and body SHALL re-render in the new language without closing

#### Scenario: Warnings stay in the cards
- **WHEN** a state-dependent warning applies (MAVLink control active in Manual control, sensorless
  jog in Steering calibration, a steering limit beyond ±550°)
- **THEN** the warning SHALL be shown inside the card, not only in the help modal

### Requirement: Throttle Calibration Card
The Throttle calibration card SHALL always show the idle and full endpoint sliders with the saved
values from telemetry (`throttle_idle_us`, `throttle_full_us`), disabled until a calibration
session is active. The session logic in the firmware (`throttle_cal_begin`, `throttle_cal_save`,
`throttle_cal_cancel`, engine-off gate, live servo follow) SHALL be unchanged.

#### Scenario: Idle card
- **WHEN** telemetry reports `throttle_calibrating` false
- **THEN** both sliders SHALL be visible, show the saved endpoints in µs, and be disabled and
  visually muted
- **AND** only the "Почати" / "Start" button SHALL be shown

#### Scenario: Calibration session active
- **WHEN** the engineer presses "Почати" and telemetry then reports `throttle_calibrating` true
- **THEN** both sliders SHALL be enabled and drive the servo live as before
- **AND** "Зберегти" / "Save" and "Скасувати" / "Cancel" SHALL replace "Почати"

#### Scenario: Session ends
- **WHEN** telemetry reports `throttle_calibrating` false after Save or Cancel
- **THEN** the sliders SHALL be disabled again and show the endpoints now stored

### Requirement: Steering Slider Lock Without Sensor
The web portal SHALL lock the percent steering controls in Тести → Ручне керування / Tests → Manual control (the steering slider and the Left / Center / Right test buttons) whenever `steer_mode` is not `position`, and SHALL explain why. The steering jog and nudge controls and the other manual controls SHALL be unaffected.

#### Scenario: Slider locked in rate mode
- **WHEN** a telemetry frame has `steer_mode` equal to `rate`
- **THEN** the steering slider and the three steering test buttons SHALL be disabled, in addition to the existing input-source gating
- **AND** a hint SHALL be shown under the slider: "Без датчика кута кермо керується стіком як швидкістю (тільки MANUAL). Повзунок вимкнено, Jog/Nudge працюють." / "No angle sensor: the stick steers by rate (MANUAL only). Slider disabled; Jog/Nudge still work."

#### Scenario: Slider locked while blocked
- **WHEN** a telemetry frame has `steer_mode` equal to `blocked`, including after the operator has taken web control with the sensor unhealthy
- **THEN** the steering slider and the three steering test buttons SHALL be disabled
- **AND** a hint SHALL be shown: "Немає датчика кута: повзунок вимкнено, використовуйте Jog/Nudge у Калібруванні." / "No angle sensor: slider disabled, use Jog/Nudge in Calibration."

#### Scenario: Slider unlocked again
- **WHEN** a later frame has `steer_mode` equal to `position`
- **THEN** the lock and the hint SHALL be removed on that frame, and the controls SHALL follow the existing input-source gating only

#### Scenario: Other controls unaffected
- **WHEN** the steering slider is locked by the steering mode
- **THEN** the steering jog and nudge controls SHALL remain enabled exactly as before (see "Steering Calibration and Sensor Zero via Web Interface")
- **AND** "take control" and the throttle, brake and gear controls SHALL keep their existing behaviour

#### Scenario: Firmware rejects a percent command without a sensor
- **WHEN** a `set_steering` command reaches the firmware while the steering angle sensor is unhealthy
- **THEN** it SHALL be rejected without driving the motor, and the response SHALL say that there is no steering sensor and that Jog/Nudge should be used

### Requirement: Speed-Limiter Floor Configuration via Web Interface
The web portal SHALL allow the operator to set the speed-limiter throttle floor without a reflash,
from a number input in the "Обмеження швидкості та кермо" / "Speed limit and steering" card,
following the same command pattern as the existing speed calibration setters. The command SHALL
be routed through `WebPortal::WebCommand` → `VehicleController::processWebCommand`.

#### Scenario: Set the floor from the portal
- **WHEN** the operator enters a floor in percent and saves it
- **THEN** a `set_speed_limit_floor` WebSocket command carrying that value SHALL be sent
- **AND** on success the system SHALL persist the value and respond with an acknowledgement naming
  the stored value in percent
- **AND** the command SHALL be accepted regardless of the active input source, at the same
  privilege level as the existing `speed_cal_ppr` and `speed_cal_circ` commands

#### Scenario: Reject an out-of-range floor
- **WHEN** a `set_speed_limit_floor` command carries a value that is not-a-number, below
  `SPEED_LIMIT_FLOOR_MIN_PCT` or above `SPEED_LIMIT_FLOOR_MAX_PCT`
- **THEN** the system SHALL respond with an error naming the accepted range (0–20 %)
- **AND** the stored floor SHALL be left unchanged
- **AND** the portal SHALL also refuse the value locally, before sending, with a translated
  message

#### Scenario: The input follows the stored value
- **WHEN** the portal renders a telemetry frame and the floor input is not being edited
- **THEN** the input SHALL show `speed_limit_floor`, as the pulses-per-revolution and
  circumference inputs show their stored values

#### Scenario: The help explains the floor
- **WHEN** the operator opens the help (ⓘ) of the Speed limit and steering card
- **THEN** it SHALL explain that the floor is the lowest throttle the speed limiter may leave at
  and above the limit, that 0 % means idle and holds the limit most tightly, and that a higher
  floor softens engine braking at the cost of a limit that may not hold
- **AND** every new label, message and help text SHALL be present in both the English and the
  Ukrainian dictionary

### Requirement: Pulses-per-Revolution Measurement via Web Interface
The "Датчик швидкості" / "Speed sensor" card SHALL offer a measurement of pulses per revolution:
the operator turns the wheel by hand an entered number of turns, and the page divides the pulses
counted by the turns. The measurement SHALL run entirely in the page from the `speed_pulse_total`
telemetry value. The only command it sends SHALL be the existing `speed_cal_ppr`, and only when
the operator saves the result.

#### Scenario: Start a measurement
- **WHEN** the WebSocket is connected, the last telemetry frame carried a numeric
  `speed_pulse_total`, and the operator enters a number of wheel turns (1–20, default 5) and
  presses "Почати" / "Start"
- **THEN** the page SHALL record the last received `speed_pulse_total` as the start value
- **AND** it SHALL lock the turns input and show "Зупинити" / "Stop" and "Скасувати" / "Cancel"
- **AND** it SHALL show a hint to lift the wheel, select neutral, turn the wheel by hand exactly
  that many turns in one direction, and press Stop only once the live count has stopped changing
- **AND** no command SHALL be sent

#### Scenario: Start is unavailable
- **WHEN** the WebSocket is disconnected, or the telemetry frames carry no numeric
  `speed_pulse_total`
- **THEN** "Почати" / "Start" SHALL be disabled
- **AND** the measurement SHALL NOT require taking web control, as `speed_cal_ppr` does not

#### Scenario: Live count
- **WHEN** a telemetry frame arrives while measuring
- **THEN** the page SHALL show "Pulses: N · turns: T", where N is
  `(speed_pulse_total − start) >>> 0`, so a 32-bit wrap of the total does not corrupt the count

#### Scenario: Stop and show the result
- **WHEN** the operator presses "Stop"
- **THEN** the page SHALL freeze the count at once, from the last `speed_pulse_total` received
  before the press, with no settle delay
- **AND** it SHALL show the division and the rounded value, "Result: N / T = x.x → R pulses/rev",
  with one decimal and a decimal comma in Ukrainian
- **AND** it SHALL offer "Зберегти як імпульсів/оберт" / "Save as pulses/rev" and "Виміряти знову" /
  "Measure again"

#### Scenario: Large fractional part warns
- **WHEN** the result is shown and `|N / T − R| ≥ 0.2`, evaluated exactly as
  `5 × |N − R × T| ≥ T`
- **THEN** the page SHALL show a warning that the turns were inexact or pulses were missed and the
  measurement should be repeated
- **AND** saving SHALL still be offered

#### Scenario: No pulses or an out-of-range result cannot be saved
- **WHEN** the result has N = 0, or R is below `SPEED_PPR_MIN` (1) or above `SPEED_PPR_MAX` (1000)
- **THEN** the page SHALL show "No pulses counted. Repeat." for N = 0, or the existing
  pulses-per-revolution range message otherwise
- **AND** "Save as pulses/rev" SHALL NOT be offered

#### Scenario: Save the result
- **WHEN** the operator presses "Save as pulses/rev" on a savable result
- **THEN** the page SHALL put R into the pulses-per-revolution input and send `speed_cal_ppr` with
  R through the same validation path as the input's own Save button
- **AND** the firmware's reply SHALL be shown in the Speed sensor card's status line
- **AND** the value SHALL persist across a reboot, as for any `speed_cal_ppr`

#### Scenario: Cancel and measure again
- **WHEN** the operator presses "Cancel" while measuring, or "Measure again" after a result
- **THEN** the block SHALL return to its idle state with the turns input unlocked and no command
  sent

#### Scenario: The measurement is not persisted
- **WHEN** the page is reloaded, or the WebSocket disconnects, during or after a measurement
- **THEN** the block SHALL show its idle state and the measured count SHALL be discarded
- **AND** nothing about the measurement SHALL be stored in the browser or in the firmware

#### Scenario: Help and translations
- **WHEN** the operator opens the help (ⓘ) of the Speed sensor card
- **THEN** it SHALL describe the procedure: lift the wheel and select neutral, chalk mark on the
  tyre, set the turns, Start, turn the wheel by hand exactly that many turns in one direction, wait
  until the count stops changing, Stop, and that a whole number is reliable while a large
  fractional part means repeat
- **AND** every new label, message and help text of the block SHALL be present in both the English
  and the Ukrainian dictionary, and SHALL re-render on a language switch

