## ADDED Requirements
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
  sensor codes and the Speed sensor settings card (pulses per revolution, wheel circumference,
  speed-limit readout, steering scaling base, bench test speed)

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
  calibration, Gear position defaults, Gear sensor codes, Speed sensor (one modal with the sensor,
  limiter, steering-scaling and test-speed help texts), Manual control, Ignition & lighting (24 V
  rail note), Relay test and System maintenance (firmware update warning)
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

## MODIFIED Requirements
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
- **AND** the response SHALL report the captured angle in degrees on success, or name the reason on failure (sensor fault, wrong side of zero, too close to zero)
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
- **AND** it SHALL state that a sensor reporting `range_latched` is cleared only by restarting the vehicle, and that jogging past about 550° risks that latch

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
