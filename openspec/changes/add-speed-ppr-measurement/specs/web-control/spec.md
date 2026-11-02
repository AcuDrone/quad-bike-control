## ADDED Requirements
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

## MODIFIED Requirements
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
