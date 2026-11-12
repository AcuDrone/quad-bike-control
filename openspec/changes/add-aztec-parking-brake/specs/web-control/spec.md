## ADDED Requirements
### Requirement: Parking Brake Card
The web portal SHALL provide a card "🅿️ Стоянкове гальмо" / "🅿️ Parking brake" in the Tests
section, placed after the "Vehicle Ignition & Lighting" card, to apply and release the aztec parking
brake, show its state, and set its travel time and drive direction.

Its commands are `park_brake` (`value` `"apply"` or `"release"`), `set_park_brake_travel_ms`
(`value` ms), `set_park_brake_thr_pass_pct` (`value` %) and `set_park_brake_invert` (`value`
bool). They SHALL be routed through
`WebPortal::WebCommand` → `VehicleController::processWebCommand` → `commandParkBrake()` or the
setting setters. They SHALL be accepted regardless of the active input source and of web control,
like `set_wheel_lock`.

The card SHALL be hidden when telemetry carries no `pbrake_state` field.

#### Scenario: Apply with two-press confirmation
- **WHEN** the operator presses "Затягнути" / "Apply"
- **THEN** nothing SHALL be sent on the first press
- **AND** the button SHALL be armed through the shared two-press confirmation helper, with the
  label "Підтвердити затягування" / "Confirm apply"
- **AND** the card status line SHALL show the warning "Стоянкове гальмо буде затягнуто. Газ на
  передачі — лише холостий." / "The parking brake will be applied. Throttle in gear stays at idle."
- **AND** a second press within 5 s SHALL send `{"cmd":"park_brake","value":"apply"}`

#### Scenario: Release with two-press confirmation
- **WHEN** the operator presses "Відпустити" / "Release" twice within 5 s
- **THEN** the first press SHALL arm the button with the label "Підтвердити відпускання" / "Confirm
  release" and the warning "Стоянкове гальмо буде відпущено — на схилі багі може покотитися." / "The
  parking brake will be released — on a slope the buggy may roll."
- **AND** the second press SHALL send `{"cmd":"park_brake","value":"release"}`

#### Scenario: Command replies
- **WHEN** a `park_brake` command is processed
- **THEN** the reply SHALL be one of:
  - "Park brake applying" or "Park brake applying (speed unknown)";
  - "Park brake releasing", with a " (state not saved)" suffix when the moving state was not saved;
  - "Park brake reversing to apply" or "Park brake reversing to release";
  - "Park brake already applying" or "Park brake already releasing";
  - "Park brake apply refused: moving at N.N km/h (max 3.0)";
  - "Failed to save the park brake state — apply refused";
  - "Park brake driver not available";
  - "Park brake: unknown request" for a value other than `"apply"` or `"release"`.
- **AND** the reply SHALL be shown in the card's status line, in the danger colour for a failure

#### Scenario: State display
- **WHEN** telemetry arrives
- **THEN** the card SHALL show the state from `pbrake_state` in the current language:
  - "Відпущено" / "Released";
  - "Затягнуто" / "Applied";
  - "Затягується…" / "Applying…";
  - "Відпускається…" / "Releasing…";
  - "Невідомо" / "Unknown".
- **AND** while a run is in progress it SHALL show the remaining time from `pbrake_remaining_ms` in
  whole seconds
- **AND** while `pbrake_thr_hold` is true it SHALL show "Газ утримується на холостому" / "Throttle
  held at idle"
- **AND** while a release run is in progress and `pbrake_thr_hold` is false, it SHALL show "Газ
  пропускається, гальмо ще відпускається" / "Throttle passing, brake still releasing"
- **AND** when `pbrake_da_wait` is `"ign_off"` it SHALL show "Запалювання вимкнено — газ не
  відпускає стоянкове гальмо" / "Ignition off — throttle does not release the parking brake"
- **AND** when `pbrake_da_wait` is `"rearm"` it SHALL show "Відпустіть газ нижче 10 % і натисніть
  знову" / "Drop the throttle below 10 % and press again"
- **AND** the Apply and Release buttons SHALL stay enabled during a run

#### Scenario: Set the travel time
- **WHEN** the operator drags the travel time slider (range input 1000–5000 ms, step 100, default
  2000) and releases it
- **THEN** the value label SHALL show "N мс" / "N ms" live while dragging, and nothing SHALL be sent
  before the release
- **AND** on release (`change`) one `set_park_brake_travel_ms` command SHALL be sent
- **AND** the firmware SHALL round it to a whole ms, validate 1000–5000, persist, apply, and reply
  "Park brake travel time set to N ms"
- **AND** an out-of-range value SHALL be answered "Park brake travel time out of range (1000-5000 ms)"
- **AND** telemetry SHALL NOT move the slider while it is being dragged

#### Scenario: Set the throttle release point
- **WHEN** the operator drags the throttle release point slider (range input 20–100 %, step 5,
  default 50) and releases it
- **THEN** the value label SHALL show "N %" together with the matching time "≈ M мс" / "≈ M ms"
  (N % of the travel time in use, or "до кінця ходу" / "until the run ends" at 100 %) live while
  dragging, and nothing SHALL be sent before the release
- **AND** on release (`change`) one `set_park_brake_thr_pass_pct` command SHALL be sent
- **AND** the firmware SHALL round it to a whole number, validate 20–100, persist, apply, and reply
  "Park brake throttle release point set to N %"
- **AND** an out-of-range value SHALL be answered "Park brake throttle release point out of range
  (20-100 %)"
- **AND** telemetry SHALL NOT move the slider while it is being dragged

#### Scenario: Change the direction
- **WHEN** the operator presses the direction toggle ("Напрямок: звичайний / інвертований" /
  "Direction: normal / inverted") twice within 5 s
- **THEN** the first press SHALL arm it with the warning "Напрямок зміниться, стан стане
  «невідомо»." / "The direction will change and the state becomes unknown."
- **AND** the second press SHALL send `set_park_brake_invert` with the inverted value
- **AND** the firmware SHALL reply "Park brake direction inverted — state unknown" or "Park brake
  direction normal — state unknown", or "Park brake direction change refused: actuator moving"
  during a run

#### Scenario: Save failure for a setting
- **WHEN** the NVS write fails for `set_park_brake_travel_ms`, `set_park_brake_thr_pass_pct` or
  `set_park_brake_invert`
- **THEN** the firmware SHALL reply "Failed to save the park brake setting" and keep the value in
  use unchanged

#### Scenario: No connection to the board
- **WHEN** the operator presses any park-brake control while the WebSocket is not open
- **THEN** nothing SHALL be sent, no button SHALL be armed, and the card's status line SHALL show
  «Немає з'єднання з платою» / "No connection to the board" in the danger colour

#### Scenario: Help text
- **WHEN** the operator opens the card's ⓘ help
- **THEN** it SHALL explain, in uk and en:
  - apply only by command, refused above 3 km/h;
  - the drive-away release above 10 % throttle in L/H/R with the ignition on (a new press is needed
    if the throttle was already up), and no release in N or with the ignition off;
  - the throttle release point: idle is held for that share of the travel time, then the throttle
    ramps up while the brake finishes releasing (anti roll-back on a slope; 100 % = hold until the
    end);
  - the end of travel by time;
  - the state after a power loss mid-run ("Невідомо" / "Unknown");
  - the bench procedure: with the rear wheels free, press Release from "Unknown". If the cable
    tightens, change the direction and press Release again. Then time a full travel each way and
    set the longer one.
- **AND** Ukrainian text SHALL call the vehicle "багі"
