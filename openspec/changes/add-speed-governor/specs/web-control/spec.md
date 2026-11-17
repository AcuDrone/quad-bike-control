## ADDED Requirements
### Requirement: Speed Governor Configuration via Web Interface
The Calibration section SHALL contain a card "🎯 Регулятор швидкості" / "🎯 Speed governor",
placed right after the "Обмеження швидкості та кермо" / "Speed limit and steering" card, holding the
governor switch and its settings (see "Speed Governor Settings"). The settings SHALL NOT be placed
in any Tests-section card. Every command SHALL be accepted regardless of the active input source,
like `set_speed_limit_slew`.

#### Scenario: Toggle the governor
- **WHEN** the operator presses the "Регулятор швидкості" / "Speed governor" button
- **THEN** a `set_gov_enabled` command carrying the opposite of the reported `gov_en` SHALL be sent
- **AND** on success the system SHALL persist the value and reply "Speed governor ON" or "Speed
  governor OFF"; the button SHALL show the state from telemetry only (no optimistic update), and
  `--` while unknown

#### Scenario: Set a tuning value with a slider
- **WHEN** the operator releases the Kp, Ki, acceleration, deceleration or start throttle limit
  slider, or taps its "−" / "+" step button
- **THEN** the page SHALL send ONE `set_gov_kp`, `set_gov_ki`, `set_gov_acc`, `set_gov_dec` or
  `set_gov_launch` with the value in firmware units (% per m/s, %/s per m/s, m/s², m/s², %); nothing
  SHALL be sent while the slider is being dragged
- **AND** each row SHALL show its label, the current value (with its unit for acceleration,
  deceleration and the start throttle limit; decimal comma in Ukrainian), a slider with the range
  ends written under its two ends, and a "−" and a "+" button that move the value one step
- **AND** the sliders SHALL span Kp 0.2–10 step 0.1, Ki 0–5 step 0.1, acceleration 0.3–3.0 m/s²
  step 0.1, deceleration 0.5–5.0 m/s² step 0.1 and the start throttle limit 1–40 % step 0.5, so the
  page cannot produce a value outside the range
- **AND** on success the reply SHALL name the stored value ("Governor Kp set to 1.50", …), Kp and Ki
  with two decimals, the others with one
- **AND** a NaN or out-of-range value SHALL be answered with an error naming the range and SHALL
  leave the stored value unchanged

#### Scenario: Sliders follow the stored values
- **WHEN** a telemetry frame is rendered
- **THEN** each slider and its value SHALL show its `gov_*` value, except a slider that is being
  dragged or whose sent value telemetry has not yet confirmed (at most 3 s); a stored value off the
  slider's step grid SHALL be shown as stored in the value text
- **AND** before telemetry carries the `gov_*` values the sliders and step buttons SHALL be disabled
  and show `--`

#### Scenario: Refused or unsent value
- **WHEN** the firmware refuses a governor value, or the page has no connection when a slider is
  released or a step button tapped, or the connection drops before the value is confirmed
- **THEN** the slider and its value SHALL return at once to the value in use from telemetry, and
  the card's status line SHALL show the error (or "not sent: no connection")

#### Scenario: Hints and help
- **WHEN** the card is shown
- **THEN** the acceleration and deceleration rows SHALL show their km/h-per-second equivalent,
  updated live while the slider is dragged, the
  card SHALL say that the throttle input becomes a speed from 0 to the speed limit, that the
  governor never brakes, and that without a speed signal the throttle is direct (full stick = full
  throttle)
- **AND** the card SHALL recommend, as advice and not as an interlock, tuning at the 10 km/h limit
  first and driving the low limits (10 and 20 km/h) in L on the buggy
- **AND** the help (ⓘ) SHALL describe each setting, its range, the field tuning order (what to
  lower when the speed hunts, what to raise when it is sluggish), the throttle-resolution limit
  and the autopilot-mode limitation
- **AND** the switch and each of the five setting rows SHALL carry its own small help (ⓘ) icon beside
  its label, with a finger-sized tap area, opening a help for that one setting in the same shape:
  what it is in one plain sentence, what the vehicle does when the value goes up ("Більше →" /
  "More →") and down ("Менше →" / "Less →"), the symptom that calls for changing it with the
  direction and step, and for Kp and Ki the rule Ki = Kp / 3
- **AND** the card and the help SHALL NOT state a default value as a number, because the defaults
  differ per vehicle while the page is shared; the sliders show the values in use
- **AND** every new label, hint, message and help text SHALL exist in the English and the Ukrainian
  dictionary (the buggy is "багі" in Ukrainian)

#### Scenario: Throttle rate label covers both
- **WHEN** the "Speed limit and steering" card is shown
- **THEN** the `thr_slew` row SHALL be labelled "Плавність газу (обмежувач і регулятор)" /
  "Throttle rate (limiter and governor)", with the command `set_speed_limit_slew` and key `thr_slew`
  unchanged

#### Scenario: Web throttle shows the commanded speed
- **WHEN** the governor is ACTIVE and the web throttle slider is shown
- **THEN** the slider value SHALL be followed by the setpoint in km/h from `gov_sp`
