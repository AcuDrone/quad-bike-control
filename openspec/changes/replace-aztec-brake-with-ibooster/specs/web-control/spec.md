## ADDED Requirements
### Requirement: Service Brake Card
The web portal SHALL provide, in the Calibration section, a card "🛑 Гальмо (iBooster)" / "🛑 Brake
(iBooster)" in place of the former brake tuning card, to set the maximum stroke, the apply rate and
the release rate, and to show the unit's state. Its commands SHALL be routed through
`WebPortal::WebCommand` → `VehicleController::processWebCommand`, SHALL be accepted regardless of the
active input source, and SHALL persist before applying.

#### Scenario: Sliders send on release
- **WHEN** the operator drags one of the sliders — maximum stroke 2.0–16.0 mm step 0.5, apply rate
  1–40 ml/s step 1, release rate 1–40 ml/s step 1, defaults 16.0 mm / 8 / 8 — and releases it
- **THEN** the value label SHALL follow live while dragging and no command SHALL be sent before the
  release
- **AND** on release one command SHALL be sent: `set_brake_max_mm`, `set_brake_apply_rate` or
  `set_brake_release_rate`
- **AND** the firmware SHALL validate, persist, apply and answer "Brake max stroke set to N mm",
  "Brake apply rate set to N ml/s" or "Brake release rate set to N ml/s"
- **AND** an out-of-range value (for the maximum stroke also one off the 0.5 mm grid) SHALL be
  answered "Brake … out of range (…)" and a failed write
  "Failed to save the brake setting"

#### Scenario: Refused while braking
- **WHEN** a setting arrives while a brake request is active
- **THEN** the firmware SHALL answer "Brake settings: not while braking" and change nothing

#### Scenario: Status lines
- **WHEN** telemetry arrives
- **THEN** the card SHALL show the brake state in words, the unit's status / internal state /
  driver_brake_apply numbers, the rod position in mm, whether the unit sees our frames and the
  available pressure in bar
- **AND** the sliders SHALL show the values in use unless being dragged

#### Scenario: Help and translations
- **WHEN** the operator opens the card help (ⓘ)
- **THEN** it SHALL explain that 100 % brake is the maximum stroke, that the rates limit how fast the
  rod moves when applying and releasing, that there is no service brake with the ignition off or on
  fail-safe, and what to do on a fault (ignition off and on; if that does not help, remove the
  iBooster's power)
- **AND** every new label, hint, message and help text SHALL be present in both the English and the
  Ukrainian dictionary, the Ukrainian text calling the vehicle "багі"

## MODIFIED Requirements
### Requirement: Parking Brake Card Service-Brake Release Help
The help (ⓘ) of the Parking brake card ("🅿️ Стоянкове гальмо" / "🅿️ Parking brake") SHALL state, in
uk and en, that:
- pressing the service brake (RC handset, GCS joystick or the web Brake control) releases an applied
  parking brake once the service brake reports that it has reached the pressed position;
- with the ignition off (the iBooster asleep), or with the service brake unavailable, a press
  releases the parking brake at once while the service brake does **not** hold: the buggy ("багі")
  is then held by nothing;
- this includes the **automatic brake on RC loss**: the parking brake then releases, and when the
  automatic brake ends (15 s by default) the buggy ("багі") is held by nothing;
- a brake already held when the parking brake is applied does not release it; let go and press again.

The card SHALL get no new setting, switch or status line for this feature.

#### Scenario: Help text present
- **WHEN** the operator opens the Parking brake card's help
- **THEN** it SHALL contain the service-brake release rule, the no-brake warning for an unavailable
  service brake and the RC-loss warning in the current language

#### Scenario: No new controls
- **WHEN** the card is rendered
- **THEN** it SHALL show the same controls and status lines as before, apart from the help text

## REMOVED Requirements
### Requirement: Brake Tuning Card
**Reason**: The travel time and hold drive belonged to the removed X7 actuator.
**Migration**: Replaced by "Service Brake Card" in the same Calibration position; its commands
`set_brake_travel_ms` and `set_brake_hold_pwm` are removed and answered as unknown commands.
