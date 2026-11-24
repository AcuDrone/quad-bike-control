## ADDED Requirements
### Requirement: Parking Brake Card Service-Brake Release Help
The help (ⓘ) of the Parking brake card ("🅿️ Стоянкове гальмо" / "🅿️ Parking brake") SHALL state, in
uk and en, that:
- pressing the service brake (RC handset, GCS joystick or the web Brake control) releases an applied
  parking brake once the service brake has reached the pressed position;
- this includes the **automatic brake on RC loss**: the parking brake then releases, and when the
  automatic brake ends (15 s by default) the buggy ("багі") is held by nothing;
- a brake already held when the parking brake is applied does not release it; let go and press again.

The card SHALL get no new setting, switch or status line for this feature.

#### Scenario: Help text present
- **WHEN** the operator opens the Parking brake card's help
- **THEN** it SHALL contain the service-brake release rule and the RC-loss warning in the current
  language

#### Scenario: No new controls
- **WHEN** the card is rendered with firmware 1.5.37
- **THEN** it SHALL show the same controls and status lines as before, apart from the help text
