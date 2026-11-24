## ADDED Requirements

### Requirement: Headlight Washer Control via Web Interface
The Tests section card "Vehicle Ignition & Lighting" SHALL hold a "Омити фари" / "Wash headlights"
button under the front-wheel lock button, sending the WebSocket command `{"cmd":"wash_headlights"}`
(no value). The command SHALL be handled in `VehicleController::processWebCommand()` through the
queued web-command path into the `RelayController` wash request, and SHALL NOT be gated by the input
source (like `set_light` and `set_wheel_lock`). Replies:
- success: "Headlight washer ON for N.N s" (a restart: "Headlight washer restarted for N.N s");
- refusals: "Headlight washer refused - washer not fitted - enable it in Calibration", "Headlight
  washer refused - ignition is OFF", "Headlight washer refused - engine start in
  progress", "Headlight washer refused - relay board unavailable".

There SHALL be no web stop button: the pulse ends by itself (at most 20 s) and every ignition OFF
ends it.

#### Scenario: Wash from the web
- **WHEN** the operator presses the button with the ignition `ACC` and the pulse length 3000 ms
- **THEN** `wash_headlights` SHALL be sent, the reply "Headlight washer ON for 3.0 s" SHALL be shown
  under the button and Relay_8 SHALL be energized for 3 s

#### Scenario: Refusal text with the ignition OFF
- **WHEN** the operator presses the button with the ignition `OFF`
- **THEN** the reply "Headlight washer refused - ignition is OFF" SHALL be shown under the button in
  the error colour and nothing SHALL switch

#### Scenario: Button shows the running pulse
- **WHEN** telemetry reports `washer_on` true
- **THEN** the button SHALL be highlighted and show the remaining time from `washer_left_ms`
  ("2,1 с" / "2.1 s"); it SHALL return to normal when `washer_on` is false
- **AND** pressing it again while it runs SHALL restart the pulse

#### Scenario: Button while the washer is not fitted
- **WHEN** telemetry reports `washer_fitted` false (or the key is absent)
- **THEN** the button SHALL be disabled and greyed with the hint "Омивач не ввімкнено — Калібрування →
  «Омивач фар»" / "Washer not enabled — Calibration → Headlight washer"
- **AND** a `wash_headlights` command sent anyway (e.g. from a script) SHALL get the reply "Headlight
  washer refused - washer not fitted - enable it in Calibration" and no journal line

#### Scenario: Hint while the ignition is OFF
- **WHEN** the ignition state in telemetry is `OFF`
- **THEN** a muted hint "Працює лише з увімкненим запалюванням" / "Works only with the ignition on"
  SHALL be shown under the button, and the button SHALL stay clickable while the WebSocket is
  connected so the firmware's refusal text is shown

### Requirement: Headlight Washer Pulse Length Configuration via Web Interface
The Calibration section SHALL contain a card "💦 Омивач фар" / "💦 Headlight washer" (directly after
the "Engine start" card). At the top of the card a toggle "Омивач встановлено" / "Washer fitted"
SHALL send `{"cmd":"set_washer_fitted","value":<true|false>}` (any input source, persisted first and
applied at once, no reboot; replies "Headlight washer fitted: ON" / "Headlight washer fitted: OFF" /
"Failed to save the washer fitted setting"); like the gear-boost and speed-governor switches its
displayed state SHALL be taken from the telemetry `washer_fitted`, not from the click. Below it, a
slider "Тривалість омивання" / "Wash duration": seconds 2–20, step
0.5, value shown on the right ("3,0 с" / "3.0 s"). Releasing the slider SHALL send exactly one
`{"cmd":"set_washer_pulse_ms","value":<round(seconds × 1000)>}`; nothing SHALL be sent while it is
dragged, and telemetry SHALL NOT move the thumb while it is held. The firmware SHALL round the value
to a whole ms, refuse `NaN` and values outside 2000–20000 ms, persist first and then apply, and
SHALL accept the command from any input source. The slider SHALL stay editable while the washer is
not fitted (the length is kept for later); the card SHALL then show a muted note "Омивач вимкнено —
тривалість не діє" / "Washer off — duration not in use". Replies: "Washer pulse set to N ms", "Washer pulse
out of range (2000-20000 ms)", "Failed to save the washer pulse". The card SHALL have its own reply
line, a hint line and an ⓘ help entry `washer_help` (uk + en) explaining: the fitted switch (off by default after the update; turn it on only on a vehicle with
the pump; off hides the function from the ground station; the relay test works either way), what
Relay_8 drives, that
one press gives one pulse of this length and a new press restarts it, that it works only with the
ignition on (ACC or engine running) and never during an engine start, that ignition OFF stops it,
the range 2–20 s and the default 3 s, and that a long pulse empties the washer reservoir and loads
the battery. The setting SHALL NOT be placed in a Tests-section card.

#### Scenario: Set the pulse length
- **WHEN** the operator releases the slider at 7.5
- **THEN** `set_washer_pulse_ms` with value 7500 SHALL be sent once and the reply "Washer pulse set to
  7500 ms" SHALL be shown in the card
- **AND** after a reboot the slider SHALL read 7.5 s

#### Scenario: Enable the washer from the card
- **WHEN** the operator turns "Washer fitted" on
- **THEN** `set_washer_fitted` true SHALL be sent, the toggle SHALL show on once telemetry reports
  `washer_fitted` true, and the Tests button SHALL become active

#### Scenario: Out-of-range value from a script
- **WHEN** `set_washer_pulse_ms` arrives with 1999, 20001 or `NaN`
- **THEN** the reply SHALL be "Washer pulse out of range (2000-20000 ms)" and nothing SHALL be stored

#### Scenario: Older firmware
- **WHEN** telemetry carries no `washer_pulse_ms` key
- **THEN** the slider SHALL keep 3 s and the card SHALL stay usable without errors

## MODIFIED Requirements
### Requirement: Web Relay Test Control
The web portal SHALL provide a relay test for wiring checks: one on/off button per relay
(Relay_1..Relay_8), sent as the WebSocket command
`{"cmd":"set_relay_test","relay":<1..8>,"value":<true|false>}` and handled in
`VehicleController::processWebCommand()` through the queued web-command path into
`RelayController::setRelayTest()`. The command SHALL NOT be gated by input source; its gate is the
relay-test gate (ignition `OFF`, no function relay energized — the lock included).

#### Scenario: Toggle a relay on from the web UI
- **WHEN** the operator clicks a relay's button while it is off, the ignition is OFF and the lock is
  released
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
- **WHEN** a `set_relay_test` ON command arrives while the ignition is not OFF, the front light is on
  or the lock is engaged
- **THEN** an error response starting with "Relay test:" SHALL be sent and no relay SHALL change

#### Scenario: Relay test card in the UI
- **WHEN** the Tests section is shown
- **THEN** a "Relay Test" card SHALL show eight equal-size buttons in a 2-column grid (1 column at
  768 px or less), styled like the gear buttons
- **AND** each button SHALL show "Реле N" / "Relay N" on its first line and the relay's function on
  a smaller second line — on the aztec buggy 1 ignition, 2 starter (max 2 s), 3 light, 4 lock (with
  5+6), 5 lock (with 4+6), 6 lock (with 4+5), 7 spare, 8 headlight washer ("омивач фар") — in
  English and Ukrainian ("блокування (з 5+6)" …; the quad labels likewise name Relay_8 the headlight
  washer and Relay_7 spare)
- **AND** the aztec label set SHALL be chosen when the telemetry carries the aztec-only `gear_codes`
  object, and the quad bike's set otherwise (`wheel_lock_fitted` is `true` on both vehicles and no
  longer tells them apart)
- **AND** an energized relay SHALL be shown with the same highlight as the active gear button and
  `aria-pressed="true"`; the button SHALL NOT carry ON/OFF text, and its tooltip SHALL name the
  relay, its function and its ON/OFF state
- **AND** each button's state SHALL follow the telemetry `relay_test` bitfield, not the last click
- **AND** a button that is off SHALL be disabled while the ignition is not OFF, the front light is
  on, the lock is engaged, the relay board is faulted or the WebSocket is disconnected; a button that
  is on SHALL stay clickable while connected so it can be switched off
- **AND** the response to a relay test command SHALL be shown in the card

