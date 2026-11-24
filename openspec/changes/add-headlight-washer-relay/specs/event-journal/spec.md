## ADDED Requirements

### Requirement: Headlight Washer Journal Events
The event journal SHALL record headlight-washer pulses under the `RELAY` source, without adding
lines for normal operation beyond one per accepted request:
- `RELAY I washer_on src=<web|gcs> ms=<pulse length> restart=<0|1>` — journaled by
  `VehicleController` for every accepted wash request;
- `RELAY I washer_abort why=<ign_off|crank|all_off|fault|gcs_stop|not_fitted> left_ms=<remaining>` — journaled
  by `RelayController` when a running pulse is ended before its time.

The normal end of a pulse and refused requests SHALL NOT be journaled (including the "not fitted" refusal; a refusal is answered to the
web or GCS requester and logged on the serial console under the `RELAY` / `VEHICLE` debug features).
The existing deduplication (one `(source, event)` pair per `JOURNAL_DEDUPE_MS`) SHALL bound repeated
presses.

#### Scenario: Pulse from the web
- **WHEN** a web wash request is accepted with a 3000 ms pulse
- **THEN** one line `RELAY I washer_on src=web ms=3000 restart=0` SHALL be journaled
- **AND** no line SHALL be journaled when the pulse ends after 3000 ms

#### Scenario: Pulse aborted by ignition OFF
- **WHEN** the ignition is switched `OFF` 1000 ms into a 3000 ms pulse
- **THEN** `RELAY I washer_abort why=ign_off left_ms=2000` (± one loop) SHALL be journaled

#### Scenario: Repeated presses
- **WHEN** the operator presses the wash button five times within 1 s
- **THEN** only the first `washer_on` line SHALL be written and the next written `washer_on` line SHALL
  carry the suppressed count
