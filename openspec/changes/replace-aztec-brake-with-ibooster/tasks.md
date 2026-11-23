## 0. Preconditions
- [x] 0.1 Owner answers to Q1–Q6 received 2026-11-23 and folded into design.md (D4, D6, D7, D8, D10,
  D12) and the deltas.
- [x] 0.1a Proposal approved by the owner in this form (and, after the review fixes and notes 21–31, the commit).
- [ ] 0.2 `move-can-rx-to-interrupt` implemented on `master`, bench-checked (its §10, in particular
  10.1: GPIO38 follows MCP2515 `INT`, `can_int = ok`) and merged into `aztec`. Work on branch `aztec`
  only; never cherry-pick into `master` or `old-board`.
- [x] 0.3 `move-aztec-vesc-to-can` is NOT implemented before this change; note in its proposal that
  it must be re-based on this change (Hardware Configuration, Shared-Bus Steering Angle Frame
  Reception, version).
- [x] 0.4 Version 1.6.0 on `aztec` (owner Q6), after the prerequisite is merged in as 1.5.38; check
  that no change in `openspec/changes/` claims it; `master` stays on 1.5.x, its next minor is 1.7.0.
- [ ] 0.5 The iBooster is wired per design D1 / spec "Bosch iBooster Gen2 Service Brake Unit" and the
  termination is measured (9.1) before the first power-up with the firmware.

## 1. Pure protocol and law (header-only, `<stdint.h>` only, no Arduino, no `Constants.h`)
- [x] 1.1 `include/IboosterProto.h`, ported from `Arduino/iBooster-TEST/src/ibooster_proto.h`:
  IDs, `ibCrc8()`, `ibSum39D()`, `ibChecksumOk(id, d, dlc)`, flow constants and `ibClampFlow()`,
  `ibPmcBytes()`, `ibBuild(cnt, req, flow, speedByte, b, c, d)` (speed byte in `0x38B` byte 2, `0x05`
  in byte 3, `0x40` request bits only when `req`, `0x80` never, `0x38D` = `00 00 00 50 47`),
  `ibDecode39D()`, `ibDecode38E()` (ext status nibble, fields A / B in 0.01 mm, quality nibbles),
  `ibDecode38F()` (pRunout byte 3, partner byte 4), `ibRodCentiMm()`. Bench-only helpers
  (`ibBuildEx*`, dynamic pressure) are not ported.
- [x] 1.2 `include/IboosterLaw.h`: `IbState`; `IbCommand`, `IbUnitView`, `IbStepOut`;
  `ibTargetCmm(pct, maxMm10)` (clamp to 16.00 mm); `ibFlowUnits(targetCmm, rodCmm, applyUnits,
  releaseUnits)` (kp 250 / mm, deadband 0.05 mm, clamps); `ibSpeedByte(speedMs, valid)`;
  `ibTxEnabled(ignOn, msSinceUnitFrame)`; `ibStep(prev, cmd, unit, nowMs)` → next state, request bit,
  flow, events, per design D4 / D5 / D7 (READY condition, status 6 = ready, FAULT / LOST entry and exit
  on unit evidence only, release hand-off at 1.0 mm, release timeout 3 s, accept timeout 1 s,
  start / no-unit / fault-persist timeouts 20 s, crank window 3 s, stale command 500 ms holds the last
  target); `ibReached(...)` (0.2 mm or stall 0.05 mm / 300 ms or pRunout 0). Constants as `IB_LAW_*`
  literals, mirrored by `static_assert` against `Constants.h` in `src/IboosterBrake.cpp`.
- [x] 1.3 `include/ParkBrakeLaw.h`: `parkBrakeSvcStep()` takes `bool reached, bool available` instead
  of `estPct`; fire = `pending && (reached || !available)` (owner Q1 (b)); `PB_LAW_SVC_TOL_PCT` and
  its `static_assert` removed; edge / source / cancel rules unchanged.
- [x] 1.4 Delete `include/BrakeLaw.h` and `test/test_brake_law/`.

## 2. Native tests
- [x] 2.1 `test/test_ibooster_proto` (reuse the bench vectors): the 48 trace frames (CRC table for
  counter 0–15, `ibBuild(n, true, 0x8300, 0)` byte for byte); idle frames have no request bits and
  `0x7E00`; flow clamp `0xFFFF` → `0x9200`, `0` → `0x6A00`, bit `0x80` never set; the ten real
  `0x39D` frames pass the sum and fail J1850, a corrupted one fails; decode of `F8 50 E1 27 00`
  (st 5, cnt 0, drv 1, is 0, raw 319) and `68 6B 35 28 00` (st 6, is 5, raw 321); real `0x38E`
  `F5 29 40 11 40 11` / `94 25 40 11 4F 11` and `0x38F` `90 76 42 32 80 00` / `A4 70 42 32 40 1F`
  pass J1850, partner 0 / 1, field A of `F5 29 40 11 …` = 0.00 mm, ext status 2; `ibPmcBytes(117,1)`
  = `50 47`; speed byte lands in `0x38B` byte 2 with a valid CRC.
- [x] 2.2 `test/test_ibooster_law`: every transition of the D4 table, including status 6 `is=5`
  staying READY, a reboot-type `st=2` that clears by itself (FAULT → READY), a 20 s FAULT →
  `fault_latched` once, LOST at 251 ms not at 250 ms, ASLEEP only with ignition OFF and 500 ms of
  silence, frames enabled by "heard" with ignition reported OFF; percent → rod (defaults: 5 % of 16 mm =
  0.80 mm, 100 % = 16.00 mm; 50 % of 8 mm = 4.00; clamp at 16.00); default settings 16.0 mm / 8 / 8 ml/s
  (8 ml/s = 1024 flow units); flow (deadband, kp, both caps, ml/s → units); release
  hand-off at 1.0 mm and the 3 s timeout; target > 0 while LOST → no request, READY again → BRAKING
  (owner Q2 (a)); stale command at 500 ms keeps the target; crank flag on events inside 3 s; `reached` by
  tolerance, by stall and by pRunout 0, never outside BRAKING; speed byte (invalid → 0, 1.2 km/h → 3,
  clamp 255).
- [x] 2.3 `test/test_park_brake_law`: existing cases rewritten with `reached` / `available`; new: a
  press with the brake unavailable fires on the same step; a pending press fires when the brake
  becomes unavailable; with the brake available it fires on the first `reached` only; a non-zero →
  non-zero change and a FAILSAFE-frozen target still never fire.
- [x] 2.4 `platformio.ini` `[env:native]`: add `test_ibooster_proto`, `test_ibooster_law`, remove
  `test_brake_law`, update the suite comment. `pio test -e native` green.

## 3. CAN task (`src/CANController.cpp`, `include/CANController.h`)
- [x] 3.1 `ICanPeriodicClient` (`onSlot()`, `onFrame()`), `attachPeriodicClient()`; called only from
  the task; documented as such.
- [x] 3.2 Slot clock `IB_SLOT_MS` (10): wait = min(`CAN_POLL_MS`, time to the slot); one send per
  slot, no catch-up; gap measured.
- [x] 3.3 TXB1 (TXP 2) / TXB2 (TXP 3) for the client: abort a buffer still pending from the previous
  slot (its own TXREQ only, count `tx_miss`), load `0x38D` → TXB2 and `0x38C` → TXB1, load `0x38B`
  into TXB1 on the first cycle it is free (1-tick wait while pending); TXP re-written after every
  (re-)init; TXB0 / general queue untouched; CANINTE unchanged (`0x03`).
- [x] 3.4 `applyRxFilters()`: RXF2 `0x305`, RXF3 `0x38E`, RXF4 `0x38F`, RXF5 `0x39D`; constants
  `IB_ID_*`; filter log line updated.
- [x] 3.5 Drain: offer each frame to `onFrame()` first; consumed frames skip the RX ring except while
  the sniffer window is open.
- [x] 3.5a Before every task command (re-init, filter switch): clear TXREQ on TXB1 / TXB2 as well as
  TXB0 (per buffer), count brake aborts as `tx_miss`; no brake frame loaded while a command runs or
  the chip is down; TXP re-written after success; first slot after it built from the current command.
- [x] 3.6 While the client reports a request active: LWS-silence and OBD no-data watchdog re-inits are
  postponed in the loop's trigger path (trigger kept, posted on the first `update()` after the
  request; a sequence already in flight continues); bus-off recovery and the task's init retry
  while the chip is down (boot or after a failed re-init sequence) unchanged; `startSniff()` refused with a new reason `BRAKE` (web reply "CAN sniffer: not while
  braking"); a sniff already open runs to its end with its closing filter switch.
- [x] 3.7 Grep check: `TXB1` / `TXB2` and the client hooks appear only in task code paths.

## 4. Brake component and vehicle wiring
- [x] 4.1 `include/IServiceBrake.h`; `include/IboosterBrake.h` / `src/IboosterBrake.cpp`: settings
  load (NVS `brake` `max_mm10`, `apply_mlps`, `release_mlps`; out-of-range → default, logged, not
  rewritten), one-time removal of `travel_ms` / `hold_pwm`, setters (validate, persist first, refuse
  while a request is active), `update()` (publish `IbCommand` under `portMUX`, read `IbSnapshot`,
  drain the event ring, journal per D10), getters for telemetry; the task-side `onSlot()` /
  `onFrame()` (decode, `ibStep()`, build frames).
- [x] 4.2 `include/Constants.h`: `IB_*` constants (slot 10, silent 500, status stale 250, start /
  no-unit / fault-persist 20000, accept 1000, crank grace 3000, cmd stale 500, gap warn 50, statustext hold-off 5000, kp 250,
  deadband 0.05, reached 0.2, stall 0.05 / 300, release hand-off 1.0 / timeout 3000, rod ceiling
  16.0), settings ranges / defaults / NVS keys, `OTA_MAX_SPEED_KMH` 1.0, each with a one-line reason
  and the bench figure; removal list of design D11.
- [x] 4.3 `VehicleController`: `IServiceBrake&` replaces `BTS7960Controller& brake_`; `applyBrake()`
  keeps the clamp and calls `setTargetPct()`; `serviceBrake_.update()` at the old
  `updateBrakeControl()` place with ignition, cranking, speed; `applyFailsafe()` loses the two
  `brake_.stop()` lines and nothing is added; `updateParkBrakeServiceRelease()` passes
  `serviceBrake_.reached()`; web commands `set_brake_max_mm`, `set_brake_apply_rate`,
  `set_brake_release_rate` (rates 1–40 ml/s); `set_brake` reply adds "brake unavailable" when not available; old
  brake members, getters, setters and commands deleted.
- [x] 4.4 `src/main.cpp`: delete the X7 instance and the endstop boot line; construct `IboosterBrake`,
  attach it to `CANController` before `begin()`; boot log line with the settings.
- [x] 4.5 `BoardInputs`: drop `brakeReleased`; bit 4 unused.

## 5. Flash and OTA discipline
- [x] 5.1 `FlashHold` (one flag set by the loop from `requestActive()`): `SpeedSensor` periodic
  `persistDistance()` and `EngineHourMeter` periodic `persist()` skip while held and run on the first
  update after; explicit and fail-safe persists unchanged.
- [x] 5.2 `EventJournal::service(steeringDriving || brakeRequestActive)`; same 10 s cap.
- [x] 5.3 Web `/update` (firmware and LittleFS): refuse before `Update.begin()` while a request is
  active or speed valid > `OTA_MAX_SPEED_KMH`, reply "OTA refused: brake applied or vehicle moving";
  `ArduinoOTA.handle()` skipped under the same condition (one log line per episode); journal clear
  refused under the same condition.

## 6. Web and telemetry
- [x] 6.1 Telemetry struct / JSON: remove `brake_travel_ms`, `brake_hold_pwm`, `brake_released`; add
  the keys of design D10; `brake_pct` = measured.
- [x] 6.2 `data/index.html`: replace the brake card (Calibration) per the web-control delta; tile;
  attention items; Tests slider state text; i18n uk + en ("багі"); delete the old keys and reply
  routing.
- [x] 6.3 Park-brake card help: the hand-over waits for the brake to report reached while it is
  available; with the ignition off or the brake unavailable a press releases at once and the buggy is
  held by neither brake (owner Q1 (b)).
- [ ] 6.4 Telemetry capacity comment updated; worst-case document measured to fit 9216 B with headroom.
  (Comment updated; the document is 10240 B on aztec since 1.5.36 and the fit is an estimate from the
  slot count, ≈ 9.38 KB — not measured. Measure on the board with the "[WEB] telemetry JSON peak"
  console line, CAN sniffer + ECU probe present.)

## 7. Journal
- [x] 7.1 `JSrc::BRAKE` and token `BRAKE` (`include/EventJournalLaw.h:29`, `jSrcToken()`); journal
  tests updated (`test_event_journal`).
- [x] 7.2 Lines of design D10 from `IboosterBrake::update()`; `fault` / `lost` paired with
  `fault_raw`; `stats` every `JOURNAL_SNAP_MS` while the ignition is on.

## 7a. MAVLink
- [x] 7a.1 `MavlinkInterface::report()` struct gains the brake availability (LOST / FAULT / OK
  flags from `IboosterBrake`); `src/MavlinkInterface.cpp` sends "Brake: LOST (no iBooster)" /
  "Brake: FAULT (iBooster)" (WARNING) and "Brake: OK" (INFO, only after an announced LOST/FAULT)
  through the shared `MAVLINK_STATUSTEXT_MIN_MS` limiter plus `IB_STATUSTEXT_MIN_MS` (5 s, latest
  state replaces the pending text, a recovery cancelling an unsent warning sends nothing).
- [x] 7a.2 Pure decision in `include/IboosterLaw.h` (`ibStatustextStep()`), native-tested: flapping
  LOST/READY every 1 s for 20 s → ≤ 5 texts, last one = current state; LOST then READY within the
  hold-off → nothing; READY without an announced warning → nothing.

## 8. Docs, changelog, version, checks
- [x] 8.1 `FIRMWARE_VERSION` per 0.4; `include/Changelog.h` entry (uk + en, operator wording): the
  buggy's service brake is now the iBooster; new brake card in Calibration (max stroke, apply / release
  rate) — set it up; no brake with the ignition off; park brake releases by the brake only with the
  brake working; OTA refused while braking or moving; upload the LittleFS image.
- [x] 8.2 `GPIO_PINOUT_CUSTOM_BOARD_S3.md`: X7 free, GPIO6/7 and LEDC 4/5 free, In5 spare, X150 table
  (pins 1 / 9 / 20 / 18 / 10; 25 / 16 not connected; X158 stays connected), X13 bus members (ECU,
  LWS, iBooster) and the termination result of 9.1 (placeholder "not yet measured" until 9.1 is
  done), RXB1 filter plan; `WEB_PORTAL_SETUP.md` (brake
  card, tile, attention, park-brake hand-over, OTA refusal); `openspec/project.md` aztec hardware line.
- [x] 8.2a `MAVLINK_SETUP.md`: the three brake `STATUSTEXT`s (text, severity, rate limit, aztec only).
- [x] 8.2b Per the project rule (every `MAVLINK_SETUP.md` change), update the API compatibility page
  `UGV/API/README.md` in the parent folder: the brake `STATUSTEXT` item → firmware 1.6.0 → aztec →
  Mission Planner / QGC support (both show `STATUSTEXT`).
- [x] 8.3 `pio run` (aztec env) without new warnings; `pio test -e native` green;
  `openspec validate replace-aztec-brake-with-ibooster --strict`.

## 9. Bench and vehicle (separate from §1–8; nothing here is done by code). Wheels off the ground
until 9.6.
- [ ] 9.1 Termination: power off, measure X150 pin 18–10 on the unit alone (≈ 120 Ω = internal
  terminator, open = none), then CAN-H–CAN-L on the assembled bus. Pass: ≈ 60 Ω with board, ECU, LWS
  and unit connected; record which nodes terminate; fix before power-up.
- [ ] 9.2 First power-up, ignition ACC: journal `BRAKE I state … to=READY` within 20 s; telemetry
  `ib_partner` true; `CAN I stats` `ovr=0`, `txf=0`; `ib_tx_miss` 0 over 10 min; `can_int = ok`;
  ignition OFF → `ASLEEP`, no frames (sniffer from a laptop or `tx` stops in the stats line).
  Watch `CAN I stats` `ovr` for 10 minutes with the unit awake: occasional RXB1 overflows (0x305 /
  0x38E collisions) are accepted, a steady rate is not (design note 31). Ignition OFF → ON within
  0.5 s: no `lost` line, no "Brake: LOST" (note 25).
- [ ] 9.3 Recovery from `failure`: provoke or wait for a `FAULT` (e.g. pull X150 pin 1 briefly while
  holding 4 mm), then ignition OFF ≥ 10 s and ON. Record whether the unit returns to READY; if not,
  pin-1 power cycle. Pass: the journal has `fault` + `fault_raw` + `vbat`, the attention box shows it,
  no request is sent while FAULT, and `recovered` after the operator action. Result goes into the
  operator docs.
- [ ] 9.4 Cranking: ten starts with the brake idle and five with 30 % held. Record battery minimum,
  unit status sequence, LOST / FAULT lines (`crank=1`), time back to READY. Pass: no `fault_latched`,
  no E-level brake line, brake resumes by itself (owner Q2 (a)).
- [ ] 9.5 First fill and bleed: before the first brake command on the vehicle, lower the maximum
  stroke (e.g. 4 mm) and the rates (e.g. 2 ml/s) on the brake card — the default 100 % is the full
  16 mm request stroke at 8 ml/s — and raise the stroke step by step (2 mm at a time). Fill and bleed
  the master cylinder and the circuit with the unit powered from the battery; slow apply / release
  cycles up to the stroke where pressure builds; no supply reboot (no counter restart in `0x39D`).
- [ ] 9.6 Stroke vs deceleration with fluid, on the ground: start the first drive with the lowered
  maximum stroke from 9.5 and raise it step by step towards the 16 mm default; record rod mm at first drag, at locked
  wheels standing, and stopping distance from 10 / 20 km/h at 50 % and 100 % for candidate max
  strokes; choose `max_mm`, `apply_mlps`, `release_mlps` (defaults 16 mm / 8 / 8 ml/s); check that
  `reached` fires (tolerance, stall or `0x38F` byte 3 = 0) at each, including 100 % at 16 mm; record
  the time from command to `reached` at 100 % (≈ 0.3 s acceptance + ≈ 1.5 s expected at 8 ml/s). When raising the rates (up to 40 ml/s), record the battery voltage at fast
  releases from full stroke (regeneration) and any unit reboot.
- [ ] 9.7 Speed byte: drive at 2–5 km/h with the brake idle and watch `ib_runout` switch 51 → 103 (or
  record the speed at which it does). Pass: no request drop caused by the speed fields.
- [ ] 9.8 Frame gaps on the vehicle bus with ECU and LWS running: hold 30 %; test build or sniffer
  confirms the slot jitter (`ib_gap_max_ms` — the interval between two sent slots, 10 ms nominal —
  ≤ 15 ms in normal running; design implementation note 12); a forced 150 ms task pause
  (test build) shows no release, a forced 300 ms pause shows release, `unit_drop` and re-apply by the
  loop. LWS `miss` and OBD unaffected.
- [ ] 9.9 Flash with a brake held (30 %, standing): journal flash writes ON, 10 gear changes (NVS),
  a park-brake apply / release, a settings save on another card; brake card save refused; web OTA,
  ArduinoOTA and journal clear refused. Pass: no `unit_drop`, `ib_gap_max_ms` < 100, the deferred
  journal flush happens within 10 s, odometer / hours written after the release.
- [ ] 9.10 Journal content on a forced fault (9.3) and a forced LOST (unplug X150 CAN while READY):
  lines and fields as design D10; `lost` with `fault_raw`; `recovered` after replug.
- [ ] 9.11 Fail-safe with a held brake (MAVLink cut, no browser): record that the brake drops when the
  ignition opens and the rod returns; supply voltage at that moment (regeneration). This documents the
  accepted limitation; no pass / fail on the behaviour itself.
- [ ] 9.12 Park brake: APPLIED, ignition ACC, unit READY, apply rate 1 ml/s, web brake 20 % and 50 % →
  the release must NOT start before the rod (`brake_rod_mm`) has moved (take-up, design note 21),
  then starts when `reached`; ignition OFF → a press releases at once (owner Q1 (b)); unit in FAULT / unplugged → a
  press releases at once. Wheels chocked for the last two.
- [ ] 9.12a Release at 1 and 2 ml/s from the full stroke: no `release_slow`, the rod comes down under
  request to 1 mm (note 22). Web firmware upload started, then a web brake: upload aborted with the
  reason on the page, old firmware still running (note 23). (Do not test the LittleFS abort without a
  USB cable at hand.)
- [ ] 9.13 GCS: unplug X150 CAN while READY → "Brake: LOST (no iBooster)" WARNING in Mission Planner /
  QGC; replug → "Brake: OK"; plug / unplug every 1 s for 20 s → at most one brake text per 5 s.

## 10. Archive
- [ ] 10.1 Archive after `add-brake-tuning-card`, `add-aztec-parking-brake`,
  `repurpose-x10-battery-sense`, `add-park-brake-release-by-service-brake`,
  `add-event-journal-and-link-recovery` and `move-can-rx-to-interrupt`, once §9 passes;
  `openspec validate --strict` after archiving.
