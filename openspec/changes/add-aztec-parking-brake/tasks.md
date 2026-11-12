## 0. Preconditions
- [x] 0.1 Proposal approved by the owner. The open questions were answered on 2026-11-12 and are folded into design.md (D11) and the specs.
- [ ] 0.2 `repurpose-x10-battery-sense` archived before this change (this change's `vehicle-actuators` "Hardware Configuration" delta is based on its text); note in `move-aztec-vesc-to-can` that its "Hardware Configuration" delta must be rebased onto this change's text.
- [x] 0.3 Work on branch `aztec` only; do NOT cherry-pick into `master` (X6 is the quad's steering VESC).

## 1. Constants and pure law
- [x] 1.1 `include/Constants.h`:
  - pins: `PIN_PARK_BRAKE_LPWM` GPIO_NUM_4 (X6 pin 2), `PIN_PARK_BRAKE_RPWM` GPIO_NUM_5 (X6 pin 3);
  - LEDC: `LEDC_CH_PARK_BRAKE_RPWM` 6, `LEDC_CH_PARK_BRAKE_LPWM` 7;
  - `PARK_BRAKE_DRIVE_DUTY` 255;
  - travel: `PARK_BRAKE_TRAVEL_MS_DEFAULT` 2000, `PARK_BRAKE_TRAVEL_MS_MIN` 1000,
    `PARK_BRAKE_TRAVEL_MS_MAX` 5000, `PARK_BRAKE_TRAVEL_MARGIN_MS` 500,
    `PARK_BRAKE_REVERSE_DWELL_MS` 200;
  - throttle release point: `PARK_BRAKE_THR_PASS_PCT_DEFAULT` 50, `PARK_BRAKE_THR_PASS_PCT_MIN` 20,
    `PARK_BRAKE_THR_PASS_PCT_MAX` 100;
  - `PARK_BRAKE_APPLY_MAX_SPEED_MS` 0.833f (3 km/h), `PARK_BRAKE_DRIVE_AWAY_THROTTLE_PCT` 10.0f;
  - NVS: `PARK_BRAKE_NVS_NAMESPACE` "pbrake", keys `PARK_BRAKE_STATE_NVS_KEY` "state",
    `PARK_BRAKE_TRAVEL_NVS_KEY` "travel_ms", `PARK_BRAKE_THR_PASS_NVS_KEY` "thr_pass",
    `PARK_BRAKE_INVERT_NVS_KEY` "invert";
  - EFI flags: `EFI_DIGITAL_FLAG_PARK_BRAKE` 0x04, `EFI_DIGITAL_FLAG_PARK_BRAKE_UNSETTLED` 0x08;
  - comments: update the header comment ("GPIO4/5 — FREE" → parking brake), the GPIO43/44 comment
    ("X6 (GPIO4/5) is freed for a 4th BTS7960"), the LEDC 6/7 "free for future use" comment and
    the "bit2+ reserved" comment.
- [x] 1.2 `include/ParkBrakeLaw.h` (header-only, no Arduino):
  - state enum and stored-code decode (1 → RELEASED, 2 → APPLIED, else UNKNOWN);
  - `parkBrakeRunMs(travelMs)`, `parkBrakeTravelMsValid()`, `parkBrakeTravelMsOrDefault()`;
  - command transition (state × request × speed check → action / result);
  - `parkBrakeApplySpeedOk(speedMs, valid, suspicious)` → OK / REFUSED / UNKNOWN_SPEED;
  - `parkBrakeThrPassMs(travelMs, pct)` (pct < 100 → travel × pct / 100; 100 → whole run),
    `parkBrakeThrPassPctValid()`, `parkBrakeThrPassPctOrDefault()`;
  - `parkBrakeHoldsThrottle(state, targetGear, releaseElapsedMs, thrPassMs)` (APPLIED / APPLYING /
    UNKNOWN hold in L/H/R; RELEASING holds until the release point);
  - drive-away latch step with the ignition input (arm at ≤ threshold while APPLIED/UNKNOWN and
    ignition on; disarm when ignition OFF; fire on > threshold in L/H/R with no gear change and
    ignition on, no debounce);
  - `pbrake_da_wait` reason (`none` / `ign_off` / `rearm`);
  - direction mapping (request × invert → sign).

## 2. Driver and controller
- [x] 2.1 `src/main.cpp`: second `BTS7960Controller parkBrakeActuator`; `begin(PIN_PARK_BRAKE_RPWM, PIN_PARK_BRAKE_LPWM, LEDC_CH_PARK_BRAKE_RPWM, LEDC_CH_PARK_BRAKE_LPWM)` next to the X7 brake (reuses `LEDC_TIMER_1`), `stop()` after; error log on failure; pass it to `VehicleController`.
- [x] 2.2 `include/ParkBrakeController.h` / `src/ParkBrakeController.cpp`:
  - load `"pbrake"` at begin (state decode, travel and release point or default with an
    out-of-range log, invert);
    no movement at boot;
  - `request(apply|release)` → result;
  - `update()` runs the timer and the 200 ms reverse dwell, stops at the end of the run and
    persists the settled state;
  - persist the moving state before driving (apply refused on a write failure; release proceeds
    with "state not saved");
  - getters: state, remaining ms, release drive elapsed ms, travel, release point, invert;
  - setters: `setTravelMs()` (persist first), `setThrPassPct()` (persist first, from the next
    release run), `setInvert()` (refused while running; persist invert,
    then state UNKNOWN);
  - `DebugFeature::BRAKE` logs: start, reversal, end, refusals, NVS failures.
- [x] 2.3 `VehicleController`:
  - own `ParkBrakeController`;
  - `commandParkBrake(request, source)`: driver check, speed interlock for APPLY (sensor valid ||
    suspicious, else "speed unknown"), returns the result code;
  - call `parkBrake_.update()` in `update()` next to `updateBrakeControl()`;
  - no park-brake call in `applyFailsafe()` (a run continues).
- [x] 2.4 `VehicleController` throttle paths (`processMavlinkCommands()`, the web re-apply in `update()`, `processThrottleCommand()`):
  - feed the clipped operator demand and the ignition state
    (`relayController_.getIgnitionState() != OFF`) to the drive-away latch; start a release when it
    fires; rate-limited `BRAKE` log for a press blocked by ignition off / no re-arm;
  - when `parkBrakeHoldsThrottle()`, evaluate `applySpeedLimit(demand)` but call `throttle_.idle()`
    instead of `setThrottlePercent()`;
  - web `set_throttle` reply while held: "Throttle set — held at idle until the park brake is
    released", or with the ignition off "Throttle set — held at idle: ignition off, the park brake
    stays applied";
  - an APPLY accepted while the throttle is passing (release past its point) puts the throttle back
    to idle in the same loop;
  - gear-change ownership (boost PID, shift idle hold) and throttle calibration unchanged.
- [x] 2.5 `VehicleController::processWebCommand`: `park_brake` (strValue "apply"/"release"), `set_park_brake_travel_ms` (round, 1000–5000), `set_park_brake_thr_pass_pct` (round, 20–100), `set_park_brake_invert` (bool) in the "any input source" block; reply texts exactly as in the `web-control` delta.

## 3. Telemetry
- [x] 3.1 `WebPortal::Telemetry` + `TelemetryManager` + `createTelemetryJSON`: `pbrake_state` (literal), `pbrake_remaining_ms`, `pbrake_thr_hold`, `pbrake_da_wait` (literal), `pbrake_travel_ms`, `pbrake_thr_pass_pct`, `pbrake_invert`; update the capacity comment (+7 slots, 9216 unchanged).
- [x] 3.2 `src/main.cpp` `report.digitalFlags`: OR in bit2 (APPLIED) and bit3 (APPLYING / RELEASING / UNKNOWN).

## 4. Web UI (`data/index.html`)
- [x] 4.1 Card `id="pbrake-card"` "🅿️ Стоянкове гальмо" / "🅿️ Parking brake" in the Tests section after "Vehicle Ignition & Lighting":
  - state line, remaining seconds, throttle-hold note;
  - Apply / Release through `confirmTwice()` with status line `pbrake-status`;
  - travel slider 1000–5000, step 100, default 2000, label on `input`, one send on `change`, dragging flag (as
    `brake-travel-slider`);
  - throttle release point slider 20–100 %, step 5, default 50, label "N % ≈ M ms" (or "until the
    run ends" at 100 %), one send on `change`, dragging flag;
  - notes for `pbrake_da_wait` (ignition off / drop the throttle and press again) and "throttle
    passing, brake still releasing";
  - direction toggle through `confirmTwice()`;
  - `steer_no_connection` when the WebSocket is closed;
  - card hidden without `pbrake_state`;
  - reply routing for "Park brake" / "Failed to save the park brake" / "Throttle set — held"
    replies;
  - help (ⓘ) text with the bench procedure.
- [x] 4.2 i18n en + uk: card title, state names, button and armed labels, three warnings, hold note, slider label, direction labels, help; "багі" in Ukrainian; no unused keys; `confirmRefreshLang()` covers the new armed buttons.

## 5. Tests, docs, release
- [x] 5.1 `test/test_park_brake_law/test_main.cpp` + `platformio.ini` `test_filter` line and header comment:
  - stored-code decode (0/3/4/5/missing → UNKNOWN);
  - run ms = travel + 500;
  - travel 999 / 5001 rejected, 1000 / 5000 accepted;
  - the full D4 transition table;
  - apply speed: 0.83 OK, 0.84 refused, never-pulsed → speed unknown, suspicious 2.0 refused;
  - hold predicate for every state × N/L/H/R/UNKNOWN; RELEASING at 2000 ms / 50 %: held at 999 ms,
    passing at 1000 ms; at 100 %: held until the run ends (2500 ms); release point 19 / 101
    rejected, 20 / 100 accepted;
  - ignition: no fire with ignition OFF (`da_wait` = `ign_off`); demand held at 30 % while
    switching ignition OFF → ON does not fire (`rearm`) until a dip to ≤ 10 %; no debounce (fires
    in the first sample above 10 %);
  - drive-away latch: fires on 0 → 30 in L, not on 8 in L, not in N, not during a gear change,
    not when the demand was already above 10 % as APPLIED settled, then fires after a dip to 5;
  - direction mapping with invert 0/1.
- [x] 5.2 `GPIO_PINOUT_CUSTOM_BOARD_S3.md`:
  - §1 overview, §2 GPIO4/5 rows ("`PIN_PARK_BRAKE_LPWM/RPWM` — LEDC ch7/ch6, aztec parking
    brake");
  - §3 X6 row (parking-brake BTS7960; IS on mux CH3/CH0, not sampled);
  - §4 UART2 row note and LEDC table (6/7 parking brake, 10 kHz, 8-bit; 0/3 free);
  - §9 new constants and CD4051 future note (X6 IS for park-brake current end-of-travel);
  - "aztec only — on master X6 is the steering VESC".
- [x] 5.3 `MAVLINK_SETUP.md` `pt_compensation` bitmask table: bit2, bit3, value range 0–15 (and fix the stale `Constants.h:351-352` line reference to point at the `EFI_DIGITAL_FLAG_*` block).
- [x] 5.4 `FIRMWARE_VERSION` → "1.5.29"; `include/Changelog.h` 1.5.29 entry, uk + en, operator wording:
  - new parking brake on X6, apply / release from the web card with two-press confirmation;
  - apply refused above 3 km/h;
  - releases by itself when the throttle rises above 10 % in L/H/R (web or Pixhawk) with the
    ignition on; no release in N or with the ignition off;
  - anti roll-back: the throttle is held at idle for the first part of the release (default half
    the travel, 1 s at 2 s) and then ramps up while the brake finishes releasing; the share is set
    on the card (20–100 %);
  - a throttle already held above 10 % while applying, or when the ignition is switched on, does
    not release it — drop the throttle and press again; the card says why;
  - works with the ignition off (the actuator is on permanent battery power);
  - state "Невідомо" / "Unknown" after the first flash or a power loss mid-travel;
  - LittleFS image must be uploaded;
  - set the direction and the travel time on the bench first (default 2000 ms is an estimate);
  - "багі" in Ukrainian.
- [x] 5.5 `pio test -e native` green; `pio run -e esp32-s3-devkitc-1` and `pio run -t buildfs` succeed.
- [x] 5.6 `openspec validate add-aztec-parking-brake --strict` passes.

## 6. Bench (buggy on stands, rear wheels free)
- [ ] 6.1 Upload firmware and the LittleFS image; the card shows "Невідомо" / "Unknown", 2000 ms, direction normal; the actuator does not move at power-up, at a reset button press or on an OTA reboot.
- [ ] 6.2 Direction: press Release (two-press) from Unknown. If the cable tightens, toggle the direction (state → Unknown) and press Release again. Record which X6 pin tightens (GPIO4 / GPIO5) in the pinout doc.
- [ ] 6.3 Travel time: time a full apply and a full release with a stopwatch (from the press to the limit-switch stop; actuator audibly stops). Set the longer one rounded up to 100 ms. Repeat once to confirm the cable is fully pulled and fully slack at the ends.
- [ ] 6.4 Apply with the wheels spinning above 3 km/h → refused with the speed in the status line, nothing moves; at standstill → applied.
- [ ] 6.5 Drive-away (ignition ON): brake applied, L selected, throttle 0 → 30 %. The release starts at once, the servo stays at idle for 50 % of the travel (1000 ms at the 2000 ms default), then the throttle rises with the opening slew while the actuator finishes (run end ≈ 2500 ms); `pbrake_thr_hold` goes false while the state is still "releasing" and GCS bit3 stays set until the end. Repeat in R, and with the release point at 100 % (idle until the run ends).
- [ ] 6.5a Drive-away from the MAVLink throttle (Pixhawk in MANUAL, then an autopilot mode): same behaviour as 6.5.
- [ ] 6.5b Slope: on an incline, tune the throttle release point so the buggy neither rolls back nor drags noticeably against the rear brake; record the chosen value.
- [ ] 6.5c Ignition OFF: brake applied, L, throttle 0 → 30 % → no release, throttle idle, card shows "ignition off"; Release button still works. Ignition ON with the throttle held at 30 % → no release, card shows "drop the throttle and press again"; dip to 0 and press → release.
- [ ] 6.5d During a release past the release point (throttle passing), press Apply at standstill → the run reverses and the throttle drops to idle at once; drop the throttle to 0 mid-release → the run completes, the brake stays released.
- [ ] 6.6 Throttle 8 % in L with the brake applied → idle, no release. N with the brake applied → revving works (up to the 8.5 % clip), no release.
- [ ] 6.7 Apply while holding 30 % in L → after the apply no release; release only after the throttle dips to ≤ 10 % and rises again.
- [ ] 6.8 Opposite command mid-run → short pause, then the full run the other way; same command mid-run → "already …", the remaining time is not reset.
- [ ] 6.9 Pull the 12 V mid-run → after power-up the state is Unknown and nothing moves. Reboot after a completed apply → Applied; after a completed release → Released.
- [ ] 6.10 GCS (MAVLink inspector): `EFI_STATUS.pt_compensation` bit2 set when Applied, bit3 during a run / Unknown, both 0 when Released; the lock / light bits unchanged.
- [ ] 6.11 Fail-safe (unplug the MAVLink link with MAVLink as the source): an applied brake does not move; a running release completes.
- [ ] 6.12 Service brake (X7) card and `brake_pct` behave as before while the park brake is applied and while it runs.
- [ ] 6.13 Ignition OFF: apply and release both work (the actuator is on permanent battery power).
