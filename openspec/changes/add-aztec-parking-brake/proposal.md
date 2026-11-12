# Change: Aztec parking brake — timed actuator on X6, operator apply/release, drive-away release

## Why
The buggy's hand-brake lever pulls a cable to the rear caliper. A DC linear actuator that pulls the
same cable is already installed and wired through a BTS7960 on board connector X6 (L_PWM = GPIO4,
R_PWM = GPIO5, L_IS / R_IS on CD4051 mux CH3 / CH0). The firmware does not drive X6 today
(`include/Constants.h` and `GPIO_PINOUT_CUSTOM_BOARD_S3.md` mark GPIO4/5 "FREE — reserved for a 4th
BTS7960"), so the buggy cannot be parked unattended: the hydraulic service brake on X7 is a driving
brake whose position is an open-loop estimate held by PWM and is not a parking brake.

The actuator has built-in limit switches (it stops itself at both ends) and is self-locking (it
holds the cable tension with no power). There is no position feedback. Its motor supply is on the
permanent battery feed (not ACC, not the ignition contactor), so it can move in any ignition state.
The owner's estimate of the full travel time is about 2 s, which is not measured yet. Which PWM pin
tightens the cable is not known yet.

Aztec only. On the quad bike (`master`) X6 carries the steering VESC, so this change is not meant to
be merged or cherry-picked into `master`.

## What Changes
- **Driver.** A second `BTS7960Controller` instance on X6: `PIN_PARK_BRAKE_LPWM` = GPIO4 (X6 pin 2),
  `PIN_PARK_BRAKE_RPWM` = GPIO5 (X6 pin 3), LEDC channels 7 / 6 (free in code and documented as
  free), on the shared motor timer `LEDC_TIMER_1` at `MOTOR_PWM_FREQ` (10 kHz, 8-bit) — the same
  timer and frequency as the X7 brake. The X6 IS lines (mux CH3 / CH0) are not sampled.
- **Timed travel, no hold PWM.** APPLY or RELEASE drives full duty (255) in that direction for the
  web-tunable travel time plus a fixed margin (`PARK_BRAKE_TRAVEL_MARGIN_MS` = 500 ms), then stops
  the bridge (both PWM 0). There is no hold drive: the actuator is self-locking and its limit
  switches cut the motor at the ends.
- **State.** `UNKNOWN`, `RELEASED`, `APPLIED`, `APPLYING`, `RELEASING`. The state is persisted in a new
  NVS namespace `"pbrake"` (key `state`) when a run starts (as `APPLYING` / `RELEASING`) and when it
  ends (as `APPLIED` / `RELEASED`). At boot a stored `APPLIED` / `RELEASED` is restored; a stored
  moving state, a missing key or an invalid value is `UNKNOWN`. The actuator never moves at boot on
  its own.
- **Commands (source-agnostic entry point).** `VehicleController::commandParkBrake(request, source)`
  returns a result code; the web is the only caller in this change. The GCS path (MAV_CMD_USER_2)
  can call it later without changes to the park-brake logic.
  - APPLY only by explicit operator command. There is no automatic apply: not on RC / link loss,
    not on fail-safe, not on ignition off, not at boot.
  - APPLY is refused while the vehicle moves faster than `PARK_BRAKE_APPLY_MAX_SPEED_MS`
    (0.833 m/s = 3 km/h) by the hall speed sensor. The web reply is "Park brake apply refused:
    moving at N.N km/h (max 3.0)". If the sensor has not pulsed since boot, APPLY is allowed and the
    reply says "(speed unknown)" (owner-confirmed).
  - APPLY and RELEASE are accepted in any ignition state, because the actuator is on permanent
    battery power.
  - RELEASE by explicit command is always allowed (any speed, any input source, any ignition state).
  - A repeated command in the direction already running is ignored (the run is not restarted). An
    opposite command during a run stops the bridge, waits `PARK_BRAKE_REVERSE_DWELL_MS` (200 ms) at
    zero duty, then runs the full travel plus margin in the new direction. A command at the end
    where the brake already is drives the full run again, so the operator can re-assert the state.
- **Drive-away release.** A RELEASE starts when all of these hold:
  - the park brake is `APPLIED` or `UNKNOWN`;
  - the ignition is on: the firmware's commanded ignition-relay state is not `OFF`, so `ACC`,
    `IGNITION` and `CRANKING` all count (`RelayController::getIgnitionState()`, owner decision
    2026-11-12, design D12);
  - the target gear is L, H or R (not N);
  - the operator's throttle demand rises above `PARK_BRAKE_DRIVE_AWAY_THROTTLE_PCT` (10 %), with
    no dwell or debounce.

  Details:
  - Any throttle source can trigger it: the web, and everything arriving from the Pixhawk over
    MAVLink, manual or autopilot.
  - It needs a rising edge: the demand must have been at or below 10 % while the brake was
    `APPLIED` / `UNKNOWN` **and the ignition was on**. Applying with the throttle already above
    10 %, or switching the ignition on with the throttle already above 10 %, keeps the brake
    applied and the throttle at idle. The operator must drop to 10 % or below and press again.
  - With the ignition OFF, throttle never releases the park brake, and the brake stays applied with
    the throttle at idle. The operator's explicit Release command still works in any ignition
    state.
  - The card and telemetry (`pbrake_da_wait`) say why a press did nothing: "ignition off" or "drop
    the throttle and press again".
  - The release is not tied to engine start or RPM.
  - In N, throttle never releases the park brake, and revving in place stays as today, capped by
    the existing neutral clip (`TRANS_UNKNOWN_GEAR_THROTTLE_MAX` 8.5 %).
  - If the throttle drops during a release run, the run completes and the brake stays released.
    There is no automatic re-apply.
- **Throttle hold, released part-way (anti roll-back).** In gear (target L, H or R), the operator's
  throttle (web and MAVLink) is replaced by idle while the park brake is:
  - `APPLIED`, `APPLYING` or `UNKNOWN` (not releasing); or
  - `RELEASING` and less than the **throttle release point** has elapsed: `thr_pass_pct` % of
    `travel_ms`, counted from the start of the release drive.

  From that point on, the operator's throttle passes through the existing opening slew, from idle,
  while the actuator finishes its run. The rule applies to every RELEASING run, whether it was
  started by drive-away or by the operator's Release, and whether the brake started `APPLIED` or
  `UNKNOWN`.
  - Default 50 %, range 20–100 %. With 100 % the throttle is held until the run ends, margin
    included. Below 20 % is not offered: the owner rejected "no hold at all".
  - Default numbers: 2000 ms travel → the throttle starts ramping 1000 ms into the release drive,
    and the run ends at 2500 ms.
  - An Apply arriving after the throttle was passed reverses the run (APPLYING) and puts the
    throttle back to idle at once.
  - `UNKNOWN` is treated as possibly applied, and a fail-safe during a run lets the run complete.
  - All of the drive-away and hold rules are owner decisions (D11, D12).
- **Tuning (NVS `"pbrake"`).** `travel_ms` (uint16, 1000–5000 ms, default 2000 — the owner's
  estimate, to be replaced by the bench measurement), `thr_pass` (uint8, throttle release point
  20–100 % of `travel_ms`, default 50) and `invert` (uint8 0/1, default 0). Changing the direction sets the
  state to `UNKNOWN` and is refused while a run is in progress. Web commands `park_brake`
  (`"apply"` / `"release"`), `set_park_brake_travel_ms`, `set_park_brake_thr_pass_pct` and `set_park_brake_invert` are accepted
  from any input source.
- **Telemetry (web).** `pbrake_state` (`"unknown"|"released"|"applied"|"applying"|"releasing"`),
  `pbrake_remaining_ms`, `pbrake_thr_hold`, `pbrake_da_wait` (`"none"|"ign_off"|"rearm"`),
  `pbrake_travel_ms`, `pbrake_thr_pass_pct`, `pbrake_invert`.
- **Telemetry (GCS).** `EFI_STATUS.pt_compensation` digital-output bitmask: new bit2
  `EFI_DIGITAL_FLAG_PARK_BRAKE` (0x04) = `APPLIED`, and new bit3 `EFI_DIGITAL_FLAG_PARK_BRAKE_UNSETTLED`
  (0x08) = `APPLYING`, `RELEASING` or `UNKNOWN`. These sit next to bit0 `EFI_DIGITAL_FLAG_WHEEL_LOCK`,
  in the same mechanism, assembled in `src/main.cpp`.
- **Web UI.** New card "🅿️ Стоянкове гальмо" / "🅿️ Parking brake" in the Tests section after
  "Vehicle Ignition & Lighting". It shows the state, the remaining run time, a throttle-hold
  note, and why a throttle press did not release the brake (ignition off / drop the throttle and
  press again). "Затягнути" / "Apply" and "Відпустити" / "Release" both use the existing two-press
  confirmation. The card also has the travel time slider (same pattern as the "🛑 Гальмо" card), a
  throttle-release-point slider (20–100 %, step 5, default 50) and
  a two-press direction toggle, a status line and help (ⓘ) with the bench procedure, in uk + en. The
  card is hidden when telemetry has no `pbrake_state`.
- **Pure law** `include/ParkBrakeLaw.h` (header-only) with native tests `test/test_park_brake_law`.
- **Docs.** `GPIO_PINOUT_CUSTOM_BOARD_S3.md` (§1/§2 GPIO4/5 rows, §3 X6 row, §4 UART2 note and LEDC
  table, §9 new constants, the IS-mux note), `MAVLINK_SETUP.md` (bitmask table, value range 0–15),
  the `Constants.h` header comment.
- **Version.** Firmware **1.5.29**, with a Changelog entry (uk + en). The LittleFS image must be
  uploaded because of the new card and i18n keys.

Out of scope (see design.md "Follow-ups"):
- Current-based end-of-travel through the BTS7960 IS lines (CD4051 CH3/CH0, mux control U2 @ 0x1C).
  That is a later, separate change.
- The GCS → board command path (MAV_CMD_USER_2) and a GCS button. The GCS plugin / QGC decode of
  bits 2–3 lives in the `gcs` repository.
- A new RC channel (none is added).
- Any automatic apply.
- Journal (`EventJournal`) entries for park-brake events. Debug log only, `DebugFeature::BRAKE`.

## Impact
- Affected specs:
  - `vehicle-systems`: ADDED "Aztec Parking Brake Actuator", "Parking Brake Apply Speed Interlock",
    "Parking Brake Drive-Away Release and Throttle Hold", "Parking Brake State Persistence",
    "Configurable Parking Brake Travel Time and Direction".
  - `vehicle-actuators`: MODIFIED "BTS7960 Motor Driver Control", "Hardware Configuration".
  - `web-control`: ADDED "Parking Brake Card".
  - `web-telemetry`: ADDED "Parking Brake Telemetry".
  - `mavlink-interface`: ADDED "Parking Brake State in the EFI_STATUS Digital-Output Bitmask".
- Pending-change overlap. The `vehicle-actuators` "Hardware Configuration" delta here is written on
  top of `repurpose-x10-battery-sense`'s text (implemented, not yet archived), so that change must
  be archived first. `move-aztec-vesc-to-can` (not started) also modifies "Hardware Configuration"
  and must be rebased onto this text before it is archived. Neither change touches "BTS7960 Motor
  Driver Control".
- Affected code: `include/Constants.h`, `include/ParkBrakeLaw.h` (new), `include/ParkBrakeController.h`
  / `src/ParkBrakeController.cpp` (new), `include/VehicleController.h`, `src/VehicleController.cpp`,
  `src/main.cpp`, `include/WebPortal.h`, `src/WebPortal.cpp`, `src/TelemetryManager.cpp`,
  `data/index.html`, `include/Changelog.h`, `platformio.ini`, `test/test_park_brake_law/test_main.cpp`
  (new).
- Behaviour on the vehicle after flashing:
  - On first boot the state is `UNKNOWN`. In gear, throttle is held at idle until a release has
    run: either the operator presses Release, or a throttle press above 10 % in L/H/R starts a
    drive-away release (ignition on). With the defaults the throttle is held for 1000 ms of
    that release (50 % of 2000 ms), then ramps up while the actuator finishes (run end 2500 ms).
  - Revving in N is unchanged.
  - Nothing moves at boot.
  - The X7 service brake is unchanged and independent.
- Hardware: none. The actuator and the BTS7960 on X6 are already installed.
- Branches: aztec only. Do not cherry-pick into `master`, where X6 is the quad's steering VESC.
