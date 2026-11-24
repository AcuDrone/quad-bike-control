## 0. Preconditions
- [x] 0.1 Proposal approved by the owner, including the "no protection" risk (proposal.md top) and
  assumption A1 (hand-over wait; alternative: release on the press).
- [x] 0.2 Work on branch `aztec` only; never cherry-pick into `master`.
- [ ] 0.3 Q2 answered: is the X7 service-brake actuator powered with the ignition OFF? Confirm with
  the owner, or measure at X7 with the ignition OFF. If it is not, add "ignition relay not OFF" to
  the trigger (design A2) and to the `vehicle-systems` delta.
- [x] 0.4 Archive `add-aztec-parking-brake` first. Then turn this change's `vehicle-systems` delta
  into ADDED + a MODIFIED "Aztec Parking Brake Actuator", in which the scenario "Service brake
  independent" reads: "a service brake command SHALL NOT change the park-brake state, except through
  the requirement Parking Brake Release by Service-Brake Press".
- [x] 0.5 `add-crank-time-limit-setting` (1.5.36) implemented and merged into `aztec` first. This
  change is 1.5.37 and adds no telemetry key.

## 1. Pure law
- [x] 1.1 `include/ParkBrakeLaw.h`:
  - `PB_LAW_SVC_TOL_PCT` 1.0f, checked against `BRAKE_TOLERANCE` by `static_assert` in
    `src/ParkBrakeController.cpp`, like the existing `PB_LAW_*` literals;
  - `enum class ParkBrakeSvcSource { NONE, MAVLINK, WEB }` (NONE = FAILSAFE);
  - `struct ParkBrakeSvcRelease { float prevTargetPct; bool pending; ParkBrakeSvcSource prevSource; }`;
  - `ParkBrakeSvcStep parkBrakeSvcStep(const ParkBrakeSvcRelease&, ParkBrakeSvcSource src, float
    targetPct, float estPct, ParkBrakeState st)` → next state + `fire`, exactly per design D1–D3:
    - source changed → re-seed `prevTargetPct`, cancel pending, no fire;
    - source NONE → update the sample, cancel pending, no fire;
    - `prevTargetPct == 0 && targetPct > 0` with state APPLIED/UNKNOWN → pending;
    - pending and `targetPct == 0`, or state not APPLIED/UNKNOWN → cancel;
    - pending and `estPct >= targetPct − PB_LAW_SVC_TOL_PCT` → fire, clear pending;
    - always store `prevTargetPct = targetPct`.

## 2. Native tests (`test/test_park_brake_law/test_main.cpp`, keep the existing ones)
- [x] 2.1 Rising edge 0 → 50 % from APPLIED: no fire until the estimate reaches 49 %, then fire once.
- [x] 2.2 The same from UNKNOWN: fires.
- [x] 2.3 Held brake at apply time: target 60 % while the state goes RELEASED → APPLYING → APPLIED,
  target unchanged → never fires; then 0 → 60 % → fires after the hand-over.
- [x] 2.4 Stale target under FAILSAFE: source NONE with a target 25 % → no fire; source back to
  MAVLINK with the same 25 % → no fire; with 40 % → no fire (non-zero → non-zero).
- [x] 2.5 Source takeover MAVLINK → WEB with an inherited 80 % → no fire; WEB 80 % unchanged → no
  fire; WEB 0 → 80 % → fires after the hand-over. A pending wait across MAVLINK → WEB is cancelled.
- [x] 2.6 Command dropped before the estimate reaches the target → no fire, pending cleared.
- [x] 2.7 Second press after a dropped one → fires after its hand-over.
- [x] 2.8 Press during APPLYING, RELEASING or RELEASED → no fire; a pending wait is cancelled when the
  state leaves APPLIED/UNKNOWN.
- [x] 2.9 The law never requests APPLY (the step has no apply output; assert across all cases).
- [x] 2.10 Copilot pattern (0 → 25 % constant, from APPLIED) → fires once the estimate reaches 24 %:
  documents the accepted behaviour.
- [x] 2.11 Pending target change 30 % → 80 % → waits for 79 %.
- [x] 2.12 `pio test -e native` passes.

## 3. Vehicle controller (`include/VehicleController.h`, `src/VehicleController.cpp`)
- [x] 3.1 Member `ParkBrakeSvcRelease pbSvc_` (initial: target 0, not pending, source NONE).
- [x] 3.2 In `update()`, after `updateBrakeControl()` and before `parkBrake_.update()`: map
  `currentInputSource_` to `ParkBrakeSvcSource` and step the law with `currentBrakeTarget_`,
  `currentBrakePosition_` and `parkBrake_.getState()`. On `fire`, call
  `parkBrake_.request(ParkBrakeRequest::RELEASE, ParkBrakeSpeedCheck::OK)` exactly as drive-away
  does; on STARTED/REVERSING clear `pbArmed_` and log under BRAKE "[PBRAKE] release by service brake
  (mavlink|web), target N %".
- [x] 3.3 Diff review: `applyBrake()`, `updateBrakeControl()`, the drive-away latch,
  `commandParkBrake()`, the throttle hold, the 3 km/h interlock and the governor are untouched.

## 4. Web help (`data/index.html`)
- [x] 4.1 Parking brake card help (ⓘ), uk + en, per the `web-control` delta:
  - uk: "Натискання робочого гальма (пульт, джойстик GCS або «Гальмо» у вебі) відпускає затягнуте
    стоянкове гальмо, щойно робоче гальмо дотисне. Це стосується й автоматичного гальма при втраті
    зв'язку з пультом: стоянкове гальмо відпуститься, а коли автоматичне гальмо закінчиться
    (типово 15 с), багі не триматиме нічого. Якщо гальмо вже натиснуте в момент затягування —
    відпустіть і натисніть знову.";
  - en: "Pressing the service brake (RC, GCS joystick or the web Brake) releases an applied parking
    brake once the service brake has reached the pressed position. This includes the automatic brake
    on RC loss: the parking brake releases, and when the automatic brake ends (15 s by default)
    nothing holds the buggy. A brake already held when the parking brake is applied does not release
    it — let go and press again."
- [x] 4.2 No new control, setting, status line or telemetry key.

## 5. Build and validate
- [x] 5.1 `pio run` for the aztec environment.
- [x] 5.2 `openspec validate add-park-brake-release-by-service-brake --strict`.

## 6. Docs
- [x] 6.1 `WEB_PORTAL_SETUP.md`: the parking-brake section — release by a service-brake press, the
  hand-over wait, the RC-loss consequence, and the trailer-ramp procedure.
- [x] 6.2 `MAVLINK_SETUP.md`: servo channel 2 below centre (any sender, including the Copilot's
  `RC_CHANNELS_OVERRIDE` brake) releases the parking brake on a new press; the RC-loss consequence
  (ADR-0004 brake, `brake_hold_s`).
- [x] 6.3 `../../API/README.md`: on the park-brake row (or a new row), "also released by a
  service-brake press over RC / joystick / web, from 1.5.37, buggy only"; no new API item. Checked
  against `MAVLINK_SETUP.md`.

## 7. Version and changelog
- [x] 7.1 `include/Changelog.h` 1.5.37, uk + en ("багі"): "Стоянкове гальмо відпускається
  натисканням робочого гальма (з пульта, джойстика або вебу), щойно робоче гальмо дотисне. Увага:
  автоматичне гальмо при втраті зв'язку з пультом теж відпускає стоянкове гальмо." / "The parking
  brake is released by pressing the service brake (RC, joystick or web) once the service brake has
  reached the pressed position. Warning: the automatic brake on RC loss also releases the parking
  brake."
- [x] 7.2 `FIRMWARE_VERSION` "1.5.37" in `include/Constants.h`; LittleFS upload noted (help text).

## 8. Bench (wheels off the ground where noted)
- [ ] 8.1 Web: parking brake APPLIED, web Brake 0 → 80 %. The parking brake releases about
  0.8 × `travel_ms` later. Let the brake go mid-run: the run completes and nothing re-applies.
- [ ] 8.2 RC: press and hold → releases after the hand-over. Hold the brake, apply the parking brake
  from the web → stays APPLIED; let go and press again → releases. A quick tap shorter than the
  hand-over → nothing. Test in N with the engine off, and in L with the engine running (the throttle
  hold up to the release point stays as today).
- [ ] 8.3 **Trailer ramp** (or a steep slope), N, engine off, parking brake APPLIED: press and hold
  the service brake; check that the buggy is held by the service brake alone when the parking brake
  lets go (lengthen `brake`/`travel_ms` if it releases before the pads bite); ease off and roll down
  under control.
- [ ] 8.4 **RC loss, wheels off the ground**, parking brake APPLIED, Copilot enabled: switch the RC
  handset off. **Expected and accepted:** the Copilot brakes (about 2.5–5 s), the parking brake
  releases, and after `brake_hold_s` (15 s) the service brake lets go, leaving nothing holding the
  wheels. Show this to the crew once.
- [ ] 8.5 Unplug the MAVLink cable with the brake held (firmware FAILSAFE, browser closed): nothing
  releases. Reconnect with the same brake value: nothing releases.
- [ ] 8.6 Confirm Q2 on the vehicle: with the ignition OFF, a web Brake 80 % actually presses the
  service brake.
