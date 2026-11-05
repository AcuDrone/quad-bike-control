## 1. Pure logic and tests
- [ ] 1.1 `include/TransmissionLaw.h`: `ShiftConfirm { armed, sinceMs; reset(); step(now, startMs, sensorMatches, confirmMs, timeoutMs) → WAIT|CONFIRMED|TIMEOUT }` (window from the first matching read, restarted by any non-matching read, timeout measured from `startMs`).
- [ ] 1.2 `test/test_transmission_sync`: confirmed exactly at 200 ms of continuous match; a mismatch at 150 ms restarts the window; no match → TIMEOUT at 2000 ms (dwell) / 1000 ms (pause); match arriving at 1900 ms still times out at 2000 ms; `pio test -e native` passes.

## 2. Firmware
- [ ] 2.1 `Constants.h`: `TRANS_DWELL_CONFIRM_MS` (200, comment: 300–400 if the linkage bounces); comments on `TRANS_OVERSHOOT_DWELL_MS` / `TRANS_SEQUENCE_STEP_DWELL_MS` say "timeout"; bump `FIRMWARE_VERSION` to "1.5.15".
- [ ] 2.2 `OVERSHOOT_DWELL`: reset `ShiftConfirm` on entry; each loop step it with `physicalGear == targetGear_`; CONFIRMED or TIMEOUT → RETURN as today; log "Dwell confirmed after N ms" / "Dwell timeout N ms, gear assumed". `needsThrottleBoost()` untouched.
- [ ] 2.3 Inter-step pause (phase NONE, sequence queued): `ShiftConfirm` with `physicalGear == targetGear_` and `TRANS_SEQUENCE_STEP_DWELL_MS` timeout replaces the fixed `sequenceStepDwellStart_` wait; reset whenever the pause is (re)armed; log "Step X confirmed after N ms" / "pause timeout".
- [ ] 2.4 `getPhysicalGear()`: idle-retry deferral only when phase NONE **and** `queuedGear_ == GEAR_UNKNOWN`.
- [ ] 2.5 Debug log "Shift X -> Y took N ms" when a single step or a whole sequence completes.

## 3. Docs, bench
- [ ] 3.1 `Changelog.h` 1.5.15 bullet (uk + en).
- [ ] 3.2 Build `esp32-s3-devkitc-1` clean.
- [ ] 3.3 Bench: L→N→L with the serial log: each step shows "Dwell confirmed after N ms" (N ≈ 200–300), pauses confirmed, N→L total ≈2.6 s in "Shift ... took".
- [ ] 3.4 Bench: gear sensor connector unplugged → a shift still completes, each dwell logs "Dwell timeout 2000 ms" and each pause times out after 1000 ms.
- [ ] 3.5 Bench: with the boost on, RPM blip visibly shorter and still released at the end of the dwell; with the boost off, idle hold released at the end of the shorter dwell.
