## Context
The MT05 ECU on the aztec buggy answers OBD PID 0x0C with 0 and broadcasts no RPM frame (probe
fw 1.5.6, sniffer fw 1.5.7). RPM gates gear changes (`isEngineRunning()`), the starter cut-off and
crank interlock (`RelayController::update(engineRpm)`), the web START interlock, transmission init,
the gear-change boost PID, the hour meter, telemetry and `EFI_STATUS.rpm`. The ECU tachometer output
(grey J1 pin 6) is a ≈12 V, ≈50 % duty square wave driven by the ECU with or without the dash; the
dash shows ≈1500 RPM at idle. A crank-sensor reference measured ≈600 Hz at idle on the 24-1 wheel
(23 teeth/rev) → ≈1565 RPM, so the expected tacho rate is ≈25–26 Hz at ppr 1.

## Goals / Non-Goals
- Goals: a reliable engine RPM on aztec from the tacho wire; CAN RPM stays the fallback; runtime
  ppr calibration without the dash; every RPM consumer moves to one effective value.
- Non-Goals: tapping the crank sensor; a new RPM tile; changing the master (quad) branch;
  MAP-based RPM estimation.

## Hardware
```
TACHO (J1-6) ──1.5k 0.25W──► PC817 pin1 (A)      PC817 pin4 (C) ──► X3.1 (5 V)
vehicle GND  ◄───────────── PC817 pin2 (K)      PC817 pin3 (E) ──► X3.2 (GPIO14 via 1.2K/2.2K + TVS)
                                                 X3.4 (GND) unused, no pull-up resistor
```
- Emitter follower (decided 2026-11-04, replacing the first common-emitter + 4.7 kΩ pull-up draft):
  the on-board 1.2K + 2.2K ≈ 3.4 kΩ divider to GND is the emitter load. The pull-up version would have
  given only 5 × 3.4 / 8.1 ≈ 2.1 V at X3.2 → ≈1.36 V at GPIO14, below the ESP32-S3 V_IH (≈2.48 V).
- LED current with 1.5 kΩ: ≈7.5 mA at 12.5 V, ≈9 mA at 14.5 V; resistor dissipation ≈0.12 W (0.25 W
  part). PC817 CTR ≥ 50 % gives ≥ 3.7 mA collector current available; the 3.4 kΩ load needs ≈1.5 mA
  → ≥ 2.5× margin. Isolated; the ECU sees a ≈7.5–9 mA load.
- Signal at GPIO14 is NOT inverted: LED on (ECU output high) → ≈4.8 V at X3.2 → ≈3.1 V at GPIO14
  (high); LED off → the 2.2K pulls GPIO14 to 0 V. The firmware times one edge (RISING), so duty does
  not matter.
- The crank sensor is not tapped: loading a VR/Hall crank input risks misfire / timing errors.
- Bench task 7.1 measures GPIO14 (expected ≈3.1 V high, must be ≥ 2.5 V; ≈0 V low) before the counts
  are trusted.

## Decisions
- **ISR edge timestamps, not PCNT.** RPM comes from the period: at 25 Hz a 1 s count window gives
  ±1 count = ±4 % and 1 s latency, while the period with `micros()` resolution and ≈2–5 µs ISR
  latency gives ≈0.01 % per period. The ESP32-S3 PCNT unit only counts; it does not timestamp edges,
  so a PCNT design would still need an interrupt per edge (or a watch point every pulse), which brings
  back the ISR without the benefit. The MCPWM capture unit does timestamp in hardware, but its API is
  heavier and the extra precision is not needed. Decision: `attachInterrupt(PIN_TACHO, isr, RISING)`
  on an IRAM handler that stores `micros()` into a 5-entry ring buffer (4 periods) and an edge
  counter. Software glitch filter: an edge less than `TACHO_MIN_PERIOD_US` (1000 µs, i.e. 1 kHz) after
  the last accepted edge is dropped — at ppr 4 and 10 000 RPM the signal is 667 Hz. Load: ≤ 700
  interrupts/s of ≈1 µs each. The ISR shares the GPIO ISR service with the planned MCP2515 INT ISR
  (`move-can-rx-to-interrupt`); `attachInterrupt` handles that.
- **Averaging**: mean of the last up to 4 periods (≈160 ms at idle), valid from the second edge.
  RPM above `TACHO_MAX_RPM` (10 000) is discarded (keeps the previous reading). Timeout
  `TACHO_TIMEOUT_MS` 500 → 0 Hz / 0 RPM and the buffer is cleared.
- **ppr**: float in NVS `"tacho"`/`"ppr"`, allowed {0.5, 1, 2, 3, 4}, default 1.0. Calibration
  `raw = Hz × 60 / ref`, snapped to the nearest allowed value; refused if no pulses, ref outside
  300–8000, or `|raw − snapped| > 0.25 × snapped`. `reset_tacho_ppr` REMOVES the key (back to the
  default, and no longer "calibrated").
- **"Fitted"** = an accepted edge since boot (latched) OR a stored `ppr` key. Pulses alone fit it on
  the first run; the NVS key makes a stopped engine read "0 from TACHO" right after boot instead of
  falling back to CAN. A pure NVS flag alone was rejected: it would need an extra command and a
  forgotten flag would hide CAN. Un-fitting: `reset_tacho_ppr` + reboot with no pulses.
- **Injection point: `VehicleController`**, not `CANController`. `VehicleData.engineRPM` keeps the raw
  CAN value (useful for diagnosis, and the CAN spec stays true). VehicleController gets
  `EffectiveRpm { uint16_t rpm; bool valid; RpmSource source; uint32_t sampleMs; }` computed once per
  `update()` via `tachoSelectRpm()` in `TachoLaw.h`, with accessors `getEngineRpm()`,
  `isEngineRpmValid()`, `getRpmSource()`. Selection:
  1. tacho fitted and live → TACHO;
  2. else CAN valid and CAN RPM > 0 → CAN (a broken tacho wire on an ECU that does report RPM must not
     make a running engine look stopped — the dangerous direction for the crank interlock);
  3. else tacho fitted → 0, TACHO, valid;
  4. else CAN valid → CAN; else NONE (invalid, 0).
  Consumers switched: `isEngineRunning()` (VehicleController.h:389), `relayController_.update()`
  (VehicleController.cpp:324), web START interlock (:1421), boost PID activation / error / new-sample
  test (:1725, :1750, :1756, :1767 — `sampleMs` is the CAN `lastUpdateTime` or the last tacho edge in ms),
  hour meter (:321), TelemetryManager (:150, moved out of the CAN block), main.cpp MAVLink report
  (:314, plus `rpmValid`). `CAN_POLL_INTERVAL_RPM_BOOST` switching is kept so the fallback stays
  fresh.
- **MAVLink**: `EFI_STATUS.rpm` = effective RPM, NaN only for source NONE (it is no longer gated on
  `canValid`). No `TACHO_HZ` `NAMED_VALUE_FLOAT`: all NVFs share one message id and collide in a
  name-agnostic store (the reason gear, odometer and hours live in `EFI_STATUS`); Hz is a calibration
  value, available in web telemetry.
- **Web**: new calibration card after "Speed limit and steering" in the Calibration section; the
  existing RPM tile shows `engine_rpm` outside the CAN gate, with a "TACHO"/"CAN" badge, "—" for NONE.
  Replies starting "Tacho" are routed to the card like "Battery" replies.

## Threshold safety analysis (1500 → 800)
| Use | Site | Effect of 800 |
|---|---|---|
| Starter cut-off while CRANKING | RelayController.cpp:342 | Starter released once the engine fires and passes 800, well above cranking speed (≈250–400 RPM, bench-checked in 7.5). At 1500 (= idle) a started engine idling at 1450 kept the starter engaged until `CRANKING_TIMEOUT` — 800 is safer. |
| Armed-crank suppression | RelayController.cpp:319 | An idling engine is now recognised with a 700 RPM margin; at 1500 a slightly low idle could let the starter engage on a running engine. Safer. |
| Web START interlock | VehicleController.cpp:1421 | Same as above. Safer. |
| MAVLink gear gate | VehicleController.cpp:734 | Gears accepted from 800; idle is ≈1500, so normal idle no longer sits on the edge. A stopped engine (0 within 0.5 s) still blocks. |
| Transmission init on first engine-running | :294 | Fires at 800 instead of 1500 — harmless. |
| Boost PID activation | :1725 | Activates from 800; PID still targets `boostTargetRpm_`. |
| Throttle-cal / gear-code refusals | :1217, :1283 | Stricter (refuse at 800) — safer. |
| Hour meter | :321 | Uses `ENGINE_HOURS_MIN_RPM` (300), unchanged. |
The only new failure mode is a spurious ≥800 reading while cranking (cuts the starter early → the
engine does not start; the operator retries). Mitigated by the 4-period mean, the 1 ms glitch filter
and the plausibility cap. The `Constants.h` comment that calls 1500 "above idle" and the
`isEngineRunning()` doc comment are updated. Quad (master) keeps 1500.

## Risks / Trade-offs
- GPIO14 high level marginal (worn opto, low battery) → bench task 7.1 first; 2.5× CTR margin.
- Ignition noise on a long unshielded TACHO wire → opto isolation, 1 ms glitch filter, averaging;
  twisted pair with GND recommended in the pinout doc.
- ppr ambiguity (e.g. 1 vs 2 if the reference is far off) → ±25 % acceptance window; the reply shows
  the raw ratio.
- The ECU tacho may be silent while cranking → the starter then ends on RPM ≥ 800 once it runs, or on
  timeout, as today.

## Overlaps with active changes
- `repurpose-x10-battery-sense` (implemented, not archived) MODIFIES mavlink-interface "Vehicle State
  Reporting via Standard MAVLink Messages"; this change does not touch that requirement and ADDS
  "Effective Engine RPM in MAVLink Reports" instead (its "rpm … CAN" wording is superseded at archive).
  Same pattern for the `cal_bat_v` command plumbing and the "Battery" reply routing that this change
  copies.
- `move-aztec-vesc-to-can` and `repurpose-x10-battery-sense` both MODIFY vehicle-actuators "Hardware
  Configuration"; to avoid a third concurrent edit, `PIN_TACHO` = GPIO14 is specified in the new
  tacho-sensor capability. Archive order: fold GPIO14 into the pin list when the later of these is
  archived.
- `move-can-rx-to-interrupt` adds a GPIO ISR (MCP2515 INT) and modifies the OBD-II reading
  requirement; no requirement overlap, but both share the GPIO ISR service.
- `add-can-bus-sniffer`, `fix-steering-continuous-tracking`: no overlap.

## Migration Plan
Flash fw 1.5.9 + LittleFS. Without the tacho wired nothing changes except the 800 threshold
(source CAN). Wire the tacho → it is fitted on the first pulse; calibrate once. Rollback: reflash
1.5.8; the `"tacho"` NVS namespace is ignored by older firmware.

## Resolved Questions
1. Wiring: emitter follower with 1.5 kΩ LED resistor, no pull-up (see Hardware).
2. ppr changes are NOT refused while the engine runs: `cal_tacho_ppr` needs a running engine by
   definition, and `set_tacho_ppr` / `reset_tacho_ppr` are harmless at runtime — the effective RPM
   simply changes on the next sample.

## Field revision 1.5.10 — noise-robust period
**Finding.** On the buggy the dash shows a ≈1400 RPM idle but fw 1.5.9 read 1100–2800 RPM, jumping.
1.5.9 averaged the last 4 periods between RISING edges behind a fixed 1 ms glitch filter. A 2× swing
is the signature of EXTRA edges (one spurious edge mid-period turns one 43 ms period into two ≈21 ms
ones; candidates: ignition noise coupled into the TACHO wire, or ringing on the slow PC817 edges
crossing the GPIO threshold twice), and the low readings of occasional missed / odd edges. A mean
absorbs every outlier; the signal cannot be inspected remotely.

**Decisions.**
- **Median instead of mean.** Period = median of the last `TACHO_MEDIAN_PERIODS` (8) periods (ring of
  9 edges), mean of the two middle values for an even count. With 8 periods, up to 3 outliers on one
  side leave it unchanged: one spurious edge (2 half periods) or one missed edge (1 double period) is
  invisible. Below 3 periods the mean is used (a median of 1–2 is no better). Lag: ≈4 periods
  (≈170 ms at idle), acceptable for the gear gate / starter cut-off / PID.
- **Lock window in the ISR.** Besides the absolute floor (raised 1000 → 1500 µs = 667 Hz, still above
  ppr 4 at 10 000 RPM), once the median rests on ≥ 3 periods an edge closer than
  `TACHO_LOCK_FRACTION` (0.5) × the median to the last accepted edge is rejected and counted; the
  reference does not move, so the next real pulse is measured from the last real one. Self-correcting:
  if the engine speeds up past 2× between updates, every second pulse is accepted, the median halves
  at most and the window re-admits all pulses on the next pass; after `TACHO_TIMEOUT_MS` of silence
  the lock is cleared (by `update()` and by the ISR's first edge). `update()` writes the
  pre-multiplied threshold (µs) under the spinlock so the ISR does integer compares only (no FPU use
  in an Xtensa ISR); the pure filter is `tachoEdgeAccepted(sinceUs, lockPeriodUs, fraction, floorUs)`
  with the integer core `tachoEdgeAcceptedMin` that the ISR calls.
- **Diagnostics instead of a scope.** `update()` keeps the last 8 raw periods and, per 1 s window,
  min / max period and accepted / rejected edges per second; telemetry `tacho_periods`,
  `tacho_min_ms`, `tacho_max_ms`, `tacho_acc`, `tacho_rej`, `tacho_rej_total`; a monospace block in the
  tacho card; one `[TACHO]` VEHICLE log line per second while live. Reading it: alike periods and
  `rej/s` ≈ 0 = clean; halves among full periods / high `rej/s` = noise → hardware fix (100 nF across
  the PC817 LED, TACHO wire away from the coil leads); doubles = missed edges (weak drive).
- Telemetry JSON capacity 8192 → 9216 (+≈200 B of slots and copied strings on a document whose
  sniff + probe worst case was already near 8 KB).
- Hz / RPM / ppr / calibration snapping / source selection unchanged. `TACHO_AVG_PERIODS` is replaced
  by `TACHO_MEDIAN_PERIODS`; new `TACHO_LOCK_FRACTION`, `TACHO_DIAG_WINDOW_MS`.

## Field revision 1.5.11 — pulse-width edge validation
**Finding.** fw 1.5.10 with ppr 1 at a ≈1400 RPM dash idle: periods "42.9 21.6 64.2 44.5 46.2 23.2
23.0 46.1", min / max 21.3 / 46.2 ms, 26–28 accepted and 0–3 rejected edges/s. The ECU TACHO is a
≈50 % duty square wave (≈6.35 V AC RMS at every RPM), so the periods are exact halves / doubles of
each other. Two hypotheses: **(A)** the true signal is ≈46 Hz (2 ppr) and the GPIO misses pulses
because the high level is marginal (≈3.0 V vs V_IH 2.475 V); **(B)** the true signal is ≈23 Hz
(1 ppr) and the slow FALLING edge (emitter follower turning off through the 2.2K, then the board
100 Ω + 100 nF, τ ≈ 0.25 ms) produces a spurious RISING interrupt at exactly 50 % of the period. The
1.5.10 lock window (reject an edge < 0.5 × median) is harmful in both: once the median lands on the
double period it rejects true edges.

**Decisions.**
- **Both edges, pulse-width validated.** The ISR is attached on CHANGE, reads the pin level directly
  from `GPIO_IN_REG` (IRAM-safe) and timestamps every edge with `esp_timer_get_time()`. The pure,
  integer-only state machine `tachoEdgeStep(TachoEdgeState&, level, nowUs, minLevelUs)` →
  `TachoEdgeEvent {NONE, ACCEPTED_RISE, GLITCH}` lives in `TachoLaw.h`:
  - a RISING edge after a LOW ≥ `TACHO_MIN_LEVEL_US` (2000 µs) starts a high pulse whose rise is a
    *candidate*; a RISING edge after a shorter LOW is a GLITCH (bounce on the falling edge, or a
    glitch low inside the high pulse) and the current high pulse CONTINUES (rise time unchanged);
  - at the FALLING edge a candidate whose HIGH lasted ≥ `TACHO_MIN_LEVEL_US` is ACCEPTED_RISE with
    the original rise time (once per pulse; latency ≤ one half period); a shorter high stays pending
    until the next rise decides: a short low → glitch inside the pulse, a long low → the pulse was
    narrow and is rejected (GLITCH);
  - a repeated level (both edges of a glitch elapsed before the ISR read the pin) is ignored;
  - init from the pin level at `begin()` (a high found at boot is never accepted, a low counts as
    long); `tachoEdgeSilence()` on the timeout makes a low in progress long and drops a candidate
    stuck high (a stale rise time never enters the ring).
- **Lock window removed** (`TACHO_LOCK_FRACTION`, `s_lockMinUs`, `tachoLockMinUs`,
  `tachoEdgeAccepted*`). `TACHO_MIN_PERIOD_US` (1500 µs) stays as a sanity floor between accepted
  rises (`tachoPeriodAboveFloor`). Median of 8 unchanged.
- **2 ms threshold.** At 50 % duty high and low each last ≥ 5 ms below 3000 RPM at 2 ppr (≈10.7 ms at
  idle); 2 ms rejects bounce / ringing (RC ≈ 0.25 ms) and ignition spikes. Ceiling 250 Hz (7500 RPM
  at 2 ppr, 3750 RPM at 4 ppr) — above anything this engine reaches at ppr ≤ 2.
- **Diagnostics distinguish A from B.** Last high span and last validated low, plus the 1 s
  min / max of the high span (kept by the ISR, read-and-reset atomically at the window close):
  telemetry `tacho_hi_ms`, `tacho_lo_ms`, `tacho_hi_min_ms`, `tacho_hi_max_ms`; the card line
  "high / low, ms: 10.7 / 10.9 (high min/max 10.2 / 11.1)"; the `[TACHO]` log gains hi / lo. High ≈ low
  ≈ 10.7 ms at ≈46 Hz → 2 ppr (A); ≈ 21.5 ms at ≈23 Hz → 1 ppr (B). JSON capacity stays 9216 (+≈90 B).
- **Hardware margin.** The field tech changes the LED series resistor 1.5 kΩ → 1 kΩ (≈11 mA at
  12.5 V, ≈0.18 W at 14.5 V in a 0.25 W part) for more drive on the marginal high level.
