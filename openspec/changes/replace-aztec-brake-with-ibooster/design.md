# Design: replace-aztec-brake-with-ibooster

## Context

### The brake today (aztec 1.5.37, snapshot 6b21b69)
- **Command.** RC / GCS joystick: servo channel 2 below centre → `MavlinkInterface::getBrake()`
  (`src/MavlinkInterface.cpp:1260-1268`, 1500 → 1100 µs = 0 → 100 %), applied every loop in
  `processMavlinkCommands()` (`src/VehicleController.cpp:1155-1157`). Web: `set_brake` →
  `processBrakeCommand()` (`:964-965`, `:2558-2568`). Both go through `applyBrake()`
  (`:2947-2955`): below 5 % → 0, above 100 → 100, stored in `currentBrakeTarget_`.
- **Actuator.** `updateBrakeControl()` (`:2957-3087`, called at `:692`) drives the X7
  `BTS7960Controller` (GPIO7 / GPIO6, LEDC ch 4 / 5, `include/Constants.h:71-76`) at ±255, advances
  the open-loop estimate `currentBrakePosition_` by `brakeRatePctPerMs(travel_ms)`
  (`include/BrakeLaw.h`), retracts past the In5 endstop (`BOARD_IN_BIT_BRAKE_LIMIT`,
  `include/Constants.h:227`; `src/BoardInputs.cpp:187`) for `BRAKE_SENSOR_OVERRUN_TIME` and pushes
  with `hold_pwm` when holding. The estimate starts at 100 % so a boot fully retracts (`:66`).
- **Settings.** NVS `brake` / `travel_ms`, `hold_pwm` (`:430-451`, `:2243-2305`), web card
  "🛑 Гальмо" in Calibration (`data/index.html:1149-1168`), telemetry `brake_pct`,
  `brake_travel_ms`, `brake_hold_pwm`, `brake_released` (`src/WebPortal.cpp:721-724`,
  `src/TelemetryManager.cpp:129-134`). All from the unarchived `add-brake-tuning-card`.
- **Readers of brake state.**
  - Throttle: `applyOperatorThrottle(..., currentBrakeTarget_ > 0.0f)` (`:641`, `:1133`, `:1396`) →
    governor stop / limiter (`commandThrottlePct()`, `:1788`, `:1879`). Uses the **command**.
  - Park brake: `updateParkBrakeServiceRelease()` (`:702-727`) → `parkBrakeSvcStep(...,
    currentBrakeTarget_, currentBrakePosition_, ...)` (`include/ParkBrakeLaw.h:262-279`), hand-over
    when the estimate is within `PB_LAW_SVC_TOL_PCT` (1 %) of the target. Uses the **estimate**.
  - Web tile `telemetry-brake` (`data/index.html:729-730`, `:3344-3345`), the Tests brake slider
    (`:1528-1531`), the boot endstop log line (`src/main.cpp:234-236`).
  - Not in MAVLink telemetry. Gear change and ignition / crank logic do not read the brake.
- **Fail-safe.** `applyFailsafe()` (`:1017-1046`): steering centre, throttle idle,
  `brake_.stop()` once (`:1027`; `updateBrakeControl()` overrides it on the next loop, so the target
  is in effect frozen), transmission stop, `relayController_.allOff()`, odometer and hours persisted.
  On exit `brake_.stop()` again (`:1043`).
- **Ignition.** `IgnitionState` OFF / ACC / IGNITION / CRANKING (`include/RelayController.h:97-102`);
  the Relay_1 ignition mask is HIGH in every state except OFF, including CRANKING. A faulted relay
  expander reports OFF (`:256-258`) although Relay_1 may physically keep its last state.
- **CAN today** (`src/CANController.cpp:617-643`, `include/Constants.h:1155-1172`): RXB0 mask
  `0x7F8`, RXF0/RXF1 `0x7E8`; RXB1 mask `0x7FF`, RXF2–RXF5 all `0x305`. `PIN_CAN_INT` GPIO38 is
  defined only (`include/Constants.h:91`).

### What the prerequisite gives (`move-can-rx-to-interrupt`, approved)
- One FreeRTOS task `can_io` (priority 12, core 1) owns every SPI access, woken by INT on GPIO38 or a
  2 ms poll (its D1, D2). RX ring 64 → loop; TX ring 8 → TXB0 only, FIFO, per-buffer 20 ms abort,
  no ABAT (D3, D4). TXB1 / TXB2 left free; its D5 sketches the follow-up: a slot clock, TXB1 / TXB2
  at a higher TXP, "TX-complete interrupts or a short re-check".
- Init / re-init run in the task, bounded, with a CANINTE readback that must equal `0x03` (D6,
  spec "Attempt executed by the CAN task"). The loop decides when; while an attempt is in flight no
  other command is posted and the watchdogs do not re-trigger.
- **Final text after implementation on `master`** (committed 1.5.38; its D4 / D6 / D7 / D9
  implementation notes, spec "Queued CAN Transmit", "MCP2515 Initialization Retry", "CAN Driver
  Diagnostics"): a frame queued while the chip is **down** (failed init / re-init) is failed and
  dropped from the ring, not kept; frames queued **during** a command wait and go out after it; a
  frame still pending in **TXB0** when any task command starts (re-init, filter switch) is aborted
  alone, because the MCP2515 changes mode only after pending transmissions and an unacknowledged
  frame would hold every library mode switch to its 200 ms timeout. An aborted TXB0 frame is
  classified only once TXREQ reads clear: ABTF set = failed, clear = sent (the attempt already on the
  wire completed); still pending 20 ms after the abort = failed. A result requester that gives up
  withdraws its frame (never sent late); the LWS zero-set flags `[05 00]` as possibly sent as soon as
  it is queued. Command results travel in a single-entry slot, not the 8-entry event ring. While the
  chip is down — after a failed boot init **or a failed re-init sequence** — the task retries the
  init every 5 s from the last attempt of any kind, gate first; a loop command goes first. The
  journal `bmax` / telemetry `can_busy_max_us` time only the RX / TX / EFLG service pass; command time
  is `cmd_max_us` on the console. A sniff now sets `mode_ = SNIFFING` at once in an "opening" phase,
  and `startSniff()` refuses while another command is in the task; a failed restore at the end
  requests a `sniff_restore` re-init.
- Its D11: flash erases stall both cores 30–50 ms and the task with them; the follow-up must keep
  flash writes away from an active request and must not let a watchdog re-init interrupt the
  periodic frames during one.

### iBooster facts used (bench, `Arduino/iBooster-TEST/README.md` «Що виміряно на стенді»)
| Fact | Value | Used in |
|---|---|---|
| Our frames | `0x38B` DLC 4 `[CRC][Q|cnt] spd 05`, `0x38C` DLC 4 `[CRC][Q|cnt] flow_lo flow_hi`, `0x38D` DLC 7 `[CRC][cnt] 00 00 00 50 47`, every 10 ms; CRC-8 J1850 over bytes 1..end; one shared 4-bit counter; matches 48 trace frames byte for byte (`src/ibooster_proto.h`, `test/test_proto`) | D3 |
| Request bits | `0x40` in byte 1 of `0x38B` and `0x38C` while requesting, `0x00` at idle (bench `ibBuild()`); `0x80` in `0x38B` drops the request | D3, D5 |
| Flow | LE, `0x7E00` = 0, 0.0078125 ml/s/bit (128 / ml/s), usable `0x6A00…0x9200` (±40 ml/s); rod ≈ 1.9–2.0 mm/s per ml/s | D5 |
| Pressure word | `50 47` = 117 (5.1 bar) qualifier 1; qualifier 0 or 2 → instant drop; ≳ 130 bar stops the apply | D3 |
| Unit frames | `0x38E` DLC 6 / 10 ms (byte 1 high nibble 2 idle, 4→5 request; fields A, B = raw/64 − 5 mm), `0x38F` DLC 6 / 20 ms (byte 3 pRunout bar, 0 at the stroke stop; byte 4 `0x40` sees us / `0x80` not), `0x39D` DLC 5 / 40 ms (status, internal state, driver_brake_apply, rod = raw/64 − 5 mm; byte 0 = (0x03 + 0x9D + b1..b4) & 0xFF) | D3, D4 |
| Timing | request accepted 260–290 ms; holds to −0.05 mm with P, kp 250 flow units / mm, deadband 3 raw; stroke stop 16.62–16.64 mm | D5 |
| Gaps | idle: any gap harmless, unit notices after 100–160 ms and recovers; request: 50 / 100 ms harmless, ≥ 200 ms → release 130–180 ms after the last frame (`0x38B` alone 240–280 ms), re-accepts 170–300 ms after frames resume, **does not re-apply** | D3, D7, D9 |
| Return | clearing the request from 2 mm: 14 mm/s, ~0.4 s; from 1 mm 4.7 mm/s | D5 |
| Self cycle | every ~21.6 s `st=6 is=5` for ~2 s, rod +0.03 mm, independent of us | D4 |
| Failure | 22.11: `st=2 drv=3` at 3.8 mm on thin supply wires, latched until a pin-1 power cycle. ECU reboot on regeneration: ~200 ms silence, counter from 0, `st=2`, then ~12 s to `ready` **by itself** | D4 |

## Goals / Non-Goals
- Goals: a working, observable brake on the iBooster with the smallest firmware that is safe to run
  on the vehicle; no change to how the brake is commanded; bench-proven frame content only;
  everything that is a guess named and turned into a vehicle task.
- Non-Goals: pressure sensing; fail-safe braking; power-cycling the unit; the unit's second bus;
  MAVLink brake reporting beyond three `STATUSTEXT`s (D10); a task watchdog; `master`.

## Decisions

### D1 — Owner decisions (fixed, recorded)
1. **The X7 actuator is removed** (hardware and code). The brake exists only through the iBooster.
   No fallback path, no build switch.
2. **No pressure sensor in this version.** Brake command = rod stroke, closed on the rod position the
   unit reports. `0x38D` carries the constant "5.1 bar, qualifier 1" used on the bench. A pressure
   sensor is a later change (best free ADC1 input: GPIO10 on X5 — noted, not designed).
3. **Power.** X150 pin 1 straight from the battery (4 mm², no diode, no converter): the unit
   regenerates into its supply on rod return and reboots on a supply that cannot sink current (bench:
   lab PSU rose to ~15 V, ECU restarted). Wake (pin 20) from the existing ACC / ignition line on
   Relay_1. No new relay; Relay_7/8 stay reserved. Accepted consequences: ignition OFF (also the
   fail-safe `allOff()`, starter escalation, relay-board faults that zero the port) → the unit
   sleeps → no service brake; the firmware cannot power-cycle the unit, so a latched `failure`
   needs the operator (ignition cycle may or may not clear it — task 9.3; a pin-1 power cycle is
   the confirmed way).
4. **Fail-safe unchanged** ("it will be seen after tests"). The only new behaviour on iBooster
   trouble: journal it, show it in telemetry, stop requesting until the unit is ready again.
5. **aztec only.** aztec → master is cherry-pick only; nothing of this change goes to master.

### D2 — Component shape
- **`IServiceBrake`** (`include/IServiceBrake.h`, like `IMotorDriver` / `IRelayPort`), loop-only:
  - `setTargetPct(float pct)` — the `applyBrake()` value (0 or 5–100);
  - `float targetPct()`, `float appliedPct()` (measured, 0–100 % of the max stroke, 0 when unknown);
  - `bool reached()` (D8), `bool available()` (READY / BRAKING / RELEASING), `bool requestActive()`
    (BRAKING / RELEASING), `BrakeState state()`;
  - `update(const BrakeLoopInputs&)` — once per loop: ignition state, cranking, speed, now.
- **`IboosterBrake`** (`include/IboosterBrake.h`, `src/IboosterBrake.cpp`) implements it. It owns the
  settings (NVS `brake`), turns percent into a rod target, publishes the command to the task, reads
  the task's snapshot, drains the task's brake events into the journal, and feeds telemetry.
- **`VehicleController`** holds `IServiceBrake&` instead of `BTS7960Controller& brake_`.
  `applyBrake()` keeps its clamp and calls `setTargetPct()`. `updateBrakeControl()` becomes
  `serviceBrake_.update(...)` at the same place (`:692`), so the park-brake law after it sees this
  loop's state.
- **Task side.** `CANController` stays free of iBooster knowledge. It gains one registration,
  `attachPeriodicClient(ICanPeriodicClient*)`, whose two methods run **only in the `can_io` task**:
  `onSlot(nowMs, CanSlotTx& out)` (fill up to three frames for this 10 ms slot) and
  `bool onFrame(const CanFrame&, nowMs)` (consume a frame; true = not for the RX ring).
  `IboosterBrake` implements it; its task-side state is written only by the task.
- **Why the rod loop is in the task and not in `loop()`.** The unit releases after ~150 ms without
  frames and does not re-apply; `loop()` iterations above 50 ms are normal enough to be journaled
  (`JOURNAL_LOOP_OVERRUN_MS`), ArduinoOTA runs inside `loop()`. The rod loop needs the newest
  `0x39D` and must change the flow in the same slot that is sent; putting both in the task keeps the
  timing in one place (prerequisite D1 rationale). This is a deliberate exception to the
  prerequisite's "consumers stay in the main loop" (its D1 thread model) and is written into the
  MODIFIED "CAN Receive Queue".
- **What crosses the boundary** (each a plain struct copied under its own `portMUX`, one writer):

  | Record | Writer → reader | Content | Size |
  |---|---|---|---|
  | `IbCommand` | loop → task, every loop | `stampMs`, `targetCmm` (rod target, 0.01 mm), `applyUnits`, `releaseUnits` (flow limits in `0x38C` units), `speedByte`, `ignOn`, `cranking` | 16 B |
  | `IbSnapshot` | task → loop, every slot and on every decoded frame | state, `st` / `is` / `drv`, `rodCmm`, fields A / B, `extStatus` (0x38E), `runoutBar`, `partner`, frame ages, `reqBit`, `flowUnits` sent, `reached`, last raw `0x39D` / `0x38E` / `0x38F`, counters (slots, TX misses, checksum errors, max slot gap, unit drops, stale-command max) | ≈ 80 B |
  | brake event ring (8) | task → loop | `{ms, kind, from, to, st, is, drv, rodCmm, tgtCmm, reqBit, flowUnits, raw 39D/38E/38F}` for state transitions, unit drops, slot gaps, stale-command episodes; full → drop oldest, counted | 8 × 40 B |

  The task never journals, logs or touches NVS (prerequisite rule). The loop never touches SPI.

### D3 — Frames and the 10 ms schedule (in `can_io`)
- **Slot clock.** `IB_SLOT_MS` 10. The task's wait becomes `min(CAN_POLL_MS, time to next slot)`
  (prerequisite D5). Slots are counted from `micros()`; a late slot is sent once (no catch-up burst)
  and the gap is measured (D7).
- **Buffers.** General frames keep TXB0 (TXP 1). TXB2 (TXP 3) and TXB1 (TXP 2) belong to the
  periodic client. Per slot:
  1. if TXB1 or TXB2 still has TXREQ from the previous slot → clear that buffer's TXREQ only, count
     `tx_miss` (no ABAT, as the prerequisite);
  2. build the three frames with one counter value (D5 gives the content);
  3. load `0x38D` into TXB2 and `0x38C` into TXB1, request both;
  4. `0x38B` waits: while it is pending the task's wait timeout is one tick (1 ms); the first cycle
     that sees TXB1 free loads it. Two frames take ≈ 0.4 ms on the wire, so all three leave within
     ≈ 1–2 ms of the slot.
  Order: `0x38D` is watched by the unit even at idle, `0x38C` carries the flow, `0x38B` is the most
  tolerant one (240–280 ms).
- **Across a task command (re-init, filter switch) and while the chip is down** — forced by the
  final prerequisite text: before the task starts **any** command it clears TXREQ on TXB1 and TXB2
  as well as TXB0 (each buffer alone, never ABAT), for the same reason (a pending, unacknowledged
  brake frame would hold the mode switch); the aborted brake frames count as `tx_miss`, not as
  general `tx_fail`. No brake frame is loaded while a command runs or while the chip is down, and
  nothing is queued for later: the brake frames are rebuilt every slot from the current command, so
  after the command the first slot carries fresh content (counter continued, TXP re-written). The
  brake is re-commanded by the normal loop: the target persists, and if the gap (≈ 11 ms healthy,
  up to ≈ 0.6 s typical worst case, not a strict bound per the prerequisite's D6) made the unit drop the request, it re-accepts and the P loop drives the
  rod back (D5); the gap is journaled `tx_gap`, a drop `unit_drop`. That is why re-inits and filter
  switches are kept away from a request where they can be (D9).
- **Why not TX-complete interrupts** (prerequisite D5 option): TX1IE / TX2IE change CANINTE, and the
  prerequisite's re-init check requires CANINTE = `0x03`; the 1 ms re-check needs no change there.
  The TXP bits are written after every (re-)init (the reset clears them).
- **No frames when asleep.** Frames are sent only while `ignOn` (ignition not OFF in the last
  command) **or** a valid unit frame arrived within `IB_SILENT_MS` (500 ms). With the ignition OFF
  and the unit asleep, nobody may ACK (ECU and LWS are on the same switched line); sending would only
  drive the TX error counter. The "or heard" term covers a relay expander fault that reports OFF while
  Relay_1 is physically on (Context).
- **Reception.** RXB1 keeps mask `0x7FF`: RXF2 = `0x305`, **RXF3 = `0x38E`, RXF4 = `0x38F`,
  RXF5 = `0x39D`**. RXF1 stays as it is (earmarked by `move-aztec-vesc-to-can`). RXB1 then carries
  ≈ 275 frames/s; this needs the INT wake (prerequisite D2); on the 2 ms poll alone some `0x305` /
  `0x38E` frames can be lost to RX1OVR, which the counters show (`can_int`, `ovr`), and `0x39D`
  staleness (250 ms = six frames) still protects the loop.
- **Routing.** The task offers each drained frame to `onFrame()` first. `0x38E` / `0x38F` / `0x39D`
  with a valid checksum (J1850 / sum) are decoded into the snapshot; a bad checksum is counted and
  dropped. They go into the RX ring only while the sniffer window is open, so the sniffer still sees
  them; otherwise the ring load stays at today's ≈ 110 fps.
- **Bus load.** Ours 300 fps + unit 175 fps + LWS 100 + OBD ≈ 10 → ≈ 585 fps, ≈ 13 % of 500 kbit/s.
  `0x305` has a lower ID than ours and keeps winning arbitration.

### D4 — Unit state machine (`IboosterLaw.h`, stepped by the task once per slot)
| State | Entered when | Frames | Request |
|---|---|---|---|
| `ASLEEP` | ignition OFF and no unit frame for `IB_SILENT_MS` (500 ms) | none | — |
| `STARTING` | ignition on (or the unit is heard) from `ASLEEP`; or a unit frame returns in `LOST` | idle | no |
| `READY` | fresh `0x39D` (≤ `IB_STATUS_STALE_MS` 250 ms) with status 5 or 6, `drv` ≠ 3, and `0x38F` byte 4 = `0x40` | idle | no |
| `BRAKING` | READY / RELEASING and rod target > 0 | P loop | yes |
| `RELEASING` | BRAKING and rod target = 0 | controlled return | yes, until rod ≤ 1 mm |
| `LOST` | frames enabled and `0x39D` older than 250 ms | idle | no |
| `FAULT` | fresh `0x39D` with status 2 or `drv` = 3 | idle | no |

- Status 6 counts as ready: the unit's own `st=6 is=5` cycle every ~21.6 s and the ~1 s `to_idle`
  after our release must not look like a fault or a brake.
- **Leaving FAULT / LOST** happens only on the unit's own evidence (READY condition again). The
  firmware never latches beyond the unit: a reboot-type `st=2` clears by itself in ~12 s (bench), a
  true `failure` stays until the operator power-cycles. `FAULT` lasting `IB_FAULT_PERSIST_MS`
  (20 s, longer than the 12 s re-init) is journaled once as `fault_latched`.
- **Timeouts.** `STARTING` longer than `IB_START_TIMEOUT_MS` (20 s; bench reboot → ready ≈ 12 s)
  → journal `start_timeout`, stay in STARTING. `LOST` longer than 20 s → `no_unit`. Request set but
  `is` ≠ 3 for `IB_ACCEPT_TIMEOUT_MS` (1000 ms; bench 260–290 ms) → `unit_drop` (counted, keep
  requesting: the unit re-accepts by itself after a gap).
- **Cranking** (owner: no special handling yet). `cranking` = ignition CRANKING or within
  `IB_CRANK_GRACE_MS` (3 s) after it. The machine runs exactly the same; only the journal level of a
  LOST / FAULT that starts in that window is `W` with `crank=1` instead of `E`, so a cranking dip is
  visible and is not reported as a permanent fault unless it persists (`fault_latched`).
- **Target held while not available** (owner Q2 (a), 2026-11-23). The rod target is kept. When the
  unit is READY again and the target is still > 0, BRAKING resumes by itself to the current command:
  the brake is a level command like the RC channel, and not braking when braking is commanded is the
  worse surprise.

### D5 — Rod control law
- **Percent → rod.** `target_mm = pct / 100 × max_mm`, `max_mm` ≤ `IB_ROD_CEIL_MM` (16.0). With the
  default `max_mm` = 16.0, 100 % brake is the full external-request stroke: the target sits 0.6 mm
  below the measured hard stop (16.62–16.64 mm, dry), so on a dry unit the rod reaches the target by
  the 0.2 mm tolerance and never pushes into the stop. With fluid, pressure may stop the rod earlier;
  then `reached` comes from the stall rule or `0x38F` byte 3 = 0 (D8). The ceiling stays 16.0 mm, not
  16.6: the stop was measured on one dry unit and the margin keeps the P loop from driving into it.
  The 5 % dead zone of `applyBrake()` stays, so the smallest request is 5 % of `max_mm` (0.8 mm at
  the default, below the 1.0 mm release hand-off — such a brake is released by clearing the request
  at once).
- **P loop** on the `0x39D` rod (bench-proven source; `0x38E` field A is four times faster but
  offset by −0.17…+0.27 mm — logged in the snapshot, a candidate after vehicle data):
  `flow = clamp(IB_ROD_KP × (target − rod), −release_units, +apply_units)`, `IB_ROD_KP` 250 flow
  units per mm (bench, ≈ 1.95 ml/s per mm), `IB_ROD_DEADBAND_MM` 0.05 → flow 0. Flow sent =
  `0x7E00 + flow`, always clamped to `0x6A00…0x9200`.
- **Rates.** `apply_mlps` and `release_mlps` (settings, × 128 → units) cap the flow. Defaults 8 ml/s
  ≈ 15–16 mm/s (owner decision 2026-11-23): at the cap the rod covers 16 mm in about 1 s, after the
  unit's 0.26–0.29 s acceptance delay. The P loop leaves the cap when the error falls below
  8 × 128 / 250 ≈ 4 mm; the last millimetres then close with a time constant of ≈ 0.26 s
  (kp 250 units / mm ≈ 3.8 /s), so "within 0.2 mm" of a full 16 mm target comes ≈ 1.5 s after
  acceptance — this tail is set by the constant kp, not by the rate setting. 8 ml/s is above anything
  run on the bench (≤ 4 ml/s commanded; a failure at ≈ 7.5 mm/s was blamed on thin supply wires), so
  the first vehicle runs check it (tasks 9.5, 9.6).
- **Release.** Target 0 from BRAKING → RELEASING: keep the request, return under P with the release
  cap until rod ≤ `IB_RELEASE_HANDOFF_MM` (1.0 mm), then clear the request; the unit returns the
  last millimetre by itself (bench: 4.7 mm/s from 1 mm). The request is never dropped from a large
  stroke (the only bench drop from the stop, by qualifier 0, ran at ~95 mm/s and rebooted the ECU
  on the PSU). `IB_RELEASE_TIMEOUT_MS` (3000) → clear anyway, journal `release_slow`.
- **Re-commanding after a gap.** Nothing special: the target persists and the P loop runs every
  slot, so when the unit re-accepts after a gap and the rod has fallen, the loop drives it back.
  The gap and the drop are journaled (D10).
- **Frame content.** `0x38B` byte 1 = `0x40` while requesting, `0x00` otherwise (bench and trace;
  `0x80` never); byte 2 = vehicle speed; byte 3 = `0x05`. `0x38C` byte 1 = the same request bits.
  `0x38D` = `[CRC][cnt] 00 00 00 50 47`.
- **Vehicle speed byte.** `round(km/h / 0.4)`, clamped 0–255, from `SpeedSensor::getSpeedMs()`
  when `isValid()`, else 0. The scale (0.4 / bit) is the Rivian DBC value, unit unverified; the only
  measured effect is "≥ 3 counts = moving" (pRunout 51 → 103 bar), harmless either way. A wrong scale
  shows in task 9.7. The "faulty" qualifier is never sent (it drops the request).

### D6 — Settings and what replaces the tuning card
| Setting | NVS `brake` key | Type | Range, step | Default | Note |
|---|---|---|---|---|---|
| Max stroke (100 %) | `max_mm10` | uint8, 0.1 mm | 2.0–16.0 mm, 0.5 | 16.0 mm | full request stroke; lowered for the first test, tuned with fluid (9.5, 9.6) |
| Apply rate | `apply_mlps` | uint8, ml/s | 1–40, 1 | 8 | ≈ 2 mm/s per ml/s → ≈ 15–16 mm/s |
| Release rate | `release_mlps` | uint8, ml/s | 1–40, 1 | 8 | the return before the hand-off; 16 → 1 mm in ≈ 1 s |

- Calibration section (project rule), card "🛑 Гальмо (iBooster)" in place of the old card: three
  sliders (send on release, like the governor card), status lines (unit state, `st / is / drv`, rod
  mm, partner seen, pRunout), help ⓘ, uk + en. Commands `set_brake_max_mm`, `set_brake_apply_rate`,
  `set_brake_release_rate`: persisted first, then applied; **refused while a request is active**
  ("Brake settings: not while braking") — no flash write during a request (D9) and no stroke change
  mid-brake.
- **No migration.** `travel_ms` / `hold_pwm` mean nothing for a rod. They are removed once at boot
  (`Preferences::remove`, before CAN starts, brake idle); a rollback to 1.5.37 then runs on its
  defaults, which no longer matter (no actuator).
- **Defaults — owner decision 2026-11-23, replacing the earlier Q5 defaults (8 mm, 5 / 5 ml/s):**
  maximum stroke 16 mm, apply 8 ml/s, release 8 ml/s; ranges unchanged (2–16 mm step 0.5,
  1–40 ml/s). 100 % brake therefore means the full external-request stroke. Because the effect of
  16 mm with fluid is unknown, the first-fill and first-drive procedures (tasks 9.5, 9.6) start with
  a lowered maximum stroke and raise it step by step.
- Range 1–40 ml/s: 40 ml/s is the unit's usable flow (`0x9200` / `0x6A00`), ≈ 80 mm/s; the owner
  needs the room to tune the braking response on the vehicle. Fast releases from a large
  stroke regenerate into the supply (bench reboots on a PSU) — task 9.6 records the battery voltage
  when the rates are raised.

### D7 — Loop stall (no task watchdog) — decided by default (Q3 (a), matches today's frozen target)
- The task checks `IbCommand.stampMs`. Older than `IB_CMD_STALE_MS` (500 ms) → the command is
  **stale**: the task keeps the last target and speed byte, keeps the state machine and the frames
  running, sets `cmdStale`, and records the episode length.
- Why hold: it is closest to today. When `loop()` stops today, the LEDC hardware keeps the X7
  BTS7960 at its last duty, so a held brake stays held, and `currentBrakeTarget_` is frozen. With the
  loop stopped nothing else can stop the vehicle (no MAVLink fail-safe, the throttle servo keeps its
  pulse), so releasing would be the worse choice. The bound only defines what is reported; the hold is
  open-ended until the loop returns, the ignition drops, or the ESP32 resets (then frames stop and
  the unit releases within ~150 ms — known, Risks).
- Journal on return: `BRAKE W cmd_stale ms=<n> tgt=<mm> req=<0|1>` (from the event ring).
- Slot gaps (flash erase, task starvation) are measured separately: a gap above `IB_GAP_WARN_MS`
  (50 ms) between two sent slots → `BRAKE W tx_gap ms=<n> req=<0|1>`, session maximum in telemetry.

### D8 — "Brake reached" for the park brake; "brake commanded" for the throttle
- `reached()` = state BRAKING, the unit has accepted (`is` = 3), and either
  `|rod − target| ≤ IB_REACHED_TOL_MM` (0.2 mm) or the rod has stalled: flow > 0 and less than
  0.05 mm of progress over `IB_STALL_MS` (300 ms), or `0x38F` byte 3 = 0 (stroke stop). The stall
  term covers a rod stopped by hydraulic pressure before the target (unknown with fluid, 9.6).
- **Park-brake release by a press — owner Q1 (b), 2026-11-23: release on the press alone when the
  iBooster is unavailable, as today.** `parkBrakeSvcStep()` takes `reached` and `available` (bools)
  instead of the estimate; the press definition (target 0 → > 0 under MAVLINK or WEB), the rising
  edge, the source re-seed and the cancel rules are unchanged, and the "any gear, any ignition"
  scenario stays true.
  - **Unit available** (READY / BRAKING / RELEASING): the release waits for `reached`. This is the
    behaviour closest to today's code path: today the release fires only when the service-brake
    estimate has reached the target (`parkBrakeSvcStep()` hand-over, `include/ParkBrakeLaw.h:274`);
    the measured rod replaces the estimate, nothing else changes.
  - **Unit not available** (ASLEEP, STARTING, LOST, FAULT): there is nothing to wait for — the fire
    condition is `pending && (reached || !available)`, so the release starts on the loop of the
    press. A press pending while the unit becomes unavailable fires at that moment.
  - Accepted consequence (proposal box): the buggy can then have neither brake — parking brake
    released, service brake not acting.
- Governor / limiter / slew keep `currentBrakeTarget_ > 0` (the command): the operator's intent
  closes the throttle even when the unit is down. The governor's "brake command path unchanged"
  requirement stays true (the command path is unchanged); no delta.

### D9 — Flash writes and OTA during a request
"Request active" = BRAKING or RELEASING. Budget: a 30–50 ms erase stall sits inside the measured
harmless 50–100 ms gaps; two in a row or a slow part do not.
| Writer | Rule |
|---|---|
| Journal flush | deferred like the steering drive: `EventJournal::service(steeringDriving \|\| brakeRequestActive)`, same `JOURNAL_FLUSH_DEFER_MAX_MS` (10 s) cap — the RAM tail must reach flash eventually; a flush is mostly a page program, an erase only every 4 KB of journal |
| Odometer (every 1 km, `src/SpeedSensor.cpp:168`), engine hours (every 600 s, `src/EngineHourMeter.cpp:75`) | deferred while a request is active (`FlashHold` flag), written on the first update after it ends; fail-safe and explicit persists unchanged |
| Brake card settings | refused while a request is active (D6) |
| Gear persist, park-brake state, other settings, journal switch | **not deferred**: one operator-caused NVS write each (≤ one sector erase); deferring a gear or park state would risk a stale state after a power loss; proven with a held brake in task 9.9 |
| Web OTA / LittleFS upload (`src/WebPortal.cpp:323-`, `Update.begin` `:394`), ArduinoOTA (`ArduinoOTA.handle()` `:79`), journal clear (`:297`) | **refused** while a request is active or the speed is valid and above `OTA_MAX_SPEED_KMH` (1.0): web upload answers an error before `Update.begin()`; `ArduinoOTA.handle()` is not called (the invitation times out), one log line per episode; journal clear answers an error. Invalid speed counts as standing (it is invalid after every boot until the wheels turn) |
- With the ignition OFF the unit sleeps, no request exists, nothing is refused.
- **CAN task commands during a request** (checked against the final prerequisite text):
  - LWS-silence and OBD-II no-data **watchdog re-inits stay postponed**: the loop decides when to
    post a re-init (prerequisite D6), so the gate sits in the loop's trigger path — a trigger raised
    during a request is kept pending and posted on the first `update()` after the request ends,
    with the prerequisite's spacing, retry budget and journal unchanged. Retries of a sequence that
    was already in flight when the request started continue (the chip is already failing).
  - **Bus-off** recovery and the task's own **init retry** while the chip is down (after a failed
    boot init or a failed re-init sequence) are not postponed: no frame leaves the chip anyway.
  - **Starting a sniff** is refused during a request (reason `BRAKE`). A sniff already open when a
    request starts runs to its end; its closing filter switch is not postponed (a healthy switch is
    a few ms, the brake frames are aborted before it and resume after it, D3), because holding the
    accept-all filters open would flood the ring instead.
  - ECU probe and LWS zero-set use TXB0 only and no mode change: allowed.

### D10 — Journal, telemetry, what the operator sees
- **New journal source `BRAKE`** (MODIFIED "Event Line Format"). Normal braking is **not**
  journaled (BRAKING / RELEASING transitions would flood and dedupe). Lines (≤ 120 B):
  - `BRAKE I state from=<s> to=<s> st=<n> is=<n> drv=<n> rod=<mm> ign=<0|1> crank=<0|1>` for
    ASLEEP / STARTING / READY / LOST / FAULT transitions;
  - `BRAKE E fault st= is= drv= rod= tgt= req= flow= vbat= crank=0` (level `W` with `crank=1`) and
    the companion `BRAKE I fault_raw r39d=<10 hex> r38e=<12 hex> r38f=<12 hex>` (a separate event
    so the 2 s dedupe never eats it; the pair fits the line limit only split);
  - `BRAKE E lost age=<ms> tgt= req= vbat= crank=` + `fault_raw`; `BRAKE I recovered after_s=`;
    `BRAKE E fault_latched s=20`; `BRAKE E start_timeout`; `BRAKE E no_unit`;
  - `BRAKE W unit_drop gap_ms=<n> rod= tgt=`, `BRAKE W tx_gap ms= req=`, `BRAKE W cmd_stale ms=
    tgt= req=`, `BRAKE W release_slow rod=`;
  - `BRAKE I stats tx= miss= gap_max= bad= drops= cmd_max=` every `JOURNAL_SNAP_MS` while the
    ignition is on (window values).
  `vbat` is `getBatteryV()` (X10 sense) or `-` when invalid.
- **Telemetry** (removed: `brake_travel_ms`, `brake_hold_pwm`, `brake_released`):
  `brake_pct` (measured, % of max stroke), `brake_target_pct`, `brake_rod_mm`, `brake_state`
  (token), `ib_st`, `ib_is`, `ib_drv`, `ib_partner`, `ib_runout`, `brake_max_mm`,
  `brake_apply_mlps`, `brake_release_mlps`, `ib_tx_miss`, `ib_gap_max_ms`, `ib_faults` — 15 keys,
  ≈ 280 B of JSON; the 9216-byte document's headroom (≈ 0.64 KB per the prerequisite) holds them,
  checked in task 6.4.
- **Web.** Diagnostics tile "Гальмо" / "Brake": `NN % · X.X мм` when available; otherwise the state
  in the warning / danger colour ("спить (запалювання вимкнено)" / "asleep (ignition off)" is muted). Attention box: FAULT
  and LOST (danger), STARTING beyond 20 s (warning), each with one action line ("увімкніть /
  вимкніть запалювання; не допомогло — зніміть живлення iBooster"). The Tests brake slider keeps
  working and shows the state next to its value; a request while unavailable is accepted as a
  target (it resumes, D4) and the reply says "brake unavailable".

- **GCS** (owner Q4 (a), 2026-11-23): one `STATUSTEXT` per brake availability change, through the
  existing shared `MAVLINK_STATUSTEXT_MIN_MS` (250 ms) limiter, severities as the firmware already
  uses them (`src/MavlinkInterface.cpp:962-1010`: WARNING for degraded, INFO for back to normal):
  - `Brake: LOST (no iBooster)` — WARNING, on entry into LOST;
  - `Brake: FAULT (iBooster)` — WARNING, on entry into FAULT;
  - `Brake: OK` — INFO, on READY after an announced LOST / FAULT.
  ASLEEP / STARTING are not announced (normal with the ignition switching). Anti-flapping: at most
  one brake text per `IB_STATUSTEXT_MIN_MS` (5 s); a change inside the hold-off replaces the
  pending text (latest state wins), and a pending `OK` that cancels an unsent LOST/FAULT is dropped,
  so a flapping unit costs at most one text per 5 s and the last one sent is always the current
  state. The state is also in the journal and the web, so a suppressed text loses nothing.
  The flag comes from `IboosterBrake` through the existing `MavlinkInterface::report()` struct
  (`src/main.cpp:465`, like the governor's). Docs: `MAVLINK_SETUP.md` and, per the project rule, the
  API compatibility page `UGV/API/README.md` (task 8.4).

### D11 — Removal list
- Code: X7 `BTS7960Controller brakeActuator` and its `begin()` (`src/main.cpp:43-44`, `:76`,
  `:273-279`), the endstop boot line (`:234-236`); `brake_`, `currentBrakePosition_`,
  `brakeMovementStartTime_`, `lastBrakeUpdateTime_`, `brakeSensorTriggerTime_`, `brakeIsMoving_`,
  `brakeTravelMs_`, `brakeHoldPwm_`, `isBrakeReleased()`, `getBrakeTravelMs()`, `getBrakeHoldPwm()`,
  `setBrakeTravelMs()`, `setBrakeHoldPwm()`, their web commands; the two `brake_.stop()` in
  `applyFailsafe()`; `BoardInputs::Snapshot::brakeReleased`.
- Constants: `PIN_BRAKE_RPWM`, `PIN_BRAKE_LPWM`, `LEDC_CH_BRAKE_RPWM`, `LEDC_CH_BRAKE_LPWM`,
  `BOARD_IN_BIT_BRAKE_LIMIT`, `BRAKE_FULL_TRAVEL_TIME`, `BRAKE_TOLERANCE` (→ `PB_LAW_SVC_TOL_PCT`
  goes too), `BRAKE_SENSOR_OVERRUN_TIME`, `BRAKE_HOLD_SPEED`, `BRAKE_TRAVEL_MS_*`,
  `BRAKE_HOLD_PWM_*`, `BRAKE_TRAVEL_NVS_KEY`, `BRAKE_HOLD_NVS_KEY`; `BOARD_INPUT_ACTIVE_LOW` loses
  its only user (removed). `BRAKE_NVS_NAMESPACE` stays.
- Files: `include/BrakeLaw.h`, `test/test_brake_law/`.
- Pins freed: GPIO6 / GPIO7 (X7, left unconfigured), LEDC channels 4 / 5 (the park brake's
  `begin()` now configures the shared motor timer first, `src/BTS7960Controller.cpp:34-50` already
  handles that), In5 (X17) spare.
- Web: the old card, its i18n keys (`data/index.html:1947-1958`, `:2567-2578`), its reply routing
  (`:3270-3273`).
- Docs: `GPIO_PINOUT_CUSTOM_BOARD_S3.md` (`:59-60`, `:108`, `:271-272`, `:318`, `:610`, `:635`,
  `:650` brake rows; X13 / §4 bus members and termination `:114`, `:252-262`; new X150 table);
  `WEB_PORTAL_SETUP.md` (`:90`, `:145`, `:309-328` park-brake hand-over now "brake reached");
  `openspec/project.md` aztec line ("2x Linear Actuators … brakes"). `MAVLINK_SETUP.md` and
  `UGV/API/README.md` gain the three brake `STATUSTEXT`s (D10).
- Specs: see proposal Impact.

### D12 — Version and order
- `openspec/project.md`: "breaking calibration or wiring = minor bump". This is a wiring change and a
  recalibration (new card) → **minor**.
- **Owner Q6 (a), 2026-11-23: 1.6.0 on `aztec`**, after `move-can-rx-to-interrupt` is merged in as
  1.5.38. `master` continues 1.5.x; its next minor bump is 1.7.0 (1.6.x belongs to aztec).
- Collision watch: `move-aztec-vesc-to-can` claims 1.7.0 (stale) — it must be renumbered when it is
  re-based (task 0.3).

## Risks / Trade-offs
- **No brake with the ignition OFF / on fail-safe** → accepted (D1). Visible on the tile.
- **No brake at all after a press with the unit unavailable** (Q1 (b)): the press releases the park
  brake and the service brake does not act → owner decision; stated in the proposal box, the card
  help and `WEB_PORTAL_SETUP.md`.
- **ESP32 reset or brown-out while braking** → frames stop, the unit releases within ~150 ms and
  does not re-apply until the firmware is back and READY. Same class as the fail-safe; journaled at
  boot only by the reset reason.
- **Cranking sag reboots the unit** → ~12 s without a brake after a start; journaled `crank=1`
  (task 9.4 measures).
- **Flash erase longer than ~100 ms** during a request → the unit releases, the loop re-applies
  after it re-accepts (≈ 0.2–0.3 s + travel). Mitigated by D9; measured in 9.9.
- **RXB1 load** (≈ 275 fps in one buffer) → needs INT; with polling only, frames can be lost; `0x39D`
  staleness bounds it.
- **First concurrent control loop** in the firmware (task-side state, one writer per record, pure
  law native-tested).
- **Defaults are starting points, not measured values** (max stroke at the full request stroke,
  8 ml/s rates, speed scale, reached tolerance, stall) → named constants / settings and vehicle
  tasks; 100 % brake at 16 mm with fluid is untested, so the first vehicle test runs with a lowered
  maximum stroke (9.5, 9.6).
- **Unit termination unknown** → measured before the first power-up on the vehicle (9.1).

## Migration Plan
1. Prerequisite on `master`, released and merged into `aztec`.
2. This change on `aztec` (code, tests, docs), firmware 1.6.0 + LittleFS image.
3. Hardware: remove X7 actuator and harness, fit the unit, wire X150, measure termination.
4. Bench on the vehicle (tasks 9.x) with the wheels off the ground first.
5. Rollback: flash 1.5.38 (or 1.5.37) **and refit the X7 actuator** — the old firmware has no
   iBooster path. Without the actuator, a rollback means no service brake.

## Owner answers (2026-11-23) — recorded above
- Q1 (b) release on the press alone when the unit is unavailable; waits for `reached` when it is
  available — D8.
- Q2 (a) resume braking to the current command — D4.
- Q3 (a) hold the last target on a loop stall — D7 (decided by default, not asked).
- Q4 (a) `STATUSTEXT` per LOST / FAULT / recovered, rate-limited — D10.
- Q5 range widened to 1–40 ml/s — D6. Its defaults (8 mm, 5 / 5 ml/s) were replaced by a later
  owner decision the same day: 16 mm, 8 / 8 ml/s — D5, D6.
- Q6 1.6.0 on aztec, master's next minor 1.7.0 — D12.

## Remaining open points
- None for the owner. Vehicle-tuned values (max stroke, rates, reached tolerance / stall, speed
  scale) are settled by the §9 tasks.

## Implementation notes (2026-11-23, implementation on `aztec`)
Deviations from the approved text found while implementing; the spec deltas were amended where a
scenario changes (`openspec validate --strict` clean). No owner decision is changed.

1. **Old rod sample → flow 0** (D5; spec "iBooster Rod Control", scenario "Old rod sample"). The
   approved law ran P on the newest `0x39D` until it was 250 ms old. At 8 ml/s (≈ 16 mm/s) a rod
   value 250 ms old is up to ≈ 4 mm behind, so P on it can push well past the target. The law now
   sends flow 0 (the rod holds — bench: no drift at zero flow) while the newest `0x39D` is older than
   `IB_ROD_HOLD_MS` = 100 ms (two and a half 40 ms frames) and resumes P on the next fresh frame. No
   integrator exists, so nothing winds up across the hold.
2. **A frame held in the chip through a stall is not fresh** (D3; spec "iBooster Frame Reception",
   scenario "Frame held in the chip through a task stall"). The task stamps frames when it drains
   them. During a flash-erase stall the MCP2515 keeps the first frame that arrived and drops the
   rest (RXnOVR), so the frame drained right after the stall can be up to the stall length old. A
   unit frame drained more than `IB_RX_TRUST_MS` = 25 ms after the previous slot is therefore kept
   for the journal only (counted as "late"), never used as status / rod / partner / time.
3. **No LOST with the ignition reported OFF** (D4; spec "Unit lost", new "Unit going to sleep with
   the ignition reported OFF"). With the approved rule every ignition OFF would have produced
   `READY → LOST` (an E-level `lost` line and a "Brake: LOST" STATUSTEXT) 250 ms before `ASLEEP`
   (500 ms). LOST now needs the ignition reported on; with it reported OFF the silence leads to
   ASLEEP at 500 ms. A request already running then continues with flow 0 until ASLEEP (the
   relay-expander-fault case of D3, "or heard"); no new request starts on a stale `0x39D`.
4. **Unit no longer ready → STARTING** (D4; spec "Unit no longer ready"). The approved table had no
   exit from READY / BRAKING / RELEASING for a fresh, fault-free `0x39D` that stops meeting the READY
   condition (status 1 / 3 / 4, or `0x38F` byte 4 ≠ `0x40` in READY). The state then becomes
   STARTING and the request is cleared. During a request the partner byte is not checked (it flips
   to `0x80` for ~0.1 s after a frame gap; the accept watch covers a unit that stops accepting).
   FAULT is left only for READY (unit evidence) or ASLEEP (ignition OFF + silence); a FAULTed unit
   that falls silent stays FAULT. STARTING has no LOST exit (its own 20 s `start_timeout`).
5. **One state transition per slot.** `BRAKING → RELEASING` and the hand-off `RELEASING → READY`
   happen in consecutive slots (10 ms), also for a brake already below 1 mm (5 % = 0.8 mm).
6. **BRAKE state lines exempt from the journal dedupe** (D10; spec "Availability transitions"). The
   journal's 2 s (source, event) dedupe would have eaten the second line of every quick pair
   (`ASLEEP → STARTING → READY` at ignition on, `LOST → STARTING → READY`), including the spec's own
   "Brake source" example. `jDedupeExempt()` now exempts `BRAKE state`; `IboosterBrake` limits those
   lines itself (burst 8, then 1 / s, held-back count appended as `sup=<n>`). Other BRAKE events keep
   the dedupe.
7. **Watchdog re-init after a request: re-evaluated, not latched** (D9; spec "Watchdog
   re-initializations postponed"). While the brake client holds the bus, the LWS-silence and OBD
   no-data watchdogs do not post (nor journal) a re-init; on the first `update()` after the request
   the condition is evaluated again and posts if it still holds. A latched trigger would re-init a
   bus that recovered during the request — a needless ~0.6 s gap.
8. **`reached` only for the current target** (D8; spec "Reached"). The loop runs faster than the
   10 ms slot, so a snapshot can still carry `reached` for the previous target in the loop of a new
   press. `IboosterBrake::reached()` is true only when the snapshot's target equals the target the
   loop has just published.
9. **Snapshot published once per slot**, not also on every decoded frame (D2 table): the rod loop
   and the state machine run per slot anyway; the loop sees data ≤ 10 ms old.
10. **Frame counter** advances in every slot whose frames are sent (not while the chip is down or
    the unit asleep); spec "iBooster Periodic Frames" wording amended.
11. **`tx_miss`** counts each brake frame aborted in its buffer at the next slot or before a command,
    or never loaded in its slot. An aborted frame whose attempt was already on the wire may in fact
    have gone out (ABTF not read for brake frames) — the counter errs high.
12. **`ib_gap_max_ms`** is the interval between two sent slots (10–11 ms nominal), so task 9.8's
    "< 5 ms" is amended to "≤ 15 ms".
13. **Telemetry document** is 10240 B on aztec since 1.5.36 (not 9216): the 15 keys (−3 removed)
    leave ≈ 0.86 KB — an estimate from the slot count; the "[WEB] telemetry JSON peak" console line
    measures it on the board (spec "Telemetry document capacity" amended).
14. **`VehicleController`** holds `IServiceBrake&` for every control call and `IboosterBrake&` only for
    the brake card and telemetry (one object, two views). `currentBrakeTarget_` stays the throttle
    paths' "brake commanded" and the park-brake law's input; `applyBrake()` forwards it with
    `setTargetPct()`.
15. **MAVLink**: `StateReport` carries the brake state code (`brakeState`) and the pure
    `ibStatustextStep()` (in `IboosterLaw.h`) decides the text inside `MavlinkInterface::report()`.
16. **`FlashHold`** is a one-flag header (`include/FlashHold.h`), not in the proposal's file list.
    `EventJournal::service()` keeps its signature; `main.cpp` passes
    `steeringDriving || brakeRequestActive`.
17. **Web OTA refusal** also skips the LittleFS unmount (the refusal is decided before the update
    type) and the final HTTP handler answers 409; `ArduinoOTA.handle()` is not called while refused
    unless an ArduinoOTA upload is already running. The journal clear answers 409
    `{"ok":false,…}`.
18. **Max stroke setting** accepts any 0.1 mm value in 2.0–16.0 mm (the web slider steps 0.5 mm).
19. **Found on the way (not part of the design):** the `aztec` merge of 1.5.38 had left
    `include/Changelog.h` as invalid JSON (the 1.5.38 `en` array was not closed), which breaks
    `/api/changelog` and "What's new". Fixed together with the 1.6.0 entry.
20. **Park-brake test** `test_svc_boot_estimate_hands_over_at_once` was removed with the estimate it
    described; the other cases were rewritten for `reached` / `available` and four were added.

### Implementation notes after the independent review (2026-11-23)
21. **Take-up is not "reached"** (D8; spec "Reached", new "Rod does not move", park-brake
    "Service brake ready but its rod does not move"). After accepting a request the unit first takes
    up ~1.8–2 mm of free travel (0x38E field B moves, the 0x39D output rod does not — bench). At
    ≤ ~3.5 ml/s that lasts longer than the 300 ms stall window, so the approved stall rule declared
    `reached` with no force and released the parking brake. Now a stall and `0x38F` byte 3 = 0 count
    only after the rod has advanced `IB_ARM_MM` = 0.3 mm from its first position in the BRAKING
    episode (`armed`); the tolerance rule is unchanged (a target within 0.2 mm of the rod at rest —
    only possible below 0.2 mm, e.g. 5–10 % of a 2 mm stroke — is literally reached; that is the
    commanded brake, noted for the owner). Field B was not used: the law would need a second rod
    field for a rule the output rod already answers. A rod that never arms while accepted and
    pushing for `IB_NO_MOVE_MS` = 3 s (take-up at 1 ml/s ≈ 1 s) is reported once as
    `BRAKE W no_move`; `reached` stays false, the request stays. **Park brake in that case
    (recommendation, implemented, owner to confirm):** the pending hand-over is **cancelled without a
    release** (`BRAKE W pbrake_handover_cancel why=no_move`); the parking brake stays applied until
    the Release command or a new press. Rationale: the owner's "release on the press" covers a unit
    that is visibly unavailable (state on the tile); a unit that claims READY but produces no force
    is not visible as such, and releasing then would leave the buggy unheld without warning. The
    wait is therefore bounded by 3 s of accepted pushing (or the press ending).
22. **Release time budget from the stroke and the rate** (D5; spec "Controlled release"). The fixed
    3 s cut a legitimate release at 1–2 ml/s from a long stroke (16 → 1 mm at 1 ml/s ≈ 7.9 s) and let
    the unit snap the rod back by itself — what D5 forbids. Budget = 1 s + 2 × nominal return from
    the entry rod to 1 mm at the release rate (~1.9 mm/s per ml/s), 3–30 s (30 s when the entry rod
    is unknown). Past the budget the request is cleared (`release_slow`) only once the rod has
    stopped coming down (< 0.1 mm in the last 1 s) or at the 30 s cap — bounded, no stuck state.
23. **Web upload aborted when a brake starts** (D9; ota-updates delta). The refusal was decided only
    at chunk 0, so an upload already running kept erasing flash during a request. Every chunk now
    checks the refusal flag first and calls `Update.abort()`; the page shows the reason. An aborted
    **LittleFS** upload leaves the filesystem partly written: the web page (and the journal file) are
    gone until a LittleFS upload by USB (`pio run -t uploadfs`) or ArduinoOTA — there is no fallback
    page. Accepted over erasing flash while the brake depends on the CAN task; flagged to the owner.
    At most the write of the chunk in progress when the request starts can still erase (one sector,
    30–50 ms, inside the bench's harmless 50–100 ms gaps).
24. **OTA also refused while a brake is merely commanded** (target > 0). ArduinoOTA runs inside
    `loop()` and blocks it for the whole upload, so the command the CAN task holds is frozen at its
    value when the upload began. With "request active" as the only gate, a target > 0 held while the
    unit was unavailable (owner Q2 (a)) would start a request by itself when the unit became ready —
    during the upload. Refusing while a brake is commanded closes that: an ArduinoOTA upload starts
    only with target 0 and no request, and nothing can change the frozen command until it ends
    (RC, GCS and web inputs are all handled in `loop()`). The task keeps sending idle frames; the
    stale-command flag is set and journaled after the upload (`cmd_stale`).
25. **Ignition OFF → ON re-enters STARTING** (D4; spec "Ignition switched off and on again"). An
    OFF → ON blip shorter than 500 ms (no ASLEEP) left the state READY with a silent unit, then LOST
    (E-level line + "Brake: LOST"). The ON edge now moves READY / BRAKING / RELEASING / LOST to
    STARTING (silent), and READY then needs 0x39D and 0x38F frames newer than the edge (otherwise
    the still-"fresh" pre-blip frames re-qualified READY at once). Supersedes the corresponding part
    of note 3.
26. **Periodic odometer / engine-hour saves checked every update** (spec "Periodic saves deferred").
    The due checks sat inside "wheel moved" / "second carried", so a save postponed by a request ran
    only at the next movement / engine second. They now run at the top of every update. The
    postponement lasts at most as long as the request; a request ends with the brake release, with
    the ignition OFF (unit asleep — and the ignition-OFF save runs) or on fail-safe (whose save runs at
    once, unchanged). No extra cap: a cap would put the erase inside a running request.
27. **Re-press during RELEASING needs a fresh 0x39D**, matching "no new request starts on a stale
    `0x39D`" (spec "Re-press during release").
28. **Implausible rod discarded** (spec "Implausible rod"): a checksum-valid 0x39D with a rod outside
    −1 … 40 mm is counted as bad. One comparison, and a corrupted-but-summing frame can otherwise
    command a large flow for up to 100 ms.
29. **Maximum stroke on the 0.5 mm grid** enforced by the firmware (supersedes note 18).
30. **Two earlier law tests encoded the defect of note 21** (`test_reached_by_stall_in_step`,
    `test_reached_by_runout_zero` had the rod sitting at its stall / stop position from the first
    accepted slot). They now start from rest and move first.
31. **RXB1 overflow** when 0x305 and 0x38E arrive together is accepted (owner/reviewer): it is counted
    (`ovr`, journal `CAN I stats`), and a lost 0x39D is bounded by the 100 ms rod hold and the 250 ms
    staleness rule. Bench 9.2 watches `ovr` for 10 minutes with the unit awake.
