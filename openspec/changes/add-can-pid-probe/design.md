## Context
`CANController` (`src/CANController.cpp`) drives an MCP2515 (500 kbps, SPI CS GPIO 10, MOSI 11,
SCK 12, MISO 13) with a non-blocking `IDLE → WAITING_RESPONSE` state machine (`:71-154`). Each
`update()` either sends one OBD-II Mode 01 request (`{0x02,0x01,PID,0x00…}` to functional ID
`0x7DF`, `sendOBDRequest` `:160-178`) or performs one non-blocking receive that accepts
`0x7E8–0x7EF` responses where `byte[1]==0x41 && byte[2]==PID` (`tryReceiveResponse` `:180-214`).
Response timeout is 200 ms (`CAN_RESPONSE_TIMEOUT`), with up to 3 retries; `selectNextPID`
(`:216-231`) picks the most-overdue PID. The active table (`include/CANController.h:81-101`,
`PID_COUNT=3`) polls 0x0C RPM (500 ms; 50 ms during gear-change boost), 0x05 coolant (2000 ms),
0x11 TPS (2000 ms). Stale-data timeout is 5000 ms (`CAN_DATA_STALE_TIMEOUT`).

The Bashan 330's Delphi MT05-family ECU answers this polling but has no published PID map, so we
need an on-demand probe to learn what it supports before expanding telemetry.

## Goals / Non-Goals
- Goals: a web-triggered, bounded, one-shot capability probe that reports supported-PID bitmaps,
  per-candidate-PID answers (raw + decoded), and a DTC summary; deliver results to the web UI and
  serial log; wire MAP (0x0B) into normal polling/telemetry as the first evidence-based addition.
- Non-Goals: no continuous DTC monitoring, no DTC clear (Mode 04), no permanent DTC UI, no
  multi-frame ISO-TP reassembly (documented limitation), and no enabling of the other candidate
  PIDs (deferred to a future change driven by the bench results).

## Decisions

### Probe as a third top-level state, reusing the existing helpers
Do NOT redesign the existing polling state machine. Add a coarse controller mode alongside the
normal polling machine:

- `Mode { NORMAL, PROBING }`. `can_probe` sets `PROBING` and initializes a small probe
  sub-state cursor; `update()` dispatches to the probe sequencer while `PROBING`, and to the
  existing `IDLE/WAITING_RESPONSE` machine while `NORMAL`.
- The probe sequencer reuses `sendOBDRequest()` verbatim for Mode 01 requests and adds one thin
  helper for the Mode 03 request (same `sendMsgBuf` to `0x7DF` with `{0x01,0x03,…}`). It reuses
  the same non-blocking receive/drain loop as `tryReceiveResponse()`, generalized to match either
  a Mode 01 positive response (`0x41`) for the expected PID or a Mode 03 response (`0x43`). The
  per-request timeout stays `CAN_RESPONSE_TIMEOUT` (200 ms) with **1 retry** per request (probe is
  a diagnostic sweep, not steady-state polling, so one retry keeps worst-case duration bounded).
- The sequencer walks an ordered work list, advancing the cursor on each answered/timed-out
  request so exactly one request is in flight at a time (mirrors the normal machine and keeps
  `update()` non-blocking):
  1. Mode 01 PID 0x00 (supported 0x01–0x20). Record the 4-byte bitmap. If bit for 0x20 is set,
     enqueue 0x20; likewise 0x20→0x40, 0x40→0x60. Only walk the next bitmap when the previous
     bitmap's "PID 0x20 supported" bit indicates more (`byte[3] & 0x01` of that bitmap group).
  2. One test request per candidate PID regardless of bitmap:
     0x04 (calc load), 0x0B (MAP), 0x0E (timing advance), 0x0F (IAT), 0x42 (control-module
     voltage), 0x0D (speed), 0x2F (fuel level), 0x5C (oil temp), 0x14 (O2 S1). Sending regardless
     of the bitmap catches ECUs (like this one) whose bitmap is incomplete or nonstandard.
  3. One Mode 03 request (stored DTC read).
  On completion the sequencer sets a results-ready flag/timestamp and returns to `Mode::NORMAL`.

### Results model
A fixed-size in-memory snapshot owned by `CANController`, retrievable via a getter
(`getProbeResults()`), mirroring the existing `getVehicleData()` pattern:

- `bitmaps[4]` — one entry per group (0x00/0x20/0x40/0x60): `{queried, raw[4]}`.
- `pids[N]` — one entry per candidate PID: `{pid, supportedByBitmap, answered, rawLen, raw[4],
  decoded (float), hasDecoded}`. Decoding uses the standard OBD-II formulas for the PIDs we know
  (e.g. MAP = A kPa, IAT = A−40 °C, module voltage = ((A*256)+B)/1000 V); PIDs without a known
  formula store raw bytes only with `hasDecoded=false`.
- `dtc { queried, count, codes[K] }` — decoded 2-byte DTCs from the single Mode 03 frame.
- `state { running, complete, completedAtMs }` plus a short human-readable status string.

Sizes are small fixed arrays (no heap): candidate PID count is a compile-time constant, and DTC
codes are capped at what one classic 8-byte frame carries (up to 2 full DTCs after the count
byte). If the ECU reports more than 2 DTCs the extra codes require multi-frame ISO-TP, which the
probe does not reassemble — this is recorded as a `multiFrameTruncated` flag and documented as a
known limitation, not a defect.

### Delivering results to the web UI: embed in telemetry while fresh (chosen)
The web UI already receives a 5 Hz `ws.textAll(json)` telemetry broadcast built in
`WebPortal::createTelemetryJSON` (`src/WebPortal.cpp:573+`), and `TelemetryManager` already copies
`VehicleData` into the `Telemetry` struct each cycle (`src/TelemetryManager.cpp:68-85`). The
simplest mechanism consistent with that pattern is to have `TelemetryManager` copy the probe
snapshot into the `Telemetry` struct and have `createTelemetryJSON` emit a nested `probe` object
**only while results are fresh** (a bounded window, e.g. `complete && now-completedAtMs <
PROBE_RESULT_TTL`, plus `probe.running` so the UI can show progress). Once the TTL elapses the
`probe` object is omitted and the panel keeps its last values until the next probe.

Rationale for this over a one-shot dedicated WS message type:
- No new send path, no client-side message-router branch, and no risk of a client that connected
  after the probe missing a fire-once message — a freshly connected client still sees the
  in-window `probe` object on the next 5 Hz frame.
- Results are static once the probe completes (~10 small fields + short arrays), so re-sending
  them for a few frames is cheap and stays within the existing JSON budget. The doc capacity is
  already 2048 bytes (`src/WebPortal.cpp:576`); the `probe` object is only present transiently, so
  steady-state telemetry size is unchanged. If the combined payload approaches the budget the
  `probe` object can be trimmed to raw+decoded essentials before shipping.
- Matches the existing "include CAN fields only when connected" conditional-emission idiom.

The `running` flag lets the button show an in-progress state and lets the UI reject a second
click until the current probe finishes.

### Polling suspension and resume
Entering `PROBING` stops the normal scheduler from issuing requests (the mode dispatch simply
does not run the `IDLE/WAITING_RESPONSE` branch). `VehicleData` is left untouched during the
probe — its `lastUpdateTime` is not advanced — so on resume the normal machine picks up exactly
where it left off. RPM boost interval changes are inert during a probe because the normal machine
is paused; the existing `setRPMPollInterval` calls from `VehicleController` still just write the
table entry.

### Edge cases
- **Gear change active**: if a gear change is in progress when `can_probe` arrives (the same
  condition that drives RPM boost, `transmission.needsThrottleBoost()` — already surfaced as
  `gear_switching` telemetry), the probe is rejected/deferred and the command responds with a
  busy/deferred result rather than suspending polling mid-shift. This protects the higher-rate RPM
  feedback the shift relies on.
- **CAN disconnected**: if `VehicleData.dataValid` is false when `can_probe` arrives, the probe
  short-circuits immediately with a "no ECU / disconnected" result (no bus traffic generated) so
  the UI gets a fast, clear answer.
- **Partial responses**: any candidate PID or bitmap group that times out (after its single
  retry) is recorded as `answered=false` and the sequencer advances; a Mode 03 with no response is
  recorded as `dtc.queried=true, count=unknown`. A probe never hangs — every step has the 200 ms
  timeout and the cursor always advances.

### RPM-staleness safety reasoning
Ignition/relay logic in `main.cpp` uses `engineRPM` for engine-running detection, and stale CAN
data is invalidated after `CAN_DATA_STALE_TIMEOUT` = 5000 ms. Worst-case probe duration is
bounded: at most 4 bitmap groups + 9 candidate PIDs + 1 Mode 03 = 14 requests, each ≤ 200 ms
timeout × (1 send + 1 retry) ≈ 400 ms worst case, i.e. ≈ 5.6 s absolute worst case but typically
~3 s because a healthy ECU answers in ~50 ms (most requests complete well under their timeout).
Because the probe is only allowed when a gear change is NOT active (engine-running detection is
not in a safety-critical transition), and typical duration is well under 5000 ms, RPM will
normally not be marked stale during a probe. To remove even the worst-case edge, the probe budget
(retries × candidate set) is kept small enough that the expected duration stays under the stale
timeout; the worst-case tail is acceptable because it only defers, never falsifies, engine-running
detection (a transient `dataValid=false` fails safe to the existing timeout fallbacks). This
reasoning is why the probe uses 1 retry (not 3) and a fixed candidate list.

### MAP scaling and consumers
MAP (PID 0x0B) decodes as `A` in kPa absolute (0–255 kPa), single data byte. It is added to the
active poll table at the 2000 ms interval class (same class as coolant/TPS). A new
`VehicleData.mapKpa` (`uint8_t`) field carries it; `parseAndStore` gains a `case PID_MAP`.
`TelemetryManager` copies it into the `Telemetry` struct alongside the other CAN fields (only when
`dataValid`), and `createTelemetryJSON` emits `map_kpa` inside the existing
`can_status == "connected"` block. The web UI shows it in the CAN telemetry card in kPa. No
MAVLink `EFI_STATUS` consumer change is required by this change.

### Serial log mirroring
Probe progress and the final results table are mirrored to the serial console via
`Debug::printfFeature(DebugFeature::CAN, ...)` / `printlnFeature`, matching the existing CAN
logging in `CANController.cpp` (`:36`, `:108-112`, `:126-128`). This gives a bench operator a
readable transcript (per-PID answered/raw/decoded and the DTC summary) even without the web UI.

## Risks / Trade-offs
- Transient `dataValid=false` if a probe runs unusually long → mitigated by the small fixed budget
  and gear-change deferral; fails safe to existing timeout fallbacks.
- Multi-frame DTC responses (>2 DTCs) are truncated → documented limitation, flagged in results.
- Sending candidate PIDs the bitmap says are unsupported may draw a negative/no response →
  handled as a normal timeout; intentional, because this ECU's bitmap is not trusted.

## Migration Plan
Additive only. No stored data or API removed. Enabling the remaining candidate PIDs is a separate
future change gated on the captured bench results.

## Open Questions
- Exact decoded formulas for any non-standard PIDs this ECU exposes will be confirmed from the
  bench probe transcript before a future change enables them.
