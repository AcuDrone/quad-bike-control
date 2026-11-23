# Change: The MCP2515 gets one owner, a CAN task woken by the INT line

## Why
- **CAN is polled from the cooperative `loop()`.** `CANController::update()` reads at most 5 frames
  per call (32 while sniffing) and the MCP2515 has only two receive buffers. The Bosch LWS
  steering-angle frame `0x305` (100 Hz) has RXB1 to itself, so there is one frame of slack: any loop
  iteration longer than ≈ 10–20 ms loses LWS frames. Long iterations are normal today (web / JSON
  work, NVS writes, journal flushes, ArduinoOTA, which runs inside `loop()` for the whole upload).
  The LWS counter-jump detector (`steer_lws_miss`) and the per-minute `lws_fps` in the journal SNAP
  already show misses.
- **The bus is about to carry a brake.** A follow-up change, on the `aztec` branch only, puts a
  Bosch iBooster Gen2 on the same bus. It sends `0x38E` at 100 Hz, `0x38F` at 50 Hz and `0x39D` at
  25 Hz, and needs three frames from us every 10 ms. While a brake request is active, a gap in our
  frames longer than ≈ 100 ms releases the brake (bench-measured). A loop-polled driver whose
  transmit busy-waits inside `loop()` cannot meet that.
- **The current driver has robustness gaps** (verified on `master` 1.5.36, design "Audit"):
  - if `begin()` fails at boot, the driver is never retried for the whole power cycle;
  - a re-init against an unresponsive chip spins ≈ 2 s inside `loop()`, because each library mode
    switch waits up to 200 ms and the filter programming does sixteen of them without stopping at
    the first failure;
  - after three failed OBD-II sends, `abortTX()` aborts **all** transmit buffers, whoever loaded
    them;
  - the `mcp_can` library is built with `DEBUG_MODE 1` and prints to `Serial` on every filter write
    and every failure; the library dependency is not pinned.
- This change makes the CAN driver fit for the iBooster follow-up and for `move-aztec-vesc-to-can`,
  without anything iBooster- or VESC-specific.

## What Changes
- **One owner of the chip.** A new FreeRTOS task `can_io` performs **every** SPI access to the
  MCP2515: init, re-init, filter programming, receive, transmit, error-flag reads. `loop()` never
  touches SPI again. There is no SPI mutex (design D1).
  - Priority 12 (above AsyncTCP 10, below lwIP / WiFi / esp_timer / IPC), core 1 (the loop's core),
    4 KB stack. The task never logs.
  - Slow configuration work (init, re-init, filter switch) runs with the task temporarily at the
    loop's priority (1), so a misbehaving chip cannot starve the loop or the web server (design D6).
- **INT wake-up with a polling fallback.** A FALLING-edge ISR on GPIO38 (`PIN_CAN_INT`, already
  routed on `Control_v0`, unused today) only notifies the task. The task also wakes every
  `CAN_POLL_MS` (2 ms) on its own, so a board without the INT wire still works.
  - `old-board` (hand-wired DevKitC-1) has no INT wire and no `PIN_CAN_INT`. There the code
    compiles to polling only (`#ifdef PIN_CAN_INT`), with no edit to that branch's `Constants.h`.
  - A test build flag `CAN_TEST_NO_INT` skips `attachInterrupt()`, for the bench.
- **Receive queue to the loop.** The task drains the chip (≤ 16 frames per wake) into a 64-frame
  ring. `update()` empties the ring on every call, in every mode, and routes each frame to the
  consumers that stay in `loop()`: the LWS decoder, the OBD-II / probe state machines and the
  sniffer. When the ring is full, the **oldest** frame is dropped and counted (design D3).
- **Queued, non-blocking transmit.** OBD-II requests and the LWS zero-set frames go into an 8-frame
  TX ring. The task loads them into **TXB0 only** (FIFO) with raw SPI writes, then watches TXREQ.
  - A frame not sent within `CAN_TX_TIMEOUT_MS` (20 ms) is aborted **in its own buffer only**
    (TXREQ cleared on TXB0) and counted as a TX failure. Global ABAT and the
    "abort after 3 consecutive failures" logic are removed (design D4).
  - The zero-set asks for a TX result and aborts with `tx_failed` if its frame is not sent.
  - TXB1 and TXB2 are left unused, for the follow-up's periodic frames (design D5).
- **Boot failure is retried.** The task is created even when the first init fails. It retries the
  init every `CAN_INIT_RETRY_MS` (5 s) for the whole power cycle. On success the controller comes up
  as if `begin()` had succeeded, and the steering leaves `can_down` by itself (design D7).
- **Re-init is bounded and off the loop.** The loop still decides when to re-init (bus-off, the
  OBD-II and LWS watchdogs, sniffer restore) and keeps the existing retry and journaling rules. The
  task executes each attempt:
  - a liveness gate first: SPI reset, then CANSTAT and CANCTRL must read their reset values, else
    the attempt fails in ≈ 6 ms without any library mode switch;
  - filter programming stops at the first failing call;
  - worst case per attempt ≈ 0.6 s in the task (chip answers the reset but refuses a mode
    change); ≈ 11 ms on a healthy chip; the loop never waits (design D6).
- **Error flags in the task.** EFLG is read every `CAN_EFLG_POLL_MS` (10 ms) in every mode. RX0OVR /
  RX1OVR are counted and cleared there. TXBO is reported to the loop, which journals `bus_off` and
  requests the re-init exactly as today. TEC / REC / EFLG snapshots feed the existing health log.
- **Library.** `platformio.ini` pins `mcp_can` to commit `34807287` (the checkout the design was
  verified against) and adds `-DDEBUG_MODE=0`, so the library never prints to `Serial` (design D8).
  The library itself is not modified.
- **Diagnostics** (design D9):
  - a journal line `CAN I stats rx= drop= hwm= ovr= int= poll= tx= txf= bmax= miss=` every
    `JOURNAL_SNAP_MS` (60 s), counts per window;
  - telemetry keys `can_int`, `can_rx_drop`, `can_rx_ovr`, `can_rx_hwm`, `can_tx_fail`,
    `can_busy_max_us` (session values);
  - one muted line on the web "Стан CAN" / "CAN Status" tile (Diagnostics): the RX mode
    (interrupt / polling / no INT wire) and any drops or overflows. No persisted settings.
- **Preserved unchanged** (design D10): the OBD-II poll table and scheduler, the ECU probe, the
  sniffer, the LWS zero-set, the LWS silence watchdog, the OBD-II no-data watchdog with ignition
  gating, re-init verification and its journal economy, first-frame timing, the acceptance filters.
- **Pure logic, native-tested.** New header-only `include/CanIoLaw.h`: the frame ring
  (drop-oldest / reject-newest, high-water mark), the MCP2515 TX header encoding, READ STATUS
  decoding, the TX timeout decision, the INT state from window counters, the liveness gate. New
  suite `test/test_can_io`.
- **Version.** `FIRMWARE_VERSION` 1.5.36 → **1.5.38** on `master` (1.5.37 is taken on `aztec`),
  with a `Changelog.h` entry (uk/en). The project rule gives a **patch**: no recalibration and no
  wiring change. The owner may prefer 1.6.0 to mark the architecture change (design D12).

## Follow-up (not in this change)
- **iBooster on `aztec`.** That change will add a periodic transmit schedule **inside the
  `can_io` task**: the task's wait timeout becomes "the sooner of the poll period and the next
  10 ms slot", the three frames go into TXB1 / TXB2 at a higher TXP than the general queue, and
  possibly TX-complete interrupts. It also adds the acceptance filters and the receive decoding for
  `0x38E` / `0x38F` / `0x39D`. This change only makes that possible (design D5).
- **What this change does not solve for the brake** (design D11): a flash erase (NVS write,
  journal flush, web or ArduinoOTA write) stalls both cores for tens of milliseconds, and no task
  runs then. The follow-up must keep flash writes away from an active brake request, and must not
  let a watchdog re-init interrupt the periodic frames during one.

## Out of scope
- iBooster frames, VESC-on-CAN, any acceptance-filter change.
- Moving the consumers (LWS decoding, OBD-II parsing, sniffer table) into the task.
- A task watchdog. `openspec/project.md` calls a watchdog mandatory, but the firmware has none
  today. That gap is known and is not fixed here.
- TWAI and DroneCAN.

## Relationship to other pending changes (archive order)
- **Must be archived after** these three changes, whose CAN requirements exist today only as
  unarchived deltas. Two of them are MODIFIED here; the third is preserved and quoted:
  1. `add-can-bus-sniffer`: `CAN Bus Sniffer` is MODIFIED here.
  2. `add-event-journal-and-link-recovery`: `MCP2515 Re-initialization Verification` is MODIFIED
     here; `Steering Angle Frame Silence Watchdog` is preserved unchanged.
  3. `gate-obd-watchdog-and-pull-up-vesc-rx`: `OBD-II No-Data Watchdog Ignition Gating` is
     preserved unchanged.
  Archiving this change first would fail: its MODIFIED headers would not yet exist in
  `openspec/specs/can-controller/spec.md`.
- The archive-order list of the previous version of this proposal
  (`replace-aztec-steering-sensor-with-bosch-lws` and the 1.4.0–1.5.1 changes) is obsolete. Those
  changes are archived.
- **`move-aztec-vesc-to-can` depends on this change and is now stale.** It cites firmware 1.6.0,
  a `queueTx()` called under "one SPI mutex", and quotes this change's old `CAN Communication Error
  Handling` text. It must be re-based on this version before it is implemented (its version
  1.7.0 too). This change does not edit it. Design D5 checks that the queue, the TX design and the
  filter routine do not block it.

## Impact
- Affected specs:
  - `can-controller`:
    - MODIFIED: CAN Controller OBD-II Data Reading (start-up, polling from the receive queue),
      CAN Communication Error Handling (recovery in the task, queue drain replaces the 5-frame
      drain), MCP2515 Re-initialization Verification (attempts executed by the task, bounded),
      CAN Bus Sniffer (window opened by the task, all queued frames recorded, overflow counted by
      the task);
    - ADDED: MCP2515 Single-Owner CAN Task, CAN Receive Queue, Queued CAN Transmit, MCP2515
      Initialization Retry, CAN Driver Diagnostics.
  - `web-telemetry`: ADDED CAN Driver Telemetry.
- Affected code:
  - New: `include/CanIoLaw.h`, `test/test_can_io/test_main.cpp`.
  - Changed: `include/CANController.h`, `src/CANController.cpp` (task, ISR, rings, events,
    router; `drainRxBuffers()` and the inline drains removed); `include/Constants.h` (CAN task
    constants, `PIN_CAN_INT` comment, `CAN_MAX_CONSEC_SEND_FAILURES` and
    `CAN_SNIFF_DRAIN_PER_UPDATE` removed, version); `include/WebPortal.h`,
    `src/TelemetryManager.cpp`, `src/WebPortal.cpp` (six keys); `data/index.html` (CAN tile
    line, uk/en); `include/Changelog.h`; `platformio.ini` (lib pin, `DEBUG_MODE`, native suite).
  - Unchanged: `src/LwsSensor.cpp`, `src/SteeringController.cpp`, `src/VehicleController.cpp`
    (public `CANController` API kept).
  - Docs: `GPIO_PINOUT_CUSTOM_BOARD_S3.md` (GPIO38 now used), `WEB_PORTAL_SETUP.md` (tile line),
    `DRONECAN_RESEARCH.md` ("the firmware polls" remarks).
- Hardware: none on `Control_v0` (GPIO38 is already routed to MCP2515 `INT`). `old-board` stays
  polled.
- Safety: steering-angle freshness no longer depends on loop latency. The loop can no longer be
  blocked by the CAN chip. A dead INT line degrades to 2 ms polling from a priority-12 task, still
  far better than today.
