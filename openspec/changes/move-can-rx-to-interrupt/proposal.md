# Change: MCP2515 reception moves to an interrupt-driven reader task

## Why
CAN reception is polled from the cooperative main loop. `CANController::update()` reads at most 5
frames per call, through `drainRxBuffers()` (IDLE), `tryReceiveResponse()` (WAITING_RESPONSE) or
`tryReceiveProbeResponse()` (probe). The MCP2515 has only two receive buffers, and the Bosch LWS
`0x305` frame (100 Hz) has RXB1 to itself, so one frame of slack. A long loop iteration (web/JSON
work, NVS writes, OTA) or an AsyncTCP handler that keeps the CPU loses LWS frames. The
`steer_lws_miss` counter in telemetry shows it.

The pending change `move-aztec-vesc-to-can` adds VESC STATUS_1/4/5 frames, sent back-to-back about
260 µs apart at 500 kbit/s. Polled reception with two hardware buffers cannot catch a 3-frame
burst. Interrupt/task-driven reception is a hard prerequisite of that change.

## What Changes
- **Interrupt.** The MCP2515 `INT` line (GPIO38, `PIN_CAN_INT`) gets a FALLING-edge ISR. The ISR
  does no SPI. It only counts and gives a binary semaphore (design D1).
- **Reader task.** A new FreeRTOS task `can_rx` (priority 12, 4 KB stack, pinned to core 1 next to
  the Arduino loop) is now the only code that reads the MCP2515 receive buffers (design D2):
  - It waits on the semaphore with a 2 ms timeout, so it falls back to polling if the INT line
    is dead.
  - After each wake it reads frames while `checkReceive()` reports one, up to 16 frames per wake.
- **Software queue.** Frames go into a FreeRTOS queue (32 entries). Each entry holds the ID in the
  `readMsgBuf()` format with the ext/RTR flags, the DLC, 8 data bytes and the receive time.
  - `CANController::update()` drains the queue on every call, in every OBD-II, probe and zero-set
    state.
  - A single router hands `0x305` to `LwsSensor` and OBD-II replies to the waiting request, and
    logs the rest as unmatched. The consumers stay in the main loop, so `LwsSensor` and the OBD-II
    state machine keep their single-threaded model (design D3).
- **One SPI mutex.** Every MCP2515 access is serialised by one mutex: library calls and the raw
  `readRegister()`/`modifyRegister()`. Waits on it are bounded (10 ms). A timeout is counted and
  logged, never infinite. Re-initialisation holds the mutex for its whole duration, and the task
  skips reading while it runs (design D4, D7).
- **Non-blocking transmit.** OBD-II requests are queued into a free MCP2515 TX buffer by a new
  `queueTx()` helper (one short locked SPI sequence) instead of `sendMsgBuf()`. `sendMsgBuf()`
  busy-waits up to 2 × 2.5 ms for the frame to leave, which would block the reader task long
  enough to lose a burst. The LWS zero-set keeps `sendMsgBuf()`: it is rare and operator-started,
  and it needs a confirmed transmit (design D7).
- **Interrupt sources.** CANINTE = RX0IE | RX1IE only. The library's `mcp2515_init()` already
  writes this value. `CANController` re-asserts it with its existing raw `modifyRegister()` after
  every `begin()` and re-init, and reads it back. The `mcp_can` library is **not** modified.
  EFLG, TXBO and RX overflow are still read from the main loop, now every 100 ms in every
  mode (design D5).
- **Acceptance filters unchanged** (RXB0 OBD-II `0x7E8`–`0x7EF`, RXB1 LWS `0x305`, BUKT rollover)
  (design D8).
- **Removed:** `drainRxBuffers()` and the inline 5-frame drains in `tryReceiveResponse()` and
  `tryReceiveProbeResponse()`. There is no compile-time polled/interrupt switch (design D9).
- **Diagnostics.**
  - New counters:
    - `can_isr_count`;
    - `can_rx_task_frames`;
    - `can_rx_queue_drops`;
    - the queue high-water mark;
    - the per-wake cap hits;
    - SPI lock timeouts;
    - the cumulative RX overflow count;
    - the task's stack high-water mark.
  - All of them are printed in one rate-limited `[CAN] rx:` line.
  - An INT-inactive detector (frames arrive but no ISR fired for 5 s) logs
    `[CAN] INT inactive, polling fallback` once, and `[CAN] INT active` on recovery
    (design D6).
- **Telemetry.** The new keys are `can_int_ok` and `can_rx_queue_drops`. The web "Стан CAN" tile
  gets one muted line, "RX: переривання" or "RX: опитування (INT не працює)", plus the drop count
  when it is non-zero.
- **Pure logic, host-tested.**
  - `include/CanRxMonitor.h`: the INT-inactive detector.
  - `include/Mcp2515Tx.h`: the TX header encoding and free-buffer choice.
  - New native suite `test/test_can_rx_irq`.
- **Library pin.** `platformio.ini` pins `mcp_can` to the commit the design was checked against
  (`34807287`), because the design relies on its `mcp2515_init()` CANINTE value and its
  `readMsg()` flag clearing.
- **Version.** `FIRMWARE_VERSION` 1.5.1 → **1.6.0**, with a `Changelog.h` entry (uk/en) dated
  2026-10-29. No recalibration and no wiring change: GPIO38 is already routed to the MCP2515 `INT`
  on `Control_v0`.

## Out of scope
- New acceptance filters or VESC frames. They belong to `move-aztec-vesc-to-can`.
- Moving the consumers (`LwsSensor`, OBD-II parsing) into the task.
- TWAI and DroneCAN.
- Removing the flash-write stalls of OTA/NVS: with the cache disabled no task runs. See design
  "Risks".

## Relationship to other pending changes (archive order)
This change quotes the pending `can-controller` deltas of
`replace-aztec-steering-sensor-with-bosch-lws` (`CAN Communication Error Handling`). It SHALL be
archived after that change, and after the other changes of the 1.4.0–1.5.1 baseline, in this order:
1. `replace-aztec-steering-sensor-with-bosch-lws`
2. `redesign-web-ui-service-console`
3. `add-aztec-steering-rate-mode`
4. `auto-detect-aztec-steering-sensor-direction`

It touches no requirement those last three modify. Its web-telemetry delta is ADDED-only.

`move-aztec-vesc-to-can` (renumbered to 1.7.0) depends on this change functionally. It SHALL be
archived **immediately after** this one. Its MODIFIED `CAN Communication Error Handling` block
is re-based on this change's text.

## Impact
- Affected specs:
  - `can-controller`:
    - MODIFIED: CAN Controller OBD-II Data Reading (the WAITING_RESPONSE scenario reads the
      queue, and requests are queued for transmission), CAN Communication Error Handling
      (re-init under the lock, queue drain replaces the 5-frame hardware drain)
    - ADDED: Interrupt-Driven CAN Reception, MCP2515 SPI Access Serialization, CAN Receive Path
      Diagnostics
  - `web-telemetry`: ADDED CAN Receive Path Telemetry
  - `debug-logging`: unchanged. The new log lines are specified in `can-controller` and follow
    the existing `Debug::printfFeature(DebugFeature::CAN, …)` rate-limited style.
- Affected code:
  - New: `include/CanRxMonitor.h`, `include/Mcp2515Tx.h`, `test/test_can_rx_irq/test_main.cpp`.
  - Changed:
    - `include/CANController.h`, `src/CANController.cpp`: task, ISR, queue, mutex, `queueTx()`,
      router, diagnostics; drains removed.
    - `include/Constants.h`: `PIN_CAN_INT` comment and new CAN RX constants; `FIRMWARE_VERSION`.
    - `include/TelemetryManager.h`, `src/TelemetryManager.cpp`, `src/WebPortal.cpp`: two keys.
    - `data/index.html`: CAN tile line and i18n.
    - `include/Changelog.h`.
    - `platformio.ini`: `mcp_can` pin, native `test_filter`.
  - Docs: `GPIO_PINOUT_CUSTOM_BOARD_S3.md`, `openspec/project.md`, `WEB_PORTAL_SETUP.md`,
    `DRONECAN_RESEARCH.md` (the "polled" remarks).
- Hardware: none. GPIO38 is already wired to MCP2515 `INT`.
- Safety: steering-angle freshness improves. Every MCP2515 access becomes concurrent, and the
  single mutex with bounded waits (D4, D7) is what keeps that safe. A dead INT line degrades to
  2 ms polling by the task. That is still far better than today's loop polling.
