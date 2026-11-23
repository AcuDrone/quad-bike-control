# Design: one owner for the MCP2515 (`can_io` task)

## Context
- **Hardware.**
  - MCP2515 + TJA1051T/3 on `Control_v0`, 8 MHz crystal, 500 kbit/s, SPI at 10 MHz
    (CS GPIO39, SCK 40, MISO 41, MOSI 42, `include/Constants.h:73-76`). The MCP2515 is the only
    device on that SPI bus (no other `SPI.` user in `src/` or `include/`).
  - `INT` is routed to GPIO38: `#define PIN_CAN_INT GPIO_NUM_38 // MCP2515 INT — DEFINED ONLY, the
    driver polls` (`include/Constants.h:77`). Nothing in the firmware attaches an interrupt or
    creates a task today (no `attachInterrupt` / `xTaskCreate` in `src/` or `include/`).
  - `old-board` (hand-wired DevKitC-1): CAN on GPIO10–13 and **no `PIN_CAN_INT` at all**
    ("The MCP2515 INT line is not wired on this board — the driver polls",
    `git show old-board:include/Constants.h`, line 66).
- **Library.** coryjfowler `mcp_can` 1.5.1, checked out at `34807287`, pulled **unpinned** from
  `https://github.com/coryjfowler/MCP_CAN_lib.git` (`platformio.ini:15`).
  - `mcp_can_dfs.h:40-42`: `DEBUG_MODE` defaults to 1. Under it, `init_Mask()` / `init_Filt()`
    print "Starting to Set Mask!" / "Setting Filter Successful!" on **every** call, and every
    failure path prints too (`mcp_can.cpp:893-921`, `:979-1030`). Each `applyRxFilters()` therefore
    writes ≈ 16 lines to `Serial`, ungated by the firmware's `Debug` flags.
  - `mcp2515_requestNewMode()` spins without yielding until CANSTAT shows the mode, at most
    200 ms (`mcp_can.cpp:209-224`). `init_Mask()` / `init_Filt()` each switch to CONFIG and back,
    i.e. two such waits per call (`:893-921`).
  - `mcp2515_reset()` contains `delay(5)` (`:33-40`). `mcp2515_init()` writes
    `CANINTE = RX0IF | RX1IF` (`:622`).
  - `sendMsg()` busy-waits up to `TIMEOUTVALUE` (2500 µs, `mcp_can_dfs.h:47`) for a free TX buffer
    and again up to 2500 µs for TXREQ to clear (`mcp_can.cpp:1147-1180`). It uses all three TX
    buffers.
  - `readMsg()` uses READ STATUS, reads the buffer, then clears `RXnIF` with BIT MODIFY
    (`:1220-1240`); that clear releases `INT`.
  - `abortTX()` sets CANCTRL.ABAT, which aborts every pending buffer (`:1375-1384`).
- **Console.** `Serial` is the USB-Serial-JTAG `HWCDC` (`ARDUINO_USB_MODE=1`, Arduino core 3.3.7).
  `HWCDC::write()` waits up to `tx_timeout_ms` = 100 ms when a host is attached but not reading
  (`cores/esp32/HWCDC.cpp:48`, `:456`).
- **Scheduling.** `loopTask` priority 1 on core 1. AsyncTCP `async_tcp` priority 10, **no core
  affinity** (`AsyncTCP.h:50`, `:68`); it runs the web handlers, including the web OTA
  `Update.write()` (`src/WebPortal.cpp:410`). ArduinoOTA runs inside `loop()`:
  `ArduinoOTA.handle()` (`src/WebPortal.cpp:79`) receives the whole upload synchronously. WiFi,
  lwIP, esp_timer and IPC run at 18–24. Tick 1 kHz.
- **Where CAN runs today.** `VehicleController::initCAN()` calls `canController_.begin()` once in
  `setup()` (`src/VehicleController.cpp:317`, from `src/main.cpp:300`). `loop()` →
  `vehicleController.update()` (`src/main.cpp:380`) → `canController_.update()`
  (`src/VehicleController.cpp:355`), after `setLwsPowered()` (`:337-339`). Steering reads the
  LWS state later in the same iteration.
- **Receive paths today** (`src/CANController.cpp`):
  - NORMAL / IDLE: `drainRxBuffers()` (≤ 5 frames, `:851-872`), then `checkRxOverflow()` and
    `logHealth()` (`:256-258`). These two run **only** in IDLE.
  - NORMAL / WAITING_RESPONSE: `tryReceiveResponse()` (≤ 5 frames, stops at the first match,
    `:436-489`).
  - PROBING: `tryReceiveProbeResponse()` (≤ 5 frames, `:1024-1069`).
  - SNIFFING: `updateSniff()` (≤ `CAN_SNIFF_DRAIN_PER_UPDATE` = 32 frames, plus its own EFLG read
    and clear, `:1489-1514`).
  - Every path calls `dispatchSteeringFrame()` first (`:878-897`): `0x305` → `LwsSensor::onFrame(buf,
    len, millis())`, plus the watchdog / first-frame bookkeeping.
- **Transmit paths today.** OBD-II Mode 01 and Mode 03 (`:425`, `:1013`) and the LWS zero-set
  (`:908`) all use the library's blocking `sendMsgBuf()`.
- **Timing.** LWS `0x305` every 10 ms in RXB1 (one frame of slack); `STEER_LWS_TIMEOUT_MS` 100 ms
  (`include/Constants.h:472`). Loop iterations > 50 ms are journaled as `loop_overrun`
  (`JOURNAL_LOOP_OVERRUN_MS`, `:1052`).
- **Consumers of this change.**
  - The iBooster follow-up (`aztec` only): RX `0x38E` 100 Hz, `0x38F` 50 Hz, `0x39D` 25 Hz; TX three
    frames every 10 ms; a gap > ≈ 100 ms in our frames releases an active brake request.
  - `move-aztec-vesc-to-can`: extended-ID status bursts of three frames ≈ 260 µs apart, duty
    frames up to every 10 ms.

## Audit of `master` 1.5.36 (verified)
| # | Finding | Where | Handled in |
|---|---------|-------|------------|
| a | `begin()` failure deletes the library object and returns; `update()` returns at once while `!initialized_`; nothing ever calls `begin()` again → no CAN, no LWS (`can_down`) for the whole power cycle | `src/CANController.cpp:145-150`, `:154-160`, `:187-193`, `:220-222`; `src/SteeringController.cpp:148` | D7 |
| b | A re-init attempt runs inside `loop()`: `begin()` (reset 5 ms + up to 200 ms), `applyRxFilters()` (8 calls, `ok &=` does not stop at the first failure, each failing call up to 200 ms), `setMode()` (up to 200 ms). Unresponsive chip ≈ 2 s per attempt, up to 3 attempts 100 ms apart | `src/CANController.cpp:671-713`, `:636-643`; `mcp_can.cpp:209-224` | D6 |
| c | After 3 consecutive failed OBD-II sends: `abortTX()` (ABAT, **all** buffers) and the ABAT clear | `src/CANController.cpp:323-335`, `CAN_MAX_CONSEC_SEND_FAILURES` (`include/Constants.h:971`) | D4 |
| d | `DEBUG_MODE 1` prints from inside library calls | `mcp_can_dfs.h:40-42` | D8 |
| e | The library dependency is unpinned | `platformio.ini:15` | D8 |
| f | RX overflow and TEC/REC health are checked only in NORMAL / IDLE, never while waiting for a reply or probing | `src/CANController.cpp:256-258` | D2 |
| g | With MISO reading 0x00 (no chip), CANSTAT decodes as NORMAL mode, so `setMode(MCP_NORMAL)` can "succeed" without a chip | `mcp_can.cpp:219-221` | D6 (gate reads CANCTRL too) |

## Goals / Non-Goals
- Goals:
  - no LWS frame lost to loop or web latency (flash erases excepted, D11);
  - back-to-back bursts of at least three frames received;
  - no SPI access, busy-wait or chip wait in `loop()`;
  - re-init and boot init bounded, retried, and off the loop;
  - a TX design into which a 10 ms periodic schedule fits later without restructuring;
  - consumers keep their single-threaded model;
  - graceful degradation without the INT wire;
  - counters that show on the vehicle that it works.
- Non-Goals:
  - iBooster frames, the periodic TX schedule itself, VESC frames, filter changes;
  - surviving flash erases;
  - moving decoding into the task;
  - a task watchdog;
  - TWAI / DroneCAN.

## Decisions

### D1 — One owner task, no SPI mutex
- **Decision.** A FreeRTOS task `can_io` is the only code that talks to the MCP2515: the library
  object (`MCP_CAN`), the raw `readRegister()` / `modifyRegister()` and the new raw TX writes.
  `loop()` talks to the task only through four bounded, non-blocking channels:

  | Channel | Direction | Content | Full |
  |---------|-----------|---------|------|
  | RX ring (64) | task → loop | received frames | drop oldest, count |
  | TX ring (8) | loop → task | frames to send, with a sender tag | reject newest, count |
  | command slot | loop → task | `INIT`, `REINIT`, `FILTERS_NORMAL`, `FILTERS_SNIFF` | one command at a time; the loop never posts a second before the first is done |
  | event ring (8) | task → loop | command results, TX results for frames that asked for one, bus-off | cannot fill (every event answers a loop request or is latched once per bus-off) |

  The rings are the header-only `CanFrameRing<N>` from `include/CanIoLaw.h` (D3), each guarded by
  a `portMUX` spinlock held only for one 16-byte copy. The task is woken by direct-to-task
  notification: from the ISR (`vTaskNotifyGiveFromISR`), and from the loop after it puts a frame
  in the TX ring or posts a command (`xTaskNotifyGive`).
- **Why not one SPI mutex** (the previous version of this design):
  - The slow operations (init, re-init, filter programming, library mode switches of up to
    200 ms) would still be called from `loop()` and would still hold the loop, which item (b) of
    the audit forbids. Moving them off the loop means a request to some other context anyway.
  - The follow-up's 10 ms frames need a single place that owns timing. With a mutex, any loop-side
    holder (a sniffer filter switch, a re-init) delays that place by its own hold time.
  - Every `mcp_can_->` call would need a lock, with no compiler help against a missed one. With
    one owner, the rule is structural: the library object exists only inside the task's code.
  - Cost of the owner design: the sniffer start and the re-init become asynchronous (request →
    result on a later `update()`). Both already are, in effect: the re-init retries "on later
    `update()` calls", and the sniffer is a 10 s window.
- **Thread model.** `LwsSensor`, `VehicleData`, the probe / sniffer results, the zero-set state,
  the watchdogs and all journaling stay loop-only. The task touches only the chip, the rings, the
  command slot and its own counters (single writer each, 32-bit aligned, read by the loop).

### D2 — The task loop
- Created in `begin()` (from `setup()`, core 1), **before** the first init, and never deleted.
  `xTaskCreatePinnedToCore(canIoTask, "can_io", CAN_TASK_STACK /*4096*/, this,
  CAN_TASK_PRIORITY /*12*/, &task_, CAN_TASK_CORE /*1*/)`.
- One cycle:
  ```
  for (;;) {
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(CAN_POLL_MS));   // INT, loop request, or 2 ms poll
      t0 = micros();
      if (command pending)            run it (D6), post the result event
      if (!chipUp)                    { fail queued TX as "down"; init retry due? (D7); continue; }
      st = READ STATUS (1 SPI transaction: RX0IF, RX1IF, TX0REQ, TX1REQ, TX2REQ)
      drain: while (st has RXnIF && n < CAN_RX_DRAIN_CAP /*16*/) readMsgBuf → RX ring; st = READ STATUS
      TX:    TXB0 done / timed out? (D4); TXB0 free and TX ring not empty → load + RTS
      every CAN_EFLG_POLL_MS (10 ms): EFLG; RXnOVR → count + clear; TXBO → bus-off event once;
                                       TEC / REC snapshot
      account wake type (INT / poll), frames, busy time = micros() - t0
  }
  ```
- **Priority 12.** Above `async_tcp` (10), whose web / OTA handlers are among the things that
  delay CAN today; "loop + 1" would still be pre-empted by them. Below lwIP (18), esp_timer (22),
  WiFi (23), IPC (24). Each cycle is short (16 frames × ≈ 40 µs of SPI ≈ 0.7 ms worst, typically
  one frame ≈ 50 µs) and always ends in a blocking wait, so it cannot starve anything. The slow
  configuration work runs at priority 1 (D6).
- **Core 1** (the loop's core, `ARDUINO_RUNNING_CORE`). Core 0 carries WiFi at 18–23; on core 1
  the task is pre-empted only by core-1 ISRs and the IPC task. That is the better place for the
  follow-up's 10 ms jitter budget. Task and loop share the core, so the only concurrency is
  pre-emption, which the rings cover.
- **Stack 4096 bytes.** Call depth `mcp_can` → `SPIClass`, a 16-byte local frame, no formatting
  (the task never calls `printf`, `Debug` or `EventJournal`). The high-water mark is published
  (`uxTaskGetStackHighWaterMark`, bytes) in the boot journal line `CAN I init result=ok attempt=1
  after_s=0 stk=<n>` and on the console, so it can be cut later on evidence. *(Implementation
  note: not in the stats line, whose field list the spec fixes; the init path — library `begin()`
  → mode waits → SPI — is the deepest call chain the task has.)*
- **READ STATUS is the authority** for "a frame is pending", not the INT pin level. That keeps
  the drain correct with a dead or missing INT line.
- **The drain cap (16 per wake)** bounds one cycle. Accepted traffic today is ≈ 100–110 fps; with
  the iBooster ≈ 300 fps; the cap is reached only in a sniff of a flooding bus. A cap hit is
  counted and the next cycle (≤ 2 ms) continues.
- **Poll cost.** 500 wakes/s × (READ STATUS ≈ 10 µs + switch ≈ 5 µs) ≈ 0.8 % of core 1.

### D3 — Receive queue and routing
- `struct CanFrame { uint32_t id; uint8_t len; uint8_t data[8]; }` (16 bytes padded). `id` keeps
  the `readMsgBuf()` encoding: bit 31 = extended, bit 30 = remote. The existing comparisons
  (`canId == STEER_LWS_CAN_ID`, `0x7E8..0x7EF`) and the sniffer's decoding
  (`src/CANController.cpp:1442-1447`) work unchanged, and extended frames (VESC) pass through.
- **Depth 64** (1 KB):
  - ≥ 600 ms of today's accepted traffic and ≥ 200 ms at the iBooster's ≈ 300 fps, i.e. longer
    than any normal loop overrun (journaled above 50 ms);
  - beyond that the steering is in LWS `timeout` anyway, and a deeper ring would only deliver
    stale angles.
- **Full ring: drop the oldest.** After a long loop stall the consumers get the newest 64 frames.
  For the LWS (absolute angle in every frame) and for OBD-II (a reply older than 200 ms is
  already timed out) the newest frames are the useful ones. Counted in `rx_drop`. The task never
  blocks on the ring.
- **`update()` empties the ring first, on every call, in every mode** (NORMAL IDLE /
  WAITING_RESPONSE, PROBING, SNIFFING, zero-set, and while a re-init is in flight), at most 64
  frames per call, before any state machine step. Each frame goes through one router:
  1. `dispatchSteeringFrame()` as today (standard `0x305` → LWS sink with `millis()` at dispatch,
     watchdog arming, first-frame timing);
  2. SNIFFING: `recordSniffFrame()` (also for `0x305`, as today);
  3. NORMAL / WAITING_RESPONSE: the first `0x7E8..0x7EF` frame with `0x41` + the expected PID is
     latched into `responseData_` / `responseLen_`; PROBING: the existing Mode 01 / Mode 03 match
     is latched into the probe reply buffer;
  4. anything else → `logUnmatchedFrame()` (rate-limited, as today).
  `tryReceiveResponse()` / `tryReceiveProbeResponse()` become "take the latched reply". Sending a
  new request clears the latch. The OBD-II and probe state machines are otherwise unchanged.
- **`LwsSensor::onFrame()` keeps `millis()` at dispatch.** `LwsSensor::rollWindow()` assumes a
  non-decreasing clock across `onFrame()` and `evaluate()` (`src/LwsSensor.cpp:36-48`). A receive
  timestamp from the task could step it backwards. The age is at most one loop iteration, as
  today.
- Removed: `drainRxBuffers()`, the 5-frame loops in `tryReceiveResponse()` /
  `tryReceiveProbeResponse()`, the 32-frame loop in `updateSniff()` and
  `CAN_SNIFF_DRAIN_PER_UPDATE`. There is no compile-time polled/task switch: the poll fallback
  (D2) is the only reason one would want polling, and two receive architectures would each need
  their own bench proof.

### D4 — Transmit: TX ring, TXB0 FIFO, per-buffer timeout
- **API.** `bool CANController::queueTx(const CanFrame& f, CanTxTag tag, bool wantResult)`, loop
  only. It puts the frame in the TX ring (8) and notifies the task. Returns false when the ring is
  full (counted; never happens with today's ≤ 1 frame in flight). Tags today: `OBD`, `LWS_CFG`.
- **The task loads general frames into TXB0 only**, one at a time, in FIFO order:
  1. READ STATUS shows TX0REQ clear → one SPI WRITE burst from TXB0CTRL (written `0x01`: TXP = 1,
     TXREQ 0) through TXB0SIDH..: the 5 header bytes
     (`canTxHeader()` in `CanIoLaw.h`, same encoding as the library's `mcp2515_write_id()`,
     `mcp_can.cpp:690-716`) + DLC + data; then RTS TX0 (`0x81`). TXB0CTRL.TXP = 1.
  2. The task remembers {tag, wantResult, load time}.
  3. A later cycle sees TX0REQ clear → **sent** (no abort was requested; ABAT is never used).
  4. Still pending after `CAN_TX_TIMEOUT_MS` (20 ms) → BIT MODIFY TXB0CTRL.TXREQ = 0 (aborts
     **this buffer only**). 20 ms is far above any arbitration wait at sane load (an 8-byte frame
     is ≈ 0.25 ms) and well below the 200 ms OBD-II reply timeout. *(Implementation note, after
     review: the chip does not abort an attempt already on the wire, and if that attempt succeeds
     TXREQ clears normally. So after an abort the result is decided only when TXREQ reads clear,
     by TXB0CTRL.ABTF — set = **failed** (aborted), clear = **sent** (`CanIoLaw::txOutcome()`).
     ABTF is cleared when TXREQ is set again, so it is never stale there. TXREQ still set
     `CAN_TX_TIMEOUT_MS` after the abort (wedged controller) → reported failed, buffer kept
     blocked until it clears. The abort at the start of a command (below) waits up to 2 ms for
     TXREQ to clear and classifies the same way, before the reset erases the evidence.)*
  5. If `wantResult`, a `TX_DONE {tag, ok}` event goes to the loop. Counters `tx` / `tx_fail`
     either way.
- **Why TXB0 only.** Strict FIFO for the general senders without depending on the chip's
  equal-priority rule between buffers; ≤ 1 frame per cycle (≤ 500 fps) is far above the need
  (OBD-II ≈ 3–20 fps, zero-set 2 frames, the VESC change ≤ 100 fps). TXB1 and TXB2 stay free
  (D5).
- **Global ABAT is never used.** The audit item (c) path is removed: `abortTX()`, the ABAT clear,
  `consecutiveSendFailures_`, `CAN_MAX_CONSEC_SEND_FAILURES`. A wedged frame (typically no ACK
  because every other node is unpowered with the ignition OFF) now costs 20 ms of retransmission
  in TXB0 and is aborted alone. A genuinely wedged controller is still caught by bus-off (TXBO,
  D2) and by the OBD-II no-data watchdog, exactly as today.
- **OBD-II.** `sendOBDRequest()` / `sendMode03Request()` queue the frame without asking for a
  result; "sent" means "queued". A frame that never leaves shows up as the 200 ms reply timeout,
  already handled. A full TX ring → the request fails and is rescheduled as today.
- **LWS zero-set.** `sendLwsCfg()` queues with `wantResult`. `serviceLwsZero()` gains two wait
  phases (after `[05 00]` and after `[53 30]`) that wait for the `TX_DONE` event.
  `ok = false` or a full ring → abort with `tx_failed`, as the spec already requires. The
  `STEER_LWS_ZERO_GAP_MS` gap is measured from the confirmed send of `[05 00]`. Payloads and the
  "only transmitter on `0x7C0`" rule are unchanged. *(Implementation note, after review: a frame
  can reach the sensor although its requester heard nothing in time, so (1) `lwsZeroResetSent()`
  becomes true as soon as `[05 00]` is **queued** — the steering erases its Left/Right limits
  whatever the TX result (`SteeringController::serviceZero()`), erring toward "calibration
  lost"; only a `[05 00]` that never entered the ring leaves the limits; (2) the task always
  answers (sent, ABTF-classified abort after ≈ 20 ms, failed at once while the chip is down,
  aborted at a command start), so the wait is only a backstop, `CAN_TX_RESULT_WAIT_MS` = 1000 ms
  (a lost event, or ≈ 0.6 s of configuration work at priority 1); (3) a requester that gives up
  writes the frame's sequence number to a cancel cell and the task never loads a cancelled frame
  (`CanIoLaw::txCancelled()`), so nothing goes out late. `[53 30]` is handled the same way: any
  outcome other than a confirmed send aborts with `tx_failed`, with the limits already erased.)*
- **While the chip is down** (failed boot init, failed re-init) nothing can be sent: the task
  takes every queued frame out of the ring and fails it (`tx_fail`), and a sender that asked for
  a result hears it at once, so the zero-set aborts instead of hanging; a queued OBD-II request
  is stale by then anyway and ends in its 200 ms reply timeout. **During** a re-init or filter
  switch the task is busy in the command, so frames queued meanwhile wait in the ring and go out
  once it succeeds. A frame still pending in TXB0 when a command starts is aborted alone (TXREQ
  cleared) and reported sent or failed by ABTF (step 4): the MCP2515 changes mode only after pending transmissions, so an
  unacknowledged frame would otherwise hold every library mode switch to its 200 ms timeout.
  *(Implementation note: the approved text kept all frames in the ring while the chip was down.)*

### D5 — Room for the follow-ups (not built now)
- **iBooster periodic frames (follow-up on `aztec`).** That change adds, inside this task:
  - a slot clock: the wait becomes `min(CAN_POLL_MS, time to the next 10 ms slot)`;
  - its three frames loaded into **TXB1 / TXB2** at a higher TXP (2–3) than TXB0 (1), so a
    general frame retransmitting in TXB0 never delays them on the wire; with two buffers for
    three frames per slot it will need either TX-complete interrupts (TX1IE / TX2IE, cleared by
    the task, D2 copes with more INT sources as long as the task clears them) or a short
    re-check within the slot;
  - its acceptance filters and decoders, its own jitter / gap counters.
  Nothing in this change has to be undone for that: the task already owns time, the INT wake, and
  two untouched TX buffers. Taking TXB1/TXB2 now would not "fall out for free" (it needs a slot
  clock and its own failure semantics), so it is not done here.
- **`move-aztec-vesc-to-can`.**
  - Extended IDs pass through the ring with bit 31 set (D3); a three-frame burst is read in one
    wake (≤ 16) as long as INT works; with polling only (2 ms), RXB0 + RXB1 hold two of three —
    that change already requires INT.
  - Its transmit entry point becomes a `queueTx()` tag (`VESC`); duty frames every 10 ms fit the
    TXB0 FIFO. Its "no SPI mutex / `queueTx()` under the lock" text must be re-based.
  - Its filter change edits the one routine the task runs for `FILTERS_NORMAL` and after every
    (re-)init (`applyRxFilters()`); the router gains its dispatch at step 1.
  - Its proposal cites this change as 1.6.0 and quotes the old `CAN Communication Error
    Handling` delta; both must be updated there before it is implemented.

### D6 — Init and re-init in the task, bounded
- **The loop still decides.** Triggers stay where they are (bus-off event, OBD-II no-data
  watchdog, LWS silence watchdog, sniffer restore failure), with the same spacing, retry budget
  (`CAN_REINIT_MAX_ATTEMPTS` 3, `CAN_REINIT_RETRY_MS` 100 ms) and journal economy
  (`src/CANController.cpp:657-713`). `attemptReinit()` now posts `REINIT` and returns; the result
  event (`ok`, `begin` / `filt` / `mode` flags, CANSTAT, EFLG) completes the attempt on a later
  `update()` with the existing logging, counting and retry decision. While an attempt is in
  flight no other command is posted and the watchdogs do not re-trigger.
- **The task executes one attempt:**
  1. lower its own priority to `CAN_TASK_CONFIG_PRIORITY` (1, the loop's), so library spins are
     time-sliced with the loop (1 ms tick) and never starve it or `async_tcp`;
  2. **liveness gate**: SPI RESET (`0xC0`), 5 ms, then CANSTAT must read CONFIG (`0x80` in bits
     7:5) **and** CANCTRL its reset value `0x87`. A floating MISO (0x00 or 0xFF) fails here
     (audit g). Fail → result `begin=0`, no library call;
  3. `begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ)`; stop on failure;
  4. `applyRxFilters()` (or `applySniffFilters()`), **stopping at the first failing call**;
  5. `setMode(MCP_NORMAL)` (one retry, as the boot path always had);
     then CANSTAT and CANINTE read back (CANINTE must be `0x03`, as the
     library writes it, `mcp_can.cpp:622`; a mismatch fails the attempt);
  6. restore priority 12, give itself one notification (frames latched during the reset are read
     at once), post the result.
- **Bounds.** Healthy chip ≈ 11 ms (two 5 ms reset waits + SPI). No chip ≈ 6 ms (gate). Chip that
  answers the reset but refuses a mode change: ≤ 200 ms in `begin()`, or ≤ 400 ms in the first
  filter call, then stop: ≈ 0.6 s typical worst case, in the task at loop priority. The loop never
  waits. *(Note: not a strict bound — the library's mode-wait counter is a `byte` of `millis()`,
  so a wait pre-empted across the 200 ms mark can wrap and run up to another ≈ 256 ms; at priority
  1 that needs the loop or another task to hold the core for a long stretch. Still never in the
  loop.)*
- Frames already in the RX ring are valid and kept. TX frames queued during the attempt wait; a
  frame pending in TXB0 when it starts is aborted and classified by ABTF (D4).
- **Sniffer filter switches** (`FILTERS_SNIFF` at start, `FILTERS_NORMAL` at the end) use steps
  1, 4 and 6 only. `startSniff()` keeps its refusals; on acceptance it posts `FILTERS_SNIFF` and
  returns `STARTED`; the window starts on the success event. *(Implementation note: `mode_ =
  SNIFFING` is set at once, in an "opening" phase that records nothing, so OBD-II polling pauses
  at the start as the spec requires and a probe / zero-set / second sniff is refused as busy while
  the filters are being opened; `startSniff()` also refuses while another command, e.g. a
  re-init, is in the task.)* On failure the task puts the normal filters back and the sniff
  completes at once with `openFailed` set and no frames; if the restore fails too, the loop
  requests a full re-init (`sniff_restore`) through the normal journaled re-init path. The end of
  the window behaves as today (`src/CANController.cpp:1516-1533`), with the filter restore done by
  the task (and the same `sniff_restore` re-init if it fails).

### D7 — Boot init and its retry
- `begin()` (in `setup()`): SPI pins and `SPI.begin()` as today, create the rings and the task,
  attach the ISR, post `INIT`, then **wait in `setup()`** for the result for at most
  `CAN_BOOT_INIT_WAIT_MS` (1000 ms; healthy ≈ 11 ms, no chip ≈ 6 ms). The boot log lines and the
  return value keep their meaning, so `initCAN()` and `src/main.cpp:300-302` are unchanged.
- On failure the task stays alive and retries the init every `CAN_INIT_RETRY_MS` (5 s) for the
  whole power cycle (gate first, so a missing chip costs ≈ 6 ms per 5 s). The first failure is
  journaled `CAN E init result=fail begin= filt= mode=`; further failures are counted, not
  journaled.
- On a later success the task posts the result; `update()` then does what a successful `begin()`
  does today (`initialized_ = true`, `lastReinitTime_`, PID stagger, `src/CANController.cpp:195-208`)
  and journals `CAN I init result=ok attempt=<n> after_s=<s>`. Steering leaves `can_down` by itself
  (`src/SteeringController.cpp:148`), the zero-set stops refusing `NOT_INITIALIZED`.
- Only a failure to create the task or the rings (out of memory) makes CAN permanently down; that
  is logged and `begin()` returns false.
- *(Implementation note, after review — extends the approved behaviour, the owner is told: the
  same gated 5 s retry also runs while the chip is down after a **failed re-init** sequence
  (3 attempts, `CAN E reinit result=fail`). Without it the chip stayed down with every queued frame
  failing at once, EFLG not polled and the LWS watchdog blind, until the OBD-II no-data watchdog
  (15 s, ignition ACC/ON only) tried again. The retry is due `CAN_INIT_RETRY_MS` after the last
  attempt of any kind (`CanIoLaw::initRetryDue()`), so it never interleaves with the loop's
  100 ms-spaced attempts; a loop command always goes first. Failed retries are counted, not
  journaled; a success is journaled once as `CAN I reinit reason=task_retry result=ok attempt=<n>
  after_s=<s>`, counted in `can_reinit` and followed by the usual `first_frame` line.)*
- *(Implementation note, after review: a command result and the retry-success report each travel
  in their own single-entry slot (`CanSlot` in `CanIoLaw.h`), not the 8-entry event ring, which
  keeps only TX results and bus-off. One command is in flight at a time, so the slot cannot
  overflow, and a ring filled by a flapping bus-off during a long loop stall can no longer swallow
  a result and leave `cmdInFlight_` / `reinitPending_` stuck until reboot. A loop-side timeout was
  not chosen: the task always completes a command within its bound, and a timeout would need its
  own "late result" handling.)*

### D8 — Library: pinned, silent, unmodified
- `platformio.ini`: `https://github.com/coryjfowler/MCP_CAN_lib.git#34807287506afc6b4aef7c577392c1c4e32d1b45`.
  The design relies on `mcp2515_init()` writing CANINTE `0x03`, on `readMsg()` clearing `RXnIF`, on
  `mcp2515_write_id()`'s encoding (reproduced in `CanIoLaw.h`), and on the 200 ms mode wait.
- `-DDEBUG_MODE=0` in `[env]` `build_flags` (the header uses `#ifndef DEBUG_MODE`). No other code
  in `src/`, `include/`, the other libraries or the Arduino core uses that macro (checked). Inside
  the task a `Serial` write could block up to 100 ms (`HWCDC`, Context) and would interleave with
  the loop's console output; with the flag the library never prints. The native env overrides
  `build_flags` and does not compile the library.

### D9 — Diagnostics
- **Counters** (task-written, loop-read): `rx` frames, `rx_drop` (ring full), `rx_hwm` (ring
  high-water mark), `ovr` (RX0OVR + RX1OVR flags seen), `int_wakes`, `poll_frames` (frames found on
  a wake that was a timeout, not an INT), `cap_hits`, `tx`, `tx_fail`, `tx_qfull`, `busy_max_us`
  (longest cycle), `busy_us` (sum), stack high-water mark, init retries. *(Implementation note,
  after review: `busy_max_us` — journal `bmax`, telemetry `can_busy_max_us` — times only the
  receive / transmit / EFLG service pass; command and init-retry time (two 5 ms resets, ≈ 11 ms)
  is kept apart as `cmd_max_us` on the console `[CAN] task:` line, so the per-cycle cost of D2
  stays readable on the vehicle.)*
- **INT state** (`canIntState()` in `CanIoLaw.h`), per `CAN_INT_WINDOW_MS` (5 s): no `PIN_CAN_INT`
  or `CAN_TEST_NO_INT` → `none`; frames read and zero INT wakes → `dead`; INT wakes seen → `ok`;
  no frames → previous state (no evidence). Start: `ok` (unproven).
- **Journal**, every `JOURNAL_SNAP_MS` (60 s) while the task exists, one line from
  `CANController::update()` (`JOURNAL_LINE_MAX` 120 is respected, ≈ 100 B):
  `CAN I stats rx=6000 drop=0 hwm=3 ovr=0 int=5990 poll=10 tx=180 txf=0 bmax=310 miss=0`
  — window deltas except `hwm` and `bmax` (window maxima); `miss` = the window delta of
  `LwsSensor::missedFrames()`. A separate line rather than a third SNAP line, because the SNAP
  provider's line B is already ≈ 70 of its 96 bytes (`src/main.cpp:115`,
  `src/EventJournal.cpp:389-399`) and a third line would change the `event-journal` capability.
  Transitions of the INT state are journaled once: `CAN W int state=dead` / `CAN I int state=ok`.
- **Telemetry** (session values): `can_int` ("ok" / "dead" / "none"), `can_rx_drop`, `can_rx_ovr`,
  `can_rx_hwm`, `can_tx_fail`, `can_busy_max_us`. ≈ 6 slots, ≈ 110 B of JSON; the telemetry
  document has ≈ 0.64 KB of headroom (`src/WebPortal.cpp:671-687`), size unchanged.
- **Web**: one muted line under "Стан CAN" / "CAN Status" (Diagnostics): "RX: переривання" /
  "RX: interrupt", "RX: опитування (INT не працює)" / "RX: polling (INT dead)" in the warning
  colour, "RX: опитування (без INT)" / "RX: polling (no INT wire)"; plus "втрачено N" / "dropped N"
  and "переповнення N" / "overflow N" when non-zero. Not an attention item. No persisted setting,
  so nothing goes into Calibration.
- The `[CAN] health:` and `[CAN] RX overflow` console lines stay (now fed from the task's
  snapshot and counters, in every mode).

### D10 — Behaviour that must not change
| Behaviour | Requirement (source) | How it survives |
|-----------|----------------------|-----------------|
| OBD-II poll table, scheduler, one request in flight, 200 ms timeout, stale data 5 s | `CAN Controller OBD-II Data Reading`, `CAN Communication Error Handling` (specs) | state machine untouched; replies come from the router |
| ECU probe | `ECU Capability Probe` (specs) | probe sequencer untouched; replies from the router |
| Steering-angle acceptance paths | `Shared-Bus Steering Angle Frame Reception` (specs) | `applyRxFilters()` unchanged, run by the task |
| Zero-set payloads and refusals | `Steering Sensor Zero-Set Sequence` (specs) | queued with a TX result, D4 |
| LWS silence watchdog, ignition gating, 5 s spacing, journal economy | `Steering Angle Frame Silence Watchdog` (`add-event-journal-and-link-recovery`) | loop-side, unchanged; re-init executed by the task |
| First-frame timing | same requirement | loop-side, at dispatch |
| Re-init verification, 3 attempts, journal lines | `MCP2515 Re-initialization Verification` (same change) | MODIFIED only for "executed by the task, bounded" |
| OBD-II watchdog ignition gating | `OBD-II No-Data Watchdog Ignition Gating` (`gate-obd-watchdog-and-pull-up-vesc-rx`) | loop-side, unchanged (`CanWatchdogLaw.h`) |
| Sniffer | `CAN Bus Sniffer` (`add-can-bus-sniffer`) | MODIFIED: frames from the ring, overflow from the task, async open |
| `clearEngineRpm()`, `setLwsPowered()`, `getReinitCount()`, public API | (code, `include/CANController.h`) | unchanged |

### D11 — Flash erases and OTA: what this solves and what it does not
- **Solved.** A long `loop()` iteration (web / JSON work, a slow NVS write in the loop, a journal
  flush, the whole ArduinoOTA upload) no longer loses frames at the chip: the task keeps draining
  into the ring and keeps sending queued frames. A busy `async_tcp` handler (web OTA writes) no
  longer delays reception (priority 12 > 10).
- **Not solved: consumers freeze with the loop.** During ArduinoOTA the LWS decoder, the steering
  and OBD-II parsing do not run; the ring then keeps the newest 64 frames and counts drops. That
  is as today for the vehicle (the steering never ran during ArduinoOTA).
- **Not solved: flash erases.** Erasing or writing SPI flash disables the cache on both cores; no
  flash-resident task runs, `can_io` and the SPI driver included. A 4 KB sector erase is typically
  30–50 ms (much longer on some flash parts). With one LWS frame of slack in RXB1, each erase can
  still lose ≈ 3–5 LWS frames (no LWS `timeout` unless one erase exceeds 100 ms). Sources: NVS
  writes (settings), journal flushes (only with the flash-write switch on), web OTA, ArduinoOTA,
  LittleFS upload.
- **Consequence for the iBooster follow-up.** A 30–50 ms erase is inside its ≈ 100 ms budget, a
  slow erase may not be, and two erases back-to-back are not. The follow-up must keep flash writes
  away from an active brake request (as the journal already defers its flush while the steering
  drives) and treat OTA as "vehicle parked, no brake request". Making the task and the SPI driver
  IRAM-resident or using flash auto-suspend are possible later steps, not part of either change.

### D12 — Version
- `openspec/project.md` "Versioning and Changelog": "breaking calibration or wiring = minor bump,
  fixes = patch". This change needs no recalibration and no wiring change on any board → **patch**.
- `master` is at 1.5.36; `aztec` already used 1.5.37 (`git show aztec:include/Constants.h`:
  `FIRMWARE_VERSION "1.5.37"`). No active change claims 1.5.38 → **1.5.38**.
- **Flag for the owner:** it is an architecture change (first task, first ISR in the firmware).
  Choosing 1.6.0 instead is the owner's call; if so, `move-aztec-vesc-to-can` (which assumes 1.6.0
  for this change and 1.7.0 for itself) keeps its numbers. With 1.5.38 that change must be
  renumbered when it is re-based anyway.

## Risks / Trade-offs
- **First concurrent code in the firmware.** A missed channel would be a race. Mitigations: the
  library object and the raw SPI helpers are private to the task's code path (a grep task:
  `mcp_can_->`, `readRegister(`, `modifyRegister(` appear only in task functions); rings and
  counters have a single writer each; the ring logic is native-tested.
- **Priority 12 above AsyncTCP.** A bug that makes the task spin (READ STATUS always reporting a
  frame) would starve the web server. The per-wake cap and the blocking wait bound it to ≈ 0.7 ms
  per 2 ms; `cap_hits` and `busy_max_us` make it visible.
- **Custom TX code next to the library.** The header encoding is native-tested against vectors
  from the library's own encoder; READ STATUS, WRITE, RTS and the TXREQ abort are the documented
  MCP2515 instructions.
- **Asynchronous sniffer start.** The operator sees `STARTED` before the filters are open; an open
  failure shows as a completed sniff with `openFailed`. Rare, and visible.
- **INT edge loss.** INT is a level; a FALLING-edge ISR can miss an edge when INT never went high
  between frames. The drain loop reads until READ STATUS is empty, and the 2 ms poll catches the
  rest; `poll_frames` makes such cases visible.
- **GPIO38 not yet proven on hardware.** If it does not toggle, the change still works on the 2 ms
  poll and reports `can_int = dead`. `move-aztec-vesc-to-can` would then need the board looked at.

## Migration Plan
1. Implement and test on `master`, release 1.5.38 (firmware + LittleFS image for the tile line).
2. Merge `master` → `aztec` (plain merge, as for every shared change; `aztec` → `master` is never
   merged): resolve `FIRMWARE_VERSION` to 1.5.38 and keep `aztec`'s 1.5.37 changelog entry below
   the new one.
3. Merge `master` → `old-board`: its `Constants.h` has no `PIN_CAN_INT`, so the code builds
   polling-only; `can_int` reports `none`.
4. Bench on both vehicles (tasks.md §8) before field use.
5. Rollback = flash 1.5.36 (`master`) / 1.5.37 (`aztec`) and their LittleFS images. Nothing is
   stored in NVS.

## Open Questions
- Version 1.5.38 (rule) or 1.6.0 (architecture marker)? See D12.
- Keep TXB1 / TXB2 unused until the iBooster change (recommended), or let the general queue use
  all three buffers now and re-partition later?
- Boot init retry every 5 s forever (recommended), or give up after N attempts?
- If the bench shows GPIO38 does not follow MCP2515 `INT` on `Control_v0`: ship on the 2 ms poll
  and fix the board before the iBooster/VESC changes (recommended), or hold the release?
- CAN counters as their own `CAN I stats` journal line (recommended), or as a third SNAP line
  (needs a delta on the `event-journal` capability)?
