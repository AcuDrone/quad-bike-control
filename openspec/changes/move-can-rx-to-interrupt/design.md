# Design: interrupt-driven MCP2515 reception

## Context
- **Hardware.**
  - MCP2515 + TJA1051T/3, 8 MHz crystal, 500 kbit/s.
  - SPI at 10 MHz (CS GPIO39, SCK 40, MISO 41, MOSI 42). The MCP2515 is the only device on that
    SPI bus.
  - `INT` is routed to GPIO38 (`PIN_CAN_INT`), today "DEFINED ONLY, the driver polls".
- **Library.** coryjfowler `mcp_can` 1.5.1, git checkout `34807287`, not pinned in
  `platformio.ini` today.
  - `mcp2515_init()` (called by `begin()`) writes `CANINTE = MCP_RX0IF | MCP_RX1IF` (0x03,
    `mcp_can.cpp` l.622) and enables BUKT.
  - `readMsg()` uses READ STATUS, reads the buffer with plain READs, then clears `RXnIF` with
    BIT MODIFY. That clear is what releases `INT`.
  - `sendMsg()` busy-waits up to `TIMEOUTVALUE` (2500 µs) for a free TX buffer, and then up to
    another 2500 µs for `TXREQ` to clear, i.e. for the frame to be on the wire and ACKed.
  - The library keeps per-object message state (`m_nID`, `m_nDta`, …): it is not re-entrant.
  - There is no `setINTE()` and no `getMode()`. `CANController` already has raw
    `readRegister()` / `modifyRegister()` on the same SPI settings.
- **Receive paths today.**
  - IDLE: `drainRxBuffers()`, then `checkRxOverflow()` and `logHealth()`.
  - WAITING_RESPONSE: `tryReceiveResponse()`. It stops at the first matching reply.
  - Probe: `tryReceiveProbeResponse()`.
  - Each reads ≤ 5 frames per `update()`, and every one calls `dispatchSteeringFrame()` first.
  - `LwsSensor::onFrame(buf, len, millis())` runs in the loop. `LwsSensor` is not thread-safe
    and does not need to be.
- **Timing.**
  - LWS `0x305` arrives every 10 ms and has RXB1 to itself: one frame of slack before RX1OVR.
  - `STEER_LWS_TIMEOUT_MS` = 100 ms turns a gap into a steering-sensor fault.
  - An extended DLC-8 frame lasts 262–320 µs, a standard DLC-6 frame 190–230 µs.
- **Tasks on the chip (Arduino-ESP32 3.x).**
  - `loopTask`: priority 1, core 1.
  - AsyncTCP `async_tcp`: priority 10, **no core affinity** (`CONFIG_ASYNC_TCP_RUNNING_CORE -1`).
    It runs the web handlers, including the OTA `Update.write()`.
  - WiFi, lwIP, esp_timer and IPC: priority 18–24, mostly on core 0.
  - The tick is 1 kHz.
- **Consumer of this change.** `move-aztec-vesc-to-can` (D6 there) needs back-to-back 3-frame
  bursts received, the RX overflow count at 0, and transmits that are safe against the reader.

## Goals / Non-Goals
- Goals:
  - No LWS frame lost to main-loop or web latency.
  - 3-frame bursts received.
  - Consumers unchanged in thread model.
  - Every MCP2515 access serialised with bounded waits.
  - Graceful degradation when INT is dead.
- Non-Goals:
  - Moving decoding into the task.
  - Changing the filters.
  - Surviving flash-write stalls (OTA/NVS). See "Risks".
  - TWAI and DroneCAN.

## Decisions

### D1 — The ISR does no SPI
- `attachInterrupt(PIN_CAN_INT, canIntIsr, FALLING)`, with GPIO38 as `INPUT_PULLUP`. With a
  broken trace the pin then idles HIGH and gives no spurious edges.
- `static void IRAM_ATTR canIntIsr()` does only two things:
  - `isrCount_++` (a `volatile uint32_t` written only here);
  - `xSemaphoreGiveFromISR(rxSem_, &woken); portYIELD_FROM_ISR(woken);`.
- `rxSem_` is a binary semaphore, so several edges before the task runs coalesce into one wake.
  That is correct, because the task drains until the chip is empty.
- `attachInterrupt()` is called from `begin()` in `setup()` on core 1, so the GPIO ISR is
  allocated on core 1, the same core as the task.
- After attaching, `begin()` gives the semaphore once by hand. A frame already latched (INT
  already LOW, so no edge will come) is then read at once.
- Alternative rejected: reading the frame in the ISR. SPI from an ISR with the Arduino `SPI`
  object is not allowed (its internal lock, flash-resident code), and it would race the
  library's state.

### D2 — Reader task `can_rx`
- Created in `begin()`, after a successful init and the filter setup, with
  `xTaskCreatePinnedToCore(canRxTask, "can_rx", CAN_RX_TASK_STACK, this, CAN_RX_TASK_PRIORITY,
  &rxTask_, CAN_RX_TASK_CORE)`. It is never deleted. If `begin()` fails, no task, ISR, queue or
  mutex is created, and the LWS reports `can_down` as today.
- **Priority 12** (`CAN_RX_TASK_PRIORITY`):
  - Above `async_tcp` (10), because the web/OTA handlers that starve the loop today run there.
    "loop + 1" (2) would still be pre-empted by them, so it would not fix the problem this
    change exists for.
  - Below lwIP (18), esp_timer (22), WiFi (23) and IPC (24).
  - Each wake is short (≤ 16 frames × ≈ 40 µs of SPI ≈ 0.7 ms) and always ends in a blocking
    wait, so the high priority cannot starve anything. The core-1 idle task still runs.
- **Core 1** (`CAN_RX_TASK_CORE`, = `ARDUINO_RUNNING_CORE`):
  - The same core as the loop and the ISR. The task and the loop never run truly in parallel,
    and the only concurrency is preemption, which the mutex and the queue cover.
  - Core 0 carries the WiFi stack at priorities 18–23, which would delay the task during the
    very web traffic this is meant to survive.
- **Stack 4096 bytes** (`CAN_RX_TASK_STACK`):
  - The task's call depth is `mcp_can` → `SPIClass` plus a 20-byte local frame.
  - It never logs (no `printf`, no `Debug`), so the usual 2–3 KB formatting peak does not
    apply. 4 KB leaves more than 2× margin.
  - The real high-water mark is printed in the `[CAN] rx:` line
    (`uxTaskGetStackHighWaterMark`), so it can be cut later on evidence.
- **Loop:**
  ```
  for (;;) {
      xSemaphoreTake(rxSem_, pdMS_TO_TICKS(CAN_RX_WAIT_MS));     // 2 ms: wake or poll
      if (reinitActive_) continue;                               // D4: chip being reset
      for (n = 0; n < CAN_RX_DRAIN_CAP; n++) {                   // 16 per wake
          if (!lockSpi(CAN_SPI_LOCK_TIMEOUT_MS)) { spiLockTimeouts_++; break; }
          ok = (mcp_can_->checkReceive() == CAN_MSGAVAIL)
               && (mcp_can_->readMsgBuf(&f.id, &f.len, f.data) == CAN_OK);
          unlockSpi();
          if (!ok) break;
          f.tMs = millis();
          rxTaskFrames_++;
          if (xQueueSend(rxQueue_, &f, 0) != pdTRUE) queueDrops_++;   // never blocks
      }
      if (n == CAN_RX_DRAIN_CAP) capHits_++;     // pending frames: next wake or the 2 ms poll
  }
  ```
- **The authority for "a frame is pending" is `checkReceive()`** (one READ STATUS), not the pin
  level. That keeps the drain correct with a dead INT line. The pin is not read.
- **Lock per frame**, not per wake. The loop's transmit and EFLG reads can interleave between
  frames, and no hold exceeds one frame read (≈ 40 µs).
- **The 2 ms timeout** is the polling fallback, and it also covers a lost edge: INT held LOW
  through a cap hit or a reinit never makes a new FALLING edge. It costs one READ STATUS
  (≈ 10 µs) every 2 ms while idle, about 0.5 % of one core.
- **The cap of 16 per wake** bounds the time the task holds the CPU. With today's filters the
  accepted traffic is ≈ 100 fps (≤ 130 fps with the VESC change), so the cap is never reached in
  normal operation. A cap hit is counted.

### D3 — Queue to the main loop; consumers unchanged
- `rxQueue_ = xQueueCreate(CAN_RX_QUEUE_DEPTH /*32*/, sizeof(RxFrame))`, with
  `struct RxFrame { uint32_t id; uint8_t len; uint8_t data[8]; uint32_t tMs; }` (20 bytes,
  640 bytes in all).
  - `id` keeps the `readMsgBuf()` encoding: bit 31 = extended, bit 30 = RTR. The existing check
    `canId == STEER_LWS_CAN_ID` and the VESC change's ext dispatch work unchanged.
- **Depth 32:**
  - That is ≥ 250 ms of accepted traffic at ≤ 130 fps, so the queue can ride out a loop stall
    2.5× longer than `STEER_LWS_TIMEOUT_MS`.
  - Beyond that, the steering is already in TIMEOUT, so a deeper queue would only deliver stale
    angles.
  - 640 bytes of RAM.
- **Full queue: drop the newest** (`xQueueSend(…, 0)`), and count it in `can_rx_queue_drops`.
  The task never blocks on the queue.
- **`update()`** calls `pumpRxQueue()` **first, on every call and in every mode** (NORMAL IDLE /
  WAITING_RESPONSE, PROBING, LWS zero-set). It runs before the state machine.
  - It takes at most `CAN_RX_QUEUE_DEPTH` frames with `xQueueReceive(…, 0)` and hands each to
    `routeRxFrame()`, the **single consumer**:
    1. `dispatchSteeringFrame()` → `LwsSensor::onFrame(buf, len, millis())` (as today). The VESC
       change renames it `dispatchSharedFrame()` and adds the ext dispatch here.
    2. A frame in `0x7E8`–`0x7EF` while a NORMAL request is in flight
       (`state_ == WAITING_RESPONSE`): if it is `0x41` + the expected PID, it is copied into
       `responseData_` / `responseLen_` and `responseReady_` is set.
    3. The same while a probe request is in flight: the existing Mode 01 / Mode 03 match goes
       into `probeRxBuf_` / `probeRxLen_` / `probeRxReady_`.
    4. Anything else → `logUnmatchedFrame()` (rate-limited, as today).
  - `tryReceiveResponse()` / `tryReceiveProbeResponse()` become "consume the latched reply".
    Sending a new request clears the latch, so a late reply to an old request is never taken
    for the new one. The OBD-II and probe state machines are otherwise unchanged.
- **`LwsSensor::onFrame()` keeps `millis()` at dispatch, not `RxFrame::tMs`.**
  - `LwsSensor::rollWindow()` assumes a non-decreasing clock across `onFrame()` and
    `evaluate()`. A queued frame received before the previous `evaluate()` would step it
    backwards and corrupt `fps`.
  - The age is at most one loop iteration, the same as today. `tMs` is used only for the
    "max queue age" diagnostic.
- **Thread model.** `LwsSensor`, `VehicleData`, the probe results and the zero-set state are
  still touched only by the loop. The task touches only the chip (under the mutex), the queue
  and its own counters (single writer, 32-bit aligned, read-only from the loop).

### D4 — One SPI mutex, no nested locks
- `spiMutex_ = xSemaphoreCreateMutex()` (priority inheritance: the loop holding it is boosted
  while the task waits).
- `lockSpi(ms)` / `unlockSpi()` are the only accessors. An RAII `SpiGuard` is used in
  `CANController.cpp`.
- **Covered**:
  - every `mcp_can_->…` call: `begin`, `init_Mask`/`init_Filt`, `setMode`, `checkReceive`,
    `readMsgBuf`, `sendMsgBuf`, `getError`, `errorCountTX/RX`, `abortTX`;
  - every raw `readRegister()` / `modifyRegister()` and the new `queueTx()` SPI writes.
- **Lock order.** There is exactly one application lock. `SPIClass::beginTransaction()` takes
  Arduino's internal SPI bus lock *inside* it, per transaction, and releases it before
  `endTransaction()` returns. Order: `spiMutex_` → (SPIClass internal). Nothing is taken while
  holding the SPIClass lock, and `spiMutex_` is never taken twice.
- **Locks are taken at the outermost entry points only:**
  - `begin()` (the whole init + filters + mode + CANINTE readback);
  - `recoverFromBusOff()` (whole);
  - `sendOBDRequest()` / `sendMode03Request()` (through `queueTx()`);
  - `sendLwsCfg()`;
  - the send-failure branch (`getError`, `errorCount*`, `abortTX`, the ABAT clear) as one hold;
  - the WAITING_RESPONSE timeout `getError()`;
  - `checkRxOverflow()`;
  - `logHealth()`;
  - the reader task.
- `readRegister()` / `modifyRegister()` / `applyRxFilters()` / `queueTx()` are documented as
  "caller holds the SPI lock". In debug builds they check
  `xSemaphoreGetMutexHolder(spiMutex_) == xTaskGetCurrentTaskHandle()` and count violations.
- **No logging while the lock is held.** `Serial` on USB-CDC can block. Values are read under
  the lock and printed after release. Today's `sendLwsCfg()`, `recoverFromBusOff()` and
  `begin()` print between SPI calls, and are restructured.
- **Reinit tolerance.**
  - `recoverFromBusOff()` sets `reinitActive_ = true` (`volatile bool`), takes the lock, and
    runs `begin` + `applyRxFilters` + `setMode` + the CANINTE assert.
  - It then releases the lock, clears the flag, gives `rxSem_` once (to catch frames latched
    during the reset) and logs.
  - The task checks the flag and skips without counting. If it is already waiting on the lock,
    it times out after 10 ms, counts one lock timeout and waits again.
  - Frames already in the queue are valid and are kept.

### D5 — Interrupt sources: CANINTE = RX0IE | RX1IE only
- No ERRIE, MERRE, WAKIE or TXnIE. An error or TX flag that nobody clears in the task would
  hold the level INT LOW. A FALLING-edge ISR would then go silent, and an ISR on the level would
  storm.
- `RXnIF` is cleared by `readMsg()` (BIT MODIFY), as today, so INT returns HIGH once both
  buffers are empty.
- **How it is set with the library.**
  - `mcp_can` has no `setINTE()`. Its `mcp2515_init()` (via `begin()`) already writes
    `CANINTE = 0x03`.
  - `setMode()` touches only `WAKIE`, and only when it wakes a sleeping chip, which this firmware
    never puts to sleep.
  - `CANController` therefore **re-asserts and verifies** CANINTE after every `begin()` and
    every re-init, under the lock:
    - `modifyRegister(MCP_CANINTE, 0xFF, MCP_RX0IF | MCP_RX1IF)`;
    - `readRegister(MCP_CANINTE)`;
    - the readback is logged after release, and a mismatch is logged as an error.
  - The value is `CAN_CANINTE_VALUE` in `Constants.h`. **No change to the library.**
  - `platformio.ini` pins `mcp_can` to `#34807287506afc6b4aef7c577392c1c4e32d1b45`. An
    unpinned `git` dependency could change `mcp2515_init()` or `readMsg()` under the design.
- **Error flags.** EFLG (RX0OVR / RX1OVR / TXBO), TEC and REC stay main-loop work:
  - `checkRxOverflow()` now runs on every `update()` in every mode, throttled to
    `CAN_EFLG_CHECK_MS` (100 ms; today it runs every IDLE pass and never while
    waiting or probing).
  - It adds a cumulative `rxOverflowCount_` for diagnostics and for the VESC change's bench check.
  - `logHealth()` is unchanged (5 s, only while `dataValid` is false), plus the lock.

### D6 — Fallback and diagnostics
- **Fallback is structural.** The 2 ms semaphore timeout (D2) means a dead, stuck-LOW or
  never-connected INT line still drains the chip every ≤ 2 ms from a priority-12 task. That is
  still far better than today, and there is no mode switch.
- **Detector** (pure, `include/CanRxMonitor.h`, `class CanIntMonitor`):
  - `update(nowMs, isrCount, frameCount)` is called from `update()`. It looks at fixed
    windows of `CAN_INT_CHECK_MS` (5000 ms):
    - frames > 0 and ISR hits == 0 in the window → state `INACTIVE`;
    - ISR hits > 0 → `ACTIVE`;
    - no frames → the state is kept (no evidence, e.g. the bus is silent).
  - The start state is `ACTIVE`, i.e. unproven. Transitions are reported once (`changed()`).
  - `can_int_ok` = state != `INACTIVE`.
- **Logs** (`Debug::printfFeature(DebugFeature::CAN, …)`):
  - `[CAN] INT inactive, polling fallback` once per transition to INACTIVE, and
    `[CAN] INT active` once per transition back.
  - `[CAN] rx: isr=%lu frames=%lu drops=%lu hwm=%u/%u cap=%lu lockto=%lu ovr=%lu age=%lums
    stack=%u int=%s`, every `CAN_HEALTH_LOG_INTERVAL` (5 s). Unlike `logHealth()`, it prints
    regardless of `dataValid`, because RX health matters most while the engine is running.
  - Queue drops log at most once per 5 s with the count since the last line (inside the rx
    line). The task itself never logs.
- **Queue high-water mark.** `CAN_RX_QUEUE_DEPTH - uxQueueSpacesAvailable()` is sampled at the
  start of `pumpRxQueue()`, and its maximum is kept. Max queue age = `millis() - tMs` of the
  oldest frame taken per pump, maximum kept. Both reset at each `[CAN] rx:` line.
- **Bench test of the fallback without cutting the trace.** GPIO38 is a PCB trace on
  `Control_v0`. The build flag `-DCAN_RX_TEST_NO_INT` (not in `Constants.h`, never in release)
  makes `begin()` skip `attachInterrupt()`. Everything else is identical.

### D7 — Bounded blocking and the non-blocking transmit
- **The problem with `sendMsgBuf()`.** It holds the bus (now: the mutex) until the frame has
  been ACKed: ≈ 250 µs on an idle bus, longer behind higher-priority traffic, and up to
  2 × 2.5 ms with no ACK. During that time the task cannot read.
  - With LWS alone this is harmless: one frame per 10 ms.
  - A VESC 3-frame burst arriving during an OBD-II or VESC keep-alive send would lose its third
    frame. VESC frames (SID 0) win arbitration, so our send waits behind exactly that burst.
  - So "TX holds the mutex for one SPI transaction" is not achievable with `sendMsgBuf()`.
- **Decision: `queueTx(id, ext, len, data, &txb)`.** This is new CANController code, using the
  raw SPI path and `include/Mcp2515Tx.h`.
  1. READ STATUS (`0xA0`) → the TXREQ bits of TXB0..2. `mcp2515PickFreeTx()` picks the first
     free buffer. If none is free → return `false` ("all TX buffers busy"). This is the
     equivalent of today's `CAN_GETTXBFTIMEOUT` and feeds the same send-failure /
     `abortTX()` / ABAT-clear path.
  2. One WRITE (`0x02`) burst from `TXBnSIDH`: the 5 header bytes from `mcp2515EncodeTxHeader()`
     (standard or extended, DLC) plus the data.
  3. RTS (`0x81` / `0x82` / `0x84`).
  - All of it runs under one lock hold of three short transactions (≈ 20 µs), with no
    busy-wait. Success means "queued", not "ACKed".
- **Who uses it.**
  - OBD-II Mode 01 and Mode 03 requests. A request that is never ACKed shows up as the response
    timeout (already handled), and after 3 stuck sends as "all TX buffers busy" → abort +
    ABAT clear, the same chain as today.
  - The VESC change's `sendVescFrame()` is meant to use the same helper.
- **The LWS zero-set keeps `sendMsgBuf()`.** It is operator-triggered and rare, and its
  `tx_failed` abort needs the confirmed transmit. The ≤ 5 ms hold is accepted there.
- **Bounds on every hold:**

  | Holder | Typical | Worst |
  |--------|---------|-------|
  | task frame read | ≈ 40 µs | ≈ 60 µs |
  | `queueTx()` | ≈ 20 µs | ≈ 20 µs |
  | EFLG / TEC / REC read | ≈ 10 µs | ≈ 10 µs |
  | send-failure branch with `abortTX` | < 50 µs | < 50 µs |
  | zero-set `sendMsgBuf()` | ≈ 250 µs | 5 ms |
  | re-init | — | the library's `requestNewMode()` bound of 200 ms, twice, + 5 ms reset ≈ 0.4 s |

  The re-init already blocks the loop today, and the task skips during it (D4).
- **Never infinite.**
  - Every `xSemaphoreTake(spiMutex_)` uses `CAN_SPI_LOCK_TIMEOUT_MS` (10 ms). That is more
    than every non-reinit hold above, so a timeout means something is wrong.
  - On a timeout, the loop side returns "failed" for that operation, counts `spiLockTimeouts_`,
    and logs at most once per `CAN_HEALTH_LOG_INTERVAL`. A failed OBD-II send is rescheduled as
    today, but it does **not** increment `consecutiveSendFailures_`: `abortTX()` would need the
    same lock.
  - The task counts and goes back to waiting.
  - Queue operations never block (timeout 0 on both sides).
- **Watchdog.** The task blocks at least every 2 ms and its CPU per wake is < 1 ms, so idle1
  and the loop are never starved. Nothing new blocks the loop beyond the 10 ms lock bound.

### D8 — Filters unchanged
- RXB0: mask `0x7F8`, filters 0–1 `0x7E8`. RXB1: mask `0x7FF`, filters 2–5 `0x305`. BUKT on.
- `applyRxFilters()` is only moved under the lock (called from `begin()` /
  `recoverFromBusOff()`, which hold it).
- The VESC filter (filter 1 → ext `0x00000000`) is `move-aztec-vesc-to-can`.

### D9 — `drainRxBuffers()` removed, no compile-time switch
- The task owns the hardware RX. Keeping `drainRxBuffers()` or the inline drains would create a
  second reader racing the task, under the same lock but with no ordering.
- **Removed:** `drainRxBuffers()`, and the `checkReceive()`/`readMsgBuf()` loops in
  `tryReceiveResponse()` and `tryReceiveProbeResponse()`.
- **Kept:** the queue drain in the loop (D3).
- **No `CAN_RX_INTERRUPT` switch.**
  - The runtime fallback (D2 timeout poll) already covers a dead INT line, which is the only
    reason to want polling.
  - A switch would keep two receive architectures alive, each needing its own bench proof.
  - `move-aztec-vesc-to-can` cannot work in the polled mode anyway.
  - Rollback = flash 1.5.1.
  - The diff stays reviewable because the consumer side keeps its functions and only their
    data source changes.

## Risks / Trade-offs
- **Flash-write stalls (OTA, NVS).**
  - While SPI flash is erased or written, the cache is disabled on both cores. No
    flash-resident task runs, including `can_rx` and the `mcp_can` / SPI code it calls. Only
    IRAM ISRs run.
  - Arduino `Update.write()` erases a 4 KB sector at each sector boundary, typically 30–50 ms,
    and longer on some flash parts. RXB1 holds one LWS frame, so each erase can still lose 3–5
    LWS frames. A 1.5 MB OTA can therefore add a few hundred to `steer_lws_miss`.
  - This is not a regression (polling lost them too), and frames lost to erases do not reach
    `STEER_LWS_TIMEOUT_MS` unless a single erase exceeds 100 ms.
  - The bench metric is therefore split:
    - zero misses with the web UI open and live;
    - during OTA, misses allowed but no queue drops, and no LWS `timeout` unless an erase longer
      than 100 ms is logged.
  - Mitigations out of scope: flash auto-suspend, or an IRAM-resident reader.
- **More concurrency.**
  - A missed lock is a data race on the chip. Mitigations:
    - every SPI access goes through a small set of entry points;
    - the debug-build holder check (D4);
    - a grep task: `mcp_can_->` appears only inside locked functions.
  - Priority inheritance plus bounded holds avoid inversion stalls.
- **Priority 12 above AsyncTCP.** A bug that makes the task spin, such as a stuck
  `checkReceive()` that always returns a frame, would starve the web server. The per-wake cap
  plus the blocking wait bound it to ≈ 0.7 ms out of every 2 ms. `can_rx_task_frames` and the
  cap counter make it visible.
- **`queueTx()` is custom SPI code next to the library.** The header encoding is host-tested,
  and the instruction set (READ STATUS, WRITE, RTS) is the documented MCP2515 one. The zero-set
  stays on the library path.
- **Late OBD-II replies.** Every frame is now consumed, including replies that arrive after the
  200 ms timeout. They are logged as unmatched (as the IDLE drain does today) and are never
  matched to a newer request, because the latch is cleared on send.

## Migration Plan
1. Flash 1.6.0 and the LittleFS image (new CAN tile line). No calibration and no wiring change.
2. Check the boot log:
   - `[CAN] CANINTE=0x03`;
   - the `[CAN] rx:` line shows `isr` ≈ `frames`, and `int=ok`.
3. Rollback = flash 1.5.1 and its LittleFS image. Nothing is stored in NVS.

## Open Questions
- The bench must confirm that GPIO38 really toggles with MCP2515 `INT` on this board revision
  (`isr` > 0 in the rx line). If it does not, the change still works on the 2 ms fallback and
  logs `INT inactive`. The board then needs a look before `move-aztec-vesc-to-can`.
