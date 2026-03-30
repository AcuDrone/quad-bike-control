## Context
The CAN controller polls 5 OBD-II PIDs using a blocking send-then-wait pattern. Each `receiveOBDResponse()` call spin-waits up to 1000ms. This is the worst-case bottleneck in the main loop, causing SBUS frame loss and web command delays.

## Goals / Non-Goals
- Goals: Make `update()` return in microseconds (not milliseconds). Maintain same data freshness and polling intervals.
- Non-Goals: Adding FreeRTOS tasks, interrupt-driven CAN, or changing the MCP_CAN library.

## Decisions

### Two-State Machine
```
IDLE → select most overdue PID, sendOBDRequest() → WAITING_RESPONSE
WAITING_RESPONSE → checkReceive() once:
  - response found → parseAndStore(), reset timer → IDLE
  - timeout elapsed → increment retry, reset timer → IDLE
  - nothing yet → return immediately (try next loop iteration)
```

Only two states needed. No intermediate states for retries — retries are just re-entering IDLE with the same PID becoming overdue again.

### PID Scheduling Table
Replace separate `lastRPMPoll_`/`lastTempPoll_` timers with a unified table:

```cpp
struct PIDEntry {
    uint8_t pid;
    uint32_t interval;       // ms between polls
    uint32_t nextPollTime;   // millis() when next poll is due
    uint8_t retryCount;
};

PIDEntry pidTable_[5] = {
    {0x0C, 100,  0, 0},  // RPM
    {0x0D, 100,  0, 0},  // Speed
    {0x05, 1000, 0, 0},  // Coolant temp
    {0x5C, 1000, 0, 0},  // Oil temp
    {0x11, 1000, 0, 0},  // Throttle position
};
```

`selectNextPID()` picks the PID whose `nextPollTime` has passed and is most overdue. High-frequency PIDs (RPM/speed) naturally get priority.

### Consolidated Parsing
Replace 5 `readEngineRPM()`, `readVehicleSpeed()`, etc. methods with one `parseAndStore()` switch on PID code.

### Response Timeout
Reduce `CAN_RESPONSE_TIMEOUT` from 1000ms to 200ms. A healthy ECU responds in ~50ms. With 200ms timeout, even all 5 PIDs timing out cycles through in ~1s wall-clock without ever blocking.

## Risks / Trade-offs

### MCP2515 RX Buffer Overflow
The MCP2515 has only 2 receive buffers. Reading one message per `update()` call may overflow if CAN traffic is high.
→ Mitigation: In `WAITING_RESPONSE`, drain up to 5 messages per call. Each `checkReceive()` + `readMsgBuf()` takes ~100μs via SPI — still non-blocking.

### Data Freshness Lag
Currently all PIDs are fresh when `update()` returns. With state machine, only one PID resolves per call. At 100Hz main loop, all 5 PIDs cycle in ~50ms best-case.
→ Acceptable: consumers already handle `dataValid == false`. RPM/speed at 100ms interval still match current polling intent.

### Unsolicited CAN Frames
Non-matching CAN messages are discarded — same behavior as current code, just without blocking.

## Open Questions
- None
