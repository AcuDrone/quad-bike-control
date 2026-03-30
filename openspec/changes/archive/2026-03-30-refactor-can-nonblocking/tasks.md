## 1. Implementation
- [x] 1.1 Add `OBDState` enum and `PIDEntry` struct to `CANController.h`
- [x] 1.2 Replace `lastRPMPoll_`, `lastTempPoll_`, `retryCount_` with `state_`, `activePIDIndex_`, `requestSentTime_`, `pidTable_[5]`, `responseData_[5]`, `responseLen_`
- [x] 1.3 Remove declarations of `readEngineRPM()`, `readVehicleSpeed()`, `readCoolantTemp()`, `readOilTemp()`, `readThrottlePosition()`, `receiveOBDResponse()`
- [x] 1.4 Add declarations for `tryReceiveResponse()`, `selectNextPID()`, `parseAndStore()`
- [x] 1.5 Initialize `pidTable_` in constructor with correct PIDs and intervals
- [x] 1.6 Rewrite `update()` as IDLE/WAITING_RESPONSE state machine
- [x] 1.7 Implement `tryReceiveResponse()` — single non-blocking `checkReceive()`, drain up to 5 messages
- [x] 1.8 Implement `selectNextPID()` — pick most overdue PID from table
- [x] 1.9 Implement `parseAndStore()` — switch on PID to update VehicleData field
- [x] 1.10 Remove old `read*()` methods and `receiveOBDResponse()`
- [x] 1.11 Reduce `CAN_RESPONSE_TIMEOUT` from 1000ms to 200ms in `Constants.h`

## 2. Validation
- [x] 2.1 Build compiles without errors
- [ ] 2.2 With CAN connected: RPM/speed update at ~10Hz, temps at ~1Hz
- [ ] 2.3 With CAN disconnected: main loop stays responsive, no multi-second stalls
- [ ] 2.4 Debug log shows timeout warnings without blocking
