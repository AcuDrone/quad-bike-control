# Change: Refactor CAN OBD-II Polling to Non-Blocking State Machine

## Why
`receiveOBDResponse()` blocks up to 1000ms per PID in a spin-wait loop with `delay(10)`. With 5 PIDs polled, the main loop can stall for up to 5 seconds when the ECU is unresponsive — blocking SBUS input (14ms frames), web commands, and telemetry updates.

## What Changes
- Replace blocking `receiveOBDResponse()` with non-blocking `tryReceiveResponse()` (single `checkReceive()` call per `update()`)
- Rewrite `update()` as a two-state machine (IDLE → send request, WAITING_RESPONSE → check once and return)
- Replace 5 individual `read*()` methods with a table-driven PID scheduler and `parseAndStore()` switch
- Replace `lastRPMPoll_`/`lastTempPoll_`/`retryCount_` with per-PID scheduling entries (interval, nextPollTime, retryCount)
- Reduce `CAN_RESPONSE_TIMEOUT` from 1000ms to 200ms (healthy ECU responds in ~50ms)

## Impact
- Affected specs: `can-controller` (OBD-II data reading, error handling)
- Affected code: `CANController.h`, `CANController.cpp`, `Constants.h`
- **No changes** to public API — `begin()`, `update()`, `getVehicleData()`, `isConnected()`, `getStatusString()` signatures unchanged
- **No changes** to consumers: VehicleController, TransmissionController, RelayController, TelemetryManager
