# Change: Add fuel tank level reading via OBD-II PID 0x2F

## Why
The vehicle's fuel tank level is useful for monitoring and telemetry. OBD-II PID 0x2F (Fuel Tank Level Input) is a standard Mode 01 PID that returns fuel level as a percentage (0–100%).

## What Changes
- Add `fuelLevel` field (uint8_t, 0–100%) to `VehicleData` struct
- Add PID 0x2F constant and a 6th entry to `pidTable_` (polled at 1000ms, same as temperatures)
- Add parsing case in `parseAndStore()` for PID 0x2F: formula is `A * 100 / 255`
- Bump `PID_COUNT` from 5 to 6
- Include fuel level in WebSocket telemetry broadcast

## Impact
- Affected specs: `can-controller` (OBD-II data reading)
- Affected code: `CANController.h` (VehicleData, PID_COUNT, pidTable_), `CANController.cpp` (parseAndStore), `TelemetryManager.cpp` (telemetry broadcast)
