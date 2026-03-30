## 1. Implementation
- [x] 1.1 Add `uint8_t fuelLevel` to `VehicleData` struct in `CANController.h`
- [x] 1.2 Add `PID_FUEL_LEVEL = 0x2F` constant in `CANController.h`
- [x] 1.3 Bump `PID_COUNT` from 5 to 6
- [x] 1.4 Add 6th entry `{PID_FUEL_LEVEL, CAN_POLL_INTERVAL_TEMP, 0, 0}` to `pidTable_` in constructor
- [x] 1.5 Add `case PID_FUEL_LEVEL` in `parseAndStore()`: `fuelLevel = (data[0] * 100) / 255`
- [x] 1.6 Include `fuelLevel` in telemetry broadcast (`TelemetryManager.cpp`, `WebPortal.h`, `WebPortal.cpp`)

## 2. Validation
- [x] 2.1 Build compiles without errors
- [ ] 2.2 Fuel level appears in WebSocket telemetry JSON
- [ ] 2.3 Verify with a vehicle that supports PID 0x2F
