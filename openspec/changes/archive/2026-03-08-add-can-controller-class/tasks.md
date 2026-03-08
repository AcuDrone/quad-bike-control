# Implementation Tasks: CAN Controller

## Phase 1: CANController Class Foundation

- [ ] 1.1: Add MCP_CAN library to platformio.ini (`autowp/mcp_can@^1.5.1`)
- [ ] 1.2: Add CAN configuration constants to Constants.h (pins, speeds, timeouts)
- [ ] 1.3: Create CANController.h with class declaration and VehicleData struct
- [ ] 1.4: Implement CANController::begin() - MCP2515 initialization on SPI pins
- [ ] 1.5: Implement basic SPI communication test (loopback mode)
- [ ] 1.6: Add CANController instance to VehicleController
- [ ] 1.7: Call canController.begin() in main.cpp setup()
- [ ] 1.8: Verify MCP2515 hardware detection on Serial output

## Phase 2: OBD-II Protocol Implementation

- [ ] 2.1: Implement sendOBDRequest() - Mode 01 request formatting
- [ ] 2.2: Implement receiveOBDResponse() - CAN frame parsing
- [ ] 2.3: Implement readEngineRPM() - PID 0x0C with formula ((A*256)+B)/4
- [ ] 2.4: Implement readVehicleSpeed() - PID 0x0D (direct value)
- [ ] 2.5: Implement readCoolantTemp() - PID 0x05 with formula (A - 40)
- [ ] 2.6: Implement readOilTemp() - PID 0x5C with formula (A - 40)
- [ ] 2.7: Implement readThrottlePosition() - PID 0x11 with formula (A * 100/255)
- [ ] 2.8: Add round-robin polling logic in update() (100ms RPM/speed, 1000ms temps)
- [ ] 2.9: Test OBD-II communication with vehicle ECU via OBD-II port

## Phase 3: Error Handling and Robustness

- [ ] 3.1: Implement response timeout detection (1000ms)
- [ ] 3.2: Set dataValid flag based on communication success
- [ ] 3.3: Implement stale data detection (5000ms threshold)
- [ ] 3.4: Add MCP2515 hardware error detection and recovery
- [ ] 3.5: Implement retry logic (up to 3 attempts) for failed requests
- [ ] 3.6: Add Serial logging for all error conditions
- [ ] 3.7: Test timeout behavior (disconnect CAN, verify graceful degradation)
- [ ] 3.8: Test recovery (reconnect CAN, verify automatic recovery)

## Phase 4: Transmission Safety Integration

- [ ] 4.1: Add VehicleData member to TransmissionController.h
- [ ] 4.2: Implement setVehicleData() method in TransmissionController
- [ ] 4.3: Add canChangeGear() safety check method with speed interlock logic
- [ ] 4.4: Modify setGear() to call canChangeGear() before allowing changes
- [ ] 4.5: Add TRANS_SPEED_INTERLOCK_THRESHOLD constant (5 km/h)
- [ ] 4.6: Add TRANS_CAN_TIMEOUT constant (5000ms)
- [ ] 4.7: Implement timeout fallback (allow gear change if CAN fails > 5s)
- [ ] 4.8: Allow NEUTRAL changes regardless of speed (safety override)
- [ ] 4.9: Pass VehicleData from VehicleController to TransmissionController
- [ ] 4.10: Test speed interlock (attempt gear change while moving, verify blocked)
- [ ] 4.11: Test timeout fallback (disconnect CAN, wait 5s, verify allowed)

## Phase 5: Throttle Boost During Gear Changes

- [ ] 5.1: Add needsThrottleBoost() method to TransmissionController
- [ ] 5.2: Add throttleBoostActive flag to track boost state
- [ ] 5.3: Set throttleBoostActive when gear change is in progress
- [ ] 5.4: Add TRANS_THROTTLE_BOOST_PERCENT constant (20%)
- [ ] 5.5: Add TRANS_THROTTLE_BOOST_DURATION constant (500ms)
- [ ] 5.6: Implement applyThrottleBoost() in VehicleController
- [ ] 5.7: Call applyThrottleBoost() when transmission.needsThrottleBoost() is true
- [ ] 5.8: Disable boost if brake is applied (> 10%)
- [ ] 5.9: Release boost after gear change completes or 500ms timeout
- [ ] 5.10: Ensure SBUS throttle commands override boost
- [ ] 5.11: Test throttle boost activation during gear change
- [ ] 5.12: Test boost disable on brake application

## Phase 6: Telemetry Integration

- [ ] 6.1: Add VehicleData member to TelemetryManager.h
- [ ] 6.2: Implement setVehicleData() method in TelemetryManager
- [ ] 6.3: Pass VehicleData from VehicleController to TelemetryManager
- [ ] 6.4: Add vehicle data fields to Telemetry JSON structure
- [ ] 6.5: Serialize engineRPM, vehicleSpeed, coolantTemp to JSON
- [ ] 6.6: Add canStatus field ("connected"/"disconnected")
- [ ] 6.7: Add canDataAge field (time since last update)
- [ ] 6.8: Omit vehicle data fields when CAN is disconnected
- [ ] 6.9: Update web portal UI to display vehicle data (RPM, speed, temp)
- [ ] 6.10: Add CAN status indicator to web UI (green/red)
- [ ] 6.11: Test telemetry data accuracy (compare with dashboard)

## Phase 7: Status and Diagnostics

- [ ] 7.1: Implement isConnected() method (check MCP2515 communication)
- [ ] 7.2: Implement getStatusString() method (human-readable status)
- [ ] 7.3: Log CAN initialization status on startup
- [ ] 7.4: Log connection/disconnection events
- [ ] 7.5: Add CAN status to main loop diagnostic output (every 5s)
- [ ] 7.6: Implement getVehicleData() accessor method
- [ ] 7.7: Test status reporting (verify correct states)

## Phase 8: Hardware Testing and Validation

- [ ] 8.1: Connect MCP2515 module to ESP32-C6 SPI pins
- [ ] 8.2: Verify 3.3V power and ground connections
- [ ] 8.3: Connect MCP2515 to vehicle OBD-II port (CANH/CANL)
- [ ] 8.4: Verify CAN termination (120Ω resistor if needed)
- [ ] 8.5: Test with engine off (may need ignition ACC for ECU power)
- [ ] 8.6: Test with engine idling (RPM ~800-1000)
- [ ] 8.7: Test with engine running at various RPMs
- [ ] 8.8: Test with vehicle moving (speed sensor validation)
- [ ] 8.9: Measure CAN update rates (verify 10 Hz for RPM/speed)
- [ ] 8.10: Verify temperature readings accuracy (compare to dashboard)
- [ ] 8.11: Test speed interlock with real vehicle movement

## Phase 9: Edge Case Testing

- [ ] 9.1: Test CAN init failure (no MCP2515 hardware)
- [ ] 9.2: Test OBD-II port disconnection during operation
- [ ] 9.3: Test vehicle ECU not responding (ignition off)
- [ ] 9.4: Test gear change request exactly at speed threshold (5 km/h)
- [ ] 9.5: Test multiple rapid gear change requests
- [ ] 9.6: Test throttle boost with SBUS throttle override
- [ ] 9.7: Test system behavior with invalid OBD-II responses
- [ ] 9.8: Test recovery after power cycle
- [ ] 9.9: Load test (verify no impact on control loop timing)
- [ ] 9.10: Stress test (run for 1 hour continuous operation)

## Phase 10: Documentation and Cleanup

- [ ] 10.1: Document CAN wiring in GPIO_PINOUT.md
- [ ] 10.2: Add MCP2515 connection diagram to docs
- [ ] 10.3: Document OBD-II PIDs used and their meanings
- [ ] 10.4: Add troubleshooting guide for CAN issues
- [ ] 10.5: Update README with CAN controller information
- [ ] 10.6: Add comments to all public methods
- [ ] 10.7: Clean up debug Serial output (use appropriate log levels)
- [ ] 10.8: Archive proposal with openspec archive command

---

## Dependencies
- **1.x → 2.x**: MCP2515 must initialize before OBD-II requests
- **2.x → 3.x**: OBD-II protocol must work before error handling
- **2.x, 3.x → 4.x**: CAN data must be reliable for transmission safety
- **4.x → 5.x**: Speed interlock must work before throttle boost
- **3.x → 6.x**: Error handling must be complete before telemetry
- **All → 8.x**: Software complete before hardware testing

## Parallel Work Opportunities
- **Phase 4 & 5**: Transmission integration can be done in parallel
- **Phase 6**: Telemetry can be developed alongside Phase 4/5
- **Phase 7**: Diagnostics can be added anytime after Phase 3

## Estimated Effort
- **Phase 1-3**: 4-6 hours (core functionality)
- **Phase 4-5**: 3-4 hours (transmission integration)
- **Phase 6**: 2-3 hours (telemetry)
- **Phase 7**: 1-2 hours (diagnostics)
- **Phase 8-9**: 4-6 hours (hardware testing)
- **Phase 10**: 1-2 hours (documentation)
- **Total**: 15-23 hours
