## 1. Dependencies & configuration
- [x] 1.1 Add the MAVLink 2 C library (`mavlink/c_library_v2`, ArduPilotMega dialect) to
  `platformio.ini` `lib_deps`; remove `bolderflight/Bolder Flight Systems SBUS`
- [x] 1.2 In `Constants.h`: add MAVLink UART config (`PIN_MAVLINK_RX` = old `GPIO_NUM_8`,
  `PIN_MAVLINK_TX` = `GPIO_NUM_15` (confirmed free), `MAVLINK_UART_NUM`,
  `MAVLINK_BAUD_RATE` = 115200). Do NOT use `GPIO_NUM_9` — it is `PIN_TRANS_SERVO`.
- [x] 1.6 Update `GPIO_PINOUT_S3.md`: row 8 → `PIN_MAVLINK_RX` (UART1 RX, non-inverted),
  row 15 → `PIN_MAVLINK_TX` (UART1 TX), and the UART table (115200, MAVLink2)
- [x] 1.3 In `Constants.h`: add `MAVLINK_SERVO_OUTPUT_RATE_HZ` (50), `MAVLINK_CMD_TIMEOUT_MS`
  (500), `MAVLINK_HEARTBEAT_TIMEOUT_MS` (3000), telemetry send rates, and ESP32
  system/component id
- [x] 1.4 Rename `SBUS_*` range/deadband/threshold constants to transport-neutral names
  (retaining values) and replace `SBusChannelConfig` with `ServoChannelConfig`
- [x] 1.5 Rename `InputSource::SBUS` → `InputSource::MAVLINK` and `INPUT_SOURCE_NAME_SBUS`

## 2. MavlinkInterface class
- [x] 2.1 Create `include/MavlinkInterface.h` mirroring the `SBusInput` command API
  (`getSteering/getThrottle/getGear/getBrake/getIgnitionState/getFrontLight`,
  `isSignalValid/getSignalAge/getLinkQuality`) plus `begin()`/`update()`
- [x] 2.2 Implement UART init (non-inverted, 8N1, 115200, RX+TX) and the MAVLink parse loop
  (`mavlink_parse_char`), dispatching by message id
- [x] 2.3 Decode `SERVO_OUTPUT_RAW` into a latest-command frame with timestamp + counter;
  clamp µs to valid range
- [x] 2.4 Handle `HEARTBEAT`: learn autopilot sysid/compid, refresh heartbeat timestamp
- [x] 2.5 On first autopilot heartbeat, send `MAV_CMD_SET_MESSAGE_INTERVAL` requesting
  `SERVO_OUTPUT_RAW` at 50 Hz
- [x] 2.6 Implement command/heartbeat staleness checks and `isSignalValid()`
- [x] 2.7 Port the µs→command mapping math from `SBusInput` to read `ServoChannelConfig`
  servo-output values

## 3. State reporting (return telemetry)
- [x] 3.1 Emit `HEARTBEAT` at 1 Hz (`MAV_TYPE_GROUND_ROVER`, system_status reflects fail-safe)
- [x] 3.2 Emit `EFI_STATUS` at 5 Hz from `CANController::VehicleData` (RPM, coolant temp,
  throttle, health flag from CAN validity). Vehicle speed and oil temp are NOT reported
  (ECU does not provide them); `VFR_HUD` is not emitted.
- [x] 3.3 Emit `STATUSTEXT` on gear / ignition / fail-safe state transitions (rate-limited)
- [x] 3.5 Emit `NAMED_VALUE_FLOAT` `GEAR` at the report rate so the GCS shows gear as a live,
  graphable value (Mission Planner Status tab). Sequence-encoded `[R,N,H,L]=[-1,0,1,2]`;
  **midpoint** of the two gears while the servo is moving (`isGearChangeActive()`), integer when
  settled — multi-step shifts render as an even 0.5 staircase. Value is the controller's
  assumed/commanded gear (sensorless, time-based). Adds `TransmissionController::getFromGear()`
  (+ `fromGear_` tracking) and `VehicleController::getTargetGearString()`/`getFromGearString()`.
- [x] 3.4 Wire reporting into the loop; pass CAN `VehicleData` and current vehicle state in

## 4. Integration
- [x] 4.1 Replace `SBusInput` with `MavlinkInterface` in `main.cpp` (instantiate, `begin()`,
  `update()` in loop)
- [x] 4.2 Update `VehicleController` (`.h`/`.cpp`) to consume `MavlinkInterface`; rename the
  SBUS-specific members (`previousIgnitionState_`, `lastCommandedGear_`, `processMavlinkCommands`)
- [x] 4.3 Update `TelemetryManager` (`.h`/`.cpp`) `determineInputSource()` to MAVLINK > WEB >
  FAILSAFE and take a `MavlinkInterface` reference
- [x] 4.4 Update `WebPortal`/`data/index.html` telemetry to show decoded command channels +
  MAVLink link status instead of raw SBUS channels/quality
- [x] 4.5 Delete `include/SBusInput.h` and `src/SBusInput.cpp`

## 5. Validation
- [x] 5.1 `pio run -e esp32-s3-devkitc-1` builds clean (Flash 30.7%, RAM 16.7%); MAVLink C
  library resolves under default LDF. Source sweep for SBUS references is clean
  (`grep -ri sbus src include data` returns nothing).
- [ ] 5.2 Bench test: drive a MAVLink `SERVO_OUTPUT_RAW` stream (SITL / Mission Planner /
  script) and confirm steering/throttle/gear/ignition/light decode correctly — **hardware/SITL**
- [ ] 5.3 Verify link-loss fail-safe: stop the stream and confirm fail-safe within 500 ms — **hardware/SITL**
- [ ] 5.4 Confirm `HEARTBEAT`/`EFI_STATUS`/`VFR_HUD`/`STATUSTEXT` appear in a GCS (Mission
  Planner / MAVLink Inspector) with correct values from CAN — **hardware/SITL**
- [x] 5.5 Document required ArduPilot config: TELEM2 `SERIAL2_PROTOCOL=MAVLink2`, baud 115200,
  `SERVOn_FUNCTION` mapping (steering=1, throttle=2, transmission=3, ignition=4, light=6) —
  see `MAVLINK_SETUP.md`
- [ ] 5.6 Hardware-in-loop test on the vehicle before merge — **hardware**
- [x] 5.7 `openspec validate replace-sbus-with-mavlink --strict` passes
