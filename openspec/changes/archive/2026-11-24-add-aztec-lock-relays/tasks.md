## 1. Relay map
- [x] 1.1 `include/Constants.h`: `RELAY_SET_WHEEL_LOCK = Relay_4|Relay_5|Relay_6`, `RELAY_SET_RESERVED = Relay_7|Relay_8`, `RELAY_WHEEL_LOCK_FITTED 1`; comments (masks `0x8C` / `0x38`, ⚠ non-1:1 routing), servo 7 and EFI bit0 comments
- [x] 1.2 `src/RelayController.cpp`: aztec mask static_asserts for both boards (lock `0x8C` / `0x38`, reserved `0x03` / `0xC0`); boot log prints the lock mask
- [x] 1.3 Native tests (`test/test_relay_board`): aztec lock masks on both boards, set validity incl. overlap/gap cases, lock mask ↔ relay set round trip

## 2. Lock behaviour
- [x] 2.1 `RelayController::setWheelLock()` drives the whole lock mask (verified whole-port write); not-fitted branch kept under `RELAY_WHEEL_LOCK_FITTED`
- [x] 2.2 `allOff()` holds the lock in its last commanded state (master's code, unchanged; no aztec-only switch)
- [x] 2.3 Relay test: ON refused while the lock is engaged; LOCK clears every test relay (existing fitted-lock code paths, verified)
- [x] 2.4 `VehicleController`: web `set_wheel_lock` and MAVLink channel 7 drive the lock; response text "Lock LOCKED/UNLOCKED"
- [x] 2.5 Telemetry `wheel_lock_fitted` true, `wheel_lock_on` and EFI bit0 follow the lock (existing generic paths, verified)

## 3. Web UI
- [x] 3.1 `data/index.html`: relay labels 4/5/6 (uk + en); vehicle label set chosen from `gear_codes` presence; lock card renamed "Lock" / "Блокування"; relay test help text

## 4. Docs and changelog
- [x] 4.1 `GPIO_PINOUT_CUSTOM_BOARD_S3.md` §6 tables, lock paragraph, harness note, relay test note, §6a XL9535 table, warning 8, pin-migration row
- [x] 4.2 `MAVLINK_SETUP.md` servo7 row and EFI bit0 row
- [x] 4.3 `include/RelayController.h` header comments
- [x] 4.4 `FIRMWARE_VERSION` 1.5.25 and `include/Changelog.h` entry (uk + en)

## 5. Verification
- [x] 5.1 `pio run -e esp32-s3-devkitc-1` builds
- [x] 5.2 `pio test -e native` passes
- [x] 5.3 `openspec validate add-aztec-lock-relays --strict` passes
- [ ] 5.4 Bench (PCA9557): lock from the web energizes X7, X8, X9 together (pins read back `0x8C` with ignition OFF); unlock releases all three
- [ ] 5.5 Bench: MAVLink channel 7 > 1520 µs locks, ≤ 1520 µs unlocks; EFI_STATUS bit0 follows
- [ ] 5.6 Bench: relay test ON refused while locked; LOCK with a test relay on switches it off
- [ ] 5.7 Bench (XL9535, if fitted): lock reads back `0x38`
