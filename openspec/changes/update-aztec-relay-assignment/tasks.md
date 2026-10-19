## 1. Relay map
- [x] 1.1 `include/Constants.h`: aztec masks (`RELAY_MASK_IGNITION` = Relay_1/IO4, `RELAY_MASK_STARTER` = Relay_2/IO5, `RELAY_MASK_FRONT_LIGHT` = Relay_3/IO6); add `RELAY_MASK_RESERVED` (IO7|IO3|IO2|IO0|IO1); remove `RELAY_MASK_WHEEL_LOCK`; add `RELAY_WHEEL_LOCK_FITTED 0`; update comments
- [x] 1.2 `RelayController`: build the shadow from the three function masks only, clear `RELAY_MASK_RESERVED`; compile-time check that the masks are disjoint and cover the port
- [x] 1.3 Verify the starter escalation, pre-crank dwell (`ignitionHighSince_`) and all-off paths need no change for a single ignition relay; update comments that assumed two

## 2. Wheel lock not fitted
- [x] 2.1 `RelayController::setWheelLock()` rejects a LOCK (returns `false`, logs once per boot, no port write); `getWheelLock()` always `false`; add `isWheelLockFitted()`
- [x] 2.2 `VehicleController`: `set_wheel_lock` web command answered with an error response; MAVLink channel 7 still decoded and passed through (rejected)
- [x] 2.3 Telemetry: add `wheel_lock_fitted` to the web telemetry struct and JSON
- [x] 2.4 `data/index.html`: disable the wheel-lock button and show "not fitted" (uk + en) when `wheel_lock_fitted` is `false`
- [x] 2.5 MAVLink `EFI_STATUS` bitmask layout unchanged; bit0 documented as always 0 on aztec

## 3. Docs and changelog
- [x] 3.1 `GPIO_PINOUT_CUSTOM_BOARD_S3.md` §6 tables, harness wiring note, warning 8 and the pin-migration table (aztec; quad-bike assignment lives on `master`)
- [x] 3.2 `MAVLINK_SETUP.md` servo channel 7 and `pt_compensation` bit0
- [x] 3.3 `include/RelayController.h` header comment
- [x] 3.4 `include/Changelog.h`: aztec relay assignment and "wheel lock not fitted" in the 1.2.0 entry (uk + en), no version bump

## 4. Web relay test
- [x] 4.1 `include/Constants.h`: `RELAY_TEST_TIMEOUT_MS` (30000) and `RELAY_TEST_STARTER_MAX_MS` (2000); reserved-mask comment notes the test exception
- [x] 4.2 `RelayController`: `relayIoMask()` relay-number → IO-bit lookup with static_asserts against the function masks and `RELAY_MASK_RESERVED`; `setRelayTest()` (relay 1..8, ON only with ignition OFF / no crank armed / light off), `getRelayTestBits()`; test bits in the same shadow byte and verified `writePort()`
- [x] 4.3 Auto-off in `update()` (30 s, starter 2 s); all test bits cleared on ignition leaving OFF, crank, front light ON, `allOff()`, a write fault and the starter escalation; every change logged under `DebugFeature::RELAY`
- [x] 4.4 `VehicleController`: `set_relay_test` web command (`relay` + `value`) → `processRelayTestCommand()`, error response for a rejected request; not gated by input source
- [x] 4.5 Telemetry: `relay_test` bitfield (bit0 = Relay_1 … bit7 = Relay_8) in the struct and JSON
- [x] 4.6 `data/index.html`: "Relay Test" card with eight toggles (uk + en), state from `relay_test`, ON disabled unless ignition OFF / light off / relay board healthy, responses shown in the card
- [x] 4.7 Docs: `GPIO_PINOUT_CUSTOM_BOARD_S3.md` §6 relay test note; changelog 1.2.0 line (uk + en)

## 5. Verification
- [x] 5.1 `pio run` builds with no new warnings in project code
- [x] 5.2 `openspec validate update-aztec-relay-assignment --strict` passes
- [ ] 5.3 Bench: ignition request energises Relay_1 (X4) only; port reads back `0x10`
- [ ] 5.4 Bench: crank energises Relay_2 (X5) after the pre-crank dwell and drops it on RPM / timeout; port reads back `0x30` while cranking
- [ ] 5.5 Bench: front light energises Relay_3 (X6) only; Relay_4..Relay_8 never click in any ignition / light state
- [ ] 5.6 Bench: wheel-lock button shows "not fitted" and is disabled; MAVLink channel 7 high does not energise any relay and EFI bit0 stays 0
- [ ] 5.7 Bench: with ignition OFF toggle each relay 1..8 from the Relay Test card, verify with a multimeter on X4..X11 (especially the locks on X7/X8/X9), verify the 30 s auto-off (Relay_2: 2 s), that ON is refused with ignition ACC, and that selecting ACC / light ON drops every test relay
