## 1. Implementation
- [x] 1.1 Update `SBusChannelConfig` in Constants.h: remove `BRAKE`, shift IGNITION to CH4, FRONT_LIGHT to CH5
- [x] 1.2 Modify `SBusInput::getThrottle()`: return 0–100% only when CH2 > 1500μs (center), 0% at or below center
- [x] 1.3 Modify `SBusInput::getBrake()`: read from THROTTLE channel, return 0–100% when CH2 < 1500μs, 0% at or above center
- [ ] 1.4 Update ArduPilot servo output mapping to match new channel assignments (IGNITION→CH4, FRONT_LIGHT→CH5)

## 2. Validation
- [x] 2.1 Build compiles without errors
- [ ] 2.2 Test throttle: stick above center shows 0–100%, at center shows 0%
- [ ] 2.3 Test brake: stick below center shows 0–100%, at center shows 0%
- [ ] 2.4 Verify ignition and front light work on new channels (CH4, CH5)
