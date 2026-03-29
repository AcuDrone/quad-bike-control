# Change: Combine throttle and brake into single SBUS channel

## Why
ArduPilot Rover uses a single channel for throttle/brake (forward above center, reverse/brake below center). Using two separate channels wastes CH4 and doesn't match how ArduPilot outputs throttle commands.

## What Changes
- CH2 becomes a combined throttle/brake channel: above 1500μs = throttle (0–100%), below 1500μs = brake (0–100%)
- Remove CH4 (BRAKE) from SBusChannelConfig — frees up the channel
- Modify `getThrottle()` and `getBrake()` to both read from CH2 and split at center
- Update Constants.h: remove BRAKE channel assignment, shift remaining channels if needed
- Update VehicleController to work with the new combined input

## Impact
- Affected specs: `sbus-input` (channel mapping, throttle/brake conversion)
- Affected code: `Constants.h`, `SBusInput.cpp` (getThrottle, getBrake), `VehicleController.cpp`
