# Change: Add SBusInput Class with Channel Configuration

## Why
The system currently has SBUS library integration and channel constants defined, but lacks an actual SBusInput class implementation. This prevents SBUS control from ArduPilot Rover, forcing the system to default to web-only or fail-safe modes. The TelemetryManager and VehicleController have placeholders expecting this class. Additionally, the vehicle requires ignition control (OFF/ACC/IGNITION states) and front light control for operational functionality.

## What Changes
- Create SBusInput class (include/SBusInput.h, src/SBusInput.cpp) wrapping Bolder Flight SBUS library
- Implement channel configuration structure in Constants.h for mapping SBUS channels to vehicle functions
- Add raw channel value access (880-2160μs range)
- Add mapped vehicle command methods:
  - steering % (Ch1), throttle % (Ch2), gear enum (Ch3: R/N/L), brake % (Ch4)
  - **NEW:** ignition state enum (Ch5: OFF/ACC/IGNITION)
  - **NEW:** front light on/off (Ch6: boolean)
- Implement signal validity detection and fail-safe triggering
- Add signal quality monitoring (frame rate, error counts, signal age)
- **NEW:** Create RelayController class for managing relay outputs (ignition, lights)
- Integrate SBusInput with TelemetryManager for input source priority
- Integrate SBusInput with VehicleController for command processing and relay control

## Impact
- Affected specs: **sbus-input** (MODIFIED - add implementation details, ignition, lights)
- Affected code:
  - NEW: include/SBusInput.h, src/SBusInput.cpp
  - NEW: include/RelayController.h, src/RelayController.cpp (manages relay outputs)
  - MODIFIED: src/main.cpp (instantiate SBusInput and RelayController)
  - MODIFIED: src/TelemetryManager.cpp (integrate SBUS input source detection)
  - MODIFIED: src/VehicleController.cpp (process SBUS commands including ignition/lights)
  - MODIFIED: include/Constants.h (add channel configuration structure, ignition enums, relay assignments)
- Benefits: Enables primary control input from ArduPilot, completes vehicle control system with ignition and lighting
- Risks: Signal loss handling must be robust; relay switching must fail-safe to OFF state
