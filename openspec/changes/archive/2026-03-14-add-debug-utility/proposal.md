# Change: Add Runtime-Toggleable Debug Utility

## Why
Currently, all Serial.print/println calls execute unconditionally, resulting in serial output that cannot be disabled at runtime. Debug output consumes bandwidth, CPU cycles, and cannot be toggled dynamically through the web portal for troubleshooting without recompilation.

## What Changes
- Create a Debug utility class with print/println methods that check DEBUG_ENABLED at runtime
- Add web portal API endpoint to toggle DEBUG_ENABLED flag
- Refactor all existing Serial.print/println calls (70+ occurrences across 11 files) to use Debug::print/println
- Maintain backward compatibility with existing DEBUG_ENABLED constant in Constants.h

## Impact
- Affected specs: **NEW** debug-logging (new capability)
- Affected code:
  - src/main.cpp (30+ Serial calls)
  - src/WebPortal.cpp (20+ Serial calls)
  - src/RelayController.cpp (8 Serial calls)
  - src/CANController.cpp, SBusInput.cpp, VehicleController.cpp, TransmissionController.cpp, BTS7960Controller.cpp, EncoderCounter.cpp, ServoController.cpp
  - include/Constants.h (DEBUG_ENABLED definition)
- Benefits:
  - Runtime debug toggling via web portal (no recompile/reflash needed)
  - Reduced serial bandwidth when debugging disabled
  - Consistent debug output formatting
  - Easier troubleshooting in deployed systems
