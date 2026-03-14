# Change: Add Feature-Specific Logging Control

## Why
Currently, the Debug utility has a single global on/off switch that controls all debug output. When debugging specific issues (e.g., transmission calibration problems or CAN communication issues), developers need to wade through logs from all systems. Feature-specific logging allows enabling logs for only the relevant subsystems, reducing noise and making debugging more efficient.

## What Changes
- Extend Debug utility to support per-feature logging flags (TRANSMISSION, CAN, SBUS, SERVO, BRAKE, RELAY, WEB, VEHICLE, TELEMETRY)
- Add DebugFeature enum to identify logging categories
- Store feature-specific flags in NVS for persistence across reboots
- Implement two-tier logging: master debug switch AND feature flag must both be enabled for output
- All feature flags default to OFF
- Control only through code (no web UI) - developers configure flags programmatically or via NVS tools

## Impact
- Affected specs: web-control (remove web UI debug toggle, clarify code-only control)
- Affected code:
  - `include/Debug.h`: Add DebugFeature enum, feature-specific logging methods
  - `src/Debug.cpp`: Implement feature flag storage, loading, saving to NVS
  - `include/Constants.h`: Add default feature flag states (all OFF)
  - All component files: Update Debug calls to specify feature category
- Backward compatible: Existing `Debug::println()` calls continue to work with master switch
- Migration: Existing web UI debug toggle remains functional for master switch
