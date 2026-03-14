# Implementation Tasks

## 1. Debug System Enhancement
- [x] 1.1 Add DebugFeature enum to Debug.h with categories (TRANSMISSION, CAN, SBUS, SERVO, BRAKE, RELAY, WEB, VEHICLE, TELEMETRY)
- [x] 1.2 Add feature flag storage (bitmask or individual bools) to Debug class
- [x] 1.3 Add isFeatureEnabled(DebugFeature) static method
- [x] 1.4 Add setFeatureEnabled(DebugFeature, bool) static method
- [x] 1.5 Add loadFeatureFlags() to load from NVS on initialization
- [x] 1.6 Add saveFeatureFlags() to persist to NVS
- [x] 1.7 Update Debug::begin() to load both master and feature flags
- [x] 1.8 Add feature-specific logging methods: printFeature(), printlnFeature(), printfFeature()

## 2. Constants and Defaults
- [x] 2.1 Add DEBUG_FEATURE_DEFAULT_STATE constant to Constants.h (false = OFF)
- [x] 2.2 Document feature flag control approach (code-only, no web UI)

## 3. NVS Storage Schema
- [x] 3.1 Store feature flags in "debug" NVS namespace with keys: "feat_trans", "feat_can", etc.
- [x] 3.2 Ensure backward compatibility with existing "enabled" master switch key
- [x] 3.3 Handle missing keys gracefully (default to DEBUG_FEATURE_DEFAULT_STATE)

## 4. Update Logging Calls - Transmission
- [x] 4.1 Update TransmissionController.cpp to use DebugFeature::TRANSMISSION
- [x] 4.2 Replace Debug::println with Debug::printlnFeature(TRANSMISSION, ...)
- [x] 4.3 Replace Debug::printf with Debug::printfFeature(TRANSMISSION, ...)

## 5. Update Logging Calls - CAN
- [x] 5.1 Update CANController.cpp to use DebugFeature::CAN
- [x] 5.2 Replace logging calls with feature-specific versions

## 6. Update Logging Calls - SBUS
- [x] 6.1 Update SBusInput.cpp to use DebugFeature::SBUS
- [x] 6.2 Replace logging calls with feature-specific versions

## 7. Update Logging Calls - Servo
- [x] 7.1 Update ServoController.cpp to use DebugFeature::SERVO
- [x] 7.2 Replace logging calls with feature-specific versions

## 8. Update Logging Calls - Brake
- [x] 8.1 Update BTS7960Controller.cpp to use DebugFeature::BRAKE
- [x] 8.2 Replace logging calls with feature-specific versions

## 9. Update Logging Calls - Relay
- [x] 9.1 Update RelayController.cpp to use DebugFeature::RELAY
- [x] 9.2 Replace logging calls with feature-specific versions

## 10. Update Logging Calls - Web
- [x] 10.1 Update WebPortal.cpp to use DebugFeature::WEB
- [x] 10.2 Replace logging calls with feature-specific versions

## 11. Update Logging Calls - Vehicle
- [x] 11.1 Update VehicleController.cpp to use DebugFeature::VEHICLE
- [x] 11.2 Replace logging calls with feature-specific versions

## 12. Update Logging Calls - Telemetry
- [x] 12.1 Update TelemetryManager.cpp to use DebugFeature::TELEMETRY (no logs exist - skipped)

## 13. Documentation
- [x] 13.1 Add code comments explaining two-tier logging (master + feature)
- [x] 13.2 Document how to enable feature flags (constants or NVS tools)
- [x] 13.3 Add example usage in Debug.h header comments

## 14. Testing and Validation
- [x] 14.1 Test master debug OFF disables all output regardless of feature flags
- [x] 14.2 Test master debug ON + feature OFF suppresses that feature's logs
- [x] 14.3 Test master debug ON + feature ON enables that feature's logs
- [x] 14.4 Test NVS persistence of feature flags across reboots
- [x] 14.5 Test backward compatibility with existing debug toggle
- [x] 14.6 Verify all feature categories have appropriate log coverage
- [x] 14.7 Test enabling multiple features simultaneously
- [x] 14.8 Test default state (all features OFF) on fresh NVS
