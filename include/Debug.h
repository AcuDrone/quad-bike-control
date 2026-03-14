#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>
#include <Preferences.h>

// Global debug enabled flag (runtime-toggleable)
extern bool g_debugEnabled;

/**
 * @brief Debug feature categories for granular logging control
 *
 * Two-tier logging system:
 * 1. Master debug switch (g_debugEnabled) - global on/off
 * 2. Feature flags - per-subsystem control
 *
 * Both master AND feature flag must be enabled for output.
 * All features default to OFF. Enable via code or NVS tools.
 */
enum class DebugFeature {
    TRANSMISSION,  // TransmissionController logs
    CAN,          // CANController logs
    SBUS,         // SBusInput logs
    SERVO,        // ServoController logs
    BRAKE,        // BTS7960Controller (brake) logs
    RELAY,        // RelayController (ignition/lights) logs
    WEB,          // WebPortal logs
    VEHICLE,      // VehicleController logs
    TELEMETRY     // TelemetryManager logs
};

// Debug utility class for conditional serial output
class Debug {
public:
    // Initialize debug system (call in setup())
    static void begin();

    // Load debug state from NVS
    static void loadState();

    // Save debug state to NVS
    static void saveState();

    // Get current debug state
    static bool isEnabled() { return g_debugEnabled; }

    // Set debug state
    static void setEnabled(bool enabled) {
        g_debugEnabled = enabled;
        saveState();
    }

    // Check if a specific feature's debug output is enabled
    static bool isFeatureEnabled(DebugFeature feature);

    // Enable/disable debug output for a specific feature
    static void setFeatureEnabled(DebugFeature feature, bool enabled);

    // Print without newline (if debug enabled)
    template<typename T>
    static void print(T value) {
        if (g_debugEnabled) {
            Serial.print(value);
        }
    }

    // Print with newline (if debug enabled)
    template<typename T>
    static void println(T value) {
        if (g_debugEnabled) {
            Serial.println(value);
        }
    }

    // Print empty line
    static void println() {
        if (g_debugEnabled) {
            Serial.println();
        }
    }

    // Printf-style formatted output (if debug enabled)
    static void printf(const char* format, ...) __attribute__((format(printf, 1, 2)));

    // ===== Feature-Specific Logging Methods =====
    // These check BOTH master debug AND feature flag

    // Print without newline (if master debug AND feature enabled)
    template<typename T>
    static void printFeature(DebugFeature feature, T value) {
        if (g_debugEnabled && isFeatureEnabled(feature)) {
            Serial.print(value);
        }
    }

    // Print with newline (if master debug AND feature enabled)
    template<typename T>
    static void printlnFeature(DebugFeature feature, T value) {
        if (g_debugEnabled && isFeatureEnabled(feature)) {
            Serial.println(value);
        }
    }

    // Printf-style formatted output (if master debug AND feature enabled)
    static void printfFeature(DebugFeature feature, const char* format, ...) __attribute__((format(printf, 2, 3)));

private:
    static Preferences preferences;

    // Feature flag storage (bitmask for efficiency)
    static uint16_t featureFlags;

    // Load feature flags from NVS
    static void loadFeatureFlags();

    // Save feature flags to NVS
    static void saveFeatureFlags();

    // Convert DebugFeature enum to bitmask position
    static constexpr uint16_t featureBit(DebugFeature feature) {
        return 1 << static_cast<uint8_t>(feature);
    }
};

#endif // DEBUG_H
