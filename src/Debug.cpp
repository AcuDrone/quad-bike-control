#include "Debug.h"
#include "Constants.h"
#include <stdarg.h>

// Global debug enabled flag (initialized from Constants.h default)
bool g_debugEnabled = DEBUG_ENABLED;

// Static member initialization
Preferences Debug::preferences;
uint16_t Debug::featureFlags = 0;  // All features OFF by default

void Debug::begin() {
    // Load saved debug state from NVS
    loadState();
    // Load feature flags from NVS
    loadFeatureFlags();
}

void Debug::loadState() {
    preferences.begin("debug", true); // Read-only mode

    // Load debug state from NVS, default to Constants.h value if not set
    bool hasKey = preferences.isKey("enabled");
    if (hasKey) {
        g_debugEnabled = preferences.getBool("enabled", DEBUG_ENABLED);
    } else {
        g_debugEnabled = DEBUG_ENABLED; // Use default from Constants.h
    }

    preferences.end();
}

void Debug::saveState() {
    preferences.begin("debug", false); // Read-write mode
    preferences.putBool("enabled", g_debugEnabled);
    preferences.end();
}

void Debug::printf(const char* format, ...) {
    if (!g_debugEnabled) {
        return;
    }

    va_list args;
    va_start(args, format);

    // Use vsnprintf to format the string
    char buffer[256];
    vsnprintf(buffer, sizeof(buffer), format, args);

    va_end(args);

    Serial.print(buffer);
}

bool Debug::isFeatureEnabled(DebugFeature feature) {
    return (featureFlags & featureBit(feature)) != 0;
}

void Debug::setFeatureEnabled(DebugFeature feature, bool enabled) {
    if (enabled) {
        featureFlags |= featureBit(feature);
    } else {
        featureFlags &= ~featureBit(feature);
    }
    saveFeatureFlags();
}

void Debug::loadFeatureFlags() {
    preferences.begin("debug", true); // Read-only mode

    // Load individual feature flags with descriptive keys
    uint16_t loaded = 0;

    if (preferences.getBool("feat_trans", DEBUG_FEATURE_DEFAULT_STATE)) {
        loaded |= featureBit(DebugFeature::TRANSMISSION);
    }
    if (preferences.getBool("feat_can", DEBUG_FEATURE_DEFAULT_STATE)) {
        loaded |= featureBit(DebugFeature::CAN);
    }
    if (preferences.getBool("feat_sbus", DEBUG_FEATURE_DEFAULT_STATE)) {
        loaded |= featureBit(DebugFeature::SBUS);
    }
    if (preferences.getBool("feat_servo", DEBUG_FEATURE_DEFAULT_STATE)) {
        loaded |= featureBit(DebugFeature::SERVO);
    }
    if (preferences.getBool("feat_brake", DEBUG_FEATURE_DEFAULT_STATE)) {
        loaded |= featureBit(DebugFeature::BRAKE);
    }
    if (preferences.getBool("feat_relay", DEBUG_FEATURE_DEFAULT_STATE)) {
        loaded |= featureBit(DebugFeature::RELAY);
    }
    if (preferences.getBool("feat_web", DEBUG_FEATURE_DEFAULT_STATE)) {
        loaded |= featureBit(DebugFeature::WEB);
    }
    if (preferences.getBool("feat_vehicle", DEBUG_FEATURE_DEFAULT_STATE)) {
        loaded |= featureBit(DebugFeature::VEHICLE);
    }
    if (preferences.getBool("feat_telem", DEBUG_FEATURE_DEFAULT_STATE)) {
        loaded |= featureBit(DebugFeature::TELEMETRY);
    }

    featureFlags = loaded;
    preferences.end();
}

void Debug::saveFeatureFlags() {
    preferences.begin("debug", false); // Read-write mode

    // Save individual feature flags with descriptive keys
    preferences.putBool("feat_trans", (featureFlags & featureBit(DebugFeature::TRANSMISSION)) != 0);
    preferences.putBool("feat_can", (featureFlags & featureBit(DebugFeature::CAN)) != 0);
    preferences.putBool("feat_sbus", (featureFlags & featureBit(DebugFeature::SBUS)) != 0);
    preferences.putBool("feat_servo", (featureFlags & featureBit(DebugFeature::SERVO)) != 0);
    preferences.putBool("feat_brake", (featureFlags & featureBit(DebugFeature::BRAKE)) != 0);
    preferences.putBool("feat_relay", (featureFlags & featureBit(DebugFeature::RELAY)) != 0);
    preferences.putBool("feat_web", (featureFlags & featureBit(DebugFeature::WEB)) != 0);
    preferences.putBool("feat_vehicle", (featureFlags & featureBit(DebugFeature::VEHICLE)) != 0);
    preferences.putBool("feat_telem", (featureFlags & featureBit(DebugFeature::TELEMETRY)) != 0);

    preferences.end();
}

void Debug::printfFeature(DebugFeature feature, const char* format, ...) {
    if (!g_debugEnabled || !isFeatureEnabled(feature)) {
        return;
    }

    va_list args;
    va_start(args, format);

    // Use vsnprintf to format the string
    char buffer[256];
    vsnprintf(buffer, sizeof(buffer), format, args);

    va_end(args);

    Serial.print(buffer);
}
