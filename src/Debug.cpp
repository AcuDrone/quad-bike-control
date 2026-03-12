#include "Debug.h"
#include "Constants.h"
#include <stdarg.h>

// Global debug enabled flag (initialized from Constants.h default)
bool g_debugEnabled = DEBUG_ENABLED;

// Static member initialization
Preferences Debug::preferences;

void Debug::begin() {
    // Load saved debug state from NVS
    loadState();
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
