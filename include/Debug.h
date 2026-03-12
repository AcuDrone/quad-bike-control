#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>
#include <Preferences.h>

// Global debug enabled flag (runtime-toggleable)
extern bool g_debugEnabled;

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

private:
    static Preferences preferences;
};

#endif // DEBUG_H
