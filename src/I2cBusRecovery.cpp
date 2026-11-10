#include "I2cBusRecovery.h"
#include <Wire.h>
#include <driver/gpio.h>
#include "Constants.h"
#include "Debug.h"
#include "EventJournal.h"

namespace {
uint32_t s_count = 0;
uint32_t s_lastRunMs = 0;
bool s_ran = false;
// Journal throttle: identical unsuccessful runs (SDA/SCL still stuck) are written at most once
// per minute with the number of runs since the last line; a change of outcome is always written.
int8_t s_lastReleased = -1;
uint32_t s_lastLogMs = 0;
uint32_t s_runsSinceLog = 0;
constexpr uint32_t LOG_REPEAT_MS = 60000;

inline int level(gpio_num_t pin) { return gpio_get_level(pin); }
}  // namespace

namespace I2cBusRecovery {

bool isBusError(uint8_t wireErr) {
    return wireErr == 5;   // timeout (see the header for why 4 is not evidence on this core)
}

bool beginBus() {
    const bool ok = Wire1.begin(PIN_RELAY_SDA, PIN_RELAY_SCL, I2C_BUS_FREQ_HZ);
    Wire1.setTimeOut(I2C_TIMEOUT_MS);
    return ok;
}

uint32_t count() { return s_count; }

bool recover(const char* by, uint8_t lastErr) {
    const uint32_t now = millis();
    if (s_ran && (now - s_lastRunMs) < I2C_RECOVER_MIN_MS) {
        return false;
    }

    // GPIO input reads work while the pins are routed to the I2C controller.
    int sda = level(PIN_RELAY_SDA);
    int scl = level(PIN_RELAY_SCL);
    if (sda && scl && !isBusError(lastErr)) {
        return false;   // healthy lines, plain NACK / read failure: the device re-init is enough
    }
    s_ran = true;
    s_lastRunMs = now;

    const int sda0 = sda;
    const int scl0 = scl;
    uint8_t clocks = 0;
    Wire1.end();
    if (!sda && scl) {
        // A slave interrupted mid-byte keeps SDA low waiting for its clocks: give them.
        pinMode(PIN_RELAY_SDA, INPUT);
        pinMode(PIN_RELAY_SCL, OUTPUT_OPEN_DRAIN);
        digitalWrite(PIN_RELAY_SCL, HIGH);
        delayMicroseconds(5);
        while (clocks < I2C_UNLOCK_CLOCKS && level(PIN_RELAY_SDA) == 0) {
            digitalWrite(PIN_RELAY_SCL, LOW);
            delayMicroseconds(5);
            digitalWrite(PIN_RELAY_SCL, HIGH);
            delayMicroseconds(5);
            clocks++;
        }
        // STOP: SDA low while SCL is high, then release SDA.
        pinMode(PIN_RELAY_SDA, OUTPUT_OPEN_DRAIN);
        digitalWrite(PIN_RELAY_SDA, LOW);
        delayMicroseconds(5);
        digitalWrite(PIN_RELAY_SCL, HIGH);
        delayMicroseconds(5);
        digitalWrite(PIN_RELAY_SDA, HIGH);
        delayMicroseconds(5);
        pinMode(PIN_RELAY_SDA, INPUT);
        pinMode(PIN_RELAY_SCL, INPUT);
        sda = level(PIN_RELAY_SDA);
        scl = level(PIN_RELAY_SCL);
    }
    const bool reinit = beginBus();
    s_count++;

    const int released = (sda && scl) ? 1 : 0;
    s_runsSinceLog++;
    if (released != s_lastReleased || (now - s_lastLogMs) >= LOG_REPEAT_MS) {
        EventJournal::log(JSrc::I2C, JLvl::W, "bus_stuck",
                          "sda=%d scl=%d clocks=%u released=%d reinit=%d by=%s err=%u runs=%lu",
                          sda0, scl0, (unsigned)clocks, released, reinit ? 1 : 0, by ? by : "?",
                          (unsigned)lastErr, (unsigned long)s_runsSinceLog);
        s_lastReleased = (int8_t)released;
        s_lastLogMs = now;
        s_runsSinceLog = 0;
    }
    Debug::printfFeature(DebugFeature::RELAY,
        "[I2C] Bus recovery (%s, err %u): SDA=%d SCL=%d, %u clocks -> released=%d, Wire1 restarted%s\n",
        by ? by : "?", (unsigned)lastErr, sda0, scl0, (unsigned)clocks, released,
        reinit ? "" : " (begin FAILED)");
    return true;
}

}  // namespace I2cBusRecovery
