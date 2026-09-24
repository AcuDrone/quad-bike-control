#include "EngineHourMeter.h"
#include "Debug.h"
#include <Preferences.h>

EngineHourMeter::EngineHourMeter()
    : engineSeconds_(0),
      tripSeconds_(0),
      remainderMs_(0),
      lastUpdateMs_(0),
      lastWriteSeconds_(0),
      started_(false),
      dirty_(false) {
}

void EngineHourMeter::begin() {
    Preferences prefs;
    if (prefs.begin(NVS_NAMESPACE, true)) {
        // A key that has never been written reads as the 0 default — the correct starting
        // meter for a virgin NVS. There is no seeding path and no back-dating mechanism.
        engineSeconds_ = prefs.getULong64(NVS_KEY_HOURS, 0);
        // The trip key is absent on a vehicle upgrading from the total-only firmware, which
        // reads as 0 — the correct starting point for a counter that has never been reset.
        tripSeconds_   = prefs.getULong64(NVS_KEY_TRIP, 0);
        prefs.end();
        Debug::printfFeature(DebugFeature::VEHICLE,
                             "[ENGINE] hour meter %.2f h, trip %.2f h restored\n",
                             getEngineHours(), getTripHours());
    } else {
        // Virgin NVS (the namespace does not exist yet) or a read failure: nothing was
        // restored, so say so rather than reporting a "restored" 0.00 that was never stored.
        Debug::printlnFeature(DebugFeature::VEHICLE,
            "[ENGINE] NVS read failed or namespace absent — hour meter starts at 0.00 h");
    }
    lastWriteSeconds_ = engineSeconds_;
    dirty_            = false;   // RAM matches NVS (or both are the 0 default): nothing to write
}

void EngineHourMeter::update(bool engineTurning, uint32_t nowMs) {
    if (!started_) {
        // No previous sample, so no interval exists yet. Latch the clock and wait.
        started_      = true;
        lastUpdateMs_ = nowMs;
        return;
    }

    // Unsigned subtraction, so the 49.7-day millis() wrap is handled by modular arithmetic.
    const uint32_t dtMs = nowMs - lastUpdateMs_;
    // Advance UNCONDITIONALLY, before any early return. This is what makes a skipped interval
    // DISCARDED rather than banked: when CAN comes back, the outage is not credited as hours.
    lastUpdateMs_ = nowMs;

    if (!engineTurning) {
        return;   // engine stopped, or engine state unknown — neither invents running time
    }
    if (dtMs > ENGINE_HOURS_MAX_DELTA_MS) {
        return;   // a stalled loop or a clock anomaly, not attested running time
    }

    // Carry the sub-second remainder: at a 25-50 Hz loop rate every delta is 20-40 ms, so
    // truncating to whole seconds per update would discard every single one and the meter
    // would never move.
    remainderMs_ += dtMs;
    if (remainderMs_ >= 1000) {
        // ONE carry feeds BOTH counters with the SAME amount, so total and trip can never
        // disagree about how long the engine ran. A second remainder would round twice and
        // let the trip drift against the total by a second here and there.
        const uint64_t wholeSeconds = remainderMs_ / 1000;
        engineSeconds_ += wholeSeconds;
        tripSeconds_   += wholeSeconds;
        remainderMs_   %= 1000;
        dirty_ = true;

        // Unsigned and monotonic — engineSeconds_ never decreases, so no wrap handling.
        if (engineSeconds_ - lastWriteSeconds_ >= ENGINE_HOURS_NVS_WRITE_INTERVAL_S) {
            persist();
        }
    }
}

void EngineHourMeter::persist() {
    // The dirty flag is the whole write budget. The flush triggers repeat freely — applyFailsafe()
    // fires on the first loop of EVERY boot (the input source is FAILSAFE until MAVLink or a
    // browser appears) and again on every link flap, and getIgnitionState() has no hysteresis, so
    // a servo channel jittering at a band edge can produce one "OFF transition" per inbound frame
    // at 25 Hz. Without this guard each of those would rewrite a value NVS already holds.
    if (!dirty_) {
        return;                  // NVS already holds this value — no open, no log, no wear
    }

    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, false)) {
        // Never block and never retry in a loop. Advancing the mark here is deliberate: it
        // disarms the interval trigger, which would otherwise stay true and re-attempt (and
        // re-log) on every accumulated second for as long as the engine keeps running.
        // dirty_ stays set, so the next interval — or the next ignition-OFF — tries again.
        Debug::printlnFeature(DebugFeature::VEHICLE,
            "[ENGINE] NVS open failed — hour meter not persisted");
        lastWriteSeconds_ = engineSeconds_;
        return;
    }
    // Preferences returns the number of bytes written, 0 on failure. A failed write leaves the
    // meter dirty so a later trigger retries, but the mark still advances for the same
    // no-spinning reason as above. Both keys go out under the ONE dirty flag: the write counts
    // as successful only if BOTH wrote, so a half-written pair is retried rather than being
    // marked clean with one counter stale in flash.
    const size_t wroteHours = prefs.putULong64(NVS_KEY_HOURS, engineSeconds_);
    const size_t wroteTrip  = prefs.putULong64(NVS_KEY_TRIP, tripSeconds_);
    prefs.end();
    lastWriteSeconds_ = engineSeconds_;
    if (wroteHours == 0 || wroteTrip == 0) {
        Debug::printlnFeature(DebugFeature::VEHICLE, "[ENGINE] NVS write failed");
        return;                  // still dirty — retry at the next trigger
    }
    dirty_ = false;
}

void EngineHourMeter::resetTrip() {
    // The TRIP counter alone. engineSeconds_ is deliberately NOT named here: the total is a
    // vehicle-lifetime counter with no reset on any interface, and this is the one function
    // in the class that zeroes anything.
    tripSeconds_ = 0;
    dirty_       = true;
    // Write through immediately rather than waiting for the next interval or ignition-OFF: the
    // operator has just performed a deliberate gesture and expects it to survive a power cut,
    // exactly as the trip DISTANCE reset persists on the spot.
    persist();
    Debug::printfFeature(DebugFeature::VEHICLE,
                         "[ENGINE] trip hours reset to 0 (total %.2f h unchanged)\n",
                         getEngineHours());
}
