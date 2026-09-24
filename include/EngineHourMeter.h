#ifndef ENGINE_HOUR_METER_H
#define ENGINE_HOUR_METER_H

#include <Arduino.h>
#include "Constants.h"

/**
 * @brief Engine hour meter ("мотогодини") — running time, persisted across power loss.
 *
 * TWO counters, exactly as the odometer has ODO and TRIP: a TOTAL that only ever increases and
 * has no reset on any interface, and a resettable TRIP total ("мотогодини місії") that measures
 * the same "since the operator last pressed reset" interval the trip distance does. Both are
 * incremented by the same whole-second amount, in the same place, from the same single
 * sub-second carry — they can never disagree about how long the engine ran.
 *
 * Distance is the wrong unit for most of what this machine wears out: oil, filters,
 * belts and valve clearances track RUNNING TIME, and a quad bike spends much of its
 * life idling, winching and manoeuvring — hours the odometer records as zero.
 *
 * The counting rule is "the crank is turning, on CAN's own testimony": time accrues
 * only while `VehicleData.dataValid` is true AND the reported RPM is at or above
 * `ENGINE_HOURS_MIN_RPM`. While CAN is invalid the engine state is UNKNOWN, and an
 * unknown state must not invent hours — the elapsed interval is DISCARDED, never
 * banked, so a bus outage cannot be paid back on reconnect. The same principle by
 * which the odometer refuses to integrate the decayed speed estimate.
 *
 * Time comes from measured `millis()` deltas, not from a fixed per-iteration tick, so
 * the total tracks real time regardless of the loop rate. A single delta longer than
 * `ENGINE_HOURS_MAX_DELTA_MS` is thrown away whole: a stalled loop or a clock anomaly
 * must not be able to inject time the engine did not run.
 *
 * Storage is EXACT `uint64_t` whole seconds with the sub-second remainder carried
 * between updates, so nothing is lost to truncation. (A float accumulator would
 * eventually stop moving altogether: at 1000 h expressed in seconds a float32 ULP is
 * 0.25 s, and a 20 ms loop increment would add nothing at all.) Hours are a
 * PRESENTATION of that counter — never read back, re-accumulated, or used to
 * reconstruct it.
 *
 * This class is the single owner of the NVS namespace `"engine"`, keys `"hours_s"` and
 * `"trip_s"` (uint64 seconds each; a key that has never been written reads 0). Both are
 * written by the same `persist()` call under one dirty flag, so a shutdown can never save
 * one and lose the other.
 *
 * The TOTAL is a vehicle-lifetime counter: it only ever increases, there is NO reset on any
 * interface — no command, no web control, no constant — and no seeding path. The only way
 * back to zero is an NVS erase. `resetTrip()` zeroes the TRIP counter alone and MUST NOT
 * touch the total; it has no MAVLink command and no web control of its own, being invoked
 * from the one place the trip DISTANCE is reset so that a single operator gesture clears
 * both readings of the same interval.
 */
class EngineHourMeter {
public:
    EngineHourMeter();

    /**
     * @brief Load the meter from NVS and log the restored value.
     *
     * Must be called from `setup()`, not from a constructor: the owning controller is
     * a global built before NVS is ready. A missing key reads 0 — the correct starting
     * meter for a virgin NVS, and there is no seeding path.
     */
    void begin();

    /**
     * @brief Accumulate the interval since the previous call. Never blocks.
     *
     * @param engineTurning true only when CAN data is VALID and the reported engine RPM
     *                      is at or above `ENGINE_HOURS_MIN_RPM`. False covers both
     *                      "engine stopped" and "engine state unknown"; neither adds time.
     * @param nowMs         `millis()` at the call site.
     *
     * The first call only latches the clock. Afterwards the elapsed delta is added when
     * the engine is turning, dropped otherwise, and dropped in either case when it
     * exceeds `ENGINE_HOURS_MAX_DELTA_MS`. Both counters receive the SAME whole-second
     * amount from the SAME carry. Triggers an NVS write once the total has grown by
     * `ENGINE_HOURS_NVS_WRITE_INTERVAL_S` since the last one.
     */
    void update(bool engineTurning, uint32_t nowMs);

    /** @brief Exact accumulated running time in whole seconds — the authoritative TOTAL counter */
    uint64_t getEngineSeconds() const { return engineSeconds_; }

    /** @brief TOTAL running time in hours — a PRESENTATION of getEngineSeconds(), never read back */
    float getEngineHours() const { return (float)(engineSeconds_ / 3600.0); }

    /** @brief Exact TRIP running time in whole seconds since the last `resetTrip()` */
    uint64_t getTripSeconds() const { return tripSeconds_; }

    /** @brief TRIP running time in hours — a PRESENTATION of getTripSeconds(), never read back */
    float getTripHours() const { return (float)(tripSeconds_ / 3600.0); }

    /**
     * @brief Zero the TRIP hours and write them through immediately. The TOTAL is untouched.
     *
     * Deliberately has NO interface of its own — no MAVLink command, no `param1` magic, no web
     * control. It is called from the single site that performs the latched TRIP DISTANCE reset,
     * so `MAV_CMD_USER_1` / `param1 = 1` clears both readings of the same "since last reset"
     * interval in one operator gesture. A zero read afterwards is a GENUINE zero, not "unknown".
     */
    void resetTrip();

    /**
     * @brief Flush BOTH counters to NVS now (ignition OFF / fail-safe).
     *
     * A no-op — no NVS open, no log — when nothing has changed since the last successful
     * write. Never blocks and never retries in a loop; a failed open is logged and the
     * call returns with the in-RAM counters intact and still accumulating. The write counts
     * as successful only when BOTH keys wrote, so a half-written pair stays dirty and is
     * retried at the next trigger.
     */
    void persist();

private:
    static constexpr const char* NVS_NAMESPACE = "engine";
    static constexpr const char* NVS_KEY_HOURS = "hours_s";
    static constexpr const char* NVS_KEY_TRIP  = "trip_s";

    uint64_t engineSeconds_;      // the TOTAL counter — only ever loaded or incremented
    uint64_t tripSeconds_;        // the TRIP counter — loaded, incremented, or zeroed by resetTrip()
    uint32_t remainderMs_;        // sub-second carry, so no delta is lost to truncation
    uint32_t lastUpdateMs_;       // millis() at the previous update(), wrap-safe when subtracted
    uint64_t lastWriteSeconds_;   // value of engineSeconds_ at the last write attempt
    bool     started_;            // false until the first update() has latched the clock
    bool     dirty_;              // RAM differs from NVS — the whole write budget hangs on this
};

#endif  // ENGINE_HOUR_METER_H
