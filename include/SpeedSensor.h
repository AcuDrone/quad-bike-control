#ifndef SPEED_SENSOR_H
#define SPEED_SENSOR_H

#include <Arduino.h>
#include "Constants.h"
#include "driver/pulse_cnt.h"

/**
 * @brief Vehicle speed from the 3-pin hall sensor wired straight to PIN_SPEED_SENSOR.
 *
 * The pulse train is counted by the ESP32-S3 PCNT peripheral (hardware glitch
 * filter, hardware overflow accumulation), and `update()` samples the running
 * count on a fixed window from the cooperative loop — no ISR, no FreeRTOS task.
 *
 * Speed is an INDEPENDENT source from the CAN bus: `VehicleData.vehicleSpeed`
 * stays dead (OBD-II PID 0x0D is deliberately disabled) and this class carries
 * its own validity, so a CAN outage never blanks a good speed reading.
 *
 * "Stopped" and "disconnected" are indistinguishable at a passive pulse sensor,
 * so the two facts are reported separately:
 *  - `getSpeedMs()` decays to 0 after `SPEED_STALE_TIMEOUT_MS` of silence —
 *    the physically correct reading for a stopped vehicle;
 *  - `isValid()` is the health signal: false until the first pulse since boot,
 *    and latched false when pulses cease faster than a plausible deceleration
 *    (the fingerprint of a mid-motion wire fault). Each consumer picks its own
 *    fail-safe direction from that.
 *
 * This class is the single owner of the NVS namespace `"speed"`, which holds the
 * calibration (pulses/rev + wheel circumference) AND the two distance counters
 * (`odo_mm` / `trip_mm`). The max-speed limiter has NO persisted configuration:
 * its ceiling comes solely from the autopilot's SPEED_MAX parameter and is
 * applied in `VehicleController`.
 *
 * Distance is accumulated in exact `uint64_t` MILLIMETRES from counted pulses
 * only — the decayed estimate emitted during pulse silence adds nothing, and a
 * sample rejected by the wrap guard adds nothing. The odometer only ever
 * increases; the trip counter is cleared solely by `resetTrip()`.
 *
 * All speeds here are METRES PER SECOND. km/h exists only at the presentation
 * edge (web JSON, human-readable debug strings).
 */
class SpeedSensor {
public:
    SpeedSensor();

    /**
     * @brief Load calibration from NVS and configure the PCNT unit.
     * @return true if the PCNT unit is counting (false leaves the sensor invalid forever)
     */
    bool begin();

    /**
     * @brief Sample the pulse counter at most every SPEED_SAMPLE_INTERVAL_MS.
     * Never blocks; safe to call every loop().
     */
    void update();

    // ---- Readings ----------------------------------------------------------

    /** @brief Latest speed in m/s (0 when stale — see isValid() for health) */
    float getSpeedMs() const { return speedMs_; }

    /** @brief True only when the reading can be trusted (pulsed since boot, not suspicious) */
    bool isValid() const { return initialized_ && everPulsed_ && !suspicious_; }

    /** @brief True while an implausible pulse loss is latched (mid-motion fault fingerprint) */
    bool isSuspicious() const { return suspicious_; }

    /** @brief Running pulse total since boot (diagnostics / bench calibration) */
    uint32_t getPulseTotal() const { return pulseTotal_; }

    // ---- Noise diagnostics -------------------------------------------------
    // Pure instrumentation for the ignition-interference hunt: nothing here feeds speed,
    // distance or validity. They quantify what the glitch filter and the burst guard are
    // still letting through, so the next layer can be sized from measurements.

    /** @brief Pulses read from PCNT in the MOST RECENT sample window (0 when the wheel is still) */
    uint32_t getRawPulsesLastWindow() const { return rawPulsesLastWindow_; }

    /** @brief Windows discarded since boot by the SPEED_MAX_PULSES_PER_SAMPLE burst guard */
    uint32_t getRejectedWindows() const { return rejectedWindows_; }

    /** @brief Sum of the pulses in those discarded windows (how big the bursts were) */
    uint32_t getRejectedPulses() const { return rejectedPulses_; }

    /** @brief Pulses in windows that look like isolated strays: <3 pulses AND below the gear interlock speed */
    uint32_t getStrayPulses() const { return strayPulses_; }

    // ---- Distance counters (NVS "speed", keys "odo_mm" / "trip_mm") ---------

    /** @brief Exact total odometer in millimetres — the authoritative counter */
    uint64_t getOdoMm() const { return odoMm_; }

    /** @brief Exact trip distance in millimetres — the authoritative counter */
    uint64_t getTripMm() const { return tripMm_; }

    /** @brief Total odometer in km — a PRESENTATION of getOdoMm(), never read back */
    float getOdoKm() const { return (float)(odoMm_ * 1e-6); }

    /** @brief Trip distance in km — a PRESENTATION of getTripMm(), never read back */
    float getTripKm() const { return (float)(tripMm_ * 1e-6); }

    /** @brief Zero the TRIP counter and persist immediately. The odometer is untouched. */
    void resetTrip();

    /**
     * @brief Flush both counters to NVS now (ignition OFF / fail-safe / trip reset).
     * A no-op — no NVS open, no log — when nothing has changed since the last successful write.
     */
    void persistDistance();

    // ---- Calibration (NVS "speed") -----------------------------------------

    uint16_t getPulsesPerRev() const { return pulsesPerRev_; }
    float getWheelCircumferenceMm() const { return wheelCircumferenceMm_; }

    /** @brief Set pulses per revolution; persists immediately. False if out of range. */
    bool setPulsesPerRev(int32_t ppr);

    /** @brief Set wheel circumference in mm; persists immediately. False if out of range. */
    bool setWheelCircumferenceMm(float mm);

private:
    /** @brief distance_per_pulse = circumference / pulses_per_rev, recomputed on calibration change */
    void recomputeDistancePerPulse();

    /**
     * @brief All the speed/distance/health work for one sample window.
     * Split out of update() purely so the diagnostics log below runs on EVERY exit path
     * (burst rejection, counted pulses, silence) instead of being duplicated at each return.
     */
    void processWindow(uint32_t now, uint32_t dtMs, uint32_t delta);

    /**
     * @brief Emit the noise line at most once per 2 s, and only when there is something to say:
     * edges were counted in the latest window, or a window was rejected since the last line.
     * Diagnostics only — it changes no state a consumer can see.
     */
    void maybeLogWindow(uint32_t now);

    pcnt_unit_handle_t    unit_;
    pcnt_channel_handle_t channel_;
    bool     initialized_;

    // Calibration
    uint16_t pulsesPerRev_;
    float    wheelCircumferenceMm_;
    float    distancePerPulseMm_;

    // Sampling state
    uint32_t lastSampleMs_;
    uint32_t lastPulseMs_;       // millis() of the last window that saw edges
    uint32_t lastTotal_;         // pulse total at the previous sample
    uint32_t pulseTotal_;        // running total since begin()
    float    speedMs_;
    float    lastMovingSpeedMs_; // last non-zero speed, for the decel plausibility check

    // Distance counters (exact millimetres — never decremented, never zeroed except tripMm_)
    uint64_t odoMm_;             // total odometer, vehicle lifetime
    uint64_t tripMm_;            // resettable trip distance
    uint64_t lastOdoWriteMm_;    // odoMm_ at the last write ATTEMPT (the 1 km write trigger)
    bool     distanceDirty_;     // true when RAM differs from what NVS holds — the ONLY thing
                                 // that lets persistDistance() open NVS at all. Without it a
                                 // fail-safe on every boot, a link flap, or a servo channel
                                 // jittering at an ignition band edge would each rewrite
                                 // unchanged values (the last case at the 25 Hz frame rate).

    // Noise diagnostics — counters only, never consumed by the speed/distance maths.
    uint32_t rawPulsesLastWindow_; // PCNT delta of the latest window, before any guard
    uint32_t rejectedWindows_;     // windows thrown away by the burst guard
    uint32_t rejectedPulses_;      // pulses contained in those thrown-away windows
    uint32_t strayPulses_;         // pulses in "isolated stray" windows (see getStrayPulses())
    uint32_t lastNoiseLogMs_;      // millis() of the last [SPEED] window line (2 s rate limit)
    bool     rejectedSinceLog_;    // a rejection happened since the last log — force one line

    // Health
    bool     everPulsed_;
    bool     suspicious_;
    bool     staleHandled_;      // stale transition already evaluated for this silence
};

#endif // SPEED_SENSOR_H
