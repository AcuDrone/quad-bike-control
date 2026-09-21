#ifndef SPEED_SENSOR_H
#define SPEED_SENSOR_H

#include <Arduino.h>
#include "Constants.h"
#include "driver/pulse_cnt.h"

/**
 * @brief Vehicle speed from the 3-pin 12V hall sensor on the X2 opto input.
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
 * This class is the single owner of the NVS namespace `"speed"`, which now holds
 * only the calibration (pulses/rev + wheel circumference). The max-speed limiter
 * has NO persisted configuration: its ceiling comes solely from the autopilot's
 * SPEED_MAX parameter and is applied in `VehicleController`.
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

    // Health
    bool     everPulsed_;
    bool     suspicious_;
    bool     staleHandled_;      // stale transition already evaluated for this silence
};

#endif // SPEED_SENSOR_H
