#ifndef TACHO_SENSOR_H
#define TACHO_SENSOR_H

#include <Arduino.h>
#include "Constants.h"
#include "TachoLaw.h"

/**
 * @brief Engine RPM from the ECU tachometer output on PIN_TACHO (GPIO14, X3 Hall_1 A)
 * through an external PC817 emitter follower (change add-tacho-rpm-input, aztec).
 *
 * An IRAM ISR on the RISING edge stores `esp_timer_get_time()` (µs) into a ring buffer of
 * TACHO_AVG_PERIODS + 1 edges, dropping any edge closer than TACHO_MIN_PERIOD_US to the last
 * accepted one (software glitch filter). `update()` copies the buffer inside a short critical
 * section, averages the last up to TACHO_AVG_PERIODS periods and derives Hz and RPM
 * (`Hz × 60 / ppr`). A reading above TACHO_MAX_RPM is discarded (the previous one is kept).
 * After TACHO_TIMEOUT_MS without an accepted edge Hz / RPM read 0 and the buffer is cleared.
 *
 * "Fitted" = an accepted edge since boot OR a `ppr` key in NVS "tacho"; it latches for the boot.
 * Only ONE instance may exist (the ISR state is file-static); VehicleController owns it.
 *
 * This class is the single owner of the NVS namespace "tacho" (key "ppr", float).
 */
class TachoSensor {
public:
    TachoSensor();

    /** @brief Load ppr from NVS, configure the pin and attach the ISR. Call once from setup(). */
    void begin();

    /** @brief Recompute Hz / RPM from the edge buffer; never blocks. Call every loop. */
    void update();

    /** @brief Averaged edge frequency (Hz), 0 when timed out */
    float getHz() const { return hz_; }

    /** @brief Tacho-computed RPM, 0 when timed out */
    uint16_t getRpm() const { return rpm_; }

    /** @brief Pulses per revolution in effect */
    float getPpr() const { return ppr_; }

    /** @brief True while an accepted edge arrived within TACHO_TIMEOUT_MS */
    bool isLive() const { return live_; }

    /** @brief Pulse seen since boot, or a ppr key stored in NVS (latched for the boot) */
    bool isFitted() const { return fitted_; }

    /** @brief True when a ppr key is stored in NVS (calibrated or set) */
    bool isPprStored() const { return pprStored_; }

    /** @brief millis() of the last accepted edge (the RPM sample timestamp) */
    uint32_t lastEdgeMs() const { return lastEdgeMs_; }

    /** @brief Accepted edges since boot (diagnostics) */
    uint32_t getEdgeTotal() const { return edgeTotal_; }

    /** @brief Persist then apply an allowed ppr. False (old ppr kept) if not allowed or NVS fails. */
    bool setPpr(float ppr);

    /** @brief Remove the stored ppr key and apply TACHO_PPR_DEFAULT. False if NVS fails. */
    bool resetPpr();

private:
    bool     initialized_;
    float    ppr_;
    bool     pprStored_;
    bool     fitted_;
    bool     live_;
    bool     cleared_;      // buffer already cleared for the current silence
    float    hz_;
    uint16_t rpm_;
    uint32_t edgeTotal_;    // last copied ISR edge counter
    uint32_t lastEdgeMs_;
};

#endif // TACHO_SENSOR_H
