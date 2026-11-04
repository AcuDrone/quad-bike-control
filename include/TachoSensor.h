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
 * TACHO_MEDIAN_PERIODS + 1 edges. It drops any edge closer than TACHO_MIN_PERIOD_US to the last
 * accepted one (absolute floor) and, once locked, any edge closer than TACHO_LOCK_FRACTION × the
 * current median period (lock window, fw 1.5.10: ignition noise / ringing on the opto edges);
 * dropped edges are counted. `update()` copies the buffer inside a short critical section, takes
 * the MEDIAN of the last up to TACHO_MEDIAN_PERIODS periods (mean below 3 periods), derives Hz and
 * RPM (`Hz × 60 / ppr`) and writes the lock threshold back for the ISR. A reading above
 * TACHO_MAX_RPM is discarded (the previous one is kept). After TACHO_TIMEOUT_MS without an
 * accepted edge Hz / RPM read 0 and the buffer and the lock are cleared.
 *
 * Diagnostics (field tech, no scope needed): the last 8 raw accepted periods, and over a rolling
 * TACHO_DIAG_WINDOW_MS window the min / max period and the accepted / rejected edges per second.
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

    /** @brief Edges rejected by the floor / lock window since boot (diagnostics) */
    uint32_t getRejectedTotal() const { return rejTotal_; }

    /** @brief Robust (median) period estimate in µs, 0 when not live (diagnostics) */
    uint32_t getMedianPeriodUs() const { return medianUs_; }

    /** @brief Number of raw periods held for diagnostics (0..TACHO_DIAG_PERIODS) */
    uint8_t getDiagPeriodCount() const { return diagN_; }

    /** @brief Raw accepted period i (µs), oldest first, i < getDiagPeriodCount() */
    uint32_t getDiagPeriodUs(uint8_t i) const { return i < diagN_ ? diagUs_[i] : 0; }

    /** @brief Last raw periods as "42.9 43.1 ..." (ms, one decimal, oldest first) */
    void formatDiagPeriods(char* out, size_t outLen) const {
        tachoFormatPeriodsMs(diagUs_, diagN_, out, outLen);
    }

    /** @brief Shortest / longest accepted period in the last diagnostics window, ms (0 = none) */
    float getWindowMinMs() const { return winMinMs_; }
    float getWindowMaxMs() const { return winMaxMs_; }

    /** @brief Accepted / rejected edges per second over the last diagnostics window */
    uint16_t getAcceptedPerSec() const { return accPerSec_; }
    uint16_t getRejectedPerSec() const { return rejPerSec_; }

    /** @brief Raw periods kept for diagnostics */
    static constexpr uint8_t TACHO_DIAG_PERIODS = 8;

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
    uint32_t medianUs_;     // robust period estimate (0 = not live)

    // Diagnostics
    uint32_t rejTotal_;                      // last copied ISR rejected-edge counter
    uint32_t diagUs_[TACHO_DIAG_PERIODS];    // last raw accepted periods, oldest first
    uint8_t  diagN_;
    uint32_t winStartMs_;                    // current window start
    uint32_t winAccStart_;                   // edgeTotal_ at the window start
    uint32_t winRejStart_;                   // rejTotal_ at the window start
    uint32_t winMinUs_;                      // running min / max period in the current window
    uint32_t winMaxUs_;
    float    winMinMs_;                      // published values of the last full window
    float    winMaxMs_;
    uint16_t accPerSec_;
    uint16_t rejPerSec_;

    void pushDiagPeriod(uint32_t us);
    void closeDiagWindow(uint32_t nowMs);
};

#endif // TACHO_SENSOR_H
