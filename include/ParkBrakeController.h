#ifndef PARK_BRAKE_CONTROLLER_H
#define PARK_BRAKE_CONTROLLER_H

#include <Arduino.h>
#include "Constants.h"
#include "BTS7960Controller.h"
#include "ParkBrakeLaw.h"

/**
 * @brief Aztec parking-brake actuator on X6 (change add-aztec-parking-brake).
 *
 * A self-locking linear actuator with built-in limit switches pulls the hand-brake cable through
 * a BTS7960 (PIN_PARK_BRAKE_RPWM / _LPWM, LEDC 6 / 7, shared LEDC_TIMER_1). There is no position
 * feedback, so every movement is TIMED: full duty (PARK_BRAKE_DRIVE_DUTY) for travel_ms +
 * PARK_BRAKE_TRAVEL_MARGIN_MS, then both PWM 0 and no hold drive.
 *
 * Owns the run timer, the PARK_BRAKE_REVERSE_DWELL_MS reverse dwell, the state and NVS
 * "pbrake" (state / travel_ms / thr_pass / invert). The state is written as the moving value
 * BEFORE the bridge is driven and as the settled value AFTER it stops, so a reset mid-run comes
 * back UNKNOWN. Nothing is driven at boot, and nothing here ever starts a run by itself: runs
 * start only through request(), called by VehicleController::commandParkBrake() and its
 * drive-away release. The speed interlock, the drive-away trigger and the throttle hold are
 * VehicleController's policy; the decisions live in include/ParkBrakeLaw.h.
 */
class ParkBrakeController {
public:
    explicit ParkBrakeController(BTS7960Controller& driver);

    /**
     * @brief Load NVS "pbrake" (state decode, travel / release point or default with an
     * out-of-range log, invert). Call once from setup(), after the driver's begin().
     * Never drives the bridge.
     */
    void begin();

    /** @brief Outcome of request(): result code, and for a release whether the moving state was not saved */
    struct Outcome {
        ParkBrakeResult result;
        bool stateNotSaved;   // RELEASE started although the moving state could not be written
    };

    /**
     * @brief Execute an APPLY / RELEASE whose speed check the caller already made (ParkBrakeLaw.h
     * parkBrakeTransition()). APPLY is refused (REFUSED_NVS) when the moving state cannot be
     * saved; RELEASE runs anyway and reports stateNotSaved.
     */
    Outcome request(ParkBrakeRequest req, ParkBrakeSpeedCheck speed);

    /** @brief Advance the run: reverse dwell → drive → stop + persist the settled state. Every loop. */
    void update();

    // ---- State ----
    ParkBrakeState getState() const { return state_; }
    bool isDriverOk() const { return driver_.isInitialized(); }
    /** @brief ms left in the current run (reverse dwell + travel + margin), 0 when settled */
    uint32_t getRemainingMs() const;
    /** @brief ms into the release DRIVE (0 during the reverse dwell and when not releasing) */
    uint32_t getReleaseElapsedMs() const;
    /** @brief Throttle release point of the current release run, ms into the drive (ParkBrakeLaw.h) */
    uint32_t getThrPassMs() const;

    // ---- Settings in use ----
    uint16_t getTravelMs() const { return travelMs_; }
    uint8_t getThrPassPct() const { return thrPassPct_; }
    bool isInverted() const { return invert_; }

    /** @brief Validate 1000–5000, persist first, then apply from the next run. False = unchanged. */
    bool setTravelMs(uint16_t ms);
    /** @brief Validate 20–100, persist first, then apply from the next release run. False = unchanged. */
    bool setThrPassPct(uint8_t pct);

    enum class InvertResult : uint8_t { OK, REFUSED_MOVING, NVS_FAILED };
    /**
     * @brief Refused while a run is in progress. Persists `invert`, then sets and persists the
     * state UNKNOWN (the old state was measured with the old polarity).
     */
    InvertResult setInvert(bool invert);

private:
    enum class Phase : uint8_t { IDLE, DWELL, DRIVE };

    BTS7960Controller& driver_;
    ParkBrakeState state_;
    Phase phase_;
    ParkBrakeRequest runDir_;     // direction of the current run
    uint32_t phaseStartMs_;       // millis() the dwell / drive started
    uint32_t runMs_;              // drive length of the current run (captured at its start)
    uint32_t runThrPassMs_;       // throttle release point of the current run (captured at its start)
    uint16_t travelMs_;
    uint8_t thrPassPct_;
    bool invert_;

    bool persistState(ParkBrakeState s);
    void startDrive(uint32_t now);
};

#endif // PARK_BRAKE_CONTROLLER_H
