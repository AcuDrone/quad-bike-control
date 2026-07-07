#ifndef THROTTLE_CONTROLLER_H
#define THROTTLE_CONTROLLER_H

#include <Arduino.h>
#include "Constants.h"
#include "ServoController.h"

/**
 * @brief Throttle servo controller with runtime microsecond calibration.
 *
 * Owns the throttle ServoController and maps 0..100% throttle onto a calibrated
 * [idleUs, fullUs] pulse-width window. The idle/full endpoints are persisted in
 * NVS ("throttle" namespace) and are editable at runtime via the web calibration
 * flow; the compile-time THROTTLE_DEFAULT_*_US values are only fallbacks.
 *
 * Calibration is a session. beginCalibration() suspends normal throttle output so
 * previewed values are not overwritten by MAVLink or the gear-boost PID; jogIdle()/
 * jogFull() drive the servo live to the pending endpoint (the servo holds the last
 * previewed value); saveCalibration() validates and persists; cancelCalibration()
 * reverts. Entry is refused while the engine is running.
 */
class ThrottleController {
public:
    ThrottleController();

    /**
     * @brief Initialize the throttle servo and load calibration from NVS.
     * Moves the servo to the calibrated idle position.
     */
    bool begin(gpio_num_t pin, uint8_t channel);

    // ---- Normal control (no-ops while a calibration session is active) ----
    void setThrottlePercent(float pct);   // 0..100% -> [idleUs_, fullUs_]
    void setThrottleUs(uint16_t us);       // direct µs (clamped to servo range) — used by the gear-boost PID
    void idle();                           // move to idleUs_

    // ---- Calibration session ----
    bool beginCalibration(bool engineRunning);  // false (refused) if engineRunning
    void jogIdle(uint16_t us);                   // set pending idle + drive servo live
    void jogFull(uint16_t us);                   // set pending full + drive servo live
    bool saveCalibration(String& err);           // validate + persist pending -> active; false + err on reject
    void cancelCalibration();                    // discard pending, return servo to idle

    // ---- Accessors ----
    uint16_t getIdleUs() const { return idleUs_; }
    uint16_t getFullUs() const { return fullUs_; }
    uint16_t getCurrentUs() const { return servo_.getMicroseconds(); }
    bool isCalibrating() const { return calibrating_; }
    uint16_t percentToUs(float pct) const;

private:
    ServoController servo_;
    uint16_t idleUs_;          // active calibration endpoints
    uint16_t fullUs_;
    uint16_t pendingIdleUs_;   // working endpoints during a calibration session
    uint16_t pendingFullUs_;
    bool calibrating_;

    void loadCalibration();
    static uint16_t clampToServoRange(int32_t us);
};

#endif // THROTTLE_CONTROLLER_H
