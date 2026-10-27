#ifndef LWS_SENSOR_H
#define LWS_SENSOR_H

#include <stdint.h>
#include "Lws305.h"

/**
 * @brief Steering-angle sensor fault token (design D5), in priority order.
 * Telemetry token via lwsFaultToken().
 */
enum class LwsFault : uint8_t {
    NONE,
    ZEROING,         // a zero-set sequence is running
    CAN_DOWN,        // CAN controller not initialised / no sink attached
    TIMEOUT,         // no ACCEPTED frame for the timeout (never received, checksum-only, frozen counter)
    RANGE_LATCHED,   // fault code 9: out-of-range latch, cleared only by restarting the vehicle
    SENSOR_FAULT,    // OK bit clear or any other nonzero fault code
    NOT_CALIBRATED,  // calibrated bit clear (no stored zero)
    ANGLE_INVALID    // angle 0x7FFF
};

/** @brief Telemetry token: none, zeroing, can_down, timeout, range_latched, sensor_fault, not_calibrated, angle_invalid */
const char* lwsFaultToken(LwsFault f);

/**
 * @brief Bosch LWS frame validation, health and counters.
 *
 * Arduino-free: time is always passed in, so the decision logic is unit-tested natively.
 * Fed by CANController (onFrame() for every standard-ID 0x305 frame from every receive
 * path) and evaluated by SteeringController once per loop (evaluate()).
 *
 * A frame is ACCEPTED only if DLC >= LWS_DLC_MIN, the checksum passes and the rolling
 * counter differs from the last accepted frame's. A repeated counter is a counter error
 * (duplicate / frozen sender); a counter jump > 1 is counted as missed frames but the
 * frame is accepted, since every frame carries the absolute angle.
 */
class LwsSensor {
public:
    /**
     * @param timeoutMs     no accepted frame for longer than this -> TIMEOUT (STEER_LWS_TIMEOUT_MS)
     * @param recoverFrames consecutive good accepted frames needed to be healthy (STEER_LWS_RECOVER_FRAMES)
     */
    LwsSensor(uint32_t timeoutMs, uint8_t recoverFrames);

    /** @brief Feed one received 0x305 frame. @return true if it was accepted */
    bool onFrame(const uint8_t* buf, uint8_t len, uint32_t nowMs);

    /**
     * @brief Derive the fault token (D5 priority) and the health flag.
     * Faults take effect immediately; recovery needs recoverFrames consecutive good
     * accepted frames, during which the previous fault token stays reported (the
     * token is never NONE while unhealthy).
     * @param zeroing  a zero-set sequence is running
     * @param canUp    the CAN controller is initialised and this sink is attached
     */
    LwsFault evaluate(uint32_t nowMs, bool zeroing, bool canUp);

    /** @brief Health from the last evaluate(): token NONE for >= recoverFrames consecutive accepted frames */
    bool isHealthy() const { return healthy_; }
    LwsFault fault() const { return fault_; }

    // --- Last accepted frame -------------------------------------------------
    bool hasFrame() const { return haveFrame_; }
    /** @brief Last accepted frame arrived within the timeout */
    bool isFresh(uint32_t nowMs) const { return haveFrame_ && (nowMs - lastMs_) <= timeoutMs_; }
    uint32_t lastFrameMs() const { return lastMs_; }
    const uint8_t* lastFrame() const { return last_; }

    int16_t angleRaw() const { return lwsAngleRaw(last_); }         // sensor sign (+ = left), 0.1°
    bool angleValid() const { return haveFrame_ && lwsAngleValid(last_); }
    /** @brief Firmware position: the NEGATED sensor angle (+ = right), 0.1° */
    int32_t position() const { return -(int32_t)lwsAngleRaw(last_); }
    bool statusOk() const { return lwsOk(last_); }
    bool statusCal() const { return lwsCal(last_); }
    bool statusAdj() const { return lwsAdj(last_); }
    uint8_t faultCode() const { return lwsFault(last_); }
    bool speedValid() const { return haveFrame_ && lwsSpeedValid(last_); }
    /** @brief Signed rotation rate, deg/s, firmware sign (+ = turning right = clockwise) */
    int32_t rateDegS() const { return lwsDirCW(last_) ? (int32_t)lwsSpeedDegS(last_) : -(int32_t)lwsSpeedDegS(last_); }

    // --- Counters (cumulative since boot) --------------------------------------
    uint32_t crcErrors() const { return crcErr_; }
    uint32_t counterErrors() const { return cntErr_; }
    uint32_t missedFrames() const { return miss_; }
    uint32_t acceptedFrames() const { return accepted_; }
    /** @brief Accepted frames in the last completed one-second window */
    uint16_t fps() const { return fps_; }

private:
    const uint32_t timeoutMs_;
    const uint8_t recoverFrames_;

    uint8_t last_[8];
    bool haveFrame_;
    uint32_t lastMs_;

    uint32_t crcErr_;
    uint32_t cntErr_;
    uint32_t miss_;
    uint32_t accepted_;

    uint32_t windowStartMs_;
    uint16_t windowCount_;
    uint16_t fps_;
    bool windowStarted_;

    uint8_t goodStreak_;       // consecutive accepted frames whose own token is NONE
    LwsFault fault_;
    bool healthy_;

    /** @brief Token derived from the status bits / angle of the last accepted frame */
    LwsFault frameFault() const;
    void rollWindow(uint32_t nowMs);
};

#endif // LWS_SENSOR_H
