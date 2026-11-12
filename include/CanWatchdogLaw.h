#ifndef CAN_WATCHDOG_LAW_H
#define CAN_WATCHDOG_LAW_H

/**
 * @file CanWatchdogLaw.h
 * @brief OBD-II no-data watchdog decision (header-only, no Arduino includes; native-tested in
 *        test/test_can_watchdog). Change gate-obd-watchdog-and-pull-up-vesc-rx.
 *
 * The ECU, like the Bosch LWS, is powered from the ACC line (ignition Relay_1): with the ignition
 * OFF the bus is legitimately silent, so the watchdog is gated on the same ignition rule as the
 * LWS silence watchdog (ACC or IGNITION). The silence is counted only from the later of
 *   - the last OBD-II reply (or, before any reply, the last re-init / begin()), and
 *   - the moment the ignition was last switched on,
 * so switching ACC/ON starts a fresh window and stale time spent with the ignition OFF can never
 * trip it at once. The minimum spacing from the last re-init of any kind is kept unchanged.
 * All arithmetic is millis()-wrap safe (unsigned differences only).
 */

#include <stdint.h>

namespace CanWatchdogLaw {

/**
 * @param ignitionOn    ignition ACC or IGNITION (same rule as the LWS watchdog)
 * @param now           millis()
 * @param lastReplyMs   millis() of the last valid OBD-II reply, 0 = none since boot
 * @param lastReinitMs  millis() of begin() / the last re-init of any kind
 * @param ignitionOnMs  millis() of the last ignition off -> on transition
 * @param noDataMs      silence that triggers a re-init (CAN_REINIT_NO_DATA_MS)
 * @param minIntervalMs minimum spacing from the last re-init (CAN_REINIT_MIN_INTERVAL_MS)
 * @param silentMsOut   if non-null, receives the silence counted inside the window (0 when gated)
 * @return true when the OBD-II no-data watchdog shall re-initialize the MCP2515 now
 */
inline bool obdNoDataDue(bool ignitionOn, uint32_t now, uint32_t lastReplyMs, uint32_t lastReinitMs,
                         uint32_t ignitionOnMs, uint32_t noDataMs, uint32_t minIntervalMs,
                         uint32_t* silentMsOut = nullptr) {
    if (silentMsOut) *silentMsOut = 0;
    if (!ignitionOn) return false;
    uint32_t silent = (lastReplyMs != 0) ? (now - lastReplyMs) : (now - lastReinitMs);
    const uint32_t sinceOn = now - ignitionOnMs;
    if (sinceOn < silent) silent = sinceOn;   // the window starts no earlier than ignition-on
    if (silentMsOut) *silentMsOut = silent;
    return silent >= noDataMs && (now - lastReinitMs) >= minIntervalMs;
}

}  // namespace CanWatchdogLaw

#endif  // CAN_WATCHDOG_LAW_H
