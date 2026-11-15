// CrankLaw.h — pre-crank dwell validation, the armed-crank decision and the validated
// crank RPM (change add-precrank-dwell-setting).
//
// NO Arduino / Constants.h dependency: this header is compiled by the native unit
// tests (test/test_crank_law). VehicleController owns the per-vehicle dwell (NVS
// "crank"/"dwell_ms") and the ignition-OFF RPM clear; RelayController calls
// crankArmedDecide() from update(). This file is only the arithmetic and decisions.
// The literals below MUST match include/Constants.h:
//   CRANK_DWELL_MS_MIN 1000, CRANK_DWELL_MS_MAX 8000, ACC_PRECRANK_DWELL_MS 3000.
#pragma once

#include <stdint.h>

static const uint32_t CRANK_LAW_DWELL_MS_MIN     = 1000;   // CRANK_DWELL_MS_MIN
static const uint32_t CRANK_LAW_DWELL_MS_MAX     = 8000;   // CRANK_DWELL_MS_MAX
static const uint32_t CRANK_LAW_DWELL_MS_DEFAULT = 3000;   // ACC_PRECRANK_DWELL_MS

inline bool crankDwellMsValid(uint32_t ms) {
  return ms >= CRANK_LAW_DWELL_MS_MIN && ms <= CRANK_LAW_DWELL_MS_MAX;
}

// NVS load / setter helper: a value outside the range is replaced by the default
// (the caller logs it and never rewrites the stored value).
inline uint16_t crankDwellMsOrDefault(uint32_t stored) {
  return (uint16_t)(crankDwellMsValid(stored) ? stored : CRANK_LAW_DWELL_MS_DEFAULT);
}

// True once the ignition / ECU line has been powered for at least dwellMs.
// ignitionHighSinceMs == 0 is the "line OFF" sentinel → never elapsed.
// Unsigned subtraction, so a millis() wrap between the two stamps is harmless.
inline bool crankDwellElapsed(uint32_t ignitionHighSinceMs, uint32_t nowMs, uint32_t dwellMs) {
  if (ignitionHighSinceMs == 0) return false;
  return (uint32_t)(nowMs - ignitionHighSinceMs) >= dwellMs;
}

enum class CrankArmedAction : uint8_t {
  WAIT,      // keep holding ACC until the dwell elapses
  SUPPRESS,  // engine already running — no crank, settle in IGNITION
  ENGAGE     // dwell satisfied — engage the starter
};

// Decision for an armed (requestCrank()) crank that is not yet cranking.
// The running check comes first, exactly as before the change.
inline CrankArmedAction crankArmedDecide(uint16_t rpm, uint16_t runningThreshold,
                                         uint32_t ignitionHighSinceMs, uint32_t nowMs,
                                         uint32_t dwellMs) {
  if (rpm >= runningThreshold) return CrankArmedAction::SUPPRESS;
  if (crankDwellElapsed(ignitionHighSinceMs, nowMs, dwellMs)) return CrankArmedAction::ENGAGE;
  return CrankArmedAction::WAIT;
}

// RPM handed to the crank logic: only a value the ECU reported while the CAN data is
// valid; 0 otherwise (a stale stored value must never suppress or cut a crank).
inline uint16_t crankRpm(bool valid, uint16_t rpm) {
  return valid ? rpm : (uint16_t)0;
}

// Edge of the applied relay ignition state into OFF (clears the stored CAN engine RPM).
inline bool ignitionWentOff(bool wasOn, bool isOn) {
  return wasOn && !isOn;
}
