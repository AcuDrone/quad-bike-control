// WasherLaw.h — headlight washer ("омивач фар", Relay_8) decisions: pulse-length
// validation, the start interlock and the wrap-safe pulse timer
// (change add-headlight-washer-relay).
//
// NO Arduino / Constants.h dependency: this header is compiled by the native unit
// tests (test/test_relay_board). VehicleController owns the per-vehicle settings
// (NVS "washer"/"pulse_ms" and "washer"/"fitted"); RelayController owns the pulse and
// calls washerStartDecide(), washerExpired() and washerRemainingMs(). This file is only
// the arithmetic and decisions.
// The literals below MUST match include/Constants.h (static_assert in RelayController.cpp):
//   WASHER_PULSE_MS_MIN 2000, WASHER_PULSE_MS_MAX 20000, WASHER_PULSE_MS_DEFAULT 3000.
#pragma once

#include <stdint.h>

static const uint32_t WASHER_LAW_PULSE_MS_MIN     = 2000;    // WASHER_PULSE_MS_MIN
static const uint32_t WASHER_LAW_PULSE_MS_MAX     = 20000;   // WASHER_PULSE_MS_MAX
static const uint32_t WASHER_LAW_PULSE_MS_DEFAULT = 3000;    // WASHER_PULSE_MS_DEFAULT

inline bool washerPulseMsValid(uint32_t ms) {
  return ms >= WASHER_LAW_PULSE_MS_MIN && ms <= WASHER_LAW_PULSE_MS_MAX;
}

// NVS load / setter helper: a value outside the range is replaced by the default
// (the caller logs it and never rewrites the stored value).
inline uint16_t washerPulseMsOrDefault(uint32_t stored) {
  return (uint16_t)(washerPulseMsValid(stored) ? stored : WASHER_LAW_PULSE_MS_DEFAULT);
}

enum class WasherVerdict : uint8_t {
  START,                 // energize Relay_8 for the pulse length
  REFUSED_NOT_FITTED,    // "washer not fitted - enable it in Calibration"
  REFUSED_UNAVAILABLE,   // "relay board unavailable"
  REFUSED_IGNITION_OFF,  // "ignition is OFF"
  REFUSED_STARTING       // "engine start in progress" (crank armed or CRANKING)
};

// Ignition state as RelayController::IgnitionState's ordinal: 0 OFF, 1 ACC, 2 IGNITION,
// 3 CRANKING. Checked in this order: fitted switch, relay board, ignition OFF, engine start.
// "Ignition on" = ACC (engine off) or IGNITION (engine running, or after a start attempt
// that ended); a crank armed (pre-crank dwell, reported as ACC) or CRANKING refuses.
inline WasherVerdict washerStartDecide(bool fitted, bool available, uint8_t ignState,
                                       bool crankArmed, bool cranking) {
  if (!fitted) return WasherVerdict::REFUSED_NOT_FITTED;
  if (!available) return WasherVerdict::REFUSED_UNAVAILABLE;
  if (ignState == 0) return WasherVerdict::REFUSED_IGNITION_OFF;
  if (ignState >= 3 || crankArmed || cranking) return WasherVerdict::REFUSED_STARTING;
  return WasherVerdict::START;
}

// True once the pulse has run for at least limitMs since sinceMs. Unsigned subtraction,
// so a millis() wrap between the two stamps is harmless.
inline bool washerExpired(uint32_t sinceMs, uint32_t nowMs, uint32_t limitMs) {
  return (uint32_t)(nowMs - sinceMs) >= limitMs;
}

// Remaining pulse time, ms; 0 once expired (never underflows).
inline uint32_t washerRemainingMs(uint32_t sinceMs, uint32_t nowMs, uint32_t limitMs) {
  const uint32_t elapsed = (uint32_t)(nowMs - sinceMs);
  return elapsed >= limitMs ? 0u : (uint32_t)(limitMs - elapsed);
}
