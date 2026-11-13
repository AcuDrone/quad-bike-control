// BrakeLaw.h — brake full travel time / hold drive arithmetic and validation
// (change add-brake-tuning-card).
//
// NO Arduino / Constants.h dependency: this header is compiled by the native unit
// tests (test/test_brake_law). VehicleController owns the brake estimate, the timing
// and the per-vehicle values (NVS "brake"); this file is only the arithmetic.
// The literals below MUST match include/Constants.h:
//   BRAKE_TRAVEL_MS_MIN 1000, BRAKE_TRAVEL_MS_MAX 3000, BRAKE_FULL_TRAVEL_TIME 1500,
//   BRAKE_HOLD_PWM_MIN 0, BRAKE_HOLD_PWM_MAX 100, BRAKE_HOLD_SPEED 30.
#pragma once

#include <stdint.h>

static const int32_t BRAKE_LAW_TRAVEL_MS_MIN     = 1000;   // BRAKE_TRAVEL_MS_MIN
static const int32_t BRAKE_LAW_TRAVEL_MS_MAX     = 3000;   // BRAKE_TRAVEL_MS_MAX
static const int32_t BRAKE_LAW_TRAVEL_MS_DEFAULT = 1500;   // BRAKE_FULL_TRAVEL_TIME
static const int32_t BRAKE_LAW_HOLD_PWM_MIN      = 0;      // BRAKE_HOLD_PWM_MIN
static const int32_t BRAKE_LAW_HOLD_PWM_MAX      = 100;    // BRAKE_HOLD_PWM_MAX
static const int32_t BRAKE_LAW_HOLD_PWM_DEFAULT  = 30;     // BRAKE_HOLD_SPEED

// Estimate rate at full drive (±255): 100 % over the full travel time, in %/ms.
// The caller multiplies by |speed| / 255. A non-positive travel time falls back to the default
// (never a division by zero).
inline float brakeRatePctPerMs(int32_t travelMs) {
  if (travelMs <= 0) travelMs = BRAKE_LAW_TRAVEL_MS_DEFAULT;
  return 100.0f / (float)travelMs;
}

inline bool brakeTravelMsValid(int32_t ms) {
  return ms >= BRAKE_LAW_TRAVEL_MS_MIN && ms <= BRAKE_LAW_TRAVEL_MS_MAX;
}

inline bool brakeHoldPwmValid(int32_t pwm) {
  return pwm >= BRAKE_LAW_HOLD_PWM_MIN && pwm <= BRAKE_LAW_HOLD_PWM_MAX;
}

// NVS load helpers: a stored value outside its range is replaced by the default
// (the caller logs it and never rewrites the stored value).
inline int32_t brakeTravelMsOrDefault(int32_t stored) {
  return brakeTravelMsValid(stored) ? stored : BRAKE_LAW_TRAVEL_MS_DEFAULT;
}

inline int32_t brakeHoldPwmOrDefault(int32_t stored) {
  return brakeHoldPwmValid(stored) ? stored : BRAKE_LAW_HOLD_PWM_DEFAULT;
}
