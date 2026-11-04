// TachoLaw.h — engine RPM from the ECU tachometer output (aztec, change add-tacho-rpm-input).
//
// NO Arduino / Constants.h dependency: this header is compiled by the native unit tests
// (test/test_tacho). Callers pass the constants in.
//
//  period : mean of the last up to N periods between accepted RISING edges (µs timestamps)
//  Hz     = 1e6 / mean_period_us
//  RPM    = Hz × 60 / ppr                      ppr ∈ {0.5, 1, 2, 3, 4}
//  cal    : raw = Hz × 60 / reference RPM, snapped to the nearest allowed ppr; refused when no
//           pulses, the reference is outside [refMin, refMax], or |raw − snapped| > tol × snapped
//  source : live TACHO → CAN (valid, rpm > 0) → fitted TACHO 0 → CAN (valid) → NONE
#pragma once

#include <math.h>
#include <stdint.h>

/** @brief Allowed pulses-per-revolution values, ascending */
static constexpr float TACHO_PPR_ALLOWED[] = {0.5f, 1.0f, 2.0f, 3.0f, 4.0f};
static constexpr uint8_t TACHO_PPR_ALLOWED_COUNT =
    (uint8_t)(sizeof(TACHO_PPR_ALLOWED) / sizeof(TACHO_PPR_ALLOWED[0]));

/** @brief Frequency from a period in microseconds (0 for a non-positive period) */
inline float tachoHzFromPeriodUs(float periodUs) {
  return (periodUs > 0.0f && !isnan(periodUs)) ? (1.0e6f / periodUs) : 0.0f;
}

/** @brief RPM from frequency and pulses per revolution (0 when either is unusable) */
inline float tachoRpm(float hz, float ppr) {
  if (!(hz > 0.0f) || !(ppr > 0.0f)) return 0.0f;
  return hz * 60.0f / ppr;
}

/**
 * @brief Mean period over the last up to `maxPeriods` periods.
 * @param edgesUs chronological edge timestamps (oldest first), µs, uint32 wrap-safe
 * @param count number of valid timestamps in edgesUs
 * @return mean period in µs, 0 when fewer than two edges are available
 */
inline float tachoMeanPeriod(const uint32_t* edgesUs, uint8_t count, uint8_t maxPeriods) {
  if (edgesUs == nullptr || count < 2 || maxPeriods == 0) return 0.0f;
  uint8_t periods = (uint8_t)(count - 1);
  if (periods > maxPeriods) periods = maxPeriods;
  const uint32_t newest = edgesUs[count - 1];
  const uint32_t oldest = edgesUs[count - 1 - periods];
  const uint32_t span = newest - oldest;   // unsigned: correct across a micros() wrap
  return (float)span / (float)periods;
}

/** @brief True when the RPM is a believable engine speed (finite, ≥ 0, ≤ maxRpm) */
inline bool tachoPlausible(float rpm, float maxRpm) {
  return !isnan(rpm) && !isinf(rpm) && rpm >= 0.0f && rpm <= maxRpm;
}

/** @brief Wrap-safe timeout: true when at least timeoutMs elapsed since lastEdgeMs */
inline bool tachoTimedOut(uint32_t nowMs, uint32_t lastEdgeMs, uint32_t timeoutMs) {
  return (uint32_t)(nowMs - lastEdgeMs) >= timeoutMs;
}

/** @brief True when ppr is one of the allowed values */
inline bool tachoPprAllowed(float ppr) {
  if (isnan(ppr)) return false;
  for (uint8_t i = 0; i < TACHO_PPR_ALLOWED_COUNT; i++) {
    if (fabsf(ppr - TACHO_PPR_ALLOWED[i]) < 1.0e-3f) return true;
  }
  return false;
}

/** @brief Nearest allowed ppr to a raw ratio (ties go to the smaller value) */
inline float tachoSnapPpr(float raw) {
  float best = TACHO_PPR_ALLOWED[0];
  if (isnan(raw)) return best;
  float bestDiff = fabsf(raw - best);
  for (uint8_t i = 1; i < TACHO_PPR_ALLOWED_COUNT; i++) {
    const float d = fabsf(raw - TACHO_PPR_ALLOWED[i]);
    if (d < bestDiff) {
      bestDiff = d;
      best = TACHO_PPR_ALLOWED[i];
    }
  }
  return best;
}

/** @brief True when raw lies within ±tol × snapped of its snapped value */
inline bool tachoSnapWithin(float raw, float snapped, float tol) {
  return !isnan(raw) && fabsf(raw - snapped) <= tol * snapped;
}

/** @brief Calibration tunables (TACHO_CAL_* in Constants.h) */
struct TachoCalParams {
  float refMinRpm;   // TACHO_CAL_REF_MIN_RPM
  float refMaxRpm;   // TACHO_CAL_REF_MAX_RPM
  float tolerance;   // TACHO_CAL_TOLERANCE
};

/** @brief Outcome of tachoComputeCal() */
enum class TachoCalResult : uint8_t {
  OK,
  NO_PULSES,          // tacho timed out (Hz 0): nothing to calibrate against
  REF_OUT_OF_RANGE,   // reference NaN or outside [refMinRpm, refMaxRpm]
  RATIO_OFF           // raw ratio more than tolerance away from the nearest allowed ppr
};

/**
 * @brief Compute ppr from the CURRENT averaged frequency and a reference RPM.
 * @param rawOut  written whenever it was computed (RATIO_OFF and OK), for the reply text
 * @param pprOut  written only on OK
 */
inline TachoCalResult tachoComputeCal(float hz, float refRpm, const TachoCalParams& p,
                                      float& rawOut, float& pprOut) {
  if (!(hz > 0.0f)) return TachoCalResult::NO_PULSES;
  if (isnan(refRpm) || refRpm < p.refMinRpm || refRpm > p.refMaxRpm) {
    return TachoCalResult::REF_OUT_OF_RANGE;
  }
  const float raw = hz * 60.0f / refRpm;
  rawOut = raw;
  const float snapped = tachoSnapPpr(raw);
  if (!tachoSnapWithin(raw, snapped, p.tolerance)) return TachoCalResult::RATIO_OFF;
  pprOut = snapped;
  return TachoCalResult::OK;
}

/** @brief Where the effective engine RPM came from */
enum class RpmSource : uint8_t { NONE = 0, TACHO = 1, CAN = 2 };

/** @brief Telemetry token for a source ("TACHO" / "CAN" / "NONE"), static storage */
inline const char* rpmSourceName(RpmSource s) {
  switch (s) {
    case RpmSource::TACHO: return "TACHO";
    case RpmSource::CAN:   return "CAN";
    default:               return "NONE";
  }
}

/** @brief The one engine RPM every consumer reads */
struct EffectiveRpm {
  uint16_t  rpm;       // 0 when invalid
  bool      valid;
  RpmSource source;
  uint32_t  sampleMs;  // ms timestamp of the sample (last tacho edge or CAN lastUpdateTime)
};

/**
 * @brief Effective RPM selection.
 *  1. tacho fitted and live      → TACHO
 *  2. CAN valid and CAN RPM > 0  → CAN (a broken tacho wire must never hide a running engine)
 *  3. tacho fitted               → 0, TACHO, valid (a genuine stopped engine)
 *  4. CAN valid                  → CAN; else NONE (invalid, 0)
 */
inline EffectiveRpm tachoSelectRpm(bool fitted, bool live, uint16_t tachoRpmVal, uint32_t tachoMs,
                                   bool canValid, uint16_t canRpm, uint32_t canMs) {
  if (fitted && live) return EffectiveRpm{tachoRpmVal, true, RpmSource::TACHO, tachoMs};
  if (canValid && canRpm > 0) return EffectiveRpm{canRpm, true, RpmSource::CAN, canMs};
  if (fitted) return EffectiveRpm{0, true, RpmSource::TACHO, tachoMs};
  if (canValid) return EffectiveRpm{canRpm, true, RpmSource::CAN, canMs};
  return EffectiveRpm{0, false, RpmSource::NONE, 0};
}
