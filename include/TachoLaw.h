// TachoLaw.h — engine RPM from the ECU tachometer output (aztec, change add-tacho-rpm-input).
//
// NO Arduino / Constants.h dependency: this header is compiled by the native unit tests
// (test/test_tacho). Callers pass the constants in.
//
//  edges  : an edge closer than floorUs to the last accepted one is dropped; while locked, an
//           edge closer than fraction × the robust period estimate is dropped too (lock window,
//           fw 1.5.10 — ignition noise / ringing on the opto edges)
//  period : MEDIAN of the last up to N periods between accepted RISING edges (µs timestamps);
//           mean when fewer than 3 periods exist (fw 1.5.10; the mean of 4 was fooled by spurious edges)
//  Hz     = 1e6 / period_us
//  RPM    = Hz × 60 / ppr                      ppr ∈ {0.5, 1, 2, 3, 4}
//  cal    : raw = Hz × 60 / reference RPM, snapped to the nearest allowed ppr; refused when no
//           pulses, the reference is outside [refMin, refMax], or |raw − snapped| > tol × snapped
//  source : live TACHO → CAN (valid, rpm > 0) → fitted TACHO 0 → CAN (valid) → NONE
#pragma once

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stddef.h>

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

/** @brief Largest period count tachoMedianPeriod() handles (larger requests are clamped) */
static constexpr uint8_t TACHO_MEDIAN_MAX = 16;

/**
 * @brief Robust period: median of the last up to `maxPeriods` periods.
 *
 * One spurious edge splits a period into two short ones and one missed edge merges two into one
 * double-length period; with 8 periods the median ignores up to 3 such outliers on one side.
 * Fewer than 3 periods → the mean (tachoMeanPeriod), a median of 1–2 values being no better.
 * An even count returns the mean of the two middle values.
 * @param edgesUs chronological edge timestamps (oldest first), µs, uint32 wrap-safe
 * @return period in µs, 0 when fewer than two edges are available
 */
inline float tachoMedianPeriod(const uint32_t* edgesUs, uint8_t count, uint8_t maxPeriods) {
  if (edgesUs == nullptr || count < 2 || maxPeriods == 0) return 0.0f;
  if (maxPeriods > TACHO_MEDIAN_MAX) maxPeriods = TACHO_MEDIAN_MAX;
  uint8_t periods = (uint8_t)(count - 1);
  if (periods > maxPeriods) periods = maxPeriods;
  if (periods < 3) return tachoMeanPeriod(edgesUs, count, maxPeriods);
  uint32_t p[TACHO_MEDIAN_MAX];
  const uint8_t first = (uint8_t)(count - 1 - periods);
  for (uint8_t i = 0; i < periods; i++) {
    p[i] = edgesUs[first + i + 1] - edgesUs[first + i];   // unsigned: wrap-safe
  }
  for (uint8_t i = 1; i < periods; i++) {                  // insertion sort, ≤ 16 values
    const uint32_t v = p[i];
    uint8_t j = i;
    while (j > 0 && p[j - 1] > v) { p[j] = p[j - 1]; j--; }
    p[j] = v;
  }
  const uint8_t mid = (uint8_t)(periods / 2);
  if (periods & 1u) return (float)p[mid];
  return ((float)p[mid - 1] + (float)p[mid]) * 0.5f;
}

/**
 * @brief Lock-window threshold in µs: fraction × the robust period estimate (0 = no lock).
 * Computed in update(); the ISR only compares integers (no FPU use inside the ISR).
 */
inline uint32_t tachoLockMinUs(uint32_t lockPeriodUs, float fraction) {
  if (lockPeriodUs == 0 || !(fraction > 0.0f)) return 0;
  return (uint32_t)lroundf((float)lockPeriodUs * fraction);
}

/** @brief Integer core of the edge filter, used by the ISR: floor first, then the lock threshold */
inline bool tachoEdgeAcceptedMin(uint32_t sinceUs, uint32_t lockMinUs, uint32_t floorUs) {
  return sinceUs >= floorUs && sinceUs >= lockMinUs;
}

/**
 * @brief Edge filter: accept an edge `sinceUs` after the last ACCEPTED one when it clears the
 * absolute floor and, while locked (lockPeriodUs > 0), fraction × the current period estimate.
 */
inline bool tachoEdgeAccepted(uint32_t sinceUs, uint32_t lockPeriodUs, float fraction, uint32_t floorUs) {
  return tachoEdgeAcceptedMin(sinceUs, tachoLockMinUs(lockPeriodUs, fraction), floorUs);
}

/**
 * @brief Periods in ms with one decimal, space separated, oldest first ("42.9 43.1 21.4").
 * Writes "" for n == 0; always NUL-terminates (truncates when outLen is too small).
 */
inline void tachoFormatPeriodsMs(const uint32_t* periodsUs, uint8_t n, char* out, size_t outLen) {
  if (out == nullptr || outLen == 0) return;
  out[0] = '\0';
  size_t pos = 0;
  for (uint8_t i = 0; i < n && periodsUs != nullptr; i++) {
    const uint32_t tenths = (periodsUs[i] + 50u) / 100u;   // µs → 0.1 ms, rounded
    const int w = snprintf(out + pos, outLen - pos, i ? " %lu.%lu" : "%lu.%lu",
                           (unsigned long)(tenths / 10u), (unsigned long)(tenths % 10u));
    if (w < 0 || (size_t)w >= outLen - pos) { out[outLen - 1] = '\0'; return; }
    pos += (size_t)w;
  }
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
