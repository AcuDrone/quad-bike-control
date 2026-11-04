// TachoLaw.h — engine RPM from the ECU tachometer output (aztec, change add-tacho-rpm-input).
//
// NO Arduino / Constants.h dependency: this header is compiled by the native unit tests
// (test/test_tacho). Callers pass the constants in.
//
//  edges  : pulse-width validation (fw 1.5.11, replaces the 1.5.10 lock window): the ISR sees
//           BOTH edges; a RISING edge is accepted only when the LOW before it and the HIGH after
//           it each last >= minLevelUs (tachoEdgeStep); an accepted rise closer than floorUs to
//           the previous accepted one is still dropped (sanity floor)
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

/** @brief Outcome of one tachoEdgeStep() */
enum class TachoEdgeEvent : uint8_t {
  NONE = 0,        // nothing to report (a level change that is neither accepted nor a glitch)
  ACCEPTED_RISE,   // the current high pulse is validated: its rise time is TachoEdgeState::riseUs
  GLITCH           // an edge rejected by the pulse-width check (count it as rejected)
};

/**
 * @brief Pulse-width edge validator state (fw 1.5.11). Integer only, ISR-safe; all durations are
 * wrap-safe unsigned differences of µs timestamps.
 *
 *  RISE after a LOW >= min      : a new high pulse starts, its rise is a CANDIDATE (riseUs = now).
 *                                 If the previous candidate was never validated (its high was
 *                                 < min: a narrow pulse) the step returns GLITCH.
 *  RISE after a LOW <  min      : a glitch low inside a high pulse, or bounce on the falling edge:
 *                                 GLITCH; the current high pulse CONTINUES (riseUs, candidate and
 *                                 accepted state unchanged).
 *  FALL                         : the high span (now − riseUs) is measured; a candidate whose high
 *                                 lasted >= min is ACCEPTED_RISE (once per pulse). A shorter one
 *                                 stays pending: the low that follows decides (glitch → the pulse
 *                                 continues, long → the pulse was narrow and is rejected).
 *  same level as before         : NONE (both edges of a glitch elapsed before the ISR read the pin).
 */
struct TachoEdgeState {
  uint8_t  level;        // last level processed, 0 / 1
  bool     candidate;    // the current high pulse's rise awaits validation
  bool     riseKnown;    // riseUs is a real rise time (false for a high level found at init)
  bool     lowLong;      // the current low counts as long (init / after a silence), fallUs unused
  uint32_t riseUs;       // rise time of the current (or last) high pulse
  uint32_t fallUs;       // last falling edge time = start of the current low
  uint32_t lastHighUs;   // last measured high span (µs, diagnostics; 0 = none yet)
  uint32_t lastLowUs;    // last validated low (µs, diagnostics; 0 = none yet)
  bool     highMeasured; // set by the step that wrote lastHighUs, cleared by every other step
};

/** @brief Start the validator from the pin level read at begin() */
inline void tachoEdgeInit(TachoEdgeState& s, uint8_t level, uint32_t nowUs) {
  s.level = level ? 1u : 0u;
  s.candidate = false;     // a high found at init has an unknown start: never accepted
  s.riseKnown = false;
  s.lowLong = (s.level == 0);   // a low found at init: the first rise is a candidate
  s.riseUs = nowUs;
  s.fallUs = nowUs;
  s.lastHighUs = 0;
  s.lastLowUs = 0;
  s.highMeasured = false;
}

/**
 * @brief After a silence (no accepted edge for the timeout): a low in progress counts as long, and a
 * pending candidate (signal stuck high) is dropped so a stale rise time never enters the ring.
 */
inline void tachoEdgeSilence(TachoEdgeState& s) {
  if (s.level == 0) s.lowLong = true;
  s.candidate = false;
}

/**
 * @brief Feed one pin-change interrupt: `level` is the pin level read in the ISR, `nowUs` its
 * timestamp. On ACCEPTED_RISE the validated rise time is `s.riseUs`.
 */
inline TachoEdgeEvent tachoEdgeStep(TachoEdgeState& s, uint8_t level, uint32_t nowUs, uint32_t minLevelUs) {
  level = level ? 1u : 0u;
  s.highMeasured = false;
  if (level == s.level) return TachoEdgeEvent::NONE;
  s.level = level;
  if (level) {
    // RISING
    const bool lowOk = s.lowLong || (uint32_t)(nowUs - s.fallUs) >= minLevelUs;
    if (!lowOk) return TachoEdgeEvent::GLITCH;   // the high pulse continues
    if (!s.lowLong) s.lastLowUs = (uint32_t)(nowUs - s.fallUs);
    const bool narrowDropped = s.candidate;      // previous high never reached minLevelUs
    s.lowLong = false;
    s.riseUs = nowUs;
    s.riseKnown = true;
    s.candidate = true;
    return narrowDropped ? TachoEdgeEvent::GLITCH : TachoEdgeEvent::NONE;
  }
  // FALLING
  s.fallUs = nowUs;
  s.lowLong = false;
  if (!s.riseKnown) return TachoEdgeEvent::NONE;
  const uint32_t highUs = (uint32_t)(nowUs - s.riseUs);
  s.lastHighUs = highUs;
  s.highMeasured = true;
  if (s.candidate && highUs >= minLevelUs) {
    s.candidate = false;
    return TachoEdgeEvent::ACCEPTED_RISE;
  }
  return TachoEdgeEvent::NONE;
}

/** @brief Absolute sanity floor on accepted rises: true when sinceUs (to the previous accepted rise) >= floorUs */
inline bool tachoPeriodAboveFloor(uint32_t sinceUs, uint32_t floorUs) {
  return sinceUs >= floorUs;
}

/** @brief µs → ms with one decimal, rounded, as float (diagnostics; 0 stays 0) */
inline float tachoUsToMs1(uint32_t us) {
  return (float)((us + 50u) / 100u) / 10.0f;
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
