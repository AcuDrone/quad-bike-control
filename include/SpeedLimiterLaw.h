// SpeedLimiterLaw.h — maximum-speed limiter law: proportional taper band, configurable floor,
// integral trim and ceiling slew (change add-speed-limiter-integral-floor, design D2 / D3 / D4).
//
// NO Arduino / Constants.h dependency: this header is compiled by the native unit
// tests (test/test_speed_limiter). Callers pass the constants in through SpeedLimiterParams.
//
//  band(L)  = clamp(taperFraction × L, taperMinMs, taperMaxMs)
//  P(v)     = 100 below L − band, linear down to the floor F at L, F at and above L
//  trim T   : reset to 0 at v ≤ L − band; falls at Ki × (v − L) above L (step ≤ slew);
//             rises at Ki × (L − v) (step ≤ rise cap) only while the demand binds
//             (D ≥ C − bindMargin); otherwise held. Clamped to [0, 100 − F].
//  ceiling C: target = clamp(P + T, F, 100), C moves toward it by at most slew × dt,
//             then C is clamped to [F, 100]. Output = min(D, C).
// The caller owns the no-limit / invalid-sensor fail-open branch (state.reset()).
//
// Effective limit (change add-gear-speed-ceiling): the limit fed to speedLimiterStep() is
// min(usable SPEED_MAX, fixed ceiling of the expected gear), chosen by speedLimitEffective().
// A side that is not > 0 (zero, negative, NaN) contributes nothing; a tie reports SPEED_MAX.
#pragma once

#include <stdint.h>

/** @brief Where the effective limit came from */
enum class SpeedLimitSource : uint8_t {
  NONE,       // no limit at all
  SPEED_MAX,  // the autopilot's SPEED_MAX (also on a tie with the gear ceiling)
  GEAR        // the fixed ceiling of the expected gear
};

struct SpeedLimitEffective {
  float limitMs;            // effective limit, m/s (0 = no limit)
  SpeedLimitSource source;
};

/**
 * @brief Effective limit = the smaller of SPEED_MAX and the gear ceiling.
 * @param speedMaxMs usable SPEED_MAX in m/s, or 0 when there is none
 * @param gearCeilMs fixed ceiling of the expected gear in m/s, or 0 in N / UNKNOWN
 * A value that is not > 0 (incl. NaN) does not count. Both count → the smaller, tie → SPEED_MAX.
 * Neither → {0, NONE}.
 */
static inline SpeedLimitEffective speedLimitEffective(float speedMaxMs, float gearCeilMs) {
  const bool hasMax = (speedMaxMs > 0.0f);    // false for NaN
  const bool hasGear = (gearCeilMs > 0.0f);   // false for NaN
  SpeedLimitEffective r;
  if (hasMax && (!hasGear || speedMaxMs <= gearCeilMs)) {
    r.limitMs = speedMaxMs;
    r.source = SpeedLimitSource::SPEED_MAX;
  } else if (hasGear) {
    r.limitMs = gearCeilMs;
    r.source = SpeedLimitSource::GEAR;
  } else {
    r.limitMs = 0.0f;
    r.source = SpeedLimitSource::NONE;
  }
  return r;
}

struct SpeedLimiterParams {
  float taperFraction;   // band as a fraction of the limit (0.20)
  float taperMinMs;      // minimum band, m/s
  float taperMaxMs;      // maximum band, m/s
  float floorPct;        // lowest ceiling, % (0 = calibrated idle)
  float kiPctSPerMs;     // trim integral gain, %/s per m/s of error
  float risePctS;        // cap on the trim's rise rate, %/s
  float slewPctS;        // cap on the ceiling's (and the trim's fall) rate, %/s
  float bindMarginPct;   // demand within this of the ceiling counts as "binding", %
};

struct SpeedLimiterState {
  float ceilPct;   // applied ceiling, % (100 = withdrawing nothing)
  float trimPct;   // integral trim, % (0..100 − floor)
  bool primed;     // false = the next evaluation is the first one

  SpeedLimiterState() { reset(); }
  void reset() {
    ceilPct = 100.0f;
    trimPct = 0.0f;
    primed = false;
  }
};

static inline float speedLimiterClamp(float x, float lo, float hi) {
  return (x < lo) ? lo : ((x > hi) ? hi : x);
}

/** @brief Taper band width (m/s) for a limit (m/s) */
static inline float speedLimiterBand(float limitMs, const SpeedLimiterParams& p) {
  return speedLimiterClamp(p.taperFraction * limitMs, p.taperMinMs, p.taperMaxMs);
}

/** @brief Proportional term P(v), %: 100 below L − band, linear to the floor at L, floor above */
static inline float speedLimiterTaper(float speedMs, float limitMs, float bandMs, float floorPct) {
  const float start = limitMs - bandMs;
  if (speedMs <= start) return 100.0f;
  if (speedMs >= limitMs || bandMs <= 0.0f) return floorPct;
  return 100.0f - (100.0f - floorPct) * ((speedMs - start) / bandMs);
}

/**
 * @brief One limiter evaluation (design D3). The limit must be > 0 and the speed valid; the
 * caller handles those cases by resetting the state and passing the demand through.
 * @param dtS seconds since the previous evaluation (ignored on the first one; clamped to 0..1)
 * @return min(demand, applied ceiling)
 */
static inline float speedLimiterStep(SpeedLimiterState& s, const SpeedLimiterParams& p,
                                     float demandPct, float speedMs, float limitMs, float dtS) {
  const float floorPct = speedLimiterClamp(p.floorPct, 0.0f, 100.0f);
  const float band = speedLimiterBand(limitMs, p);
  const float taper = speedLimiterTaper(speedMs, limitMs, band, floorPct);

  if (!s.primed) {
    // First evaluation after any reset: adopt the taper directly, zero trim, no slew.
    s.trimPct = 0.0f;
    s.ceilPct = speedLimiterClamp(taper, floorPct, 100.0f);
    s.primed = true;
    return (demandPct < s.ceilPct) ? demandPct : s.ceilPct;
  }

  const float dt = speedLimiterClamp(dtS, 0.0f, 1.0f);
  const float e = speedMs - limitMs;  // > 0: over the limit
  if (speedMs <= limitMs - band) {
    s.trimPct = 0.0f;                                         // outside the band: taper intact
  } else if (e > 0.0f) {
    const float step = p.kiPctSPerMs * e * dt;               // over: remove authority
    const float cap = p.slewPctS * dt;
    s.trimPct -= (step < cap) ? step : cap;
  } else if (demandPct >= s.ceilPct - p.bindMarginPct) {
    const float step = p.kiPctSPerMs * (-e) * dt;            // under and binding: give some back
    const float cap = p.risePctS * dt;
    s.trimPct += (step < cap) ? step : cap;
  }                                                           // else: driver below the ceiling, hold
  s.trimPct = speedLimiterClamp(s.trimPct, 0.0f, 100.0f - floorPct);

  const float target = speedLimiterClamp(taper + s.trimPct, floorPct, 100.0f);
  const float maxStep = p.slewPctS * dt;
  s.ceilPct += speedLimiterClamp(target - s.ceilPct, -maxStep, maxStep);
  s.ceilPct = speedLimiterClamp(s.ceilPct, floorPct, 100.0f);
  return (demandPct < s.ceilPct) ? demandPct : s.ceilPct;
}
