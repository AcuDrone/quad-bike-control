// ThrottleSlewLaw.h — throttle OPENING rate limit (change add-throttle-open-slew).
//
// NO Arduino / Constants.h dependency: this header is compiled by the native unit
// tests (test/test_throttle_slew). ThrottleController owns the timing (dt) and the
// per-vehicle rate; this file is only the arithmetic.
//
//  target is clamped to 0..100 %
//  rate <= 0            -> target              (limit off: exactly the old behaviour)
//  target <= lastPct    -> target              (closing / holding is NEVER slowed — safety)
//  otherwise            -> min(target, lastPct + rate × dt)
//
// Applied once, at the servo input (ThrottleController::setThrottlePercent), so the MAVLink,
// web and speed-limiter paths all get it. idle() and direct µs writes bypass it.
#pragma once

inline float throttleSlewStep(float targetPct, float lastPct, float dtS, float ratePctS) {
  if (!(targetPct > 0.0f)) targetPct = 0.0f;   // also maps NaN to 0 (closed)
  if (targetPct > 100.0f) targetPct = 100.0f;
  if (!(ratePctS > 0.0f)) return targetPct;
  if (targetPct <= lastPct) return targetPct;
  if (!(dtS > 0.0f)) dtS = 0.0f;
  const float limit = lastPct + ratePctS * dtS;
  return (targetPct < limit) ? targetPct : limit;
}

struct ThrottleSlewState {
  float lastPct;   // last commanded percent (what the servo was told), 0..100

  ThrottleSlewState() : lastPct(0.0f) {}
  void reset(float pct = 0.0f) { lastPct = pct; }
  float step(float targetPct, float dtS, float ratePctS) {
    lastPct = throttleSlewStep(targetPct, lastPct, dtS, ratePctS);
    return lastPct;
  }
};
