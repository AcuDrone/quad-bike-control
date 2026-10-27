// SteerRate.h — steering control mode selection and the sensorless rate mapping
// (change add-aztec-steering-rate-mode, design D1 / D3).
//
// NO Arduino / Constants.h dependency: this header is compiled by the native unit
// tests (test/test_steer_rate). Callers pass the constants in.
//
//  - steerModeSelect(): POSITION while the LWS sensor is healthy; RATE when it is not,
//    the input source is MAVLink and the autopilot is in MANUAL (or its mode is
//    unknown/stale, the same rule as the speed scaling); BLOCKED otherwise.
//  - steerRateDuty(): stick percent → signed open-loop duty.
//      |pct| < deadband → 0, else d = (|pct| − deadband) / (100 − deadband) (0 < d ≤ 1),
//      duty = round(minDuty + (maxDuty − minDuty) · d²), sign of pct (+ = right).
//    With deadband 2, min 140, max 255: 2 %→140, 25 %→146, 50 %→168, 75 %→204, 100 %→255.
#pragma once

#include <stdint.h>

/** @brief Steering control mode (also the MAVLink STEER_MODE value) */
enum class SteerMode : uint8_t { POSITION = 0, RATE = 1, BLOCKED = 2 };

/** @brief Telemetry token: position / rate / blocked */
static inline const char* steerModeToken(SteerMode m) {
  switch (m) {
    case SteerMode::POSITION: return "position";
    case SteerMode::RATE:     return "rate";
    case SteerMode::BLOCKED:  return "blocked";
  }
  return "blocked";
}

/**
 * @brief Select the steering mode (design D1).
 * @param sensorOk        debounced LWS health flag
 * @param sourceIsMavlink active input source is MAVLink
 * @param apModeKnown     a fresh autopilot flight mode is available
 * @param apModeIsManual  that mode is Rover MANUAL (ignored when !apModeKnown)
 */
static inline SteerMode steerModeSelect(bool sensorOk, bool sourceIsMavlink,
                                        bool apModeKnown, bool apModeIsManual) {
  if (sensorOk) return SteerMode::POSITION;
  if (sourceIsMavlink && (!apModeKnown || apModeIsManual)) return SteerMode::RATE;
  return SteerMode::BLOCKED;
}

/**
 * @brief Stick percent → signed rate duty (design D3).
 * @param pct      −100..+100 (clamped to ±100), + = right
 * @param deadband percent; |pct| below it gives 0
 * @return −maxDuty..+maxDuty, 0 inside the deadband
 */
static inline int16_t steerRateDuty(float pct, float deadband, uint8_t minDuty, uint8_t maxDuty) {
  if (!(pct == pct)) return 0;                       // NaN → stop
  const bool neg = pct < 0.0f;
  float a = neg ? -pct : pct;
  if (a > 100.0f) a = 100.0f;
  if (a < deadband) return 0;
  const float span = 100.0f - deadband;
  float d = span > 0.0f ? (a - deadband) / span : 1.0f;
  if (d > 1.0f) d = 1.0f;
  if (d < 0.0f) d = 0.0f;
  const float duty = (float)minDuty + (float)(maxDuty - minDuty) * d * d;
  int16_t out = (int16_t)(duty + 0.5f);
  if (out > maxDuty) out = maxDuty;
  return neg ? (int16_t)-out : out;
}
