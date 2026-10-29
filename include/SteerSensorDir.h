// SteerSensorDir.h — steering-angle sensor direction (normal / mirrored mount) and its
// inference at limit capture (change auto-detect-aztec-steering-sensor-direction, design D1 / D2).
//
// NO Arduino / Constants.h dependency: this header is compiled by the native unit
// tests (test/test_steer_sensor_dir). Callers pass the constants in.
//
//  - The Bosch LWS angle is + = LEFT for a normal mount. The firmware sign is + = RIGHT,
//    so position = −angle (inv = false), or +angle for a mirrored mount (inv = true).
//    The signed rate follows the same rule.
//  - steerDirCapture(): the Left/Right capture button is the only ground truth for "left".
//      |pos| < minSpan                       -> REFUSE_TOO_CLOSE   (never toggles)
//      on the expected side (left: pos < 0)  -> STORE
//      wrong side && opposite limit unset    -> TOGGLE_AND_STORE   (!inv, −pos, clear the opposite)
//      wrong side && opposite limit stored   -> REFUSE_CONTRADICTS (no ping-pong)
#pragma once

#include <stdint.h>

/** @brief Firmware position (0.1°, + = right) from the raw sensor angle (0.1°, sensor sign) */
static inline int32_t steerSensorPosition(int16_t angleRaw, bool inv) {
  return inv ? (int32_t)angleRaw : -(int32_t)angleRaw;
}

/**
 * @brief Signed rotation rate, deg/s, firmware sign (+ = turning right).
 * @param speedDegS unsigned speed from the frame
 * @param cw        direction bit (1 = clockwise, sensor's own sense)
 * @param inv       sensor mounted mirrored
 */
static inline int32_t steerSensorRate(int32_t speedDegS, bool cw, bool inv) {
  const int32_t s = cw ? speedDegS : -speedDegS;
  return inv ? -s : s;
}

/** @brief Outcome of a limit capture with respect to the sensor direction */
enum class SteerDirAction : uint8_t { STORE, TOGGLE_AND_STORE, REFUSE_TOO_CLOSE, REFUSE_CONTRADICTS };

struct SteerDirResult {
  SteerDirAction action;
  bool inv;            // flag after the capture
  int32_t pos;         // position to store, under that flag (unchanged on a refusal)
  bool clearOpposite;  // the opposite limit must be cleared (TOGGLE_AND_STORE only)
};

/**
 * @brief Decide a limit capture (design D2).
 * @param left        "Set Left" (true) or "Set Right" (false)
 * @param pos         current position under the current flag, 0.1°
 * @param inv         current flag
 * @param oppositeSet the opposite limit is stored
 * @param minSpan     STEER_CAL_MIN_SPAN, 0.1°
 */
static inline SteerDirResult steerDirCapture(bool left, int32_t pos, bool inv,
                                             bool oppositeSet, int32_t minSpan) {
  const int32_t mag = pos < 0 ? -pos : pos;
  if (mag < minSpan) return {SteerDirAction::REFUSE_TOO_CLOSE, inv, pos, false};
  const bool expected = left ? (pos < 0) : (pos > 0);
  if (expected) return {SteerDirAction::STORE, inv, pos, false};
  if (oppositeSet) return {SteerDirAction::REFUSE_CONTRADICTS, inv, pos, false};
  return {SteerDirAction::TOGGLE_AND_STORE, !inv, -pos, true};
}
