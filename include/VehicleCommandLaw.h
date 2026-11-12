// VehicleCommandLaw.h — GCS vehicle commands over MAV_CMD_USER_2 (change add-gcs-vehicle-commands):
// command decode / whitelist, the per-function interlock decisions, the RC "last change wins"
// trackers, the capability mask, QB_CAPS, the firmware / hardware version codes and IGN_STATE.
//
// NO Arduino / Constants.h / mavlink dependency: this header is compiled by the native unit
// tests (test/test_vehicle_cmd_law). MavlinkInterface decodes and acknowledges, VehicleController
// executes; this file is only the decisions. The literals below MUST match include/Constants.h:
//   MAVLINK_CMD_PARAM_EPSILON 0.01f, VCMD_PROTOCOL_VERSION 1, VCMD_CAP_* bits,
//   RC_GEAR_LOW_MIN 880 .. RC_GEAR_REVERSE_MAX 2160, RC_IGNITION_* bands,
//   RC_FRONT_LIGHT_THRESHOLD / RC_WHEEL_LOCK_THRESHOLD 1520.
#pragma once

#include <stdint.h>
#include <math.h>

// ---- Protocol constants -------------------------------------------------------------------
static const uint8_t VCMD_LAW_PROTOCOL_VERSION = 1;       // VCMD_PROTOCOL_VERSION
static const float   VCMD_LAW_PARAM_EPSILON    = 0.01f;   // MAVLINK_CMD_PARAM_EPSILON

// Capability bits (result_param2 of the caps query, QB_CAPS bits 0-15)
static const uint16_t VCMD_LAW_CAP_GEARS_RNL = 0x01;
static const uint16_t VCMD_LAW_CAP_GEAR_H    = 0x02;
static const uint16_t VCMD_LAW_CAP_IGNITION  = 0x04;
static const uint16_t VCMD_LAW_CAP_LIGHT     = 0x08;
static const uint16_t VCMD_LAW_CAP_LOCK      = 0x10;
static const uint16_t VCMD_LAW_CAP_HANDBRAKE = 0x20;

// TEMPORARILY_REJECTED reasons (result_param2)
static const uint8_t VCMD_REASON_NONE         = 0;
static const uint8_t VCMD_REASON_ENGINE_OFF   = 1;   // engine not running
static const uint8_t VCMD_REASON_SPEED        = 2;   // speed > 1.4 m/s (transmission interlock)
static const uint8_t VCMD_REASON_FAILSAFE     = 3;   // fail-safe / not under MAVLink control
static const uint8_t VCMD_REASON_PBRAKE_SPEED = 4;   // speed > 3 km/h (handbrake apply)

// param1
enum class VehicleCmdFunction : uint8_t {
  QUERY = 0, GEAR = 1, IGNITION = 2, LIGHT = 3, LOCK = 4, HANDBRAKE = 5
};

// Numeric values equal MAV_RESULT_* (static_assert'ed in MavlinkInterface.cpp).
enum class VehicleCmdResult : uint8_t {
  ACCEPTED = 0, TEMPORARILY_REJECTED = 1, DENIED = 2, UNSUPPORTED = 3, FAILED = 4
};

struct VehicleCmdOutcome {
  VehicleCmdResult result;
  uint8_t reason;   // VCMD_REASON_* for TEMPORARILY_REJECTED, else 0
};

inline VehicleCmdOutcome vcmdAccepted() { return {VehicleCmdResult::ACCEPTED, VCMD_REASON_NONE}; }
inline VehicleCmdOutcome vcmdRejected(uint8_t reason) { return {VehicleCmdResult::TEMPORARILY_REJECTED, reason}; }
inline VehicleCmdOutcome vcmdFailed() { return {VehicleCmdResult::FAILED, VCMD_REASON_NONE}; }

// Values (param2)
static const int32_t VCMD_GEAR_R = -1, VCMD_GEAR_N = 0, VCMD_GEAR_H = 1, VCMD_GEAR_L = 2;
static const int32_t VCMD_IGN_OFF = 0, VCMD_IGN_ACC = 1, VCMD_IGN_ON = 2;
static const int32_t VCMD_QUERY_CAPS = 0, VCMD_QUERY_FW = 1, VCMD_QUERY_HW = 2;

// ---- Decode -------------------------------------------------------------------------------
enum class VehicleCmdDecodeKind : uint8_t { QUERY, FUNCTION, UNSUPPORTED, DENIED };

struct VehicleCmdDecoded {
  VehicleCmdDecodeKind kind;
  VehicleCmdFunction fn;
  int32_t value;
};

// True when x is finite and within the tolerance of an integer in [-1000, 1000]; out = it.
inline bool vcmdNearInt(float x, int32_t& out) {
  if (!isfinite(x)) return false;
  if (x < -1000.0f || x > 1000.0f) return false;
  const float r = roundf(x);
  if (fabsf(x - r) > VCMD_LAW_PARAM_EPSILON) return false;
  out = (int32_t)r;
  return true;
}

// Order (design D3): unknown function → UNSUPPORTED; capability bit clear → UNSUPPORTED;
// param2 not in the table → DENIED; gear H with bit1 clear → UNSUPPORTED.
inline VehicleCmdDecoded vehicleCmdDecode(float param1, float param2, uint16_t capsMask) {
  VehicleCmdDecoded d{VehicleCmdDecodeKind::UNSUPPORTED, VehicleCmdFunction::QUERY, 0};
  int32_t fn = 0;
  if (!vcmdNearInt(param1, fn) || fn < 0 || fn > 5) return d;
  d.fn = (VehicleCmdFunction)fn;

  uint16_t needBit = 0;
  switch (d.fn) {
    case VehicleCmdFunction::QUERY:     needBit = 0; break;
    case VehicleCmdFunction::GEAR:      needBit = VCMD_LAW_CAP_GEARS_RNL; break;
    case VehicleCmdFunction::IGNITION:  needBit = VCMD_LAW_CAP_IGNITION; break;
    case VehicleCmdFunction::LIGHT:     needBit = VCMD_LAW_CAP_LIGHT; break;
    case VehicleCmdFunction::LOCK:      needBit = VCMD_LAW_CAP_LOCK; break;
    case VehicleCmdFunction::HANDBRAKE: needBit = VCMD_LAW_CAP_HANDBRAKE; break;
  }
  if (needBit != 0 && (capsMask & needBit) == 0) return d;   // UNSUPPORTED

  int32_t v = 0;
  bool ok = vcmdNearInt(param2, v);
  if (ok) {
    switch (d.fn) {
      case VehicleCmdFunction::QUERY:    ok = (v >= VCMD_QUERY_CAPS && v <= VCMD_QUERY_HW); break;
      case VehicleCmdFunction::GEAR:     ok = (v >= VCMD_GEAR_R && v <= VCMD_GEAR_L); break;
      case VehicleCmdFunction::IGNITION: ok = (v >= VCMD_IGN_OFF && v <= VCMD_IGN_ON); break;
      default:                           ok = (v == 0 || v == 1); break;
    }
  }
  if (!ok) {
    d.kind = VehicleCmdDecodeKind::DENIED;
    return d;
  }
  if (d.fn == VehicleCmdFunction::GEAR && v == VCMD_GEAR_H && (capsMask & VCMD_LAW_CAP_GEAR_H) == 0) {
    return d;   // UNSUPPORTED: H not on this vehicle
  }
  d.kind = (d.fn == VehicleCmdFunction::QUERY) ? VehicleCmdDecodeKind::QUERY : VehicleCmdDecodeKind::FUNCTION;
  d.value = v;
  return d;
}

// ---- Interlocks (design D5) -----------------------------------------------------------------
// underMavlink = input source is MAVLINK AND the command stream is valid (design D4).

struct VcmdGearInputs {
  bool underMavlink;
  bool engineRunning;
  bool transInitialised;
  bool alreadyAtTarget;   // the transmission's destination already equals the request
  bool speedOk;           // the transmission speed interlock allows this gear (N always)
};

// ACCEPTED with setGear = true → the caller calls setGear(); ACCEPTED with setGear = false →
// nothing moves (already there).
inline VehicleCmdOutcome vcmdGearDecide(const VcmdGearInputs& in, bool& setGear) {
  setGear = false;
  if (!in.underMavlink) return vcmdRejected(VCMD_REASON_FAILSAFE);
  if (!in.engineRunning || !in.transInitialised) return vcmdRejected(VCMD_REASON_ENGINE_OFF);
  if (in.alreadyAtTarget) return vcmdAccepted();
  if (!in.speedOk) return vcmdRejected(VCMD_REASON_SPEED);
  setGear = true;
  return vcmdAccepted();
}

enum class VcmdIgnAction : uint8_t { NONE, OFF, ACC, IGNITION_NO_CRANK, CRANK };

inline VehicleCmdOutcome vcmdIgnitionDecide(int32_t value, bool underMavlink, bool relaysOk,
                                            bool engineRunning, VcmdIgnAction& action) {
  action = VcmdIgnAction::NONE;
  if (value != VCMD_IGN_OFF && !underMavlink) return vcmdRejected(VCMD_REASON_FAILSAFE);
  if (!relaysOk) return vcmdFailed();
  if (value == VCMD_IGN_OFF) action = VcmdIgnAction::OFF;
  else if (value == VCMD_IGN_ACC) action = VcmdIgnAction::ACC;
  else action = engineRunning ? VcmdIgnAction::IGNITION_NO_CRANK : VcmdIgnAction::CRANK;
  return vcmdAccepted();
}

// Headlight: accepted in every source.
inline VehicleCmdOutcome vcmdLightDecide(bool relaysOk) {
  return relaysOk ? vcmdAccepted() : vcmdFailed();
}

// Diff lock: reason 3 when not under MAVLink control.
inline VehicleCmdOutcome vcmdLockDecide(bool underMavlink, bool relaysOk) {
  if (!underMavlink) return vcmdRejected(VCMD_REASON_FAILSAFE);
  return relaysOk ? vcmdAccepted() : vcmdFailed();
}

// ---- RC channels: absent-aware decode and "last change wins" (design D6) --------------------
static const int8_t   VCMD_RC_ABSENT       = -128;
static const uint16_t VCMD_RC_VALID_MIN_US = 880;    // RC_CHANNEL_VALID_MIN_US
static const uint16_t VCMD_RC_VALID_MAX_US = 2160;   // RC_CHANNEL_VALID_MAX_US

inline bool vcmdRcInBand(uint16_t us) { return us >= VCMD_RC_VALID_MIN_US && us <= VCMD_RC_VALID_MAX_US; }

// Gear: L 880-1306, N 1307-1733, R 1734-2160 → VCMD_GEAR_L / _N / _R; else ABSENT. No H on RC.
inline int8_t vcmdRcGear(uint16_t us) {
  if (!vcmdRcInBand(us)) return VCMD_RC_ABSENT;
  if (us <= 1306) return (int8_t)VCMD_GEAR_L;
  if (us <= 1733) return (int8_t)VCMD_GEAR_N;
  return (int8_t)VCMD_GEAR_R;
}

// Ignition: OFF 880-1200, ACC 1201-1520, ON 1521-2160; else ABSENT.
inline int8_t vcmdRcIgnition(uint16_t us) {
  if (!vcmdRcInBand(us)) return VCMD_RC_ABSENT;
  if (us <= 1200) return (int8_t)VCMD_IGN_OFF;
  if (us <= 1520) return (int8_t)VCMD_IGN_ACC;
  return (int8_t)VCMD_IGN_ON;
}

// On/off switch (light, lock): > 1520 = 1; else 0; out of band = ABSENT.
inline int8_t vcmdRcSwitch(uint16_t us) {
  if (!vcmdRcInBand(us)) return VCMD_RC_ABSENT;
  return (us > 1520) ? 1 : 0;
}

// One RC function: apply only on a change of the in-band value, or the first value after reset().
struct RcEdge {
  bool has = false;
  int8_t last = 0;
  bool step(int8_t v) {
    if (v == VCMD_RC_ABSENT) return false;   // a disappearing channel is not a change
    if (!has || v != last) {
      has = true;
      last = v;
      return true;
    }
    return false;
  }
  void reset() { has = false; }
};

// The four RC functions plus the pending RC gear (design D6, owner decisions D-A, D-E, D-H).
struct VcmdRcState {
  RcEdge gear, ignition, light, lock;
  bool gearPending = false;
  int8_t pendingGear = 0;

  // Every loop in which RC is not in control: the next valid frame re-applies everything.
  void resync() {
    gear.reset();
    ignition.reset();
    light.reset();
    lock.reset();
    gearPending = false;
  }
  // RC gear: an edge becomes (or replaces) the pending RC gear. Returns true on an edge.
  bool stepGear(int8_t v) {
    if (!gear.step(v)) return false;
    gearPending = true;
    pendingGear = v;
    return true;
  }
  // The pending RC gear was applied (or the target already equals it).
  void gearDone() { gearPending = false; }
  // An accepted GCS gear command supersedes a pending RC gear (last change wins).
  void onGcsGearAccepted() { gearPending = false; }
};

// ---- Capabilities, versions, broadcasts ------------------------------------------------------
inline uint16_t vehicleCapsMask(bool gearsRnl, bool gearH, bool ignition, bool light, bool lock,
                                bool handbrake) {
  return (uint16_t)((gearsRnl ? VCMD_LAW_CAP_GEARS_RNL : 0) | (gearH ? VCMD_LAW_CAP_GEAR_H : 0) |
                    (ignition ? VCMD_LAW_CAP_IGNITION : 0) | (light ? VCMD_LAW_CAP_LIGHT : 0) |
                    (lock ? VCMD_LAW_CAP_LOCK : 0) | (handbrake ? VCMD_LAW_CAP_HANDBRAKE : 0));
}

// QB_CAPS value: mask in bits 0-15, bits 16-23 zero, protocol version in bits 24-31.
inline int32_t qbCapsValue(uint16_t mask, uint8_t protocolVersion) {
  return (int32_t)(((uint32_t)protocolVersion << 24) | (uint32_t)mask);
}

// "major.minor.patch" → major*10000 + minor*100 + patch; a malformed string gives 0.
constexpr int32_t firmwareVersionCode(const char* s) {
  int32_t part[3] = {0, 0, 0};
  int idx = 0;
  bool digit = false;
  for (const char* p = s; *p != '\0'; ++p) {
    if (*p >= '0' && *p <= '9') {
      part[idx] = part[idx] * 10 + (*p - '0');
      digit = true;
    } else if (*p == '.' && digit && idx < 2) {
      idx++;
      digit = false;
    } else {
      return 0;
    }
  }
  if (idx != 2 || !digit) return 0;
  return part[0] * 10000 + part[1] * 100 + part[2];
}

// Hardware version: bits 0-7 board revision, 8-15 vehicle type, 16-23 detected relay board.
inline int32_t hardwareVersionCode(uint8_t boardRevision, uint8_t vehicleType, uint8_t relayBoard) {
  return (int32_t)((uint32_t)boardRevision | ((uint32_t)vehicleType << 8) | ((uint32_t)relayBoard << 16));
}

// IGN_STATE from the applied relay state (RelayController::IgnitionState order: OFF 0, ACC 1,
// IGNITION 2, CRANKING 3). ACC with a crank armed is the pre-crank dwell → 3.
inline uint8_t ignStateCode(uint8_t relayState, bool crankPending) {
  if (relayState >= 3) return 3;
  if (relayState == 1 && crankPending) return 3;
  return relayState;
}

// ---- Vehicle-command sink (implemented by VehicleController, registered by main.cpp) ---------
class VehicleCommandSink {
public:
  virtual ~VehicleCommandSink() {}
  // Execute one decoded, validated function command (param1 1-5) and report the outcome.
  virtual VehicleCmdOutcome executeVehicleCommand(VehicleCmdFunction fn, int32_t value) = 0;
};
