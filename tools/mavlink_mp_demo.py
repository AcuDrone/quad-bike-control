#!/usr/bin/env python3
"""
Inject QuadBike telemetry into a MAVLink network via mavlink2rest, so you can see
how it renders in Mission Planner (and the QuadBike header plugin) WITHOUT the ESP32.

It reproduces exactly what the firmware now sends from component 25:
  - HEARTBEAT                         (1 Hz)
  - EFI_STATUS                        (5 Hz)  one packet carrying all three:
        engine_load               = GEAR  (sequence pos R=-1 N=0 H=1 L=2, midpoint while shifting)
        rpm                       = RPM
        cylinder_head_temperature = TMP   (plugin: blue <65, white 65-102, red >102)

Usage:
  python3 tools/mavlink_mp_demo.py                 # default host 10.0.0.1
  python3 tools/mavlink_mp_demo.py 10.0.0.1
  python3 tools/mavlink_mp_demo.py 10.0.0.1 --hold-gear N   # don't cycle gears, hold one

The gear walks the physical sequence one step at a time (N->H->L->H->N->R->N...),
emitting the MIDPOINT value (0.5 / 1.5 staircase) while "shifting" so you can see the
plugin animate, then settling on the integer.
"""
import argparse, json, math, sys, time, urllib.request

SYSID = 1
COMPID = 25            # MAVLINK_COMPONENT_ID — the ESP32's identity
TELEM_HZ = 5.0
DT = 1.0 / TELEM_HZ

# Physical gear sequence and its encoded position (matches firmware encodeGear()).
GEAR_POS = {"R": -1.0, "N": 0.0, "H": 1.0, "L": 2.0}
POS_GEAR = {v: k for k, v in GEAR_POS.items()}
SEQUENCE = ["N", "H", "L", "H", "N", "R", "N"]   # demo cycle through every gear


def post(url, msg, seq):
    env = {"header": {"system_id": SYSID, "component_id": COMPID, "sequence": seq & 0xFF},
           "message": msg}
    req = urllib.request.Request(url, data=json.dumps(env).encode(), method="POST",
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=4) as r:
            return r.status
    except Exception as e:
        return f"ERR {e}"


def efi(gear, rpm, coolant):
    # All telemetry in one EFI_STATUS: engine_load=GEAR, rpm=RPM, cyl_head_temp=TMP.
    return {"type": "EFI_STATUS", "ecu_index": 0.0, "rpm": float(rpm), "fuel_consumed": 0.0,
            "fuel_flow": 0.0, "engine_load": float(gear), "throttle_position": 0.0,
            "spark_dwell_time": 0.0, "barometric_pressure": 0.0, "intake_manifold_pressure": 0.0,
            "intake_manifold_temperature": 0.0, "cylinder_head_temperature": float(coolant),
            "ignition_timing": 0.0, "injection_time": 0.0, "exhaust_gas_temperature": 0.0,
            "throttle_out": 0.0, "pt_compensation": 0.0, "health": 1,
            "ignition_voltage": 0.0, "fuel_pressure": 0.0}


def heartbeat():
    return {"type": "HEARTBEAT", "custom_mode": 0,
            "mavtype": {"type": "MAV_TYPE_GROUND_ROVER"},
            "autopilot": {"type": "MAV_AUTOPILOT_INVALID"},
            "base_mode": {"bits": 0},
            "system_status": {"type": "MAV_STATE_ACTIVE"},
            "mavlink_version": 3}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("host", nargs="?", default="10.0.0.1")
    ap.add_argument("--hold-gear", choices=list(GEAR_POS), default=None,
                    help="hold one gear instead of cycling")
    args = ap.parse_args()
    url = f"http://{args.host}/mavlink2rest/mavlink"
    print(f"Injecting to {url}  (sysid {SYSID}, comp {COMPID})  Ctrl-C to stop\n")

    seq = 0
    t0 = time.time()
    last_hb = -1.0

    # gear state machine
    seq_idx = 0
    cur_pos = GEAR_POS[args.hold_gear] if args.hold_gear else GEAR_POS["N"]
    tgt_pos = cur_pos
    shifting = False
    next_shift_at = 3.0          # seconds before first shift
    SHIFT_TIME = 0.7             # seconds spent at the midpoint

    while True:
        now = time.time()
        t = now - t0
        t_ms = t * 1000.0

        # ---- gear sequence simulation ----
        if not args.hold_gear:
            if not shifting and t >= next_shift_at:
                seq_idx = (seq_idx + 1) % len(SEQUENCE)
                tgt_pos = GEAR_POS[SEQUENCE[seq_idx]]
                if tgt_pos != cur_pos:
                    shifting = True
                    shift_started = t
            if shifting:
                if t - shift_started >= SHIFT_TIME:
                    cur_pos = tgt_pos
                    shifting = False
                    next_shift_at = t + 3.0

        if shifting:
            # step one sequence position toward target; report the midpoint
            step = 1.0 if tgt_pos > cur_pos else -1.0
            gear_val = cur_pos + step * 0.5
            gear_lbl = f"{POS_GEAR.get(cur_pos,'?')}->{POS_GEAR.get(cur_pos+step,'?')}"
        else:
            gear_val = cur_pos
            gear_lbl = POS_GEAR.get(cur_pos, "?")

        # ---- engine values ----
        # RPM: idle ~900, blips to ~3200 during a shift, gentle oscillation otherwise
        base_rpm = 900 + 350 * (math.sin(t * 0.7) + 1)
        rpm = base_rpm + (1800 if shifting else 0)
        # TMP: slow sweep 40 -> 115 -> 40 so you see blue / white / red bands (~40 s period)
        tmp = 77 + 38 * math.sin(t * 2 * math.pi / 40.0)

        # ---- send (5 Hz): one EFI_STATUS carrying all three ----
        post(url, efi(gear_val, rpm, tmp), seq); seq += 1

        # ---- HEARTBEAT (1 Hz) ----
        if now - last_hb >= 1.0:
            last_hb = now
            post(url, heartbeat(), seq); seq += 1

        band = "blue" if tmp < 65 else ("red" if tmp > 102 else "white")
        print(f"\rGEAR {gear_val:+.1f} ({gear_lbl:<5})  RPM {rpm:5.0f}  TMP {tmp:5.1f}°C [{band}] ",
              end="", flush=True)
        time.sleep(DT)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nstopped")
