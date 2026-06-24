#!/usr/bin/env python3
"""
Simulate the ESP32's OUTBOUND telemetry into BlueOS (mavlink2rest), so you can
preview how it renders in Mission Planner WITHOUT the real ESP32 connected.

Sends, as system 1 / component 25 (MAV_COMP_ID_USER1 — same identity the firmware uses):
  - HEARTBEAT       (1 Hz, MAV_TYPE_GROUND_ROVER)
  - EFI_STATUS      (rpm, coolant = cylinder_head_temperature, throttle_position, health)
  - NAMED_VALUE_FLOAT "GEAR"  (sequence-encoded: R=-1 N=0 H=1 L=2, .5 = mid-shift)
  - STATUSTEXT      (optional, on a timer)

mavlink2rest schema notes (discovered against BlueOS): char[] fields are arrays of
1-char strings; enums are {"type": "..."}; EFI_STATUS needs all 19 fields present.

Examples
--------
  ./mavlink_telemetry_sim.py --host 10.0.0.1 --demo
  ./mavlink_telemetry_sim.py --rpm 3000 --coolant 92 --throttle 40 --gear L
  ./mavlink_telemetry_sim.py --gear N --dry-run
"""
import argparse, json, math, sys, time, urllib.request, urllib.error

GEAR_VAL = {"R": -1.0, "N": 0.0, "H": 1.0, "L": 2.0}


def chars(s, n):
    """char[n] as mavlink2rest expects: list of n one-char strings (space-padded)."""
    s = (s + (" " * n))[:n]
    return [c for c in s]


def envelope(sysid, compid, seq, message):
    return {"header": {"system_id": sysid, "component_id": compid, "sequence": seq},
            "message": message}


def heartbeat():
    return {"type": "HEARTBEAT", "custom_mode": 0,
            "mavtype": {"type": "MAV_TYPE_GROUND_ROVER"},
            "autopilot": {"type": "MAV_AUTOPILOT_INVALID"},
            "base_mode": {"bits": 0},
            "system_status": {"type": "MAV_STATE_ACTIVE"},
            "mavlink_version": 3}


def efi_status(rpm, coolant, throttle, healthy=True):
    # All 19 fields required (14 base + extensions). Only rpm/coolant/throttle/health used.
    return {"type": "EFI_STATUS",
            "ecu_index": 0.0, "rpm": float(rpm), "fuel_consumed": 0.0, "fuel_flow": 0.0,
            "engine_load": 0.0, "throttle_position": float(throttle), "spark_dwell_time": 0.0,
            "barometric_pressure": 0.0, "intake_manifold_pressure": 0.0,
            "intake_manifold_temperature": 0.0, "cylinder_head_temperature": float(coolant),
            "ignition_timing": 0.0, "injection_time": 0.0, "exhaust_gas_temperature": 0.0,
            "throttle_out": 0.0, "pt_compensation": 0.0,
            "health": 1 if healthy else 0, "ignition_voltage": 0.0, "fuel_pressure": 0.0}


def named_value_float(name, value, t_ms):
    return {"type": "NAMED_VALUE_FLOAT", "time_boot_ms": int(t_ms),
            "value": float(value), "name": chars(name, 10)}


def statustext(text, severity="MAV_SEVERITY_INFO"):
    return {"type": "STATUSTEXT", "severity": {"type": severity},
            "text": chars(text, 50), "id": 0, "chunk_seq": 0}


def post(url, payload, timeout=4.0):
    req = urllib.request.Request(url, data=json.dumps(payload).encode(), method="POST",
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, r.read().decode("utf-8", "replace")[:120]
    except urllib.error.HTTPError as e:
        return e.code, e.read().decode("utf-8", "replace")[:120]
    except Exception as e:
        return None, str(e)


def main():
    p = argparse.ArgumentParser(description="Simulate ESP32 outbound telemetry into BlueOS.")
    p.add_argument("--host", default="10.0.0.1")
    p.add_argument("--base", default="/mavlink2rest")
    p.add_argument("--sysid", type=int, default=1)
    p.add_argument("--compid", type=int, default=25)
    p.add_argument("--duration", type=float, default=0.0, help="seconds (0 = until Ctrl-C)")
    p.add_argument("--rpm", type=float, default=2500.0)
    p.add_argument("--coolant", type=float, default=85.0)
    p.add_argument("--throttle", type=float, default=20.0)
    p.add_argument("--gear", choices=list(GEAR_VAL), default="N")
    p.add_argument("--statustext", default=None, help="send this STATUSTEXT once per second")
    p.add_argument("--demo", action="store_true", help="animate rpm/coolant/gear")
    p.add_argument("--dry-run", action="store_true")
    args = p.parse_args()

    send_url = f"http://{args.host}{args.base}/mavlink"

    def state(t):
        if args.demo:
            rpm = 2500 + 1200 * math.sin(t * 2 * math.pi / 5.0)
            coolant = 80 + 15 * math.sin(t * 2 * math.pi / 30.0)
            throttle = 35 + 25 * math.sin(t * 2 * math.pi / 7.0)
            # GEAR triangle wave over the staircase [-1 .. 2] in 0.5 steps
            steps = [-1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0, 1.5, 1.0, 0.5, 0.0, -0.5]
            gear = steps[int(t) % len(steps)]
            return rpm, coolant, throttle, gear
        return args.rpm, args.coolant, args.throttle, GEAR_VAL[args.gear]

    if args.dry_run:
        rpm, coolant, throttle, gear = state(0.0)
        print(json.dumps(envelope(args.sysid, args.compid, 0, efi_status(rpm, coolant, throttle)), indent=2))
        print(json.dumps(envelope(args.sysid, args.compid, 0, named_value_float("GEAR", gear, 0)), indent=2))
        return 0

    print(f"Simulating ESP32 telemetry -> {send_url} as sys {args.sysid}/comp {args.compid}. Ctrl-C to stop.")
    seq = 0
    t0 = time.time()
    last_hb = last_report = last_status = 0.0
    try:
        while True:
            now = time.time(); t = now - t0
            if args.duration and t >= args.duration:
                break
            rpm, coolant, throttle, gear = state(t)

            if now - last_hb >= 1.0:
                last_hb = now
                st, body = post(send_url, envelope(args.sysid, args.compid, seq, heartbeat())); seq = (seq + 1) & 0xFF
                if st != 200:
                    print(f"[HEARTBEAT] {st} {body}", file=sys.stderr)

            if now - last_report >= 0.2:   # 5 Hz
                last_report = now
                # EFI_STATUS (standard; shows in MAVLink Inspector, not MP's Status tab)
                st, body = post(send_url, envelope(args.sysid, args.compid, seq, efi_status(rpm, coolant, throttle))); seq = (seq + 1) & 0xFF
                if st != 200:
                    print(f"[EFI_STATUS] {st} {body}", file=sys.stderr)
                # NAMED_VALUE_FLOATs — these DO surface in MP (Status tab as customFieldN,
                # labeled by name in the Tuning graph dropdown). Order fixes the customField slot.
                for nm, val in (("GEAR", gear), ("RPM", rpm), ("COOLANT", coolant), ("THROTTLE", throttle)):
                    st, body = post(send_url, envelope(args.sysid, args.compid, seq, named_value_float(nm, val, t * 1000))); seq = (seq + 1) & 0xFF
                    if st != 200:
                        print(f"[{nm}] {st} {body}", file=sys.stderr)

            if args.statustext and now - last_status >= 1.0:
                last_status = now
                post(send_url, envelope(args.sysid, args.compid, seq, statustext(args.statustext))); seq = (seq + 1) & 0xFF

            print(f"\rt={t:5.1f}s  rpm={rpm:6.0f}  coolant={coolant:4.0f}C  thr={throttle:4.0f}%  gear={gear:+.1f}",
                  end="", file=sys.stderr)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    print("\nStopped.", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
