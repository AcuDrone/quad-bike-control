#!/usr/bin/env python3
"""
Inject generated SERVO_OUTPUT_RAW (and optional HEARTBEAT) into BlueOS via the
mavlink2rest REST API, to exercise the ESP32 MAVLink interface without an
autopilot driving the channels.

BlueOS exposes mavlink2rest (reverse-proxied) at:  http://<host>/mavlink2rest/
  - POST <base>/mavlink                          -> send a MAVLink message
  - GET  <base>/helper/message/<MESSAGE_NAME>     -> JSON template for a message

Channel map matches ServoChannelConfig in include/Constants.h:
  servo1 = STEERING        (-100..+100  -> 1000..2000, center 1500)
  servo2 = THROTTLE/BRAKE  (combined: >1500 throttle, <1500 brake)
  servo3 = TRANSMISSION    (R/N/L)
  servo4 = IGNITION        (OFF/ACC/IGNITION)
  servo6 = FRONT_LIGHT     (>1520 = ON)

Examples
--------
  # Stream center/idle at 25 Hz (keeps the link "valid", no fail-safe)
  ./mavlink_inject.py --host 10.0.0.1

  # Hold specific commands
  ./mavlink_inject.py --steering -40 --throttle 30 --gear L --ignition IGNITION --light on

  # Apply brake (combined channel below center)
  ./mavlink_inject.py --brake 60

  # Animated demo: sweep steering, cycle gears, toggle light (watch GEAR in MP)
  ./mavlink_inject.py --demo --heartbeat

  # Print the JSON instead of sending it
  ./mavlink_inject.py --gear R --dry-run
"""

import argparse
import json
import math
import sys
import time
import urllib.request
import urllib.error

# Channel indices (1-based) — mirror ServoChannelConfig
CH_STEERING = 1
CH_THROTTLE = 2          # combined throttle (>center) / brake (<center)
CH_TRANSMISSION = 3
CH_IGNITION = 4
CH_FRONT_LIGHT = 6

US_MIN, US_CENTER, US_MAX = 1000, 1500, 2000

# Representative microsecond values within the firmware's decode ranges
GEAR_US = {"R": 1000, "N": 1400, "L": 1800}            # RC_GEAR_* ranges
IGNITION_US = {"OFF": 1000, "ACC": 1400, "IGNITION": 1800}  # RC_IGNITION_* ranges
LIGHT_US = {True: 2000, False: 1000}                   # RC_FRONT_LIGHT_THRESHOLD = 1520


def steering_to_us(pct: float) -> int:
    pct = max(-100.0, min(100.0, pct))
    return int(round(US_CENTER + pct * (US_MAX - US_MIN) / 200.0))


def throttle_to_us(pct: float) -> int:
    pct = max(0.0, min(100.0, pct))
    return int(round(US_CENTER + pct * (US_MAX - US_CENTER) / 100.0))


def brake_to_us(pct: float) -> int:
    pct = max(0.0, min(100.0, pct))
    return int(round(US_CENTER - pct * (US_CENTER - US_MIN) / 100.0))


def post_json(url: str, payload: dict, timeout: float = 2.0):
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(url, data=data, method="POST",
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return resp.status, resp.read().decode("utf-8", "replace")


def get_json(url: str, timeout: float = 3.0):
    req = urllib.request.Request(url, method="GET")
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def servo_output_template() -> dict:
    """All-fields SERVO_OUTPUT_RAW message (no enums, safe to hardcode)."""
    msg = {"type": "SERVO_OUTPUT_RAW", "time_usec": 0, "port": 0}
    for i in range(1, 17):
        msg[f"servo{i}_raw"] = US_CENTER
    return msg


def build_envelope(sysid: int, compid: int, seq: int, message: dict) -> dict:
    return {"header": {"system_id": sysid, "component_id": compid, "sequence": seq},
            "message": message}


def main() -> int:
    p = argparse.ArgumentParser(description="Inject MAVLink SERVO_OUTPUT_RAW into BlueOS (mavlink2rest).")
    p.add_argument("--host", default="10.0.0.1", help="BlueOS host/IP (default 10.0.0.1)")
    p.add_argument("--base", default="/mavlink2rest", help="mavlink2rest base path (default /mavlink2rest)")
    p.add_argument("--rate", type=float, default=25.0, help="send rate in Hz (default 25)")
    p.add_argument("--duration", type=float, default=0.0, help="seconds to run (0 = until Ctrl-C)")
    p.add_argument("--sysid", type=int, default=1, help="injected system id (default 1 = autopilot)")
    p.add_argument("--compid", type=int, default=1, help="injected component id (default 1 = autopilot)")
    p.add_argument("--steering", type=float, default=0.0, help="steering %% -100..100")
    p.add_argument("--throttle", type=float, default=None, help="throttle %% 0..100 (ch2 above center)")
    p.add_argument("--brake", type=float, default=None, help="brake %% 0..100 (ch2 below center)")
    p.add_argument("--gear", choices=list(GEAR_US), default="N", help="gear (default N)")
    p.add_argument("--ignition", choices=list(IGNITION_US), default="OFF", help="ignition (default OFF)")
    p.add_argument("--light", choices=["on", "off"], default="off", help="front light (default off)")
    p.add_argument("--heartbeat", action="store_true", help="also stream a 1 Hz HEARTBEAT")
    p.add_argument("--demo", action="store_true", help="animate steering/gear/light for a visible test")
    p.add_argument("--dry-run", action="store_true", help="print one message JSON and exit (no network)")
    args = p.parse_args()

    base = f"http://{args.host}{args.base}"
    send_url = f"{base}/mavlink"

    if args.throttle is not None and args.brake is not None:
        p.error("--throttle and --brake share channel 2; set only one")

    # Optional heartbeat template (has enum fields) — fetch from the helper so the
    # exact mavlink2rest schema is used.
    hb_template = None
    if args.heartbeat and not args.dry_run:
        try:
            env = get_json(f"{base}/helper/message/HEARTBEAT")
            hb_template = env["message"]
            hb_template["type"] = "HEARTBEAT"
        except Exception as e:
            print(f"[warn] could not fetch HEARTBEAT template ({e}); skipping heartbeat", file=sys.stderr)

    def channels_for(t: float):
        """Return (steering_us, ch2_us, gear, ignition, light_on) for time t."""
        if args.demo:
            steer = 80.0 * math.sin(t * 2 * math.pi / 6.0)          # sweep ±80% over 6 s
            gear = ["N", "L", "N", "R"][int(t / 3.0) % 4]            # step gears every 3 s
            light = (int(t / 2.0) % 2 == 0)                          # toggle every 2 s
            ch2 = throttle_to_us(15.0)                              # gentle throttle
            ign = "IGNITION"
            return steering_to_us(steer), ch2, gear, ign, light
        # static mode from CLI
        if args.brake is not None:
            ch2 = brake_to_us(args.brake)
        elif args.throttle is not None:
            ch2 = throttle_to_us(args.throttle)
        else:
            ch2 = US_CENTER
        return steering_to_us(args.steering), ch2, args.gear, args.ignition, (args.light == "on")

    def make_servo_message(t: float) -> dict:
        steer_us, ch2_us, gear, ign, light_on = channels_for(t)
        msg = servo_output_template()
        msg[f"servo{CH_STEERING}_raw"] = steer_us
        msg[f"servo{CH_THROTTLE}_raw"] = ch2_us
        msg[f"servo{CH_TRANSMISSION}_raw"] = GEAR_US[gear]
        msg[f"servo{CH_IGNITION}_raw"] = IGNITION_US[ign]
        msg[f"servo{CH_FRONT_LIGHT}_raw"] = LIGHT_US[light_on]
        return msg

    if args.dry_run:
        env = build_envelope(args.sysid, args.compid, 0, make_servo_message(0.0))
        print(json.dumps(env, indent=2))
        return 0

    print(f"Injecting SERVO_OUTPUT_RAW -> {send_url} at {args.rate:.0f} Hz "
          f"(sys {args.sysid}/comp {args.compid}){' + HEARTBEAT' if hb_template else ''}. Ctrl-C to stop.")

    period = 1.0 / args.rate
    seq = 0
    t0 = time.time()
    last_hb = 0.0
    sent = 0
    try:
        while True:
            now = time.time()
            t = now - t0
            if args.duration and t >= args.duration:
                break

            try:
                post_json(send_url, build_envelope(args.sysid, args.compid, seq, make_servo_message(t)))
                seq = (seq + 1) & 0xFF
                sent += 1
            except urllib.error.URLError as e:
                print(f"[error] POST failed: {e}  (check --host/--base and that mavlink2rest is up)",
                      file=sys.stderr)
                time.sleep(0.5)

            if hb_template and (now - last_hb) >= 1.0:
                last_hb = now
                try:
                    post_json(send_url, build_envelope(args.sysid, args.compid, seq, dict(hb_template)))
                    seq = (seq + 1) & 0xFF
                except urllib.error.URLError:
                    pass

            if sent % int(max(1, args.rate)) == 0:
                print(f"\rsent={sent}  t={t:5.1f}s", end="", file=sys.stderr)

            sleep = period - (time.time() - now)
            if sleep > 0:
                time.sleep(sleep)
    except KeyboardInterrupt:
        pass

    print(f"\nStopped. Sent {sent} SERVO_OUTPUT_RAW messages.", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
