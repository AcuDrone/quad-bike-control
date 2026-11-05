#!/usr/bin/env python3
"""Host side of the USB steering diagnostics console (firmware >= 1.5.12, SteerDiag).

Connect the ESP32-S3 board's USB-C port to the Mac, vehicle standing still, steering
calibrated (both limits captured), VESC powered. The script talks to the firmware's
USB-CDC console only (115200, line commands) — never the VESC UART.

Firmware commands (see include/SteerDiag.h for the full contract):
  diag on | diag off | status | stop | s | help
  ramp L|R [max=220] [step=2] [stepms=150] [start=40]
  pulse L|R <duty> <ms>          (ms <= 1000)
  hold  L|R <duty> <ms>          (ms <= 1500)
  goto <deg>                     (normal closed loop, needs both limits)

Firmware output lines:
  OK <cmd>                       accepted
  E,<cmd>,<reason> / E,abort,<reason>
  I,status,k=v,...  I,diag,on|off,reason=..  I,steer_diag,ready,fw=..
  D,<t_ms>,<duty>,<pos 0.1deg>,<rate deg/s>,<cur A>,<vin V>,<state>,<ref 0.1deg>   (50 Hz)
  S,<cmd>,k=v,...                summary
  K                              keepalive (firmware USB TX workaround) — ignored, only in raw.log

Every run writes into --out (default ./diag-logs/<timestamp>/): raw.log (every line received),
one CSV per experiment (D rows) and summaries.csv (one row per S line).

Examples:
  python3 tools/steer_diag.py status
  python3 tools/steer_diag.py sweep-breakaway -n 3
  python3 tools/steer_diag.py sweep-pulse --duties 120,140,160 --ms 30,50,70,100 -n 2
  python3 tools/steer_diag.py center -n 6 --offset 10
  python3 tools/steer_diag.py ramp R --max 200
  python3 tools/steer_diag.py hold L 180 800
  python3 tools/steer_diag.py raw
"""

import argparse
import csv
import datetime
import glob
import os
import statistics
import sys
import threading
import time

try:
    import serial  # pyserial
except ImportError:  # pragma: no cover - environment dependent
    serial = None

BAUD = 115200
D_HEADER = ["t_ms", "duty", "pos_01deg", "rate_degs", "cur_a", "vin_v", "state", "ref_01deg"]


def die(msg, code=1):
    print("error: " + msg, file=sys.stderr)
    sys.exit(code)


def parse_kv(fields):
    out = {}
    for f in fields:
        if "=" in f:
            k, v = f.split("=", 1)
            out[k] = v
    return out


def is_noise(text):
    """Firmware keepalive line (`K`, see include/SteerDiag.h) or an empty line."""
    return text == "" or text == "K"


def fnum(v, default=float("nan")):
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


class Diag:
    def __init__(self, port, out_dir, verbose=False):
        if serial is None:
            die("pyserial is not installed: python3 -m pip install --user pyserial")
        self.out_dir = out_dir
        os.makedirs(out_dir, exist_ok=True)
        self.raw = open(os.path.join(out_dir, "raw.log"), "a", buffering=1)
        self.summary_rows = []
        self.exp_count = 0
        self.verbose = verbose
        self.buf = b""
        # Open without toggling DTR/RTS: on the ESP32-S3 USB-Serial/JTAG an RTS/DTR edge
        # can reset the chip. The levels are set on a port-less Serial() so pyserial applies
        # them at open(). macOS may still reset the board on open; connect() then waits
        # until it answers again and reports healthy.
        s = serial.Serial()
        s.baudrate = BAUD
        s.timeout = 0.05
        s.dtr = False
        s.rts = False
        s.port = port
        try:
            s.open()
        except serial.SerialException as exc:
            die("cannot open %s: %s" % (port, exc))
        self.ser = s

    # --- low level ---------------------------------------------------------
    def send(self, line):
        self.raw.write("%.3f > %s\n" % (time.time(), line))
        self.ser.write((line + "\n").encode("ascii"))
        self.ser.flush()

    def readline(self, timeout=0.05):
        """Next meaningful line, or None on timeout. Keepalive `K` lines and blank lines are
        logged to raw.log but never returned, so no parser / wait / display ever sees them."""
        end = time.time() + timeout
        while True:
            if b"\n" in self.buf:
                line, self.buf = self.buf.split(b"\n", 1)
                text = line.decode("utf-8", "replace").strip()
                self.raw.write("%.3f < %s\n" % (time.time(), text))
                if is_noise(text):
                    continue
                if self.verbose:
                    print("  < " + text)
                return text
            try:
                chunk = self.ser.read(256)
            except serial.SerialException as exc:
                die("serial read failed (board reset or unplugged?): %s" % exc)
            if chunk:
                self.buf += chunk
                continue
            if time.time() >= end:
                return None

    def wait_for(self, pred, timeout):
        end = time.time() + timeout
        while time.time() < end:
            line = self.readline(min(0.1, max(0.0, end - time.time())))
            if line is None:
                continue
            if line.startswith("E,abort"):
                print("  ! " + line)
            if pred(line):
                return line
        return None

    # --- protocol ----------------------------------------------------------
    def connect(self, timeout=15.0):
        """Wait for the banner or a status reply (handles a reset on port open)."""
        end = time.time() + timeout
        while time.time() < end:
            self.send("status")
            line = self.wait_for(lambda l: l.startswith("I,status") or l.startswith("I,steer_diag,ready"), 1.0)
            if line is None:
                continue
            if line.startswith("I,steer_diag,ready"):
                continue  # just booted: ask again for a status line
            return self.parse_status(line)
        die("no answer from the firmware within %.0f s (is it >= 1.5.12, is this the right port?)" % timeout)

    def wait_healthy(self, timeout=10.0):
        """Poll status until sensor=1 and drv=1 (the first ~1-2 s after a reset read 0).
        Returns (status dict, healthy)."""
        end = time.time() + timeout
        st = {}
        while True:
            self.send("status")
            line = self.wait_for(lambda l: l.startswith("I,status"), 1.0)
            if line is not None:
                st = self.parse_status(line)
                if st.get("sensor") == "1" and st.get("drv") == "1":
                    return st, True
            if time.time() >= end:
                return st, False
            time.sleep(0.25)

    @staticmethod
    def parse_status(line):
        return parse_kv(line.split(",")[2:])

    def status(self):
        for _ in range(2):  # 3 s, one resend, then fail
            self.send("status")
            line = self.wait_for(lambda l: l.startswith("I,status"), 3.0)
            if line is not None:
                return self.parse_status(line)
        die("no status reply (3 s, resent once)")

    def diag_on(self):
        self.send("diag on")
        line = self.wait_for(lambda l: l == "OK diag_on" or l.startswith("E,diag_on"), 3.0)
        if line != "OK diag_on":
            die("diag on refused: %s" % line)

    def diag_off(self):
        self.send("diag off")
        self.wait_for(lambda l: l == "OK diag_off", 3.0)

    def stop(self):
        self.send("stop")
        self.wait_for(lambda l: l == "OK stop", 2.0)

    def experiment(self, cmd, label=None, timeout=25.0):
        """Run one firmware experiment; returns (summary dict or None, rows, error or None)."""
        name = cmd.split()[0].lower()
        self.send(cmd)
        line = self.wait_for(lambda l: l == "OK " + name or l.startswith("E,"), 3.0)
        if line is None:
            return None, [], "no_reply"
        if line.startswith("E,"):
            return None, [], line
        rows = []
        summary, error = None, None
        end = time.time() + timeout
        while time.time() < end:
            l = self.readline(0.2)
            if l is None:
                continue
            if l.startswith("D,"):
                f = l.split(",")
                if len(f) == 9:
                    rows.append(f[1:])
            elif l.startswith("S," + name + ","):
                summary = parse_kv(l.split(",")[2:])
                break
            elif l.startswith("E,abort"):
                error = l
                break
        else:
            error = "timeout"
            self.stop()
        self.exp_count += 1
        tag = label or cmd.replace(" ", "_").replace("=", "")
        path = os.path.join(self.out_dir, "%03d_%s.csv" % (self.exp_count, tag))
        with open(path, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["# " + cmd])
            w.writerow(D_HEADER)
            w.writerows(rows)
        if summary is not None:
            row = {"n": self.exp_count, "cmd": cmd, "kind": name}
            row.update(summary)
            self.summary_rows.append(row)
            self.write_summaries()
        return summary, rows, error

    def write_summaries(self):
        keys = []
        for r in self.summary_rows:
            for k in r:
                if k not in keys:
                    keys.append(k)
        with open(os.path.join(self.out_dir, "summaries.csv"), "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=keys)
            w.writeheader()
            w.writerows(self.summary_rows)

    def angle(self):
        return fnum(self.status().get("angle"))

    def close(self):
        try:
            self.ser.close()
        finally:
            self.raw.close()


# --- helpers ---------------------------------------------------------------
def find_port():
    ports = sorted(glob.glob("/dev/cu.usbmodem*"))
    if not ports:
        die("no /dev/cu.usbmodem* port found — plug the board in or pass --port")
    if len(ports) > 1:
        print("several ports found, using %s (others: %s)" % (ports[0], ", ".join(ports[1:])))
    return ports[0]


def table(headers, rows):
    cells = [[str(c) for c in r] for r in rows]
    widths = [max([len(h)] + [len(r[i]) for r in cells]) for i, h in enumerate(headers)]
    print("  ".join(h.rjust(w) for h, w in zip(headers, widths)))
    print("  ".join("-" * w for w in widths))
    for r in cells:
        print("  ".join(c.rjust(w) for c, w in zip(r, widths)))


def stats(values):
    v = [x for x in values if x == x]
    if not v:
        return "-"
    if len(v) == 1:
        return "%.2f" % v[0]
    return "%.2f (%.2f..%.2f)" % (statistics.mean(v), min(v), max(v))


def print_status(st):
    print("diag=%s angle=%s° rate=%s°/s limits=%s..%s° cal=%s sensor=%s vesc=%s fault=%s "
          "cur=%sA vin=%sV speed=%skm/h(valid=%s) loop=%s fw=%s" % (
              st.get("diag"), st.get("angle"), st.get("rate"), st.get("limL"), st.get("limR"),
              st.get("cal"), st.get("sensor"), st.get("drv"), st.get("fault"), st.get("cur"),
              st.get("vin"), st.get("speed"), st.get("speedValid"), st.get("loop"), st.get("fw")))


def ramp_cmd(direction, a):
    return "ramp %s max=%d step=%d stepms=%d start=%d" % (direction, a.max, a.step, a.step_ms, a.start)


def show_summary(kind, s, err):
    if err:
        print("  %s: %s" % (kind, err))
    elif s:
        print("  " + " ".join("%s=%s" % kv for kv in s.items()))


def recenter(d, limit_deg):
    """goto 0 when the wheel has walked further than limit_deg (keeps sweeps off the locks)."""
    a = d.angle()
    if a == a and abs(a) > limit_deg:
        print("  recentring from %.1f°" % a)
        d.experiment("goto 0", label="recenter")


# --- subcommands -----------------------------------------------------------
def cmd_status(d, a):
    pass  # main() already printed the healthy status


def cmd_stop(d, a):
    d.stop()
    print("stopped")


def cmd_single(d, a):
    d.diag_on()
    if a.cmd == "ramp":
        line = ramp_cmd(a.dir.upper(), a)
    elif a.cmd in ("pulse", "hold"):
        line = "%s %s %d %d" % (a.cmd, a.dir.upper(), a.duty, a.ms)
    else:
        line = "goto %.1f" % a.deg
    s, rows, err = d.experiment(line)
    print("%s: %d log rows" % (line, len(rows)))
    show_summary(a.cmd, s, err)


def cmd_sweep_breakaway(d, a):
    d.diag_on()
    res = {"L": [], "R": []}
    rows = []
    for i in range(a.n):
        for direction in ("L", "R"):
            recenter(d, a.recenter_deg)
            s, _, err = d.experiment(ramp_cmd(direction, a), label="ramp_%s_%d" % (direction, i + 1))
            if err or not s:
                print("  ramp %s #%d: %s" % (direction, i + 1, err))
                rows.append([direction, i + 1, "-", "-", "-", "-", err or "?"])
                continue
            res[direction].append(s)
            rows.append([direction, i + 1, s.get("break_duty"), s.get("break_cur"), s.get("break_ms"),
                         s.get("coast"), s.get("result")])
            print("  ramp %s #%d: break_duty=%s cur=%sA coast=%s°" % (
                direction, i + 1, s.get("break_duty"), s.get("break_cur"), s.get("coast")))
            time.sleep(a.pause)
    print()
    table(["dir", "run", "break_duty", "cur_A", "t_ms", "coast_deg", "result"], rows)
    print()
    agg = []
    for direction in ("L", "R"):
        ok = [s for s in res[direction] if s.get("result") == "moved"]
        agg.append([direction, len(ok), stats([fnum(s.get("break_duty")) for s in ok]),
                    stats([fnum(s.get("break_cur")) for s in ok]), stats([fnum(s.get("coast")) for s in ok])])
    table(["dir", "moved", "break_duty mean (min..max)", "cur_A", "coast_deg"], agg)


def cmd_sweep_pulse(d, a):
    d.diag_on()
    duties = [int(x) for x in a.duties.split(",") if x]
    lengths = [int(x) for x in a.ms.split(",") if x]
    results = {}
    for duty in duties:
        for ms in lengths:
            for i in range(a.n):
                for direction in ("L", "R"):
                    recenter(d, a.recenter_deg)
                    line = "pulse %s %d %d" % (direction, duty, ms)
                    s, _, err = d.experiment(line, label="pulse_%s_%d_%d_%d" % (direction, duty, ms, i + 1))
                    if err or not s:
                        print("  %s: %s" % (line, err))
                        continue
                    # degrees in the drive direction (positive = moved the commanded way)
                    sign = 1.0 if direction == "R" else -1.0
                    results.setdefault((duty, ms, direction), []).append(
                        (sign * fnum(s.get("moved_end")), fnum(s.get("coast")), fnum(s.get("peak_cur"))))
                    print("  %s: moved %.1f° (coast %s°)" % (line, sign * fnum(s.get("moved_end")), s.get("coast")))
                    time.sleep(a.pause)
    print()
    rows = []
    for duty in duties:
        for ms in lengths:
            row = [duty, ms]
            for direction in ("L", "R"):
                r = results.get((duty, ms, direction), [])
                row += [stats([x[0] for x in r]), stats([x[1] for x in r])]
            rows.append(row)
    table(["duty", "ms", "L deg/pulse", "L coast", "R deg/pulse", "R coast"], rows)
    print("\nTarget for the web nudge: ~0.3–0.5° per tap in both directions.")


def cmd_center(d, a):
    d.diag_on()
    rows = []
    for i in range(a.n):
        start = a.offset if i % 2 == 0 else -a.offset
        s0, _, err0 = d.experiment("goto %.1f" % start, label="pre_%+.0f_%d" % (start, i + 1))
        if err0 or not s0:
            print("  pre-position to %.1f°: %s" % (start, err0))
            continue
        s, _, err = d.experiment("goto 0", label="center_from_%+.0f_%d" % (start, i + 1))
        if err or not s:
            print("  goto 0 from %.1f°: %s" % (start, err))
            rows.append([i + 1, s0.get("end"), "-", "-", "-", "-", "-", err])
            continue
        rows.append([i + 1, s.get("start"), s.get("overshoot"), s.get("residual"), s.get("end"),
                     s.get("t_stop"), s.get("restarts"), s.get("result")])
        print("  from %s°: overshoot %s°, residual %s°" % (s.get("start"), s.get("overshoot"), s.get("residual")))
        time.sleep(a.pause)
    print()
    table(["run", "from_deg", "overshoot_deg", "residual_deg", "end_deg", "t_stop_ms", "restarts", "result"], rows)
    res = [fnum(r[3]) for r in rows if r[3] != "-"]
    if res:
        print("\nresidual |mean| %.2f°, max |res| %.2f° (park window 0.4°, restart 1.0°)" % (
            statistics.mean([abs(x) for x in res]), max(abs(x) for x in res)))


def cmd_raw(d, a):
    print("raw mode: type firmware commands (diag on, status, ramp R, ...); Ctrl-D / Ctrl-C to quit")
    stop = threading.Event()

    def reader():
        while not stop.is_set():
            line = d.readline(0.1)
            if line is not None:
                print(line)

    t = threading.Thread(target=reader, daemon=True)
    t.start()
    try:
        for line in sys.stdin:
            line = line.strip()
            if line:
                d.send(line)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        t.join(0.5)


# --- main ------------------------------------------------------------------
def build_parser():
    p = argparse.ArgumentParser(
        prog="steer_diag.py",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", help="serial port (default: first /dev/cu.usbmodem*)")
    p.add_argument("--out", help="log directory (default ./diag-logs/<timestamp>/)")
    p.add_argument("--keep-on", action="store_true", help="leave diag mode on at the end (default: diag off)")
    p.add_argument("-v", "--verbose", action="store_true", help="echo every received line")
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("status", help="one status line")
    sub.add_parser("stop", help="cut the motor, cancel the experiment")

    def ramp_args(sp):
        sp.add_argument("--max", type=int, default=220, help="max duty (default 220)")
        sp.add_argument("--step", type=int, default=2, help="duty step (default 2)")
        sp.add_argument("--step-ms", type=int, default=150, help="ms per step (default 150)")
        sp.add_argument("--start", type=int, default=40, help="start duty (default 40)")

    sp = sub.add_parser("ramp", help="break-away ramp in one direction")
    sp.add_argument("dir", choices=["L", "R", "l", "r"])
    ramp_args(sp)
    for name, hlp in (("pulse", "one open-loop pulse (ms <= 1000)"), ("hold", "constant duty (ms <= 1500)")):
        sp = sub.add_parser(name, help=hlp)
        sp.add_argument("dir", choices=["L", "R", "l", "r"])
        sp.add_argument("duty", type=int)
        sp.add_argument("ms", type=int)
    sp = sub.add_parser("goto", help="closed-loop move to an angle, logged")
    sp.add_argument("deg", type=float)

    sp = sub.add_parser("sweep-breakaway", help="ramp L and R N times each; table of break-away duty, current, coast")
    sp.add_argument("-n", type=int, default=3, help="runs per direction (default 3)")
    sp.add_argument("--pause", type=float, default=0.5, help="s between runs")
    sp.add_argument("--recenter-deg", type=float, default=10.0, help="goto 0 first when |angle| exceeds this")
    ramp_args(sp)

    sp = sub.add_parser("sweep-pulse", help="duty x ms grid of pulses both ways; table of degrees per pulse")
    sp.add_argument("--duties", default="120,140,160", help="comma list (default 120,140,160)")
    sp.add_argument("--ms", default="30,50,70,100", help="comma list (default 30,50,70,100)")
    sp.add_argument("-n", type=int, default=2, help="repeats per cell and direction (default 2)")
    sp.add_argument("--pause", type=float, default=0.3, help="s between pulses")
    sp.add_argument("--recenter-deg", type=float, default=10.0, help="goto 0 first when |angle| exceeds this")

    sp = sub.add_parser("center", help="goto 0 N times from alternating +/-offset; residual and overshoot")
    sp.add_argument("-n", type=int, default=6, help="runs (default 6)")
    sp.add_argument("--offset", type=float, default=10.0, help="start angle magnitude, deg (default 10)")
    sp.add_argument("--pause", type=float, default=0.5, help="s between runs")

    sub.add_parser("raw", help="interactive pass-through")
    return p


def main():
    args = build_parser().parse_args()
    if serial is None:
        die("pyserial is not installed for %s — run: python3 -m pip install --user pyserial" % sys.executable)
    port = args.port or find_port()
    out = args.out or os.path.join("diag-logs", datetime.datetime.now().strftime("%Y%m%d-%H%M%S"))
    d = Diag(port, out, verbose=args.verbose)
    print("port %s, logs in %s" % (port, out))
    try:
        st = d.connect()
        if args.cmd not in ("raw", "stop"):
            st, ok = d.wait_healthy(10.0)
            print_status(st)
            if not ok:
                die("steering not healthy after 10 s (sensor=%s drv=%s)" % (st.get("sensor"), st.get("drv")))
        handler = {
            "status": cmd_status, "stop": cmd_stop, "ramp": cmd_single, "pulse": cmd_single,
            "hold": cmd_single, "goto": cmd_single, "sweep-breakaway": cmd_sweep_breakaway,
            "sweep-pulse": cmd_sweep_pulse, "center": cmd_center, "raw": cmd_raw,
        }[args.cmd]
        handler(d, args)
    except KeyboardInterrupt:
        print("\ninterrupted — stopping motor")
        d.stop()
    finally:
        if args.cmd not in ("status", "raw") and not args.keep_on:
            try:
                d.diag_off()
                st, ok = d.wait_healthy(10.0)
                print("final: " + ("" if ok else "(not healthy) "), end="")
                print_status(st)
            except Exception:  # noqa: BLE001 - best effort on the way out
                pass
        d.close()


if __name__ == "__main__":
    main()
