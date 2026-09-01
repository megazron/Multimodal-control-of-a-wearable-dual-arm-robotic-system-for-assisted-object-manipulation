#!/usr/bin/env python3
"""IS THE TEENSY CONNECTED, AND IS IT SENDING FAST? TWO NUMBERS, NOT ONE.

    python3 scripts/measure_master_rate.py                    # 20 s on the board
    python3 scripts/measure_master_rate.py --port /dev/pts/5
    python3 scripts/measure_master_rate.py --self-test

WHY TWO NUMBERS. "Am I receiving the data fast" has two answers on this rig
and they are a factor of three apart:

    FRAME RATE    lines per second arriving on the wire
    UPDATE RATE   how often a channel's value actually CHANGES

The firmware prints one decimal place and smooths before printing, so a
channel moving slowly crosses a printable boundary only occasionally and
repeats its previous value in between. The wire is busy; the information is
not. Every statistic computed across TRANSMITTED ROWS rather than DISTINCT
SENSOR UPDATES has been wrong on this project for that reason -- it is how a
noise floor of exactly zero was measured on all fourteen channels, and how two
channel verdicts stood incorrectly for weeks.

So this reports both, per channel, and the ratio between them. A high frame
rate with a low update rate is not a healthy link; it is a link carrying
almost nothing, and it looks identical to a healthy one if you only count
lines.

IT TAKES THE PORT EXCLUSIVELY and refuses if something already holds it,
because the arm's node claims the port with the same lock and two readers
split the stream between them -- which invalidated a full day of measurements
once.
"""
from __future__ import annotations

import argparse
import fcntl
import json
import os
import re
import statistics
import sys
import time

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CHAN = re.compile(r"(k[12]j[1-7]):(-?\d+(?:\.\d+)?)")
IMU = re.compile(r"(K[12]IMU):([-\d.,]+)")


def open_port(path, baud=115200):
    """Open exclusively, or say who has it. Never share the stream."""
    import serial
    try:
        ser = serial.Serial(path, baud, timeout=1.0)
    except Exception as exc:                                 # noqa: BLE001
        raise SystemExit(
            "cannot open %s: %s\n"
            "  * is the board attached to this machine?  On WSL the Teensy\n"
            "    reaches Linux over usbip and must be attached explicitly:\n"
            "        usbipd.exe list                 # find the 16c0: device\n"
            "        usbipd.exe attach --wsl --busid <id>\n"
            "  * is a stack already running?  master_pose_node holds this\n"
            "    port exclusively and only one reader is allowed."
            % (path, exc))
    try:
        fcntl.flock(ser.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        ser.close()
        raise SystemExit(
            "%s is open but ANOTHER PROCESS HOLDS THE LOCK.\n"
            "  That is almost certainly master_pose_node. Two readers split\n"
            "  the stream between them and both see half the frames, which\n"
            "  is indistinguishable from a board sending at half rate.\n"
            "  Stop the stack, or measure from inside it." % path)
    return ser


def collect(ser, seconds):
    """Read for `seconds` and keep every line with its arrival time."""
    rows, t0 = [], time.monotonic()
    ser.reset_input_buffer()
    while time.monotonic() - t0 < seconds:
        raw = ser.readline()
        if not raw:
            continue
        rows.append((time.monotonic(), raw.decode("utf-8", "replace").strip()))
    return rows, time.monotonic() - t0


def analyse(rows, elapsed):
    """Frame rate, gap distribution, and the per-channel UPDATE rate."""
    good = [(t, s) for t, s in rows if CHAN.search(s)]
    bad = len(rows) - len(good)
    out = {
        "seconds": round(elapsed, 2),
        "lines": len(rows),
        "parsed": len(good),
        "malformed": bad,
        "frame_hz": round(len(good) / elapsed, 2) if elapsed else 0.0,
        "bytes_per_s": round(sum(len(s) + 1 for _t, s in rows) / elapsed, 0)
        if elapsed else 0.0,
    }
    if len(good) < 4:
        out["verdict"] = "too few frames to say anything"
        return out

    gaps = [b[0] - a[0] for a, b in zip(good, good[1:])]
    gaps_ms = sorted(g * 1000.0 for g in gaps)
    out["gap_ms"] = {
        "median": round(statistics.median(gaps_ms), 2),
        "p95": round(gaps_ms[int(0.95 * (len(gaps_ms) - 1))], 2),
        "max": round(gaps_ms[-1], 2),
    }
    # A STALL IS IN THE TAIL, NOT THE MEAN. A link that delivers perfectly for
    # a second and then pauses for 300 ms has a fine average rate and is not
    # usable for teleoperation.
    out["stalls_over_100ms"] = sum(1 for g in gaps_ms if g > 100.0)

    # ---- per channel: how often does the VALUE change? -----------------
    series = {}
    for _t, s in good:
        for name, val in CHAN.findall(s):
            series.setdefault(name, []).append(float(val))
    chans = {}
    for name, vals in sorted(series.items()):
        changes = sum(1 for a, b in zip(vals, vals[1:]) if a != b)
        uniq = len(set(vals))
        chans[name] = {
            "samples": len(vals),
            "distinct_values": uniq,
            "update_hz": round(changes / elapsed, 2) if elapsed else 0.0,
            "carried_fraction": round(1.0 - changes / max(len(vals) - 1, 1), 3),
            "span": round(max(vals) - min(vals), 3),
            "always_zero": all(v == 0.0 for v in vals),
        }
    out["channels"] = chans
    live = [c["update_hz"] for c in chans.values() if not c["always_zero"]]
    out["update_hz_median_over_live_channels"] = (
        round(statistics.median(live), 2) if live else 0.0)
    out["channels_always_zero"] = sorted(
        n for n, c in chans.items() if c["always_zero"])
    out["information_ratio"] = (
        round(out["update_hz_median_over_live_channels"] / out["frame_hz"], 3)
        if out["frame_hz"] else 0.0)
    return out


def report(a):
    print("MASTER SERIAL LINK")
    print("  %d lines in %.1f s -- %.1f frames/s, %.0f bytes/s, %d malformed"
          % (a["lines"], a["seconds"], a["frame_hz"], a["bytes_per_s"],
             a["malformed"]))
    if "gap_ms" not in a:
        print("  %s" % a.get("verdict", "no data"))
        return
    g = a["gap_ms"]
    print("  gap between frames: median %.1f ms, 95th %.1f ms, worst %.1f ms"
          % (g["median"], g["p95"], g["max"]))
    if a["stalls_over_100ms"]:
        print("  *** %d gap(s) over 100 ms -- the link stalls ***"
              % a["stalls_over_100ms"])
    print()
    print("  %-8s %8s %8s %9s %8s" %
          ("channel", "frames", "updates", "carried", "span"))
    for n, c in a["channels"].items():
        print("  %-8s %8d %7.1f%s %8.0f%% %8.1f%s"
              % (n, c["samples"], c["update_hz"], "Hz",
                 100 * c["carried_fraction"], c["span"],
                 "   ALWAYS ZERO" if c["always_zero"] else ""))
    print()
    print("  FRAME RATE  %.1f Hz   -- lines arriving" % a["frame_hz"])
    print("  UPDATE RATE %.1f Hz   -- median over channels that ever move"
          % a["update_hz_median_over_live_channels"])
    print("  Only %.0f%% of the frames carry a new reading. The rest repeat "
          "the previous" % (100 * a["information_ratio"]))
    print("  value, because the firmware prints one decimal and smooths "
          "before printing.")
    if a["channels_always_zero"]:
        print()
        print("  %d channel(s) never left zero: %s"
              % (len(a["channels_always_zero"]),
                 ", ".join(a["channels_always_zero"])))
        print("  That is a reading of the WIRING, not of the operator -- hold "
              "the master still")
        print("  and it looks the same, so this says nothing until somebody "
              "moves those joints.")


def self_test():
    """Against the virtual board, whose rates are SET rather than observed."""
    import subprocess
    import tempfile
    ok = fail = 0

    def check(name, cond, got=""):
        nonlocal ok, fail
        if cond:
            ok += 1
            print("  ok    %s %s" % (name, got))
        else:
            fail += 1
            print("  FAIL  %s %s" % (name, got))

    want_frame = 50.0

    def run(mode, secs=8.0, extra=()):
        with tempfile.TemporaryDirectory() as d:
            pf = os.path.join(d, "port")
            p = subprocess.Popen(
                [sys.executable,
                 os.path.join(WS, "scripts", "virtual_teensy.py"),
                 "--mode", mode, "--duration", "30",
                 "--frame-hz", str(want_frame), "--portfile", pf] + list(extra),
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            try:
                for _ in range(120):
                    if os.path.exists(pf) and open(pf).read().strip():
                        break
                    time.sleep(0.1)
                port = open(pf).read().strip()
                if not port:
                    return None, None
                import serial
                ser = serial.Serial(port, 115200, timeout=1.0)
                rows, el = collect(ser, secs)
                ser.close()
                return analyse(rows, el), port
            finally:
                p.terminate()
                p.wait(timeout=5)

    # ---- A MOVING BOARD. Real recorded rows, so the channels change.
    a, port = run("replay")
    check("the virtual board came up", a is not None, port or "")
    if a is None:
        print("\n%d checks, %d failed" % (ok + fail + 1, fail + 1))
        return 1
    # The frame rate is checked on the IDLE board below, not here: replay
    # reads recorded rows and runs slower than the rate it is asked for, so
    # asserting the set rate against it would be a test of the fixture.
    check("a moving board gives a sane frame rate",
          20.0 < a["frame_hz"] < 200.0, "%.1f Hz" % a["frame_hz"])
    # THE CHECK THAT MATTERS. A probe that reported the frame rate as the
    # update rate would pass every other test here and be worthless, because
    # conflating those two is the exact defect this program exists to expose.
    u = a["update_hz_median_over_live_channels"]
    check("the update rate is measured SEPARATELY from the frame rate",
          0.0 < u < a["frame_hz"],
          "%.1f Hz update against %.1f Hz frames" % (u, a["frame_hz"]))
    check("the carried fraction is reported",
          all("carried_fraction" in c for c in a["channels"].values()))

    # ---- THE CONTROL. A perfectly still operator must give ZERO updates at
    # a full frame rate. If this reads anything above zero the movement was
    # manufactured downstream, which is the diagnosis the virtual board
    # exists for -- and it is also the case a frame-counting probe cannot
    # tell apart from a healthy link.
    b, _ = run("idle")
    check("a STILL board delivers frames at the rate it was SET to (%.0f Hz)"
          % want_frame,
          b is not None and abs(b["frame_hz"] - want_frame) < 6.0,
          "%.1f Hz" % (b["frame_hz"] if b else -1))
    check("and reports ZERO updates, not a healthy-looking rate",
          b is not None and b["update_hz_median_over_live_channels"] == 0.0,
          "%.1f Hz" % (b["update_hz_median_over_live_channels"] if b else -1))
    print("\n%d checks, %d failed" % (ok + fail, fail))
    return 1 if fail else 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--json", default="")
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args(argv)
    if a.self_test:
        return self_test()

    ser = open_port(a.port, a.baud)
    print("reading %s for %.0f s ..." % (a.port, a.seconds))
    try:
        rows, el = collect(ser, a.seconds)
    finally:
        ser.close()
    res = analyse(rows, el)
    res["port"] = a.port
    report(res)
    if a.json:
        with open(a.json, "w") as fh:
            json.dump(res, fh, indent=2)
        print("\nwrote %s" % a.json)
    return 0


if __name__ == "__main__":
    sys.exit(main())
