#!/usr/bin/env python3
"""A VIRTUAL TEENSY on a pty: replay real recorded frames, or hold still.

    python3 scripts/virtual_teensy.py --mode idle     --duration 90
    python3 scripts/virtual_teensy.py --mode replay   --duration 90
    python3 scripts/virtual_teensy.py --mode script   --duration 120

It creates a pty, prints its /dev/pts/N path, and streams frames in exactly
the wire format `master_pose_node.parse_arm()` expects:

    k1j1:<deg> ... k1j7:<deg> K1IMU:ax,ay,az,gx,gy,gz
    k2j1:<deg> ... k2j7:<deg> K2IMU:ax,ay,az,gx,gy,gz
    fsr1:<n>,fsr2:<n>,btn1:<0|1>,btn2:<0|1>

WHY THIS EXISTS. Every end-to-end claim about the master path -- degraded
mode, the radial fallback, clutch indexing, idle EE spread -- previously
needed a Teensy on the bench, so none of it could be checked between lab
visits, and the checks that WERE run had no ground truth to compare against.
Here the input is exactly known, which is the part hardware cannot give.

  idle    every channel pinned to its zero reference. A perfect operator
          holding perfectly still. Anything that moves downstream is
          manufactured by the software, and that is the whole diagnosis.
  replay  real rows from the 2026-08-06 capture, faults and all -- the
          incoherent jumps are the recorded ones, not synthesised.
  script  idle, plus scripted BUTTON presses on a timetable, for exercising
          the clutch state machine deterministically.

The pot update rate is deliberately SLOWER than the frame rate (the real
board streams ~50 Hz of lines carrying a 14.7-17 Hz sensor), because that
aliasing is what made 82-88% of recorded rows bit-identical and invalidated
every per-row statistic taken before it was understood.
"""
import argparse
import csv
import math
import os
import sys
import time

DEF_CAPTURE = ("recordings/trajectory_capture/capture_20260806_130205/"
               "all_segments.csv")


def zeros(arm):
    """Raw zero-reference degrees from config/master_zero_<arm>.txt."""
    path = os.path.join(os.path.dirname(__file__), "..", "config",
                        "master_zero_%s.txt" % arm)
    out = [0.0] * 7
    with open(path) as f:
        for line in f:
            if line.strip().startswith("joint_"):
                k, v = line.split(":", 1)
                out[int(k.strip().split("_")[1]) - 1] = float(v)
    return out


def a_rest(arm):
    """Accelerometer at rest. 1 g down the sensor's own z, which is what the
    hanging-arm zero capture records; elevation then reads -90 deg, the
    convention capture_zero.py asserts."""
    return (0.0, 0.0, 1.0)


def load_replay(path, limit=200000):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            try:
                lj = [float(r["l_j%d" % (i + 1)]) for i in range(7)]
                rj = [float(r["r_j%d" % (i + 1)]) for i in range(7)]
                la = [float(r["l_a%s" % c]) for c in "xyz"]
                ra = [float(r["r_a%s" % c]) for c in "xyz"]
                lg = [float(r["l_g%s" % c]) for c in "xyz"]
                rg = [float(r["r_g%s" % c]) for c in "xyz"]
            except (KeyError, ValueError, TypeError):
                continue
            rows.append((lj, rj, la, ra, lg, rg))
            if len(rows) >= limit:
                break
    return rows


def frame(lj, rj, la, ra, lg, rg, fsr1, fsr2, b1, b2):
    parts = [" ".join("k1j%d:%.2f" % (i + 1, lj[i]) for i in range(7)),
             "K1IMU:%.4f,%.4f,%.4f,%.4f,%.4f,%.4f" % (*la, *lg),
             " ".join("k2j%d:%.2f" % (i + 1, rj[i]) for i in range(7)),
             "K2IMU:%.4f,%.4f,%.4f,%.4f,%.4f,%.4f" % (*ra, *rg),
             "fsr1:%.0f,fsr2:%.0f,btn1:%d,btn2:%d" % (fsr1, fsr2, b1, b2)]
    return (" ".join(parts) + "\n").encode()


def parse_script(text):
    """"t:btn1:btn2,..." -> [(t, b1, b2)] sorted by time."""
    out = []
    for item in text.split(","):
        if not item.strip():
            continue
        t, b1, b2 = item.split(":")
        out.append((float(t), int(b1), int(b2)))
    return sorted(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", default="idle",
                    choices=["idle", "replay", "script"])
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--frame-hz", type=float, default=50.0)
    ap.add_argument("--sensor-hz", type=float, default=15.0)
    ap.add_argument("--capture", default=DEF_CAPTURE)
    ap.add_argument("--script", default="",
                    help='e.g. "5:1:0,6:0:0,12:1:0,13:0:0"')
    ap.add_argument("--reach-sweep", action="store_true",
                    help="drive left j7 in a slow triangle, to exercise the "
                         "radial fallback with a known input")
    ap.add_argument("--portfile", default="/tmp/virtual_teensy_port")
    ap.add_argument("--ctl", default="/tmp/virtual_teensy_ctl",
                    help="control file: write 'j7=+22' or 'btn2=1' to it and "
                         "the board obeys on the next frame. This is what "
                         "makes clutch indexing testable deterministically -- "
                         "a human at a mannequin cannot produce repeatable "
                         "button edges or repeatable arm excursions.")
    a = ap.parse_args()

    master, slave = os.openpty()
    name = os.ttyname(slave)
    with open(a.portfile, "w") as f:
        f.write(name)
    print("VIRTUAL TEENSY on %s  (mode=%s, %.0f Hz frames, %.0f Hz sensor)"
          % (name, a.mode, a.frame_hz, a.sensor_hz), flush=True)
    print("  serial_port:=%s" % name, flush=True)

    try:
        os.unlink(a.ctl)
    except OSError:
        pass
    ctl_off = {}                      # joint index -> extra degrees
    ctl_btn = {1: 0, 2: 0}

    def poll_ctl():
        """Read and consume one command. Deliberately last-write-wins and
        stateless: the verifier writes one instruction at a time."""
        try:
            with open(a.ctl) as f:
                txt = f.read().strip()
            os.unlink(a.ctl)
        except OSError:
            return
        for tok in txt.replace(";", ",").split(","):
            tok = tok.strip()
            if not tok or "=" not in tok:
                continue
            k, v = tok.split("=", 1)
            k = k.strip().lower()
            if k.startswith("btn"):
                ctl_btn[int(k[3:])] = int(float(v))
            elif k.startswith("j"):
                ctl_off[int(k[1:]) - 1] = float(v)

    lz, rz = zeros("left"), zeros("right")
    la0, ra0 = a_rest("left"), a_rest("right")
    rep = load_replay(a.capture) if a.mode == "replay" else []
    if a.mode == "replay" and not rep:
        print("  no replay rows -- falling back to idle", flush=True)
    script = parse_script(a.script) if a.script else []

    dt = 1.0 / a.frame_hz
    sensor_dt = 1.0 / a.sensor_hz
    t0 = time.monotonic()
    last_sensor = -1e9
    cur = None
    n = 0
    b1 = b2 = 0
    try:
        while True:
            now = time.monotonic()
            el = now - t0
            if el >= a.duration:
                break
            # The sensor updates SLOWER than the frame rate, so consecutive
            # frames repeat -- exactly as the real board does.
            if now - last_sensor >= sensor_dt:
                last_sensor = now
                if rep:
                    lj, rj, la, ra, lg, rg = rep[n % len(rep)]
                    n += 1
                else:
                    lj, rj = list(lz), list(rz)
                    la, ra = list(la0), list(ra0)
                    lg, rg = [0.0] * 3, [0.0] * 3
                if a.reach_sweep:
                    # Triangle on left j7 about its zero, +/-20 deg over 20 s.
                    ph = (el % 20.0) / 20.0
                    tri = 4.0 * abs(ph - 0.5) - 1.0
                    lj[6] = lz[6] + 20.0 * tri
                cur = (lj, rj, la, ra, lg, rg)
            poll_ctl()
            for t, x, y in script:
                if el >= t:
                    b1, b2 = x, y
            if ctl_btn[1] or ctl_btn[2]:
                b1, b2 = ctl_btn[1], ctl_btn[2]
            elif not script:
                b1, b2 = ctl_btn[1], ctl_btn[2]
            lj, rj, la, ra, lg, rg = cur
            if ctl_off:
                lj = list(lj)
                for i, off in ctl_off.items():
                    lj[i] = lz[i] + off
            os.write(master, frame(lj, rj, la, ra, lg, rg, 20, 20, b1, b2))
            time.sleep(max(0.0, dt - (time.monotonic() - now)))
    except (KeyboardInterrupt, OSError):
        pass
    finally:
        os.close(master)
        os.close(slave)
    print("  virtual teensy stopped after %.1f s" % (time.monotonic() - t0))
    return 0


if __name__ == "__main__":
    sys.exit(main())
