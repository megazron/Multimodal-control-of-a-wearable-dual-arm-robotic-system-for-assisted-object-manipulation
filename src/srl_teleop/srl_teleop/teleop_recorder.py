#!/usr/bin/env python3
"""
teleop_recorder.py — button-gated 50 Hz recording of a scripted pass
=============================================================================
The operator controls the pace, not a clock. For each segment:

    print the instruction  ->  WAITING FOR PRESS
    operator presses       ->  RECORDING (rows written)
    operator presses again ->  SEGMENT SAVED
    nothing is recorded between segments, so repositioning is free

Rows land ONLY between the two presses, so a slow deliberate sweep and a
scramble back to rest never end up in the same segment. That matters: the
gain matrix is regressed per segment, and un-gated repositioning motion
would otherwise be fitted as if it were signal.

BUTTON GATING
  Each arm's own button gates its segments. The physical->index mapping has
  never been confirmed (an old CSV showed btn2 moving during the left-arm
  prompt despite BUTTON1_PIN=2 being the left button), so it is MEASURED at
  the start of every run rather than assumed, and written to a sidecar
  <csv>.meta.json.

  The clutch segments are gated by the OTHER arm's button on purpose: the
  left button has to stay free to toggle the left clutch, and a press cannot
  mean both "toggle the clutch" and "end the segment".

  The buttons are firmware TOGGLES, so a press is a CHANGE in the reported
  value, not a level. Changes inside DEBOUNCE_S of the last accepted one are
  ignored so a single press cannot register twice.

REDO
  A press-and-release inside MIN_SEGMENT_S discards that segment and
  re-prompts for the same one.

Run:
  ros2 run srl_teleop teleop_recorder
"""
import csv
import json
import os
import sys
import threading
import time

# INTERVALS USE time.monotonic(). Under WSL the wall clock steps
# backwards on host resync - it produced a measured send latency of
# -2321 ms once. Wall-clock time.time() is kept ONLY where the value
# is a human-readable timestamp, never for a duration.

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformListener

RATE_HZ = 50.0
OUT_DIR = os.path.expanduser("~/kortex_ws/recordings")

DEBOUNCE_S = 0.30          # one physical press must not read as several
MIN_SEGMENT_S = 0.5        # shorter than this = deliberate abort, redo it

BOLD = "\033[1m"
DIM = "\033[2m"
GRN = "\033[32m"
YEL = "\033[33m"
CYN = "\033[36m"
RED = "\033[31m"
R = "\033[0m"

SWEEP = "slowly and smoothly, no rush"

# (label, gate_arm, instruction). gate_arm is whose BUTTON ends the segment.
PER_ARM = [
    ("rest",             "{A} arm: hold still at REST, hanging down, a few seconds."),
    ("A_down_to_up",     "{A} arm: start LOW, sweep UP to the top, " + SWEEP + "."),
    ("A_up_to_down",     "{A} arm: start HIGH, sweep DOWN to the bottom, " + SWEEP + "."),
    ("A_left_to_right",  "{A} arm: start far LEFT, sweep across to far RIGHT, " + SWEEP + "."),
    ("A_right_to_left",  "{A} arm: start far RIGHT, sweep across to far LEFT, " + SWEEP + "."),
    ("A_forward_to_back", "{A} arm: start FORWARD, sweep BACK behind you, " + SWEEP + "."),
    ("A_back_to_forward", "{A} arm: start BACK, sweep FORWARD away from you, " + SWEEP + "."),
    ("shoulder_rot",     "{A} arm: keep the hand where it is and TWIST the upper arm "
                         "about its own axis, back and forth. (This is the j1 test.)"),
    ("A_diag",           "{A} arm: start LOW-LEFT, sweep diagonally to HIGH-RIGHT, "
                         + SWEEP + "."),
]

CLUTCH = [
    ("clutch_engaged_move", "LEFT arm: clutch is ENGAGED. Move FORWARD and hold there."),
    ("clutch_disengage",    "Press the LEFT button ONCE to DISENGAGE the clutch, "
                            "then keep the LEFT arm still."),
    ("clutch_reposition",   "LEFT arm: move it somewhere DIFFERENT. The robot should "
                            "NOT follow."),
    ("clutch_reengage",     "Press the LEFT button ONCE to RE-ENGAGE, then keep still."),
    ("clutch_after",        "LEFT arm: hold still (measuring the re-engage jump)."),
]


def build_protocol(arms):
    """Every segment is gated by the OPPOSITE arm's button.

    An arm's own button also TOGGLES that arm's clutch, so gating a left-arm
    sweep with the left button disengages the left clutch at the very moment
    the sweep starts. Measured in the 2026-07-31 pass: most segments survived
    because the start and end presses cancelled out, but right_A_left_to_right
    spent 24% of its samples with the clutch disengaged -- i.e. a quarter of
    that sweep commanded a frozen pose and silently biased the regression
    toward zero gain. Gating from the other arm removes the coupling
    entirely.
    """
    other = {"left": "right", "right": "left"}
    seq = []
    for arm in arms:
        gate = other[arm] if other[arm] in arms else arm
        for label, instr in PER_ARM:
            seq.append((f"{arm}_{label}", gate, instr.format(A=arm.upper())))
    # Left-arm clutch test: gated by the right button for the same reason.
    gate = "right" if "right" in arms else arms[0]
    for label, instr in CLUTCH:
        seq.append((label, gate, instr))
    return seq


class Recorder(Node):
    def __init__(self):
        super().__init__("teleop_recorder")
        self.declare_parameter("arms", ["left", "right"])
        self.declare_parameter("out", "")
        self.arms = list(self.get_parameter("arms").value)

        self.cmd = {a: None for a in self.arms}
        self.raw = {a: None for a in self.arms}
        self.mstat = {a: None for a in self.arms}
        self.istat = {a: None for a in self.arms}
        self.fsr = None

        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)

        for a in self.arms:
            self.create_subscription(
                PoseStamped, f"/master_arm_pose_{a}",
                lambda m, k=a: self.cmd.__setitem__(k, m), 20)
            self.create_subscription(
                Float64MultiArray, f"/master_arm_raw_{a}",
                lambda m, k=a: self.raw.__setitem__(k, list(m.data)), 20)
            self.create_subscription(
                Float64MultiArray, f"/master_status_{a}",
                lambda m, k=a: self.mstat.__setitem__(k, list(m.data)), 20)
            self.create_subscription(
                Float64MultiArray, f"/ik_status_{a}",
                lambda m, k=a: self.istat.__setitem__(k, list(m.data)), 20)
        self.create_subscription(
            Float64MultiArray, "/master_fsr_buttons",
            lambda m: setattr(self, "fsr", list(m.data)), 20)

        os.makedirs(OUT_DIR, exist_ok=True)
        out = self.get_parameter("out").value
        self.path = out or os.path.join(
            OUT_DIR, "teleop_%s.csv" % time.strftime("%Y%m%d_%H%M%S"))
        self.fh = open(self.path, "w", newline="")
        self.w = csv.writer(self.fh)
        self.w.writerow(self.header())
        self.fh.flush()
        self.rows = 0
        self.pending = []

        self.btn_last = {1: None, 2: None}
        self.last_edge_t = 0.0
        self.button_index = {}          # arm -> 1 or 2
        self.tty = sys.stdout.isatty()

        # Spin on a background executor rather than calling spin_once() once
        # per recorded row. One callback per 20 ms cannot service eight
        # subscriptions plus /tf and /tf_static, and the TF listener is the
        # one that starves: it needs the static transforms before ANY lookup
        # succeeds. That produced a full recording with every *_ee_* column
        # empty, which silently destroys the gain matrix -- the whole point
        # of the capture. A dedicated spin thread keeps every callback fed.
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self)
        self._spin_thread = threading.Thread(target=self._exec.spin, daemon=True)
        self._spin_thread.start()

    # ------------------------------------------------------------ csv ---

    def header(self):
        h = ["t", "wall", "segment"]
        for a in self.arms:
            h += [f"{a}_cmd_{c}" for c in ("x", "y", "z", "qx", "qy", "qz", "qw")]
            h += [f"{a}_ee_{c}" for c in ("x", "y", "z")]
            h += [f"{a}_j{i}" for i in range(1, 8)]
            h += [f"{a}_a{c}" for c in ("x", "y", "z")]
            h += [f"{a}_g{c}" for c in ("x", "y", "z")]
            h += [f"{a}_{c}" for c in ("clutch", "scale", "elev", "azim",
                                       "reach", "valid", "dropouts", "elev_held")]
            h += [f"{a}_ik_{c}" for c in ("success", "fail", "direct",
                                          "slewed", "rejected")]
        h += ["fsr1", "fsr2", "btn1", "btn2"]
        return h

    def ee(self, arm):
        try:
            t = self.buf.lookup_transform(
                "world", f"{arm}_end_effector_link",
                rclpy.time.Time()).transform.translation
            return [t.x, t.y, t.z]
        except Exception:
            return ["", "", ""]

    def commit(self):
        """Flush the pending segment to disk. Called only on SEGMENT SAVED."""
        for r in self.pending:
            self.w.writerow(r)
            self.rows += 1
        self.fh.flush()
        n = len(self.pending)
        self.pending = []
        return n

    def discard(self):
        """Drop the pending segment. A redo must leave NO trace in the CSV --
        otherwise the aborted attempt is welded onto the good take under the
        same label, complete with a multi-second gap in the middle of what
        the analyser will treat as one continuous sweep."""
        n = len(self.pending)
        self.pending = []
        return n

    def row(self, t, segment):
        r = [round(t, 4), round(time.time(), 4), segment]
        for a in self.arms:
            c = self.cmd[a]
            if c is None:
                r += [""] * 7
            else:
                p, o = c.pose.position, c.pose.orientation
                r += [p.x, p.y, p.z, o.x, o.y, o.z, o.w]
            r += self.ee(a)
            raw = self.raw[a]
            # 13 = 7 pots + 3 accel + 3 gyro. Older streams sent 10; pad so a
            # mixed-version run still lines up with the header.
            r += (raw[:13] if raw and len(raw) >= 13
                  else (list(raw[:10]) + [""] * 3 if raw and len(raw) >= 10
                        else [""] * 13))
            ms = self.mstat[a]
            r += (ms[:8] if ms and len(ms) >= 8 else [""] * 8)
            ik = self.istat[a]
            r += (ik[:5] if ik and len(ik) >= 5 else [""] * 5)
        r += (self.fsr[:4] if self.fsr and len(self.fsr) >= 4 else [""] * 4)
        # Buffered, not written: the segment is only committed once the
        # operator ends it with a press long enough not to be a redo.
        self.pending.append(r)

    def cmd_xyz(self, arm):
        c = self.cmd.get(arm)
        if c is None:
            return None
        p = c.pose.position
        return (p.x, p.y, p.z)

    # --------------------------------------------------------- buttons ---

    def btn(self, idx):
        """Reported value of btn1/btn2, or None if no frame yet."""
        if not self.fsr or len(self.fsr) < 4:
            return None
        return self.fsr[1 + idx]        # data[2]=btn1, data[3]=btn2

    def poll_edge(self):
        """Return the index (1 or 2) that just changed, debounced, else None.

        The buttons are firmware toggles, so a press shows up as a CHANGE in
        the reported value rather than a level -- edge detection, not
        thresholding.
        """
        now = time.monotonic()
        for idx in (1, 2):
            v = self.btn(idx)
            if v is None:
                continue
            if self.btn_last[idx] is None:
                self.btn_last[idx] = v          # first sight: seed, don't fire
                continue
            if v != self.btn_last[idx]:
                self.btn_last[idx] = v
                if now - self.last_edge_t < DEBOUNCE_S:
                    continue                    # bounce, not a second press
                self.last_edge_t = now
                return idx
        return None

    def spin(self):
        """No-op: a background executor thread services all callbacks now.
        Kept so the call sites read the same."""
        return


def wait_for_topics(n, timeout=30.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        n.spin()
        if n.fsr is not None and all(n.cmd[a] is not None for a in n.arms):
            return True
        time.sleep(0.02)
    return False


def calibrate_buttons(n):
    """Measure which reported index each physical button drives."""
    print("\n" + "=" * 70)
    print(BOLD + "BUTTON MAPPING - measured, not assumed" + R)
    print("=" * 70)
    mapping = {}
    for side in ("left", "right"):
        print("\n  %sPress the %s button once, now.%s"
              % (YEL, side.upper(), R))
        idx = None
        while idx is None:
            n.spin()
            idx = n.poll_edge()
            time.sleep(0.01)
        used_by = {v: k for k, v in mapping.items()}
        if idx in used_by:
            print("  %sbtn%d changed again - it is already mapped to the %s "
                  "button. Both physical buttons appear to drive the same "
                  "channel; falling back to btn1=left, btn2=right.%s"
                  % (RED, idx, used_by[idx], R))
            return {"left": 1, "right": 2}, False
        mapping[side] = idx
        print("  %s-> %s button drives btn%d%s" % (GRN, side.upper(), idx, R))
    return mapping, True


def run_segment(n, label, gate_idx, gate_arm, instruction, t0):
    """One button-gated segment. Returns (rows, seconds, motion_m) or None
    if the operator aborted it for a redo."""
    print("\n" + "-" * 70)
    print(BOLD + label + R)
    print("  " + instruction)
    print("  %sWAITING FOR PRESS%s  - press the %s button to START recording."
          % (CYN, R, gate_arm.upper()))

    while True:
        n.spin()
        if n.poll_edge() == gate_idx:
            break
        time.sleep(0.01)

    start = time.monotonic()
    start_xyz = n.cmd_xyz(gate_arm)
    print("  %sRECORDING%s  - press the %s button again to END."
          % (GRN, R, gate_arm.upper()))
    dt = 1.0 / RATE_HZ
    last_report = 0.0

    while True:
        n.spin()
        n.row(time.monotonic() - t0, label)
        if n.poll_edge() == gate_idx:
            break
        el = time.monotonic() - start
        if el - last_report >= 0.5:
            last_report = el
            msg = "    %5.1f s   %5d rows" % (el, len(n.pending))
            if n.tty:
                print("\r" + msg, end="", flush=True)
            else:
                print(msg, flush=True)
        time.sleep(dt)

    if n.tty:
        print()
    secs = time.monotonic() - start
    end_xyz = n.cmd_xyz(gate_arm)
    motion = 0.0
    if start_xyz and end_xyz:
        motion = sum((a - b) ** 2 for a, b in zip(start_xyz, end_xyz)) ** 0.5

    if secs < MIN_SEGMENT_S:
        dropped = n.discard()
        print("  %sDISCARDED%s - %.2f s is shorter than %.1f s, treating that as "
              "a redo. %d buffered rows dropped, nothing written. "
              "Same segment again." % (YEL, R, secs, MIN_SEGMENT_S, dropped))
        return None

    wrote = n.commit()
    print("  %sSEGMENT SAVED%s  %.1f s, %d rows, master moved %.3f m"
          % (GRN, R, secs, wrote, motion))
    return wrote, secs, motion


def main(args=None):
    rclpy.init(args=args)
    n = Recorder()

    print("=" * 70)
    print(BOLD + "TELEOP RECORDER - button gated" + R)
    print("Recording to %s" % n.path)
    print("=" * 70)
    print("\nWaiting for master_pose_node topics...")
    if not wait_for_topics(n):
        print("%s!!! No /master_fsr_buttons or /master_arm_pose_* within 30 s.%s"
              % (RED, R))
        print("    Is master_pose_node running and the Teensy attached?")
        n.fh.close()
        n.destroy_node()
        rclpy.shutdown()
        return 1
    print("  ok - topics live.")

    mapping, measured = calibrate_buttons(n)

    proto = build_protocol(n.arms)
    print("\n" + "=" * 70)
    print("%d segments. You control the pace: press to start, press to stop."
          % len(proto))
    print("A press-and-release under %.1f s discards that segment and repeats it."
          % MIN_SEGMENT_S)
    print("=" * 70)

    t0 = time.monotonic()
    done = []
    mpath = n.path + ".meta.json"

    def write_meta():
        """Rewritten after EVERY segment, so a kill -TERM or a closed
        terminal cannot lose the button mapping and the segment index."""
        with open(mpath, "w") as fh:
            json.dump({
                "csv": n.path,
                "button_mapping": mapping,
                "button_mapping_measured": measured,
                "segments": [{"label": s[0], "rows": s[1], "seconds": s[2],
                              "motion_m": s[3]} for s in done],
            }, fh, indent=2)

    write_meta()
    try:
        i = 0
        while i < len(proto):
            label, gate_arm, instruction = proto[i]
            gate_idx = mapping.get(gate_arm, 1)
            res = run_segment(n, label, gate_idx, gate_arm, instruction, t0)
            if res is None:
                continue                    # redo the same segment
            done.append((label,) + res)
            write_meta()
            i += 1
    except KeyboardInterrupt:
        print("\n\ninterrupted - keeping what was recorded so far")
    finally:
        n.discard()          # a half-finished segment is never committed
        n.fh.close()
        write_meta()
        print("\n" + "=" * 70)
        print("Wrote %d rows across %d segments" % (n.rows, len(done)))
        print("  CSV  : %s" % n.path)
        print("  meta : %s" % mpath)
        print("=" * 70)
        n.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
