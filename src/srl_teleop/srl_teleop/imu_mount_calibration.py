#!/usr/bin/env python3
"""
imu_mount_calibration.py — estimate the wrist IMU's mount rotation
=============================================================================
The IMU is glued to the wrist at an unknown but FIXED orientation relative
to the FK tip frame. Everything in the IMU-aided pot repair depends on that
rotation, so it is measured here rather than assumed.

    normalize(accel)  ~=  R_mount @ fk_rotation(q).T @ g_hat

Hold the master arm in 8-10 varied STATIC postures. Each posture gives one
predicted gravity direction (from the pots, via FK) and one measured
gravity direction (from the accelerometer). Fitting the single rotation
that best aligns the two sets across all postures is Wahba's problem,
solved in closed form by Kabsch/SVD -- no iteration, no seed, no local
minima. See master_calibration.solve_imu_mount().

The per-posture residual is the diagnostic that matters. A few degrees is
a good glue joint. Large residuals do NOT mean "retry the fit" -- they mean
the FK, AXIS_MAP or the pot zeros are wrong, and nothing should be built on
top until that is resolved.

LIVE (needs master_pose_node streaming):
  ros2 run srl_teleop imu_mount_calibration --ros-args -p arm:=left
    -> hold a posture, press Enter to capture, 'w' to fit+write, 'q' to quit

OFFLINE (re-fit from a full_state_recorder CSV):
  python3 -m srl_teleop.imu_mount_calibration --csv <file.csv> [--arm left]
  add --write to store the result in config/imu_mount_<arm>.txt
"""
import argparse
import csv
import math
import sys

import numpy as np

from srl_teleop import master_calibration as mc

# Per-posture spread above which a "static" hold is not actually static.
# Only checkable in live mode, where many samples per posture are averaged.
DEFAULT_MAX_POT_STD_DEG = 1.5
DEFAULT_SAMPLES_PER_POSTURE = 40

# Two postures closer than this (largest load-bearing joint difference) are
# treated as the same posture recorded twice. See dedupe_postures().
DEFAULT_MIN_SEPARATION_DEG = 10.0


def arm_prefixes(arm):
    """CSV/serial naming: the left master arm is k1/left, the right is k2/right."""
    return ("k1", "left") if arm == "left" else ("k2", "right")


def screen_posture(raw, accel, label=""):
    """Shared admission test for a candidate calibration posture.

    Returns (ok, reason). A posture is only usable when every load-bearing
    pot is live and the accelerometer is reading essentially pure gravity.
    """
    dead = mc.zero_dropouts(raw)
    if dead:
        return False, "dead channel(s) %s" % ", ".join("j%d" % (i + 1) for i in dead)
    mag = float(np.linalg.norm(accel))
    if not mc.accel_magnitude_ok(accel):
        return False, "|accel| = %.3f g, outside 1 +/- %.2f g (not static)" % (
            mag, mc.ACCEL_GATE_G)
    return True, "|accel| = %.3f g" % mag


def _circ_diff_deg(a, b):
    """Shortest signed a-b in (-180, 180]; pots wrap at 0/360."""
    return ((a - b + 180.0) % 360.0) - 180.0


def posture_separation_deg(raw_a, raw_b):
    """How far apart two postures are, as the largest load-bearing joint
    difference. Used to reject near-duplicates."""
    return max(abs(_circ_diff_deg(float(raw_a[i]), float(raw_b[i])))
               for i in mc.LOAD_BEARING_IDX)


def dedupe_postures(candidates, min_separation_deg):
    """Drop postures that repeat one already accepted.

    Wahba's problem weights every sample equally, so N copies of the same
    posture pull the fit N times harder while adding no new information --
    and then agree with each other, which reads as a confident fit when it
    is really one constraint. A recording session that parks this arm while
    the other is being exercised produces exactly that. Keep the first of
    each cluster.
    """
    kept, dropped = [], []
    for cand in candidates:
        label, raw = cand[0], cand[3]
        twin = next((k for k in kept
                     if posture_separation_deg(raw, k[3]) < min_separation_deg), None)
        if twin is None:
            kept.append(cand)
        else:
            dropped.append((label, "duplicate of '%s' (max joint separation "
                                   "%.1f deg < %.1f)" % (
                                       twin[0],
                                       posture_separation_deg(raw, twin[3]),
                                       min_separation_deg)))
    return kept, dropped


def samples_from_csv(path, arm):
    """Load candidate postures from a full_state_recorder CSV.

    Yields (label, q_radians, accel, note) for accepted rows and reports
    the rejected ones, so a thin fit is never mistaken for a clean one.
    """
    jp, ap = arm_prefixes(arm)
    calib = mc.PotCalibration.load(arm)
    accepted, rejected = [], []

    with open(path, newline="") as fh:
        for row in csv.DictReader(fh):
            label = row.get("label", "").strip() or "(unlabelled)"
            try:
                raw = [float(row["raw_%sj%d" % (jp, i)]) for i in range(1, 8)]
                accel = [float(row["raw_%s_a%s" % (ap, c)]) for c in "xyz"]
            except (KeyError, TypeError, ValueError):
                rejected.append((label, "missing/blank pot or IMU columns"))
                continue
            ok, reason = screen_posture(raw, accel, label)
            if not ok:
                rejected.append((label, reason))
                continue
            accepted.append((label, calib.apply(raw), np.array(accel), raw))

    return accepted, rejected


def report(fit, rejected=(), source=""):
    """Print the fit and, most importantly, the per-posture residuals."""
    print()
    print("=" * 68)
    print("IMU MOUNT CALIBRATION%s" % (" -- %s" % source if source else ""))
    print("=" * 68)

    if rejected:
        print("\nRejected postures (%d):" % len(rejected))
        for label, reason in rejected:
            print("  %-24s %s" % (label, reason))

    print("\nGravity convention: g_hat = %s in the master FK base frame"
          % fit["g_hat_name"])
    alts = fit.get("alternatives", {})
    if len(alts) > 1:
        print("  (fitted both; rms by convention: %s)"
              % ", ".join("%s = %.2f deg" % (k, v) for k, v in sorted(alts.items())))

    print("\nR_mount (tip frame -> sensor frame):")
    for r in fit["R"]:
        print("   [ %+.6f  %+.6f  %+.6f ]" % tuple(r))
    print("  det = %+.9f" % np.linalg.det(fit["R"]))

    print("\nPer-posture residual (angle between predicted and measured gravity):")
    for label, res in zip(fit["labels"], fit["residuals_deg"]):
        flag = ""
        if res > 15.0:
            flag = "   <== BAD"
        elif res > 8.0:
            flag = "   <-- high"
        print("  %-24s %7.2f deg%s" % (label, res, flag))

    print("\n  postures : %d" % fit["n"])
    print("  rms      : %.2f deg" % fit["rms_deg"])
    print("  max      : %.2f deg" % fit["max_deg"])

    rms = fit["rms_deg"]
    print()
    if rms <= 5.0:
        print("VERDICT: good fit. The FK, AXIS_MAP and pot zeros are mutually")
        print("         consistent with the measured gravity directions.")
    elif rms <= 12.0:
        print("VERDICT: marginal. Usable for coarse work, but a joint recovery")
        print("         built on this inherits the error. Worth investigating.")
    else:
        print("VERDICT: BAD FIT. A single fixed rotation does NOT explain the")
        print("         data. Do not build joint recovery on this -- the FK,")
        print("         AXIS_MAP or the pot zeros are wrong. See notes below.")
    print("=" * 68)
    return rms


def run_offline(args):
    accepted, rejected = samples_from_csv(args.csv, args.arm)
    accepted, dupes = dedupe_postures(accepted, args.min_separation)
    rejected = list(rejected) + dupes
    accepted = [(lb, q, a) for lb, q, a, _ in accepted]
    if len(accepted) < 2:
        print("Only %d usable posture(s) in %s -- need at least 2 (8-10 wanted)."
              % (len(accepted), args.csv), file=sys.stderr)
        for label, reason in rejected:
            print("  rejected %-24s %s" % (label, reason), file=sys.stderr)
        return 1

    fit = mc.solve_imu_mount(accepted)
    report(fit, rejected, source=args.csv)

    if args.write:
        mount = mc.IMUMount(args.arm, fit["R"], fit["g_hat_name"], meta={
            "n": fit["n"], "rms_deg": fit["rms_deg"], "max_deg": fit["max_deg"],
            "labels": fit["labels"], "residuals_deg": fit["residuals_deg"],
            "source": args.csv,
        })
        mount.validate()
        print("\nWrote %s" % mount.save())
    else:
        print("\n(not written -- pass --write to store it)")
    return 0


# --------------------------------------------------------------------------
# Live capture. Kept import-light so the offline path works without rclpy.
# --------------------------------------------------------------------------

def run_live(argv):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray

    class MountCalNode(Node):
        def __init__(self):
            super().__init__("imu_mount_calibration")
            self.declare_parameter("arm", "left")
            self.declare_parameter("samples_per_posture", DEFAULT_SAMPLES_PER_POSTURE)
            self.declare_parameter("max_pot_std_deg", DEFAULT_MAX_POT_STD_DEG)
            self.arm = self.get_parameter("arm").value
            self.n_samples = int(self.get_parameter("samples_per_posture").value)
            self.max_std = float(self.get_parameter("max_pot_std_deg").value)

            self.calib = mc.PotCalibration.load(self.arm)
            self.buf = []
            self.postures = []
            self.create_subscription(
                Float64MultiArray, "/master_arm_raw_%s" % self.arm, self.on_raw, 50)
            self.get_logger().info(
                "Listening on /master_arm_raw_%s. Hold a posture, then press "
                "Enter here to capture it." % self.arm)

        def on_raw(self, msg):
            # [j1..j7 degrees, ax, ay, az]
            if len(msg.data) >= 10:
                self.buf.append(list(msg.data[:10]))
                if len(self.buf) > self.n_samples:
                    self.buf.pop(0)

        def capture(self, label):
            """Average the recent window, and refuse a posture that moved."""
            if len(self.buf) < self.n_samples:
                print("  only %d/%d samples buffered -- is master_pose_node "
                      "publishing?" % (len(self.buf), self.n_samples))
                return
            block = np.array(self.buf[-self.n_samples:])
            raw = block[:, :7].mean(axis=0)
            accel = block[:, 7:10].mean(axis=0)

            std = block[:, :7].std(axis=0)
            noisy = [i for i in mc.LOAD_BEARING_IDX if std[i] > self.max_std]
            if noisy:
                print("  NOT static: %s moved (std %s deg > %.1f). Hold still "
                      "and retry." % (
                          ", ".join("j%d" % (i + 1) for i in noisy),
                          ", ".join("%.2f" % std[i] for i in noisy), self.max_std))
                return

            ok, reason = screen_posture(raw, accel, label)
            if not ok:
                print("  rejected: %s" % reason)
                return
            self.postures.append((label, self.calib.apply(raw), accel))
            print("  captured '%s'  (%s)  -- %d posture(s) so far"
                  % (label, reason, len(self.postures)))

    rclpy.init(args=argv)
    node = MountCalNode()
    import threading
    spin = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    print("\nEnter = capture posture, 'w' = fit and write, 'q' = quit\n")
    rc = 0
    try:
        while True:
            cmd = input("posture %d> " % (len(node.postures) + 1)).strip().lower()
            if cmd == "q":
                break
            if cmd == "w":
                if len(node.postures) < 2:
                    print("  need at least 2 postures (8-10 wanted).")
                    continue
                fit = mc.solve_imu_mount(node.postures)
                report(fit, source="live capture, %d postures" % fit["n"])
                mount = mc.IMUMount(node.arm, fit["R"], fit["g_hat_name"], meta={
                    "n": fit["n"], "rms_deg": fit["rms_deg"],
                    "max_deg": fit["max_deg"], "labels": fit["labels"],
                    "residuals_deg": fit["residuals_deg"],
                    "source": "live capture",
                })
                mount.validate()
                print("\nWrote %s" % mount.save())
                break
            node.capture(cmd or "posture_%d" % (len(node.postures) + 1))
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


def main(args=None):
    argv = sys.argv[1:] if args is None else args
    if "--csv" in argv:
        p = argparse.ArgumentParser(prog="imu_mount_calibration")
        p.add_argument("--csv", required=True)
        p.add_argument("--arm", default="left", choices=["left", "right"])
        p.add_argument("--write", action="store_true")
        p.add_argument("--min-separation", type=float,
                       default=DEFAULT_MIN_SEPARATION_DEG,
                       help="drop postures closer than this (deg) to one "
                            "already accepted; 0 disables")
        return run_offline(p.parse_args(argv))
    return run_live(args)


if __name__ == "__main__":
    sys.exit(main())
