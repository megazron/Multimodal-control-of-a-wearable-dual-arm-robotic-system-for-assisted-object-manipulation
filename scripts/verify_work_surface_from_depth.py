#!/usr/bin/env python3
"""LIVE END-TO-END PROOF FOR U-3: depth -> a measured work-surface height.

    python3 scripts/verify_work_surface_from_depth.py      # needs the stack

`srl_experiments.work_surface` has had `set_measured()` since it was written
and no producer, so the table height was declared and never measured. The unit
tests in `src/srl_perception/test/test_surface_from_depth.py` prove the
arithmetic against constructed depth frames. THIS script proves the other
half: that the real node, on the real topics, through the real TF chain,
actually publishes a height and that `work_surface.check()` sees it.

HOW THE VIEW IS OBTAINED. Same mechanism as verify_scan_view.py: a static
`world -> scan_cam_<arm>` transform plus the mock camera bound to that frame.
From the HOME pose the wrist cameras do not frame the work surface at all, so
running this at home would measure nothing and prove nothing. The arm does
not move, which is a stronger guarantee than trusting that it arrived.

TWO CONTROLS, AND WITHOUT THEM THIS SCRIPT IS DECORATION:

  * the measured height must match clip_scene.TABLE_TOP, the height the mock
    was told to render, to within work_surface.TOL_M;
  * asked about a region the camera cannot see, the node must REFUSE and say
    so, rather than falling back to the declared height. That is the failure
    the declared/measured split exists to make visible, so a run where it
    cannot happen tests nothing.

WHAT THIS DOES NOT ESTABLISH. The mock renders geometry, not appearance. It
says the arithmetic, the frames and the plumbing are right. Depth noise,
holes, multipath, the real intrinsics and whether a real table's surface
returns depth at all remain UNMEASURED and need the real cameras.
"""

import json
import os
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "scripts"))
sys.path.insert(0, os.path.join(WS, "src", "srl_experiments"))

import find_scan_pose as FSP                                   # noqa: E402
from srl_experiments import work_surface as WSURF              # noqa: E402

POSES = os.path.join(WS, "recordings", "baselines", "scan_pose.json")
ARM = "left"
# The region the camera is asked about. The table spans y 0.10..0.72; this
# stays inside it so the answer cannot be contaminated by the floor.
GOOD_REGION = ([-0.60, 0.60], [0.15, 0.60])
# Somewhere the camera cannot see. The refusal must name the scan pose.
BLIND_REGION = ([4.0, 5.0], [4.0, 5.0])


class Sink(Node):
    def __init__(self):
        super().__init__("work_surface_probe")
        self.z = None
        self.status = []
        self.create_subscription(Float64, "/perception/work_surface_z",
                                 self._z, 1)
        self.create_subscription(String, "/perception/work_surface_status",
                                 self._s, 10)

    def _z(self, m):
        self.z = float(m.data)

    def _s(self, m):
        if not self.status or self.status[-1] != m.data:
            self.status.append(m.data)

    def spin(self, secs):
        t = time.time()
        while time.time() - t < secs and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)


def _run(env, arm, frame, region):
    return subprocess.Popen(
        ["python3", "-u", os.path.join(WS, "src", "srl_perception",
                                       "srl_perception", "work_surface_node.py"),
         "--ros-args", "-p", "arm:=" + arm, "-p", "frame_id:=" + frame,
         "-p", "region_x:=[%f,%f]" % tuple(region[0]),
         "-p", "region_y:=[%f,%f]" % tuple(region[1]),
         "-p", "rate_hz:=4.0"],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True)


def main():
    if not os.path.exists(POSES):
        print("no %s -- run scripts/find_scan_pose.py --save first" % POSES)
        return 2
    import clip_scene as CS
    truth = CS.TABLE_TOP

    d = json.load(open(POSES))["poses"]
    if ARM not in d:
        print("no scan pose for the %s arm" % ARM)
        return 2
    import numpy as np
    eye = np.asarray(d[ARM]["cam_xyz"], float)
    tgt = np.asarray(d[ARM]["target"], float)
    q = FSP._R_to_q(FSP._look_at(eye, tgt))
    frame = "scan_cam_%s" % ARM
    env = dict(os.environ, FASTDDS_BUILTIN_TRANSPORTS="SHM")

    procs = [
        subprocess.Popen(
            ["ros2", "run", "tf2_ros", "static_transform_publisher",
             "--x", str(eye[0]), "--y", str(eye[1]), "--z", str(eye[2]),
             "--qx", str(q[0]), "--qy", str(q[1]), "--qz", str(q[2]),
             "--qw", str(q[3]), "--frame-id", "world",
             "--child-frame-id", frame],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True),
        subprocess.Popen(
            ["python3", "-u", os.path.join(WS, "src", "srl_perception",
                                           "srl_perception",
                                           "mock_rgbd_camera.py"),
             "--ros-args", "-p", "arm:=" + ARM, "-p", "task:=t1",
             "-p", "frame_id:=" + frame],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True),
    ]
    rows = []
    try:
        time.sleep(3.0)
        # ---- 1. the measurement -------------------------------------
        node = _run(env, ARM, frame, GOOD_REGION)
        procs.append(node)
        rclpy.init()
        s = Sink()
        s.spin(20.0)
        got = s.z
        rows.append(("a height is published at all", got is not None,
                     "z = %s" % ("%.4f m" % got if got is not None
                                 else "NOTHING -- %s"
                                 % (s.status[-1] if s.status else "silent"))))
        if got is not None:
            err = abs(got - truth)
            rows.append(("it matches the height the mock RENDERS",
                         err <= WSURF.TOL_M,
                         "measured %.4f m against %.4f m, %+.1f mm "
                         "(tolerance %.0f mm)"
                         % (got, truth, (got - truth) * 1000,
                            WSURF.TOL_M * 1000)))
            # The consumer path: does work_surface actually take it up.
            WSURF.clear_measured()
            sub = WSURF.subscribe(s)
            s.spin(4.0)
            ok, msg = WSURF.check(require_measured=True)
            rows.append(("work_surface.check() sees it through subscribe()",
                         WSURF.source() == "measured",
                         "source=%s -- %s" % (WSURF.source(), msg[:90])))
            s.destroy_subscription(sub)
            WSURF.clear_measured()
        try:
            os.killpg(os.getpgid(node.pid), 15)
        except Exception:                                      # noqa: BLE001
            pass
        procs.remove(node)

        # ---- 2. THE CONTROL: a region the camera cannot see ----------
        s.z = None
        s.status.clear()
        blind = _run(env, ARM, frame, BLIND_REGION)
        procs.append(blind)
        s.spin(14.0)
        refused = [m for m in s.status if "NOT MEASURED" in m]
        rows.append(("CONTROL: a blind region REFUSES, and names the fix",
                     s.z is None and bool(refused)
                     and "scan pose" in refused[-1],
                     (refused[-1][:110] if refused
                      else "no refusal published; z = %s" % s.z)))
    finally:
        for p in procs:
            try:
                os.killpg(os.getpgid(p.pid), 15)
            except Exception:                                  # noqa: BLE001
                pass
        try:
            rclpy.shutdown()
        except Exception:                                      # noqa: BLE001
            pass

    print("=" * 72)
    print("WORK SURFACE FROM DEPTH -- the producer work_surface never had")
    print("=" * 72)
    bad = 0
    for label, ok, detail in rows:
        print("   %-48s %-4s %s" % (label, "OK" if ok else "FAIL", detail))
        bad += not ok
    print("\n%d of %d checks fail" % (bad, len(rows)))
    print("\nSTILL UNMEASURED, AND THE MOCK CANNOT REACH IT: depth noise, "
          "holes,\nmultipath, the real intrinsics, and whether a real table "
          "surface returns\ndepth at all. Those need the Kinova cameras.")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
