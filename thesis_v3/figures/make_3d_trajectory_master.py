#!/usr/bin/env python3
"""Master-arm-cohort counterpart to make_3d_trajectory_full.py: master hand,
simulated robot EE and REAL robot EE (forward kinematics on its own
encoders) for one Direct and one Shared-autonomy master-arm session (M2,
Object tracking -- the cohort's cleanest-tracking pair, both under 3 deg
joint RMS). No pick/place markers: this cohort's trail_extracted.csv has no
gripper/grip-command column at all (confirmed by inspection), so unlike the
VR cohort there is no real signal to mark a grasp from -- omitted rather
than invented, per this project's own standing rule against fabricating a
measurement.

Unlike the VR cohort's trail.csv, this cohort's trail_extracted.csv has no
ee_left/right_x/y/z (simulated EE Cartesian) column either -- only
sim_<arm>_j1-j7 and real_<arm>_j1-j7 (joint angles) plus master_<arm>_x/y/z
(already Cartesian). So BOTH simulated and real EE are FK'd here, not just
real. Needs the sim up for /compute_fk:

    python3 scripts/sim_session.py --stack teleop --keep-up -- true &
    python3 thesis_v3/figures/make_3d_trajectory_master.py
"""

import csv
import os
import time

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d import Axes3D  # noqa: E402,F401

import rclpy  # noqa: E402
from moveit_msgs.msg import RobotState  # noqa: E402
from moveit_msgs.srv import GetPositionFK  # noqa: E402
from rclpy.node import Node  # noqa: E402
import sys as _sys
_sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from session_paths import session_dir  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
OUT = os.path.join(os.path.dirname(__file__), "pilot", "track_3d_master.pdf")

GREY, BLUE, RED, GREEN = "0.35", "#2c6fbb", "#c1392b", "#3f9142"
plt.rcParams.update({
    "font.size": 9, "axes.titlesize": 10,
    "axes.spines.top": False, "axes.spines.right": False,
})

SESSIONS = [
    ("Direct", "20260902_132710"),   # M2, pick and place
    ("Shared autonomy", "20260902_132941"),
]
HAND = "right"
ARM_LEN = 7


class FK(Node):
    def __init__(self):
        super().__init__("track_3d_master_fk")
        self.cli = self.create_client(GetPositionFK, "/compute_fk")
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("waiting for /compute_fk ...")

    def ee(self, arm, q):
        rs = RobotState()
        rs.joint_state.name = ["%s_joint_%d" % (arm, i + 1) for i in range(ARM_LEN)]
        rs.joint_state.position = [float(v) for v in q]
        req = GetPositionFK.Request()
        req.header.frame_id = "world"
        req.fk_link_names = ["%s_end_effector_link" % arm]
        req.robot_state = rs
        fut = self.cli.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.005)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            return None
        p = r.pose_stamped[0].pose.position
        return (p.x, p.y, p.z)


NEEDED = (["master_%s_%s" % (HAND, a) for a in "xyz"] +
          ["sim_%s_j%d" % (HAND, j + 1) for j in range(ARM_LEN)] +
          ["real_%s_j%d" % (HAND, j + 1) for j in range(ARM_LEN)])


def load(session):
    path = os.path.join(session_dir(session), "trail_extracted.csv")
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return [r for r in rows if all((r.get(k) or "") != "" for k in NEEDED)]


def main():
    rclpy.init()
    fk = FK()

    fig = plt.figure(figsize=(7.2, 3.1))
    axes = []
    for panel, (label, session) in enumerate(SESSIONS):
        rows = load(session)
        mx = [float(r["master_%s_x" % HAND]) for r in rows]
        my = [float(r["master_%s_y" % HAND]) for r in rows]
        mz = [float(r["master_%s_z" % HAND]) for r in rows]

        step = 12
        idx = list(range(0, len(rows), step))
        sim_pts, real_pts = [], []
        for i in idx:
            qs = [float(rows[i]["sim_%s_j%d" % (HAND, j + 1)]) for j in range(ARM_LEN)]
            qr = [float(rows[i]["real_%s_j%d" % (HAND, j + 1)]) for j in range(ARM_LEN)]
            ps, pr = fk.ee(HAND, qs), fk.ee(HAND, qr)
            if ps is not None:
                sim_pts.append(ps)
            if pr is not None:
                real_pts.append(pr)
        sx, sy, sz = zip(*sim_pts)
        rx, ry, rz = zip(*real_pts)

        ax = fig.add_subplot(1, 2, panel + 1, projection="3d")
        ax.plot(mx, my, mz, color=BLUE, lw=0.5, alpha=0.55,
                label="operator's hand (master)" if panel == 0 else None)
        ax.plot(sx, sy, sz, color=RED, lw=1.2,
                label="simulated robot EE (FK)" if panel == 0 else None)
        ax.plot(rx, ry, rz, color=GREEN, lw=1.2, ls="--",
                label="real robot EE (FK)" if panel == 0 else None)
        ax.set_title(label)
        ax.set_xlabel("x (m)", labelpad=0)
        ax.set_ylabel("y (m)", labelpad=0)
        ax.set_zlabel("z (m)", labelpad=-2)
        ax.tick_params(labelsize=7)
        ax.locator_params(nbins=5)
        axes.append(ax)

    fig.subplots_adjust(left=0.03, right=0.98, bottom=0.16, top=0.90, wspace=0.15)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=3, frameon=False,
               fontsize=7, bbox_to_anchor=(0.52, -0.02))

    fig.savefig(OUT)
    print("wrote", OUT)

    fk.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
