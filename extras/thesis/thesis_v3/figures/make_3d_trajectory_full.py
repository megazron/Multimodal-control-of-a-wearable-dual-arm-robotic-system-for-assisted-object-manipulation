#!/usr/bin/env python3
"""Master hand, simulated robot EE and REAL robot EE (forward kinematics on
its own encoders, not simulated) over one Direct and one Shared-autonomy
session, with picked/placed markers from the gripper's own logged
transitions. Needs the sim up for /compute_fk:

    python3 scripts/sim_session.py --stack teleop --keep-up -- true &
    python3 extras/thesis/thesis_v3/figures/make_3d_trajectory_full.py

Real EE has no recorded Cartesian counterpart (ee_left_x/y/z in trail.csv is
FK of the SIM joint state, confirmed by reading scripts/record_all.py), so
this FKs real_left_j1-j7 directly via /compute_fk, the same service and
pattern scripts/describe_pose.py uses.
"""

import csv
import math
import os
import sys
import time

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d import Axes3D  # noqa: E402,F401

import rclpy  # noqa: E402
from moveit_msgs.msg import RobotState  # noqa: E402
from moveit_msgs.srv import GetPositionFK  # noqa: E402
from rclpy.node import Node  # noqa: E402

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), *[".."] * 4))
OUT = os.path.join(os.path.dirname(__file__), "pilot", "track_3d_full.pdf")

GREY, BLUE, RED, GREEN = "0.35", "#2c6fbb", "#c1392b", "#3f9142"
plt.rcParams.update({
    "font.size": 9, "axes.titlesize": 10,
    "axes.spines.top": False, "axes.spines.right": False,
})

SESSIONS = [
    ("Direct", "20260904_172909_Feifan_VRDIrect_ObjecttTracking"),
    ("Shared autonomy", "20260904_174535_Feifan_VRShared_ObjectTracking"),
]
HAND = "left"
ARM_LEN = 7


class FK(Node):
    def __init__(self):
        super().__init__("track_3d_full_fk")
        self.cli = self.create_client(GetPositionFK, "/compute_fk")
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("waiting for /compute_fk ...")

    def real_ee(self, q):
        rs = RobotState()
        rs.joint_state.name = ["%s_joint_%d" % (HAND, i + 1) for i in range(ARM_LEN)]
        rs.joint_state.position = [float(v) for v in q]
        req = GetPositionFK.Request()
        req.header.frame_id = "world"
        req.fk_link_names = ["%s_end_effector_link" % HAND]
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
          ["ee_%s_%s" % (HAND, a) for a in "xyz"] +
          ["real_%s_j%d" % (HAND, j + 1) for j in range(ARM_LEN)] +
          ["vr_%s_grip" % HAND])


def load(session):
    path = os.path.join(ROOT, "recordings", "sessions", session, "trail.csv")
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return [r for r in rows if all((r.get(k) or "") != "" for k in NEEDED)]


def pick_place(rows):
    prev = 0.0
    picked = placed = None
    for i, row in enumerate(rows):
        g = float(row.get("vr_%s_grip" % HAND) or 0.0)
        if picked is None and prev < 0.5 <= g:
            picked = i
        elif picked is not None and placed is None and prev >= 0.5 > g:
            placed = i
            break
        prev = g
    return picked, placed


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
        sx = [float(r["ee_%s_x" % HAND]) for r in rows]
        sy = [float(r["ee_%s_y" % HAND]) for r in rows]
        sz = [float(r["ee_%s_z" % HAND]) for r in rows]

        step = 12
        real_idx = list(range(0, len(rows), step))
        real_pts = []
        for i in real_idx:
            q = [float(rows[i]["real_%s_j%d" % (HAND, j + 1)]) for j in range(ARM_LEN)]
            p = fk.real_ee(q)
            if p is not None:
                real_pts.append(p)
        rx, ry, rz = zip(*real_pts)

        pick_i, place_i = pick_place(rows)

        ax = fig.add_subplot(1, 2, panel + 1, projection="3d")
        ax.plot(mx, my, mz, color=BLUE, lw=0.6, alpha=0.8,
                label="operator's hand (master)" if panel == 0 else None)
        ax.plot(sx, sy, sz, color=RED, lw=1.1,
                label="simulated robot EE" if panel == 0 else None)
        ax.plot(rx, ry, rz, color=GREEN, lw=1.1, ls="--",
                label="real robot EE (FK)" if panel == 0 else None)
        if pick_i is not None:
            ax.scatter(*[[v[pick_i]] for v in (sx, sy, sz)], color="black",
                       marker="o", s=45, zorder=5,
                       label="picked" if panel == 0 else None)
        if place_i is not None:
            ax.scatter(*[[v[place_i]] for v in (sx, sy, sz)], color="black",
                       marker="^", s=45, zorder=5,
                       label="placed" if panel == 0 else None)
        ax.set_title(label)
        ax.set_xlabel("x (m)", labelpad=0)
        ax.set_ylabel("y (m)", labelpad=0)
        ax.set_zlabel("z (m)", labelpad=-2)
        ax.tick_params(labelsize=7)
        axes.append(ax)

    fig.subplots_adjust(left=0.03, right=0.98, bottom=0.14, top=0.90, wspace=0.12)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=5, frameon=False,
               fontsize=7, bbox_to_anchor=(0.52, -0.02))

    fig.savefig(OUT)
    print("wrote", OUT)

    fk.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
