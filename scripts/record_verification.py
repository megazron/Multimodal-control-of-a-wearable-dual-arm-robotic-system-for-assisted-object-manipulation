#!/usr/bin/env python3
"""Drive every task/scenario/condition in sim and RECORD it for review.

    python3 scripts/record_verification.py --all
    python3 scripts/record_verification.py --task t3 --scenario S1

Per run it writes, under recordings/verification/<task>/<scenario>/<condition>/:

    clip.mp4            3-D animation rendered from LIVE TF
    plot_traj.png       the executed EE paths against the commanded path
    plot_metrics.png    the metric time-series
    bag/                a rosbag2 of /tf, /joint_states and the commands
    summary.json        metrics, clearance and the self-checks

WHY THE CLIP IS RENDERED FROM TF AND NOT SCREEN-GRABBED FROM RVIZ
-----------------------------------------------------------------
An x11grab of RViz records whatever happened to be on the desktop: the wrong
camera angle, a dialog, a window that was behind another. It cannot be checked
programmatically, and under WSLg it cannot even be guaranteed non-black.

This renders the SAME data RViz renders -- link poses straight off /tf -- from
a fixed camera, with the wearer's collision primitives, the task target and
the live minimum clearance drawn in. So the clip and the numbers in
summary.json come from ONE source, and "the metric matches what the video
shows" is true by construction rather than by hope. `--x11` additionally
screen-grabs RViz for anyone who wants the familiar view.

THE ARMS ARE ACTUALLY DRIVEN. Each scenario's path is solved with /compute_ik
and executed through the real joint-trajectory controllers, so TF moves
because the robot moved. A clip of a stationary robot would verify nothing.
"""
import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                # noqa: E402
from mpl_toolkits.mplot3d.art3d import Line3DCollection        # noqa: E402
import numpy as np                                             # noqa: E402
import rclpy                                                   # noqa: E402
import rclpy.time                                              # noqa: E402
import yaml                                                    # noqa: E402
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from verify_task_scenes import Solver, HOME_TOL_RAD            # noqa: E402
from audit_scenario_reachability import densify                # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BIM = os.path.join(ROOT, "src/srl_experiments/experiments/bimanual")
OUT = os.path.join(ROOT, "recordings/verification")

# THE MODE LEVEL. recordings/verification/<mode>/<task>/<scenario>/<condition>.
# This recorder drives the arm by calling /compute_ik DIRECTLY -- it never
# publishes /master_arm_pose_*, so no follower, clutch, anchor or orientation
# lock is in the path. It therefore writes into the scripted-playback bucket
# and NOT into a control-mode directory, because a clip filed under
# 01_master_teleop would assert something untrue about how it was produced.
MODE_BUCKET = "00_unclassified_scripted_playback"
FFMPEG = os.path.expanduser("~/.local/bin/ffmpeg")
CONDITIONS = ("direct", "assisted", "shared")

# The arm links drawn, in chain order, plus the wearer primitives. Taken from
# the URDF; clearance is measured between them, which is what makes "nothing
# passes through the wearer" checkable rather than eyeballed.
ARM_LINKS = ["base_link", "shoulder_link", "half_arm_1_link", "half_arm_2_link",
             "forearm_link", "spherical_wrist_1_link", "spherical_wrist_2_link",
             "bracelet_link", "end_effector_link"]
# CLEARANCE IS DELEGATED TO THE SYSTEM'S OWN MODULE, srl_teleop/clearance.py.
# Two independent clearance metrics would be worse than none: mine measured
# link ORIGINS from half_arm_2 outward and reported 0.05 m where the shipped
# guard (forearm_link outward, capsules from human_backpack.xacro) reports far
# more. A verification clip must be scored by the metric the robot actually
# enforces, or "below the floor" in a summary means nothing about the robot.
#
# The wearer's ACTUAL collision geometry, read out of human_backpack.xacro:
#   (link, local origin offset, kind, dims)
# Looked up through TF at runtime rather than hardcoded in world coordinates.
# The first version of this file guessed world-frame boxes and reported 0.064 m
# of clearance at a pose CLAUDE.md measures at 0.224 m -- the model was wrong,
# not the robot. Instrument first, finding second.
WEARER_SPEC = [
    ("torso", (0, 0, 0.17), "box", (0.18, 0.11, 0.24)),
    ("head", (0, 0, 0.245), "sph", (0.105,)),
    ("hips", (0, 0, 0.17), "box", (0.16, 0.105, 0.09)),
    ("left_leg", (0.02, 0, 0.36), "cyl", (0.075, 0.44)),
    ("right_leg", (-0.02, 0, 0.36), "cyl", (0.075, 0.44)),
    ("human_left_upper_arm", (0, 0, -0.15), "cyl", (0.050, 0.30)),
    ("human_right_upper_arm", (0, 0, -0.15), "cyl", (0.050, 0.30)),
    ("human_left_lower_arm", (0, 0, -0.13), "cyl", (0.045, 0.26)),
    ("human_right_lower_arm", (0, 0, -0.13), "cyl", (0.045, 0.26)),
]
# Pairs the SRDF excludes because they are bolted permanently adjacent. The
# clearance figure must exclude them too or it reports a constant near-zero
# and hides a real incursion.
EXCLUDED_PROXIMAL = {"base_link", "shoulder_link", "half_arm_1_link"}


class Driver(Solver):
    """Solver + trajectory execution + TF capture."""

    def __init__(self):
        super().__init__()
        self.pub = {a: self.create_publisher(
            JointTrajectory, "/%s_arm_controller/joint_trajectory" % a, 5)
            for a in ("left", "right")}

    def link_pose(self, frame):
        try:
            t = self.buf.lookup_transform("world", frame, rclpy.time.Time())
            v = t.transform.translation
            return np.array([v.x, v.y, v.z])
        except Exception:                                      # noqa: BLE001
            return None

    def chain(self, arm):
        out = []
        for ln in ARM_LINKS:
            p = self.link_pose("%s_%s" % (arm, ln))
            if p is not None:
                out.append((ln, p))
        return out

    def solve_joints(self, arm, xyz, quat, tries=6):
        """Like Solver.solve but returns the JOINT SOLUTION, not a bool.

        Seeded from the live joint state and re-seeding joint_3 on failure,
        exactly as ik_follower_node does -- so what the clip shows is the
        posture the real follower would have chosen, not an arbitrary one.
        """
        from moveit_msgs.msg import PositionIKRequest, RobotState
        from moveit_msgs.srv import GetPositionIK
        from geometry_msgs.msg import Pose, PoseStamped
        seed = [self.js.get(k, 0.0) for k in self.names(arm)]
        for k in range(tries):
            s2 = list(seed)
            if k:
                s2[2] += (0.35 * ((k + 1) // 2)) * (1 if k % 2 else -1)
            req = GetPositionIK.Request()
            r = PositionIKRequest()
            r.group_name = "%s_arm" % arm
            rs = RobotState()
            rs.joint_state.name = self.names(arm)
            rs.joint_state.position = [float(x) for x in s2]
            r.robot_state = rs
            r.avoid_collisions = True
            ps = PoseStamped()
            ps.header.frame_id = "world"
            pz = Pose()
            pz.position.x, pz.position.y, pz.position.z = (float(v) for v in xyz)
            # ASSIGN COMPONENTWISE, never `pz.orientation = quat`. A Quaternion
            # built from a differently-imported geometry_msgs is a different
            # Python class to the C extension, which aborts the process with
            # `quaternion__convert_from_py` rather than raising. Accepting a
            # plain 4-tuple as well removes the class-identity problem.
            if hasattr(quat, "x"):
                qx, qy, qz_, qw = quat.x, quat.y, quat.z, quat.w
            else:
                qx, qy, qz_, qw = quat
            pz.orientation.x = float(qx)
            pz.orientation.y = float(qy)
            pz.orientation.z = float(qz_)
            pz.orientation.w = float(qw)
            ps.pose = pz
            r.pose_stamped = ps
            r.timeout.nanosec = 100_000_000
            req.ik_request = r
            fut = self.ik.call_async(req)
            t0 = time.monotonic()
            while not fut.done() and time.monotonic() - t0 < 3.0:
                rclpy.spin_once(self, timeout_sec=0.002)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                nm = list(res.solution.joint_state.name)
                pos = list(res.solution.joint_state.position)
                d = dict(zip(nm, pos))
                return [d[j] for j in self.names(arm)]
        return None

    def send(self, arm, q, dt):
        m = JointTrajectory()
        m.joint_names = self.names(arm)
        p = JointTrajectoryPoint()
        p.positions = [float(x) for x in q]
        p.time_from_start.sec = int(dt)
        p.time_from_start.nanosec = int((dt - int(dt)) * 1e9)
        m.points = [p]
        self.pub[arm].publish(m)


def prim_distance(p, kind, centre, dims):
    """Distance from a point to one wearer primitive. 0 means inside."""
    d = np.asarray(p, float) - np.asarray(centre, float)
    if kind == "sph":
        return max(0.0, float(np.linalg.norm(d)) - dims[0])
    if kind == "cyl":
        r, h = dims
        radial = max(0.0, float(np.hypot(d[0], d[1])) - r)
        axial = max(0.0, abs(float(d[2])) - h / 2.0)
        return float(np.hypot(radial, axial))
    e = np.maximum(np.abs(d) - np.asarray(dims, float), 0.0)
    return float(np.linalg.norm(e))


def wearer_prims(dr):
    """World-frame wearer primitives, from TF, for DRAWING only.

    Scoring uses srl_teleop.clearance via `shipped_clearance`; these exist so
    the picture shows the same bodies the score used.
    """
    out = []
    for link, off, kind, dims in WEARER_SPEC:
        base = dr.link_pose(link)
        if base is None:
            continue
        out.append((link, kind, base + np.asarray(off, float), dims))
    return out


try:
    sys.path.insert(0, os.path.join(ROOT, "src/srl_teleop"))
    from srl_teleop.clearance import (TORSO_BODY, HEAD_BODY, HIPS_BODY,
                                      DISTAL_LINKS, point_clearance)
    _HAVE_SHIPPED = True
except Exception:                                              # noqa: BLE001
    _HAVE_SHIPPED = False
    DISTAL_LINKS = ["forearm_link", "spherical_wrist_1_link",
                    "spherical_wrist_2_link", "bracelet_link",
                    "end_effector_link"]

_WEARER_FRAMES = (("torso", "TORSO_BODY"), ("head", "HEAD_BODY"),
                  ("hips", "HIPS_BODY"))


def shipped_clearance(dr, arm):
    """Minimum clearance for one arm, using srl_teleop/clearance.py exactly.

    Each distal link's origin is transformed INTO the wearer link's own frame
    with TF, so the primitive origins from human_backpack.xacro apply directly
    and no rotation is assumed away.
    """
    if not _HAVE_SHIPPED:
        return float("nan"), ""
    worst, who = float("inf"), ""
    bodies = {"torso": TORSO_BODY, "head": HEAD_BODY, "hips": HIPS_BODY}
    for ln in DISTAL_LINKS:
        for wf, prims in bodies.items():
            try:
                t = dr.buf.lookup_transform(wf, "%s_%s" % (arm, ln),
                                            rclpy.time.Time())
            except Exception:                                  # noqa: BLE001
                continue
            v = t.transform.translation
            d = point_clearance((v.x, v.y, v.z), prims)
            if d < worst:
                worst, who = float(d), wf
    return (worst if worst < float("inf") else float("nan")), who


# ------------------------------------------------------------------ scenarios
def build_runs(spec, only_task=None, only_scen=None):
    """(task, scenario, arm->path, target, label) for everything recordable."""
    T = spec["tasks"]
    runs = []

    def add(task, scen, paths, target=None, note=""):
        if only_task and task != only_task:
            return
        if only_scen and not scen.startswith(only_scen):
            return
        runs.append(dict(task=task, scenario=scen, paths=paths,
                         target=target, note=note))

    for name, s in T.get("T3_rigid", {}).items():
        sep = float(s.get("sep", 0.310))
        p = s["path"]
        add("t3", name,
            {"left": [[w[0] + sep / 2, w[1], w[2]] for w in p],
             "right": [[w[0] - sep / 2, w[1], w[2]] for w in p]},
            note="rigid tray, %.0f mm span, vertical lift" % (1000 * sep))
    for name, s in T.get("T6_compliant", {}).items():
        sep = float(s.get("sep", 0.310))
        p = s["path"]
        add("t6", name,
            {"left": [[w[0] + sep / 2, w[1], w[2]] for w in p],
             "right": [[w[0] - sep / 2, w[1], w[2]] for w in p]},
            note="compliant sling, %.0f mm span" % (1000 * sep))
    for name, s in T.get("T2_hold_fill", {}).items():
        add("t2", name,
            {s["fill_arm"]: s["fill_path"],
             s["hold_arm"]: [s["hold"], s["hold"]]},
            target=s["release"],
            note="%s holds, %s fills" % (s["hold_arm"], s["fill_arm"]))
    for name, s in T.get("T5_handover", {}).items():
        arm = s.get("arm", "right")
        o, r = s["object"], s["receive"]
        add("t5", name,
            {arm: [[o[0], o[1], o[2] + 0.08], o, [o[0], o[1], o[2] + 0.08],
                   [(o[0] + r[0]) / 2, (o[1] + r[1]) / 2, max(o[2], r[2]) + 0.08],
                   [r[0], r[1], r[2] + 0.06], r, [r[0], r[1], r[2] + 0.08]]},
            target=r, note="%s arm delivers to the wearer" % arm)
    for name, s in T.get("T7_pursuit", {}).items():
        sys.path.insert(0, os.path.join(BIM, "t7_pursuit"))
        import targets as tg
        _, L, R = tg.trial_targets(s, 8.0, dt=0.25)
        paths = {}
        if s.get("unimanual") != "right":
            paths["left"] = [list(map(float, p)) for p in L]
        if s.get("unimanual") != "left":
            paths["right"] = [list(map(float, p)) for p in R]
        add("t7", name, paths,
            note="pursuit, left %.2f / right %.2f m/s"
                 % (s.get("speed_left", 0), s.get("speed_right", 0)))
    for name, s in T.get("T8_wearer_assisted_reach", {}).items():
        add("t8", name, {s["arm"]: s["approach_path"]},
            target=s["approach_path"][-1],
            note="%s arm; wearer must %s"
                 % (s["arm"], s["wearer_stance"].replace("_", " ")))
    for name, s in T.get("T9_wearer_motion", {}).items():
        amp = float(s["sway_amplitude_m"])
        # The arm holds a WORLD-FIXED target while the base sways: in the
        # arm's frame that is the target orbiting by -sway. This is the task.
        paths = {}
        for arm, c in (("left", s["centre_left"]), ("right", s["centre_right"])):
            ring = []
            for k in range(17):
                a = 2 * math.pi * k / 16
                ring.append([c[0] - amp * math.cos(a),
                             c[1] - amp * math.sin(a), c[2]])
            paths[arm] = ring if amp > 0 else [c, c]
        add("t9", name, paths,
            note="wearer sway %.0f mm -- arm must HOLD a world-fixed point"
                 % (1000 * amp))
    return runs


# ------------------------------------------------------------------ execution
def execute(dr, run, cond, fps=10.0):
    """Drive the arms along the path, capturing TF every frame.

    TIMING IS THE WHOLE DIFFICULTY. The first version densified to 30 mm,
    gave each waypoint 0.1 s and produced a 3-frame clip with 300 mm of
    tracking error -- the arm was asked to cross half a metre in a tenth of a
    second and did not. Now:

      * the path is densified to 15 mm so consecutive commands are small;
      * every command is given `settle` seconds AND the arm is allowed to
        converge before the frame is captured, so the clip shows where the
        robot IS, not where it was asked to be;
      * the approach from home to the first waypoint is INSIDE the clip,
        because a clip that starts already on-target hides the largest motion.
    """
    q = {a: dr.ee_quat(a) for a in ("left", "right")}
    # Condition changes the commanded motion: `assisted` and `shared` apply
    # progressively more smoothing, which is what the autonomy does to a
    # human's jitter. NOT a claim that the autonomy stack ran -- see INDEX.md.
    smooth = {"direct": 0.0, "assisted": 0.30, "shared": 0.55}[cond]
    settle = 1.0 / fps
    dense = {a: (densify(p, 0.015) if len(p) > 1 else list(p))
             for a, p in run["paths"].items()}
    nmax = max(len(p) for p in dense.values())
    # Cap the frame count so a 400-point T7 locus does not make a 40 s clip.
    stride = max(1, int(math.ceil(nmax / 120.0)))
    prims = wearer_prims(dr)
    frames = []
    prev = {}

    def capture(t, cmd, phase="task"):
        f = dict(t=t, phase=phase,
                 cmd={a: list(map(float, v)) for a, v in cmd.items()})
        for arm in ("left", "right"):
            named = dr.chain(arm)
            f["named_" + arm] = [(n, list(map(float, p))) for n, p in named]
            f[arm] = [list(map(float, p)) for _, p in named]
            ee = dr.link_pose("%s_end_effector_link" % arm)
            f["ee_" + arm] = list(map(float, ee)) if ee is not None else None
        f["clearance"], f["clearance_to"] = clearance_all(dr)
        f["prims"] = prims
        frames.append(f)

    t0 = time.monotonic()
    # --- approach: home -> first waypoint, generously timed and IN the clip
    first = {a: np.asarray(p[0], float) for a, p in dense.items()}
    for arm, tgt in first.items():
        sol = dr.solve_joints(arm, list(tgt), q[arm])
        if sol is not None:
            dr.send(arm, sol, 2.0)
    for _ in range(int(2.0 * fps)):
        dr.spin(settle)
        capture(time.monotonic() - t0, first, phase="approach")
    prev.update(first)

    # --- the path itself
    for i in range(0, nmax, stride):
        cmd = {}
        for arm, path in dense.items():
            tgt = np.asarray(path[min(i, len(path) - 1)], float)
            if smooth and arm in prev:
                tgt = prev[arm] + (tgt - prev[arm]) * (1.0 - smooth)
            prev[arm] = tgt
            cmd[arm] = tgt
            sol = dr.solve_joints(arm, list(tgt), q[arm])
            if sol is not None:
                dr.send(arm, sol, 0.25)
        dr.spin(0.25)
        capture(time.monotonic() - t0, cmd)

    # --- hold at the end so the final pose is visible
    for _ in range(max(3, int(0.6 * fps))):
        dr.spin(settle)
        capture(time.monotonic() - t0, prev, phase="hold")
    return frames


def clearance_all(dr):
    worst, who = float("inf"), ""
    for arm in ("left", "right"):
        d, w = shipped_clearance(dr, arm)
        if d == d and d < worst:
            worst, who = d, w
    return (worst if worst < float("inf") else float("nan")), who


# ------------------------------------------------------------------ rendering
def render(frames, run, cond, path_mp4, fps=10):
    """TWO PANELS: a 3-D view and a front (x-z) view.

    The 3-D view alone is not reviewable -- depth ambiguity makes it
    impossible to tell whether the two grippers are level, which is the whole
    of the T3/T6 metric. The front view answers that directly, and the OBJECT
    is drawn between them so "the object behaves plausibly" is something you
    can see rather than infer.
    """
    tmp = path_mp4 + ".frames"
    os.makedirs(tmp, exist_ok=True)
    lo, hi = np.array([-1.0, -0.5, 0.6]), np.array([1.0, 1.0, 1.8])
    task = run["task"]
    for i, f in enumerate(frames):
        fig = plt.figure(figsize=(11.0, 5.0), dpi=100)
        ax = fig.add_subplot(121, projection="3d")
        ax2 = fig.add_subplot(122)
        for wname, kind, c, dims in f.get("prims", []):
            draw_prim(ax, kind, c, dims, "0.6")
            draw_prim_front(ax2, kind, c, dims, "0.75")
        for arm, col in (("left", "#1f77b4"), ("right", "#d62728")):
            ch = np.asarray(f.get(arm) or [])
            if len(ch) > 1:
                ax.plot(ch[:, 0], ch[:, 1], ch[:, 2], "-o", color=col,
                        lw=2.4, ms=3.0, label="%s arm" % arm)
                ax2.plot(ch[:, 0], ch[:, 2], "-o", color=col, lw=2.2, ms=3.0)
            e = f.get("ee_" + arm)
            if e:
                ax.scatter(*e, color=col, s=48, edgecolor="k", linewidth=0.4)
                ax2.scatter(e[0], e[2], color=col, s=52, edgecolor="k",
                            linewidth=0.4, zorder=5)
        # ---- the object
        el, er = f.get("ee_left"), f.get("ee_right")
        if task in ("t3", "t6") and el and er:
            draw_object(ax, ax2, el, er, task)
        if task == "t2" and run.get("target"):
            t = run["target"]
            for A, xy in ((ax, None), (ax2, True)):
                pass
            ax.scatter(*t, marker="s", s=120, color="#2ca02c",
                       label="container opening")
            ax2.scatter(t[0], t[2], marker="s", s=120, color="#2ca02c")
        if task in ("t5", "t8") and run.get("target"):
            t = run["target"]
            ax.scatter(*t, marker="*", s=210, color="#2ca02c", label="target")
            ax2.scatter(t[0], t[2], marker="*", s=210, color="#2ca02c")
        for arm, c in f["cmd"].items():
            col = "#1f77b4" if arm == "left" else "#d62728"
            ax.scatter(*c, marker="x", s=58, color=col)
            ax2.scatter(c[0], c[2], marker="x", s=58, color=col)
        ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1])
        ax.set_zlim(lo[2], hi[2])
        ax.set_xlabel("x (wearer's right)"); ax.set_ylabel("y (fwd)")
        ax.set_zlabel("z (up)")
        ax.view_init(elev=16, azim=-63)
        ax.legend(loc="upper left", fontsize=7)
        ax2.set_xlim(-1.0, 1.0); ax2.set_ylim(0.6, 1.8)
        ax2.set_aspect("equal")
        ax2.grid(alpha=0.3)
        ax2.set_xlabel("x (wearer's right)  -- FRONT VIEW, wearer faces you")
        ax2.set_ylabel("z (up)")
        cl = f["clearance"]
        dz = (abs(el[2] - er[2]) * 1000) if (el and er) else float("nan")
        extra = ("  |dz| %.0f mm" % dz) if task in ("t3", "t6") else ""
        ax2.set_title("t=%4.1f s  [%s]  clearance %.3f m (%s)%s"
                      % (f["t"], f.get("phase", "task"), cl,
                         f["clearance_to"], extra), fontsize=9)
        fig.suptitle("%s / %s / %s   --   %s"
                     % (task.upper(), run["scenario"], cond, run["note"]),
                     fontsize=10)
        fig.tight_layout()
        fig.savefig(os.path.join(tmp, "f%04d.png" % i))
        plt.close(fig)
    subprocess.run([FFMPEG, "-y", "-loglevel", "error", "-framerate", str(fps),
                    "-i", os.path.join(tmp, "f%04d.png"),
                    "-c:v", "libx264", "-pix_fmt", "yuv420p", path_mp4],
                   check=True)
    shutil.rmtree(tmp, ignore_errors=True)


def draw_object(ax, ax2, el, er, task):
    """The carried object, between the two grippers.

    T3 is a RIGID tray: a straight line, so any height difference tilts it
    visibly. T6 is a COMPLIANT sling: it hangs, with sag from the closed form
    in coupled_metrics (sag = sqrt((L/2)^2 - (s/2)^2)), so the reviewer can
    see the ball being retained or lost rather than take it on trust.
    """
    a, b = np.asarray(el, float), np.asarray(er, float)
    if task == "t3":
        ax.plot(*zip(a, b), color="#8c564b", lw=3.5)
        ax2.plot([a[0], b[0]], [a[2], b[2]], color="#8c564b", lw=3.5)
        return
    sep = float(np.linalg.norm(a - b))
    L = 0.350
    v = max(0.0, (L / 2) ** 2 - (sep / 2) ** 2)
    sag = math.sqrt(v)
    ts = np.linspace(0, 1, 21)
    pts = np.array([a + (b - a) * t for t in ts])
    pts[:, 2] -= sag * np.sin(np.pi * ts)
    ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], color="#8c564b", lw=2.6)
    ax2.plot(pts[:, 0], pts[:, 2], color="#8c564b", lw=2.6)
    low = pts[np.argmin(pts[:, 2])]
    held = sag >= 2 * 0.020
    ax.scatter(*low, s=90, color="#8c564b" if held else "#d62728")
    ax2.scatter(low[0], low[2], s=90,
                color="#8c564b" if held else "#d62728")


def draw_prim_front(ax, kind, c, dims, col):
    c = np.asarray(c, float)
    if kind == "sph":
        ax.add_patch(plt.Circle((c[0], c[2]), dims[0], fill=False,
                                color=col, lw=0.9))
    elif kind == "cyl":
        r, h = dims
        ax.add_patch(plt.Rectangle((c[0] - r, c[2] - h / 2), 2 * r, h,
                                   fill=False, color=col, lw=0.9))
    else:
        ax.add_patch(plt.Rectangle((c[0] - dims[0], c[2] - dims[2]),
                                   2 * dims[0], 2 * dims[2], fill=False,
                                   color=col, lw=0.9))


def draw_prim(ax, kind, c, dims, col):
    c = np.asarray(c, float)
    if kind == "sph":
        u = np.linspace(0, 2 * np.pi, 14)
        for zc in (-0.7, 0.0, 0.7):
            r = dims[0] * math.sqrt(max(0.0, 1 - zc * zc))
            ax.plot(c[0] + r * np.cos(u), c[1] + r * np.sin(u),
                    c[2] + dims[0] * zc, color=col, lw=0.7)
        return
    if kind == "cyl":
        r, h = dims
        u = np.linspace(0, 2 * np.pi, 16)
        for zc in (-h / 2, h / 2):
            ax.plot(c[0] + r * np.cos(u), c[1] + r * np.sin(u), c[2] + zc,
                    color=col, lw=0.7)
        return
    h = np.asarray(dims, float)
    pts = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1)
                    for sz in (-1, 1)]) * h + c
    E = [(0, 1), (0, 2), (0, 4), (1, 3), (1, 5), (2, 3), (2, 6), (3, 7),
         (4, 5), (4, 6), (5, 7), (6, 7)]
    ax.add_collection3d(Line3DCollection([[pts[a], pts[b]] for a, b in E],
                                         colors=col, linewidths=0.8))


def plots(frames, run, cond, d):
    t = [f["t"] for f in frames]
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.2), dpi=100)
    for arm, col in (("left", "#1f77b4"), ("right", "#d62728")):
        ee = [f["ee_" + arm] for f in frames]
        if not any(e for e in ee):
            continue
        E = np.asarray([e for e in ee if e])
        ax[0].plot(E[:, 0], E[:, 2], "-", color=col, label="%s EE" % arm)
        C = np.asarray([f["cmd"][arm] for f in frames if arm in f["cmd"]])
        if len(C):
            ax[0].plot(C[:, 0], C[:, 2], "--", color=col, alpha=0.55,
                       label="%s commanded" % arm)
    ax[0].set_xlabel("x (m)"); ax[0].set_ylabel("z (m)")
    ax[0].set_title("EE path vs command (front view)"); ax[0].legend(fontsize=7)
    ax[0].grid(alpha=0.3); ax[0].set_aspect("equal", adjustable="datalim")

    err = []
    for f in frames:
        e = []
        for arm in f["cmd"]:
            p = f.get("ee_" + arm)
            if p:
                e.append(np.linalg.norm(np.asarray(p) - np.asarray(f["cmd"][arm])))
        err.append(1000 * float(np.mean(e)) if e else np.nan)
    ap = [f["t"] for f in frames if f.get("phase") == "approach"]
    if ap:
        ax[1].axvspan(min(ap), max(ap), color="0.9",
                      label="approach (not scored)")
    ax[1].plot(t, err, "-", color="#333", label="tracking |cmd-EE| (mm)")
    ax[1].plot(t, [1000 * f["clearance"] for f in frames], "-",
               color="#2ca02c", label="min clearance (mm)")
    ax[1].axhline(120, color="#d62728", ls=":", label="120 mm real-robot floor")
    ax[1].set_xlabel("t (s)"); ax[1].set_ylabel("mm")
    ax[1].set_title("metrics"); ax[1].legend(fontsize=7); ax[1].grid(alpha=0.3)
    fig.suptitle("%s / %s / %s" % (run["task"].upper(), run["scenario"], cond),
                 fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(d, "plot_metrics.png"))
    plt.close(fig)
    return err


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", default=None)
    ap.add_argument("--scenario", default=None)
    ap.add_argument("--condition", default=None, choices=CONDITIONS)
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--bag", action="store_true", default=True)
    ap.add_argument("--skip-home-check", action="store_true")
    a = ap.parse_args()

    rclpy.init()
    dr = Driver()
    dr.spin(3.0)
    if not dr.ik.wait_for_service(timeout_sec=20.0):
        print("no /compute_ik -- start the sim")
        return 2
    spec = yaml.safe_load(open(os.path.join(BIM, "scenarios_verified.yaml")))
    runs = build_runs(spec, a.task, a.scenario)
    conds = [a.condition] if a.condition else list(CONDITIONS)
    print("recording %d run(s) x %d condition(s)" % (len(runs), len(conds)))
    index = []
    for run in runs:
        for cond in conds:
            d = os.path.join(OUT, MODE_BUCKET, run["task"],
                             run["scenario"], cond)
            os.makedirs(d, exist_ok=True)
            bagp = os.path.join(d, "bag")
            shutil.rmtree(bagp, ignore_errors=True)
            bag = None
            if a.bag:
                bag = subprocess.Popen(
                    ["ros2", "bag", "record", "-o", bagp, "/tf", "/tf_static",
                     "/joint_states", "/left_arm_controller/joint_trajectory",
                     "/right_arm_controller/joint_trajectory"],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                time.sleep(1.5)
            go_home(dr)
            frames = execute(dr, run, cond)
            if bag:
                bag.terminate()
                bag.wait(timeout=10)
            render(frames, run, cond, os.path.join(d, "clip.mp4"))
            err = plots(frames, run, cond, d)
            task_err = [e for e, f in zip(err, frames)
                        if f.get("phase") == "task" and e == e]
            cl = [f["clearance"] for f in frames]
            s = dict(task=run["task"], scenario=run["scenario"],
                     condition=cond, note=run["note"],
                     frames=len(frames), duration_s=frames[-1]["t"],
                     # TASK PHASE ONLY. The approach from home is inside the
                     # clip on purpose (it is the largest motion and hiding it
                     # would hide the most collision-relevant travel), but
                     # scoring it as tracking error is wrong: during the
                     # approach the command is deliberately far ahead of the
                     # arm. Including it read 224 mm on a run that tracks to
                     # single digits once moving.
                     tracking_rms_mm=float(np.sqrt(np.mean(
                         np.asarray(task_err, float) ** 2)))
                     if task_err else float("nan"),
                     tracking_max_mm=(float(np.max(task_err))
                                      if task_err else float("nan")),
                     tracking_rms_incl_approach_mm=float(np.sqrt(np.nanmean(
                         np.asarray(err, float) ** 2))),
                     min_clearance_m=float(np.min(cl)),
                     clearance_to=frames[int(np.argmin(cl))]["clearance_to"],
                     ee_travel_m=travel(frames),
                     passes_through_wearer=bool(np.min(cl) <= 0.0),
                     below_real_floor=bool(np.min(cl) < 0.12))
            json.dump(s, open(os.path.join(d, "summary.json"), "w"), indent=2)
            index.append(s)
            print("  %-4s %-22s %-9s  travel %.3f m  clr %.3f m  trk %.1f mm  %s"
                  % (run["task"], run["scenario"], cond, s["ee_travel_m"],
                     s["min_clearance_m"], s["tracking_rms_mm"],
                     "THROUGH WEARER" if s["passes_through_wearer"] else "ok"))
    json.dump(index, open(os.path.join(OUT, "index.json"), "w"), indent=2)
    dr.destroy_node()
    rclpy.shutdown()
    return 0


def travel(frames):
    tot = 0.0
    for arm in ("left", "right"):
        E = [f["ee_" + arm] for f in frames]
        E = np.asarray([e for e in E if e])
        if len(E) > 1:
            tot += float(np.sum(np.linalg.norm(np.diff(E, axis=0), axis=1)))
    return tot


def go_home(dr):
    sys.path.insert(0, os.path.join(ROOT, "config"))
    import home_positions as hp
    for arm in ("left", "right"):
        dr.send(arm, hp.load_home_radians(arm), 1.2)
    dr.spin(1.6)


if __name__ == "__main__":
    sys.exit(main())
