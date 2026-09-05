#!/usr/bin/env python3
"""Every trajectory figure the pilot recordings support, in plain words.

Two phases, so the plots can be re-drawn without ROS:

    python3 thesis_v3/figures/gallery/trajectories/make_trajectory_gallery.py extract
    python3 thesis_v3/figures/gallery/trajectories/make_trajectory_gallery.py plot

`extract` needs the sim stack up for MoveIt's /compute_fk
(scripts/sim_session.py --stack teleop --keep-up -- true). It reads every
VR session's trail.csv (operator's hand = master_*_x/y/z, simulated robot
hand = ee_*_x/y/z, real robot joints = real_*_jN) and every master-arm
session's trail_extracted.csv (master hand, sim joints, real joints), runs
forward kinematics on every STRIDE-th row, and writes trajectories.json.
The real robot hand is FK of the real arm's own joint encoders; for the
master cohort the simulated hand is FK of the simulated joints too, since
that cohort's export has no Cartesian robot column.

The recorded VR "grip" signal is the clutch button (the gripper was never
commanded in the pilot); the first clutch engage and release are marked and
labelled as exactly that.
"""

import csv
import json
import math
import os
import sys
import time

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d import Axes3D  # noqa: E402,F401

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
SCRATCH = "/tmp/claude-1000/-home-gausms-kortex-ws/bbebc7ab-9df1-4fc1-80b7-ee284d9eaac6/scratchpad"
CACHE = os.path.join(HERE, "trajectories.json")

GREY, BLUE, RED, GREEN = "0.35", "#2c6fbb", "#c1392b", "#3f9142"
plt.rcParams.update({
    "font.size": 9, "axes.titlesize": 9,
    "axes.spines.top": False, "axes.spines.right": False,
})
ANON = {"Wen": "P1", "Hela": "P2", "Farrel": "P3", "Feifan": "P4"}
COND = {"Direct": "direct control", "Shared": "with assistance"}
TASK_SHORT = {"Object tracking": "object tracking", "Target reaching": "target reaching",
              "Position matching": "position matching", "Unspecified": "task not labelled"}
STRIDE = 10
ARM_LEN = 7
LBL_OP = "operator's hand"
LBL_SIM = "simulated robot hand"
LBL_REAL = "real robot hand (from its own joint encoders)"


def smooth(vals, win=9):
    out, half, n = [], win // 2, len(vals)
    for i in range(n):
        lo, hi = max(0, i - half), min(n, i + half + 1)
        out.append(sum(vals[lo:hi]) / (hi - lo))
    return out


# ----------------------------------------------------------------- extract
def extract():
    import rclpy
    from moveit_msgs.msg import RobotState
    from moveit_msgs.srv import GetPositionFK
    from rclpy.node import Node

    class FK(Node):
        def __init__(self):
            super().__init__("trajectory_gallery_fk")
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
            return [p.x, p.y, p.z]

    rclpy.init()
    fk = FK()
    out = {"stride": STRIDE, "sessions": []}
    fails = 0

    vr = json.load(open(os.path.join(SCRATCH, "vr_study_sessions.json")))
    for s in vr:
        hand = s["active_hand"]
        path = os.path.join(ROOT, "recordings", "sessions", s["session"], "trail.csv")
        need = (["master_%s_%s" % (hand, a) for a in "xyz"] + ["ee_%s_%s" % (hand, a) for a in "xyz"]
                + ["real_%s_j%d" % (hand, j + 1) for j in range(ARM_LEN)] + ["vr_%s_grip" % hand, "t"])
        with open(path) as f:
            rows = [r for r in csv.DictReader(f) if all((r.get(k) or "") != "" for k in need)]
        rec = {"cohort": "VR", "participant": ANON.get(s["participant"], s["participant"]),
               "task": s["task"], "condition": s["condition"], "hand": hand,
               "t": [], "op": [], "sim": [], "real": [], "engage": None, "release": None}
        prev = 0.0
        for i, r in enumerate(rows):
            g = float(r["vr_%s_grip" % hand] or 0.0)
            if rec["engage"] is None and prev < 0.5 <= g:
                rec["engage"] = len(rec["t"]) if i % STRIDE == 0 else len(rec["t"])
            elif rec["engage"] is not None and rec["release"] is None and prev >= 0.5 > g:
                rec["release"] = len(rec["t"])
            prev = g
            if i % STRIDE:
                continue
            p = fk.ee(hand, [float(r["real_%s_j%d" % (hand, j + 1)]) for j in range(ARM_LEN)])
            if p is None:
                fails += 1
                continue
            rec["t"].append(float(r["t"]))
            rec["op"].append([float(r["master_%s_%s" % (hand, a)]) for a in "xyz"])
            rec["sim"].append([float(r["ee_%s_%s" % (hand, a)]) for a in "xyz"])
            rec["real"].append(p)
        out["sessions"].append(rec)
        print(rec["participant"], rec["task"], rec["condition"], len(rec["t"]), "samples", flush=True)

    ma = json.load(open(os.path.join(SCRATCH, "master_arm_sessions.json")))
    for s in ma:
        hand = s["active_hand"]
        path = os.path.join(ROOT, "recordings", "sessions", s["session"], "trail_extracted.csv")
        need = (["master_%s_%s" % (hand, a) for a in "xyz"]
                + ["sim_%s_j%d" % (hand, j + 1) for j in range(ARM_LEN)]
                + ["real_%s_j%d" % (hand, j + 1) for j in range(ARM_LEN)] + ["t"])
        with open(path) as f:
            rows = [r for r in csv.DictReader(f) if all((r.get(k) or "") != "" for k in need)]
        rec = {"cohort": "master", "participant": s["participant"], "task": s["task"],
               "condition": s["condition"], "hand": hand,
               "t": [], "op": [], "sim": [], "real": [], "engage": None, "release": None}
        for i in range(0, len(rows), STRIDE):
            r = rows[i]
            ps = fk.ee(hand, [float(r["sim_%s_j%d" % (hand, j + 1)]) for j in range(ARM_LEN)])
            pr = fk.ee(hand, [float(r["real_%s_j%d" % (hand, j + 1)]) for j in range(ARM_LEN)])
            if ps is None or pr is None:
                fails += 1
                continue
            rec["t"].append(float(r["t"]))
            rec["op"].append([float(r["master_%s_%s" % (hand, a)]) for a in "xyz"])
            rec["sim"].append(ps)
            rec["real"].append(pr)
        out["sessions"].append(rec)
        print(rec["participant"], rec["task"], rec["condition"], len(rec["t"]), "samples", flush=True)

    out["fk_failures"] = fails
    json.dump(out, open(CACHE, "w"))
    print("wrote", CACHE, "FK failures:", fails)
    fk.destroy_node()
    rclpy.shutdown()


# -------------------------------------------------------------------- plot
def label(rec):
    return "%s, %s, %s" % (rec["participant"], TASK_SHORT.get(rec["task"], rec["task"]),
                           COND[rec["condition"]])


def short(rec):
    return "%s, %s,\n%s" % (rec["participant"], TASK_SHORT.get(rec["task"], rec["task"]),
                            COND[rec["condition"]])


def cols(pts):
    return [p[0] for p in pts], [p[1] for p in pts], [p[2] for p in pts]


def gap_mm(rec):
    return [1000 * math.dist(a, b) for a, b in zip(rec["sim"], rec["real"])]


def rms(vals):
    return math.sqrt(sum(v * v for v in vals) / len(vals)) if vals else float("nan")


def draw3d(ax, rec, markers=True, op_alpha=0.45):
    ox, oy, oz = cols(rec["op"])
    ax.plot(smooth(ox), smooth(oy), smooth(oz), color=BLUE, lw=0.5, alpha=op_alpha, label=LBL_OP)
    sx, sy, sz = cols(rec["sim"])
    ax.plot(sx, sy, sz, color=RED, lw=1.1, label=LBL_SIM)
    rx, ry, rz = cols(rec["real"])
    ax.plot(rx, ry, rz, color=GREEN, lw=1.1, ls="--", label=LBL_REAL)
    if markers and rec["engage"] is not None and rec["engage"] < len(sx):
        i = rec["engage"]
        ax.scatter([sx[i]], [sy[i]], [sz[i]], color="black", marker="o", s=30, zorder=5,
                   label="first clutch engage")
    if markers and rec["release"] is not None and rec["release"] < len(sx):
        i = rec["release"]
        ax.scatter([sx[i]], [sy[i]], [sz[i]], color="black", marker="^", s=30, zorder=5,
                   label="first clutch release")
    ax.set_xlabel("x (m)", labelpad=-2, fontsize=7)
    ax.set_ylabel("y (m)", labelpad=-2, fontsize=7)
    ax.set_zlabel("z (m)", labelpad=-4, fontsize=7)
    ax.tick_params(labelsize=6, pad=0)
    ax.locator_params(nbins=4)


def legend_below(fig, ax, ncol=3, y=0.0):
    h, l = ax.get_legend_handles_labels()
    seen, hh, ll = set(), [], []
    for a, b in zip(h, l):
        if b not in seen:
            seen.add(b); hh.append(a); ll.append(b)
    fig.legend(hh, ll, loc="lower center", ncol=ncol, frameon=False, fontsize=7,
               bbox_to_anchor=(0.5, y))


def save(fig, name):
    pdf = os.path.join(HERE, name + ".pdf")
    fig.savefig(pdf)
    fig.savefig(os.path.join(HERE, name + ".png"), dpi=200)
    plt.close(fig)
    print("wrote", name)


def grid_shape(n):
    if n <= 2:
        return 1, n
    if n <= 4:
        return 2, 2
    if n <= 6:
        return 2, 3
    return 3, 3


def plot(data):
    sessions = data["sessions"]
    by_p = {}
    for r in sessions:
        by_p.setdefault(r["participant"], []).append(r)
    order = ["P1", "P2", "P3", "P4", "P5", "M1", "M2", "M3"]
    index = []

    # 1. per-participant 3-D grids + single panels
    for p in order:
        recs = by_p.get(p, [])
        if not recs:
            continue
        nr, nc = grid_shape(len(recs))
        fig = plt.figure(figsize=(2.6 * nc + 0.4, 2.5 * nr + 0.8))
        ax0 = None
        for k, rec in enumerate(recs):
            ax = fig.add_subplot(nr, nc, k + 1, projection="3d")
            draw3d(ax, rec, markers=(rec["cohort"] == "VR"))
            ax.set_title(short(rec), fontsize=7.5, pad=0)
            ax0 = ax0 or ax
        fig.subplots_adjust(left=0.02, right=0.98, bottom=0.14 if nr > 1 else 0.22, top=0.92,
                            wspace=0.08, hspace=0.25)
        legend_below(fig, ax0, ncol=3 if p.startswith("M") else 5, y=0.0)
        save(fig, "3d_grid_%s" % p)
        index.append(("3d_grid_%s" % p, "Every %s session in 3-D: operator's hand, simulated robot hand, real robot hand" % p,
                      "%d sessions" % len(recs), "APPENDIX"))
        for k, rec in enumerate(recs):
            fig = plt.figure(figsize=(3.4, 3.2))
            ax = fig.add_subplot(111, projection="3d")
            draw3d(ax, rec, markers=(rec["cohort"] == "VR"))
            ax.set_title(short(rec), fontsize=8, pad=0)
            fig.subplots_adjust(left=0.02, right=0.98, bottom=0.2, top=0.9)
            legend_below(fig, ax, ncol=2, y=0.0)
            save(fig, "3d_single_%s_%02d" % (p, k + 1))

    # 2. 2-D projections per participant (top view and side view)
    for p in order:
        recs = by_p.get(p, [])
        if not recs:
            continue
        for view, (ia, ib, xl, yl) in {"top": (0, 1, "x, forward (m)", "y, left-right (m)"),
                                       "side": (0, 2, "x, forward (m)", "z, height (m)")}.items():
            nr, nc = grid_shape(len(recs))
            fig, axes = plt.subplots(nr, nc, figsize=(2.4 * nc + 0.4, 2.3 * nr + 0.9), squeeze=False)
            for k, rec in enumerate(recs):
                ax = axes[k // nc][k % nc]
                o = cols(rec["op"]); s = cols(rec["sim"]); r = cols(rec["real"])
                ax.plot(smooth(o[ia]), smooth(o[ib]), color=BLUE, lw=0.5, alpha=0.45, label=LBL_OP)
                ax.plot(s[ia], s[ib], color=RED, lw=1.0, label=LBL_SIM)
                ax.plot(r[ia], r[ib], color=GREEN, lw=1.0, ls="--", label=LBL_REAL)
                ax.set_title(short(rec), fontsize=7.5)
                ax.set_aspect("equal", adjustable="datalim")
                ax.tick_params(labelsize=6)
                ax.set_xlabel(xl, fontsize=7); ax.set_ylabel(yl, fontsize=7)
            for k in range(len(recs), nr * nc):
                axes[k // nc][k % nc].axis("off")
            fig.subplots_adjust(left=0.08, right=0.98, bottom=0.16 if nr > 1 else 0.3, top=0.9,
                                wspace=0.35, hspace=0.55)
            legend_below(fig, axes[0][0], ncol=3, y=0.0)
            save(fig, "%s_view_%s" % (view, p))
            index.append(("%s_view_%s" % (view, p), "%s sessions seen from the %s" % (p, view),
                          "%d sessions" % len(recs), "APPENDIX"))

    # 3. best direct + best shared per VR participant, composite
    vr_ps = [p for p in order if p.startswith("P") and p in by_p]
    picks = []
    for p in vr_ps:
        for cond in ("Direct", "Shared"):
            cands = [r for r in by_p[p] if r["condition"] == cond]
            if cands:
                picks.append(min(cands, key=lambda r: rms(gap_mm(r))))
            else:
                picks.append(None)
    fig = plt.figure(figsize=(6.0, 2.6 * len(vr_ps) + 0.6))
    ax0 = None
    for k, rec in enumerate(picks):
        if rec is None:
            continue
        ax = fig.add_subplot(len(vr_ps), 2, k + 1, projection="3d")
        draw3d(ax, rec)
        ax.set_title("%s (gap %.0f mm)" % (label(rec), rms(gap_mm(rec))), fontsize=7.5, pad=0)
        ax0 = ax0 or ax
    fig.subplots_adjust(left=0.02, right=0.98, bottom=0.06, top=0.96, wspace=0.05, hspace=0.3)
    legend_below(fig, ax0, ncol=3, y=0.0)
    save(fig, "composite_best_per_operator")
    index.append(("composite_best_per_operator",
                  "Each operator's cleanest direct and assisted session, side by side",
                  "%d panels; 'gap' is the average distance between simulated and real hand" % sum(1 for x in picks if x),
                  "MAIN"))

    # 4. gap over time per participant
    for p in order:
        recs = by_p.get(p, [])
        if not recs:
            continue
        fig, axes = plt.subplots(len(recs), 1, figsize=(6.4, 1.15 * len(recs) + 0.8), sharex=False, squeeze=False)
        for k, rec in enumerate(recs):
            ax = axes[k][0]
            g = gap_mm(rec)
            t = [x - rec["t"][0] for x in rec["t"]]
            ax.plot(t, g, color=GREY, lw=0.7)
            ax.axhline(30, color=RED, lw=0.6, ls=":")
            ax.text(0.99, 0.85, "%s -- mean %.1f mm" % (label(rec), sum(g) / len(g)),
                    transform=ax.transAxes, ha="right", va="top", fontsize=7)
            ax.tick_params(labelsize=6)
            ax.set_ylim(0, max(35, max(g) * 1.05))
        axes[-1][0].set_xlabel("time into session (s)", fontsize=8)
        fig.text(0.01, 0.5, "gap between simulated and real hand (mm)", rotation=90, va="center", fontsize=8)
        fig.text(0.99, 0.01, "dotted red: 30 mm, the gripper's capture gate", ha="right", fontsize=6.5, color=RED)
        fig.subplots_adjust(left=0.1, right=0.98, bottom=0.12 if len(recs) > 2 else 0.25, top=0.98, hspace=0.35)
        save(fig, "gap_over_time_%s" % p)
        index.append(("gap_over_time_%s" % p, "How far apart the simulated and real hands were through each %s session" % p,
                      "means %s mm" % ", ".join("%.1f" % (sum(gap_mm(r)) / len(r["t"])) for r in recs), "APPENDIX"))

    # 5. centred shape overlays (operator hand vs simulated hand), per participant
    for p in order:
        recs = by_p.get(p, [])
        if not recs:
            continue
        nr, nc = grid_shape(len(recs))
        fig, axes = plt.subplots(nr, nc, figsize=(2.4 * nc + 0.4, 2.3 * nr + 0.9), squeeze=False)
        for k, rec in enumerate(recs):
            ax = axes[k // nc][k % nc]
            o = cols(rec["op"]); s = cols(rec["sim"])
            def centre(v):
                m = sum(v) / len(v); return [x - m for x in v]
            ax.plot(centre(smooth(o[0])), centre(smooth(o[2])), color=BLUE, lw=0.6, alpha=0.6,
                    label=LBL_OP + " (centred)")
            ax.plot(centre(s[0]), centre(s[2]), color=RED, lw=1.0, label=LBL_SIM + " (centred)")
            ax.set_title(short(rec), fontsize=7.5)
            ax.set_aspect("equal", adjustable="datalim")
            ax.tick_params(labelsize=6)
            ax.set_xlabel("forward, about its own mean (m)", fontsize=6.5)
            ax.set_ylabel("height, about its own mean (m)", fontsize=6.5)
        for k in range(len(recs), nr * nc):
            axes[k // nc][k % nc].axis("off")
        fig.subplots_adjust(left=0.08, right=0.98, bottom=0.16 if nr > 1 else 0.3, top=0.9,
                            wspace=0.35, hspace=0.55)
        legend_below(fig, axes[0][0], ncol=2, y=0.0)
        save(fig, "shape_overlay_%s" % p)
        index.append(("shape_overlay_%s" % p, "Shape of the operator's path against the robot's, each centred on its own mean (%s)" % p,
                      "%d sessions" % len(recs), "APPENDIX"))

    # 6. summary: per-session Cartesian sim-vs-real RMS
    items = sorted(sessions, key=lambda r: rms(gap_mm(r)))
    fig, ax = plt.subplots(figsize=(6.4, 0.22 * len(items) + 1.0))
    ys = range(len(items))
    for y, rec in zip(ys, items):
        v = rms(gap_mm(rec))
        ax.barh(y, v, color=BLUE if rec["condition"] == "Direct" else RED, height=0.7)
        ax.text(v + 0.5, y, label(rec), va="center", fontsize=6)
    ax.set_yticks([])
    ax.set_xlim(0, max(rms(gap_mm(r)) for r in items) * 1.35)
    ax.axvline(30, color=GREY, lw=0.6, ls=":")
    ax.text(30.5, -0.9, "30 mm capture gate", fontsize=6.5, color=GREY, va="top")
    ax.set_xlabel("typical gap between simulated and real hand over the session (mm)", fontsize=8)
    from matplotlib.patches import Patch
    ax.legend(handles=[Patch(color=BLUE, label="direct control"), Patch(color=RED, label="with assistance")],
              frameon=False, fontsize=7, loc="lower right")
    fig.subplots_adjust(left=0.02, right=0.98, bottom=0.12, top=0.99)
    save(fig, "summary_gap_all_sessions")
    stats = {}
    for coh in ("VR", "master"):
        vals = sorted(rms(gap_mm(r)) for r in sessions if r["cohort"] == coh)
        stats[coh] = (vals[0], vals[len(vals) // 2], vals[-1])
    index.append(("summary_gap_all_sessions", "Every session ranked by the gap between simulated and real hand",
                  "VR min/median/max %.1f/%.1f/%.1f mm; master %.1f/%.1f/%.1f mm" % (stats["VR"] + stats["master"]),
                  "MAIN"))

    # INDEX.md
    with open(os.path.join(HERE, "INDEX.md"), "w") as f:
        f.write("# Trajectory gallery\n\n")
        f.write("Generated by `make_trajectory_gallery.py` (`extract` then `plot`). Every figure is `.pdf` + `.png` (200 dpi).\n")
        f.write("Data: 23 VR sessions (P1-P5, `trail.csv`) and 9 master-arm sessions (M1-M3, `trail_extracted.csv`), every %d-th row. "
                "Operator's hand = `master_<hand>_x/y/z`; simulated robot hand = `ee_<hand>_x/y/z` (VR) or forward kinematics of `sim_<hand>_jN` (master cohort); "
                "real robot hand = forward kinematics of `real_<hand>_jN` through MoveIt's `/compute_fk`. FK failures: %d. "
                "The operator's trace is smoothed (9-sample moving average) and drawn faint; the two robot traces are untouched. "
                "Markers on VR panels are the FIRST CLUTCH ENGAGE (circle) and FIRST CLUTCH RELEASE (triangle): the recorded 'grip' is the clutch button and the gripper was never commanded. "
                "The 'shape overlay' figures centre each trace on its own mean so path SHAPES can be compared across two frames and scales; they are not an absolute overlay. "
                "'Gap' = Euclidean distance between the simulated and real hand at the same instant. "
                "CAVEAT: several sessions end with a flat tail in the gap trace (e.g. P2 target reaching, direct, from ~197 s): the real-joint columns stop updating while the emergency stop is held at the end of a session, so a flat tail is stale data, not a real standing offset; per-session means include those tails.\n\n" % (data["stride"], data.get("fk_failures", -1)))
        f.write("| file | plain title | numbers | verdict |\n| --- | --- | --- | --- |\n")
        for name, title, nums, verdict in index:
            f.write("| `%s` | %s | %s | %s |\n" % (name, title, nums, verdict))
        f.write("\nSingle-session 3-D panels: `3d_single_<participant>_<nn>.pdf` (one per session, same content as the grids).\n")
    print("stats", stats)


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "plot"
    if mode == "extract":
        extract()
    else:
        plot(json.load(open(CACHE)))
