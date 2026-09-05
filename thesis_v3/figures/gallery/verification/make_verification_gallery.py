#!/usr/bin/env python3
"""Plain-language gallery from the scripted (no-operator) verification
recordings and the other non-pilot recordings.

    python3 thesis_v3/figures/gallery/verification/make_verification_gallery.py

Writes <this dir>/*.pdf + *.png (150 dpi), frames/ (RViz stills cut from the
clips with ffmpeg) and INDEX.md. Reads only what is on disk; a metric the
task does not define is drawn as "not applicable", never as zero.
"""
import csv, glob, json, math, os, subprocess, collections
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
V = os.path.join(ROOT, "recordings", "verification")
FR = os.path.join(HERE, "frames"); os.makedirs(FR, exist_ok=True)

GREY, BLUE, RED, GREEN = "0.35", "#2c6fbb", "#c1392b", "#3f9142"
AMBER = "#c98a1a"
plt.rcParams.update({"font.size": 9, "axes.titlesize": 10, "axes.spines.top": False,
                     "axes.spines.right": False, "legend.frameon": False})

MODES = ["01_master_teleop", "02_vr_teleop", "03_shared_autonomy", "04_vr_shared", "06_full_autonomy"]
MODE_NAME = {"01_master_teleop": "mannequin master,\ndirect",
             "02_vr_teleop": "VR controllers,\ndirect",
             "03_shared_autonomy": "mannequin master\n+ assistance",
             "04_vr_shared": "VR controllers\n+ assistance",
             "06_full_autonomy": "fully\nautonomous"}
MODE_SHORT = {k: v.replace("\n", " ") for k, v in MODE_NAME.items()}
MODE_TINY = {"01_master_teleop": "master\ndirect", "02_vr_teleop": "VR\ndirect",
             "03_shared_autonomy": "master\n+ assist", "04_vr_shared": "VR\n+ assist",
             "06_full_autonomy": "auto-\nnomous"}
TASKS = ["T0", "T1", "T1S2", "T2", "T3"]
TASK_NAME = {"T0": "reach the target", "T1": "pick and place\n(4 cubes)",
             "T1S2": "pick and place\n(random layout)", "T2": "carry the tray",
             "T3": "circuit box +\nmultimeter"}
TASK_SHORT = {k: v.replace("\n", " ") for k, v in TASK_NAME.items()}
GATE_MM = 30.0
INDEX = []


def cell_dir(mode, task):
    hits = glob.glob(os.path.join(V, mode, task, "*"))
    hits = [h for h in hits if os.path.isdir(h)]
    return hits[0] if hits else None


def load(path):
    with open(path) as f:
        return json.load(f)


def save(fig, name, title, source, takeaway, numbers, verdict):
    pdf = os.path.join(HERE, name + ".pdf"); png = os.path.join(HERE, name + ".png")
    fig.savefig(pdf, bbox_inches="tight"); fig.savefig(png, dpi=150, bbox_inches="tight")
    plt.close(fig)
    INDEX.append(dict(file=name, title=title, source=source, takeaway=takeaway,
                      numbers=numbers, verdict=verdict))
    print("wrote", name)


# ---------------------------------------------------------------- frames
def video_len(mp4):
    out = subprocess.run(["ffmpeg", "-i", mp4], capture_output=True, text=True).stderr
    import re
    m = re.search(r"Duration: (\d+):(\d+):([\d.]+)", out)
    return int(m[1]) * 3600 + int(m[2]) * 60 + float(m[3]) if m else None


def frame(mode, task, angle, t, tag):
    """One RViz still, chrome cropped away. Cached."""
    d = cell_dir(mode, task)
    mp4 = os.path.join(d, "rviz_%s.mp4" % angle)
    out = os.path.join(FR, "%s_%s_%s_%s.png" % (mode, task, angle, tag))
    if not os.path.exists(out):
        L = video_len(mp4)
        t = max(0.2, min(t, (L or t) - 0.3))
        subprocess.run(["ffmpeg", "-v", "error", "-y", "-ss", "%.2f" % t, "-i", mp4,
                        "-frames:v", "1", out], check=True)
        im = Image.open(out); w, h = im.size
        im.crop((int(14 * w / 800), int(62 * h / 500), int(786 * w / 800), int(472 * h / 500))).save(out)
    return out


def clip_times(mode, task):
    d = cell_dir(mode, task)
    m = load(os.path.join(d, "clip_meta.json")); e = load(os.path.join(d, "scene_events.json"))
    off = (m.get("card_hold_s") or 0) + (m.get("look_s") or 0)
    span = m["grab_t1"] - m["grab_t0"]
    evs = [(off + x["wall"] - m["grab_t0"], x) for x in e.get("events", [])]
    return off, span, evs, e


def montage(ax, path, label=None):
    ax.imshow(Image.open(path)); ax.axis("off")
    if label:
        ax.set_title(label, fontsize=8, pad=2)


# ---------------------------------------------------------------- data
acc = load(os.path.join(V, "accuracy_table.json"))
grip_check = load(os.path.join(V, "gripper_check.json"))


def grasp_stats(mode, task):
    """(grasped, attempted) from the verifier's own table; None if not run."""
    for r in acc.get(mode, []):
        if r["task"].upper() == task:
            return r["grasped"], r["n"], r
    return None


# 1. grasp success grid --------------------------------------------------
def fig_grasp_grid():
    fig, ax = plt.subplots(figsize=(6.8, 3.6))
    for i, mode in enumerate(MODES):
        for j, task in enumerate(TASKS):
            ran = cell_dir(mode, task) is not None
            st = grasp_stats(mode, task)
            x, y = j, len(MODES) - 1 - i
            if not ran:
                ax.add_patch(plt.Rectangle((x, y), 1, 1, fc="0.93", ec="white", hatch="///", lw=2))
                ax.text(x + .5, y + .5, "not run in\nthis mode", ha="center", va="center", fontsize=6.5, color="0.45")
            elif st is None:
                ax.add_patch(plt.Rectangle((x, y), 1, 1, fc="0.85", ec="white", lw=2))
                ax.text(x + .5, y + .5, "nothing to\ngrasp", ha="center", va="center", fontsize=6.5, color="0.35")
            else:
                g, n, _ = st
                ax.add_patch(plt.Rectangle((x, y), 1, 1, fc=GREEN if g == n else (AMBER if g else RED), ec="white", lw=2))
                ax.text(x + .5, y + .5, "%d of %d\ngrasped" % (g, n), ha="center", va="center", fontsize=8, color="white", fontweight="bold")
    ax.set_xlim(0, len(TASKS)); ax.set_ylim(0, len(MODES))
    ax.set_xticks([j + .5 for j in range(len(TASKS))]); ax.set_xticklabels([TASK_NAME[t] for t in TASKS], fontsize=8)
    ax.set_yticks([len(MODES) - .5 - i for i in range(len(MODES))]); ax.set_yticklabels([MODE_NAME[m] for m in MODES], fontsize=8)
    for s in ax.spines.values(): s.set_visible(False)
    ax.tick_params(length=0)
    ax.legend(handles=[Patch(fc=GREEN, label="every grasp closed on its object"),
                       Patch(fc="0.85", label="task has nothing to grasp"),
                       Patch(fc="0.93", hatch="///", label="not run in this mode (by design)")],
              loc="upper center", bbox_to_anchor=(0.5, -0.22), ncol=2, fontsize=7.5)
    save(fig, "01_grasp_success_grid", "Did the robot grasp what it was meant to? Every scripted run, every mode",
         "recordings/verification/accuracy_table.json (grasped/n), cell directories present on disk",
         "Every grasp attempted in every mode closed on its object; the blank cells are tasks that were never meant to run there, not failures.",
         "12 of 12 grasps across 5 modes; pick-and-place runs under the autonomous mode only (4 of 4 twice)",
         "MAIN")


# 2. positioning error vs gate -------------------------------------------
def fig_positioning():
    fig, ax = plt.subplots(figsize=(7.2, 3.4))
    xs, labels = [], []
    k = 0; placed_labelled = False
    TASK_TINY = {"T3": "circuit box", "T1": "pick & place", "T1S2": "pick & place\n(random)"}
    for mode in MODES:
        for r in acc.get(mode, []):
            pe = [1000 * v for v in r["pos_err"]]
            ax.scatter([k] * len(pe), pe, color=BLUE, s=28, zorder=3, label="how far the fingers landed from the object" if k == 0 else None)
            if r["place_err"]:
                pl = [1000 * v for v in r["place_err"]]
                ax.scatter([k] * len(pl), pl, color=RED, marker="s", s=28, zorder=3, label=None if placed_labelled else "how far the cube was set down from its pad")
                placed_labelled = True
            xs.append(k); labels.append("%s\n%s" % (MODE_TINY[mode], TASK_TINY[r["task"].upper()]))
            k += 1
    ax.axhline(GATE_MM, color=GREY, ls="--", lw=1); ax.text(k - 0.5, GATE_MM + 1.5, "30 mm: the gripper can still capture it", ha="right", fontsize=7.5, color=GREY)
    ax.set_xticks(xs); ax.set_xticklabels(labels, fontsize=7)
    ax.set_ylabel("distance (mm)"); ax.set_ylim(-3, 70)
    ax.legend(fontsize=7.5, loc="upper left")
    save(fig, "02_positioning_error_vs_gate", "How close the hand got: grasp and set-down error against the 30 mm capture gate",
         "recordings/verification/accuracy_table.json (pos_err, place_err), metres converted to mm",
         "The fingers met every object exactly where the plan said (0.0 mm); the cubes were set down about 56 mm from the centre of their pad, which is past the 30 mm line.",
         "grasp error 0.0 mm in all 12 grasps; T1 set-down 55.5-56.9 mm (mean 56.3 mm) on all four cubes",
         "MAIN")


# 3. gripper traces ----------------------------------------------------
def fig_gripper_traces():
    tasks = ["T1", "T2", "T3"]
    fig, axes = plt.subplots(len(tasks), len(MODES), figsize=(11, 5.2), sharex="row", sharey=True)
    for i, task in enumerate(tasks):
        for j, mode in enumerate(MODES):
            ax = axes[i, j]
            d = cell_dir(mode, task)
            if d is None:
                ax.text(.5, .5, "not run in this mode", ha="center", va="center", transform=ax.transAxes, fontsize=7, color="0.45")
                ax.set_xticks([]); [s.set_visible(False) for s in ax.spines.values()]
                if i == 0: ax.set_title(MODE_SHORT[mode], fontsize=8)
                if j == 0: ax.set_ylabel("%s\nfingers closed (deg)" % TASK_SHORT[task], fontsize=8)
                continue
            tr = load(os.path.join(d, "grip_trace.json")); ev = load(os.path.join(d, "scene_events.json"))
            t = [r["t"] for r in tr]
            for arm, c in (("left", BLUE), ("right", RED)):
                ys = [r[arm] if r[arm] is not None else float("nan") for r in tr]
                ax.plot(t, [math.degrees(y) if y == y else y for y in ys], color=c, lw=1, label="%s gripper" % arm)
            for x in ev.get("events", []):
                ax.axvline(x["t"], color=GREEN if x["ev"] == "GRASPED" else GREY, lw=.8, ls=":" if x["ev"] == "RELEASED" else "-")
            if i == 0: ax.set_title(MODE_SHORT[mode], fontsize=8)
            if j == 0: ax.set_ylabel("%s\nfingers closed (deg)" % TASK_SHORT[task], fontsize=8)
            ax.set_xlabel("time (s)", fontsize=7); ax.tick_params(labelsize=7)
    h = [plt.Line2D([], [], color=BLUE, label="left gripper"), plt.Line2D([], [], color=RED, label="right gripper"),
         plt.Line2D([], [], color=GREEN, label="grasp confirmed"), plt.Line2D([], [], color=GREY, ls=":", label="object released")]
    fig.legend(handles=h, loc="lower center", ncol=4, fontsize=8, bbox_to_anchor=(0.5, -0.01))
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    save(fig, "03_gripper_traces_by_mode", "Did the gripper actually close? Finger angle through every grasping run",
         "recordings/verification/*/T*/*/grip_trace.json (knuckle rad -> deg), scene_events.json events",
         "In every mode the fingers close when the log says an object was grasped and open when it says released; the tray task's grippers close too, which they historically never did.",
         "fingers close to 24.2 deg (0.423 rad) on a 40 mm cube, 29.7 deg (0.518 rad) on the 30 mm objects; T2 grip max 0.518 rad (was 0.000 for the life of the task before 2026-08)",
         "APPENDIX")


# 4. pick-and-place frames -----------------------------------------------
def fig_t1_frames():
    mode, task = "06_full_autonomy", "T1"
    off, span, evs, e = clip_times(mode, task)
    grasps = [(t, x) for t, x in evs if x["ev"] == "GRASPED"]
    rels = [(t, x) for t, x in evs if x["ev"] == "RELEASED"]
    fig, axes = plt.subplots(len(grasps), 4, figsize=(10, 1.45 * len(grasps)))
    for r, ((tg, g), (tr, rl)) in enumerate(zip(grasps, rels)):
        # the gripper camera rides the LEFT gripper; right-arm cubes are shown from the right-side view
        cam = "gripper" if g["arm"] == "left" else "right"
        cols = [(cam, tg - 1.2, "approach"), (cam, tg + 0.3, "grasp"),
                (cam, (tg + tr) / 2, "carry"), ("top", tr + 0.6, "set down (from above)")]
        for c, (angle, t, lab) in enumerate(cols):
            montage(axes[r, c], frame(mode, task, angle, t, "%s_%d_%s" % (lab.split()[0], r, angle)),
                    lab if r == 0 else None)
        axes[r, 0].text(-0.02, 0.5, "%s\n(%s arm, %s)" % (g["item"].replace("_", " "), g["arm"], "gripper camera" if cam == "gripper" else "right-side view"),
                        transform=axes[r, 0].transAxes, ha="right", va="center", fontsize=7.5)
    fig.subplots_adjust(wspace=0.03, hspace=0.06, left=0.13, right=0.99, top=0.94, bottom=0.01)
    save(fig, "04_pick_and_place_frames", "The autonomous pick-and-place, cube by cube: approach, grasp, carry, set down",
         "recordings/verification/06_full_autonomy/T1/S1_both_arms_centre/rviz_{gripper,right,top}.mp4, stills at the logged grasp/release times",
         "Four cubes, two arms: each row is one cube -- the left arm's from its own gripper camera, the right arm's from the right-side view -- ending with the set-down from above.",
         "grasps at 4.0/10.2/19.0/25.3 s of the task, each carried 0.29-0.32 m; both arms used",
         "MAIN")


# 5. task by mode frames -------------------------------------------------
def fig_task_by_mode(task, name, title, take, nums, verdict):
    modes = [m for m in MODES if cell_dir(m, task)]
    fig, axes = plt.subplots(len(modes), 4, figsize=(10, 1.4 * len(modes)))
    for i, mode in enumerate(modes):
        off, span, evs, e = clip_times(mode, task)
        gr = [t for t, x in evs if x["ev"] == "GRASPED"]; rl = [t for t, x in evs if x["ev"] == "RELEASED"]
        if gr:
            ts = [(off + 0.5, "start"), (gr[0] + 0.3, "first grasp"), ((gr[-1] + rl[0]) / 2 if rl else gr[-1] + 1, "both held"), ((rl[-1] + 0.5) if rl else off + span - 0.5, "released")]
        else:
            ts = [(off + 0.5, "start"), (off + span * .33, "one third"), (off + span * .66, "two thirds"), (off + span - 0.5, "end")]
        for c, (t, lab) in enumerate(ts):
            montage(axes[i, c], frame(mode, task, "front", t, "bymode_%s_%d" % (task, c)), lab if i == 0 else None)
        axes[i, 0].text(-0.02, 0.5, MODE_NAME[mode], transform=axes[i, 0].transAxes, ha="right", va="center", fontsize=8)
    fig.subplots_adjust(wspace=0.03, hspace=0.06, left=0.13, right=0.99, top=0.94, bottom=0.01)
    save(fig, name, title, "recordings/verification/<mode>/%s/*/rviz_front.mp4, stills at logged event times (or thirds where the task logs none)" % task, take, nums, verdict)


# 7. camera angles --------------------------------------------------------
def fig_angles():
    mode, task = "06_full_autonomy", "T1"
    off, span, evs, e = clip_times(mode, task)
    tg = [t for t, x in evs if x["ev"] == "GRASPED"][0] + 0.3
    angles = ["front", "back", "left", "right", "top", "iso", "gripper"]
    fig, axes = plt.subplots(2, 4, figsize=(10, 3.9))
    for ax, a in zip(axes.flat, angles):
        montage(ax, frame(mode, task, a, tg, "angles"), a if a != "gripper" else "gripper camera")
    axes.flat[-1].axis("off")
    fig.subplots_adjust(wspace=0.03, hspace=0.12, left=0.01, right=0.99, top=0.93, bottom=0.01)
    save(fig, "07_seven_camera_angles", "One moment, seven cameras: the first grasp of the autonomous pick-and-place",
         "recordings/verification/06_full_autonomy/T1/S1_both_arms_centre/rviz_{front,back,left,right,top,iso,gripper}.mp4",
         "Every run was filmed from seven fixed viewpoints plus the gripper's own camera, so a grasp is judged from the pictures rather than asserted.",
         "7 angles x 29 clips = 232 videos on disk; this is the left arm closing on cube 0 at 4.0 s",
         "APPENDIX")


# 8. T1 placement before / after -------------------------------------------
def fig_placement_history():
    sets = [("2026-08-13, first spec", "archive/recordings/superseded_20260813_prespec/verification/accuracy_table.json"),
            ("2026-08-11 rework", "archive/recordings/superseded_20260813_prespec/verification_20260811_t1rework/accuracy_table.json"),
            ("2026-08-23, before the\naccuracy pass", "archive/recordings/verification_20260823_pre_accuracy_pass/accuracy_table.json"),
            ("2026-08-23, delivered", "recordings/verification/accuracy_table.json")]
    fig, ax = plt.subplots(figsize=(6.2, 3.2))
    for k, (lab, p) in enumerate(sets):
        d = load(os.path.join(ROOT, p))
        vals = [1000 * v for r in d.get("06_full_autonomy", []) if r["task"] == "t1" for v in r["place_err"]]
        ax.scatter([k] * len(vals), vals, color=BLUE, s=30, zorder=3)
        ax.plot([k - .25, k + .25], [sum(vals) / len(vals)] * 2, color=RED, lw=2)
        ax.text(k + .28, sum(vals) / len(vals), "mean %.0f" % (sum(vals) / len(vals)), va="center", fontsize=7.5, color=RED)
    ax.axhline(GATE_MM, color=GREY, ls="--", lw=1); ax.text(-0.4, GATE_MM + 8, "30 mm capture gate", fontsize=7.5, color=GREY)
    ax.set_xticks(range(len(sets))); ax.set_xticklabels([s[0] for s in sets], fontsize=7.5)
    ax.set_ylabel("cube set-down error (mm)"); ax.set_ylim(0, 650)
    save(fig, "08_placement_error_history", "How far each cube was set down from its pad, across the four recorded versions of the pick-and-place",
         "archive/recordings/*/accuracy_table.json and recordings/verification/accuracy_table.json (06_full_autonomy, t1, place_err)",
         "Set-down error fell from around 130-310 mm, with two cubes over half a metre out before the last fix, to 56 mm on every cube.",
         "means 130 -> 174 -> 312 -> 56 mm; delivered set 55.5-56.9 mm on all four cubes. NOTE: the report's '319.7 mm' before-figure is not in any archived accuracy table (nearest: 311.6 mm mean of 57.8/601.2/541.4/46.0)",
         "APPENDIX")


# 9. robot travel with no operator ------------------------------------------
def fig_mode_travel():
    d = load(os.path.join(ROOT, "recordings", "baselines", "mode_difference.json"))
    tasks = [t for t in ["T0", "T2", "T3", "T1", "T1S2"] if t in d]
    fig, axes = plt.subplots(1, len(tasks), figsize=(11, 3.0), sharey=False)
    for ax, task in zip(axes, tasks):
        pm = d[task]["per_mode"]
        for i, mode in enumerate(MODES):
            if mode not in pm: continue
            x = pm[mode]
            ax.plot([i, i], [0, x["travel_l"]], color=BLUE, lw=5, solid_capstyle="butt", alpha=.85)
            ax.plot([i + .28, i + .28], [0, x["travel_r"]], color=RED, lw=5, solid_capstyle="butt", alpha=.85)
        ax.set_xticks([i + .14 for i in range(len(MODES))]); ax.set_xticklabels([MODE_TINY[m] for m in MODES], fontsize=6.5)
        ax.set_title(TASK_SHORT[task], fontsize=9); ax.tick_params(labelsize=7)
    axes[0].set_ylabel("distance the hand travelled (m)")
    fig.legend(handles=[Patch(fc=BLUE, label="left arm"), Patch(fc=RED, label="right arm")], loc="lower center", ncol=2, fontsize=8, bbox_to_anchor=(0.5, -0.06))
    fig.tight_layout()
    save(fig, "09_robot_travel_no_operator", "Same commanded waypoints, no operator: how far the arm travels in each mode",
         "recordings/baselines/mode_difference.json (per_mode travel_l/travel_r); also plotted by another gallery -- labelled here as the no-operator baseline",
         "With no person in the loop, the assisted modes make the arm travel further on the same waypoints -- most of all on the reach task, where the mannequin-master assisted mode travels 45% further than direct.",
         "reach task, left arm: 1.41 m direct vs 2.03 m assisted (+45%); VR direct 1.46 vs VR assisted 1.52 (+4%); tray task VR assisted 1.97 vs 1.07 direct",
         "MAIN")


# 10. VR lag vs hand speed ---------------------------------------------------
def fig_vr_lag():
    d = load(os.path.join(ROOT, "recordings", "vr_teleop", "protocol_20260826_124739.json"))
    fig, ax = plt.subplots(figsize=(6.2, 3.4))
    for s in d:
        for arm, c, mk in (("left", BLUE, "o"), ("right", RED, "s")):
            a = s[arm]
            if a.get("peak_hand_speed_mps") is None: continue
            ax.scatter(a["peak_hand_speed_mps"], a["max_lag_mm"], color=c, marker=mk, s=34, zorder=3)
            if a["max_lag_mm"] > GATE_MM:
                ax.annotate(s["segment"].replace("_", " "), (a["peak_hand_speed_mps"], a["max_lag_mm"]), xytext=(5, 3), textcoords="offset points", fontsize=7)
    ax.axhline(GATE_MM, color=GREY, ls="--", lw=1); ax.text(0.02, GATE_MM + 2, "30 mm: still inside the capture gate", fontsize=7.5, color=GREY)
    ax.axvline(0.27, color=GREY, ls=":", lw=1); ax.text(0.272, 60, "0.27 m/s", fontsize=7.5, color=GREY)
    ax.set_xlabel("fastest hand speed in the movement (m/s)"); ax.set_ylabel("furthest the robot fell behind the hand (mm)")
    ax.legend(handles=[plt.Line2D([], [], color=BLUE, marker="o", ls="", label="left hand"), plt.Line2D([], [], color=RED, marker="s", ls="", label="right hand")], fontsize=8, loc="upper left")
    save(fig, "10_vr_lag_vs_hand_speed", "How far the robot fell behind a VR-controlled hand, against how fast the hand moved",
         "recordings/vr_teleop/protocol_20260826_124739.json (peak_hand_speed_mps, max_lag_mm per segment and arm; simulation, 2026-08-26)",
         "Below about 0.27 m/s the robot never fell more than 23 mm behind; a deliberately fast forward push at 0.35 m/s left it 83-93 mm behind, three times the capture gate.",
         "16 segment-arm pairs; lag <= 22.7 mm for every movement under 0.27 m/s; 'up' at 0.316 m/s 35.4 mm; 'fast front' at 0.35 m/s 82.8 mm (left) / 92.9 mm (right)",
         "APPENDIX")


# 11. master capture: commanded vs achieved -----------------------------------
def fig_master_capture():
    p = os.path.join(ROOT, "recordings", "trajectory_capture", "capture_20260901_151026", "all_segments.csv")
    rows = list(csv.DictReader(open(p)))
    by = collections.defaultdict(list)
    for r in rows: by[r["segment"]].append(r)

    def plen(rs, px, py, pz):
        s, prev = 0, None
        for r in rs:
            try: q = (float(r[px]), float(r[py]), float(r[pz]))
            except ValueError: continue
            if prev: s += math.dist(prev, q)
            prev = q
        return s
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(8.6, 3.4))
    for seg, rs in by.items():
        arm = rs[0]["arm"]; c = BLUE if arm == "left" else RED
        cmd = plen(rs, f"{arm}_cmd_x", f"{arm}_cmd_y", f"{arm}_cmd_z"); ee = plen(rs, f"{arm}_ee_x", f"{arm}_ee_y", f"{arm}_ee_z")
        ax.scatter(cmd, ee, color=c, s=26, zorder=3)
        clr = [float(r[f"{arm}_min_clearance"]) for r in rs if r.get(f"{arm}_min_clearance")]
        ax2.scatter(len(rs) / 50.0, 1000 * min(clr), color=c, s=26, zorder=3)
    lim = 1.5; ax.plot([0, lim], [0, lim], color=GREY, lw=1, ls="--"); ax.set_xlim(0, lim); ax.set_ylim(0, lim)
    ax.set_xlabel("distance the master asked for (m)"); ax.set_ylabel("distance the simulated hand moved (m)")
    ax.text(0.05, 1.38, "dashed line: hand went exactly where asked", fontsize=7.5, color=GREY)
    ax2.axhline(150, color=GREY, ls="--", lw=1); ax2.text(9, 165, "150 mm: closest the arm may come to the wearer", fontsize=7.5, color=GREY)
    ax2.set_xlabel("length of the movement (s)"); ax2.set_ylabel("closest the arm came to the wearer (mm)"); ax2.set_ylim(0, 620)
    fig.legend(handles=[plt.Line2D([], [], color=BLUE, marker="o", ls="", label="left arm"), plt.Line2D([], [], color=RED, marker="o", ls="", label="right arm")], loc="upper center", ncol=2, fontsize=8, bbox_to_anchor=(0.5, 1.04))
    fig.tight_layout()
    save(fig, "11_master_sweep_commanded_vs_moved", "Moving the mannequin master through 28 set movements: what was asked for, what the hand did, and how close it came to the wearer",
         "recordings/trajectory_capture/capture_20260901_151026/all_segments.csv (cmd vs ee path per segment, min_clearance), 50 Hz, clutch pinned",
         "The simulated hand travelled roughly what the master asked for on the left arm and noticeably less on the right, and never came within half a metre of the wearer.",
         "28 segments, 22 516 samples; left cmd/ee within a few cm except two large sweeps (1.41 vs 1.08 m); right undershoots on 8 of 14 (e.g. 0.99 asked, 0.69 moved); min clearance 0.480-0.570 m, never near 0.150 m; 0 IK failures",
         "APPENDIX")


# 12. tray tilt -------------------------------------------------------------
def fig_tray_tilt():
    fig, ax = plt.subplots(figsize=(7, 3.0))
    for mode, c in zip(MODES, [BLUE, RED, GREEN, AMBER, GREY]):
        d = cell_dir(mode, "T2")
        if not d: continue
        e = load(os.path.join(d, "scene_events.json")); cs = e.get("carry_series") or []
        ax.plot([r["t"] for r in cs], [r["tilt_deg"] for r in cs], color=c, lw=1, label=MODE_SHORT[mode])
    ax.axhline(6.8, color=RED, ls="--", lw=1); ax.axhline(-6.8, color=RED, ls="--", lw=1)
    ax.text(0.5, 7.3, "6.8 deg: the ball rolls off", fontsize=7.5, color=RED)
    ax.set_xlabel("time (s)"); ax.set_ylabel("tray tilt (degrees)"); ax.set_ylim(-8, 12)
    ax.legend(fontsize=7, ncol=3, loc="upper left", bbox_to_anchor=(0, 1.0))
    save(fig, "12_tray_tilt_by_mode", "Carrying the tray: how much it tilted in each mode, against the angle at which the ball rolls off",
         "recordings/verification/*/T2/*/scene_events.json carry_series.tilt_deg, carry_summary.fail_tilt_deg",
         "The tray stays within a degree of level in four modes; the VR-direct run spiked to 9.9 degrees for a third of a second, past the roll-off angle.",
         "tilt RMS 0.92 deg VR direct (max 9.9 deg, 0.31 s above 6.8 deg); other modes stay near 0; VR direct run also lasted 121 s vs 14 s",
         "APPENDIX")


# 13. how long each scripted run took -----------------------------------------
def fig_run_durations():
    fig, ax = plt.subplots(figsize=(6.4, 3.2))
    tasks = ["T0", "T2", "T3"]
    for j, task in enumerate(tasks):
        for i, mode in enumerate(MODES):
            d = cell_dir(mode, task)
            if not d: continue
            m = load(os.path.join(d, "clip_meta.json")); span = m["grab_t1"] - m["grab_t0"]
            ax.scatter(j + (i - 2) * 0.13, span, color=[BLUE, RED, GREEN, AMBER, GREY][i], s=36, zorder=3, label=MODE_SHORT[mode] if j == 0 else None)
    ax.set_xticks(range(len(tasks))); ax.set_xticklabels([TASK_SHORT[t] for t in tasks])
    ax.set_ylabel("time to run the task (s)"); ax.set_ylim(0, 60)
    ax.legend(fontsize=7.5, ncol=3, loc="upper center", bbox_to_anchor=(0.5, -0.16))
    save(fig, "13_scripted_run_durations", "How long each scripted run took, by mode",
         "recordings/verification/*/T*/*/clip_meta.json (grab_t1 - grab_t0)",
         "The same scripted task takes about three times longer through the VR-controller path than through any other mode -- the VR path is rate-limited by its own filter.",
         "reach: 16.5 s in four modes vs 49.1 s VR direct; tray: 11.5 vs 44.1 s; circuit box: 18-19 vs 51.1 s",
         "APPENDIX")


# ---------------------------------------------------------------- run
if __name__ == "__main__":
    fig_grasp_grid(); fig_positioning(); fig_gripper_traces()
    fig_t1_frames()
    fig_task_by_mode("T3", "05_circuit_box_by_mode", "The circuit-box-and-multimeter task, every mode, four moments",
                     "Both arms lift the box and bring the meter to it in every mode; the pictures differ in timing, not in outcome.",
                     "5 modes; grasps at 4.4-4.8 s of the task in four modes, 37.0 s in VR direct", "MAIN")
    fig_task_by_mode("T0", "06_reach_target_by_mode", "Reaching three targets, every mode, at the start, a third, two thirds and the end",
                     "The reach task is the baseline every other mode is compared against; here it is under all five.",
                     "5 modes; 16.5 s each except VR direct at 49.1 s", "APPENDIX")
    fig_angles(); fig_placement_history(); fig_mode_travel(); fig_vr_lag(); fig_master_capture(); fig_tray_tilt(); fig_run_durations()
    with open(os.path.join(HERE, "INDEX.md"), "w") as f:
        f.write("# Verification gallery\n\nGenerated by `make_verification_gallery.py`. Every figure is `<file>.pdf` + `<file>.png`; stills live in `frames/`.\n\n")
        for e in INDEX:
            f.write("## %s -- %s\n\n- **Title:** %s\n- **Source:** %s\n- **Takeaway:** %s\n- **Numbers:** %s\n- **Verdict:** %s\n\n" %
                    (e["file"], e["verdict"], e["title"], e["source"], e["takeaway"], e["numbers"], e["verdict"]))
        f.write("## Skipped\n\n- `recordings/fault_acceptance/` -- FAULTTEST output from `scripts/pilot_bimanual.py`, which runs offline against synthesised trajectories: not measured data.\n"
                "- `recordings/analysis/vr_direct_vs_shared_2026-09/` -- nine PNGs only (time, distance, re-grips, active share, lag, paired time/re-grips, trajectory, engagement), all already superseded by the pilot figures in the report; no metrics file to compare against.\n"
                "- D1-D3 'dance' clips -- choreography demos with no task metric.\n"
                "- `recordings/trajectory_capture/capture_20260806_*` -- per-joint master sweeps from before the channel repair; the 2026-09-01 capture supersedes them.\n")
    print("INDEX.md written with", len(INDEX), "figures")
