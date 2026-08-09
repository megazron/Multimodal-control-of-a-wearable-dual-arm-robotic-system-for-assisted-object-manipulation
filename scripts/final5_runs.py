#!/usr/bin/env python3
"""Build recorder runs from the CURRENT five-task spec.

The 87 existing clips are stale twice over: they use the superseded nine-task
geometry (310 mm tray span, since respec'd to 500 mm) and they predate the
single-owner gripper fix, so their traces show the object grasped, dropped and
re-grasped in mid-air.

SCENE CODES ARE REUSED DELIBERATELY. `record_rviz`'s Scene renders by a task
code (t2 container-and-blocks, t3 tray, t6 sling, t7 targets), and that
rendering is proven. The five tasks map onto those codes rather than getting a
new renderer, so the only thing changing here is the GEOMETRY, which is what
was stale. Output directories are named f1..f5 so nothing collides with the
stale clips.

TASK 2 CARRIES A CAVEAT INTO EVERY CLIP. Grasping is not achievable under
DIRECT or VR: a top-down grasp needs 169.7 deg (left) / 164.6 deg (right) from
the anchor `orientation_mode: fixed` pins the wrist to, and nothing measures
the wrist to command it. These clips work because the recorder calls
/compute_ik DIRECTLY with the grasp quaternion, bypassing the teleoperation
orientation lock. A reader must not conclude that teleoperated grasping works,
so the caveat is burned into the overlay rather than left to the index.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/final5"))
import tasks as T                                            # noqa: E402

# Clips whose motion is produced by a direct /compute_ik call rather than by
# anything an operator could command. Anything listed here gets the caveat.
DIRECT_IK_TASKS = {"f2"}

CAVEAT = ("DIRECT-IK: not operator-commandable. "
          "Grasp needs 169.7/164.6 deg from the pinned wrist anchor.")


def _lerp(a, b, n):
    return [[a[i] + (b[i] - a[i]) * k / float(n - 1) for i in range(3)]
            for k in range(n)]


def build():
    """Return runs in the dict shape record_rviz.run_one expects."""
    runs = []

    def add(task, scen, paths, sc, scene_code, note="", conds=None):
        runs.append(dict(task=task, scenario=scen, paths=paths, sc=sc,
                         target=None, note=note, stance="",
                         scene_code=scene_code,
                         conditions=conds or ["direct", "assisted", "shared"],
                         caveat=CAVEAT if task in DIRECT_IK_TASKS else ""))

    Y = T.Y

    # ---- f1 POSITIONING. No object. The single-versus-both contrast is the
    # attention bottleneck, so all three blocks are recorded.
    for name, arms in (("S1_left_only", ("left",)),
                       ("S2_right_only", ("right",)),
                       ("S3_both", ("left", "right"))):
        tg = T.TASK1["targets"]
        paths = {}
        for a in arms:
            pts = tg[a]
            paths[a] = _lerp(pts[0], pts[2], 14)
        add("f1", name, paths,
            dict(targets={a: tg[a][2] for a in arms}), "t7",
            "positioning, %s" % "+".join(arms))

    # ---- f2 PICK AND PLACE. SHARED ONLY: modes 4 and above. There is no
    # DIRECT or VR condition to record, because the capability does not exist.
    for i, name in enumerate(("S1_near_pick", "S2_far_pick")):
        pick = T.TASK2["picks"]["left"][0 if i == 0 else 2]
        binp = T.TASK2["bins"]["left"]
        add("f2", name, {"left": _lerp(pick, binp, 16)},
            # The t2 renderer expects hold/release as well as pick/opening:
            # `hold` is where the container is held and `release` where a block
            # is dropped in. With one arm doing pick-and-place into a fixed
            # bin, the container is the bin itself and both sit at the bin.
            dict(fill_arm="left", hold_arm=None, pick=list(pick),
                 opening=list(binp), block_mm=40,
                 hold=list(binp), release=list(binp)), "t2",
            "pick and place, left arm, modes 4+ only",
            conds=["shared"])

    # ---- f3 RIGID CARRY and f4 COMPLIANT CARRY. Same paths, different
    # object: that contrast is the scientific point of running both.
    for task, code, obj in (("f3", "t3", "rigid tray, 500 mm span"),
                            ("f4", "t6", "compliant sling, 540 mm")):
        for name, wp in T.TASK3["paths"].items():
            dense = []
            for k in range(len(wp) - 1):
                dense += _lerp(wp[k], wp[k + 1], 8)
            half = T.TRAY_SEP / 2.0
            paths = {"left": [[p[0] + half, p[1], p[2]] for p in dense],
                     "right": [[p[0] - half, p[1], p[2]] for p in dense]}
            # sc["path"] is the tray CENTRE line. run_one derives the grasp
            # point from it as centre +/- sep/2, so it must be the centre and
            # not either gripper's own path.
            add(task, name, paths,
                dict(sep=T.TRAY_SEP, length=T.SLING_L, path=dense),
                code, obj)

    # ---- f5 DUAL PURSUIT. Two targets in the two DISJOINT reachable sets.
    import math
    for name, (vl, vr) in list(T.TASK5["speeds_m_s"].items())[:3]:
        amp = T.TASK5["amplitude_m"]
        cl = T.TASK5["centres"]["left"]
        cr = T.TASK5["centres"]["right"]
        n = 22
        pl, pr = [], []
        for k in range(n):
            u = 2 * math.pi * k / (n - 1.0)
            pl.append([cl[0] + amp * math.sin(u * max(vl, 0.05) * 8),
                       cl[1], cl[2] + amp * math.cos(u * max(vl, 0.05) * 8)])
            pr.append([cr[0] + amp * math.sin(u * max(vr, 0.05) * 8),
                       cr[1], cr[2] + amp * math.cos(u * max(vr, 0.05) * 8)])
        add("f5", name, {"left": pl, "right": pr},
            dict(targets={"left": cl, "right": cr}), "t7",
            "dual pursuit, speeds %.2f / %.2f m/s" % (vl, vr))

    return runs


if __name__ == "__main__":
    rs = build()
    n_clips = sum(len(r["conditions"]) for r in rs)
    print("five-task sweep: %d runs, %d clips, %d files at 7 angles"
          % (len(rs), n_clips, n_clips * 7))
    for r in rs:
        print("  %-3s %-16s %-28s %s"
              % (r["task"], r["scenario"], r["note"],
                 ",".join(r["conditions"])))
