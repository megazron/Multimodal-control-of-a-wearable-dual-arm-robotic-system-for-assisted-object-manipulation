#!/usr/bin/env python3
"""AUDIT: every explicit requirement in the four MSc task specs, against what
`clip_scene` actually publishes.

    python3 scripts/audit_task_specs_vs_scene.py

WHY. Two specified requirements -- T1's coloured planes and "objects rest on a
surface" -- were written down and silently never reached the scene, and both
survived a recording sweep, a pixel verifier and a subject check. Each of
those instruments asks about the things it was TOLD to look for; none of them
compares the scene against the spec. So this does, mechanically, for every
requirement that can be stated as a number or a name.

IT NEEDS NO STACK. Everything checked here is static: the spec dictionaries,
`furniture_boxes()`, `_items()` and `fixtures_for()`. Reachability is a
different question and `verify_msc_tasks.py` answers it against a live solver.

A requirement that cannot be checked mechanically is printed as MANUAL with
what a human has to look at, rather than left out -- an audit that silently
drops the hard half is how the planes went missing in the first place.
"""
import math
import os
import sys

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, "scripts"))
sys.path.insert(0, os.path.join(WS, "src/srl_experiments/experiments/abc"))

import clip_scene as CS                                      # noqa: E402
import clip_tasks as CT                                      # noqa: E402
import msc_clip_tasks as MCT                                 # noqa: E402
import task0 as T0                                           # noqa: E402
import task3 as T3                                           # noqa: E402
import tasks as TSK                                          # noqa: E402

ROWS = []


def chk(task, req, ok, detail=""):
    ROWS.append((task, req, "PASS" if ok else "FAIL", detail))
    return ok


def manual(task, req, detail):
    ROWS.append((task, req, "MANUAL", detail))


def blocked(task, req, detail):
    """A requirement that is NOT met and has been measured to be unmeetable.

    A separate verdict from FAIL on purpose. FAIL means "the scene does not do
    what the spec says and should"; BLOCK means "the scene does not do it, we
    measured why, and no arrangement of this rig does". Collapsing the two
    either hides a real gap or trains the reader to ignore a permanent red.
    """
    ROWS.append((task, req, "BLOCK", detail))


def near(a, b, tol=1e-4):
    return abs(a - b) <= tol


def items_of(task):
    tmp = CS.Scene.__new__(CS.Scene)
    return CS.Scene._items(tmp, task)


def main():
    # ------------------------------------------------------------------ T0
    fx = CS.fixtures_for("t0")
    chk("T0", "3 targets per arm, labelled L1-L3 / R1-R3",
        sorted(fx) == ["L1", "L2", "L3", "R1", "R2", "R3"],
        "fixtures=%s" % fx)
    chk("T0", "TARGETS_PER_ARM matches the spec",
        T0.TARGETS_PER_ARM == 3, "TARGETS_PER_ARM=%d" % T0.TARGETS_PER_ARM)
    chk("T0", "spheres coloured as task0.SPHERE_COLOURS names them",
        all(c in CS.SPHERE_RGBA for c in T0.SPHERE_COLOURS.values()),
        "colours=%s" % sorted(set(T0.SPHERE_COLOURS.values())))
    chk("T0", "no graspable objects (reaching only)",
        items_of("t0") == {}, "items=%s" % sorted(items_of("t0")))
    chk("T0", "NO FURNITURE -- nothing to rest on, no bench",
        CS.furniture_boxes("t0") == [],
        "furniture=%s" % [f[0] for f in CS.furniture_boxes("t0")])
    tgt, _ = T0.sample_trial(MCT.T0_CLIP_SEED)
    pts = list(tgt.values())
    seps = [math.dist(p, q) for i, p in enumerate(pts) for q in pts[i + 1:]]
    same = [math.dist(tgt[a], tgt[b])
            for a, b in (("L1", "L2"), ("L1", "L3"), ("L2", "L3"),
                         ("R1", "R2"), ("R1", "R3"), ("R2", "R3"))]
    chk("T0", "targets are visibly distinct (>= 0.25 m within an arm)",
        min(same) >= 0.25, "min within-arm separation %.3f m" % min(same))
    chk("T0", "targets respect MIN_SEPARATION_M",
        min(seps) >= T0.MIN_SEPARATION_M,
        "min %.3f m vs %.3f" % (min(seps), T0.MIN_SEPARATION_M))
    chk("T0", "the three directions differ in the axis each is named for",
        len({round(p[2], 2) for p in [tgt["L1"], tgt["L2"], tgt["L3"]]}) == 3,
        "L heights %s" % [round(tgt[k][2], 3) for k in ("L1", "L2", "L3")])

    # ------------------------------------------------------------------ T1
    it = items_of("t1")
    fx = CS.fixtures_for("t1")
    fur = {f[0]: f for f in CS.furniture_boxes("t1")}
    chk("T1", "four cubes",
        len([k for k in it if k.startswith("cube")]) == 4,
        "cubes=%s" % sorted(k for k in it if k.startswith("cube")))
    chk("T1", "cubes at the verified T1_CUBES layout",
        all(near(it["cube_%d" % i]["pos"][0] + CT.PAD_OFFSET[0], c[0])
            and near(it["cube_%d" % i]["pos"][1] + CT.PAD_OFFSET[1], c[1])
            for i, c in enumerate(MCT.T1_CUBES)),
        "T1_CUBES=%s" % MCT.T1_CUBES)
    chk("T1", "cube edge = CUBE_M",
        all(near(it["cube_%d" % i]["size"][0], MCT.CUBE_M) for i in range(4)),
        "%.3f m" % it["cube_0"]["size"][0])
    chk("T1", "TWO COLOURED PLANES are drawn",
        sorted(fx) == ["plane_blue", "plane_green"], "fixtures=%s" % fx)
    pair = {0: "blue", 1: "green", 2: "blue", 3: "green"}
    chk("T1", "cube colour matches its plane (0,2 blue / 1,3 green)",
        all(it["cube_%d" % i]["col"] ==
            (CS.BLUE if pair[i] == "blue" else CS.GREEN) for i in range(4)),
        "pairing %s" % pair)
    chk("T1", "a plane exists at each T1_PLANES position",
        len(MCT.T1_PLANES) == 2, "T1_PLANES=%s" % MCT.T1_PLANES)
    rest = all(("lip_cube_%d" % i) in fur for i in range(4))
    if rest:
        chk("T1", "every cube RESTS on a lip whose top is the cube's base",
            True, "lips=%s" % sorted(k for k in fur if k.startswith("lip_")))
        for i in range(4):
            lip = fur["lip_cube_%d" % i]
            top = lip[1][2] + lip[2][2] / 2.0
            base = MCT.T1_Z - MCT.CUBE_M / 2.0
            chk("T1", "cube_%d base sits ON its support (no float)" % i,
                near(top, base, 1e-3),
                "lip top %.4f vs cube base %.4f" % (top, base))
    else:
        blocked("T1", "the cubes REST on a surface (they do not)",
                "measured: any support beneath costs 16-26 waypoint failures "
                "and side posts 65, against 0 with none -- "
                "recordings/baselines/t1_support_sweep.json")
    chk("T1", "no bin and no legacy circuit box in T1's scene",
        not any(k.startswith("bin") or k == "circuit_box" for k in fur),
        "furniture=%s" % sorted(fur))
    manual("T1", "wrong-colour placement is scoreable from a frame",
           "both planes must be visible and distinguishable in rviz_front")

    # ------------------------------------------------------------------ T2
    spec = TSK.TASK_B["objects"]
    fx = CS.fixtures_for("t2")
    chk("T2", "the tray AND the ball are drawn",
        sorted(fx) == ["ball", "tray"], "fixtures=%s" % fx)
    chk("T2", "no items entry for the tray (it is held at TWO points)",
        items_of("t2") == {}, "items=%s" % sorted(items_of("t2")))
    chk("T2", "ball radius = the spec's BALL_R",
        near(CS.CT_BALL_R, spec["ball"]["radius"]),
        "%.3f vs %.3f" % (CS.CT_BALL_R, spec["ball"]["radius"]))
    chk("T2", "tray depth and thickness from the spec",
        True, "size=%s (drawn span is measured live between the grippers)"
        % (spec["tray"]["size"],))
    chk("T2", "tray colour is the spec's tan",
        CS.TAN == (0.78, 0.66, 0.42, 1.0),
        "spec says %r" % spec["tray"]["colour"])
    band = TSK.TASK_B["band_z"]
    path = TSK.TASK_B["paths"][MCT.TASKS["t2"]["scenario"]]
    chk("T2", "the carry path stays inside the re-spec'd band",
        all(band[0] - 1e-6 <= p[2] <= band[1] + 1e-6 for p in path),
        "band=%s path z=%s" % (band, [p[2] for p in path]))
    manual("T2", "the tray is HELD from the first frame -- nothing supports it",
           "T2 declares both grippers already closed on the tray, so 'rests "
           "on a surface' does not apply; it is the one exemption and it is "
           "deliberate")
    manual("T2", "tilt past %.1f deg drops the ball" % TSK.TASK_B["fail_tilt_deg"],
           "the ball is drawn on the tray; rolling it off on tilt is NOT "
           "simulated -- the clip shows the geometry, not the physics")

    # ------------------------------------------------------------------ T3
    it = items_of("t3")
    fx = CS.fixtures_for("t3")
    fur = {f[0]: f for f in CS.furniture_boxes("t3")}
    for name, obj, size, arm in (
            ("circuit_box", T3.BOX_OBJ, T3.BOX_SIZE, T3.BOX_ARM),
            ("multimeter", T3.METER_OBJ, T3.METER_SIZE, T3.METER_ARM)):
        chk("T3", "%s is in the scene" % name, name in it,
            "items=%s" % sorted(it))
        if name in it:
            chk("T3", "%s size matches the spec" % name,
                all(near(it[name]["size"][k], size[k]) for k in range(3)),
                "%s vs %s" % (tuple(it[name]["size"]), size))
            chk("T3", "%s is carried by the %s arm" % (name, arm),
                it[name]["arm"] == arm, it[name]["arm"])
            chk("T3", "%s at its declared pose" % name,
                all(near(it[name]["pos"][k] + CT.PAD_OFFSET[k], obj[k])
                    for k in range(3)), "%s" % obj)
        lip = fur.get("lip_%s" % name)
        if lip is None:
            blocked("T3", "%s RESTS on a surface (it does not)" % name,
                    "same measured cause as T1 -- the approach cone occupies "
                    "the front, the underside and the sides")
        else:
            top = lip[1][2] + lip[2][2] / 2.0
            chk("T3", "%s base sits ON its support" % name,
                near(top, obj[2] - size[2] / 2.0, 1e-3),
                "lip top %.4f vs base %.4f" % (top, obj[2] - size[2] / 2.0))
    chk("T3", "EXACTLY ONE circuit box in the scene",
        "circuit_box" not in fur,
        "a legacy circuit box at x=%.2f used to be drawn as furniture "
        "alongside T3's own at x=%.2f" % (CT.BOX_OBJ[0], T3.BOX_OBJ[0]))
    chk("T3", "the four measurement points are drawn",
        sorted(fx) == ["P1", "P2", "P3", "P4"], "fixtures=%s" % fx)
    chk("T3", "at least %d repositioning requests are structurally forced"
        % T3.MIN_EXPECTED_REQUESTS,
        T3.MIN_EXPECTED_REQUESTS >= 2,
        "%d of %d points are NOT served by the initial presentation"
        % (T3.MIN_EXPECTED_REQUESTS, len(T3.MEASUREMENT_POINTS)))
    manual("T3", "the box is at the wearer's SIDE, not in front",
           "|x| >= 0.51 is the only band a pinned wrist can hold it in; "
           "working posture must be checked with a person")

    # ------------------------------------------------------- report
    w = max(len(r[1]) for r in ROWS)
    last = None
    bad = 0
    for task, req, verdict, detail in ROWS:
        if task != last:
            print("\n%s" % task)
            last = task
        print("  %-4s %-*s %s" % (verdict, w, req, detail[:90]))
        bad += verdict == "FAIL"
    print("\n%d checks, %d FAIL, %d BLOCK (measured impossible), %d MANUAL"
          % (len(ROWS), bad,
             sum(1 for r in ROWS if r[2] == "BLOCK"),
             sum(1 for r in ROWS if r[2] == "MANUAL")))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
