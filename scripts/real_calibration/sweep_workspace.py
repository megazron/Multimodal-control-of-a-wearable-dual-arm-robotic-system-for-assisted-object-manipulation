#!/usr/bin/env python3
"""Measure what each real arm can actually reach, and record every point.

    python3 scripts/real_calibration/sweep_workspace.py --dry-run
    python3 scripts/real_calibration/sweep_workspace.py --arm left
    python3 scripts/real_calibration/sweep_workspace.py --arm both --step 0.05

WHAT THIS PRODUCES, per arm and per direction: the furthest point the arm
REACHED -- not the furthest an optimiser believed in. Each accepted point is
driven to on the real hardware, its measured joints read back, its FK EE
recorded, its wearer clearance recorded, and both room cameras captured.

WHY IT IS DONE BY WALKING OUTWARD. A single IK call at a far target answers
"is there a pose", which is not the question. The question is "can the arm GET
there from where it is", and that is a property of the PATH. So each step is
one `SafeArm.goto`, which densifies and checks the whole sweep before sending
it, and the walk stops at the first refusal -- recording WHICH of the three
possible reasons stopped it:

    unreachable      no IK solution, after a seeded search
    clearance        a pose or a path inside the wearer floor
    did-not-arrive   commanded and the arm did not get there

Those are three different facts about the workspace and collapsing them into
"unreachable" is how `arm_reach_extents.json` came to say "outside the arm's
902 mm reach" about a point the arm could reach.

THE CAMERAS ARE PART OF THE MEASUREMENT, not decoration. Every point carries
a RealSense depth+colour capture and an HD frame, so the same run that maps
the workspace also produces the data the camera->robot extrinsic is solved
from. A background capture is taken first with the arm at home.

SAFETY. Nothing moves until the whole set of first steps has been checked in
simulation. The arm's joint stream is watched throughout and the run ABORTS
on staleness rather than continuing to publish into a dead session -- the
left arm on this rig dropped off the network three times in one afternoon and
a commanding script that does not notice will keep believing its own plan.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(WS, "scripts"))
sys.path.insert(0, os.path.join(WS, "config"))

import numpy as np

from safe_motion import SafeArm, MotionRefused, ArmVanished, check_path, \
    FLOOR_M, max_error
from arm_ik import ArmIK
import solve_home_pose as SHP

# 26 directions: every combination of -1/0/+1 except standing still. Coarse
# enough to finish, fine enough that the envelope is not a guess between axes.
def _dirs():
    out = []
    for x in (-1, 0, 1):
        for y in (-1, 0, 1):
            for z in (-1, 0, 1):
                if x == y == z == 0:
                    continue
                v = np.array([x, y, z], float)
                out.append(v / np.linalg.norm(v))
    return out


DIRS = _dirs()
DIR_NAMES = []
for x in (-1, 0, 1):
    for y in (-1, 0, 1):
        for z in (-1, 0, 1):
            if x == y == z == 0:
                continue
            DIR_NAMES.append("%+d%+d%+d" % (x, y, z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default="left",
                    choices=["left", "right", "both"])
    ap.add_argument("--step", type=float, default=0.05,
                    help="metres per outward step")
    ap.add_argument("--max-steps", type=int, default=24)
    ap.add_argument("--floor", type=float, default=FLOOR_M)
    # EVERY RUN GETS ITS OWN DIRECTORY, so a restart can never destroy the
    # recording a previous attempt made. Earlier attempts today wrote into
    # one shared folder and each retry cleared it first -- 43 captures and
    # 42 MB of real arm-and-camera data were lost that way, which is exactly
    # the data the whole exercise exists to collect.
    ap.add_argument("--out", default=None)
    ap.add_argument("--run", default=None,
                    help="run directory name; defaults to a timestamp")
    ap.add_argument("--dry-run", action="store_true",
                    help="plan and check everything, move nothing")
    ap.add_argument("--no-cameras", action="store_true")
    ap.add_argument("--dirs", default=None,
                    help="comma list of direction names, e.g. +1+0+0,+0+1+0")
    a = ap.parse_args()

    if a.out is None:
        tag = a.run or time.strftime("run_%Y%m%d_%H%M%S")
        a.out = os.path.join(WS, "recordings", "real_calibration", tag)

    import solve_home_pose as SHP
    import home_positions as hp
    sc = SHP.Scorer()

    dirs = list(zip(DIR_NAMES, DIRS))
    if a.dirs:
        want = set(s.strip() for s in a.dirs.split(","))
        dirs = [(n, v) for n, v in dirs if n in want]
        if not dirs:
            sys.exit("no directions matched %s" % a.dirs)
    arms = ["left", "right"] if a.arm == "both" else [a.arm]
    os.makedirs(a.out, exist_ok=True)

    print("=" * 72)
    print("REAL ARM WORKSPACE SWEEP")
    print("=" * 72)
    print("  arms          %s" % ", ".join(arms))
    print("  directions    %d" % len(dirs))
    print("  step          %.3f m, up to %d steps (%.2f m)"
          % (a.step, a.max_steps, a.step * a.max_steps))
    print("  wearer floor  %.3f m, checked over the WHOLE path"
          % a.floor)
    print("  output        %s" % a.out)
    print("  mode          %s" % ("DRY RUN -- nothing will move"
                                  if a.dry_run else "LIVE"))

    # ---------------------------------------------------- simulated preflight
    print("\nPREFLIGHT (simulated, no hardware):")
    plan = {}
    for arm in arms:
        ik = ArmIK(sc, arm, point="ee", table=15000)
        qh = np.array(hp.load_home_radians(arm))
        home_ee = ik._pt(qh)
        ok_dirs = 0
        rows = []
        for nm, v in dirs:
            tgt = home_ee + v * a.step
            # The preflight only has to answer "is the first step sane".
            # A full 60-seed search per direction, twice over for --arm both,
            # is minutes of nothing moving.
            q, e, why = ik.solve(tgt, tol=0.006, floor=a.floor, seam=0.25,
                                 n_seeds=12)
            if q is None:
                rows.append((nm, False, why))
                continue
            okp, whyp, worst, ns = check_path(sc, arm, qh, q, floor=a.floor)
            rows.append((nm, okp, whyp if not okp
                         else "first step OK, worst %.3f m" % worst))
            ok_dirs += 1 if okp else 0
        plan[arm] = dict(ik=ik, qh=qh, home_ee=home_ee, rows=rows)
        print("  %-5s home EE %s | %d of %d directions have a safe first step"
              % (arm, np.round(home_ee, 3).tolist(), ok_dirs, len(dirs)))
        for nm, okp, why in rows:
            if not okp:
                print("      %-8s BLOCKED: %s" % (nm, why[:78]))
    if a.dry_run:
        print("\nDRY RUN complete -- nothing moved.")
        json.dump({arm: dict(home_ee=plan[arm]["home_ee"].tolist(),
                             rows=[(n, bool(o), w)
                                   for n, o, w in plan[arm]["rows"]])
                   for arm in arms},
                  open(os.path.join(a.out, "preflight.json"), "w"), indent=1)
        print("wrote %s" % os.path.join(a.out, "preflight.json"))
        return 0

    # --------------------------------------------------------------- cameras
    cams = None
    if not a.no_cameras:
        from scene_cameras import SceneRealSense, SceneWebcam, CameraError
        try:
            rs_cam = SceneRealSense().open()
            hd = SceneWebcam().open()
            cams = (rs_cam, hd)
            print("\ncameras: RealSense K=%s, HD open"
                  % np.round(rs_cam.K[:4], 2).tolist())
        except CameraError as exc:
            print("\ncameras UNAVAILABLE: %s" % exc)
            print("continuing WITHOUT camera capture -- the workspace numbers "
                  "are still valid, the extrinsic data is not collected")
            cams = None

    import rclpy
    from rclpy.node import Node
    rclpy.init()
    node = Node("sweep_workspace")
    results = {}
    try:
        for arm in arms:
            results[arm] = sweep_one(node, arm, sc, plan[arm], dirs, a, cams)
    finally:
        if cams:
            cams[0].close()
            cams[1].close()
        node.destroy_node()
        rclpy.shutdown()

    path = os.path.join(a.out, "workspace.json")
    json.dump(results, open(path, "w"), indent=1)
    print("\nwrote %s" % path)
    return 0


def sweep_one(node, arm, sc, pl, dirs, a, cams):
    arm_io = SafeArm(node, arm, sc, floor=a.floor)
    ik, qh, home_ee = pl["ik"], pl["qh"], pl["home_ee"]
    print("\n" + "=" * 72)
    print("SWEEPING %s ARM" % arm.upper())
    print("=" * 72)
    q0 = arm_io.q()
    if q0 is None:
        raise ArmVanished("no %s joint state -- is the bridge up?" % arm)
    off = float(np.degrees(max_error(q0, qh)))
    print("  at start, %.2f deg from home" % off)
    if off > 3.0:
        print("  returning to home first")
        arm_io.goto(qh, "home", timeout_s=120.0)
    arm_io.hold_here()

    outdir = os.path.join(a.out, arm)
    os.makedirs(outdir, exist_ok=True)
    if cams:
        d, c, live = cams[0].grab(20)
        np.savez_compressed(os.path.join(outdir, "background.npz"),
                            depth=d, color=c, q=arm_io.q())
        print("  background captured (depth %.0f%% valid, %d distinct)"
              % (live["valid_frac"] * 100, live["unique_content"]))

    out = {"home_ee": home_ee.tolist(), "directions": {}, "points": []}
    # WRITE EVERY POINT THE MOMENT IT IS MEASURED.
    #
    # workspace.json was written once, after every direction of every arm had
    # finished. Anything that ended the run early -- an arm dropping, a
    # ctrl-C, a kill -- discarded the entire measurement record. Measured
    # today: the left arm completed four directions (0.450, 0.300, 0.450 and
    # 0.600 m, including two stopped by the wearer rather than by reach) and
    # saved ZERO files, because it ran without cameras and the per-point npz
    # was the only thing being written incrementally.
    #
    # A measurement that exists only in RAM until the end is a measurement
    # you do not have.
    jl = open(os.path.join(outdir, "points.jsonl"), "a", buffering=1)

    def jot(kind, payload):
        payload = dict(payload)
        payload["kind"] = kind
        payload["t_wall"] = time.time()
        jl.write(json.dumps(payload) + "\n")
        jl.flush()
        os.fsync(jl.fileno())

    jot("run_start", dict(arm=arm, home_ee=home_ee.tolist(),
                          step_m=a.step, max_steps=a.max_steps,
                          floor_m=a.floor, n_directions=len(dirs)))
    for nm, v in dirs:
        reached, why, last_q = 0.0, "not attempted", qh.copy()
        print("\n  direction %s" % nm)
        for k in range(1, a.max_steps + 1):
            dist = a.step * k
            tgt = home_ee + v * dist
            # SEED FROM THE PREVIOUS STEP. A 0.10 m move barely changes the
            # joint vector, so the last accepted solution is already in the
            # right basin. Running 60 fresh random seeds at every step was
            # most of the wall clock and bought nothing after the first --
            # the first step of a direction still gets the full search,
            # because that one is a genuine jump from home.
            ns = 60 if k == 1 else 8
            q, e, wik = ik.solve(tgt, tol=0.006, floor=a.floor, seam=0.25,
                                 n_seeds=ns, extra_seeds=(last_q,))
            if q is None and ns < 60:
                # A THIN SEARCH MAY NOT DECLARE A LIMIT.
                #
                # 8 seeds is plenty to CONTINUE from a neighbouring solution,
                # and nowhere near enough to conclude the arm cannot go
                # further -- that conclusion becomes the recorded extent of
                # the workspace. Reporting "unreachable ... 9 seeded starts"
                # is precisely the failure this whole module was written to
                # avoid: grasp_pipeline.reachable() calling a point outside
                # the arm's 902 mm reach when it had simply not looked.
                # So: fast while it is working, exhaustive before it stops.
                q, e, wik = ik.solve(tgt, tol=0.006, floor=a.floor,
                                     seam=0.25, n_seeds=200,
                                     extra_seeds=(last_q, qh))
            if q is None:
                why = "unreachable at %.3f m: %s" % (dist, wik)
                break
            try:
                meas, info = arm_io.goto(q, "%s %s %.2fm" % (arm, nm, dist),
                                         timeout_s=90.0)
            except MotionRefused as exc:
                why = "stopped at %.3f m: %s" % (dist, exc)
                break
            except ArmVanished as exc:
                print("      ABORT: %s" % exc)
                out["aborted"] = str(exc)
                json.dump(out, open(os.path.join(outdir, "partial.json"), "w"),
                          indent=1)
                raise
            ee = ik._pt(meas)
            # EVERYTHING THIS POSE KNOWS, not just the point.
            # A 3-vector says where the hand was; it does not say which way
            # it pointed, where the elbow was, or which body part was
            # nearest. All of that is free here and impossible to recover
            # later, so it is all written down.
            M = sc.cf[arm](meas)
            T_ee = M[SHP.IDX["end_effector_link"]]
            parts = sc.clearance_parts(arm, meas)
            clr = parts["moving_chain_m"]
            links = {}
            for lname, idx in SHP.IDX.items():
                links[lname] = [float(v) for v in M[idx][:3, 3]]
            rec = dict(dir=nm, step=k, commanded_m=dist,
                       t_wall=time.time(),
                       q_cmd=[float(v) for v in q],
                       ee_pose_4x4=[[float(x) for x in row] for row in T_ee],
                       ee_rotation=[[float(x) for x in r]
                                    for r in T_ee[:3, :3]],
                       link_positions=links,
                       clearance_whole_chain_m=parts["whole_chain_m"],
                       clearance_nearest_part=parts["moving_chain_to"],
                       per_wearer_link_m=parts["per_wearer_link_moving_m"],
                       target=tgt.tolist(), ee=ee.tolist(),
                       ee_err_mm=float(np.linalg.norm(ee - tgt) * 1000),
                       q_meas=meas.tolist(), clearance_m=float(clr),
                       worst_path_clearance_m=info["worst_clearance_m"],
                       path_samples=info["path_samples"],
                       arrive_err_deg=float(np.degrees(
                           info["arrive_err_rad"])), seconds=info["seconds"])
            if cams:
                try:
                    d, c, live = cams[0].grab(18)
                    hdf = cams[1].grab()
                    np.savez_compressed(
                        os.path.join(outdir, "%s_%02d.npz" % (nm, k)),
                        depth=d, color=c, hd=hdf, q=meas, q_cmd=q,
                        ee=ee, ee_pose=T_ee, K=np.array(cams[0].K, float),
                        link_names=np.array(sorted(links)),
                        link_xyz=np.array([links[n2] for n2 in sorted(links)]),
                        clearance=float(clr), t_wall=time.time())
                    rec["depth_valid"] = live["valid_frac"]
                    rec["depth_unique"] = live["unique_content"]
                except Exception as exc:                      # noqa: BLE001
                    rec["camera_error"] = str(exc)
            out["points"].append(rec)
            jot("point", rec)
            reached, last_q = dist, meas
            print("      %.2f m  EE %s  err %.1f mm  clear %.3f  path %.3f"
                  % (dist, np.round(ee, 3).tolist(), rec["ee_err_mm"],
                     clr, info["worst_clearance_m"]))
        else:
            why = "hit the %d-step limit still going" % a.max_steps
        print("      -> reached %.3f m, stopped because: %s"
              % (reached, why[:110]))
        out["directions"][nm] = dict(reached_m=reached, stopped_because=why)
        jot("direction", dict(dir=nm, reached_m=reached, stopped_because=why))
        try:
            arm_io.goto(qh, "return home", timeout_s=120.0)
        except (MotionRefused, ArmVanished) as exc:
            print("      could not return home: %s" % exc)
            out["directions"][nm]["return_failed"] = str(exc)
            raise
    jot("run_end", dict(arm=arm, directions=len(out["directions"]),
                        points=len(out["points"])))
    jl.close()
    arm_io.close()
    return out


if __name__ == "__main__":
    sys.exit(main())
