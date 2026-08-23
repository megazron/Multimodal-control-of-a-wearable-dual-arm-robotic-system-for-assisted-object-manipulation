#!/usr/bin/env python3
"""LOOK AT THE WORKSPACE FIRST, IN ORDER, AND BUILD ONE MAP OF WHAT IS THERE.

    python3 scripts/calibrate_environment.py --self-test        # no stack
    python3 scripts/calibrate_environment.py --arm left
    python3 scripts/calibrate_environment.py --arm left --order typewriter

WHAT IT IS FOR
--------------
Everything downstream of this used to run on DECLARED COORDINATES: the table
is at 1.250 and the cubes are at (+/-0.42, 0.45) because a file says so. That
survives exactly one cell. This stage replaces the file with a measurement:
sweep the arm over the workspace in straight rows, deproject the depth at
every cell into the ROBOT frame, fuse the views, and write one map --

    a support surface, MEASURED
    the objects on it, with their widths and whether they can be grasped
    the occupied points, which is what the planner has to avoid

-- and then everything else reads THAT.

IT IS NOT TASK-SPECIFIC AND MUST NOT BECOME SO. There is no T1 in here, no
cube count, no expected colour. It answers "what is on the table", whatever is
on the table, which is the only version of the question that is still true
when somebody puts a different object down.

WHY THE SWEEP IS ORDERED
------------------------
`srl_perception.calibration_sweep` walks straight serpentine rows: every step
is one cell in one axis, so an operator watching can predict where the arm
goes next and a reviewer can say where it went. Measured against the obvious
alternative on the shipped grid: 1.140 m of travel against 1.702 m, and no
long returns. The previous behaviour -- what prompted this -- was T0's three
direction probes 640 mm apart in height, which is a Fitts instrument and looks
like the arm wandering.

WHAT IS MEASURED HERE AND WHAT IS NOT
-------------------------------------
NO CAMERA HAS EVER BEEN ATTACHED TO THIS HOST. Run against the mock RGB-D
node, everything in the chain is exercised for real -- the topics, the 16UC1
millimetre encoding, the TF lookup of the optical frame, the deprojection, the
plane fit, the clustering, the fusion, the refusals -- and the DEPTH ITSELF is
rendered from a scene this repository also wrote. So this measures the
pipeline, not the room. `Map.provenance` records which sources actually
contributed, and `--require-real-camera` refuses to run against the mock at
all, for the day there is one.

The wrist camera's pose comes from FORWARD KINEMATICS, which is exact given
the URDF -- and `camera_link` has never been checked against the physical
module. A 10 mm error there lands directly in every point in the map. It is
the cheapest high-value calibration left and it is not done.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _p in (HERE, os.path.join(ROOT, "config"),
           os.path.join(ROOT, "src/srl_perception"),
           os.path.join(ROOT, "src/srl_experiments"),
           os.path.join(ROOT, "src/srl_teleop")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from srl_experiments import narration as N                   # noqa: E402
from srl_perception import calibration_sweep as CSW          # noqa: E402
from srl_perception import world_model as WM                 # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/world_map.json")

# The volume to sweep, per arm, in the ROBOT frame. NOT a task's coordinates:
# the outer bound is the arm's own measured reach and the inner bound is the
# wearer side, so it is the space this arm can look at rather than the space
# some task happens to use. Overridable from the command line, because the
# next cell is a different size.
BOUNDS = {
    "left":  dict(x=(0.30, 0.58), y=(0.30, 0.52)),
    "right": dict(x=(-0.58, -0.30), y=(0.30, 0.52)),
}
# Where the surface is EXPECTED, only so the sweep has a height to hover at.
# It is a starting guess and the map REPLACES it -- the whole point is that
# the measured value is what everything downstream uses.
GUESS_SURFACE_Z = 1.25


def _say(node, phase, **kw):
    """One sentence, to stdout for the log and to /robot_say for the window."""
    txt = N.text(phase, **kw)
    print(N.line(phase, **kw), flush=True)
    if node is not None:
        node.say(phase, txt, N.speech(phase, **kw))
    return txt


class Calibrator:
    """The live half: move, look, deproject. Everything else is pure."""

    def __init__(self, arm, node):
        self.arm = arm
        self.n = node

    def look_from(self, xyz, quat, settle_s=1.2):
        """Command the arm to a viewing pose and wait for it to actually be there.

        Returns True when the arm ARRIVED. A capture taken while the arm is
        still moving is a cloud smeared across two poses, and nothing
        downstream can tell -- `check_frame_still` exists in `vision_grasp`
        for exactly this and the same discipline applies here.
        """
        j = self.n.solve(self.arm, list(xyz), quat)
        if j is None:
            return False
        self.n.send(self.arm, j, 2.0)
        t0 = time.time()
        while time.time() - t0 < 6.0:
            self.n.spin(0.1)
            if self.n.at(self.arm, j, tol=0.02):
                self.n.spin(settle_s)
                return True
        return False

    def capture(self, source, finder="segment"):
        """One View -- points, and the INSTANCES cut out of this frame.

        REFUSES A STALE FRAME. The camera keeps publishing whether or not the
        arm moved, so a capture that accepts whatever is in the buffer will
        happily record the previous cell's view at this cell's pose and fuse a
        map out of six copies of one picture.

        WHY THE OBJECTS ARE FOUND HERE AND NOT ON THE FUSED CLOUD
        --------------------------------------------------------
        Because a mask belongs to the frame it was cut from. Object identity
        comes from appearance -- edges, colour -- which is available in the
        picture and gone once nine views are stacked into a pile of points.
        What survives stacking is distance, and distance is precisely what
        cannot tell two 40 mm cubes on a 60 mm pitch apart: the gap between
        their faces is 20 mm, which is the cluster distance, so the old finder
        reported ONE object 140 mm wide and refused it as too big for the
        jaws. Segmenting per frame does not have that failure available to it.
        """
        self.last_frame = {}
        d, bgr, K, stamp = self.n.fresh_frame(self.arm, max_age_s=1.5)
        if d is None:
            print("      no fresh depth+colour pair -- skipping this cell",
                  flush=True)
            return None
        # THE POSE THIS FRAME WAS TAKEN FROM, not the pose the arm is at now.
        # See `env_probe.frame_pose`: asking TF instead replicated every cube
        # once per view, each copy displaced by about one cell of the sweep.
        pose, src = self.n.frame_pose(self.arm, stamp)
        if pose is None:
            print("      no usable camera pose for this frame: %s" % src,
                  flush=True)
            return None
        from pick_from_table import cloud_in_robot_frame
        from srl_perception import table_scene as _TS
        try:
            pts = cloud_in_robot_frame(d, K, pose, stride=3)
        except _TS.SceneRefusal as e:
            # A CELL THE CAMERA CANNOT SEE FROM IS A SKIP, NOT A CRASH.
            #
            # Measured on the first live run: from cell 3 of 6 the wrist
            # camera had no return between 0.08 and 1.50 m -- it was looking
            # past the edge of the table into empty space -- and
            # `cloud_in_robot_frame` refuses, correctly. A sweep that dies
            # there loses the five cells it had already paid for.
            print("      nothing in range from this cell: %s" % e, flush=True)
            return None
        if len(pts) < WM.MIN_POINTS_PER_VIEW:
            print("      only %d points -- skipping this cell" % len(pts),
                  flush=True)
            return None

        # THE RAW FRAME, so the segmenter can be tuned WITHOUT moving an arm.
        # Every parameter sweep that needs a robot costs eight minutes and a
        # stack; the same sweep on a saved frame costs a second.
        self.last_frame = dict(bgr=bgr, depth=d, K=np.asarray(K, float),
                               pose=np.asarray(pose, float), stamp=stamp)
        objs = None
        if finder == "segment":
            from srl_perception import segment_lift as SL
            try:
                plane = _TS.fit_support_plane(pts)
            except _TS.SceneRefusal as e:
                print("      no support plane in this view (%s) -- skipping"
                      % e, flush=True)
                return None
            try:
                objs = SL.objects_in_view(bgr, d, K, pose, plane, stride=1,
                                          source=source)
            except SL.SegRefusal as e:
                # NAMED, NOT SILENTLY CLUSTERED. A map whose objects came from
                # two different finders has one confidence written over both.
                print("      the segmenter refused: %s" % e, flush=True)
                return None
            print("      %d region(s) on the surface" % len(objs), flush=True)
        return WM.View(pts, source, note="stamp %.3f, pose from %s, finder %s"
                       % (stamp, src, finder), objects=objs)


def run(node, arm, bounds, surface_z, step_m, order, hover_m,
        range_m=0.35, max_cells=None,
        view_out_m=0.35, view_in_m=0.12, view_up_m=0.30, dump_views=None,
        dump_frames=None, finder="segment"):
    """The whole stage. Returns (Map, report) or raises MapRefusal."""
    plan = CSW.plan(bounds["x"][0], bounds["x"][1],
                    bounds["y"][0], bounds["y"][1], surface_z,
                    step_m=step_m, hover_m=hover_m, order=order)
    cells = plan["cells"][:max_cells] if max_cells else plan["cells"]
    _say(node, "CALIBRATING", arm=arm,
         detail="%d cells in %d straight rows, %s order, %.2f m of travel"
                % (len(cells), plan["n_rows"], plan["order"],
                   plan["travel_m"]))

    cal = Calibrator(arm, node)
    views, missed = [], []
    for k, c in enumerate(cells, 1):
        # ===============================================================
        # THE VIEWING POSE LOOKS DOWN AT THE CELL FROM ABOVE AND OUTBOARD
        # ===============================================================
        # THE WRIST CAMERA LOOKS ALONG THE TOOL AXIS, and this rig's grasp
        # anchor points that axis **30.76 deg ABOVE horizontal** -- the hand
        # grasps from below. So a camera carrying the anchor can never see the
        # TOP of a horizontal surface: every ray in its field of view goes
        # upward. From beneath a table it sees the underside; from above it
        # sees the room.
        #
        # BOTH OF MY FIRST TWO ATTEMPTS GOT THIS WRONG, and the second one is
        # the instructive failure. Hovering above the cell with the anchor
        # aimed the camera at the ceiling: 10 of 12 cells returned no depth at
        # all. Standing back along the axis put the camera under the table and
        # returned a map with a support surface at **z = 1.2154 m**, flat to
        # 0.18 deg -- and the table is a 35 mm slab centred at 1.2325, so its
        # UNDERSIDE is at 1.2150. The robot had measured the underside of the
        # table to 0.4 mm and reported it, correctly, as the biggest flat
        # thing it could see. A confident, precise, useless answer.
        #
        # `solve_observe_pose` already knew the shape of the fix: the pose it
        # solved has its tool axis at -13.75 deg, DOWNWARD, from a viewpoint
        # above and outboard of the work. So each cell gets its own aimed
        # pose -- camera above the surface and outboard of the cell, tool axis
        # pointing AT the cell -- which is also what makes this a scan of the
        # workspace rather than a tour of viewpoints.
        out = math.copysign(view_out_m, c[0])
        cam = [round(c[0] + out, 5),
               round(c[1] - view_in_m, 5),
               round(surface_z + view_up_m, 5)]
        target = [c[0], c[1], surface_z]
        quat = node.look_at_quat(cam, target)
        at = cam
        _say(node, "REACHING", arm=arm, cell=k, of=len(cells), at=at)
        if not cal.look_from(at, quat):
            missed.append(dict(cell=k, at=at, why="pose not reached"))
            continue
        v = cal.capture("%s_cell_%02d" % (arm, k), finder=finder)
        if v is None:
            missed.append(dict(cell=k, at=at,
                               why="no usable frame from this pose"))
            continue
        views.append(v)
        print("      %d points" % v.n, flush=True)
        if dump_frames and v is not None:
            os.makedirs(dump_frames, exist_ok=True)
            np.savez(os.path.join(dump_frames, "%s.npz" % v.source),
                     **cal.last_frame)
        if dump_views:
            # ONE VIEW AT A TIME IS THE INSTRUMENT CHECK. A fused map that
            # disagrees with the truth says nothing about WHERE the error is:
            # a bad deprojection, a bad camera pose and a bad fusion all
            # produce one wrong map. A single view carries its own pose, so
            # its plane height and its object positions can be scored on
            # their own, and only then is the fusion worth suspecting.
            os.makedirs(dump_views, exist_ok=True)
            np.save(os.path.join(dump_views, "%s.npy" % v.source), v.points)

    if missed:
        print("   %d cell(s) contributed nothing:" % len(missed), flush=True)
        for x in missed:
            print("      cell %d at %s -- %s"
                  % (x["cell"], [round(v, 3) for v in x["at"]], x["why"]),
                  flush=True)
    _say(node, "MAPPING", arm=arm,
         detail="fusing %d view(s) of %d cell(s)" % (len(views), len(cells)))
    m = WM.build(views)
    report = dict(arm=arm, order=plan["order"], cells=len(cells),
                  rows=plan["n_rows"], cols=plan["n_cols"],
                  travel_m=plan["travel_m"], step_m=step_m, range_m=range_m,
                  views_used=len(views), cells_missed=missed,
                  finder=finder, guess_surface_z=surface_z)
    return m, report


# ------------------------------------------------------------- the self-test

def self_test(verbose=True):
    """No stack. Checks the parts that decide what the robot DOES."""
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        ok = ok and bool(cond)
        if verbose:
            print("  %-4s %s%s" % ("PASS" if cond else "FAIL", name,
                                   ("  -- " + detail) if detail else ""))

    check("the sweep planner agrees with itself", CSW.self_test(verbose=False))
    check("the world model agrees with itself", WM.self_test(verbose=False))
    check("the narration agrees with itself", N.self_test(verbose=False))

    p = CSW.plan(*BOUNDS["left"]["x"], *BOUNDS["left"]["y"],
                 GUESS_SURFACE_Z, step_m=0.07)
    check("the shipped left bounds give a real grid",
          p["n_rows"] >= 3 and p["n_cols"] >= 3,
          "%d x %d cells" % (p["n_cols"], p["n_rows"]))
    longest = max(math.dist(a[:2], b[:2])
                  for a, b in zip(p["cells"], p["cells"][1:]))
    step = max(p["step_x_m"], p["step_y_m"])
    # 0.1 mm, not 1e-9. The cell coordinates are rounded to five decimals, so
    # the real gap between two rounded rows can exceed the reported step by
    # 1e-5 m -- measured, on the shipped grid. A tolerance tighter than the
    # grid's own rounding tests the rounding and nothing else, while 0.1 mm is
    # still four orders of magnitude below the 0.24 m return this is looking
    # for.
    check("no step of the shipped sweep is a long transit",
          longest <= step + 1e-4, "longest %.5f m, step %.5f m"
          % (longest, step))

    # THE TWO ARMS SWEEP THEIR OWN SIDES and neither crosses the centreline,
    # which is measured elsewhere as 0 of 10 IK solutions at every cross-side
    # point. A sweep that wandered across it would refuse every cell there and
    # report a map with a hole in it.
    lx = BOUNDS["left"]["x"]
    rx = BOUNDS["right"]["x"]
    check("neither arm's sweep crosses the centreline",
          min(lx) > 0 and max(rx) < 0, "left %s right %s" % (lx, rx))

    # AND IT IS NOT A TASK. The stage must not IMPORT a task module: the
    # whole point is that it answers "what is on the table" for whatever is on
    # the table, and a task import is how that quietly becomes "where T1's
    # cubes are declared to be".
    #
    # CHECKED BY IMPORT, NOT BY GREPPING FOR WORDS. The first version searched
    # this file's own source for "T1_CUBES" and friends -- and found them, in
    # the list of words it was searching for. A check that fails on itself is
    # not a check; it is a mirror.
    import ast as _ast
    tree = _ast.parse(open(os.path.abspath(__file__)).read())
    imported = set()
    for nd in _ast.walk(tree):
        if isinstance(nd, _ast.Import):
            imported.update(a.name.split(".")[0] for a in nd.names)
        elif isinstance(nd, _ast.ImportFrom) and nd.module:
            imported.add(nd.module.split(".")[0])
            imported.update(a.name for a in nd.names)
    task_modules = {"t1_task", "task0", "task3", "clip_tasks",
                    "msc_clip_tasks", "tasks", "run_abc"}
    leaked = sorted(imported & task_modules)
    check("the stage imports no task module", not leaked, str(leaked))

    if verbose:
        print("calibrate_environment self-test %s"
              % ("PASSED" if ok else "FAILED"))
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default="left", choices=("left", "right"))
    ap.add_argument("--step-m", type=float, default=0.07)
    ap.add_argument("--hover-m", type=float, default=CSW.DEFAULT_HOVER_M)
    ap.add_argument("--surface-z", type=float, default=GUESS_SURFACE_Z,
                    help="where the surface is EXPECTED, only to hover at. "
                         "The map replaces it with a measurement.")
    ap.add_argument("--order", default="serpentine",
                    choices=("serpentine", "typewriter"))
    ap.add_argument("--range-m", type=float, default=0.35,
                    help="how far BACK along the tool axis the camera stands "
                         "from the cell it is looking at. The wrist camera "
                         "looks along that axis and this rig cannot point it "
                         "down, so a viewing pose is a standoff, not a hover.")
    ap.add_argument("--max-cells", type=int, default=None)
    ap.add_argument("--out", default=OUT)
    # THE VIEWING GEOMETRY, MEASURED RATHER THAN GUESSED.
    #
    # The first version stood the camera 400 mm outboard and 200 mm above the
    # surface: a 25 degree grazing angle, at which the table is a thin wedge
    # and 72% of the picture is background. `scripts/measure_view_geometry.py`
    # walked seven elevations on the live stack:
    #
    #   elev   cells reached   object pixels
    #   25.3       9 / 12          6.9 %
    #   39.0       8 / 12          8.2 %
    #   47.9       7 / 12          9.3 %
    #   56.1       6 / 12          9.4 %
    #   64.4       3 / 12         11.8 %
    #   72.1       0 / 12            --
    #   78.5       0 / 12            --
    #
    # Steeper sees more of the object and less of the room, and the arm can
    # reach less of it -- above 72 degrees nothing solves at all, which is the
    # same wall `search_centre_on_surface` hit from the other side (top-down
    # on a surface: 0 of 840 cells). The default is the knee.
    # 39 degrees: 8 of 12 cells against 7 at 47.9 and 6 at 56.1, for 8.2%
    # object pixels against 9.3 and 9.4. COVERAGE WINS HERE -- a cell the arm
    # cannot reach contributes no view at all, and at 47.9 the whole far row
    # went unreachable and the map lost the objects in it.
    ap.add_argument("--view-out-m", type=float, default=0.35)
    ap.add_argument("--view-in-m", type=float, default=0.12)
    ap.add_argument("--view-up-m", type=float, default=0.30)
    ap.add_argument("--finder", default="segment",
                    choices=("segment", "cluster"),
                    help="segment: instances cut from each PICTURE by FastSAM "
                         "and lifted through the depth, which is what the "
                         "grasping literature does. cluster: the old path, "
                         "kept as the control and for a host with no model")
    ap.add_argument("--dump-frames", default=None,
                    help="write each cell's raw RGB, depth, K and camera pose "
                         "here, so the segmenter can be tuned without a stack")
    ap.add_argument("--dump-views", default=None,
                    help="write each view's deprojected cloud here, so a "
                         "single view can be scored before the fusion is "
                         "blamed for the map")
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--require-real-camera", action="store_true",
                    help="refuse to run against mock_rgbd_camera")
    a = ap.parse_args()
    if a.self_test:
        return 0 if self_test() else 1

    from env_probe import ProbeNode          # local import: needs rclpy
    import rclpy
    rclpy.init()
    node = ProbeNode()
    try:
        if not node.wait_ready(a.arm, 120.0):
            print("REFUSING after 120 s. Still missing: %s"
                  % ", ".join(node.missing(a.arm)))
            return 2
        if a.require_real_camera and node.camera_is_mock():
            print("REFUSING: --require-real-camera and the only publisher on "
                  "the depth topic is mock_rgbd_camera.")
            return 3
        _say(node, "STARTING", arm=a.arm,
             detail="calibrating the environment before anything is planned")
        m, report = run(node, a.arm, BOUNDS[a.arm], a.surface_z,
                        a.step_m, a.order, a.hover_m, a.range_m, a.max_cells,
                        dump_views=a.dump_views, dump_frames=a.dump_frames,
                        finder=a.finder, view_out_m=a.view_out_m,
                        view_in_m=a.view_in_m, view_up_m=a.view_up_m)
        for ln in m.describe():
            print("   %s" % ln, flush=True)
        # WHAT SURFACES ARE ACTUALLY THERE, not just the one that won the fit.
        # The plane fit returns the biggest and says nothing about whether it
        # was the only candidate; the first live run measured a support
        # surface 36.6 mm below the table the mock renders and the answer
        # alone could not say why.
        hs = m.surfaces()
        if hs:
            print("   heights holding 5%% or more of the points:", flush=True)
            for b in hs:
                print("      z %.3f m   %7d points  %5.1f%%"
                      % (b["z_m"], b["n"], b["frac"] * 100), flush=True)
        _say(node, "DONE", arm=a.arm,
             detail="surface measured at %.4f m, %d object(s) found"
                    % (m.surface_z, len(m.objects)))
        os.makedirs(os.path.dirname(a.out), exist_ok=True)
        doc = m.as_dict()
        doc["sweep"] = report
        doc["camera"] = node.camera_provenance(a.arm)
        with open(a.out, "w") as f:
            json.dump(doc, f, indent=2, sort_keys=True)
        # THE OCCUPANCY, BESIDE THE MAP. `joint_planner.VoxelWorld` needs
        # POINTS, and 110k of them do not belong in a json anybody reads. This
        # file is what makes the planner able to know the table is there --
        # `world_from_map` has had no caller because nothing wrote this.
        occ = m.occupancy()
        npy = os.path.splitext(a.out)[0] + "_occupancy.npy"
        np.save(npy, occ.astype(np.float32))
        print("-> %s" % a.out)
        print("-> %s   (%d points, for the planner)" % (npy, len(occ)))
        node.publish_map(doc)
        # AND DRAW IT, repeatedly: RViz's MarkerArray display keeps the last
        # message it received, and a single publish sent before the display
        # has subscribed is a message nobody hears.
        for _ in range(12):
            node.publish_map_markers(doc)
            node.spin(0.25)
        return 0
    except WM.MapRefusal as e:
        _say(node, "REFUSED", arm=a.arm, why=str(e))
        return 4
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:                                      # noqa: BLE001
            pass


if __name__ == "__main__":
    sys.exit(main())
