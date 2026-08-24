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
from srl_perception import table_scene as TS                 # noqa: E402
from srl_perception import world_model as WM                 # noqa: E402

OUT = os.path.join(ROOT, "recordings/baselines/world_map.json")
# WHAT THE ARM LEARNED IT CAN REACH, written by the sweep and read by the
# next one. Not a separate 35-minute measurement: the sweep already solves IK
# at every cell and used to throw the answer away.
REACH = os.path.join(ROOT, "recordings/baselines/reachable_cells.json")

# The volume to sweep, per arm, in the ROBOT frame. NOT a task's coordinates:
# the outer bound is the arm's own measured reach and the inner bound is the
# wearer side, so it is the space this arm can look at rather than the space
# some task happens to use. Overridable from the command line, because the
# next cell is a different size.
# MEASURED, 2026-08-23, by `scripts/measure_reachable_volume.py`, which
# solves IK at the shipped attitude over a deliberately over-wide grid:
#
#   reachable at SOME facing+layer   x 0.10 .. 0.75,  y 0.15 .. 0.65
#                                    146 of 154 probed cells, both arms
#   reachable at EVERY facing+layer  4 cells, in the inboard-near corner
#
# The box swept before this was x 0.30..0.58, y 0.30..0.52 -- WRITTEN DOWN,
# and a small fraction of what the arms can actually observe. That is why 74
# of 144 planned cells came back "outside the envelope": not because the arms
# are small, but because the guessed box sat badly inside a much larger one.
#
# THE INBOARD BOUND IS NOT FROM THAT MEASUREMENT AND MUST NOT BE.
# IK reports x = 0.10 as reachable and IK IS NOT THE WEARER CHECK -- CLAUDE.md
# hard constraint 11: the SRDF excludes the 44 proximal pairs a shoulder mount
# actually threatens, so a pose MoveIt calls valid can have the tube inside
# the person. The innermost columns this project has measured GEOMETRICALLY
# are 0.325 (left) and 0.450 (right), and those are the inboard bounds here.
# The asymmetry is real and documented; it is not a typo.
#
# So: widened outboard and forward, where the wearer is not, and left alone
# inboard, where they are.
BOUNDS = {
    "left":  dict(x=(0.325, 0.750), y=(0.15, 0.65)),
    "right": dict(x=(-0.750, -0.450), y=(0.15, 0.65)),
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
        self.viewpoint = None

    def look_from(self, xyz, quat, settle_s=1.2, solution=None):
        """Command the arm to a viewing pose and wait for it to actually be there.

        Returns True when the arm ARRIVED. A capture taken while the arm is
        still moving is a cloud smeared across two poses, and nothing
        downstream can tell -- `check_frame_still` exists in `vision_grasp`
        for exactly this and the same discipline applies here.
        """
        j = solution if solution is not None else self.n.solve(
            self.arm, list(xyz), quat)
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

    def aim(self, arm, cam, quat, facing_deg, layer, elev_deg):
        """Remember where this capture is being taken from, so it can be
        repeated without re-deriving it from the sweep's arithmetic."""
        self.viewpoint = dict(
            arm=arm, cam=[float(v) for v in cam],
            quat=[float(quat.x), float(quat.y), float(quat.z), float(quat.w)],
            facing_deg=float(facing_deg), layer=int(layer),
            elev_deg=float(elev_deg))

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
                       % (stamp, src, finder), objects=objs,
                       viewpoint=self.viewpoint)


def load_reach(arm, key, path=REACH):
    """The cached reachable cells for THIS arm at THESE settings, or None.

    Refuses a cache whose settings key differs, because reachability is a fact
    about the arm at a geometry and every bound, layer, facing and elevation
    changes it. A stale cache would silently skip cells that became reachable
    when the volume moved.
    """
    if not os.path.exists(path):
        return None
    try:
        with open(path) as f:
            doc = json.load(f)
    except Exception:                                          # noqa: BLE001
        return None
    e = doc.get(arm)
    if not e or e.get("settings_key") != key:
        return None
    return e.get("cells")


def save_reach(arm, key, cells, path=REACH):
    """Record what solved, per facing and layer. Merged, never clobbered."""
    doc = {}
    if os.path.exists(path):
        try:
            with open(path) as f:
                doc = json.load(f)
        except Exception:                                      # noqa: BLE001
            doc = {}
    doc[arm] = dict(settings_key=key, cells=cells,
                    note=("written by the sweep itself: these are the cells "
                          "whose IK solved. Delete this file, or change any "
                          "sweep setting, and every cell is probed again."))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        json.dump(doc, f, indent=2, sort_keys=True)
    return path


# THE ARMS' OWN ENVELOPE, MEASURED (`measure_reachable_volume.py`, wide run):
# the camera can be placed anywhere in x 0.05..0.85 (magnitude), y -0.25..0.65,
# at some facing. The forward and outboard limits are real -- the probe asked
# out to y 0.95 and x 1.00 and the arm stopped short. The REARWARD limit is
# not established: the probe only asked back to y = -0.25 and got it.
#
# Reaching behind the wearer is IK-reachable and NOT thereby safe: hard
# constraint 11, `avoid_collisions` is not the wearer check. The auto bounds
# below never go inboard of the columns measured geometrically.
ENVELOPE = dict(x_max=0.85, y_min=-0.25, y_max=0.65)
# The innermost columns measured GEOMETRICALLY against the wearer, per arm.
INBOARD_LIMIT = {"left": 0.325, "right": -0.450}


def find_table(node, arms, elev_deg, surface_guess, n=3, finder="cluster",
               probe_heights_m=(0.35, 0.60)):
    """WHERE IS THE TABLE? Coarse probe first, then sweep what was found.

    WHY THIS EXISTS. `BOUNDS` was a constant -- a box in FRONT of the wearer,
    x 0.325..0.750, y 0.15..0.65 -- so the calibration adapted to any table
    HEIGHT, any table SIZE and any arrangement of objects, and to exactly one
    table POSITION. Put the table to one side and the sweep would quarter the
    empty air where the table used to be and report a confident map of
    nothing.

    A handful of wide-angle views over the arms' whole measured envelope is
    enough to find a support surface and its extent. That takes a couple of
    minutes; the full sweep then covers the table that is actually there
    instead of the one that was there when the constant was written.

    Returns (surface_z, bounds_by_arm, report). Raises `WM.MapRefusal` if no
    surface is found, rather than falling back to the constant -- a sweep of
    the wrong volume is worse than no sweep, because it produces a map.
    """
    from geometry_msgs.msg import Quaternion   # noqa: F401
    views, probed = [], []
    for arm in arms:
        cal = Calibrator(arm, node)
        sx = 1.0 if arm == "left" else -1.0
        xs = np.linspace(abs(INBOARD_LIMIT[arm]) + 0.05,
                         ENVELOPE["x_max"] - 0.05, n) * sx
        ys = np.linspace(ENVELOPE["y_min"] + 0.10,
                         ENVELOPE["y_max"] - 0.05, n)
        for f in (-25.0, 0.0, 25.0):
            axis = node.fixed_axis(arm, f, elev_deg)
            quat = node.fixed_quat(arm, f, elev_deg)
            for x in xs:
                for y in ys:
                    cam = None
                    # SEVERAL CAMERA HEIGHTS, because the guess is a guess.
                    # `--surface-z` only says where to STAND; a table 300 mm
                    # lower is still inside the depth range and the field of
                    # view, but only if the camera got high enough to look
                    # down at it. Probing one height makes the guess load
                    # bearing, which is the thing this whole stage is for
                    # removing.
                    for hz in probe_heights_m:
                        d = hz / max(1e-6, -float(axis[2]))
                        c = [round(float(x) - float(axis[0]) * d, 5),
                             round(float(y) - float(axis[1]) * d, 5),
                             round(surface_guess + hz, 5)]
                        if node.solve(arm, c, quat, tries=1) is not None:
                            cam = c
                            break
                    if cam is None:
                        continue
                    cal.aim(arm, cam, quat, f, 0, elev_deg)
                    if not cal.look_from(cam, quat):
                        continue
                    v = cal.capture("find_%s_%02d" % (arm, len(probed)),
                                    finder=finder)
                    probed.append(cam)
                    if v is not None:
                        views.append(v)
                    break                      # one view per facing per column
                if len(views) >= 2 * n:
                    break
    if not views:
        raise WM.MapRefusal(
            "the coarse probe reached %d viewpoint(s) and none returned a "
            "usable frame, so there is no surface to sweep. Nothing was "
            "assumed about where the table is." % len(probed))
    cloud = np.vstack([v.points for v in views])
    try:
        plane = TS.fit_support_plane(cloud, tol_m=0.004)
    except TS.SceneRefusal as e:
        raise WM.MapRefusal(
            "the coarse probe saw %d points and no support surface in them: "
            "%s. Refusing to sweep a volume chosen by a constant."
            % (len(cloud), e))
    h = plane.height(cloud)
    on = cloud[np.abs(h) <= 0.02]
    if len(on) < 500:
        raise WM.MapRefusal(
            "only %d points lie on the fitted surface -- too few to say where "
            "the table is." % len(on))
    c = on.mean(axis=0)
    z = float((plane.offset - c[0] * plane.normal[0]
               - c[1] * plane.normal[1]) / abs(plane.normal[2]))
    pad = 0.06
    lo_x, hi_x = float(on[:, 0].min()) - pad, float(on[:, 0].max()) + pad
    lo_y, hi_y = float(on[:, 1].min()) - pad, float(on[:, 1].max()) + pad
    # CLIPPING TO THE ENVELOPE CAN EMPTY THE INTERVAL, AND I ONLY CHECKED X.
    #
    # Found by moving the table 450 mm forward, past the arm's measured
    # forward limit of y = 0.65. The table's real extent was y 0.819..1.25;
    # clipping gave lo = max(0.819, -0.25) = 0.819 and hi = min(1.25, 0.65) =
    # 0.65 -- an INVERTED range -- and the sweep was cheerfully told to cover
    # "y 0.819..0.650". The x span was guarded and the y span was not, which
    # is the "a check that cannot fail" rule with one axis left out.
    #
    # An empty interval is not a small table. It means the table is outside
    # what this arm can reach, and the only honest output is to say so.
    lo_y_c = max(lo_y, ENVELOPE["y_min"])
    hi_y_c = min(hi_y, ENVELOPE["y_max"])
    if hi_y_c - lo_y_c < 0.05:
        raise WM.MapRefusal(
            "a surface was found at z = %.4f spanning y %.3f..%.3f, and the "
            "arms can only reach y %.2f..%.2f (measured, "
            "`reachable_envelope_wide.json`). The table is %s the arms' "
            "reach -- move it, or move the wearer. Nothing was swept and no "
            "map was written."
            % (z, lo_y, hi_y, ENVELOPE["y_min"], ENVELOPE["y_max"],
               "beyond" if lo_y > ENVELOPE["y_max"] else "behind"))
    lo_y, hi_y = lo_y_c, hi_y_c
    out = {}
    for arm in arms:
        if arm == "left":
            x0 = max(lo_x, INBOARD_LIMIT["left"])
            x1 = min(hi_x, ENVELOPE["x_max"])
        else:
            x0 = max(lo_x, -ENVELOPE["x_max"])
            x1 = min(hi_x, INBOARD_LIMIT["right"])
        if x1 - x0 < 0.05:
            continue                          # no table on this arm's side
        out[arm] = dict(x=(round(x0, 4), round(x1, 4)),
                        y=(round(lo_y, 4), round(hi_y, 4)))
    if not out:
        raise WM.MapRefusal(
            "a surface was found at z = %.4f spanning x %.3f..%.3f, and none "
            "of it is inside either arm's reachable, wearer-safe column "
            "(left from %.3f, right to %.3f). The table is out of reach."
            % (z, lo_x, hi_x, INBOARD_LIMIT["left"], INBOARD_LIMIT["right"]))
    rep = dict(views=len(views), probed=len(probed), points=len(cloud),
               on_surface=len(on), surface_z=round(z, 5),
               tilt_deg=round(plane.tilt_deg, 3),
               extent_x=[round(lo_x, 4), round(hi_x, 4)],
               extent_y=[round(lo_y, 4), round(hi_y, 4)],
               bounds=out)
    return z, out, rep


def sweep_arm(node, arm, bounds, surface_z, step_m, order, layers_m,
              facings_deg, elev_deg, max_cells=None, dump_frames=None,
              finder="segment", require_still=True, progress_cloud=None,
              use_cache=True):
    """One arm, every pass, every layer. Returns (views, report).

    THE WRIST HOLDS ONE ATTITUDE FOR A WHOLE PASS AND ONLY TRANSLATES.

    Within a pass the orientation is computed ONCE, from `fixed_axis`, and
    every cell is reached by moving the wrist to a position -- the camera
    grid is the work grid rigidly translated back along that fixed ray, so
    consecutive cells differ by a pure translation and the gripper never
    reorients. That is the printer-probe model, and it is the difference
    between a scan you can watch and an arm that swings at every step.

    Several passes at different YAWS give the sides. A vertical face is
    invisible to a camera looking straight at the surface in front of it and
    obvious to one looking along it, so "map all the sides" is passes, and
    "all the lengths" is layers.
    """
    key = CSW.settings_key(bounds["x"][0], bounds["x"][1],
                           bounds["y"][0], bounds["y"][1], surface_z, step_m,
                           layers_m, facings_deg, elev_deg)
    cached = load_reach(arm, key) if use_cache else None
    plan = CSW.plan_volume(bounds["x"][0], bounds["x"][1],
                           bounds["y"][0], bounds["y"][1], surface_z,
                           step_m=step_m, layers_m=layers_m,
                           facings_deg=facings_deg, order=order,
                           reachable=cached)
    if cached is not None:
        print("   planning from the MEASURED reachable set: %d cell(s) "
              "dropped as outside this arm's envelope"
              % plan["cells_dropped_as_unreachable"], flush=True)
    _say(node, "CALIBRATING", arm=arm,
         detail="%d cells in %d pass(es) of %d layer(s), %s order, %.2f m of "
                "travel -- the wrist holds one attitude per pass"
                % (plan["total_cells"], len(plan["passes"]),
                   plan["n_layers"], plan["order"], plan["travel_m"]))

    # ================= WHICH CELLS THIS ARM CAN OBSERVE AT ALL ==============
    # SOLVED BEFORE ANYTHING MOVES.
    #
    # Without this the sweep DISCOVERS unreachability by trying to drive
    # there: a seeded IK search with six restarts and a three-second timeout,
    # then a six-second wait for an arrival that never comes. 74 of 144 cells
    # went that way on the first full run, which is most of the twenty minutes
    # it took -- and the report then read "48% coverage", as though the scan
    # had failed at something.
    #
    # It had not. A cell outside the arm's envelope is a fact about the ROBOT,
    # and it belongs in a different number from a cell the arm could reach and
    # did not photograph. Both are reported now.
    cal = Calibrator(arm, node)
    views, missed, still_fail = [], [], 0
    seen = 0
    unreachable = 0
    solved_cells = {}
    for pi, ps in enumerate(plan["passes"], 1):
        facing = ps["facing_deg"]
        # ONE ORIENTATION, COMPUTED ONCE, FOR EVERY CELL OF THIS PASS.
        axis = node.fixed_axis(arm, facing, elev_deg)
        quat = node.fixed_quat(arm, facing, elev_deg)
        _say(node, "CALIBRATING", arm=arm,
             detail="pass %d of %d, wrist fixed at %+.0f deg yaw / %.1f deg "
                    "down, %d cells" % (pi, len(plan["passes"]), facing,
                                        elev_deg, len(ps["cells"])))
        for (cx, cy, cz, row, col, layer) in ps["cells"]:
            seen += 1
            if max_cells and seen > max_cells:
                break
            h = float(cz) - surface_z
            # The camera stands back along its OWN fixed ray so that the ray
            # lands on the work cell. A rigid translation of the work grid,
            # so the sweep is still a straight-row raster.
            d = h / max(1e-6, -float(axis[2]))
            cam = [round(cx - float(axis[0]) * d, 5),
                   round(cy - float(axis[1]) * d, 5),
                   round(surface_z + h, 5)]
            j = node.solve(arm, cam, quat, tries=2)
            if j is not None:
                solved_cells.setdefault("%+.1f|%d" % (facing, layer), []).append(
                    [round(cx, 4), round(cy, 4)])
            if j is None:
                # OUTSIDE THE ENVELOPE. Not a scan failure -- there is no
                # wrist pose at this attitude that puts the camera here.
                unreachable += 1
                missed.append(dict(cell=seen, at=cam, pass_=pi, facing=facing,
                                   layer=layer,
                                   why="outside this arm's envelope at this "
                                       "attitude"))
                continue
            cal.aim(arm, cam, quat, facing, layer, elev_deg)
            _say(node, "REACHING", arm=arm, cell=seen,
                 of=plan["total_cells"], at=cam)
            if not cal.look_from(cam, quat, solution=j):
                missed.append(dict(cell=seen, at=cam, pass_=pi, facing=facing,
                                   layer=layer, why="pose not reached"))
                continue
            if require_still:
                still, worst = node.is_still(arm)
                if not still:
                    # A FRAME TAKEN WHILE THE ARM MOVES IS A CLOUD SMEARED
                    # ACROSS TWO POSES, and every statistic downstream still
                    # comes out looking fine.
                    still_fail += 1
                    missed.append(dict(cell=seen, at=cam, pass_=pi,
                                       facing=facing, layer=layer,
                                       why="arm still moving: worst joint "
                                           "%.4f rad over the window" % worst))
                    continue
            v = cal.capture("%s_p%d_c%02d" % (arm, pi, seen), finder=finder)
            if v is None:
                missed.append(dict(cell=seen, at=cam, pass_=pi, facing=facing,
                                   layer=layer,
                                   why="no usable frame from this pose"))
                continue
            views.append(v)
            print("      %d points" % v.n, flush=True)
            # DRAW WHAT HAS BEEN MEASURED SO FAR. See
            # `env_probe.publish_progress_cloud`: without this the sweep clip
            # is an empty room until the very end, and which part of the space
            # is still blank is the one thing a watcher cannot see.
            if progress_cloud is not None:
                progress_cloud.append(v.points)
                node.publish_progress_cloud(np.vstack(progress_cloud))
            if dump_frames:
                os.makedirs(dump_frames, exist_ok=True)
                np.savez(os.path.join(dump_frames, "%s.npz" % v.source),
                         **cal.last_frame)

    # WRITE WHAT WAS LEARNED, but only from a run that probed everything.
    # A run planned FROM the cache has not tested the cells the cache excluded,
    # so saving its answer would shrink the set a little further every time --
    # a cache that eats itself.
    if cached is None and not max_cells:
        save_reach(arm, key, solved_cells)
        print("   recorded %d reachable cell(s) for next time -> %s"
              % (sum(len(v) for v in solved_cells.values()), REACH), flush=True)

    reachable = plan["total_cells"] - unreachable
    report = dict(arm=arm, order=plan["order"], passes=len(plan["passes"]),
                  cells_reachable=reachable, cells_outside_envelope=unreachable,
                  planned_from_measurement=plan["planned_from_measurement"],
                  cells_dropped_as_unreachable=plan["cells_dropped_as_unreachable"],
                  coverage_of_reachable=round(len(views) / max(1, reachable), 4),
                  facings_deg=plan["facings_deg"], layers_m=plan["layers_m"],
                  cells=plan["total_cells"], rows=plan["n_rows"],
                  cols=plan["n_cols"], travel_m=plan["travel_m"],
                  step_m=step_m, elev_deg=elev_deg,
                  views_used=len(views), cells_missed=missed,
                  stillness_refusals=still_fail,
                  coverage=round(len(views) / max(1, plan["total_cells"]), 4),
                  guess_surface_z=surface_z)
    return views, report


def run(node, arms, bounds_by_arm, surface_z, step_m, order, layers_m,
        facings_deg, elev_deg, max_cells=None, dump_frames=None,
        finder="segment", require_still=True, use_cache=True):
    """EVERY ARM INTO ONE MAP.

    The map is the robot's, not an arm's. Sweeping one arm and calling the
    result the environment leaves the other half of the space unmeasured and
    says nothing about it -- and the two arms cannot cross the centreline, so
    a one-arm map is missing exactly the half its arm can never reach.

    The views carry their arm in their source name, so `Map.provenance` says
    which arm saw what and an object seen by both is seen by both.
    """
    all_views, reports = [], []
    # ONE ACCUMULATING CLOUD ACROSS BOTH ARMS, so the second arm's sweep adds
    # to the first's picture instead of starting from an empty room again.
    progress = []
    for arm in arms:
        views, rep = sweep_arm(node, arm, bounds_by_arm[arm], surface_z,
                               step_m, order, layers_m, facings_deg, elev_deg,
                               max_cells=max_cells, dump_frames=dump_frames,
                               finder=finder, require_still=require_still,
                               progress_cloud=progress, use_cache=use_cache)
        all_views.extend(views)
        reports.append(rep)
        if rep["cells_missed"]:
            print("   %s arm: %d of %d cell(s) contributed nothing"
                  % (arm, len(rep["cells_missed"]), rep["cells"]), flush=True)
            why = {}
            for x in rep["cells_missed"]:
                why[x["why"].split(":")[0]] = why.get(
                    x["why"].split(":")[0], 0) + 1
            for k, n in sorted(why.items(), key=lambda t: -t[1]):
                print("      %3d  %s" % (n, k), flush=True)

    _say(node, "MAPPING", arm="+".join(arms),
         detail="fusing %d view(s) from %d arm(s)"
                % (len(all_views), len(arms)))
    m = WM.build(all_views)
    reach = sum(r["cells_reachable"] for r in reports)
    report = dict(arms=list(arms), per_arm=reports,
                  views_used=len(all_views),
                  cells=sum(r["cells"] for r in reports),
                  cells_reachable=reach,
                  cells_outside_envelope=sum(r["cells_outside_envelope"]
                                             for r in reports),
                  coverage_of_reachable=round(len(all_views) / max(1, reach), 4),
                  coverage=round(len(all_views)
                                 / max(1, sum(r["cells"] for r in reports)), 4),
                  travel_m=round(sum(r["travel_m"] for r in reports), 4),
                  stillness_refusals=sum(r["stillness_refusals"]
                                         for r in reports))
    return m, report


# ------------------------------------------------------------- the self-test

def self_test(verbose=True):
    """No stack. Checks the parts that decide what the robot DOES."""
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        ok = ok and bool(cond)
        if verbose:
            print("  %-62s %s%s" % (name, "PASS" if cond else "FAIL",
                                    "" if cond else "  -- " + str(detail)))

    check("the sweep planner agrees with itself", CSW.self_test(verbose=False))
    check("the world model agrees with itself", WM.self_test(verbose=False))
    check("the narration agrees with itself", N.self_test(verbose=False))

    # ---- the volume, not a plane
    pv = CSW.plan_volume(BOUNDS["left"]["x"][0], BOUNDS["left"]["x"][1],
                         BOUNDS["left"]["y"][0], BOUNDS["left"]["y"][1],
                         GUESS_SURFACE_Z, step_m=0.11)
    check("the sweep is a VOLUME: more than one height",
          pv["n_layers"] > 1, pv["layers_m"])
    check("and more than one wrist attitude, so sides get seen",
          len(pv["facings_deg"]) > 1, pv["facings_deg"])
    zs = sorted({c[2] for c in pv["passes"][0]["cells"]})
    check("every layer really is visited", len(zs) == pv["n_layers"], zs)
    check("every cell of every layer is visited exactly once",
          len({(c[0], c[1], c[2]) for c in pv["passes"][0]["cells"]})
          == pv["cells_per_pass"])

    # ---- straight rows still, inside a layer
    cells = [c for c in pv["passes"][0]["cells"] if c[5] == 0]
    step = max(pv["passes"][0]["cells"][1][0] - pv["passes"][0]["cells"][0][0],
               0.11)
    longest = max(math.dist(a_[:2], b_[:2])
                  for a_, b_ in zip(cells, cells[1:]))
    check("no step inside a layer is a long unexplained transit",
          longest <= step + 1e-6, longest)

    # ---- THE POINT OF ALL THIS: the wrist does not move inside a pass
    class _Fake:
        fixed_axis = None
    import numpy as _np
    axes = []
    for f in pv["facings_deg"]:
        el = math.radians(38.9)
        sx = -1.0
        base = _np.array([sx * math.cos(el) * 0.946,
                          math.cos(el) * 0.324, -math.sin(el)])
        base = base / _np.linalg.norm(base)
        th = math.radians(f)
        c_, s_ = math.cos(th), math.sin(th)
        R = _np.array([[c_, -s_, 0.0], [s_, c_, 0.0], [0.0, 0.0, 1.0]])
        axes.append(R @ base)
    check("each pass has ONE attitude, and the passes differ",
          len({tuple(_np.round(v, 6)) for v in axes}) == len(axes))
    els = [math.degrees(math.asin(-v[2])) for v in axes]
    check("every pass is held at the SAME elevation, only the yaw changes",
          max(els) - min(els) < 1e-6, els)

    # ---- the camera grid is a RIGID TRANSLATION of the work grid
    ax = axes[1]
    d0 = 0.30 / -ax[2]
    d1 = 0.42 / -ax[2]
    offs = set()
    for (cx, cy, cz, _r, _c, _l) in pv["passes"][1]["cells"]:
        h = cz - GUESS_SURFACE_Z
        d = h / -ax[2]
        offs.add((round(cx - (cx - ax[0] * d), 6),
                  round(cy - (cy - ax[1] * d), 6)))
    check("within a layer the camera offset is CONSTANT -- a translation, "
          "not a re-aim", len(offs) == pv["n_layers"], sorted(offs))
    check("the two layers stand back by different amounts, as they must",
          abs(d0 - d1) > 0.05, (d0, d1))

    # ---- both arms, and neither crosses the centreline
    check("both arms are swept by default",
          "both" in open(os.path.abspath(__file__)).read())
    lo = BOUNDS["left"]["x"]
    ro = BOUNDS["right"]["x"]
    check("neither arm's sweep crosses the centreline",
          min(lo) > 0 and max(ro) < 0, (lo, ro))

    # ---- it must not become task-specific
    import ast as _ast
    tree = _ast.parse(open(os.path.abspath(__file__)).read())
    bad = []
    for node in _ast.walk(tree):
        if isinstance(node, _ast.Import):
            bad += [n.name for n in node.names if "task" in n.name
                    or n.name.startswith("t1")]
        elif isinstance(node, _ast.ImportFrom) and node.module:
            if "task" in node.module or node.module.startswith("t1"):
                bad.append(node.module)
    check("the stage imports no task module", not bad, bad)

    if verbose:
        print("calibrate_environment self-test %s"
              % ("PASSED" if ok else "FAILED"))
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default="both",
                    choices=("left", "right", "both"),
                    help="BOTH by default. The map is the robot's, not an "
                         "arm's -- and the two arms cannot cross the "
                         "centreline, so a one-arm map is missing exactly the "
                         "half its arm can never reach.")
    ap.add_argument("--step-m", type=float, default=0.13,
                    help="cell pitch. Coarser than it was, because the swept "
                         "volume is now the MEASURED reachable one rather "
                         "than a small box inside it, and the sweep has to "
                         "stay a thing an operator will actually wait for.")
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
    ap.add_argument("--elev-deg", type=float, default=38.9,
                    help="how far below horizontal the wrist is held. 38.9 is "
                         "the elevation of the geometry measured best.")
    ap.add_argument("--layers-m", type=float, nargs="+",
                    default=list(CSW.DEFAULT_LAYERS_M),
                    help="heights above the surface to sweep. More than one, "
                         "because an object has SIDES and a side is invisible "
                         "from a single elevation.")
    ap.add_argument("--facings-deg", type=float, nargs="+",
                    default=list(CSW.DEFAULT_FACINGS_DEG),
                    help="one fixed wrist yaw per pass. The wrist does NOT "
                         "re-aim inside a pass -- it translates, like a "
                         "printer's probe.")
    ap.add_argument("--auto-bounds", action="store_true",
                    help="FIND THE TABLE FIRST, then sweep what was found, "
                         "instead of sweeping a box written into this file. "
                         "Costs a couple of minutes and is the difference "
                         "between adapting to any table and adapting to any "
                         "table THAT IS WHERE THE OLD ONE WAS.")
    ap.add_argument("--reprobe", action="store_true",
                    help="ignore the recorded reachable set and solve IK at "
                         "every cell again. Use after anything that changes "
                         "the arm's geometry.")
    ap.add_argument("--allow-moving-capture", action="store_true",
                    help="capture without checking the arm is stationary. "
                         "Refused by default: a frame taken mid-motion is a "
                         "cloud smeared across two poses and every statistic "
                         "downstream still looks fine.")
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

    arms = ["left", "right"] if a.arm == "both" else [a.arm]
    from env_probe import ProbeNode          # local import: needs rclpy
    import rclpy
    rclpy.init()
    node = ProbeNode()
    try:
        if not all(node.wait_ready(x, 120.0) for x in arms):
            print("REFUSING after 120 s. Still missing: %s"
                  % "; ".join("%s: %s" % (x, ", ".join(node.missing(x)))
                              for x in arms))
            return 2
        if a.require_real_camera and node.camera_is_mock():
            print("REFUSING: --require-real-camera and the only publisher on "
                  "the depth topic is mock_rgbd_camera.")
            return 3
        _say(node, "STARTING", arm=a.arm,
             detail="calibrating the environment before anything is planned")
        bounds, surf = BOUNDS, a.surface_z
        found = None
        if a.auto_bounds:
            _say(node, "CALIBRATING", arm="+".join(arms),
                 detail="finding the table before deciding where to sweep")
            surf, bounds, found = find_table(node, arms, a.elev_deg,
                                             a.surface_z)
            print("   table found: surface z = %.4f m, tilt %.2f deg, "
                  "spanning x %.3f..%.3f  y %.3f..%.3f  (%d points on it)"
                  % (found["surface_z"], found["tilt_deg"],
                     found["extent_x"][0], found["extent_x"][1],
                     found["extent_y"][0], found["extent_y"][1],
                     found["on_surface"]), flush=True)
            for k, b in bounds.items():
                print("   %-5s arm will sweep x %.3f..%.3f  y %.3f..%.3f"
                      % (k, b["x"][0], b["x"][1], b["y"][0], b["y"][1]),
                      flush=True)
            missing = [x for x in arms if x not in bounds]
            if missing:
                print("   %s arm has no table in its reachable column and is "
                      "SKIPPED -- not swept and not reported as empty"
                      % ", ".join(missing), flush=True)
                arms = [x for x in arms if x in bounds]
        m, report = run(node, arms, bounds, surf, a.step_m, a.order,
                        a.layers_m, a.facings_deg, a.elev_deg,
                        max_cells=a.max_cells, dump_frames=a.dump_frames,
                        finder=a.finder,
                        require_still=not a.allow_moving_capture,
                        use_cache=not a.reprobe)
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
        _say(node, "DONE", arm="+".join(arms),
             detail="surface measured at %.4f m, %d object(s) found, "
                    "%d of %d REACHABLE cells contributed (%.0f%%)"
                    % (m.surface_z, len(m.objects), report["views_used"],
                       report["cells_reachable"],
                       report["coverage_of_reachable"] * 100))
        os.makedirs(os.path.dirname(a.out), exist_ok=True)
        doc = m.as_dict()
        doc["sweep"] = report
        doc["camera"] = {x: node.camera_provenance(x) for x in arms}
        doc["coverage"] = report["coverage"]
        doc["bounds"] = {k: dict(x=list(v["x"]), y=list(v["y"]))
                         for k, v in bounds.items()}
        doc["bounds_source"] = ("FOUND by a coarse probe before the sweep"
                                if found else
                                "the constant in calibrate_environment.py")
        if found:
            doc["table_found"] = found
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
