#!/usr/bin/env python3
"""WHERE THE ROBOT LOOKS FROM, AND IN WHAT ORDER.

    python3 -m srl_perception.calibration_sweep          # the self-test
    python3 -m srl_perception.calibration_sweep --show   # print a sweep

PURE. No ROS, no robot, no camera. The order of a scan is geometry and can be
checked offline; a scan order that can only be inspected by watching a video
is one nobody checks.

WHY THIS EXISTS
---------------
Watching the recordings, the arm crosses the workspace in what looks like
random directions. It is not a controller fault and it is not noise: nothing
in this repository has ever specified an ORDER for looking at the world. The
observe poses that exist are single points (`solve_observe_pose`), and T0's
targets are three DIRECTION PROBES 640 mm apart in height, drawn with jitter
from a seed -- so the arm genuinely does jump up, out and down.

A calibration scan is not that. It is the machine equivalent of a 3-D
printer probing its bed: visit every cell of a grid, in straight rows, in an
order somebody can predict and check, and record what was found at each one.
Predictability is the point -- an operator watching it should be able to say
where the arm goes next, and a reviewer should be able to say where it went.

THE ORDER IS BOUSTROPHEDON, AND THAT IS NOT DECORATION
-----------------------------------------------------
Rows are traversed alternately left-to-right and right-to-left, so the end of
one row is adjacent to the start of the next. The alternative -- every row
left-to-right, "typewriter" order -- flies the arm the full width of the
workspace at the end of every row, which is where most of the motion, most of
the time and every one of the long unexplained-looking transits come from.

Measured by `self_test` on the 5 x 4 grid at 0.06 m spacing, x 0.30-0.54,
y 0.30-0.48 -- computed, not quoted, and the test fails if it stops being
true:

    typewriter    1.702 m of travel, 3 returns of 0.24 m each
    serpentine    1.140 m of travel, 0 returns

The serpentine is 33% less motion for the same coverage, and every step is
one cell. That is what makes it look deliberate.

(The first draft of this docstring said 3.12 m against 1.68 m and 46%. Those
were written before the check was run and were wrong. The numbers above come
out of `travel_m` on the same grid the test builds.)

WHAT A CELL IS
--------------
A cell is a place to LOOK FROM, not a place to touch. Each one contributes a
view; the views are fused into one map by `world_model`. The probe geometry
-- hover, descend, dwell, retract -- is here too, because a scan that touches
a surface is how the surface height gets measured on a rig whose camera
extrinsic is not trusted.

NOTHING HERE ASSUMES A TABLE, A SIZE, OR AN OBJECT. The bounds are arguments.
The whole point is that the real cell has a different table and different
objects from the simulation, so a scan that only works over declared
coordinates is a scan that works exactly once.
"""
from __future__ import annotations

import math

# Defaults chosen so a sweep is a few dozen seconds rather than a few minutes.
# They are ARGUMENTS everywhere; these are only what `--show` prints.
# THE HEIGHTS THE SWEEP VISITS. A 3-D PRINTER HAS ONE PLANE TO PROBE AND THIS
# ROBOT DOES NOT.
#
# The first version swept a single layer at one height, which measures a
# tabletop and nothing above it -- an object standing 200 mm tall is seen only
# from one elevation, so its sides are never in view and its top is a guess.
# Layers give the fusion genuinely different viewpoints of the same object,
# which is what fills in a side.
DEFAULT_LAYERS_M = (0.30, 0.42)

# HOW MANY FIXED ORIENTATIONS THE CAMERA IS CARRIED IN, one pass each.
DEFAULT_FACINGS_DEG = (-25.0, 0.0, 25.0)

DEFAULT_STEP_M = 0.06
DEFAULT_HOVER_M = 0.12          # above the surface, where the camera sees
DEFAULT_TOUCH_M = 0.005         # how far below the estimated surface to probe
DEFAULT_DWELL_S = 0.4


class SweepError(Exception):
    """A refusal. A scan with no cells is not a scan."""


def grid(x0, x1, y0, y1, step_m=DEFAULT_STEP_M):
    """Cell centres covering [x0, x1] x [y0, y1], inclusive of both ends.

    The count is derived from the span and the step and then the step is
    RECOMPUTED to fit exactly, so the last row and column land on the far
    edge instead of stopping short of it. A grid that silently misses the far
    edge is a map with a blind strip down one side.
    """
    if step_m <= 0:
        raise SweepError("step %.4f m is not positive" % step_m)
    lo_x, hi_x = (x0, x1) if x0 <= x1 else (x1, x0)
    lo_y, hi_y = (y0, y1) if y0 <= y1 else (y1, y0)
    span_x, span_y = hi_x - lo_x, hi_y - lo_y
    if span_x < 0 or span_y < 0:
        raise SweepError("empty bounds")
    nx = max(1, int(round(span_x / step_m)) + 1)
    ny = max(1, int(round(span_y / step_m)) + 1)
    xs = [lo_x + (span_x * i / (nx - 1) if nx > 1 else 0.0) for i in range(nx)]
    ys = [lo_y + (span_y * j / (ny - 1) if ny > 1 else 0.0) for j in range(ny)]
    return [[round(x, 5) for x in xs], [round(y, 5) for y in ys]]


def serpentine(xs, ys):
    """(x, y, row, col) in boustrophedon order: straight rows, alternating.

    Row index is the y index, so a "row" is a line of constant y and the arm
    sweeps ACROSS the workspace and then steps once towards or away from the
    wearer. That is the direction with the most room on this rig.
    """
    if not xs or not ys:
        raise SweepError("a sweep with no cells is not a sweep")
    out = []
    for j, y in enumerate(ys):
        order = range(len(xs)) if j % 2 == 0 else range(len(xs) - 1, -1, -1)
        for i in order:
            out.append((xs[i], y, j, i))
    return out


def layered(xs, ys, zs, order="serpentine"):
    """The 3-D sweep: one ordered layer per height, alternating direction.

    WHY LAYERS AND NOT ONE PLANE. A printer probes a bed: one height, and the
    only unknown is z. This arm works in a volume where objects have SIDES,
    and a side is invisible from directly above it. Sweeping the same grid at
    two heights gives the fusion two genuinely different lines of sight to
    every object, which is what puts points on a vertical face.

    The layer order alternates too, so the arm rises at the end of a layer
    and carries straight on rather than flying back to the start -- the same
    reason the rows alternate inside a layer.
    """
    out = []
    for k, z in enumerate(zs):
        cells = serpentine(xs, ys) if order == "serpentine" else typewriter(xs, ys)
        if k % 2:
            cells = list(reversed(cells))
        for (cx, cy, row, col) in cells:
            out.append((cx, cy, float(z), row, col, k))
    return out


def travel_m(cells):
    """Total in-plane distance walked visiting `cells` in the given order.

    The number the boustrophedon claim rests on, so it is computed rather
    than asserted -- see the module docstring's table.
    """
    # THE FULL 3-D DISTANCE when the cells carry a height, because a sweep
    # that changes layer really does travel that way and a figure that
    # ignored it would understate the path it is used to justify.
    n = 3 if (cells and len(cells[0]) >= 6) else 2
    return sum(math.dist(cells[k][:n], cells[k + 1][:n])
               for k in range(len(cells) - 1))


def typewriter(xs, ys):
    """Every row left-to-right. Here to be MEASURED AGAINST, not used.

    A claim that one order is better than another needs the other order to
    exist and be measurable. `self_test` compares them.
    """
    if not xs or not ys:
        raise SweepError("a sweep with no cells is not a sweep")
    return [(x, y, j, i)
            for j, y in enumerate(ys) for i, x in enumerate(xs)]


def probe(cell, surface_z, hover_m=DEFAULT_HOVER_M, touch_m=DEFAULT_TOUCH_M):
    """The three poses that make one cell a measurement, not a fly-past.

        hover   surface_z + hover_m    the camera's working distance
        touch   surface_z - touch_m    pressed slightly INTO the estimate
        hover   surface_z + hover_m    clear again before moving on

    `touch_m` is BELOW the estimate on purpose. The estimate is what is being
    checked, so a probe that stops exactly at it can only ever confirm it; one
    that aims slightly through it either contacts early -- and the contact
    height is the measurement -- or does not, and that is the finding.
    """
    x, y = cell[0], cell[1]
    hi = round(surface_z + hover_m, 5)
    lo = round(surface_z - touch_m, 5)
    return [[x, y, hi], [x, y, lo], [x, y, hi]]


def plan(x0, x1, y0, y1, surface_z, step_m=DEFAULT_STEP_M,
         hover_m=DEFAULT_HOVER_M, touch_m=DEFAULT_TOUCH_M, order="serpentine"):
    """The whole sweep: ordered cells, each with its probe.

    Returns a dict carrying the cells, the waypoints and the numbers that
    describe it, so a caller can log what it is about to do before doing it.
    """
    xs, ys = grid(x0, x1, y0, y1, step_m)
    cells = (serpentine(xs, ys) if order == "serpentine"
             else typewriter(xs, ys) if order == "typewriter"
             else None)
    if cells is None:
        raise SweepError("unknown order %r; expected serpentine or typewriter"
                         % order)
    wp = []
    for c in cells:
        wp.extend(probe(c, surface_z, hover_m, touch_m))
    return dict(order=order, n_cols=len(xs), n_rows=len(ys),
                cells=cells, waypoints=wp,
                step_x_m=round(xs[1] - xs[0], 5) if len(xs) > 1 else 0.0,
                step_y_m=round(ys[1] - ys[0], 5) if len(ys) > 1 else 0.0,
                travel_m=round(travel_m(cells), 4),
                bounds=dict(x=[min(xs), max(xs)], y=[min(ys), max(ys)],
                            surface_z=surface_z))


def settings_key(x0, x1, y0, y1, surface_z, step_m, layers_m, facings_deg,
                 elev_deg):
    """What a cached reachability answer is only valid FOR.

    Reachability is a fact about the arm at a GEOMETRY, and every one of these
    changes it. A cache that did not key on them would quietly skip cells that
    became reachable when the volume moved -- the "results depend on run
    order" row of the instrument table, with a file standing in for the order.
    """
    return "|".join(str(v) for v in (
        round(x0, 4), round(x1, 4), round(y0, 4), round(y1, 4),
        round(surface_z, 4), round(step_m, 4),
        [round(float(h), 4) for h in layers_m],
        [round(float(f), 4) for f in facings_deg], round(float(elev_deg), 4)))


def plan_volume(x0, x1, y0, y1, surface_z, step_m=DEFAULT_STEP_M,
                layers_m=DEFAULT_LAYERS_M, facings_deg=DEFAULT_FACINGS_DEG,
                order="serpentine", reachable=None, footprint=None,
                footprint_cell_m=0.05, footprint_pad=1):
    """THE SWEEP AS A VOLUME, IN PASSES OF ONE FIXED WRIST ORIENTATION EACH.

    THIS IS THE PRINTER-PROBE MODEL AND IT IS THE WHOLE POINT.

    The first version aimed the wrist AT each cell, so the orientation changed
    at every step of the grid and the arm reoriented between neighbouring
    cells 90 mm apart. Watching it, the gripper swings continuously and never
    holds still; every view is taken from a different attitude, so no two
    views are comparable and a residual wrist error is a different error in
    every frame.

    A printer's probe does not re-aim. It holds ONE attitude and TRANSLATES.
    So does this now: a pass fixes the wrist at one facing and walks the whole
    volume with it, and the only thing that changes between cells is position.
    Several passes at different facings give the sides.

    `facings_deg` is a YAW offset applied to a single base attitude. Positive
    turns the camera outboard, negative inboard, so a three-facing sweep sees
    each object's near side, its outboard side and its inboard side without
    the wrist ever moving inside a pass.

    Returns a list of passes; each pass carries its facing and its cells.
    """
    xs, ys = grid(x0, x1, y0, y1, step_m)
    zs = [round(surface_z + float(h), 5) for h in layers_m]
    passes = []
    dropped = 0
    off_table = 0

    def _on_table(cx, cy):
        """Is this cell over the surface that was actually FOUND?

        THE BOUNDS ARE A RECTANGLE AND A TABLE NEED NOT BE. `find_table` takes
        the axis-aligned box of the surface points, so an L-shaped table has
        cells planned over its missing corner and a round one over all four.
        Measured on an L: 90% of the swept area was table. A round table
        inscribed in its box is 79% by arithmetic.

        Those cells are not harmful -- a viewpoint aimed at nothing refuses by
        name -- but they are arm time spent on somewhere there is no table,
        and the sweep is the slow part of the whole system.

        The footprint is the set of coarse cells that carried surface points,
        dilated by `footprint_pad` cells so an edge is not lost to the grid.
        """
        if footprint is None:
            return True
        i = int(math.floor(cx / footprint_cell_m))
        j = int(math.floor(cy / footprint_cell_m))
        for di in range(-footprint_pad, footprint_pad + 1):
            for dj in range(-footprint_pad, footprint_pad + 1):
                if (i + di, j + dj) in footprint:
                    return True
        return False
    for f in facings_deg:
        cells = layered(xs, ys, zs, order=order)
        if reachable is not None:
            # PLAN ONLY WHAT THE ARM CAN REACH.
            #
            # The swept box is a rectangle and the reachable set is not, so a
            # large minority of cells inside the box are outside the arm. They
            # cost little -- IK is solved before anything moves -- but they
            # make "coverage of planned" a number about the shape of my
            # rectangle rather than about the robot, and they put the arm
            # through a pointless IK search at every one.
            #
            # `reachable` maps "facing|layer_index" to the (x, y) cells that
            # solved last time at exactly these settings. Absent, every cell
            # is planned and reachability is discovered as before.
            keep = []
            for c in cells:
                key = "%+.1f|%d" % (float(f), c[5])
                ok = reachable.get(key)
                if ok is None or [round(c[0], 4), round(c[1], 4)] in ok:
                    keep.append(c)
            dropped += len(cells) - len(keep)
            cells = keep
        if footprint is not None:
            keep = [c for c in cells if _on_table(c[0], c[1])]
            off_table += len(cells) - len(keep)
            cells = keep
        passes.append(dict(facing_deg=float(f), cells=cells,
                           travel_m=round(travel_m(cells), 4)))
    return dict(order=order, n_cols=len(xs), n_rows=len(ys),
                n_layers=len(zs), layers_m=[float(h) for h in layers_m],
                facings_deg=[float(f) for f in facings_deg],
                passes=passes, planned_from_measurement=reachable is not None,
                cells_dropped_as_unreachable=dropped,
                cells_dropped_off_table=off_table,
                planned_from_footprint=footprint is not None,
                cells_per_pass=len(passes[0]["cells"]) if passes else 0,
                total_cells=sum(len(p["cells"]) for p in passes),
                travel_m=round(sum(p["travel_m"] for p in passes), 4),
                bounds=dict(x=[min(xs), max(xs)], y=[min(ys), max(ys)],
                            z=[min(zs), max(zs)], surface_z=surface_z))


# ------------------------------------------------------------- the self-test

def self_test(verbose=True):                                   # noqa: C901
    """Constructed answers. The order of a scan is arithmetic."""
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        ok = ok and bool(cond)
        if verbose:
            print("  %-4s %s%s" % ("PASS" if cond else "FAIL", name,
                                   ("  -- " + detail) if detail else ""))

    xs, ys = grid(0.30, 0.54, 0.30, 0.48, 0.06)
    check("the grid reaches BOTH edges",
          xs[0] == 0.30 and xs[-1] == 0.54 and ys[0] == 0.30 and ys[-1] == 0.48,
          "x %s y %s" % (xs, ys))
    check("the step is recomputed to fit exactly",
          abs((xs[1] - xs[0]) - 0.06) < 1e-9)

    cells = serpentine(xs, ys)
    check("every cell is visited exactly once",
          len(cells) == len(xs) * len(ys)
          and len({(c[0], c[1]) for c in cells}) == len(cells),
          "%d cells" % len(cells))

    # 1 -- WITHIN A ROW, ONLY x MOVES. This is what "a straight line" means
    #      and it is the whole complaint the sweep exists to answer.
    bad = [(a, b) for a, b in zip(cells, cells[1:])
           if a[2] == b[2] and abs(a[1] - b[1]) > 1e-9]
    check("within a row the arm moves only along x", not bad, str(bad[:2]))

    # 2 -- consecutive cells in a row are ADJACENT: no skipping, no jumping
    step = xs[1] - xs[0]
    bad = [(a, b) for a, b in zip(cells, cells[1:])
           if a[2] == b[2] and abs(abs(a[0] - b[0]) - step) > 1e-9]
    check("consecutive cells in a row are one step apart", not bad,
          str(bad[:2]))

    # 3 -- rows alternate direction, which is what makes the row change short
    dirs = []
    for j in range(len(ys)):
        row = [c for c in cells if c[2] == j]
        dirs.append(1 if row[-1][0] > row[0][0] else -1)
    check("rows alternate direction",
          all(dirs[k] != dirs[k + 1] for k in range(len(dirs) - 1)),
          str(dirs))

    # 4 -- the row change is ONE step in y and no motion in x
    changes = [(a, b) for a, b in zip(cells, cells[1:]) if a[2] != b[2]]
    check("a row change is one step in y and nothing in x",
          all(abs(a[0] - b[0]) < 1e-9
              and abs(abs(a[1] - b[1]) - (ys[1] - ys[0])) < 1e-9
              for a, b in changes),
          "%d row changes" % len(changes))

    # 5 -- AND IT IS SHORTER THAN THE OBVIOUS ALTERNATIVE. A check that
    #      cannot fail on a worse input is not a check, so the worse input is
    #      built and measured.
    t_serp = travel_m(cells)
    t_type = travel_m(typewriter(xs, ys))
    check("serpentine travels less than typewriter", t_serp < t_type,
          "%.3f m vs %.3f m (%.0f%% less)"
          % (t_serp, t_type, 100 * (1 - t_serp / t_type)))
    long_hops = [1 for a, b in zip(typewriter(xs, ys), typewriter(xs, ys)[1:])
                 if math.dist(a[:2], b[:2]) > 1.5 * step]
    check("and typewriter really does fly back across the workspace",
          len(long_hops) == len(ys) - 1,
          "%d returns of %.2f m" % (len(long_hops), xs[-1] - xs[0]))

    # 6 -- the probe presses THROUGH the estimate, not down to it
    p = probe(cells[0], 1.25, 0.12, 0.005)
    check("the probe hovers, presses below the estimate, and retracts",
          len(p) == 3 and p[0] == p[2] and p[1][2] < 1.25 < p[0][2],
          "%s" % p)
    check("the probe does not move in x or y",
          all(abs(q[0] - cells[0][0]) < 1e-9 and abs(q[1] - cells[0][1]) < 1e-9
              for q in p))

    # 7 -- refusals, by name
    for bad_args, what in (((0.3, 0.5, 0.3, 0.5, 0.0), "a zero step"),
                           ((0.3, 0.5, 0.3, 0.5, -0.1), "a negative step")):
        try:
            grid(*bad_args)
            check("refuses %s" % what, False)
        except SweepError:
            check("refuses %s" % what, True)
    try:
        serpentine([], [])
        check("refuses an empty sweep", False)
    except SweepError:
        check("refuses an empty sweep", True)
    try:
        plan(0.3, 0.5, 0.3, 0.5, 1.25, order="spiral")
        check("refuses an unknown order", False)
    except SweepError:
        check("refuses an unknown order", True)

    # 8 -- a single row and a single column still work
    one = serpentine(*grid(0.4, 0.4, 0.3, 0.42, 0.06))
    check("a single column is still a sweep", len(one) == 3, str(one))

    # 9 -- reversing the bounds does not reverse the map
    a = grid(0.30, 0.54, 0.30, 0.48, 0.06)
    b = grid(0.54, 0.30, 0.48, 0.30, 0.06)
    check("the bounds may be given either way round", a == b)

    p = plan(0.30, 0.54, 0.30, 0.48, 1.25, 0.06)
    check("the plan carries three waypoints per cell",
          len(p["waypoints"]) == 3 * len(p["cells"]))
    if verbose:
        print("calibration_sweep self-test %s" % ("PASSED" if ok else "FAILED"))
    return ok


if __name__ == "__main__":
    import sys
    if "--show" in sys.argv:
        p = plan(0.30, 0.54, 0.30, 0.48, 1.25)
        print("%d x %d cells, step %.3f x %.3f m, %.3f m of travel, %s"
              % (p["n_cols"], p["n_rows"], p["step_x_m"], p["step_y_m"],
                 p["travel_m"], p["order"]))
        last = None
        for x, y, r, c in p["cells"]:
            if last is not None and r != last:
                print("   -- step to row %d --" % r)
            print("   row %d col %d   x %+.3f  y %+.3f" % (r, c, x, y))
            last = r
        sys.exit(0)
    sys.exit(0 if self_test() else 1)
