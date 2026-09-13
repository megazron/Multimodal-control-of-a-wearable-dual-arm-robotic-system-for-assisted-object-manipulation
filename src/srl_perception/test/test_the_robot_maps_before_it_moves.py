"""Calibrate, map, then plan against the map -- and say so while doing it.

WHAT WAS WRONG. Almost everything this rig does runs on DECLARED
COORDINATES: T1's cubes are at (+/-0.42, 0.45) because a file says so, the
table is at 1.250 because a file says so. All of it verified N=10 over
densified paths, and none of it survives a real cell with a different table
and different objects on it.

The parts that could sense instead already existed and were never composed:

  * `table_scene.analyse` finds a plane and the objects on it, from ONE view,
    and was used to PRINT A DESCRIPTION;
  * `joint_planner.VoxelWorld` lets the planner avoid things that are not the
    wearer -- and was constructed NOWHERE outside its own self-test, so the
    planner has never known the table exists;
  * `work_surface.set_measured()` exists so the surface height can come from
    depth, and docs/ENGINEERING_LOG.md records that nothing calls it.

So: eyes, a planner that can avoid obstacles, somewhere to put the answer,
and no stage joining them. These tests are about that stage.

  1  the scan has an ORDER a person can predict -- straight rows, adjacent
     cells, alternating direction -- because "it moves in random directions"
     is what watching the recordings actually looks like;
  2  many views fuse into ONE map, whose surface height is MEASURED;
  3  the map reaches the planner, and the planner then avoids the surface it
     used to sweep through;
  4  every stage says what it is doing, in one vocabulary.

Constructed geometry throughout: the ground truth is arithmetic, which is
what docs/ENGINEERING_LOG.md's standing rule allows synthetic data for. NO CAMERA HAS EVER
BEEN ATTACHED TO THIS HOST, so "the robot has seen the room" is not tested
here and is not claimed anywhere.
"""
import math
import os

import numpy as np
import pytest

from srl_experiments import narration as N
from srl_perception import calibration_sweep as CS
from srl_perception import joint_planner as JP
from srl_perception import world_model as WM


# --------------------------------------------------------------- the modules

def test_every_module_passes_its_own_self_test():
    assert CS.self_test(verbose=False)
    assert WM.self_test(verbose=False)
    assert N.self_test(verbose=False)


# ------------------------------------------------- 1. the scan has an ORDER

def test_the_scan_walks_straight_rows_not_random_directions():
    """The complaint, as an assertion. Within a row only x changes, and each
    step is exactly one cell."""
    xs, ys = CS.grid(0.30, 0.54, 0.30, 0.48, 0.06)
    cells = CS.serpentine(xs, ys)
    step = xs[1] - xs[0]
    for a, b in zip(cells, cells[1:]):
        if a[2] == b[2]:                       # same row
            assert abs(a[1] - b[1]) < 1e-9, ("y moved inside a row", a, b)
            assert abs(abs(a[0] - b[0]) - step) < 1e-9, ("skipped a cell", a, b)
        else:                                  # row change
            assert abs(a[0] - b[0]) < 1e-9, ("x moved on a row change", a, b)


def test_no_step_of_the_scan_is_a_long_unexplained_transit():
    """What "random directions" looks like on film is a long hop. The whole
    sweep must have none: every step is one cell in one axis."""
    xs, ys = CS.grid(0.30, 0.54, 0.30, 0.48, 0.06)
    cells = CS.serpentine(xs, ys)
    step = max(xs[1] - xs[0], ys[1] - ys[0])
    longest = max(math.dist(a[:2], b[:2]) for a, b in zip(cells, cells[1:]))
    assert longest <= step + 1e-9, "longest step %.3f m" % longest


def test_the_obvious_alternative_really_does_fly_back_and_forth():
    """THE CONTROL. If typewriter order had no long hops, the serpentine
    would be arbitrary and this whole property would be untestable."""
    xs, ys = CS.grid(0.30, 0.54, 0.30, 0.48, 0.06)
    tw = CS.typewriter(xs, ys)
    step = xs[1] - xs[0]
    longest = max(math.dist(a[:2], b[:2]) for a, b in zip(tw, tw[1:]))
    assert longest > 3 * step, "typewriter's longest step is only %.3f m" % longest
    assert CS.travel_m(CS.serpentine(xs, ys)) < CS.travel_m(tw)


def test_the_scan_covers_every_cell_exactly_once():
    xs, ys = CS.grid(0.30, 0.54, 0.30, 0.48, 0.06)
    cells = CS.serpentine(xs, ys)
    assert len(cells) == len(xs) * len(ys)
    assert len({(c[0], c[1]) for c in cells}) == len(cells)


def test_the_scan_takes_its_bounds_and_assumes_no_table():
    """The real cell has a different table. A sweep that only works over
    declared coordinates works exactly once."""
    a = CS.plan(0.30, 0.54, 0.30, 0.48, 1.25, 0.06)
    b = CS.plan(-0.60, -0.20, 0.10, 0.40, 0.83, 0.10)
    assert a["bounds"] != b["bounds"]
    assert b["bounds"]["surface_z"] == 0.83
    assert all(abs(w[2] - 0.83) <= 0.13 for w in b["waypoints"])


# ------------------------------------------------------- 2. views -> one map

def _table(z=0.90, n=4000, seed=1):
    rng = np.random.default_rng(seed)
    return np.column_stack([rng.uniform(0.2, 0.7, n), rng.uniform(0.2, 0.6, n),
                            np.full(n, z)])


def _cube(centre, size=0.05, n=900, seed=2):
    rng = np.random.default_rng(seed)
    c = np.asarray(centre, float)
    return np.column_stack([c[k] + rng.uniform(-size / 2, size / 2, n)
                            for k in range(3)])


def test_the_surface_height_is_measured_not_declared():
    """The number `work_surface.set_measured()` was written to receive."""
    for z in (0.83, 0.90, 1.25):
        m = WM.build([WM.View(_table(z), "wrist_left")])
        assert abs(m.surface_z - z) < 0.003, (z, m.surface_z)


def test_two_views_of_one_object_are_one_object():
    box = (0.40, 0.40, 0.93)
    m = WM.build([WM.View(np.vstack([_table(), _cube(box)]), "wrist_left"),
                  WM.View(np.vstack([_table(seed=7), _cube(box, seed=8)]),
                          "wrist_right")])
    assert len(m.objects) == 1, m.objects
    assert len(m.objects[0].seen_by) == 2


def test_two_different_objects_stay_two():
    """THE CONTROL for the merge rule: one that merged everything would pass
    the test above and be worthless."""
    m = WM.build([WM.View(np.vstack([_table(), _cube((0.40, 0.40, 0.93)),
                                     _cube((0.60, 0.40, 0.93), seed=9)]),
                          "scene")])
    assert len(m.objects) == 2, m.objects


def test_objects_at_the_separation_this_rig_actually_uses_stay_two():
    """AND AT 60 mm, WHICH IS THE ONE THAT MATTERED.

    The test above uses objects 200 mm apart and passed while the map was
    fusing real ones. T1's cubes are 60 mm apart -- 40 mm wide with a 20 mm
    gap -- and on the third live calibration run they came back as a single
    "140 mm" object, which is 100 mm of span plus a cube. Two causes, both
    mine: `MERGE_M` was 0.06, so a 60 mm separation was inside the dedup
    distance; and `table_scene`'s default `cluster_m` is 0.020, exactly the
    gap between two adjacent cube faces.

    A pick planned off that map is refused on an object that does not exist.
    """
    m = WM.build([WM.View(np.vstack([
        _table(),
        _cube((0.40, 0.40, 0.93), size=0.04, seed=3),
        _cube((0.46, 0.40, 0.93), size=0.04, seed=4)]), "scene")])
    assert len(m.objects) == 2, (
        "two cubes 60 mm apart came back as %d object(s): %s"
        % (len(m.objects), m.objects))
    for o in m.objects:
        assert o.width_m < 0.07, (
            "an object %.0f mm wide, where the cubes are 40 mm -- two have "
            "been fused" % (o.width_m * 1000))


def test_the_map_names_its_sources_and_flags_a_single_view():
    one = WM.build([WM.View(_table(), "wrist_left")])
    assert one.provenance["sources"] == ["wrist_left"]
    assert one.provenance["single_view_caveat"] is True
    assert any("ONE view only" in ln for ln in one.describe())


@pytest.mark.parametrize("bad,why", [
    ([], "an empty view list"),
    ([WM.View(np.zeros((5, 3)), "dust")], "a view with too few points"),
])
def test_it_refuses_rather_than_guessing(bad, why):
    with pytest.raises(WM.MapRefusal):
        WM.build(bad)


def test_a_cloud_with_no_surface_in_it_is_refused():
    rng = np.random.default_rng(0)
    with pytest.raises(WM.MapRefusal) as e:
        WM.build([WM.View(rng.uniform(0, 1, (3000, 3)), "noise")])
    assert "support surface" in str(e.value)


def test_an_empty_table_gives_a_surface_and_no_phantom_objects():
    m = WM.build([WM.View(_table(0.90), "empty")])
    assert not m.objects
    assert abs(m.surface_z - 0.90) < 0.003


# ------------------------------------------- 3. the map reaches the planner

def test_the_planner_can_be_given_the_map_it_never_had():
    """`VoxelWorld` was constructed nowhere outside its own self-test. This
    is the seam."""
    m = WM.build([WM.View(_table(0.90), "mock")])
    w = JP.world_from_map(m, voxel_m=0.03)
    assert len(w) > 0, "the map produced an empty world"
    assert w.n_points == len(m.occupancy())


def test_and_the_planner_then_sees_the_surface_it_used_to_sweep_through():
    """A point ON the mapped table is an obstacle; one well above it is not.

    Both directions, because a world that reported everything as a hit would
    pass the first half and stop the arm moving at all.
    """
    m = WM.build([WM.View(_table(0.90), "mock")])
    w = JP.world_from_map(m, voxel_m=0.03)
    on_table = np.array([[0.45, 0.40, 0.90]])
    well_above = np.array([[0.45, 0.40, 1.40]])
    assert w.hits(on_table, margin_m=0.0) is not None
    assert w.hits(well_above, margin_m=0.0) is None


def test_the_occupancy_includes_the_surface_and_not_only_the_objects():
    """The planner's blindness was to the TABLE, not to the cubes."""
    m = WM.build([WM.View(np.vstack([_table(0.90), _cube((0.40, 0.40, 0.93))]),
                          "mock")])
    occ = m.occupancy()
    on_plane = int(np.sum(np.abs(occ[:, 2] - 0.90) < 0.005))
    assert on_plane > 1000, "%d points on the plane" % on_plane


def test_a_checker_given_a_world_without_link_points_still_refuses():
    """The planner's own guard, unchanged. Accepting a world and not using it
    is the defect this whole file is about, one level down."""
    m = WM.build([WM.View(_table(0.90), "mock")])
    with pytest.raises(Exception):
        JP.Checker(lambda q: 1.0, [-1] * 7, [1] * 7, (0, 2, 4, 6),
                   world=JP.world_from_map(m), link_points=None)


# ------------------------------------------------- 4. it says what it is doing

def test_the_scan_can_say_which_cell_it_is_on():
    p = CS.plan(0.30, 0.54, 0.30, 0.48, 1.25, 0.06)
    s = N.text("TOUCHING", arm="left", cell=7, of=len(p["cells"]),
               at=p["cells"][6][:2] + (1.25,))
    assert "cell 7 of 20" in s
    assert "touching" in s


def test_the_map_can_say_what_it_found():
    m = WM.build([WM.View(np.vstack([_table(0.90), _cube((0.40, 0.40, 0.93))]),
                          "mock")])
    lines = m.describe()
    assert any("support surface at z" in ln for ln in lines)
    assert any("object 0 at" in ln for ln in lines)
    # and the narration can carry it
    assert "building the map" in N.text("MAPPING")


def test_the_phases_a_calibration_run_passes_through_all_exist():
    for phase in ("STARTING", "HOMING", "CALIBRATING", "MAPPING", "PLANNING",
                  "REACHING", "TOUCHING", "RETURNING", "DONE", "REFUSED"):
        assert phase in N.PHASES
        assert N.text(phase)


def test_the_narration_orders_the_run():
    assert N.is_progress("CALIBRATING", "MAPPING")
    assert N.is_progress("MAPPING", "PLANNING")
    assert not N.is_progress("PLANNING", "CALIBRATING")


# ------------------------- 5. the sweep is a VOLUME, held at one attitude

def test_the_sweep_visits_more_than_one_height():
    """A 3-D printer has one plane to probe. This arm works in a volume, and
    an object standing on the surface has SIDES that no single elevation can
    see."""
    p = CS.plan_volume(0.30, 0.58, 0.30, 0.52, 1.25, step_m=0.11)
    assert p["n_layers"] > 1, p["layers_m"]
    zs = sorted({c[2] for c in p["passes"][0]["cells"]})
    assert len(zs) == p["n_layers"], zs
    assert max(zs) - min(zs) > 0.05, zs


def test_the_sweep_takes_more_than_one_wrist_attitude():
    """One pass per facing. A vertical face is invisible to a camera looking
    straight at the surface in front of it."""
    p = CS.plan_volume(0.30, 0.58, 0.30, 0.52, 1.25, step_m=0.11)
    assert len(p["passes"]) > 1
    assert len({q["facing_deg"] for q in p["passes"]}) == len(p["passes"])


def test_every_cell_of_every_layer_is_visited_exactly_once():
    p = CS.plan_volume(0.30, 0.58, 0.30, 0.52, 1.25, step_m=0.11)
    c = p["passes"][0]["cells"]
    assert len({(x[0], x[1], x[2]) for x in c}) == len(c) == p["cells_per_pass"]


def test_rows_stay_straight_inside_a_layer():
    """The complaint that started the scan work, still asserted now that the
    sweep has layers: inside a layer every step is one cell in one axis."""
    p = CS.plan_volume(0.30, 0.58, 0.30, 0.52, 1.25, step_m=0.11)
    for layer in range(p["n_layers"]):
        cells = [c for c in p["passes"][0]["cells"] if c[5] == layer]
        for a, b in zip(cells, cells[1:]):
            moved = [abs(a[0] - b[0]) > 1e-9, abs(a[1] - b[1]) > 1e-9]
            assert sum(moved) == 1, ("two axes moved at once", a, b)


def test_the_layer_change_is_the_only_place_height_changes():
    p = CS.plan_volume(0.30, 0.58, 0.30, 0.52, 1.25, step_m=0.11)
    c = p["passes"][0]["cells"]
    changes = sum(1 for a, b in zip(c, c[1:]) if abs(a[2] - b[2]) > 1e-9)
    assert changes == p["n_layers"] - 1, changes


def test_travel_counts_the_height_it_actually_climbs():
    """THE CONTROL for the 3-D distance. A travel figure that ignored the
    layer change would understate the path it is used to justify."""
    p = CS.plan_volume(0.30, 0.58, 0.30, 0.52, 1.25, step_m=0.11)
    c = p["passes"][0]["cells"]
    flat = sum(math.dist(a[:2], b[:2]) for a, b in zip(c, c[1:]))
    full = CS.travel_m(c)
    assert full > flat + 0.05, (full, flat)


# --------------------- 6. the sweep plans from what the arm can reach

def test_a_measured_reachable_set_removes_the_cells_outside_it():
    """The swept box is a rectangle and the reachable set is not.

    Without this, a large minority of planned cells sit outside the arm and
    "coverage of planned" becomes a number about the shape of my rectangle
    rather than about the robot.
    """
    full = CS.plan_volume(0.325, 0.750, 0.15, 0.65, 1.25, step_m=0.13)
    assert not full["planned_from_measurement"]
    assert full["cells_dropped_as_unreachable"] == 0

    # Only two cells solved, at one facing and one layer.
    reach = {"%+.1f|%d" % (f, k): ([[0.325, 0.15], [0.4667, 0.15]]
                                   if (f, k) == (0.0, 0) else [])
             for f in full["facings_deg"] for k in range(full["n_layers"])}
    cut = CS.plan_volume(0.325, 0.750, 0.15, 0.65, 1.25, step_m=0.13,
                         reachable=reach)
    assert cut["planned_from_measurement"]
    assert cut["total_cells"] == 2, cut["total_cells"]
    assert cut["cells_dropped_as_unreachable"] == full["total_cells"] - 2


def test_an_absent_measurement_plans_everything():
    """THE CONTROL. A filter that dropped cells when it had no measurement
    would silently shrink the sweep to nothing on a fresh machine."""
    p = CS.plan_volume(0.325, 0.750, 0.15, 0.65, 1.25, step_m=0.13,
                       reachable=None)
    assert p["total_cells"] > 0
    assert p["cells_dropped_as_unreachable"] == 0


def test_a_facing_with_no_recorded_answer_is_still_probed():
    """An entry MISSING from the cache means "not asked", not "unreachable".
    Conflating them would let one bad run permanently blind a whole pass."""
    p = CS.plan_volume(0.325, 0.750, 0.15, 0.65, 1.25, step_m=0.13,
                       reachable={"+0.0|0": []})
    per_pass = [len(q["cells"]) for q in p["passes"]]
    assert sum(per_pass) > 0, per_pass


def test_the_settings_key_changes_when_the_geometry_does():
    """Reachability is a fact about the arm AT A GEOMETRY. A cache that did
    not key on the bounds would skip cells that became reachable when the
    volume moved."""
    base = CS.settings_key(0.325, 0.75, 0.15, 0.65, 1.25, 0.13,
                           (0.30, 0.42), (-25.0, 0.0, 25.0), 38.9)
    for changed in (
            CS.settings_key(0.300, 0.75, 0.15, 0.65, 1.25, 0.13,
                            (0.30, 0.42), (-25.0, 0.0, 25.0), 38.9),
            CS.settings_key(0.325, 0.75, 0.15, 0.65, 1.25, 0.11,
                            (0.30, 0.42), (-25.0, 0.0, 25.0), 38.9),
            CS.settings_key(0.325, 0.75, 0.15, 0.65, 1.25, 0.13,
                            (0.30, 0.50), (-25.0, 0.0, 25.0), 38.9),
            CS.settings_key(0.325, 0.75, 0.15, 0.65, 1.25, 0.13,
                            (0.30, 0.42), (-25.0, 0.0, 25.0), 45.0)):
        assert changed != base


# ------------- 7. the table is FOUND, not assumed to be where it used to be

def test_the_swept_bounds_can_come_from_a_measurement():
    """`BOUNDS` was a constant: a box in FRONT of the wearer. So the stage
    adapted to any table HEIGHT, any SIZE and any arrangement of objects, and
    to exactly one table POSITION. Put the table to one side and it would
    quarter the empty air where the table used to be and report a confident
    map of nothing.
    """
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "calenv", os.path.join(os.path.dirname(os.path.dirname(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))))),
            "scripts", "calibrate_environment.py"))
    ce = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ce)
    assert hasattr(ce, "find_table")
    # The measured envelope, and the wearer-safe inboard columns, are the two
    # things the found bounds must be clipped to.
    assert ce.ENVELOPE["x_max"] > 0.5
    assert ce.INBOARD_LIMIT["left"] > 0 > ce.INBOARD_LIMIT["right"]
    assert ce.ENVELOPE["y_min"] < 0, "the arm reaches behind the frontal plane"


def test_a_table_out_of_reach_is_refused_not_swept_as_an_inverted_range():
    """FOUND BY MOVING THE TABLE 450 mm FORWARD, past the arm's measured
    forward limit of y = 0.65.

    The table's real extent was y 0.819..1.25. Clipping to the envelope gave
    lo = max(0.819, -0.25) = 0.819 and hi = min(1.25, 0.65) = 0.65 -- an
    INVERTED range -- and the sweep was told to cover "y 0.819..0.650". The x
    span was guarded and the y span was not.

    An empty interval is not a small table; it means the table is outside what
    the arms can reach, and the only honest output is to say so.
    """
    import importlib.util
    root = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))))
    spec = importlib.util.spec_from_file_location(
        "calenv2", os.path.join(root, "scripts", "calibrate_environment.py"))
    ce = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ce)
    src = open(os.path.join(root, "scripts",
                            "calibrate_environment.py")).read()
    # The y span must be checked the same way the x span is.
    assert "hi_y_c - lo_y_c" in src, "the y interval is still unguarded"
    assert "out of reach" in src.lower() or "arms' reach" in src, \
        "an out-of-reach table must be REFUSED by name"
