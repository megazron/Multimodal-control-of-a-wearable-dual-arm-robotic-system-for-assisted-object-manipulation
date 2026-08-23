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
    depth, and CLAUDE.md records that nothing calls it.

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
what CLAUDE.md's standing rule allows synthetic data for. NO CAMERA HAS EVER
BEEN ATTACHED TO THIS HOST, so "the robot has seen the room" is not tested
here and is not claimed anywhere.
"""
import math

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
