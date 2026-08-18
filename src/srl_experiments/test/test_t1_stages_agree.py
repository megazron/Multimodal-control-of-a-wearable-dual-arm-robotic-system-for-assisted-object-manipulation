"""T1's two stages are ONE task, and as of 2026-08-18 they are one geometry.

Stage 1 places four cubes onto two coloured pads: blue cube to blue pad, green
to green, the pads straddling the centreline directly in front of the wearer.
Stage 2 is the same task with the SIDE of each cube drawn per trial and both
arms working.

THIS FILE HAS ASSERTED BOTH ANSWERS, AND THE HISTORY IS THE POINT.

  * Until 2026-08-17 the two shared geometry and this asserted the agreement.
  * Stage 1 was then deleted and rebuilt for `06_full_autonomy` alone -- pads
    across the centreline, objects RESTING on the table, its own approach --
    while stage 2 stayed at the pinned anchor with a pad PAIR per side on the
    work plane 120 mm above the table. Pulling stage 2 across on that day
    would have meant placing its cubes onto pads whose paths had never been
    walked, so the tests were flipped to assert the DIVERGENCE, loudly,
    rather than let a reader assume it was not there.
  * On 2026-08-18 stage 2's paths were walked, per seed, on stage 1's
    geometry. So the divergence is closed and these assert the agreement
    again -- with the walk behind them this time.

WHAT MAKES THE STAGES ONE TASK NOW: same table, same two pads at +/- 0.290,
same row, same approach, same landing slots, and the SAME BUILDER --
`t1_task.build()` with a drawn layout instead of a fixed one, so stage 2
cannot acquire a dwell, a standoff or a slot spacing that stage 1 does not
have.

THE COLOUR FOLLOWS THE SIDE, AND IT IS A MEASUREMENT. Neither arm can cross
the centreline: 0 of 10 IK solutions at every cross-side pad slot and every
cross-side cube, on both arms, wearer and table in the scene. So the arm that
reaches a cube is fixed by its side, the arm that reaches a pad is fixed by
the pad's side, and a cube can only be delivered to the pad on its own side.
A stage 2 that drew colour and side independently would be drawing trials the
rig cannot perform -- which is why the old stage 2 needed a pad of each colour
on each side, and why it could not share stage 1's two.

WHAT IS CHECKED, all of it arithmetic on the task definitions:

  * both stages use the same pads, the same row and the same work plane;
  * both arms always get work, which is stage 2's definition;
  * the split genuinely varies across seeds -- a draw that is 2/2 every time
    is not a draw;
  * every cube's place target IS one of the two pads, at a measured slot;
  * a cube is never sent across the centreline, which the builder refuses.
"""
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))
# `clip_scene` owns the pad SIZES and the drawn marking. It lives in scripts/.
sys.path.insert(0, os.path.join(ROOT, "scripts"))

SEEDS = [0, 1, 2, 3, 4, 5, 6, 7]


@pytest.fixture(scope="module")
def MCT():
    import msc_clip_tasks as m
    return m


@pytest.fixture(scope="module")
def T1M():
    import t1_task as m
    return m


def test_the_two_stages_now_AGREE_about_geometry(MCT, T1M):
    assert MCT.T1_PLANES is T1M.T1_PLANES
    assert MCT.T1_Z == T1M.T1_Z
    assert MCT.T1S2_Z == MCT.T1_Z, (
        "stage 2's work plane has drifted from stage 1's again. If that was "
        "deliberate, stage 2 has to be re-walked at the new height and this "
        "test rewritten to say so.")
    for arm, want in (("left", 0), ("right", 1)):
        got = [list(p) for p in MCT.T1_PLANES_BY_ARM[arm]]
        assert got == [list(T1M.T1_PLANES[want])], (arm, got)


def test_the_pads_straddle_the_centreline_and_do_not_overlap(MCT, T1M):
    xs = [p[0] for p in T1M.T1_PLANES]
    assert min(xs) < 0.0 < max(xs), xs
    import clip_scene as CS
    w = max(CS.PLANE_SIZE_BY_ARM[a][0] for a in ("left", "right"))
    gap = abs(xs[0] - xs[1]) - w
    assert gap >= 0.015, (gap, w, "a cube released over one pad could land "
                                  "on the other")


def test_every_pad_footprint_is_inside_that_arms_marked_region(MCT):
    """A participant is told to work inside the marking.

    A pad painted partly outside it invites a cube to be placed where the arm
    is measurably inside the wearer, and no IK check would ever object.

    IT IS T1'S OWN MARKING, NOT THE GLOBAL SURVEY. `REGION_CELLS` was measured
    at the pinned anchor on the work plane 120 mm above the table over
    y = 0.075 to 0.300; T1 works at its own approach, ON the table, in a single
    row at y = 0.450, so not one of those cells is over this task's work.
    `t1_marking_cells` is the boundary the task's own verification walked, and
    it is what the clip draws.

    ASKED AS AN EXTENT RATHER THAN CELL BY CELL. The marking's grid starts at
    the leftmost object minus a cube, so its cell centres are not multiples of
    the step and a cell-membership test compares two grids that were never
    meant to line up. "Enclosing" is a statement about the boundary.
    """
    import clip_scene as CS
    h = CS.REGION_STEP / 2.0
    for task in ("t1", "t1s2"):
        for arm in ("left", "right"):
            cells = CS.t1_marking_cells(arm, task)
            assert cells, (task, arm, "no marking at all")
            x0 = min(c[0] for c in cells) - h
            x1 = max(c[0] for c in cells) + h
            y0 = min(c[1] for c in cells) - h
            y1 = max(c[1] for c in cells) + h
            w, d = CS.PLANE_SIZE_BY_ARM[arm]
            for px, py in MCT.T1_PLANES_BY_ARM[arm]:
                assert x0 - 1e-9 <= px - w / 2 and px + w / 2 <= x1 + 1e-9, (
                    task, arm, "pad x %.3f +/- %.3f is outside the marking "
                    "%.3f..%.3f" % (px, w / 2, x0, x1))
                assert y0 - 1e-9 <= py - d / 2 and py + d / 2 <= y1 + 1e-9, (
                    task, arm, "pad y %.3f +/- %.3f is outside the marking "
                    "%.3f..%.3f" % (py, d / 2, y0, y1))


def test_the_marking_encloses_every_cube_of_both_stages(MCT):
    """The marking a participant is shown must contain the work it bounds.

    Stage 2 draws its cubes, so this is asked per seed: a marking taken from
    stage 1's fixed columns would cut through a stage 2 cube on the outer
    column, and the frame would show the task outside its own boundary --
    which this project has already shipped once, 116.5 mm out in z.
    """
    import clip_scene as CS
    import t1_task as T1
    h = CS.REGION_STEP / 2.0
    for seed in SEEDS:
        for arm in ("left", "right"):
            cells = CS.t1_marking_cells(arm, "t1s2", seed)
            if not cells:
                continue
            x0 = min(c[0] for c in cells) - h
            x1 = max(c[0] for c in cells) + h
            for cx, cy, pad in T1.stage2_layout(seed):
                if T1.arm_for_pad(pad) != arm:
                    continue
                assert x0 - 1e-9 <= cx - T1.CUBE_M / 2 and \
                    cx + T1.CUBE_M / 2 <= x1 + 1e-9, (seed, arm, cx, x0, x1)


def test_each_pad_is_on_its_own_arms_side(MCT):
    for x, _y in MCT.T1_PLANES_BY_ARM["right"]:
        assert x < 0, x
    for x, _y in MCT.T1_PLANES_BY_ARM["left"]:
        assert x > 0, x


@pytest.mark.parametrize("seed", SEEDS)
def test_every_stage2_place_is_a_measured_slot_on_this_arms_pad(MCT, T1M, seed):
    MCT.t1_stage2(seed=seed)
    st = MCT._T1S2
    tgt, places = st["targets"], st["places"]
    total = sum(len(v) for v in tgt.values())
    assert total == MCT.STAGE2_N_CUBES
    assert len(tgt["left"]) >= 1 and len(tgt["right"]) >= 1, (
        "both arms must get work; that is what makes it stage 2")
    for arm in ("left", "right"):
        pad_x = MCT.T1_PLANES_BY_ARM[arm][0][0]
        n = len(tgt[arm])
        want = sorted(round(pad_x + d, 4) for d in T1M.SLOT_OFFSETS[n])
        got = sorted(round(p[0], 4) for p in places[arm])
        assert got == want, (arm, seed, got, want)
        for p in places[arm]:
            assert round(p[1], 4) == round(T1M.ROW_Y, 4), (arm, p)
            assert round(p[2], 4) == round(T1M.T1_Z + T1M.PAD_T, 4), (arm, p)


@pytest.mark.parametrize("seed", SEEDS)
def test_no_stage2_cube_is_ever_sent_across_the_centreline(MCT, T1M, seed):
    for x, _y, pad in T1M.stage2_layout(seed):
        arm = T1M.arm_for_pad(pad)
        assert (x >= 0.0) == (arm == "left"), (seed, x, pad, arm)


@pytest.mark.parametrize("seed", SEEDS)
def test_every_stage2_cube_is_on_a_measured_column(MCT, T1M, seed):
    for x, y, _pad in T1M.stage2_layout(seed):
        assert round(abs(x), 4) in [round(c, 4) for c in T1M.CUBE_COLUMNS], x
        assert round(y, 4) == round(T1M.ROW_Y, 4), y


def test_the_split_actually_varies_across_seeds(T1M):
    """A draw that is 2/2 every time is not a draw.

    This is the "feature present but does nothing" row of CLAUDE.md's table:
    stage 2's whole claim is that the side is randomised, and a seed that is
    recorded but never changes the layout asserts a randomisation that did not
    happen. It has been exactly that fault once already.
    """
    splits = {tuple(sorted(T1M.stage2_split(s).items())) for s in range(24)}
    assert len(splits) >= 3, (
        "24 seeds produced only %d distinct splits: %s" % (len(splits), splits))
    left = {T1M.stage2_split(s)["left"] for s in range(24)}
    assert left >= {1, 2, 3}, (
        "the left arm never gets %s cubes over 24 seeds"
        % sorted({1, 2, 3} - left))


def test_the_same_seed_is_the_same_table_for_ever(T1M):
    for s in SEEDS:
        assert T1M.stage2_layout(s) == T1M.stage2_layout(s)


def test_a_cube_whose_colour_is_on_the_other_side_is_REFUSED(T1M):
    """The guard that makes the measurement enforceable rather than assumed.

    A cube's destination comes from the CAMERA, so a cube that renders the
    other side's colour -- which is exactly what the mislabel control creates
    -- names a pad its arm cannot reach. Emitting the path anyway would give
    a run whose every waypoint fails IK for a reason nobody would connect to
    colour.
    """
    with pytest.raises(ValueError) as e:
        T1M.build(cubes=[[0.420, T1M.ROW_Y, 1]])       # left cube, green pad
    assert "centreline" in str(e.value)


def test_a_pad_is_not_a_heap(T1M):
    """More cubes than the pad has slots is refused, not stacked."""
    with pytest.raises(ValueError):
        T1M.build(cubes=[[c, T1M.ROW_Y, 0] for c in (0.42, 0.48, 0.54, 0.60)])


def test_both_stages_DECLARE_the_approach_they_were_solved_at(MCT, T1M):
    """A task solved at its own approach must SAY so, or it is sent the anchor.

    `run_abc.set_orient(spec.get("orient"))` falls back to
    `master_calibration.WORKSPACE_ORIENT` when a task declares nothing, and it
    also rebuilds the finger-pad offset from whatever it is given. So a task
    whose coordinates were solved at another orientation and which forgets to
    declare it gets BOTH wrong: every waypoint is commanded at the pinned
    anchor, and the arrival gate looks for the pads 111.8 mm from where they
    are.

    MEASURED, on the first stage 2 recording: the pads read 1.4 to 4.2 mm from
    every cube -- because the SCENE was drawing them through the same wrong
    offset, so the two errors agreed -- while the knuckle stayed 0.00 for the
    whole run and NO GRASP WAS RECORDED AT ALL. Two descriptions sharing one
    error, which is CLAUDE.md's "everything matches" row, and it read as a
    perfect approach and a dead gripper.

    Stage 1 declared it. Stage 2 did not, and nothing compared them.
    """
    for key in ("t1", "t1s2"):
        spec = MCT.TASKS[key]
        assert spec.get("orient") is not None, (
            "%s builds its coordinates at t1_task.APPROACH and declares no "
            "orient, so run_abc will send it the pinned anchor" % key)
        assert spec["orient"] is T1M.APPROACH, (
            "%s declares an orientation that is not the one its layout was "
            "solved and walked at" % key)
