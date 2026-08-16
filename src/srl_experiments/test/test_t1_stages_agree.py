"""T1's two stages are ONE task, and they had drifted into two.

Stage 1 places four cubes onto two coloured pads: blue cube to blue pad, green
to green. Stage 2 is the same task with the SIDE of each cube randomised and
both arms working. It was placing onto coordinates drawn from each arm's own
surveyed cells -- arbitrary points, no colour rule, and nothing drawn on
screen to place onto. Two stages of one task with different targets, and the
half that is supposed to demonstrate divided attention had no colour matching
in it at all.

THE PADS CANNOT LITERALLY BE SHARED, and that is geometry rather than a
choice. Both sit at positive x, and the right arm cannot reach +0.450, which
is most of a metre across the far side of a person. So each arm gets its own
pair: same two colours, same pairing rule.

THEY ARE NO LONGER AN EXACT MIRROR, AND THAT CHANGED ON 2026-08-16. Two tests
here asserted mirrored x and a fixed 160 mm spacing. Both were correct while
the two arms' clearance-safe regions were the same shape. They are not:
`revalidate_region.py` re-walked every surveyed cell at the PINNED ANCHOR from
the new home -- the earlier survey solved at the live home wrist, which is a
different orientation entirely -- and the right arm lost 38 of its 246 cells
while the left lost 1 of 193. What is left on the right is ragged, with holes
through the columns from |x| 0.525 to 0.675.

Searched with the WHOLE pad footprint required to lie on surveyed cells, the
largest pair each arm can host is 0.210 x 0.130 at |x| 0.595/0.825 on the left
and 0.100 x 0.160 at |x| 0.740/0.860 on the right. A single mirrored pair that
fits both is 0.110 x 0.130 -- barely larger than the 0.14 x 0.10 it replaced.

So the exact-mirror invariant was dropped deliberately, and the tests below
now assert what actually has to hold: each arm's pads are on ITS OWN side,
they do not overlap, both stages use the same geometry, and -- the stronger
check that replaces mirroring -- every pad footprint lies inside that arm's
own marked region, which is the thing a participant is told to work within.

WHAT IS CHECKED, all of it arithmetic on the task definitions:

  * both stages use the same pad geometry, mirrored, with nothing invented;
  * a four-cube draw is always two blue and two green however the sides fall,
    because the colour follows the cube's place in the WHOLE draw. Keying it
    on the per-arm index gives three blue and one green on a 3/1 split, which
    is a different task;
  * every cube's place target IS one of the pads, not a drawn coordinate;
  * both arms always get work, which is stage 2's definition.
"""
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(ROOT, "src/srl_experiments/experiments/abc"))
# `clip_scene` owns the pad SIZES and the drawn marking, and both tests below
# compare the task's pad POSITIONS against them. It lives in scripts/.
sys.path.insert(0, os.path.join(ROOT, "scripts"))


@pytest.fixture(scope="module")
def MCT():
    import msc_clip_tasks as m
    return m


def test_both_stages_use_the_same_pad_geometry(MCT):
    """Stage 1's pads ARE the left arm's entry in the per-arm table."""
    left = MCT.T1_PLANES_BY_ARM["left"]
    right = MCT.T1_PLANES_BY_ARM["right"]
    assert left == [list(p) for p in MCT.T1_PLANES]
    assert len(right) == len(left) == len(MCT.PLANE_COLOURS)


def test_the_pads_on_each_side_do_not_overlap(MCT):
    """A cube released over one pad must not be able to land on the other.

    Replaces the old fixed-160-mm-spacing assertion: the two sides now carry
    different pad WIDTHS, so the invariant is a clear GAP, not a fixed pitch.
    """
    import clip_scene as CS
    for arm in ("left", "right"):
        pads = MCT.T1_PLANES_BY_ARM[arm]
        w, _d = CS.PLANE_SIZE_BY_ARM[arm]
        gap = abs(pads[1][0] - pads[0][0]) - w
        assert gap >= 0.015, (arm, gap, w)


def test_every_pad_footprint_is_inside_that_arms_marked_region(MCT):
    """The check that replaced exact mirroring, and a stronger one.

    A participant is told to work inside the marking. A pad painted partly
    outside it invites a cube to be placed where the arm is measurably inside
    the wearer, and no IK check would ever object.
    """
    import math

    import clip_scene as CS
    step = CS.REGION_STEP
    for arm in ("left", "right"):
        cells = set((round(x, 3), round(y, 3))
                    for x, y in CS.REGION_CELLS[arm])
        w, d = CS.PLANE_SIZE_BY_ARM[arm]
        for px, py in MCT.T1_PLANES_BY_ARM[arm]:
            i0 = int(math.floor((px - w / 2) / step + 0.5))
            i1 = int(math.floor((px + w / 2) / step + 0.5))
            j0 = int(math.floor((py - d / 2) / step + 0.5))
            j1 = int(math.floor((py + d / 2) / step + 0.5))
            for i in range(i0, i1 + 1):
                for j in range(j0, j1 + 1):
                    k = (round(i * step, 3), round(j * step, 3))
                    assert k in cells, (arm, (px, py), k)


def test_the_right_pads_are_on_the_right_arms_side(MCT):
    """A pad the arm cannot reach is not a target. The right arm works at
    negative x and its innermost safe column is 0.450."""
    for x, _y in MCT.T1_PLANES_BY_ARM["right"]:
        assert x < 0, x
        assert abs(x) >= 0.450 - 1e-9, x


@pytest.mark.parametrize("seed", [0, 1, 2, 3, 4])
def test_every_stage2_place_is_a_pad_and_the_colours_split_two_two(MCT, seed):
    MCT.t1_stage2(seed=seed)
    st = MCT._T1S2
    tgt, places = st["targets"], st["places"]
    total = sum(len(v) for v in tgt.values())
    assert total == MCT.STAGE2_N_CUBES
    assert len(tgt["left"]) >= 1 and len(tgt["right"]) >= 1, (
        "both arms must get work; that is what makes it stage 2")

    pads = {a: [tuple(p) for p in MCT.T1_PLANES_BY_ARM[a]]
            for a in ("left", "right")}
    colours = []
    for arm in ("left", "right"):
        for k, p in enumerate(places[arm]):
            assert (round(p[0], 6), round(p[1], 6)) in [
                (round(q[0], 6), round(q[1], 6)) for q in pads[arm]], (
                arm, k, p, "place target is not one of this arm's pads")
            idx = [(round(q[0], 6), round(q[1], 6))
                   for q in pads[arm]].index((round(p[0], 6), round(p[1], 6)))
            colours.append(idx)
    assert colours.count(0) == 2 and colours.count(1) == 2, (
        seed, colours, "a four-cube draw must be two blue and two green")


def test_the_colour_rule_is_the_same_one_stage_1_declares(MCT):
    """Stage 1's T1_PAIR is the rule; stage 2 must not invent a second one."""
    assert MCT.T1_PAIR == {0: 0, 1: 1, 2: 0, 3: 1}
    tgt = {"left": [None, None], "right": [None, None]}
    got = [MCT._stage2_pad_index(a, i, tgt)
           for a in ("left", "right") for i in range(2)]
    assert got == [MCT.T1_PAIR[n] for n in range(4)], got


def test_a_three_one_split_still_gets_two_of_each_colour(MCT):
    """The failure the whole-draw indexing exists to prevent."""
    tgt = {"left": [None, None, None], "right": [None]}
    got = [MCT._stage2_pad_index(a, i, tgt)
           for a in ("left", "right") for i in range(len(tgt[a]))]
    assert got.count(0) == 2 and got.count(1) == 2, got
