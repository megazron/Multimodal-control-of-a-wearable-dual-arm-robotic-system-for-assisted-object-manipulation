"""THE SHIPPED T1 CONSTANTS ARE THE ONES THAT WERE VERIFIED, OR THIS FAILS.

`t1_task.py` carries the layout as constants and
`recordings/baselines/t1_paths.json` is the record of `verify_t1.py` walking
it. Those are two descriptions of one thing, and this repository's own history
says two descriptions drift: the pads moved twice on 2026-08-18 and
`t1_layout.json` was left recording the first of those, 105 mm from where the
task actually sends the arm, while every check that reads the CODE went on
passing.

So the baseline is deleted rather than left stale, and this test pins the code
against the record that is regenerated every time the paths are re-walked. If
you move a coordinate, re-run

    python3 scripts/verify_t1.py --mode sequential

and this goes green again with a record behind it. If you cannot, the
coordinate does not verify and should not ship.

WHY THE VERDICT IS CHECKED TOO. A `t1_paths.json` whose layout matches the
code but whose verdict is `False` would satisfy a naive comparison while
recording that the layout does not work.
"""
import json
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "src", "srl_experiments",
                                "experiments", "abc"))

BASELINE = os.path.join(ROOT, "recordings", "baselines", "t1_paths.json")


@pytest.fixture(scope="module")
def rec():
    if not os.path.exists(BASELINE):
        pytest.skip("no t1_paths.json -- run scripts/verify_t1.py")
    return json.load(open(BASELINE))


@pytest.fixture(scope="module")
def T1():
    import t1_task as m
    return m


def test_the_baseline_says_the_layout_is_clean(rec):
    assert rec.get("clean") is True, (
        "t1_paths.json records a layout that did NOT verify; the constants in "
        "t1_task.py must not be shipped against it")
    for arm in ("left", "right"):
        a = rec["arms"][arm]
        assert a["ik_failures"] == 0, (arm, a["ik_failures"])
        assert a["floor_breaches"] == 0, (arm, a["floor_breaches"])


def test_every_shipped_coordinate_is_the_one_that_was_walked(rec, T1):
    lay = rec["layout"]
    assert [list(p) for p in T1.T1_PLANES] == [list(p) for p in lay["planes"]]
    assert [list(c) for c in T1.T1_CUBES] == [list(c) for c in lay["cubes"]]
    assert T1.T1_Z == pytest.approx(lay["z"], abs=1e-9)
    assert T1.TABLE_TOP == pytest.approx(lay["table_top"], abs=1e-9)
    assert T1.TABLE_NEAR_Y == pytest.approx(lay["near_y"], abs=1e-9)


def test_the_approach_is_the_one_that_was_walked(rec, T1):
    ap = rec["layout"]["approach"]
    assert T1.APPROACH_ELEV_DEG == pytest.approx(ap["elev"], abs=1e-9)
    assert T1.APPROACH_HEAD_DEG == pytest.approx(ap["head"], abs=1e-9)
    assert T1.APPROACH_ROLL_DEG == pytest.approx(ap["roll"], abs=1e-9)


def test_the_grasp_actually_closes_on_the_cube(rec):
    """T1-5, from FK on the finger tips rather than from a comment.

    The hand opens 85 mm. A 40 mm cube taken across a diagonal presents
    56.6 mm, which fits; T3's circuit box presents 195 mm and has verified
    clean for as long as it has existed because nothing asked this question.
    """
    for g in rec.get("grasps", []):
        assert g["solved"], g
        assert g["across_mm"] <= 85.0, g
        assert g["pad_miss_mm"] <= 2.0, g
        assert g["lowest_tip_above_table_mm"] > 0.0, (
            "the fingers are reaching through the surface the cube rests on")


def test_every_pad_slot_sits_on_a_measured_column(T1):
    """Each slot must be at a column the sweep measured clear, with margin.

    THIS USED TO BE TWO REMEMBERED NUMBERS -- "the left arm is clear from
    0.500, the right from 0.300" -- read off `centre_gap_down.json`, which was
    walked before the 2026-08-18 mount change moved both mounts 150 mm
    outboard. After that change those numbers describe a robot that no longer
    exists: `t1_pad_columns.json`, swept per column at N=10 on the geometry
    that ships, has the left arm mount-limited from |x| = 0.195 and the right
    from 0.255, and the left arm's own worst column (0.120) reads 0.1624 m
    rather than a breach.

    So the assertion reads the MEASUREMENT instead of a memory of one. A
    column with no record is a column nobody walked, and it fails.

    The margin is 20 mm over the 150 mm floor, which is this project's own
    rule: a pose that passes at the floor has nothing left to absorb the
    null-space branch scatter, and the pads were moved out precisely because
    one column read 0.1483 m on one run in three.
    """
    rec = os.path.join(ROOT, "recordings", "baselines", "t1_pad_columns.json")
    if not os.path.exists(rec):
        pytest.skip("no t1_pad_columns.json -- run "
                    "scripts/measure_pad_columns.py")
    d = json.load(open(rec))
    floor = float(d["floor"])
    for i, (px, _py) in enumerate(T1.T1_PLANES):
        arm = T1.arm_for_pad(i)
        rows = {round(float(r["column"]), 4): r for r in d["arms"][arm]}
        for slot in (round(abs(px) - T1.SLOT_DX, 4),
                     round(abs(px) + T1.SLOT_DX, 4)):
            r = rows.get(slot)
            assert r is not None, (
                "%s pad slot at |x| = %.3f is not in t1_pad_columns.json -- "
                "no one has measured this column. Add it to COLUMNS in "
                "scripts/measure_pad_columns.py and re-run."
                % (arm, slot))
            assert r["solved"] == r["of"], (arm, slot, r)
            assert r["worst_clearance"] >= floor + 0.020, (
                "%s pad slot at |x| = %.3f measured %.4f m from the wearer, "
                "against a %.3f m floor plus this project's 20 mm margin"
                % (arm, slot, r["worst_clearance"], floor))


def test_the_pads_straddle_the_centreline(T1):
    """One pad each side, and the pair centred on the person.

    T1's whole point after the rebuild is that the work happens in FRONT of
    the wearer with an arm either side of it. Two pads that drifted to the
    same side would still pass every reachability check in the file.
    """
    xs = [p[0] for p in T1.T1_PLANES]
    assert min(xs) < 0.0 < max(xs), (
        "both pads are on the same side of the centreline: %s" % xs)
    centre = sum(xs) / len(xs)
    assert abs(centre) <= 0.050, (
        "the pad pair is centred %.0f mm off the centreline, so the work is "
        "beside the wearer rather than in front" % (centre * 1000))
def test_the_cubes_start_clear_of_the_pads(T1):
    """T1-2, as geometry rather than as a hope."""
    for i, (cx, _cy) in enumerate(T1.T1_CUBES):
        arm = T1.arm_for_pad(T1.T1_PAIR[i])
        px = T1.T1_PLANES[T1.T1_PAIR[i]][0]
        gap = abs(cx) - (abs(px) + T1.PAD_W / 2.0) - T1.CUBE_M / 2.0
        assert gap > 0.0, (
            "cube %d (%s arm) at |x| = %.3f overlaps its pad, which spans to "
            "%.3f" % (i, arm, abs(cx), abs(px) + T1.PAD_W / 2.0))


def test_the_two_cubes_on_a_side_are_two_objects(T1):
    """60 mm pitch: T0's measured minimum separation. Two cubes 25 mm apart is
    one cube inside another, which an earlier run of the layout solver
    produced."""
    for arm in ("left", "right"):
        xs = sorted(abs(c[0]) for i, c in enumerate(T1.T1_CUBES)
                    if T1.arm_for_pad(T1.T1_PAIR[i]) == arm)
        for a, b in zip(xs, xs[1:]):
            assert b - a >= 0.060 - 1e-9, (arm, xs)
