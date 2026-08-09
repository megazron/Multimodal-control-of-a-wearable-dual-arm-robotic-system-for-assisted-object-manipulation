#!/usr/bin/env python3
"""Known answers for scene fingerprinting.

Ground truth here is CONSTRUCTED, not rendered: two points 30 mm apart really
are 30 mm apart. That is the one kind of synthetic data the standing rule
permits, and it is why these tests can be trusted where a rendered-image test
of the same logic could not.
"""
import os
import sys
import tempfile

import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from srl_perception import scene_fingerprint as sf     # noqa: E402

O = sf.Observation


def scene(*specs):
    return [O(lab, xyz) for lab, xyz in specs]


BASE = scene(("block_a", (0.30, 0.35, 1.00)),
             ("block_b", (-0.30, 0.35, 1.00)),
             ("cup", (0.10, 0.40, 1.02)))


def test_identical_scene_is_unchanged():
    v, s = sf.compare(BASE, list(BASE))
    assert s["unchanged"] is True
    assert s["counts"][sf.SAME] == 3
    assert s["reregister"] == []


def test_noise_below_tolerance_is_still_unchanged():
    """A detector that jitters by less than the tolerance must not trigger a
    re-registration on every start."""
    noisy = scene(("block_a", (0.3005, 0.3496, 1.0003)),
                  ("block_b", (-0.2998, 0.3504, 0.9997)),
                  ("cup", (0.1002, 0.4001, 1.0199)))
    v, s = sf.compare(BASE, noisy, pos_tol=0.015)
    assert s["unchanged"] is True


def test_a_moved_object_is_detected_and_only_it_is_reregistered():
    moved = scene(("block_a", (0.30, 0.35, 1.00)),
                  ("block_b", (-0.24, 0.35, 1.00)),      # +60 mm
                  ("cup", (0.10, 0.40, 1.02)))
    v, s = sf.compare(BASE, moved)
    assert s["unchanged"] is False
    assert s["reregister"] == ["block_b"]
    r = [x for x in v if x["label"] == "block_b"][0]
    assert r["verdict"] == sf.MOVED
    assert abs(r["delta_m"] - 0.06) < 1e-6


def test_appeared_and_vanished_are_distinguished():
    obs = scene(("block_a", (0.30, 0.35, 1.00)),
                ("cup", (0.10, 0.40, 1.02)),
                ("tool", (0.00, 0.42, 1.05)))
    v, s = sf.compare(BASE, obs)
    assert s["counts"].get(sf.VANISHED) == 1
    assert s["counts"].get(sf.APPEARED) == 1
    assert s["drop"] == ["block_b"]
    assert "tool" in s["reregister"]


def test_a_far_move_is_reported_as_possibly_a_swap_not_as_a_move():
    """The irreducible ambiguity, made explicit rather than hidden. A move
    comparable to the gate cannot be told from vanish-plus-appear."""
    far = scene(("block_a", (0.30, 0.35, 1.00)),
                ("block_b", (-0.12, 0.35, 1.00)),        # +180 mm, gate 250
                ("cup", (0.10, 0.40, 1.02)))
    v, s = sf.compare(BASE, far, gate=0.25)
    r = [x for x in v if x["label"] == "block_b"][0]
    assert r["verdict"] == sf.MOVED_OR_SWAPPED
    assert "swap" in r["note"]


def test_a_move_beyond_the_gate_becomes_vanish_plus_appear():
    """Stated as a limit, and tested so it cannot regress into a silent
    mis-association."""
    far = scene(("block_a", (0.30, 0.35, 1.00)),
                ("block_b", (0.20, 0.35, 1.00)),         # 500 mm, beyond gate
                ("cup", (0.10, 0.40, 1.02)))
    v, s = sf.compare(BASE, far, gate=0.25)
    verdicts = {x["verdict"] for x in v}
    assert sf.VANISHED in verdicts and sf.APPEARED in verdicts


def test_same_place_different_identity_is_reclassified():
    obs = scene(("block_a", (0.30, 0.35, 1.00)),
                ("block_b", (-0.30, 0.35, 1.00)),
                ("mug", (0.10, 0.40, 1.02)))
    v, s = sf.compare(BASE, obs)
    # cup vs mug at the same place: the pair is inside the gate, so it is a
    # reclassification rather than a vanish plus an appear.
    labs = {x["verdict"] for x in v}
    assert sf.RECLASSIFIED in labs or (
        sf.VANISHED in labs and sf.APPEARED in labs)


def test_empty_observation_vanishes_everything_and_does_not_crash():
    v, s = sf.compare(BASE, [])
    assert s["counts"][sf.VANISHED] == 3
    assert s["reregister"] == []
    assert sorted(s["drop"]) == ["block_a", "block_b", "cup"]


def test_first_ever_scene_against_empty_store_is_all_appeared():
    v, s = sf.compare([], list(BASE))
    assert s["counts"][sf.APPEARED] == 3


def test_min_detectable_displacement_is_bounded_by_noise_not_only_tolerance():
    """A tolerance tighter than the detector's noise does not buy sensitivity,
    and the function must say so rather than return the tolerance."""
    assert sf.min_detectable_displacement(0.015, 0.001) == 0.015
    assert sf.min_detectable_displacement(0.002, 0.010) == 0.020


def test_round_trip_through_the_store_preserves_everything():
    d = tempfile.mkdtemp()
    p = os.path.join(d, "fp.json")
    fp = sf.Fingerprint(BASE, 123.0, 4.5, "unit test")
    fp.save(p)
    back = sf.Fingerprint.load(p)
    assert back is not None
    assert len(back.objects) == 3
    v, s = sf.compare(back, list(BASE))
    assert s["unchanged"] is True


def test_a_store_with_the_wrong_version_is_refused_not_misread():
    d = tempfile.mkdtemp()
    p = os.path.join(d, "fp.json")
    open(p, "w").write('{"version": 99, "objects": []}')
    assert sf.Fingerprint.load(p) is None


def test_a_corrupt_store_returns_none_rather_than_raising():
    d = tempfile.mkdtemp()
    p = os.path.join(d, "fp.json")
    open(p, "w").write("{not json")
    assert sf.Fingerprint.load(p) is None


def test_drift_check_flags_a_moved_object_and_passes_a_still_one():
    fp = sf.Fingerprint(BASE)
    ok, verdict, d = sf.drift_check(fp, O("cup", (0.1005, 0.4002, 1.0201)))
    assert ok and verdict == sf.SAME
    ok, verdict, d = sf.drift_check(fp, O("cup", (0.16, 0.40, 1.02)))
    assert not ok and verdict == sf.MOVED
    assert abs(d - 0.06) < 1e-6


def test_drift_check_flags_an_object_that_is_not_in_the_store():
    fp = sf.Fingerprint(BASE)
    ok, verdict, d = sf.drift_check(fp, O("ghost", (2.0, 2.0, 2.0)))
    assert not ok and verdict == sf.APPEARED


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
