#!/usr/bin/env python3
"""KNOWN ANSWERS FOR MOVE_TO's TARGET RESOLVER.

"move to the front centre" has been in the spec and parsable since the
language layer was written, and no verb reached the resolver -- the executive
fell through to "I don't know how to 'goto'" and the system refused a command
it was designed to accept. These pin the half that was missing.

THE REGION IS BUILT HERE, not read from disk, so the answers are constructed
and known in advance. A test against the real survey would move whenever the
survey does and could not tell a resolver bug from a re-survey.
"""

import json
import os
import sys

import pytest

WS = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(WS, "src", "srl_autonomy"))

from srl_autonomy import named_places as NP                    # noqa: E402
from srl_autonomy import voice_intent as vi                    # noqa: E402


def _survey(tmp, left, right, z=1.12, full_path=True):
    p = os.path.join(tmp, "region.json")
    json.dump(dict(z=z, step=0.05, full_path=full_path,
                   cells=dict(left=left, right=right)), open(p, "w"))
    return p


def test_a_reachable_place_resolves_to_a_measured_cell(tmp_path):
    left = [[0.30, 0.10], [0.35, 0.10], [0.30, 0.15], [0.35, 0.15]]
    p = _survey(str(tmp_path), left, [[-0.30, 0.10]])
    spec = NP.resolve("left side", p)
    assert spec["reachable"] and spec["arm"] == "left"
    assert [spec["position"][0], spec["position"][1]] in [c for c in left]
    assert spec["position"][2] == 1.12


def test_the_centroid_is_snapped_to_a_CELL_not_returned_raw(tmp_path):
    """THE REGION IS NOT CONVEX AND THIS IS THE WHOLE POINT.

    An L of cells whose centroid falls in the missing corner. Returning the
    centroid would hand back a point that was never tested and need not
    solve; the resolver must return one of the cells.
    """
    left = [[0.30, 0.10], [0.40, 0.10], [0.50, 0.10],
            [0.30, 0.20], [0.30, 0.30]]
    p = _survey(str(tmp_path), left, [[-0.30, 0.10]])
    got = NP.resolve("left side", p)["position"][:2]
    assert got in [c for c in left], got


def test_the_front_centre_is_refused_WITH_THE_NUMBER(tmp_path):
    p = _survey(str(tmp_path), [[0.30, 0.10], [0.35, 0.10]],
                [[-0.30, 0.10]])
    with pytest.raises(NP.Unreachable) as e:
        NP.resolve("front centre", p)
    msg = str(e.value)
    assert "0 of 3" in msg          # the cells actually counted
    assert "0.30" in msg            # the nearest reachable x, measured


def test_a_front_centre_that_IS_reachable_would_be_allowed(tmp_path):
    """The refusal is a MEASUREMENT, not a rule.

    If a future re-park made the front centre reachable, this resolver must
    say yes. A hardcoded refusal would outlive the geometry that justified
    it, which is exactly what happened to the text in voice_intent.
    """
    p = _survey(str(tmp_path), [[0.05, 0.10], [0.30, 0.10]], [[-0.30, 0.10]])
    assert NP.places(p)["front centre"]["reachable"] is True


def test_home_is_joint_space_and_carries_no_cartesian_pose(tmp_path):
    p = _survey(str(tmp_path), [[0.30, 0.10]], [[-0.30, 0.10]])
    spec = NP.resolve("home", p)
    assert spec["joint_space"] is True and spec["position"] is None


def test_an_unknown_place_lists_what_it_knows(tmp_path):
    p = _survey(str(tmp_path), [[0.30, 0.10]], [[-0.30, 0.10]])
    with pytest.raises(NP.Unreachable) as e:
        NP.resolve("the moon", p)
    assert "left side" in str(e.value)


# ---- the refusals. no fallback, ever ------------------------------------
def test_a_missing_survey_raises_rather_than_inventing_a_pose(tmp_path):
    with pytest.raises(NP.SurveyUnavailable):
        NP.resolve("left side", os.path.join(str(tmp_path), "nope.json"))


def test_a_grasp_pose_only_survey_is_refused(tmp_path):
    p = _survey(str(tmp_path), [[0.30, 0.10]], [[-0.30, 0.10]],
                full_path=False)
    with pytest.raises(NP.SurveyUnavailable) as e:
        NP.resolve("left side", p)
    assert "GRASP POSE ONLY" in str(e.value)


# ---- parser and resolver must agree about what is reachable -------------
def test_the_parser_and_the_resolver_name_the_same_places(tmp_path):
    """Two tables of place names is how one of them goes stale.

    They are deliberately separate -- the parser is pure text and must test
    without a workspace on disk -- so this is the seam that needs a test.
    """
    p = _survey(str(tmp_path), [[0.30, 0.10]], [[-0.30, 0.10]])
    assert set(vi.NAMED_PLACES) == set(NP.places(p))


def test_move_to_the_front_centre_still_parses_and_still_refuses():
    """The command must be UNDERSTOOD and then declined.

    Failing to parse it would be a different and much worse answer: it would
    say the robot does not know the phrase, when in fact it knows the phrase
    and knows the geometry that forbids it.
    """
    i = vi.parse("robot move to the front centre", wake="robot")
    assert i.verb is None
    assert "front centre" in i.reason
    assert not i.ok


def test_move_to_a_reachable_side_parses_as_goto():
    i = vi.parse("robot move to the left side", wake="robot")
    assert i.verb == "goto" and i.target == "left side" and i.arm == "left"
