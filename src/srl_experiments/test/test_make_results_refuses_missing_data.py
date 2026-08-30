#!/usr/bin/env python3
"""make_results must refuse a missing recording BY NAME, never draw around it.

The repeated fault this repo guards against is an instrument that cannot fail:
a figure builder pointed at nothing that still returns a figure is exactly
that. So the known answers here are constructed the hard way -- an EMPTY root,
where every figure must come back as a named skip and zero PNGs; and a root
holding exactly one committed-shaped file, where exactly the figures that file
supports appear and the index lists them and nothing else.
"""
import json
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                    "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import make_results as MR                                     # noqa: E402


def test_an_empty_root_yields_a_named_skip_for_every_figure(tmp_path):
    made, skipped = MR.build(root=tmp_path, outdir=tmp_path / "out")
    assert made == [], "figures were drawn from a root holding NO recordings"
    assert {k for k, _ in skipped} == set(MR.FIGS), (
        "every figure must refuse; missing: %s"
        % (set(MR.FIGS) - {k for k, _ in skipped}))
    for key, reason in skipped:
        assert reason.strip(), "%s skipped with an empty reason" % key
    # the file-backed refusals must NAME what is missing
    reasons = dict(skipped)
    assert "accuracy_table.json" in reasons["accuracy"]
    assert "teleop_motion.json" in reasons["teleop_motion"]
    assert "mode_difference.json" in reasons["mode_paths"]
    # and no stray PNG exists
    pngs = list((tmp_path / "out").glob("*.png"))
    assert pngs == [], "PNGs written despite universal refusal: %s" % pngs


def test_the_index_lists_exactly_the_figures_produced(tmp_path):
    vroot = tmp_path / "recordings" / "verification"
    vroot.mkdir(parents=True)
    (vroot / "accuracy_table.json").write_text(json.dumps({
        "01_master_teleop": [{"task": "t3", "n": 2, "grasped": 2,
                              "pos_err": [0.0, 0.0], "place_err": [],
                              "smallest": 30.0}]}))
    out = tmp_path / "out"
    made, skipped = MR.build(root=tmp_path, outdir=out)
    made_keys = {k for k, _, _ in made}
    assert made_keys == {"accuracy", "grasp_matrix"}, (
        "one accuracy table supports exactly two figures, got %s" % made_keys)
    for _, png, _ in made:
        assert (out / png).exists(), "%s listed as built but absent" % png
    index = (out / "index.md").read_text()
    fig_lines = [ln for ln in index.splitlines() if ln.startswith("* **")]
    listed = {ln.split("**")[1] for ln in fig_lines}
    assert listed == made_keys | {k for k, _ in skipped}, (
        "the index must list exactly what was produced plus what refused")
    # produced and refused are in separate sections, produced first
    assert index.index("## Figures") < index.index("## Refused")
    for key, _ in skipped:
        assert key in index.split("## Refused")[1], (
            "%s refused but not listed under Refused" % key)


def test_a_partial_only_run_does_not_shrink_the_index(tmp_path):
    vroot = tmp_path / "recordings" / "verification"
    vroot.mkdir(parents=True)
    (vroot / "accuracy_table.json").write_text(json.dumps({
        "02_vr_teleop": [{"task": "t3", "n": 2, "grasped": 2,
                          "pos_err": [0.0, 0.0], "place_err": [],
                          "smallest": 30.0}]}))
    out = tmp_path / "out"
    MR.build(root=tmp_path, outdir=out)
    before = (out / "index.md").read_text()
    MR.build(root=tmp_path, outdir=out, only=["accuracy"])
    after = (out / "index.md").read_text()
    assert before == after, (
        "an --only run rewrote index.md down to its own subset")
