# `20260830_073141_upright` — the canonical vision run

The capture of `20260830_073141`, replayed through `--replay` after the scene
camera's orientation was corrected (see `../ANALYSIS.md`). Same frames, same
stages, **96 of 132 stages produced a result and 0 crashed**; the scene camera
is now the right way up and its intrinsics are the device's own.

## What is committed here

| | |
| --- | --- |
| `raw/` | the frames every figure is drawn from: colour PNG, depth as float32 metres in a `.npy`, and the intrinsics. **This is the evidence.** |
| `data/` | every stage's numbers as JSON, plus a CSV per object table |
| `measurements.csv` | all 723 measurements, every one with a unit |
| `manifest.json` | provenance, per-stage status, timings, refusal reasons |
| `report.md` | every stage with its formula and its numbers |

## What is NOT committed, and why

`stages/` (153 MB of per-stage figures) and `sheet_*.png` (73 MB of contact
sheets) are **derived**, and this repository's `.git` is already 3.5 GB. They
are reproducible exactly, with no camera and no laboratory, from the raw
frames above:

    .venv_vision/bin/python scripts/cv_pickpose_visuals.py \
        --replay recordings/vision_thesis/20260830_073141_upright \
        --out /tmp/vision_figures

The ten figures the thesis actually prints are committed, at
`thesis_v2/figures/vision/`.
