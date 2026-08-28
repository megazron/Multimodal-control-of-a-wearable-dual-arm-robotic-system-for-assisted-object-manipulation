# The project presentation

`SRL_project_presentation.pptx` — 48 slides, 16:9, PowerPoint-native (real
text boxes, real tables, no flattened images of slides). Open it in
PowerPoint, Keynote, Google Slides or LibreOffice Impress.

## Rebuild it

```
.venv_vision/bin/python scripts/make_presentation.py     # assets, then the deck
.venv_vision/bin/python scripts/preview_presentation.py  # render it and LOOK at it
```

The first command is idempotent and regenerates everything it can:

| what | from |
| --- | --- |
| data plots | `scripts/make_thesis_figures.py`, `thesis_v2/figures/make_figures.py`, redirected from PDF to PNG |
| result plots | `scripts/make_results.py` → `recordings/analysis/` |
| block diagrams | the thesis TikZ sources, compiled standalone with `xelatex` and cropped to their own ink |
| clip frames | one frame each, cut with `ffmpeg` from the committed verification clips |
| screenshots | copied from `docs/img/`, `thesis_v2/figures/` and `recordings/session/` |

Anything whose source is missing is REFUSED BY NAME on stdout and the slide is
not written. Nothing is drawn from prose.

## Why there is a preview script

There is no LibreOffice on this host, so a built deck cannot be rendered by
the usual route — and a `.pptx` that saved without error can still have a
figure off the slide or a caption sitting on the footer. `preview_presentation.py`
reads back the **saved file** and draws every shape with PIL, one PNG per
slide plus `preview/contact_sheet.png`, and reports any text frame that
overflows its box or any shape past the slide edge.

It is deliberately conservative: this host has no Calibri, so Liberation Sans
is substituted, and it is *wider* at the same point size. Text that fits in
the preview fits in PowerPoint.

Looking at the render is not optional, and it is the same rule as the GUI's
(CLAUDE.md, THE GUI RULE §3). The first build passed with a zero return code
and had seven diagrams missing, four mode diagrams clipped at the A4 page
edge, three metric rows printed over the figure beside them, and a slide of
stock photographs of strangers presented as scene-camera evidence. None of
that is visible from an exit status.

Only `contact_sheet.png` is committed; the per-slide PNGs are regenerable and
are ignored.

## What the deck claims, and what it does not

Every slide names the file it was drawn from, on the slide. Where the
measurement does not support the obvious reading, the slide says so in the
same breath:

* the accuracy figures PREDATE the 2026-08-26 VR smoothing, so they cannot
  be used to rank the modes for smoothness;
* the environment-mapping and autonomy footage is SIMULATED — what is real in
  it is the sweep order, the deprojection, the segmentation, the plane fit,
  the IK and every refusal;
* the only hardware measurement in the deck is the 36-run parking error, and
  the compensation derived from it has not been tried on hardware;
* the voice figures are PARSER scores, never transcription scores;
* no lab photograph of the assembled rig exists in this repository, and the
  one photographic image in the deck is the vendor's product shot, labelled;
* no human data has been collected at all.

Slide 44 is the whole ledger: what is measured with the file committed, and
what is not measured and not claimed.

## Spare figures

`figures/` holds 22 more images than the deck embeds — the kinematic chain,
the degradation ladder, the calibration and session-timeline diagrams, the VR
wrist and protocol plots, the master-versus-VR frames for the same task, the
grip traces, and the sling geometry. They are built by the same command and
are there to be dropped onto a new slide without another asset run. The
figure set is regenerated wholesale, so nothing has to be hunted for twice.

A build reports every asset it could not produce, by name. If a slide shows
`FIGURE REFUSED: <name> is not on disk`, that is the deck telling you an
upstream generator did not run — not a placeholder to be filled in by hand.
