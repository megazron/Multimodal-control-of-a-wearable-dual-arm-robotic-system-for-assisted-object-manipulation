# Final report

Built on the department's own **"Imperial College Individual Project
Template_LaTeX"** (title page, document class, page geometry, chapter order,
bibliography style) rather than the custom template `thesis_v2/` used.
Directory names match the template's own: `introduction/`, `background/`,
`method/`, `result/`, `discussion/`, `conclusion/`, `appendix/`, `title/`,
`bibs/`.

**Compiler: XeLaTeX** (the template's own `fontspec` / Times New Roman
request needs it):

    xelatex main && bibtex main && xelatex main && xelatex main

## Current state

- **Main body: 5,993 words, 12 figures/tables** (Introduction, Background,
  Method, Result, Discussion, Conclusion) -- against the booklet's limit of
  6,000 words / 20 figures. Word count is read from the *typeset* PDF
  (`Chapter 1` to the page before `Appendix A`), not estimated from the
  LaTeX source, because `\SI{}{}`/`\cref{}` expand into several rendered
  words each, and figure text (legends, axis labels) is vector text the
  extractor also reads. Headroom is now ~7 words -- trim before adding.
- **Abstract: 248/250 words.**
- **62 pages total.** Channel health ("pot health") prose was cut back to
  its one headline number in the main body, per direct feedback that it was
  spending graded words on the least central finding; the regression detail
  survives in `ch:results`. That freed room for: a genuine three-way 3D
  trajectory (master hand, simulated EE, and the **real robot's own EE from
  live forward kinematics on its encoders**, not simulated), with picked
  and placed marked from the gripper's logged transitions, for one Direct
  and one Shared-autonomy session; a clean raw-to-detection computer-vision
  pipeline figure replacing the two dashboard-style PNGs; a written
  explanation of the three pilot tasks and the never-recorded fourth
  (multimeter); and a real safety finding from the pilot's own `/blocking`
  topic (below). 12 of the 20 permitted figures/tables, leaving 8 spare --
  a bounding-box scene figure (mannequin/table/arms/objects) moved to
  `app:vision` (`ch:vision`) to protect the word budget instead, referenced
  from the main body. The appendix carries everything else, uncounted.
- **A real safety finding, not a synthetic one**: across all 26 recorded
  pilot-era sessions' `/blocking` topic, the geometric clearance floor
  (`clearance_floor`, the mechanism that would hold an arm 150 mm short of
  the wearer) **never activated once** -- no recorded operator drove either
  arm inside that margin. The manual/dead-man e-stop *did* fire for real:
  four holds across three sessions, one tripping twice, the first held
  22.6 s mid-session (not simply an end-of-recording gesture, which the
  table caption previously claimed for all of them -- corrected).
  Regenerate the scan: `rosbag2_py.SequentialCompressionReader` over each
  session's `bag_0.mcap.zstd`, watching `/blocking`'s per-unit `blockers[]`
  for a `false -> true` edge on `clearance_floor` and on `estop`.
- **Figures are matplotlib PDFs in the same house style as the platform's
  own data figures** (`figures/make_figures.py`'s `channels()`/`reach()`/
  `clearance()`: muted grey/blue/red, `font.size` 8-9, no display title
  baked into the image -- the LaTeX caption carries that -- collages built
  from small individual panels via `subcaption`).

## The pilot data

**Five** VR operators, not four -- P5 was missed on the first pass. Their
seven sessions (2026-09-02) predate the per-participant folder-naming
convention (generic timestamp names, no task label) and were found by
noticing the untouched generic `session/` directories actually carried
real VR trial data with a *working* `shared_autonomy` flag in
`events.jsonl`, unlike the later, better-named sessions. `parse_label()` in
the extraction script now special-cases P5 and reads condition from that
flag rather than a folder name.

Two participant cohorts, both anonymised (P1-P5 VR, M1-M3 master-arm; never
a real name, including in rendered figure pixels -- grepped in every figure
PDF's extracted text before each commit, after one script briefly put real
first names in a legend before this check caught it).

- **P1-P5**, VR controllers, 23 sessions ($n=13$ direct / $n=10$ shared),
  `sec:pilot-results` (Result) + `app:pilot` (Appendix L,
  `appendix/l_pilot_study.tex`). Pooled and paired: shorter completion
  times and fewer re-grips under shared autonomy, but **more hand travel**,
  a direction that holds in both the pooled and the paired comparison --
  stated plainly in Discussion rather than smoothed over.
- **M1-M3**, the instrumented mannequin master, 9 recorded sessions out of
  16 attempted -- the other 7 are confirmed empty or metadata-less via
  `ros2 bag info`, not an extraction gap. M1's three sessions track the
  simulated joint 15.9-21.2° RMS against M2/M3's under 3°, an order of
  magnitude worse and consistent with this project's own channel-health
  finding.
- Every pilot session polled the real Kinova arms' own joint encoders
  alongside the simulated ones, a live sim-to-real check rather than an
  offline replay: the best-tracking session holds 1.06° RMS, shown as a
  genuine 3D trajectory plot (`figures/make_3d_trajectory.py`), not a 2D
  projection.

Regenerate: `make_pilot_figures.py`, `make_3d_trajectory.py`,
`make_tracking_figure.py`, `make_master_figure.py`, each
`python3 thesis_v3/figures/<script>.py` from the repository root.
`make_3d_trajectory_full.py` (the three-way master/sim/real trajectory,
`fig:track-3d`) additionally needs `scripts/sim_session.py --stack teleop
--keep-up` running first, since the real-EE trace is FK'd live through
`/compute_fk` -- `real_left_jN` in `trail.csv` has no recorded Cartesian
counterpart (`ee_left_x/y/z` is FK of the *simulated* joint state, not the
real one; confirmed by reading `scripts/record_all.py`). The vision-pipeline
and scene-bounding-box figures (`vision/cv_pipeline.pdf`,
`vision/scene_boxes.pdf`) were built ad hoc from a raw frame pair at
`recordings/vision_thesis/20260830_073141/raw/{left_gripper,scene_hd}/` and
`scripts/srl_object_detector.py`'s own `blobs()`/`PALETTE` (cube) plus a
measured-hue threshold on the same toolchain (box, which the palette
doesn't cover); the four scene classes without a detector in this project
(mannequin, table, arm, table -- `scene_boxes.pdf`) are hand-identified
against a pixel grid on that exact frame, stated as such in its caption
rather than presented as automatic detections. No standalone regeneration
script exists yet for either -- write one from those sources before editing
them again. All scripts re-derive participant anonymisation from the raw
session names before any
plot is drawn -- if the pilot data set grows, extend each script's `ANON`
dict rather than plotting first and anonymising after.

## Checking compliance before submission

    python3 - <<'EOF'
    import pymupdf
    doc = pymupdf.open("main.pdf")
    texts = [p.get_text() for p in doc]
    b1 = next(i for i,t in enumerate(texts) if t.strip().startswith("Chapter 1"))
    b2 = next(i for i,t in enumerate(texts) if t.strip().startswith("Appendix A"))
    print(len(" ".join(texts[b1:b2]).split()), "main body words")
    EOF

Re-run after any edit to `introduction/`, `background/`, `method/`,
`result/`, `discussion/` or `conclusion/`: 6,000 words and 20 figures/tables
are department policy, not house style. This report currently has ~30
words of headroom on the word count and 8 spare figure/table slots --
spend the slots before the words if you add more evidence.
