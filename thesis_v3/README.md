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

- **Main body: ~5,995 words, 15 figures/tables** (Introduction, Method,
  Result, Discussion, Conclusion) -- against the booklet's limit of 6,000
  words / 20 figures. Word count is read from the *typeset* PDF (`Chapter 1`
  to the page before `Appendix A`), not estimated from the LaTeX source,
  because `\SI{}{}`/`\cref{}` expand into several rendered words each, and
  figure text (legends, axis labels, tikz annotations) is vector text the
  extractor also reads. Headroom is under 10 words -- trim before adding.
- **Abstract: 243/250 words.**
- **Structure follows the booklet's own logic, not just the template's
  chapter list.** The literature review is now the first section of the
  Introduction (`background/` is gone), so Chapter 1 runs problem ->
  literature -> gap -> aim, objectives and the five hypotheses (H1-H5,
  numbered as in Appendix J), the aim closing the chapter as the booklet
  asks. The Discussion opens with the two-sentence summary the booklet
  asks for and absorbs the objective-by-objective evaluation and the
  prioritised future work (with the number that sets each item); the
  Conclusion is one paragraph. `\listoffigures`/`\listoftables` are in
  (uncounted front matter).
- **"Hand travel" was the robot's path, not the operator's -- corrected
  everywhere.** `ee_path_m` is the end-effector path; the operators' own
  controller paths (`controller_path_m`, also extracted) differ by 2%
  between conditions (29.9 m direct / 30.5 m shared) while the robot's
  differs by 21%, so the extra distance under shared autonomy is the
  assistance layer's. The platform shows the same with no operator
  (`recordings/baselines/mode_difference.json`: identical scripted
  waypoints travel 4% further under VR shared than VR direct, 45% under
  master shared than master direct). Figure labels regenerated to "robot
  path (m)"; every prose occurrence relabelled; the Discussion's earlier
  speculation replaced with this measured account.
- **Other corrections this round:** the modes table now says 0.60 rad/s is
  the simulation default and the hardware bridge caps every mode at 0.15
  (every real-arm launch file); H2 (workload) is stated as untested, since
  no workload instrument was administered (re-grips are control effort, not
  workload); the multimeter task is "never recorded with a participant"
  (scripted no-operator T3 clips exist) and its 195 mm box exceeds the
  85 mm jaw regardless.
- **Added on evidence already in the repo:** the pinned-wrist cost at the
  work point (0.065 m pinned vs 0.450 m free, a factor of seven; App. I),
  the reaction-budget remedies with numbers and the 224 mm optimistic
  fallback (v2 §13.5, verified against `docs/system/23_how_good_is_it.md`),
  the full study's power (97.3% family-wise, 63.8% per named task at
  n=16, 80% needs twenty dyads; App. K), the real-RGB-D open-vocabulary
  detection rate (9.0% of 432 instances) as the justification for the
  geometric fallback, the intent-estimator sweep anchored to Dragan &
  Srinivasa's 16% break-even, the arbitration-rule figure
  (`figures/diagrams/arbitration.tex`, previously unused), and four
  literature anchors for the performance/agency trade (Dragan & Srinivasa
  2012, Kim et al. 2012, You & Hauser 2011, Collier et al. 2025 -- bib
  entries transcribed from `docs/research/01_literature_review.md`'s
  verified reference list), plus zero-word citations for TRAC-IK, RANSAC
  and two further SRL reviews.
- **65 pages total.** Channel health ("pot health") and the degradation
  architecture discussion are now moved to the appendix **in full** -- not
  just trimmed: the main body carries only "six of fourteen channels are
  alive" and a pointer to `sec:channelhealth`/`sec:degradation-appendix`;
  `fig:channelhealth` (the per-channel bar chart) and the whole "what the
  architecture bought" discussion section no longer appear in the graded
  chapters at all. That, plus tighter prose elsewhere, paid for: a **second**
  three-way 3D trajectory (the master-arm cohort's own version, alongside
  the VR one, both master hand / simulated EE / real EE from live forward
  kinematics on encoders -- previously only the VR cohort had one); a
  decluttered redraw of the VR trajectory (the master trace smoothed and
  thinned since it is the noisy raw input, not a measured result -- the
  sim/real traces are untouched); box plots (median/IQR/whiskers/outliers)
  replacing the bar+scatter pooled comparisons for time-to-finish, hand
  travel and re-grips, so the real variance is visible instead of only a
  mean bar; a depth-camera figure with each object's bounding box annotated
  with its own real measured distance, beside the raw frame for context;
  and a compact values table for the vision section (support-plane RMS and
  inlier fraction, both objects' plane-fit and depth-box ranges, the box's
  footprint). 14 of the 20 permitted figures/tables, leaving 6 spare -- the
  mannequin/table/arms/objects bounding-box figure stays in `app:vision`
  (`ch:vision`) to protect the word budget, referenced from the main body.
  The appendix carries everything else, uncounted.
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
`make_3d_trajectory_full.py`/`_full_v2.py`/`_master.py` (the three-way
master/sim/real trajectories, `fig:track-3d` and `fig:track-3d-master`) and
`make_depth_boxes.py` all need `scripts/sim_session.py --stack teleop
--keep-up` running first for the FK-based ones, since the real-EE trace is
FK'd live through `/compute_fk` -- neither cohort's CSV has a recorded
Cartesian real-EE column (`ee_left_x/y/z` in the VR cohort's `trail.csv` is
FK of the *simulated* joint state, not the real one, confirmed by reading
`scripts/record_all.py`; the master-arm cohort's `trail_extracted.csv` has
no `ee_*` column at all -- both simulated and real EE are FK'd for that
cohort). `_full_v2.py` additionally smooths and thins the master trace only
(never the sim/real traces, which must stay exact) for legibility.
`make_pilot_figures.py`'s `grouped_box_panel()` builds the three pooled box
plots (`pilot_time_box.pdf`, `pilot_distance_box.pdf`,
`pilot_regrips_box.pdf`) from the same `by_task_condition()` data as the
original bar charts, which it does not modify. The vision-pipeline,
depth-boxes and scene-bounding-box figures (`vision/cv_pipeline.pdf`,
`vision/depth_boxes.pdf`, `vision/scene_boxes.pdf`) all read the same raw
frame pair at
`recordings/vision_thesis/20260830_073141/raw/left_gripper/{colour.png,depth.npy}`
(depth: float32 metres, 0 = no return) and
`scripts/srl_object_detector.py`'s own `blobs()`/`PALETTE` (cube) plus a
measured-hue threshold on the same toolchain (box, which the palette
doesn't cover); `cv_values_table.tex`'s numbers come from that same frame's
own pipeline stage logs under
`recordings/vision_thesis/20260830_073141/data/left_gripper/`. The four
scene classes without a detector in this project (mannequin, table, arm,
objects -- `scene_boxes.pdf`) are hand-identified against a pixel grid on
that exact frame, stated as such in its caption rather than presented as
automatic detections; `scene_boxes.pdf` itself still has no standalone
regeneration script. All scripts re-derive participant anonymisation from
the raw session names before any plot is drawn, and every figure's
extracted PDF text is grepped for real names (word-boundary, not substring
-- `wen` inside `when` is a false positive this project has hit) before it
is committed.

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
