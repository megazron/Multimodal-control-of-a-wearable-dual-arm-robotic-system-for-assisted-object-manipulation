# Final report

Built on the department's own **"Imperial College Individual Project
Template_LaTeX"** (title page, document class, page geometry, chapter order,
bibliography style) rather than the custom template `extras/thesis/thesis_v2/` used.
Directory names match the template's own: `introduction/`, `background/`,
`method/`, `result/`, `discussion/`, `conclusion/`, `appendix/`, `title/`,
`bibs/`.

**Compiler: XeLaTeX** (the template's own `fontspec` / Times New Roman
request needs it):

    xelatex main && bibtex main && xelatex main && xelatex main

**Template conformance (2026-09-07).** `main.tex` and `title/title.tex` are
the template's own files with only the fill-in slots filled (title, author,
supervisors, abstract, acknowledgements): the template's package list,
encodings, `\titleformat` (which prints appendices as "Chapter A", as the
template does), `\setmainfont{Times New Roman}`, `\today` date, commented-out
lists of figures/tables, and the EPS logo via the template's own
`\includegraphics{title/logo.eps}` line. Chapter files are the template's
seven, including a separate `background/background.tex`. The packages the
content needs (siunitx, booktabs, subcaption, tikz, cleveref, ...) are added
after the template's list and change nothing the template sets. The one
deliberate deviation is `\bibliographystyle{unsrt}` for numbered citations
(template default: `alpha`); switch the word back to `alpha` to undo it.

Local build prerequisites for a faithful build (Overleaf has both):
Times New Roman (the four Windows `times*.ttf` copied into
`~/.local/share/fonts`, then `fc-cache -f`) and Ghostscript on `PATH` for
the EPS logo (here: Ubuntu's `.deb`s unpacked into `~/.local/gs`, wrapper at
`~/.local/bin/gs`).

**The pilot ran TWO tasks, not three (corrected 2026-09-07).** The session
folders were named at the console and mixed the names up: "object tracking"
folders were the pick-and-place task; "position matching/reaching" and
"target reaching" were one target-reaching task. The operator confirmed the
mapping. `figures/relabel_pilot_tasks.py` applies it to the derived data
files in `figures/pilot/data/` (now kept in the repo, ANONYMISED by
`figures/anonymise_pilot_data.py`: participant codes only, sessions named by
their timestamp key, resolved back to a folder by `figures/session_paths.py`;
the name-to-code book lives outside the repo in
`recordings/sessions/participant_codes.json`). Paired comparisons were
recomputed (robot path higher with assistance 5 of 7, was 6 of 7; operator
hand 3 of 7, was 2 of 7). Pick-and-place OUTCOMES come from the gripper
record: `figures/extract_pilot_gripper.py` reads `/real/gripper_<hand>` from
every VR bag (measured finger position) and `figures/make_gripper_table.py`
sets it beside the controller trigger passed through the live node's own
latch -- three operators held the gripper closed (P1, P2, P5), one closed it
once briefly (P3), one never (P4); the operator's account was three
pick-and-places, one lift, one reach. Table `tab:gripper-use` in Appendix L.

## Current state

- **Latest round (page-by-page check; CV section fixed):** every main-body
  page rendered and inspected. Fixes: the title-page word count was stale
  (now set from the typeset count at build time); the CV section's second
  figure repeated the flowchart's stages, so it is replaced by what each of
  the four cameras actually detected (plane found per camera: 74 / 53 /
  3.8 %) beside the learned detector's real-frame recall (9.0 % of 432,
  most classes never); the flowchart is now 3x2 with larger panels; and the
  values table summarises detection (plane per camera, objects found,
  graspable, cube and box range/footprint, detector recall) instead of two
  distance methods -- the earlier "depth-box median" ranges were farther
  than the plane-fit ranges because the 2-D box contains table pixels
  behind the object, not because of a tilted object; that wrong explanation
  is gone.
- **Previous round (the three experiments, readable trajectories):** Methods
  §The pilot study now describes the three experiments as run -- common
  setup (operator across the room, controller in each hand, robot follows
  only while the clutch is held, direct then assisted in the same sitting,
  no time limit, both buttons end the recording) and `tab:tasks` with what
  the operator did, what was measured, sessions direct/assisted and mean
  duration per task, from `vr_study_sessions.json`. No written task protocol
  exists in the repository, so the descriptions are what the labels and the
  logs support; the table is the 20th figure/table. The 3-D trajectory
  figure ("random drawn lines") is replaced by position-against-time panels
  (`figures/make_trajectory_timeseries.py`): three lines (left-right,
  forward-back, up-down) per panel, (a) master arm / operator's hand, (b)
  simulated robot's hand, (c) real robot's hand, one row per cohort; (b) and
  (c) share a scale so their match is visible at a glance.
- **Previous round (flowcharts, formulas, trajectories as a/b/c):** Methods
  now carries eight numbered equations in the main body (sensor smoothing,
  the direction-and-distance pose mapping, the clutch anchor, the
  assistance rule as a case expression, the clearance-floor test, the
  RANSAC plane criterion, the grasp-width rule, the shared-timing motion
  generator, the two sim-to-real models and the error-budget combination)
  and two new flowcharts drawn from the implementation
  (`figures/diagrams/signal_path.tex` -- operator input to real arm with
  every stopping point in red -- and `pilot_procedure.tex` -- one pilot
  session as run and the log every metric came from); the package
  dependency graph moved to Appendix C. The two 3-D trajectory figures are
  replaced by one figure of three separate panels per cohort -- (a) master
  arm / operator's hand, (b) simulated robot's hand (RViz), (c) real robot's
  hand from its own encoders -- for a VR session and a mannequin session
  (`figures/make_trajectory_abc.py`, from the gallery's FK'd data). The CV
  figure is now a six-stage flowchart of real stage images on one frame
  (`figures/make_cv_flowchart.py`; the recomputed plane-inlier fraction
  matches the pipeline's stored 73.95% exactly). Two tikz style names
  (`in`, `step`) collided with built-in tikz keys and were renamed.
- **Read against the booklet itself** (`MSc_Project_booklet_2025-26.pdf`,
  28 pages, extracted to text) and the department template zip (identical
  in structure to what this report is built on). Two things the rubric
  wants that the report was short on are now in: (1) the **ethics section
  is marked separately** and wants the six domains (scientific integrity,
  collegiality, human subjects, animal welfare, institutional integrity,
  social responsibility) and every stakeholder -- `method/method.tex`
  §Ethics now covers them in brief and `appendix/p_ethics.tex` in full,
  grounded in `docs/research/03_ethics_and_safety.md`; (2) the A* criteria
  ask for accurate terminology written "for third parties who may not be
  experts", so every main chapter had a **plain-language pass**: each
  technical term is defined in ordinary words at first use (clutch, forward
  kinematics, RANSAC, watchdog, terminal offset...), idioms replaced with
  direct statements, long dash-chained sentences split. Booklet also
  requires one file under 25 MB: `main.pdf` is ~6.7 MB.
- **Main body: ~5,955 words, 19 figures/tables** (Introduction, Method,
  Result, Discussion, Conclusion) -- against the booklet's limit of 6,000
  words / 20 figures.
- **All the trajectory graphs exist** (`figures/gallery/trajectories/`, 74
  figures, `make_trajectory_gallery.py`): every one of the 32 sessions as
  a 3-D panel (operator's hand / simulated robot hand / real robot hand
  from its own encoders via `/compute_fk`, 0 failures), per-participant
  grids, top and side views, gap-over-time traces, a best-session
  composite per operator, and a ranked all-sessions gap summary. The
  summary replaced the joint-angle tracking figure in the main body (it
  says the same thing in millimetres for every session, plainly);
  Appendix O.4 carries the composite, all eight 3-D grids and all eight
  gap-over-time figures. Cartesian sim-vs-real gap: VR median 11.8 mm
  (3.5-168.4), master cohort 24.8 mm (4.7-38.8); the 168 mm outlier is the
  P5 assisted session with the two mid-session e-stops. Word count is read from the *typeset* PDF (`Chapter 1`
  to the page before `Appendix A`), not estimated from the LaTeX source,
  because `\SI{}{}`/`\cref{}` expand into several rendered words each, and
  figure text (legends, axis labels, tikz annotations) in *vector* figures
  is text the extractor also reads. The five gallery figures added to the
  main body in the last round are included as 300-dpi rasters
  (`*_300.png`) for that reason: their label text alone was ~670 extracted
  "words". Headroom is under 10 words -- trim before adding.
- **Abstract: 243/250 words.**
- **A figure gallery from every data source** (`figures/gallery/{pilot,
  baselines,verification,vision,process}/`, ~90 figures, each `.pdf` +
  `.png`, one `INDEX.md` per directory with source, plain-English
  takeaway, numbers and a MAIN/APPENDIX/SKIP verdict, and one
  `make_*_gallery.py` per directory). Appendix O carries the appendix-grade
  ones with plain captions; five went into the main body: the e-stop
  timeline of every session, the grasp-success grid + positioning error,
  the pick-and-place frame collage (simulation renders -- there are no
  photographs of a physical arm doing a task, none exist on disk), and the
  paired slopes. Labels are plain words throughout (the user's request:
  "no jargon graphs").
- **Corrections the gallery pass forced on the text:**
  - The e-stop count. The first `/blocking` scan only opened bags in a
    `bag/` subfolder (12 sessions). All 23 VR sessions now scanned (28 bags
    incl. uncompressed `bag_0.mcap`): clearance floor still 0 activations;
    e-stop **12 holds across 11 sessions** (events.jsonl agrees exactly),
    9 in a session's last 15%, 3 not. Was "four holds across three
    sessions" -- fixed in Results, Conclusion, Abstract.
  - "Picked/placed" markers in `fig:track-3d` were clutch events:
    `vr_*_grip` equals `vr_*_engaged` in 99.84% of samples and `grip_cmd_*`
    is empty in every session -- the gripper was never commanded in the
    pilot. Legend regenerated ("first clutch engage/release"), caption
    says so.
  - Intent estimator: 35% wrong at one cube pitch (60 mm), 47.5% at two;
    "50% at one pitch" was the two-pitch figure. Fixed.
  - Pinned-wrist cost: free reach from the work point is 0.363 m
    (`orientation_cost*.json`, App. I), not 0.450 (that is the right arm's
    innermost column) -- "over five times", not "a factor of seven". Fixed
    in Results and Abstract.
  - The T3 box "exceeds the jaw regardless" clause removed: the scripted
    T3 clips grasp 2 of 2 in every mode.
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
`python3 extras/thesis/thesis_v3/figures/<script>.py` from the repository root.
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
`recordings/vision_thesis/20260830_073141/data/left_gripper/`. `scene_boxes.pdf`
(F.3) is, since 2026-09-07, genuine detector output: YOLO-World
(`yolov8s-worldv2.pt`, CPU, `.venv_vision`) on the `scene_hd` frame with the
prompts mannequin/table/robot arm/box/cube at conf 0.10, on a CROP of the
frame (the mannequin, arms and table; the lab's fixed robots at the sides
excluded, `CROP` in the script), drawn by `figures/make_scene_boxes_yolo.py`
with the raw detections kept in `scene_boxes_yolo.json`. It finds the
mannequin (0.31) and the cardboard box on the table (0.45), misses the
table, both arms and every cube, and labels the two tripods "box" -- the
caption says so. The earlier hand-drawn version is kept as
`scene_boxes_handdrawn_superseded.pdf` and is referenced by nothing. All scripts re-derive participant anonymisation from
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
