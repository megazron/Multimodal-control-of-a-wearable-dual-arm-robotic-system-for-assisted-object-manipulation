# When the instrument is the fault

Sixteen occasions on this project when a measurement was wrong and the system
was not. Each is given as: what it looked like, what it actually was, and how
it was caught.

This is a methods contribution rather than a defect list. Most write-ups treat
measurement as transparent — the apparatus reports, and the report is the
finding. On this project that assumption failed **sixteen times**, eleven of
them in a single run of work, and in every case the wrong reading was
*plausible*. None announced itself. Several would have entered the record as
results if a second check had not contradicted them.

## The rule that came out of it

> **A surprising failure is evidence about the instrument until the instrument
> has been cleared.**

With four corollaries, each of which was learned from one of the cases below:

1. Every analysis script gets a **known-answer test** — an input whose answer
   is known independently, checked before its output is believed.
2. Prefer **real recorded data** with independently established properties.
3. Use synthetic data only where the ground truth is **constructed**
   (arithmetic, geometry), never where it is **rendered** (images, physics).
4. A result that **contradicts an earlier measurement** is an instrument check,
   not a finding, until the two are reconciled.

---

## The sixteen

### 1. A noise floor of exactly zero on all fourteen channels
**Looked like:** every potentiometer perfectly quiet — impossible for an
analog channel, and briefly read as evidence the channels were dead.
**Actually was:** the recorder sampled at \SI{50}{\hertz} a sensor that
updates at \SIrange{14.7}{17}{\hertz}, so \SIrange{82}{88}{\percent} of
adjacent rows were bit-identical and every row-wise statistic was dominated by
duplicates.
**Caught by:** the figure being *too* clean. Recomputing over distinct updates
gave a real floor of \SIrange{0.2}{5.4}{\degree}.
**Cost:** this artefact was the basis of the "spread exactly 0.0 means dead"
rule used for months, which misclassified a healthy channel.

### 2. Isotropy swinging by a factor of two between runs
**Looked like:** an unrepeatable measurement of the workspace.
**Actually was:** the *statistic*, not the measurement. Min-over-directions of
a single sweep is maximally sensitive to one rare tail failure.
**Caught by:** repeating to N=10, where the inter-quartile range came out at
\SI{0.000}{\metre} on every direction. On medians it is stable.

### 3. Object detection at 0–4 %
**Looked like:** the detector failing on the task objects.
**Actually was:** the renderer. Flat-shaded primitives on a flat background
are outside the detector's training distribution.
**Caught by:** running the same model on a real photograph, where it scored
\numrange{0.89}{0.91}.

### 4. A carry path failing 0 of 19 placements
**Looked like:** the task geometry being infeasible.
**Actually was:** the path template carrying toward the wearer, the one
direction that leaves the feasible band.
**Caught by:** contradiction with an earlier result showing 19 working grip
pairs.

### 5. An AprilTag renderer producing a mirrored tag
**Looked like:** 0 detections at every distance.
**Actually was:** OpenCV's camera has +y down, so pairing object corners with
the marker image's own winding reversed it. AprilTag decoding is not
mirror-invariant.
**Caught by:** a vertical flip of the same crop decoding immediately.

### 6. A ground-truth rotation 180° out
**Looked like:** a flat \SI{22}{\degree} pose error resembling the planar-pose
ambiguity.
**Actually was:** the correction applied in the wrong frame. The two forms
**commute** for a tilt about x or y, so checking those axes could not tell them
apart.
**Caught by:** testing a general in-plane axis: \SI{21.79}{\degree} wrong form
against \SI{1.06}{\degree} right form.

### 7. A leftover-process check matching its own heredoc
**Looked like:** 2–3 stack processes surviving a clean shutdown.
**Actually was:** the check grepped for bare names, and its own command line
contained them.
**Caught by:** matching installed executable paths instead, which a shell
command line cannot contain.

### 8. A GUI test that passed on broken code
**Looked like:** the terminal console launching correctly.
**Actually was:** the test drove it through a pty and set `TERM=xterm`,
constructing the one environment where the bug cannot occur, then checked only
for panel captions — never the exit code.
**Caught by:** running the real entry point and asserting on exit status.

---

*The nine below are from the run of work in August 2026.*

### 9. A gripper model apparently 52 mm wrong
**Looked like:** the gripper never closing below \SI{50.7}{\milli\metre}, and
the shipped linear model wrong by \SI{52}{\milli\metre}.
**Actually was:** the measurement took the gap between finger-tip **link
origins**, which sit behind the pad faces.
**Caught by:** checking against the vendor's published \SI{85}{\milli\metre}
stroke. Subtracting the fully-closed origin gap gave \SI{84.84}{\milli\metre},
a \SI{0.16}{\milli\metre} agreement, and the model was then accurate to
\SI{2.74}{\milli\metre}.
**Nearly reported.** This was one command away from entering the record.

### 10. A wrist rotation of 0.0° (twice, from two different causes)
**Looked like:** the grasp orientation already matching the pinned anchor, so
no alignment needed — which would have removed the thesis's central capability
finding.
**Actually was:** first, reading the anchor *after* the test had already moved
the arm onto the grasp pose, so it compared the pose with itself; then, after
fixing that, reading it at start-up when a **previous run** had left the arms
parked on the grasp pose.
**Caught by:** contradiction with an independently known \SI{169.7}{\degree}.
The script now homes the arms and **refuses to measure** if they are more than
\SI{0.05}{\radian} off.

### 11. A known-answer check that passed on no data
**Looked like:** the capability-ladder harness validating cleanly.
**Actually was:** it parsed **zero** rows (wrong column names), and `all()`
over an empty collection is `True`.
**Caught by:** the output table being empty while the check said PASS.
**The general form:** a known-answer test that passes when there is no data is
worse than no test, because it converts absence into confirmation.

### 12. An "incoherent" fault the node correctly ignored
**Looked like:** the channel-health classifier missing an injected fault.
**Actually was:** the injected fault alternated 350°↔20°, which is a
**30° circular** jump — correctly below the 60° threshold. The node was right
and the test data was wrong.
**Caught by:** asking why only one of two injected faults was detected.

### 13. A black screenshot of a working window
**Looked like:** RViz failing to render inside the Qt container.
**Actually was:** the capture fired *after* the probe's window had closed.
**Caught by:** re-capturing during the window's lifetime.
**Note:** `isVisible()` returned `True` throughout — a container can be visible
and empty, so only the pixels settled it.

### 14. An empty sweep read as an empty scene
**Looked like:** the scene fingerprint reporting every object vanished.
**Actually was:** the node began its sweep before DDS discovery had matched
the detector, received **zero** messages, and concluded everything had gone —
silently emptying the stored fingerprint.
**Caught by:** running it twice in a row on an unchanged scene.
**The general form is the same as (11):** no data is not a negative result.

### 15. A boundary probe reporting a wall that was not there
**Looked like:** the boundary node reporting \SI{0}{\milli\metre} remaining
along a line a direct probe reached at 6 of 6.
**Actually was:** the test commanded an **identity orientation**, which is
unreachable at 0 of 6 points on that line. The node was answering the question
actually asked. After fixing one test file, a **second** test file still had
it.
**Caught by:** instrumenting the node's own probe trace, which showed it
probing correctly and finding every point reachable.
**Ruled out first, before changing any code:** the IK request format, by A/B
over seed, link name and timeout — all six configurations returned 6 of 6.

### 16. `pkill -f` killing the shell that ran it
**Looked like:** test harnesses dying mid-run with no output, repeatedly.
**Actually was:** the pattern matched the invoking shell's own command line.
**Caught by:** the exit code 144 and empty logs.
**Documented in `docs/ENGINEERING_LOG.md` before this run, and hit anyway**, three times —
which is itself the finding: a documented trap is not a solved trap.

---

## What the pattern is

Sorting the sixteen by mechanism gives four recurring shapes, and they are not
specific to robotics:

| shape | cases | why it recurs |
| --- | --- | --- |
| **Absence read as a value** | 11, 14, and 1 | "no data" and "a measurement of zero" are different, and most code paths conflate them |
| **The harness constructs the condition it tests** | 8, 3, 5, 12 | a test written from the same understanding as the code inherits its errors |
| **State left over from a previous run** | 10, 15, and the workspace runs at 0.87 rad off home | scripts are written as if they start from nothing, and they do not |
| **Order of operations inside the measurement** | 10, 13 | reading a reference after perturbing it is easy to do and invisible afterwards |

The defence that worked most often was not care. It was **redundancy**: an
independently known quantity to check against (the vendor's stroke, a
published detection score, an earlier incompatible result). Nine of the
sixteen were caught that way. Only three were caught by inspection.

## What this means for the thesis

Every quantitative claim in this work carries a maturity label and a
traceable measurement, and `docs/system/09_results_audit.md` records for each
whether its instrument was validated. Of thirty-four figures audited, fifteen
are `VALIDATED` against an independently known answer, fifteen are `SOUND`
arithmetic with no instrument to mis-calibrate, three are `UNVERIFIED` and are
not reported as results, and one is `STRUCTURAL` — a quantity that cannot
exist in the available environment.

**One capability was cut rather than shipped** on this basis: the
boundary-warning node produced a correct \SI{88}{\milli\metre} lead in 3 of 5
runs and a false alarm in the other 2, with the cause unexplained after all
candidate mechanisms were eliminated. A warning that is wrong teaches the
operator to ignore it, which spends the credibility every other alarm in the
system depends on.
