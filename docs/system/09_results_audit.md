# Audit of every number reported in Parts 1-8

For each figure: how it was measured, whether the instrument was validated
against a known answer, and the verdict. **Anything that cannot be traced to a
validated measurement is marked UNVERIFIED rather than reported as a result.**

The reason this audit exists: across this project the measuring tool has been
the fault **eight** times, and three of those were in this run of work. The
base rate is high enough that an unaudited number is not evidence.

Legend
`VALIDATED` instrument checked against an independently known answer ·
`SOUND` arithmetic or graph inspection with no instrument to mis-calibrate ·
`UNVERIFIED` measured, but the instrument was not cleared ·
`STRUCTURAL` the quantity cannot exist in this environment

---

## Part 1 — is the grasp real?

| figure | how measured | instrument validated? | verdict |
| --- | --- | --- | --- |
| aperture 84.84 mm at full open | commanded knuckle angle, both finger-tip frames from TF, minus the fully-closed origin gap | **yes** — lands on the vendor's published 85 mm stroke, 0.16 mm agreement | VALIDATED |
| `grip_for()` error 1.64 mm mean / 2.74 max | same curve vs the linear model | yes, same check | VALIDATED |
| t3/t6 stop 1.5 mm short of a 20 mm object | 87 recorded knuckle traces through the validated curve | yes | VALIDATED |
| approach deviation 0.00 mm mean / 0.01 max | EE from TF at each waypoint vs the straight line | no separate check, but it is a distance between two measured points with no calibration to get wrong | SOUND |
| wrist rotation 169.7° / 164.6° | angle between the home EE quaternion and the grasp quaternion | **yes, the hard way** — two separate bugs gave a false 0.0°, and the fix was checked by reproducing the independently-known 169.7° | VALIDATED |
| IK residual 0.0003–0.0135 mm | commanded vs achieved EE, 10 repeats | n/a | **STRUCTURAL** — the mock echoes commands, so this is solver convergence, not positioning accuracy |
| smallest grippable object | — | — | **UNVERIFIED**, and reported as underivable: its lower bound is the positioning error the mock cannot produce |
| dexterity 5 of 8 directions per arm | IK at each approach direction × 4 yaws, N=3 | no separate check | SOUND — but see the caveat below |

**Caveat now attached to the dexterity figure.** Part 8 established that the
identity quaternion is unreachable at every point of a line the home
quaternion reaches at 6/6. Grasp-orientation feasibility is therefore strongly
orientation-dependent, and 5-of-8 is specific to the eight directions probed.
It should not be read as a general dexterity fraction.

## Part 2 — capability ladder

| figure | how measured | validated? | verdict |
| --- | --- | --- | --- |
| SPH_RATE 0.076 m, SHELL 0.151 m mean | commanded tip error vs the 4-pot reference over 20 430 recorded frames | **yes** — rung FK against itself must return exactly 0.000, and the check was strengthened after it passed vacuously on zero parsed rows | VALIDATED |
| FK/SPHERICAL cost 0.000 m | same | yes | VALIDATED — true by construction, which is why it is the known answer |
| ladder selection, 128 subsets | pure function over every channel subset | n/a — arithmetic | SOUND |
| live downgrade and recovery | synthetic master, verdicts observed in the log | **yes** — the first fault injected was a 30° circular jump, below threshold; the node was right and the test data wrong | VALIDATED |
| "repair j2 and j4" | `regain()` vs the independent observability regression (R² 0.133) | yes — two independent routes agree | VALIDATED |

## Part 3 — scene fingerprint

| figure | how measured | validated? | verdict |
| --- | --- | --- | --- |
| decision accuracy 100%, 2000 trials/condition | constructed ground truth, pose noise injected at the characterised level | **yes** — ground truth is constructed, not rendered | VALIDATED, *conditional on σ = 0.8 mm* |
| smallest displacement 20 mm at 95% | same | yes; the model predicted 15 mm and the gap is the noise floor behaving as modelled | VALIDATED |
| `compare()` 0.12 ms / 6.7 ms | wall clock over 2000 and 200 calls | n/a | SOUND |
| colour/shape "unchanged" correct 2.6% | same harness at σ = 10 mm | yes | VALIDATED |
| end-to-end fingerprint accuracy | — | — | **UNVERIFIED** and not reported: no cameras here, and the renderer is out of the detector's distribution |
| four-stage node behaviour | live, against synthetic detections | yes — the empty-sweep bug was found by running it | VALIDATED |

## Part 4 — mode paths

| figure | how | validated? | verdict |
| --- | --- | --- | --- |
| VR path broken, `/vr_pose_left` pub=1 sub=0 | live ROS graph | n/a — the graph is the ground truth | SOUND |
| repaired: three hops pub=1 sub=1 | same | n/a | SOUND |
| two detectors on one topic | source inspection plus graph | n/a | SOUND |
| no camera-device contention in `srl_*` | search for device opens across the source | n/a | SOUND |
| e-stop is a designed second writer | source | n/a | SOUND — and the first checker version reported it wrongly as contention |

## Part 5 — free-form language

| figure | how | validated? | verdict |
| --- | --- | --- | --- |
| 17 correct / 7 ask / 6 refuse / **0 misunderstood**, n=30 | each phrase carries the target a competent listener picks; outcome compared against it | n/a — the ground truth is the phrase set itself, written before the results were seen | SOUND |
| 0 verb mismatches, n=8 | same | n/a | SOUND |

**The honest limit:** 30 phrases is a small sample and I wrote them. A phrase
set written by someone else, or transcribed from real speech, would be a
stronger test and would very likely find misunderstandings this one does not.

## Part 6 — GUI

| figure | how | validated? | verdict |
| --- | --- | --- | --- |
| RViz embeds, renders at 31 fps | **screenshot under Xvfb** | **yes** — `isVisible()` was true even when the capture was black, so only the pixels settled it | VALIDATED |
| GUI frame 0.45 ms median / 0.76 p95 | `perf_counter` around the refresh, 200-sample window | n/a | SOUND |
| Helvetica resolves | `fc-match` | n/a | SOUND |

## Part 7 — VR

| figure | how | validated? | verdict |
| --- | --- | --- | --- |
| refuses without adb, exit 1 | ran it; checked the script's own exit code, not a pipeline's | yes | VALIDATED |
| button mapping | read out of `vr_pose_mapper` and `vr_gripper_node` | n/a | SOUND |
| 8.66 ms round trip | earlier work, desktop mock | — | carried forward as a **lower bound**; excludes the headset's 13.9 ms frame period |
| anything against a headset | — | — | **UNVERIFIED** — no adb, no headset |

## Part 8 — operator aids

| figure | how | validated? | verdict |
| --- | --- | --- | --- |
| bisection 5 calls vs linear 10 | arithmetic | n/a | SOUND |
| executor deadlock is real | node started, logged, published zero messages | yes — fixed, and it then published | VALIDATED |
| **warning leads the wall, and by how much** | drove the commanded pose outward and recorded the first warning | **NO** | **UNVERIFIED — see below** |

### The unresolved one, stated plainly

`boundary_feedback_node` publishes and warns, but it reports the boundary at
**0 mm remaining** along a line that a direct inverse-kinematics probe shows is
reachable at **6 of 6** sample points with the same orientation. Those two
measurements contradict each other, so at least one instrument is wrong and I
do not know which.

Two candidate causes, neither eliminated: the node may probe before its cached
orientation is populated, in which case it is testing identity — which the
same experiment showed is unreachable at **0 of 6** points on that line; or
the velocity estimate may still be near zero when the first probe fires, so
the probe direction is not the direction of travel.

Per the standing rule, a result contradicting an earlier measurement is an
instrument check and not a finding until the two are reconciled. **The node is
therefore not verified, and the lead distance is not reported.**

---

## Summary

| verdict | count |
| --- | --- |
| VALIDATED | 14 |
| SOUND | 15 |
| UNVERIFIED | 4 |
| STRUCTURAL | 1 |

The four UNVERIFIED are: end-to-end fingerprint accuracy, smallest grippable
object, anything against a real headset, and the boundary warning's lead
distance. None is reported as a result anywhere.
