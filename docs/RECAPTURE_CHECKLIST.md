# Re-capture checklist — what must hold, and what each measurement unblocks

Verifiable at the end of the 35 minutes. Each row names the segments the
measurement comes from and the precondition that must hold **during** them.
The recorder now enforces most preconditions itself and aborts the segment if
one fails, so the check at the end is a confirmation, not a hope.

## Before you start

```bash
bash scripts/check_channels.sh          # ~3 min, diffs against the baseline
```

**If this still shows 6 of 14 usable, stop.** Blocks C–F cannot produce a
usable gain matrix, scale or smoothing parameter from incoherent inputs, and
the lateral/gyro question cannot be settled. Only block A is worth recording
until the wiring is repaired — and `check_channels.sh` IS that test.

Then:

```bash
# terminal 1
bash scripts/run_teleop.sh gate:=false
# terminal 2 — only if you want real-arm data (see row 4)
bash scripts/start_real.sh arm:=both
# terminal 3
ros2 run srl_experiments record_trajectories            # sim-only
ros2 run srl_experiments record_trajectories --need-real  # requires the bridge
```

Hold the master at neutral and **press each arm's button twice** (left = btn2,
right = btn1) before starting, so the clutch reference is latched at neutral.
Do not restart `master_pose_node` to achieve this — closing the port resets
the Teensy and drops it off WSL.

---

## 1. Lateral axis / gyro validation

| | |
| --- | --- |
| **Segments** | C (all 12) and E (all 24), both arms |
| **Precondition** | clutch **ENGAGED** on the arm under test for the whole segment; e-stop clear; `l_j1`/`r_j1` at least INTERMITTENT (azimuth source); gyro columns `*_gx/gy/gz` non-constant |
| **Unblocks** | recomputing azimuth from the gyro offline, regressing the gain matrix, and reporting the pairwise-angle residual against the 54.8°/54.7° baseline |
| **Verify after** | every C/E segment has `left_clutch`/`right_clutch` = 1 throughout, and `estop` = 0 |

The 2026-08-06 attempt failed here: the clutch was out for 41 of 42 of these
segments, so the commanded pose was frozen and there was nothing to regress.

**Ship the gyro path only if the triad becomes measurably more orthogonal.**
If it does not, say so and do not ship — the same discipline that correctly
killed the FK hybrid.

## 2. Workspace scaling

| | |
| --- | --- |
| **Segments** | C and D (28 segments), both arms — D supplies the diagonals that reveal anisotropy |
| **Precondition** | clutch engaged; scale recorded per row (`*_scale`); the operator reaching genuine mechanical extremes, not a comfortable range |
| **Unblocks** | the measured master range per axis, hence the per-arm scale that maps the operator's full range onto the reachable workspace |
| **Verify after** | `left_scale`/`right_scale` constant and known; per-direction master excursions differ from each other by less than about 2× (otherwise the operator, not the rig, set the limit) |

Needs the reachability measurement too — **re-run it from a genuine home
pose**, with the sim freshly launched and the followers stopped:

```bash
python3 scripts/measure_workspace.py --arm both --step 0.01 --max 0.50
```

## 3. Smoothness tuning

| | |
| --- | --- |
| **Segments** | F (6) for rate dependence; A (14) for the noise floor; C for the command-step distribution |
| **Precondition** | F performed at genuinely different speeds (slow/medium/fast should differ ~2× in median |v|); clutch engaged; channels used by the current mode not INCOHERENT |
| **Unblocks** | `ema_alpha` from the noise floor and motion bandwidth, `max_step_rad` from the p95 command step, `accel_gate_g` from measured `|a|` |
| **Verify after** | noise floor is **non-zero** on every live channel — a 0.000 means duplicate rows got in again and the whole derivation is void |

## 4. Real-arm gain matrix

| | |
| --- | --- |
| **Segments** | C and E, both arms, with the real stack up |
| **Precondition** | `--need-real`; bridge **enabled** (`real_bridge_enabled` = 1); Kortex session connected; both arms homed WITHIN tolerance; clutch engaged |
| **Unblocks** | d(real EE)/d(master) — the only measurement that shows whether the robot actually follows, as opposed to the command being correct |
| **Verify after** | `*_real_joint_*` and `*_real_ee_*` columns populated and **varying**; `real_bridge_enabled` = 1 throughout |

This has never been measured. The one prior moving-master attempt showed the
command sweeping at gain ~1.0 while the real EE gain was **~0.1** — that gap
is the entire question, and it needs these columns.

---

## End-of-session verification, in one command

```bash
python3 - <<'PY'
import csv, sys, collections
rows=list(csv.DictReader(open(sys.argv[1] if len(sys.argv)>1 else
    'recordings/trajectory_capture/all_segments.csv')))
seg=collections.defaultdict(list)
for r in rows: seg[r['segment']].append(r)
bad=[]
for s,rs in sorted(seg.items()):
    arm=rs[0]['arm']; blk=rs[0]['block']
    cl=sum(float(r.get(arm+'_clutch') or 0) for r in rs)/len(rs)
    es=sum(float(r.get('estop') or 0) for r in rs)/len(rs)
    br=sum(1 for r in rs if r.get('real_bridge_enabled')=='1')/len(rs)
    if blk in 'CDEF' and cl<0.9: bad.append((s,'clutch %.0f%%'%(100*cl)))
    if es>0.01: bad.append((s,'estop %.0f%%'%(100*es)))
print("segments: %d   rows: %d"%(len(seg),len(rows)))
print("noise check: n_rejected_raw final =", rows[-1].get('n_rejected_raw'))
print("real columns populated:", any(r.get('left_real_joint_1') for r in rows))
print("PROBLEMS:" if bad else "all segments clean")
for s,w in bad: print("  ",s,w)
PY
```

Anything listed is a segment that must be redone. A clean run means the four
measurements above are all derivable.
