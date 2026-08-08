#!/usr/bin/env python3
"""PILOT: every task end to end, scripted, with a fault injected mid-trial.

Confirms: metrics captured, counterbalancing balanced, invalid trials marked
and excluded as pre-registered, analysis produces its numbers.

Runs offline against synthesised trajectories -- the trajectory GENERATOR is
the scripted operator, and the metrics under test are the shipped ones. The
ground truth here is CONSTRUCTED (a known tilt, a known tracking offset), not
rendered, which is the distinction the measurement rule turns on.
"""
import csv, itertools, json, math, os, sys
import numpy as np
import yaml

R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(R, "src/srl_experiments/experiments/bimanual"))
sys.path.insert(0, os.path.join(R, "src/srl_experiments/experiments/bimanual/t7_pursuit"))
sys.path.insert(0, os.path.join(R, "src/srl_experiments"))
import coupled_metrics as cm
import targets as tg

CONDS = ("direct", "assisted", "shared")
OUT = os.path.join(R, "recordings/pilot_bimanual")
SC = yaml.safe_load(open(os.path.join(
    R, "src/srl_experiments/experiments/bimanual/scenarios_verified.yaml")))


def latin_square(n, seed=0):
    """Balanced Latin square over n conditions, one row per participant."""
    rows = []
    for i in range(n):
        rows.append([(i + j) % n for j in range(n)])
    return rows


def sim_transport(scenario, cond, fault=False, dt=0.02, rng=None):
    """Scripted operator on a coupled transport. Error grows with divided
    attention (DIRECT worst, SHARED best) -- the PREDICTION under test."""
    rng = rng or np.random.default_rng(0)
    path = np.asarray(scenario["path"], float)
    sep = float(scenario["sep"])
    seg = 120
    t, L, Rr = [], [], []
    noise = {"direct": 1.0, "assisted": 0.55, "shared": 0.25}[cond]
    for k in range(len(path) - 1):
        for i in range(seg):
            f = i / seg
            p = path[k] * (1 - f) + path[k + 1] * f
            tt = (k * seg + i) * dt
            # coordination error: a slow drift plus noise, scaled by condition
            dh = noise * (0.008 * math.sin(1.1 * tt) + rng.normal(0, 0.002))
            ds = noise * (0.010 * math.sin(0.7 * tt) + rng.normal(0, 0.003))
            if fault and tt > 1.5:          # autonomy silently drops out
                dh *= 4.0
                ds *= 4.0
            s = sep + ds
            t.append(tt)
            L.append([p[0] - s / 2, p[1], p[2] + dh / 2])
            Rr.append([p[0] + s / 2, p[1], p[2] - dh / 2])
    return np.array(t), L, Rr


def sim_pursuit_calibration(scenario, b_self, b_cross, base_mm=20.0,
                            dt=0.02, dur=20.0):
    """CALIBRATION trials: RMS error is LINEAR in the two speeds BY
    CONSTRUCTION, so the pilot can recover exactly what it injects.

    The EE is placed at a fixed offset from its target whose MAGNITUDE is the
    intended RMS error. No lag, no noise -- the point is not to model an
    operator, it is to prove that

        summarise_pursuit -> interference_coefficient

    carries a known number through the whole pipeline unchanged. The
    realistic generator below is what models an operator; this proves the
    instrument. A unit test that recovers 7.5 in isolation proves the
    arithmetic, not the pipeline.
    """
    t, TL, TR = tg.trial_targets(scenario, dur, dt)
    vL = scenario.get("speed_left", 0.0)
    vR = scenario.get("speed_right", 0.0)

    def offset(v_self, v_other):
        mag_mm = base_mm + b_self * v_self + b_cross * v_other
        d = (mag_mm / 1000.0) / math.sqrt(3.0)
        return np.array([d, d, d])
    return t, TL + offset(vL, vR), TR + offset(vR, vL), TL, TR


def sim_pursuit(scenario, cond, dt=0.02, dur=20.0, rng=None):
    """Scripted operator on pursuit. Lag and error grow with the OTHER arm's
    speed -- the interference effect the analysis must recover."""
    rng = rng or np.random.default_rng(1)
    t, TL, TR = tg.trial_targets(scenario, dur, dt)
    gain = {"direct": 1.0, "assisted": 0.5, "shared": 0.15}[cond]
    vL = scenario.get("speed_left", 0.0)
    vR = scenario.get("speed_right", 0.0)
    B_SELF, B_CROSS = 0.09, 0.06          # metres of error per (m/s)
    def follow(T, v_self, v_other):
        amp = gain * (0.010 + B_SELF * v_self + B_CROSS * v_other)
        lag = int(round((0.08 + 0.25 * gain) / dt))
        E = np.roll(T, lag, axis=0)
        E[:lag] = T[0]
        return E + rng.normal(0, amp / 3.0, T.shape) + amp * 0.5
    return t, follow(TL, vL, vR), follow(TR, vR, vL), TL, TR


def main():
    os.makedirs(OUT, exist_ok=True)
    rng = np.random.default_rng(7)
    n_part = 6
    square = latin_square(3)
    rows_t3, rows_t6, rows_t7 = [], [], []
    manifest = []
    fault_trials = set()

    print("PILOT -- %d participants, Latin-square within task\n" % n_part)
    for pid in range(n_part):
        order = square[pid % 3]
        for task, store, coupling in (("T3_rigid", rows_t3, "rigid"),
                                      ("T6_compliant", rows_t6, "compliant")):
            for ci in order:
                cond = CONDS[ci]
                for sname, sc in SC["tasks"][task].items():
                    if not sc["verified"]:
                        continue
                    # one unexpected-failure trial per participant
                    fault = (task == "T3_rigid" and cond == "assisted"
                             and sname == "S2_long_height" and pid % 2 == 0)
                    t, L, Rr = sim_transport(sc, cond, fault, rng=rng)
                    m = cm.summarise_transport(t, L, Rr, sc["sep"], coupling)
                    m.update(participant="P%02d" % pid, task=task,
                             condition=cond, scenario=sname,
                             valid=not fault,
                             invalid_reason="autonomy silently disabled"
                                            if fault else "",
                             wear_time_s=round(300 + 60 * len(store), 1))
                    store.append(m)
                    if fault:
                        fault_trials.add(("P%02d" % pid, task, sname))
        for ci in order:
            cond = CONDS[ci]
            for sname, sc in SC["tasks"]["T7_pursuit"].items():
                if not sc["verified"]:
                    continue
                t, EL, ER, TL, TR = sim_pursuit(sc, cond, rng=rng)
                sl = cm.summarise_pursuit(t, EL, TL)
                sr = cm.summarise_pursuit(t, ER, TR)
                rows_t7.append(dict(
                    participant="P%02d" % pid, task="T7_pursuit",
                    condition=cond, scenario=sname, valid=True,
                    invalid_reason="",
                    speed_left=sc.get("speed_left", 0.0),
                    speed_right=sc.get("speed_right", 0.0),
                    unimanual=sc.get("unimanual", ""),
                    rms_error_mm_left=sl["rms_error_mm"],
                    rms_error_mm_right=sr["rms_error_mm"],
                    max_error_mm_left=sl["max_error_mm"],
                    phase_lag_s_left=sl["phase_lag_s"],
                    frac_on_target_left=sl["frac_on_target"],
                    wear_time_s=round(300 + 20 * len(rows_t7), 1)))
        manifest.append(dict(participant="P%02d" % pid, order=[CONDS[i] for i in order]))

    for name, rows in (("t3", rows_t3), ("t6", rows_t6), ("t7", rows_t7)):
        p = os.path.join(OUT, "%s_trials.csv" % name)
        with open(p, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=sorted({k for r in rows for k in r}))
            w.writeheader(); w.writerows(rows)
        print("  %-3s %4d trials -> %s" % (name, len(rows), os.path.basename(p)))
    json.dump(manifest, open(os.path.join(OUT, "manifest.json"), "w"), indent=1)

    # ---- counterbalancing
    print("\nCOUNTERBALANCING")
    pos = {c: [0, 0, 0] for c in CONDS}
    for m in manifest:
        for i, c in enumerate(m["order"]):
            pos[c][i] += 1
    for c in CONDS:
        print("  %-9s position counts %s" % (c, pos[c]))
    bal = all(len(set(v)) == 1 for v in pos.values())
    print("  balanced: %s" % bal)

    # ---- validity
    print("\nVALIDITY")
    inval = [r for r in rows_t3 if not r["valid"]]
    print("  %d invalid trials, all marked with a cause: %s"
          % (len(inval), sorted({r["invalid_reason"] for r in inval})))
    good = [r for r in rows_t3 if r["valid"]]
    print("  excluded from analysis: %d of %d" % (len(inval), len(rows_t3)))

    # ---- headline analyses
    print("\nT3 vs T6 -- coordination error by condition (valid trials)")
    print("  %-10s %-22s %s" % ("condition", "T3 tilt RMS (deg)", "T6 sep err RMS (mm)"))
    for c in CONDS:
        a = [r["tilt_rms_deg"] for r in rows_t3 if r["condition"] == c and r["valid"]]
        b = [r["sep_err_rms_mm"] for r in rows_t6 if r["condition"] == c and r["valid"]]
        print("  %-10s %-22.3f %.3f" % (c, np.mean(a), np.mean(b)))
    d3 = np.mean([r["tilt_rms_deg"] for r in rows_t3 if r["condition"]=="direct" and r["valid"]])
    s3 = np.mean([r["tilt_rms_deg"] for r in rows_t3 if r["condition"]=="shared" and r["valid"]])
    d6 = np.mean([r["sep_err_rms_mm"] for r in rows_t6 if r["condition"]=="direct"])
    s6 = np.mean([r["sep_err_rms_mm"] for r in rows_t6 if r["condition"]=="shared"])
    print("  DIRECT->SHARED reduction: T3 %.0f%%   T6 %.0f%%"
          % (100*(1-s3/d3), 100*(1-s6/d6)))

    # ---------------- CALIBRATION: injected must equal recovered -----------
    print("\nT7 PIPELINE CALIBRATION -- injected vs recovered, end to end")
    B_SELF_INJ, B_CROSS_INJ = 90.0, 60.0        # mm of RMS error per (m/s)
    cal = []
    for sname, sc in SC["tasks"]["T7_pursuit"].items():
        if not sc["verified"] or sc.get("unimanual"):
            continue
        for vL in (0.05, 0.10, 0.20, 0.30):
            for vR in (0.05, 0.10, 0.20, 0.30):
                s2 = dict(sc, speed_left=vL, speed_right=vR)
                t, EL, ER, TL, TR = sim_pursuit_calibration(
                    s2, B_SELF_INJ, B_CROSS_INJ)
                sl = cm.summarise_pursuit(t, EL, TL)
                cal.append(dict(speed_left=vL, speed_right=vR,
                                rms_error_mm_left=sl["rms_error_mm"]))
        break
    fitc = cm.interference_coefficient(cal, "left")
    print("  injected   b_self=%.1f  b_cross=%.1f  mm per (m/s)"
          % (B_SELF_INJ, B_CROSS_INJ))
    print("  recovered  b_self=%.1f  b_cross=%.1f  R2=%.4f  (n=%d)"
          % (fitc["b_self"], fitc["b_cross"], fitc["r2"], fitc["n"]))
    cal_ok = (abs(fitc["b_self"] - B_SELF_INJ) < 0.5
              and abs(fitc["b_cross"] - B_CROSS_INJ) < 0.5)
    print("  AGREE (within 0.5 mm/(m/s)): %s" % cal_ok)

    print("\nT7 -- CROSS-ARM INTERFERENCE (headline)")
    bim = [r for r in rows_t7 if not r["unimanual"] and r["condition"] == "direct"]
    fit = cm.interference_coefficient(bim, "left")
    print("  n=%d  b_self=%.1f  b_cross=%.1f mm per (m/s)  b_int=%.1f  R2=%.3f"
          % (fit["n"], fit["b_self"], fit["b_cross"], fit["b_interaction"], fit["r2"]))
    uni = [r for r in rows_t7 if r["unimanual"] == "left" and r["condition"]=="direct"]
    if uni and bim:
        ue = np.mean([r["rms_error_mm_left"] for r in uni])
        be = np.mean([r["rms_error_mm_left"] for r in bim
                      if abs(r["speed_left"]-0.20) < 1e-6])
        print("  dual-task cost (left, speed 0.20): %.2f  (unimanual %.1f mm -> bimanual %.1f mm)"
              % (cm.dual_task_cost(be, ue), ue, be))
    print("\n  WHAT THIS CHECKS, precisely.")
    print("  The CALIBRATION block above is the end-to-end proof: error is")
    print("  linear in the speeds by construction, and the pipeline returns")
    print("  the injected 90/60 exactly. This block uses the REALISTIC")
    print("  generator instead, whose error is a nonlinear combination of an")
    print("  amplitude term and a lag term, so its fit is a linear")
    print("  approximation and is NOT expected to equal the amplitude")
    print("  constants. What is checked here is the ordering the design")
    print("  predicts: b_cross > 0 and b_self > b_cross.")
    ok = fit["b_cross"] > 0 and fit["b_self"] > fit["b_cross"]
    print("  ordering as predicted (b_cross>0, b_self>b_cross): %s" % ok)
    print("\n  Reported per the protocol as TOTAL speed sensitivity: the")
    print("  realistic generator's fit (%.0f/%.0f) exceeds the amplitude"
          % (fit["b_self"], fit["b_cross"]))
    print("  constants because a fixed tracking lag turns into positional")
    print("  error v*L, which also scales with speed. b_cross is DEFINED as")
    print("  the total; phase_lag_s is reported beside it so the lag")
    print("  component stays visible.")
    return 0 if (bal and inval and ok and cal_ok) else 1

if __name__ == "__main__":
    sys.exit(main())
