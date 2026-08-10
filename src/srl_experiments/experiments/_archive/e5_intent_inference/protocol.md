# E5 — Intent inference characterisation

**Type:** system evaluation. **Participants:** NOT required.

## Question
How accurate is the intent estimate as a function of object spacing, distractor
count and remaining distance to target — and what does a WRONG inference cost?

## Why it exists
It **sets the Part 5 thresholds**. `p_threshold` and `distance_threshold_m` in
`handover_arbiter` are currently defaults chosen by argument. After E5 they are
chosen by measurement, and the choice is recorded with the data that justified
it.

## Design
Object spacing {0.05, 0.10, 0.20, 0.35} m × distractor count {0, 1, 2, 4},
4 repeats. Thresholds are swept offline over the logged distributions, so one
data collection evaluates every threshold rather than one run per threshold.

## Measures
- accuracy of the top hypothesis at the moment of handover;
- accuracy as a function of remaining EE-to-target distance — the quantity the
  `distance_threshold_m` gate actually keys on;
- number of intent switches per trial;
- **the cost of a wrong inference**: the extra completion time on trials where
  the top hypothesis at handover was wrong. A wrong ASSIST costs a cancel and
  a re-approach, and that time is what makes a low threshold expensive.

## The three hard cases, checked here as well as in the unit tests
1. **No objects** — the estimator must report `no_objects`, not a uniform
   distribution over nothing and not a stale one.
2. **Equally likely objects** — the tie must be reported as a tie
   (`ambiguous`), and the arbiter must NOT enter ASSIST on one.
3. **Operator changes their mind mid-reach** — the estimate must abandon the
   old goal. Latching is worse than having no inference at all.

## Pre-registered hypotheses
- **H5.1** Accuracy increases monotonically as EE-to-target distance falls.
  Test: Spearman's ρ, one-tailed.
- **H5.2** Accuracy falls as spacing falls and as distractor count rises.
  Test: logistic mixed model with spacing and count as fixed effects.
- **H5.3** There exists a (p, d) pair whose expected cost — P(wrong)×cost(wrong)
  + P(late)×cost(late) — is lower than both the current default and the
  no-assist policy. If no such pair exists, **shared autonomy should not be
  enabled**, and that is a real possible outcome of this experiment.

## Stop / invalidate
25 s timeout. INVALID on channel dropout or e-stop.
