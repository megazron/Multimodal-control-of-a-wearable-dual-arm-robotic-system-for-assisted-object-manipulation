# TWO-PERSON MEASURES — the wearer is a participant

*Created 2026-08-08 with the reframing. The wearer used to be apparatus. They
are now a participant, with their own consent, their own dependent variables
and the right to stop the session.*

Where a validated instrument exists it is used **verbatim and cited**. Where
none exists the item is written here, marked **[BESPOKE]**, and its wording is
fixed in advance so it cannot drift between participants.

---

## 1. WEARER measures

The wearer's position has no exact precedent: they bear the mass and the risk,
have **no control input whatsoever**, and cannot see the arms behind them.
The nearest validated literature is the autonomous-vehicle **passenger** — a
person who has ceded control and remains physically exposed.

| construct | instrument | when | why this one |
| --- | --- | --- | --- |
| **Perceived safety** | Godspeed III Perceived Safety, 3 items, 5-pt semantic differential (anxious/relaxed, agitated/calm, quiescent/surprised) | after every block | The standard HRI instrument. Short enough to repeat per block without becoming the task. |
| **Trust in the OPERATOR** | Jian et al. (2000) *Trust between People and Automation*, 12 items, reworded so the referent is **the person** | after every block | The wearer's trust has two distinct objects and they must not be summed. Trusting a competent operator and trusting the autonomy are different beliefs that can move in opposite directions. |
| **Trust in the AUTONOMY** | Jian et al. (2000), same 12 items, referent **the robot's own decisions** | after every block | as above |
| **Agency / acceptability of having none** | **[BESPOKE]**, 4 items, 7-pt: (1) "I felt in control of what the arms did" (reverse-scored, expected floor); (2) "I was comfortable **not** being in control"; (3) "I wanted a way to stop the arms myself"; (4) "The arms felt like they were part of me" | after every block | Item 1 is a **manipulation check**, not a finding: the wearer has no input, so it must floor. If it does not, they are misreading the question and the block's other agency data is suspect. Item 4 is asked to be **refuted** — see §5. |
| **Predictability of the arms' motion** | **[BESPOKE]**, 3 items, 7-pt: "I could tell where the arms were about to go"; "The arms moved smoothly and evenly"; "The arms surprised me" (reverse) | after every block | **This is the mediator in H1c.** If autonomy costs wearer trust, the hypothesis is that it does so through predictability, not through autonomy as such — and that distinction changes the design implication from "less autonomy" to "more legible autonomy". |
| **Comfort / exertion** | **Borg CR10**, standard anchors, plus a body map (shoulders / lumbar / neck) | every block boundary **and** on any stop | Validated, one number, takes ten seconds. The body map separates *load* from *posture*, which the arms affect differently. |
| **Collaborator or platform** | **[BESPOKE]** forced-choice + 7-pt: "During that block I felt more like **a collaborator** / **a platform**", then "how strongly" | after every block | Deliberately blunt and deliberately forced-choice. A Likert scale would let everyone sit at the midpoint, and the point of T8 is that it should MOVE this item. |
| **Arousal, not self-reported** | **electrodermal activity (skin conductance)**, wrist or finger, continuous; SCR peaks time-locked to arm-motion onsets | continuous | SRL Proxemics found subjective and physiological measures **dissociate** — arousal rose under high autonomy while participants often reported acceptance. A wearer cannot be blinded to condition, so a measure they cannot consciously manage is the strongest thing we have. |

### 1.1 Two wearer measures that are outcomes, not questionnaire items

- **`wearer_stop_events`** — the wearer can halt at any time. Every stop is
  logged with time, condition, and the wearer's own stated reason. **A stop
  is data, not attrition**, and is reported in the results rather than
  dropped. See §5 of `02_baseline_and_hypotheses.md`.
- **`flinch_events`** — a startle or withdrawal, from the EDA trace plus
  video, time-locked to arm motion. Counted per block. This is the behaviour
  the perceived-safety scale is a proxy for.

---

## 2. OPERATOR measures

| construct | instrument | when |
| --- | --- | --- |
| **Workload** | **NASA-TLX**, raw (unweighted) — as already used in the E-set | after every block |
| **Trust in the autonomy** | Jian et al. (2000), 12 items | after every block |
| **AWARENESS OF THE WEARER'S STATE** | **[BESPOKE]**, and the important one — see below | during, and after every block |
| all existing task metrics | completion time, tracking error, path efficiency, tilt/separation, blocks placed, IK success, clearance | continuous |

### 2.1 Wearer-state awareness — a probe, not a rating

**The operator cannot feel the wearer.** Asking them to rate their own
awareness measures confidence, not awareness, and the two are known to come
apart. So awareness is measured by **probe accuracy**:

At two unpredictable moments per block the experimenter freezes the arms and
asks the operator three questions with **objectively knowable answers**:

1. "Which way is the wearer leaning right now?" (forward / back / left / right / upright)
2. "How close is your left gripper to them?" (< 10 cm / 10–30 cm / > 30 cm)
3. "Has the wearer moved their feet since the block started?" (yes / no)

Scored against the logged truth: wearer IMU for (1) and (3), the clearance
trace for (2). This yields `awareness_accuracy` in [0, 1] per block.

A **confidence** rating is taken alongside each answer, so
**calibration** — confidence minus accuracy — is available. Over-confidence
in a person who cannot feel the risk they are creating for someone else is a
safety finding in its own right, and it is invisible to a self-report scale.

---

## 3. DYADIC measures

| measure | how | note |
| --- | --- | --- |
| **`comm_events`** | count of utterances between operator and wearer, from the two audio channels, coded as **request / acknowledgement / warning / social** | The channel is deliberately open. Muting it would make T8 impossible and would not model any real deployment. |
| **`comm_initiator`** | who spoke first in each exchange | Feeds H3: initiation should shift from operator toward wearer with practice. |
| **`wearer_body_motion`** | **the disturbance, measured, not assumed**: wearer torso IMU at 100 Hz → RMS excursion and dominant frequency per trial | This is a **covariate in every task** and the **IV in T9**. Without it, a bad trial cannot be attributed. |
| **`coordination_latency_s`** | T8: `t_wearer_completes_reposition − t_operator_requests` | The headline T8 metric. |
| **`anticipation`** | T8: the sign of `t_wearer_starts_moving − t_operator_requests`. **Negative means the wearer moved BEFORE being asked.** | The substantive claim in H3. A negative value is a predictive model of another person's intent, built by someone with no control over the limbs. |
| **`role_order`** | which member wore first | Counterbalanced across dyads; logged per session. |
| **`prior_acquaintance`** | stranger / colleague / friend | Cannot be balanced at n=12; disclosed. |

---

## 4. Counterbalancing roles, and logging it

Each dyad member does **both** roles on the role-swapped tasks (T5, T9).
Order is counterbalanced **across dyads**:

```
dyad 01  member A wears first     dyad 02  member B wears first
dyad 03  member A wears first     dyad 04  member B wears first   ...
```

so role-order imbalance is exactly 0 for even n. Logged per session in the
manifest as `role_order: A_wears_first | B_wears_first`, plus `wore_first`
and `operated_first` per participant ID.

**Role is NOT swapped on T2, T3, T6, T7 or T8.** Swapping everywhere doubles a
session that already runs to the fatigue limit, and the wearer's role is
nearly identical across those five. The cost — no within-dyad role contrast on
those tasks — is a deliberate loss of power, recorded here so it is not
mistaken for an oversight.

---

## 5. Three measurement traps specific to this design

**5.1 The wearer cannot be blinded.** They feel every motion, so the autonomy
level is obvious to them in a way it is not to a reader of the protocol.
Mitigations: autonomy is **never named** in their instructions; the condition
is never announced; and EDA is recorded precisely because it is not under
conscious control. **Wearer self-report must never be presented without the
physiological trace beside it.**

**5.2 The embodiment item is asked in order to be REFUTED.** Our arms are
never mechanically linked to the wearer's limbs — Fusion's *Enforced* mode is
the one that would license ownership language and we do not implement it. If
item 4 of the agency scale comes out high, that is a **demand-characteristic
warning**, not a finding of embodiment. Stated in advance so a high score
cannot be reported as a discovery after the fact.

**5.3 A dyad is one observation.** The two members talked to each other and in
T8 had to. Ratings within a dyad are not independent. Analysis is on
dyad-level aggregates or mixed models with a dyad random effect; treating 12
dyads as 24 participants would roughly halve the true standard errors.

---

## 6. Instrument sources

- Jian, Bisantz & Drury (2000), *Foundations for an empirically determined
  scale of trust in automated systems*, IJCE 4(1) 53-71.
- Bartneck et al. (2009), *Measurement instruments for the anthropomorphism,
  animacy, likeability, perceived intelligence and safety of robots* —
  Godspeed, series III.
- Hart & Staveland (1988), NASA-TLX; raw TLX per Hart (2006).
- Borg (1998), CR10 scale.
- SRL Proxemics, CHI 2026 — the precedent for pairing EDA with trust and
  safety report in exactly this setting: https://arxiv.org/html/2602.00494v1
- AV-passenger risk perception and the subjective/physiological dissociation:
  https://pmc.ncbi.nlm.nih.gov/articles/PMC10790836/
