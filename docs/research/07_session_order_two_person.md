# SESSION ORDER FOR TWO PEOPLE

*2026-08-08. Both arrive together, both consent, both are participants.*

The single-person session was 75 minutes and was already at the fatigue limit.
Two people do not double it — they constrain it, because **the binding
constraint is now the wearer**, and the wearer's fatigue is a different curve
from the operator's.

---

## 1. The two fatigue curves, and why one session length is wrong for both

| | WEARER | OPERATOR |
| --- | --- | --- |
| what they bear | **>17 kg** of arms, harness and pack | an **uncompensated** master arm |
| type of load | isometric, postural | dynamic, repeated reaching |
| where it hurts | shoulders, lumbar spine, neck | deltoid, forearm |
| curve | **monotonic and unforgiving** — static load does not recover during a trial, only during a break | sawtooth — recovers partly between trials |
| worst case | pain, then a stop | slower, sloppier reaching |
| how it confounds | with **condition order**, and with any measure taken late | same |

**The wearer sets the ceiling.** A wearer at Borg 7 is not producing usable
safety ratings, and the ethical position is worse than the statistical one.
So: **wearing blocks are capped at 12 minutes with the pack ON**, and the pack
comes **off** during every break — not loosened, off.

---

## 2. The order

**Total: 165 minutes door to door, of which 78 minutes is task time and no
single person wears the pack for more than 12 minutes consecutively.**

```
   PHASE                                          min   who        pack
   ─────────────────────────────────────────────────────────────────────
 0 Arrival, both together                          5    both        off
 1 CONSENT — separately, in different rooms       10    each        off
 2 Baselines: demographics, prior robot use,      10    both        off
   Borg baseline, EDA sensor fitted to wearer
 3 Safety brief + STOP DRILL                      10    both        off
 4 Familiarisation: operator drives, wearer         8    A wears     ON
   wears, no data recorded
   ── BREAK, pack OFF, Borg ─────────────────      5    both        off
 5 BLOCK 1   T5 + T9      (role: A wears)         12    A wears     ON
   ── BREAK, pack OFF, Borg, questionnaires ──     6    both        off
 6 BLOCK 2   T3 + T6      (role: A wears)         12    A wears     ON
   ── BREAK, pack OFF, Borg, questionnaires ──     6    both        off
 7 BLOCK 3   T2 + T7 + T8 (role: A wears)         12    A wears     ON
   ── ROLE SWAP + fitting + re-brief ─────────    10    both        off
 8 BLOCK 4   T5 + T9      (role: B wears)         12    B wears     ON
   ── BREAK, pack OFF, Borg, questionnaires ──     6    both        off
 9 Debrief, both, separately then together        10    both        off
   ─────────────────────────────────────────────────────────────────────
                                        TOTAL    165 min
```

### Why this order

**Consent is taken separately, in different rooms.** Two people who arrived
together will not decline in front of each other. The wearer in particular is
consenting to something the operator is not — bearing 17 kg and physical risk
they cannot control — and a joint consent session makes refusal socially
expensive. This is the single most important line in this document.

**The stop drill is rehearsed before any data.** The wearer physically
performs a stop and sees the arms halt, twice. A safety mechanism the wearer
has not personally exercised is not a safety mechanism they trust, and trust
is a dependent variable here.

**T5 goes FIRST, in Block 1, while the wearer is freshest.** It is the only
task with a functional wearer role, so it is the one where wearer fatigue
would most contaminate the measure. It is also the task the promotion argument
rests on.

**T9 is paired with T5** because the wearer is active in both — receiving in
T5, swaying in T9 — so the "wearer as collaborator" items are asked about two
active blocks rather than one active and one passive.

**T3 and T6 are paired** because they differ *only* in the coupling; running
them adjacent under the same fatigue state is what makes that comparison
clean. Splitting them across a role swap would confound coupling with
whoever was wearing.

**T8 goes last of A's blocks**, because the learning hypothesis (H3) needs
repetitions and it is the least fatiguing for the wearer — repositioning is
self-paced movement, which is *relief* from static load rather than an
addition to it.

### Role swap: one block, not four

Only **Block 4 (T5 + T9)** is repeated with roles swapped. Full crossing of
role × task would need eight blocks and a 4½-hour session that neither person
could finish.

**This is a deliberate loss of statistical power on the role factor, and the
consequence must be stated in the paper:** the within-dyad role contrast
exists only for T5 and T9. For T2, T3, T6, T7 and T8 the role contrast is
**between dyads only**, via the counterbalanced `role_order`.

T5 and T9 were chosen because they are the two tasks where the wearer's role
differs most from being a mount — receiving an object, and generating the
disturbance — so they are where a role effect is most likely to exist at all.

---

## 3. Counterbalancing, per dyad

- **Autonomy condition** (direct / assisted / shared): Williams square,
  assigned **per dyad**, so immediate sequence is balanced as well as position.
- **Role order**: alternated across dyads (A-wears-first / B-wears-first), so
  imbalance is exactly 0 for even n.
- **Task order within a block**: fixed, because the pairings above are
  deliberate. Order effects within a pair are absorbed by the autonomy
  counterbalancing.

Logged in the manifest: `dyad_id`, `role_order`, `wore_first`,
`operated_first`, `williams_row`, `prior_acquaintance`, `mounting`.

---

## 4. Stopping rules — three, and any one ends the block

1. **The wearer says stop.** No justification required, no persuasion, ever.
   Logged with their stated reason. **This is data, not attrition.**
2. **Borg CR10 ≥ 7** for either person at a block boundary → that person does
   not start another wearing block. If it is the wearer, the session ends.
3. **Any clearance-floor breach or e-stop trip** → block ends, trial marked
   invalid with the cause, and the rig is re-checked before continuing.

A session that ends early is reported with the phase reached. Sessions are
**not** replaced with a fresh dyad to "top up n" — that would select for
tolerant wearers, which is the exact population whose safety ratings are least
informative.

---

## 5. What the wearer is told, and what they are not

**Told:** what the arms will do; that they have no control; that they can stop
at any moment and how; that they will be asked how safe they felt; that EDA
is recorded.

**Not told:** the autonomy level of any block, or that autonomy varies at all.
Their instructions never use the word. **They cannot be blinded to the
motion** — they feel it — but they can be blinded to its *label*, and that is
the difference between rating the behaviour and rating the word.

Disclosed in full at debrief, before they leave.

---

## 6. Honest accounting of the duration

- **165 minutes** door to door.
- **78 minutes** of pack-on task time across both people, **48 minutes**
  maximum for one person, in four blocks of ≤ 12.
- **33 minutes** of breaks with the pack off.
- The rest is consent, briefing, fitting, the role swap and debrief.

Compared with the 75-minute single-person session, the increase is **not**
because there is twice the task — the task time is 78 minutes against 45.
It is consent taken twice and separately, a stop drill, a role swap with
re-fitting, and breaks long enough for a person carrying 17 kg. **None of
those are compressible**, and the first is not negotiable.

**If the session must be shortened**, cut in this order: (1) drop the role
swap entirely and go between-dyads on role, saving 28 minutes; (2) drop T8,
saving 6; (3) drop T2, saving 6. **Do not** cut the breaks, the stop drill or
the separate consent.
