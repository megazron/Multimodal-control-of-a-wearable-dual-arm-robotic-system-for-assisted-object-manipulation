# TODO AUDIT

**Total: 28**

| category | count |
| --- | --- |
| **CITE** | 9 |
| **MEASURE** | 3 |
| **FIGURE** | 4 |
| **WRITE** | 7 |
| **CONFIRM** | 5 |

## CITE (9)

- [ ] `background/background.tex:123` — CITE: a shared-autonomy formalism for the policy-blending claim --- Dragan and Srinivasa, and Javdani et al.\ on hindsight optimisation
- [ ] `background/background.tex:125` — CITE: Fitts' law and the ISO 9241-9 pointing methodology, for the Task~1 throughput measure
- [ ] `background/background.tex:127` — CITE: NASA-TLX (Hart and Staveland 1988; Hart 2006 for raw TLX)
- [ ] `background/background.tex:128` — CITE: an embodiment / agency instrument for the wearer measures
- [ ] `background/background.tex:129` — CITE: an open-vocabulary detector for the vision mode --- YOLO-World or Grounding DINO
- [ ] `introduction/introduction.tex:10` — CITE: an SRL review establishing the taxonomy and the augmentation-versus-restoration distinction --- Tong et al.~2021 [1] and Prattichizzo et al.~2021 [4] are both in the planning report's bibliography
- [ ] `introduction/introduction.tex:18` — CITE: the MUVE / Dr.~Octopus platform page [2]
- [ ] `introduction/introduction.tex:31` — CITE: Guggenheim and Asada~2021 [12] on miniature physical masters and inherent haptic feedback
- [ ] `introduction/introduction.tex:39` — CITE: Saraiji et al., Fusion, SIGGRAPH 2018 Emerging Technologies

## MEASURE (3)

- [ ] `conclusion/conclusion.tex:62` — MEASURE: the experimental programme in full --- no participant data exists
- [ ] `result/result.tex:51` — MEASURE: every task metric with a participant. The protocols, scenarios, metrics and success detection exist and have been exercised with scripted input, but no human has driven the master arm.
- [ ] `result/result.tex:120` — MEASURE: capture these trip traces to file --- the figures above are from the engineering log and no artefact was retained

## FIGURE (4)

- [ ] `main.tex:105` — FIGURE: Imperial College logo --- run \texttt{epstopdf title/logo.eps} once ghostscript is available, then restore the \texttt{includegraphics} in \repo{title/title.tex}.
- [ ] `method/method.tex:32` — FIGURE: the master kinematic chain with link lengths and joint axes labelled, rendered from \repo{\_source/artefacts/CAD/PotArm.step}
- [ ] `method/method.tex:34` — FIGURE: photographs or CAD renders of the printed links J1--J7 from \repo{\_source/artefacts/3dprint/}
- [ ] `method/method.tex:217` — FIGURE: one control-flow diagram per mode (direct teleoperation, VR, shared autonomy, full autonomy), drawn in TikZ, showing where the operator's authority ends and the machine's begins in each

## WRITE (7)

- [ ] `appendix/appendix.tex:4` — WRITE: package-by-package table of the repository, with the role of each node -- generatable from the source tree
- [ ] `appendix/appendix.tex:9` — WRITE: reproduce the verified scenario coordinates from recordings/baselines/scenario\_audit.yaml as a table
- [ ] `appendix/appendix.tex:14` — WRITE: the clip index, summarising the eighty-seven recorded runs and their automatic check results
- [ ] `main.tex:96` — 
- [ ] `main.tex:110` — WRITE: abstract, last, once the Results chapter is final.
- [ ] `main.tex:115` — WRITE: acknowledgements.
- [ ] `method/method.tex:221` — WRITE: the per-mode description, once the figures exist. The shared autonomy arbiter servos wrist ORIENTATION only and never position, which is the safety argument of that design and should be stated here explicitly.

## CONFIRM (5)

- [ ] `discussion/discussion.tex:158` — CONFIRM: whether ethics approval was sought and its status
- [ ] `introduction/introduction.tex:51` — CONFIRM: whether the divergence analysis should appear as an appendix in the submitted document or remain a supporting file
- [ ] `main.tex:99` — CONFIRM: title with Dr.\ Zhang. Option~2 of three was accepted pending supervisor confirmation; the alternatives and their trade-offs are in \repo{thesis\_report/DIVERGENCE.md} Part~D.
- [ ] `main.tex:102` — CONFIRM: word limit and submission format with Dr.\ Zhang / Prof.\ Burdet. The 12{,}000--15{,}000 word target used here is not authoritative.
- [ ] `method/method.tex:36` — CONFIRM: the potentiometer part is a WH148 PH1 single-turn unit, from the CAD in Downloads --- confirm the ADC resolution and pin map from \repo{teensy\_final.ino}, which was located but not yet ingested
