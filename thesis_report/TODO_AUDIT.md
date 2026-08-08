# TODO AUDIT

Every `\todo{}` in the thesis, grouped so it can be worked as a checklist.
Regenerate with `make todos` for the raw list, or re-run the generator in
this file's history.

**Total: 43**

| category | count | meaning |
| --- | --- | --- |
| **CITE** | 24 | a reference must be found and verified; none may be invented |
| **MEASURE** | 6 | an experiment or logged run is required to produce the number |
| **WRITE** | 8 | prose, a figure or a table still to be produced |
| **CONFIRM** | 5 | a fact only the author can settle |

---

## CITE (24)

- [ ] `background/background.tex:6` — CITE: an origin paper for supernumerary robotic limbs -- Parietti and Asada, bracing and body support
- [ ] `background/background.tex:10` — CITE: supernumerary fingers -- Prattichizzo et al.
- [ ] `background/background.tex:11` — CITE: a recent survey or quantitative review of SRL design and embodiment
- [ ] `background/background.tex:18` — CITE: exploiting body redundancy to control supernumerary robotic limbs -- Lisini Baldi et al., IJRR 2025
- [ ] `background/background.tex:20` — CITE: shared control of SRLs using mixed reality and mouth-and-tongue interfaces, Biosensors 2025
- [ ] `background/background.tex:23` — CITE: body schema / embodiment of extra limbs
- [ ] `background/background.tex:36` — CITE: Saraiji, Sasaki, Matsumura, Minamizawa and Inami, Fusion: full body surrogacy for collaborative communication, ACM SIGGRAPH 2018 Emerging Technologies, Article 7
- [ ] `background/background.tex:48` — CITE: SRL Proxemics: Spatial Guidelines for Supernumerary Robotic Limbs in Near-Body Interactions, CHI 2026
- [ ] `background/background.tex:62` — CITE: motion/force control of mobile manipulators via extended uncertainty and disturbance estimation
- [ ] `background/background.tex:66` — CITE: Zhang et al. 2024, motion-compensation control of SRAs subject to human-induced disturbances
- [ ] `background/background.tex:68` — CITE: A Human Motion Compensation Framework for a Supernumerary Robotic Arm, arXiv:2310.10029
- [ ] `background/background.tex:78` — CITE: Dragan and Srinivasa, formalising assistive teleoperation / policy blending
- [ ] `background/background.tex:83` — CITE: Javdani et al., shared autonomy via hindsight optimisation
- [ ] `background/background.tex:84` — CITE: evidence on when blending helps and when it hurts
- [ ] `background/background.tex:89` — CITE: Kinova Gen3 technical specification
- [ ] `background/background.tex:96` — CITE: ROS 2
- [ ] `background/background.tex:96` — CITE: MoveIt 2
- [ ] `background/background.tex:97` — CITE: Beeson and Ames, TRAC-IK
- [ ] `background/background.tex:107` — CITE: YOLO-World open-vocabulary detection
- [ ] `background/background.tex:108` — CITE: Grounding DINO
- [ ] `background/background.tex:110` — CITE: Whisper / faster-whisper
- [ ] `background/background.tex:113` — CITE: OpenVLA or a comparable VLA, for the comparison being declined
- [ ] `introduction/introduction.tex:14` — CITE: Fusion, Saraiji et al., SIGGRAPH 2018 Emerging Technologies -- the direct predecessor for the operator/wearer architecture
- [ ] `introduction/introduction.tex:51` — CITE: Zhang et al., Motion-Compensation Control of Supernumerary Robotic Arms Subject to Human-Induced Disturbances, Advanced Intelligent Systems 2024

---

## MEASURE (6)

- [ ] `conclusion/conclusion.tex:47` — MEASURE: the full experimental programme -- no human participant data exists
- [ ] `method/method.tex:65` — MEASURE: the residual lateral mapping error after the potentiometer wiring is repaired, since the present figure is confounded by incoherent channels
- [ ] `result/result.tex:23` — MEASURE: re-run the real-arm bring-up with logging to file, so the achieved control rate, homing residuals and session behaviour become citable
- [ ] `result/result.tex:151` — MEASURE: capture these trip traces to file; the figures above are from the engineering log and no artefact was retained
- [ ] `result/result.tex:164` — MEASURE: final counts for the four-view re-recording --- clips passing the content check, clips in which the object travels with the arm, and clips in which the gripper opens, closes on the object and reopens
- [ ] `result/result.tex:219` — MEASURE: save this probe's output to a file --- it currently prints to standard output only

---

## WRITE (8)

- [ ] `appendix/appendix.tex:4` — WRITE: package-by-package table of the repository, with the role of each node -- generatable from the source tree
- [ ] `appendix/appendix.tex:9` — WRITE: reproduce the verified scenario coordinates from recordings/baselines/scenario\_audit.yaml as a table
- [ ] `appendix/appendix.tex:14` — WRITE: the clip index, summarising the eighty-seven recorded runs and their automatic check results
- [ ] `main.tex:57` — 
- [ ] `main.tex:65` — WRITE: abstract, last, once the Results chapter is final.
- [ ] `main.tex:70` — WRITE: acknowledgements.
- [ ] `result/result.tex:53` — WRITE: figure -- polar or 3-D plot of the 26-direction reach envelope per arm, generated from workspace\_n10\_20260806.json
- [ ] `result/result.tex:101` — WRITE: figure -- per-channel classification with dropout fraction and jump rate, generated from channels\_20260806.json

---

## CONFIRM (5)

- [ ] `discussion/discussion.tex:121` — CONFIRM: whether ethics approval was sought, and its status, since this determines how the design is described
- [ ] `introduction/introduction.tex:103` — CONFIRM: that the contribution list matches what the final Results chapter supports, once the outstanding measurements are taken
- [ ] `main.tex:60` — CONFIRM: project title, author name, supervisor and co-supervisor on the title page, and the Imperial logo (run \texttt{epstopdf title/logo.eps} once ghostscript is installed).
- [ ] `method/method.tex:182` — CONFIRM: whether the real-arm bridge should be described in Method at all, given that no artefact from a physical arm exists in the repository
- [ ] `result/result.tex:255` — CONFIRM: total passing test count at submission time

---

## Notes on the three categories that block submission

**CITE** — `bibs/sample.bib` holds one entry, a CTAN article, which is
irrelevant to this subject. Roughly a dozen sources in
`docs/research/01_literature_review.md` §7 were located and checked against a
publisher record and can seed the bibliography; the remainder must be found.
No reference anywhere in this draft was invented.

**MEASURE** — these are experiments, not writing tasks. The largest is the
entire human-participant programme, for which no data exists. Several others
are cheap: a number of probe scripts print results to standard output and save
nothing, so re-running them with file output would make figures already quoted
in the engineering log properly citable.

**CONFIRM** — facts only the author can settle: name, supervisor,
co-supervisor, project title, and the status of ethics approval.

