# Final report

Built on the department's own **"Imperial College Individual Project
Template_LaTeX"** (title page, document class, page geometry, chapter order,
bibliography style) rather than the custom template `thesis_v2/` used.
Directory names match the template's own: `introduction/`, `background/`,
`method/`, `result/`, `discussion/`, `conclusion/`, `appendix/`, `title/`,
`bibs/`.

**Compiler: XeLaTeX** (the template's own `fontspec` / Times New Roman
request needs it):

    xelatex main && bibtex main && xelatex main && xelatex main

## Current state

- **Main body: 5,930 words, 4 figures/tables** (Introduction, Background,
  Method, Result, Discussion, Conclusion) -- against the booklet's limit of
  6,000 words / 20 figures. Word count is read from the *typeset* PDF
  (`Chapter 1` to the page before `Appendix A`), not estimated from the
  LaTeX source, because `\SI{}{}`/`\cref{}` expand into several rendered
  words each.
- **Abstract: 248/250 words.**
- **62 pages total** (title, abstract, acknowledgements, contents, 6 main
  chapters, 14 appendix chapters, bibliography). `thesis_v2` ran to 158
  pages; this is a restructuring of the same underlying content into the
  booklet's word/figure ceiling for the body plus a genuinely condensed
  appendix, not a re-write from scratch.
- **Figures are matplotlib PDFs in the same house style as the platform's
  own data figures** (`figures/make_figures.py`'s `channels()`/`reach()`/
  `clearance()`: muted grey/blue/red, `font.size` 8-9, no display title
  baked into the image -- the LaTeX caption carries that -- collages built
  from small individual panels via `subcaption`, matching how
  `appendix/a_master.tex` already composed the master-arm CAD figure).

## The pilot data

Two participant cohorts, both anonymised (P1-P4 VR, M1-M3 master-arm; never
a real name, including in rendered figure pixels -- checked by grepping
every figure PDF's extracted text before each commit).

- **P1-P4**, VR controllers, 16 sessions, `sec:pilot-results` (Result) +
  `app:pilot` (Appendix L, `appendix/l_pilot_study.tex`).
- **M1-M3**, the instrumented mannequin master, 9 recorded sessions out of
  16 attempted -- the other 7 are confirmed empty or metadata-less via
  `ros2 bag info`, not an extraction gap. M1's three sessions track the
  simulated joint 15.9-21.2° RMS against M2/M3's under 3°, an order of
  magnitude worse and consistent with this project's own channel-health
  finding; `appendix/l_pilot_study.tex` \S L.2.
- Every pilot session polled the real Kinova arms' own joint encoders
  alongside the simulated ones, which is a live sim-to-real check, not an
  offline replay: the best-tracking session holds 1.06° RMS
  (`figures/make_tracking_figure.py`, `fig:track`).

Regenerate: `python3 thesis_v3/figures/make_pilot_figures.py`,
`make_tracking_figure.py`, `make_master_figure.py`, each from the repository
root. All three re-derive participant anonymisation from the raw session
names before any plot is drawn -- if the pilot data set grows, extend the
`ANON` dict in each script rather than plotting first and anonymising after.

## Checking compliance before submission

    python3 - <<'EOF'
    import pymupdf
    doc = pymupdf.open("main.pdf")
    texts = [p.get_text() for p in doc]
    # find "Chapter 1" and "Appendix A" page indices, word-count the text
    # between them -- that is the booklet's counted body.
    EOF

Re-run after any edit to `introduction/`, `background/`, `method/`,
`result/`, `discussion/` or `conclusion/`: 6,000 words and 20 figures/tables
are department policy, not house style, and this report currently has
~70 words of headroom.
