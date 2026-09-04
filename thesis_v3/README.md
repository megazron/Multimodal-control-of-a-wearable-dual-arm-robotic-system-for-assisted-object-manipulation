# Final report (submission-ready structure)

Self-contained LaTeX project. Upload the whole folder to Overleaf, or build
locally.

**Compiler: XeLaTeX.** On Overleaf: Menu > Settings > Compiler > XeLaTeX. The
document uses `fontspec` to select Times New Roman where it is available and
TeX Gyre Termes, which is metric-compatible, where it is not.

Locally:

    xelatex main && bibtex main && xelatex main && xelatex main

## Relationship to `thesis_v2/`

`thesis_v2/` compiled to roughly 30,600 words and 60 figures/tables in its
main body. The Department of Bioengineering MSc project booklet
(`MSc_Project_booklet_2025-26.pdf`, section "C) The final project report") caps the
**final report body at 6,000 words and 20 figures/tables**; anything over
that is unlikely to be marked above 59% regardless of content. `thesis_v2`
was written before that limit was checked against the actual booklet.

`thesis_v3` is a restructuring, not a rewrite from scratch: the booklet's
required shape is **Abstract (<=250 words) > Introduction > Methods >
Results > Discussion > Conclusion > References**, with everything else
movable to an **Appendix that carries neither word nor figure limit**. So:

* `chapters/01_introduction.tex` through `05_conclusion.tex` are new,
  compressed chapters written to fit inside the booklet's limits. Current
  main body: **5,611 words, 6 figures/tables** (see the title page's own
  count and footnote for the counting method).
* Every chapter from `thesis_v2` that didn't fit that budget moved into
  `chapters/appendix_*.tex`, unabridged, in booklet-sanctioned "extra
  material... to allow you to disseminate all the necessary information to
  someone who might want to repeat your work" fashion. Nothing was deleted.
* `chapters/appendix_o_pilot_study.tex` is new: the full record of the pilot
  user study (`sec:pilot-results` in the main body has the headline numbers).
  Four operators, sixteen sessions, direct VR control against
  shared-autonomy assistance -- the first real-operator data this project
  has, run under ICL SETREC approval 7111986. Participants are anonymised as
  P1-P4 throughout, including in every rendered figure pixel (`figures/pilot/`
  was regenerated from anonymised source data -- check any new figure added
  here for a leaked name before publishing it).
* The title-page word count and the `\chaptertitlename` fix (appendices now
  correctly say "Appendix A", not "Chapter A" -- a pre-existing defect
  inherited from `thesis_v2`) are the only changes to `main.tex` beyond the
  chapter list and the two duplicate-label fixes in `appendix_j_workspace.tex`
  (`fig:lateral`, `fig:rviz-home` each existed twice under thesis_v2's
  structure; the appendix copies were renamed).

## Layout

    main.tex                      document setup, packages, drawing styles
    title/title.tex                title page, including the word-count line
    chapters/00_frontmatter.tex    abstract (<=250 words) and acknowledgements
    chapters/01-05_*.tex           the booklet-compliant main body
    chapters/appendix_[a-o]_*.tex  everything else, in the order it is first
                                   referenced from the main body
    figures/                       as thesis_v2, plus figures/pilot/ (new)
    figures/make_figures.py        regenerates three inherited data figures
    bibs/references.bib            references

## Regenerating the pilot figures

The nine pilot figures in `figures/pilot/` are produced by a script that
lives outside this directory (it reads raw session recordings under
`recordings/sessions/`, not tracked here). If the pilot data set grows, the
figures must be regenerated from participant-anonymised data -- the script
maps real session names to P1, P2, ... before any plot is drawn, and that
mapping must be applied before regenerating, not after.

## Checking compliance before submission

    python3 - <<'EOF'
    import pymupdf
    doc = pymupdf.open("main.pdf")
    texts = [p.get_text() for p in doc]
    # find the Chapter 1 / Appendix A page boundary and print the word count
    # of everything between them (the booklet's counted body).
    EOF

Re-run this after any edit to `chapters/01_introduction.tex` through
`05_conclusion.tex`: the 6,000-word and 20-figure ceilings are hard
department policy, not house style.
