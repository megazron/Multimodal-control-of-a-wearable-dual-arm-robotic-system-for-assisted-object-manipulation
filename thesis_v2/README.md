# Thesis draft

Self-contained LaTeX project. Upload the whole folder to Overleaf, or build
locally.

**Compiler: XeLaTeX.** On Overleaf: Menu > Settings > Compiler > XeLaTeX. The
document uses `fontspec` to select Times New Roman where it is available and
TeX Gyre Termes, which is metric-compatible, where it is not.

Locally:

    xelatex main
    bibtex main
    xelatex main
    xelatex main

## Layout

    main.tex              document setup, packages, the drawing styles used by
                          every flow chart, and the placeholder boxes
    title/title.tex       title page
    chapters/             one file per chapter, in reading order
    figures/              data figures as PDF, plus figures/tikz/ for the
                          drawn ones
    figures/make_figures.py
                          regenerates the three data figures from the stored
                          measurement records
    bibs/references.bib   references

## Placeholders

Three kinds of reserved space appear in the compiled document as dashed boxes,
so a missing item is visible rather than silently absent:

* **blue** for a screenshot that still needs to be taken,
* **grey** for a photograph of the physical platform,
* **orange** for a figure to be reproduced from published work, which needs
  permission first.

Each box says what the image has to show. Replace the command with
`\includegraphics` when the file exists.

Chapter 9 is the participant results chapter. It is deliberately empty: every
table and figure the study would produce is laid out, with dashes where the
numbers go.

## Regenerating the data figures

From the repository root:

    .venv_vision/bin/python thesis_v2/figures/make_figures.py

This reads the stored measurement records under `recordings/baselines/` and
writes `channels.pdf`, `reach_directions.pdf` and `clearance_tasks.pdf`. No
figure in this thesis is drawn by hand from numbers typed out of a table.
