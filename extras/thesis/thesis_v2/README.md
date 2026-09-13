# Thesis draft

Self-contained LaTeX project. Upload the whole folder to Overleaf, or build
locally.

**Compiler: XeLaTeX.** On Overleaf: Menu > Settings > Compiler > XeLaTeX. The
document uses `fontspec` to select Times New Roman where it is available and
TeX Gyre Termes, which is metric-compatible, where it is not.

Locally:

    xelatex main && bibtex main && xelatex main && xelatex main

## Relationship to `extras/thesis/thesis_report/`

This is a revision of the draft in `extras/thesis/thesis_report/`, not a replacement for it.
That folder is untouched. The chapters carried over from it are the master
arm, the development history, the system description, the instrument chapter,
the results and the workspace characterisation. What is new here:

* three new chapters covering work done after that draft was written:
  perception and environment mapping, motion generation and
  simulation-to-hardware transfer, and the experimental design as its own
  chapter rather than a section;
* a participant results chapter that is complete in structure and empty of
  data;
* nine new flow charts and diagrams, sharing the drawing vocabulary already
  used by the four data-path diagrams;
* reserved space, drawn as dashed boxes in the compiled PDF, for the
  screenshots and photographs that still need to be taken;
* three data figures regenerated from the stored measurement records, because
  the inherited versions were empty or mislabelled.

## Layout

    main.tex              document setup, packages, drawing styles, the
                          reserved-space boxes
    title/title.tex       title page
    chapters/             one file per chapter, in reading order
    figures/              data figures as PDF
    figures/diagrams/     the TikZ data-path and design diagrams
    figures/tikz/         the TikZ flow charts added in this revision
    figures/make_figures.py
                          regenerates three data figures from the records
    bibs/references.bib   references
    CRITERIA.md           the evaluation criteria, also reproduced as an
                          appendix

## Reserved space

Three kinds of box appear in the compiled document so that a missing item is
visible rather than silently absent:

* **red**, an image that is required (photographs of the platform and the
  master arm, an annotated view at the home pose);
* **blue**, a screenshot of the simulator, the operating window or a live
  view;
* **orange**, a figure to be reproduced from published work, which needs
  written permission first.

Each box states what the image has to show. Replace the command with
`\includegraphics` when the file exists.

## Regenerating the data figures

From the repository root:

    .venv_vision/bin/python extras/thesis/thesis_v2/figures/make_figures.py

This reads `recordings/baselines/` and writes `channels.pdf`,
`reach_directions.pdf` and `clearance_tasks.pdf`.

## Length

Approximately 29,000 body words. If a 12,000-15,000 limit is confirmed, the
intended reduction is structural: move the master arm, development history and
instrument chapters to appendices, which keeps them examinable while removing
about 5,500 words from the body.
