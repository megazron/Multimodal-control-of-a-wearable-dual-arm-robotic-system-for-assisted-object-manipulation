# The system report

`../SYSTEM_REPORT_2026-08-26.pdf` — 202 pages: the narrative account
(chapters 1–16), the defect register (chapter 17, 265 numbered faults), the
full change log (chapter 18, all 392 commits) and the program inventory
(appendices A–C).

Rebuild:

    python3 gen_programs.py     # regenerates gen_programs.tex from src/ and scripts/
    python3 gen_changes.py      # regenerates gen_changes.tex from git log
    pdflatex report.tex         # three times, for the contents and the longtables
    cp report.pdf ../SYSTEM_REPORT_2026-08-26.pdf

Both generators are run from the repository root. `gen_failures.tex` is
written by hand from `docs/system/findings.md` and is not generated.
