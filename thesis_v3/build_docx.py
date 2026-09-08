#!/usr/bin/env python3
"""Build an editable Word version of the report from the LaTeX source.

Pandoc does the LaTeX-to-docx conversion; this script does what pandoc
cannot, so the Word file reads like the PDF:

  * every \\cref/\\Cref/\\ref/\\eqref/\\subref is resolved to the NUMBER the
    LaTeX build assigned (from main.aux), so "Figure 3.2" in Word is the
    same figure as in the PDF;
  * every caption gets its "Figure 3.2:" / "Table 3.1:" prefix and every
    heading its chapter/section number (from main.aux and main.toc);
  * numbered equations become display math with the number appended;
  * TikZ diagrams (\\input{figures/diagrams/*.tex}) are pre-rendered to PNG
    with the report's own preamble, PDF figures are rasterised to PNG, and
    the paths are rewritten (Word cannot embed PDF or TikZ);
  * the template's title page is rebuilt as a plain front page, the abstract
    and acknowledgements become unnumbered headings, and the bibliography is
    produced by citeproc in IEEE numeric style, matching the PDF's [n].

Prerequisites: a completed XeLaTeX build (main.aux, main.toc), pandoc on
PATH, python-docx, and the build directory prepared by the calling shell
(rendered diagrams under <build>/diagrams, figures under <build>/figs,
reference.docx and ieee.csl in <build>).

    python3 thesis_v3/build_docx.py <build_dir> <out.docx>
"""
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
BUILD = sys.argv[1]
OUT = sys.argv[2]

# --------------------------------------------------------------------------
# 1. the numbers LaTeX assigned
# --------------------------------------------------------------------------
aux = open(os.path.join(HERE, "main.aux"), encoding="utf-8", errors="replace").read()
labels, kinds = {}, {}
for m in re.finditer(r"\\newlabel\{([^}]+)\}\{\{([^}]*)\}", aux):
    name, val = m.group(1), m.group(2)
    if name.endswith("@cref"):
        base = name[:-5]
        mm = re.match(r"\[([a-zA-Z]*)\]\[[^\]]*\]\[[^\]]*\](.*)", val)
        if mm:
            kinds[base] = mm.group(1)
            labels.setdefault(base, mm.group(2))
    else:
        labels.setdefault(name, val)
KIND_WORD = {"figure": "Figure", "table": "Table", "equation": "Eq.", "chapter": "Chapter", "appendix": "Appendix",
             "section": "Section", "subsection": "Section", "subsubsection": "Section", "subfigure": "Figure",
             "item": "item", "enumi": "item", "algorithm": "Algorithm", "listing": "Listing"}


def refword(lab, cap):
    k = kinds.get(lab, "")
    w = KIND_WORD.get(k, "Section")
    if k == "chapter" and labels.get(lab, "").isalpha():
        w = "Appendix"
    if not cap:
        w = w[0].lower() + w[1:] if w not in ("Eq.",) else w
    return w


def resolve_cref(m, cap):
    labs = [x.strip() for x in m.group(1).split(",")]
    parts = []
    for lab in labs:
        num = labels.get(lab)
        if num is None:
            parts.append("[%s]" % lab)
            continue
        if kinds.get(lab) == "equation":
            parts.append("%s (%s)" % ("Eq." if True else "eq.", num))
        else:
            parts.append("%s %s" % (refword(lab, cap), num))
    if len(parts) == 1:
        return parts[0]
    # same kind: "Figures 3.1 and 3.2"
    ks = {kinds.get(l) for l in labs}
    if len(ks) == 1 and len(labs) > 1 and list(ks)[0] not in ("equation",):
        w = refword(labs[0], cap)
        w = {"Appendix": "Appendices", "appendix": "appendices", "Eq.": "Eqs."}.get(w, w + "s")
        nums = [labels.get(l, "?") for l in labs]
        return "%s %s and %s" % (w, ", ".join(nums[:-1]), nums[-1])
    return ", ".join(parts[:-1]) + " and " + parts[-1]


# --------------------------------------------------------------------------
# 2. flatten the source
# --------------------------------------------------------------------------
def read(path):
    return open(os.path.join(HERE, path), encoding="utf-8").read()


def strip_comments(s):
    return re.sub(r"(?<!\\)%.*", "", s)


def flatten(s):
    def repl(m):
        p = m.group(1)
        if not p.endswith(".tex"):
            p += ".tex"
        if p.startswith("figures/diagrams/"):
            png = os.path.join("diagrams", os.path.basename(p)[:-4] + ".png")
            return "\\includegraphics[width=\\linewidth]{%s}" % png
        if p.startswith("title/"):
            return ""
        return flatten(strip_comments(read(p)))
    return re.sub(r"\\input\{([^}]+)\}", repl, s)


main = strip_comments(read("main.tex"))
body = main[main.index("\\begin{document}") + len("\\begin{document}"):main.index("\\end{document}")]
body = flatten(body)

# title page and abstracts -> plain front matter
title = re.search(r"\\title\{(.*?)\}", main, re.S).group(1).replace("\n", " ")
author = re.search(r"\\author\{(.*?)\}", main).group(1)
front = r"""
\begin{center}
\includegraphics[width=8cm]{figs/title/logo.png}

\vspace{1cm}
{\Large MSC INDIVIDUAL PROJECT}

{\Large IMPERIAL COLLEGE LONDON}

DEPARTMENT OF BIOENGINEERING

\vspace{1cm}
{\huge \textbf{%s}}

\vspace{1cm}
\textit{Author:} %s

\textit{Supervisor:} Dr.\ Dandan Zhang

\textit{Co-Supervisor:} Dr.\ Etienne Burdet

\vspace{1cm}
September 2026
\end{center}
\newpage
""" % (title, author)
body = re.sub(r"\\begin\{abstract\}", "\\\\chapter*{Abstract}", body, count=1)
body = re.sub(r"\\renewcommand\{\\abstractname\}\{Acknowledgements\}\s*\\begin\{abstract\}", "\\\\chapter*{Acknowledgements}", body)
body = body.replace("\\end{abstract}", "")
body = re.sub(r"\\tableofcontents|\\listoffigures|\\listoftables", "", body)
body = re.sub(r"\\bibliographystyle\{[^}]*\}|\\bibliography\{[^}]*\}", "", body)
body = front + body

# --------------------------------------------------------------------------
# 3. numbers into headings, captions, equations, references
# --------------------------------------------------------------------------
toc = open(os.path.join(HERE, "main.toc"), encoding="utf-8", errors="replace").read()
entries = re.findall(r"\\contentsline \{(chapter|section|subsection)\}\{\\numberline \{([^}]*)\}(.*?)\}\{\d+\}\{", toc)
ptr = [0]


def head(m):
    level, star, ttl = m.group(1), m.group(2), m.group(3)
    if star:
        return m.group(0)
    if level == "subsubsection":
        return m.group(0)
    # take the next toc entry of this level (they appear in document order)
    while ptr[0] < len(entries) and entries[ptr[0]][0] != level:
        ptr[0] += 1
    if ptr[0] < len(entries):
        num = entries[ptr[0]][1]; ptr[0] += 1
        return "\\%s{%s %s}" % (level, num, ttl)
    return m.group(0)


body = re.sub(r"\\(chapter|section|subsection|subsubsection)(\*?)\{((?:[^{}]|\{[^{}]*\})*)\}", head, body)


def caption_prefix(env_text):
    """Prefix the caption of one figure/table environment with its number."""
    lab = None
    for m in re.finditer(r"\\label\{([^}]+)\}", env_text):
        if kinds.get(m.group(1)) in ("figure", "table"):
            lab = m.group(1); break
    def cap(m):
        text = m.group(2)
        if lab and lab in labels:
            word = "Figure" if kinds[lab] == "figure" else "Table"
            return "\\caption{%s %s: %s}" % (word, labels[lab], text)
        return "\\caption{%s}" % text
    # only the environment's own caption, not sub-captions: split off subfigures first
    subs = list(re.finditer(r"\\begin\{subfigure\}.*?\\end\{subfigure\}", env_text, re.S))
    protected = env_text
    keep = {}
    for i, sm in enumerate(subs):
        key = "@@SUB%d@@" % i; keep[key] = sm.group(0); protected = protected.replace(sm.group(0), key)
    protected = re.sub(r"\\caption(\[[^\]]*\])?\{((?:[^{}]|\{(?:[^{}]|\{[^{}]*\})*\})*)\}", cap, protected, count=1)
    for key, val in keep.items():
        # sub-captions: letter from the label
        sl = re.search(r"\\label\{([^}]+)\}", val)
        letter = ""
        if sl and sl.group(1) in labels:
            letter = "(%s) " % labels[sl.group(1)][-1]
        val = re.sub(r"\\caption\{", "\\\\caption{" + letter, val, count=1)
        protected = protected.replace(key, val)
    return protected


body = re.sub(r"\\begin\{(figure|table)\}.*?\\end\{\1\}", lambda m: caption_prefix(m.group(0)), body, flags=re.S)


def eq(m):
    inner = m.group(1)
    lm = re.search(r"\\label\{([^}]+)\}", inner)
    inner = re.sub(r"\\label\{[^}]+\}", "", inner).strip().rstrip(",.")
    num = labels.get(lm.group(1)) if lm else None
    tag = " \\qquad (%s)" % num if num else ""
    return "\\[ %s%s \\]" % (inner, tag)


body = re.sub(r"\\begin\{equation\}(.*?)\\end\{equation\}", eq, body, flags=re.S)
body = re.sub(r"\\Cref\{([^}]+)\}", lambda m: resolve_cref(m, True), body)
body = re.sub(r"\\cref\{([^}]+)\}", lambda m: resolve_cref(m, False), body)
body = re.sub(r"\\eqref\{([^}]+)\}", lambda m: "(%s)" % labels.get(m.group(1), "?"), body)
body = re.sub(r"\\subref\{([^}]+)\}", lambda m: labels.get(m.group(1), "?")[-1], body)
body = re.sub(r"\\ref\{([^}]+)\}", lambda m: labels.get(m.group(1), "?"), body)
body = re.sub(r"\\label\{[^}]+\}", "", body)

# figure paths -> the rasterised copies
def fig(m):
    opts, p = m.group(1) or "", m.group(2)
    if p.startswith("diagrams/") or p.startswith("figs/"):
        return m.group(0)
    q = p
    if q.lower().endswith(".pdf"):
        q = q[:-4] + ".png"
    elif not q.lower().endswith((".png", ".jpg", ".jpeg")):
        q = q + ".png"
    return "\\includegraphics%s{figs/%s}" % (opts, q)


body = re.sub(r"\\includegraphics(\[[^\]]*\])?\{([^}]+)\}", fig, body)
body = body.replace("\\appendix", "")
body = re.sub(r"\\clearpage|\\newpage(?!\n\\end)", "", body)  # keep the one after the front page? pandoc ignores anyway
body = re.sub(r"\\begin\{figure\}\[[^\]]*\]", "\\\\begin{figure}", body)
body = re.sub(r"\\begin\{table\}\[[^\]]*\]", "\\\\begin{table}", body)
body = body.replace("\\centering", "")

preamble = r"""\documentclass{report}
\usepackage{graphicx,siunitx,booktabs,amsmath,amssymb,subcaption,multirow,array}
\newcommand{\repo}[1]{\texttt{#1}}
\newcommand{\HRule}{}
\begin{document}
"""
src = preamble + body + "\n\\end{document}\n"
tex_out = os.path.join(BUILD, "thesis_flat.tex")
open(tex_out, "w", encoding="utf-8").write(src)
print("flattened source:", tex_out, len(src), "chars;", len(entries), "toc entries,", ptr[0], "used")

# --------------------------------------------------------------------------
# 4. pandoc
# --------------------------------------------------------------------------
cmd = ["pandoc", tex_out, "-f", "latex", "-t", "docx", "-o", OUT,
       "--reference-doc", os.path.join(BUILD, "reference.docx"),
       "--resource-path", BUILD, "--top-level-division=chapter",
       "--citeproc", "--bibliography", os.path.join(HERE, "bibs", "references.bib"),
       "--csl", os.path.join(BUILD, "ieee.csl"), "--metadata", "reference-section-title=Bibliography",
       "--metadata", "link-citations=false"]
r = subprocess.run(cmd, capture_output=True, text=True, cwd=BUILD)
warn = [l for l in r.stderr.splitlines() if l.strip()]
print("pandoc exit", r.returncode, "warnings", len(warn))
for l in warn[:15]:
    print("  ", l[:160])
if r.returncode:
    sys.exit(1)

# --------------------------------------------------------------------------
# 5. tidy in Word terms: chapters start on a new page, captions italic-free
# --------------------------------------------------------------------------
from docx import Document  # noqa: E402
from docx.shared import Pt  # noqa: E402
from docx.oxml import OxmlElement  # noqa: E402
from docx.oxml.ns import qn  # noqa: E402
d = Document(OUT)
from docx.enum.text import WD_ALIGN_PARAGRAPH  # noqa: E402
d.styles["Heading 1"].paragraph_format.page_break_before = True
from docx.shared import RGBColor  # noqa: E402
for p in d.paragraphs:
    if p.style.name.startswith("Heading"):
        for r in p.runs:
            r.font.color.rgb = RGBColor(0, 0, 0)
paras = d.paragraphs
first_h1 = next(p for p in paras if p.style.name == "Heading 1")
# the front page: centre everything before the first heading and size it like the template
front_paras = []
for p in paras:
    if p is first_h1:
        break
    front_paras.append(p)
for p in front_paras:
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    t = p.text.strip()
    for r in p.runs:
        if t in ("MSC INDIVIDUAL PROJECT", "IMPERIAL COLLEGE LONDON"):
            r.font.size = Pt(20); r.font.small_caps = True
        elif t == "DEPARTMENT OF BIOENGINEERING":
            r.font.size = Pt(14); r.font.small_caps = True
        elif r.bold:
            r.font.size = Pt(20)
        else:
            r.font.size = Pt(13)
    p.paragraph_format.space_after = Pt(14)
# a real Word table of contents on its own page (Word fills it on open / F9)
toc_style = "TOC Heading" if "TOC Heading" in [st.name for st in d.styles] else "Title"
toc_head = first_h1.insert_paragraph_before("Contents", style=toc_style)
toc_head.paragraph_format.page_break_before = True
toc_head.alignment = WD_ALIGN_PARAGRAPH.CENTER
for r in toc_head.runs:
    r.font.size = Pt(20); r.font.bold = True; r.font.color.rgb = None
fld = first_h1.insert_paragraph_before()
run = fld.add_run()
for tag, text in (("begin", None), ("instr", ' TOC \\o "1-1" \\h \\z \\u '), ("separate", None), ("text", "Right-click and choose Update Field to build the table of contents."), ("end", None)):
    if tag in ("begin", "separate", "end"):
        e = OxmlElement("w:fldChar"); e.set(qn("w:fldCharType"), tag); run._r.append(e)
    elif tag == "instr":
        e = OxmlElement("w:instrText"); e.set(qn("xml:space"), "preserve"); e.text = text; run._r.append(e)
    else:
        e = OxmlElement("w:t"); e.text = text; run._r.append(e)
# ask Word to refresh fields when the file is opened
settings = d.settings.element
uf = OxmlElement("w:updateFields"); uf.set(qn("w:val"), "true"); settings.append(uf)
d.save(OUT)
print("wrote", OUT, os.path.getsize(OUT) // 1024, "kB")
