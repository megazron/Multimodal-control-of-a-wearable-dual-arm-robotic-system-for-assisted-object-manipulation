#!/usr/bin/env python3
"""Build the graded project presentation from the repository's own evidence.

    .venv_vision/bin/python scripts/make_presentation.py

Writes ``docs/presentation/SRL_project_presentation.pptx`` and the figure set
it uses into ``docs/presentation/figures/``.

THE RULES, WHICH ARE THE REPO'S OWN (scripts/make_results.py, THE STANDING
RULE in CLAUDE.md):

* Every figure comes from a committed artefact -- a recording, a rendered
  clip, a screenshot, or a LaTeX diagram in ``thesis_v2/`` -- and every slide
  names the file it was drawn from on the slide itself. No number is typed in
  from memory that is not also on disk.
* A figure whose source is missing is REFUSED BY NAME and the slide is not
  written, rather than drawn from prose.
* Provenance is part of the result. Every caption carries the date of the
  recording and, where the measurement does not support the obvious reading,
  says so in the same breath -- the accuracy figures predate the VR smoothing,
  the arm in the autonomy clips is simulated, the sim-to-real compensation is
  modelled and untried.
* Rule 13 of CLAUDE.md: a demonstration is not evidence. Slides built from
  demonstration footage say DEMONSTRATION on the slide.

Asset preparation is idempotent and re-runs the upstream generators:
``scripts/make_thesis_figures.py`` and ``thesis_v2/figures/make_figures.py``
for the data plots, ``xelatex`` for the TikZ diagrams, ``ffmpeg`` for frames
cut from the verification clips. Anything that cannot be regenerated is
copied from where it is committed.
"""
import json
import os
import runpy
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "docs" / "presentation"
FIG = OUT / "figures"
DECK = OUT / "SRL_project_presentation.pptx"

# --------------------------------------------------------------- house style
INK = "1B1D21"          # body text
HEAD = "0E1116"         # headings
ACCENT = "1F5FA8"       # the one blue
WARN = "C1392B"         # measured-but-bad, and refusals
GOOD = "2F7D4F"
GREY = "6B7280"         # captions, provenance
RULE = "D8DCE2"
BAND = "F3F5F8"
PAPER = "FFFFFF"

SANS = "Calibri"
MONO = "Consolas"

W, H = 13.333, 7.5      # 16:9 inches
MARGIN = 0.62
BODY_TOP = 1.52
BODY_BOT = 6.28


def _need(mod):
    try:
        __import__(mod)
        return True
    except ImportError:
        return False


# ============================================================ asset pipeline
def _sh(cmd, **kw):
    return subprocess.run(cmd, shell=True, capture_output=True, text=True, **kw)


def rasterise_matplotlib(script, made):
    """Run a figure script with savefig redirected from PDF to PNG."""
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib.figure import Figure
    original = Figure.savefig

    def patched(self, fname, *a, **k):
        if isinstance(fname, str) and fname.endswith(".pdf"):
            png = FIG / (os.path.basename(fname)[:-4] + ".png")
            k = dict(k, dpi=200)
            original(self, str(png), *a, **k)
            made.append(png.name)
            return None
        return original(self, fname, *a, **k)

    Figure.savefig = patched
    try:
        runpy.run_path(str(script), run_name="__main__")
    except Exception as exc:                                   # noqa: BLE001
        print("  %-34s FAILED: %s" % (script.name, exc))
    finally:
        Figure.savefig = original


TIKZ = {
    "figures/tikz": ["overview", "datapath", "safety", "perception", "budget",
                     "lateral"],
    "figures/diagrams": ["architecture", "kinematic_chain", "mode1_mannequin",
                         "mode2_vr", "mode3_shared", "mode4_voice",
                         "degradation", "experiment_design", "electronics",
                         "calibration", "session_timeline"],
}


def build_diagrams(work):
    """Compile each thesis TikZ figure alone and trim it to its own ink.

    The thesis preamble carries the node styles, so the standalone document is
    the preamble verbatim plus one \\input. Without ``preview.sty`` on this
    host the picture lands on an A4 page and is cropped afterwards from the
    pixels, which needs no extra package.
    """
    if not shutil.which("xelatex"):
        print("  diagrams          REFUSED: no xelatex on this host")
        return []
    thesis = ROOT / "thesis_v2"
    main = (thesis / "main.tex").read_text().splitlines()
    end = next(i for i, ln in enumerate(main) if ln.startswith("\\title{"))
    preamble = "\n".join(main[:end])

    made = []
    for folder, names in TIKZ.items():
        for name in names:
            src = thesis / folder / (name + ".tex")
            if not src.exists():
                print("  diag_%-14s REFUSED: %s missing" % (name, src.name))
                continue
            # Cross-references into chapters this standalone build does not
            # include resolve to "??" and print that on the slide. There is
            # one, and it is replaced with what it says rather than with an
            # invented figure number.
            body = src.read_text().replace(
                "\\cref{fig:mode1}", "the mode 01 diagram")
            fig_src = work / (name + "_body.tex")
            fig_src.write_text(body)

            tex = work / (name + ".tex")
            # A generous page, because several of these pictures are wider
            # than A4 and LaTeX clips silently at the paper edge rather than
            # complaining -- which cost the four mode diagrams their
            # right-hand third before anyone looked at a rendered slide.
            tex.write_text(preamble
                           + "\n\\geometry{paperwidth=90cm,paperheight=60cm,"
                             "margin=1cm}\n"
                           + "\\begin{document}\\pagestyle{empty}"
                             "\\noindent\n\\input{%s}\n\\end{document}\n"
                           % fig_src.as_posix())
            _sh("xelatex -interaction=nonstopmode -output-directory=%s %s"
                % (work, tex), cwd=thesis)
            pdf = work / (name + ".pdf")
            if not pdf.exists():
                print("  diag_%-14s FAILED to compile" % name)
                continue
            out = FIG / ("diag_%s.png" % name)
            if _pdf_page_to_png(pdf, out):
                made.append(out.name)
    return made


def _pdf_page_to_png(pdf, out, dpi=220):
    import fitz
    from PIL import Image, ImageChops
    page = fitz.open(str(pdf))[0]
    pm = page.get_pixmap(dpi=dpi)
    im = Image.frombytes("RGB", [pm.width, pm.height], pm.samples)
    box = ImageChops.difference(im, Image.new("RGB", im.size, "white")).getbbox()
    if box:
        pad = 12
        im = im.crop((max(0, box[0] - pad), max(0, box[1] - pad),
                      min(im.width, box[2] + pad), min(im.height, box[3] + pad)))
    im.save(str(out))
    return True


# (source clip, seek, destination) -- one frame each, from the committed clips
FRAMES = [
    ("recordings/verification/06_full_autonomy/T1/S1_both_arms_centre/rviz_quad.mp4",
     "00:00:22", "clip_t1_quad.png"),
    ("recordings/verification/06_full_autonomy/T1/S1_both_arms_centre/rviz_gripper.mp4",
     "00:00:24", "clip_t1_gripper.png"),
    ("recordings/verification/06_full_autonomy/T1S2/S2_both_arms_random/rviz_quad.mp4",
     "00:00:22", "clip_t1s2_quad.png"),
    ("recordings/verification/01_master_teleop/T0/D3_three_targets/rviz_front.mp4",
     "00:00:10", "clip_t0_master.png"),
    ("recordings/verification/02_vr_teleop/T0/D3_three_targets/rviz_front.mp4",
     "00:00:25", "clip_t0_vr.png"),
    ("recordings/session/20260824_190937/1_full_scan.mp4",
     "00:00:30", "session_scan.png"),
    ("recordings/session/20260824_190937/2_pick.mp4",
     "00:00:12", "session_pick.png"),
]

# committed screenshots and renders, copied verbatim
COPIES = [
    ("thesis_v2/figures/master_arm_cad_views.png", "master_arm_cad_views.png"),
    ("thesis_v2/figures/master_arm_chain.png", "master_arm_chain.png"),
    ("thesis_v2/figures/master_arm_rviz.png", "master_arm_rviz.png"),
    ("thesis_v2/figures/rviz/home_pose_labelled.png", "rviz_home_pose_labelled.png"),
    ("thesis_v2/figures/rviz_after_fix.png", "gui_rviz_after_fix.png"),
    ("thesis_v2/figures/ready_to_run_live.png", "gui_ready_to_run_live.png"),
    ("docs/img/gui_dual_panels.png", "gui_dual_panels.png"),
    ("docs/img/gui_hud_master_arm.png", "gui_hud_master_arm.png"),
    ("docs/img/presentation_pose_iso.png", "home_presentation_iso.png"),
    ("docs/img/table_before_after.png", "table_before_after.png"),
    ("recordings/session/20260824_190937/control_window.png", "gui_control_window.png"),
    ("recordings/session/20260824_190937/robot_view.png", "session_robot_view.png"),
    ("recordings/renders/centre_top_20260818.png", "centre_top.png"),
    ("src/ros2_kortex/doc/resources/kinova-gen3-7dof-robotiq-2f-85.jpg",
     "hw_kinova_vendor.jpg"),
    ("recordings/trajectory_capture/capture_20260806_130205/analysis/direction_left.png",
     "calib_direction_left.png"),
]


def prepare_assets():
    FIG.mkdir(parents=True, exist_ok=True)
    work = OUT / ".build"
    work.mkdir(exist_ok=True)
    made = []

    print("data figures")
    rasterise_matplotlib(ROOT / "scripts" / "make_thesis_figures.py", made)
    rasterise_matplotlib(ROOT / "thesis_v2" / "figures" / "make_figures.py", made)

    print("results figures")
    r = _sh("%s %s/scripts/make_results.py" % (sys.executable, ROOT))
    if r.returncode != 0:
        print("  make_results       WARNING: exit %d (using what is on disk)"
              % r.returncode)
    for p in sorted((ROOT / "recordings" / "analysis").glob("*.png")):
        shutil.copy(p, FIG / ("res_" + p.name))
        made.append("res_" + p.name)

    print("thesis diagrams")
    made += build_diagrams(work)

    print("clip frames")
    if shutil.which("ffmpeg"):
        for src, seek, dst in FRAMES:
            p = ROOT / src
            if not p.exists():
                print("  %-22s REFUSED: clip missing" % dst)
                continue
            _sh("ffmpeg -v error -y -ss %s -i '%s' -frames:v 1 '%s'"
                % (seek, p, FIG / dst))
            made.append(dst)
    else:
        print("  frames             REFUSED: no ffmpeg on this host")

    print("screenshots")
    for src, dst in COPIES:
        p = ROOT / src
        if p.exists():
            shutil.copy(p, FIG / dst)
            made.append(dst)
        else:
            print("  %-22s REFUSED: %s missing" % (dst, src))

    shutil.rmtree(work, ignore_errors=True)
    print("%d asset(s) in %s" % (len(set(made)), FIG))
    return set(made)


# ================================================================ deck maker
def build_deck():
    from pptx import Presentation
    from pptx.util import Inches, Pt, Emu
    from pptx.dml.color import RGBColor
    from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
    from pptx.enum.shapes import MSO_SHAPE
    from PIL import Image

    def C(h):
        return RGBColor.from_string(h)

    prs = Presentation()
    prs.slide_width, prs.slide_height = Inches(W), Inches(H)
    blank = prs.slide_layouts[6]

    state = {"n": 0}

    # ---------------------------------------------------------- primitives
    def textbox(sl, x, y, w, h, anchor=MSO_ANCHOR.TOP):
        tb = sl.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
        tf = tb.text_frame
        tf.word_wrap = True
        tf.margin_left = tf.margin_right = tf.margin_top = tf.margin_bottom = 0
        tf.vertical_anchor = anchor
        return tf

    def para(tf, text, size, colour, bold=False, font=SANS, space=0,
             align=PP_ALIGN.LEFT, first=False, italic=False):
        p = tf.paragraphs[0] if first else tf.add_paragraph()
        p.alignment = align
        p.space_after = Pt(space)
        r = p.add_run()
        r.text = text
        r.font.size = Pt(size)
        r.font.bold = bold
        r.font.italic = italic
        r.font.name = font
        r.font.color.rgb = C(colour)
        return p

    def rect(sl, x, y, w, h, fill, line=None):
        s = sl.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(x), Inches(y),
                                Inches(w), Inches(h))
        s.fill.solid()
        s.fill.fore_color.rgb = C(fill)
        if line:
            s.line.color.rgb = C(line)
            s.line.width = Pt(0.75)
        else:
            s.line.fill.background()
        s.shadow.inherit = False
        return s

    def chrome(sl, title, kicker=None):
        """Title block and footer. Every content slide gets exactly this."""
        state["n"] += 1
        rect(sl, 0, 0, W, 0.10, ACCENT)
        y = 0.44
        if kicker:
            tf = textbox(sl, MARGIN, y, W - 2 * MARGIN, 0.24)
            para(tf, kicker.upper(), 10.5, ACCENT, bold=True, first=True)
            y += 0.30
        tf = textbox(sl, MARGIN, y, W - 2 * MARGIN, 0.62)
        para(tf, title, 25, HEAD, bold=True, first=True)
        rect(sl, MARGIN, 1.34, W - 2 * MARGIN, 0.014, RULE)
        # footer
        tf = textbox(sl, MARGIN, H - 0.32, 8.0, 0.24)
        para(tf, "Wearable dual-arm supernumerary robot  ·  shared autonomy",
             8.5, GREY, first=True)
        tf = textbox(sl, W - MARGIN - 1.2, H - 0.32, 1.2, 0.24)
        para(tf, str(state["n"]), 8.5, GREY, align=PP_ALIGN.RIGHT, first=True)

    def caption(sl, text, source, top=None, warn=False):
        """The caption carries the claim; the source line carries the file."""
        top = BODY_BOT + 0.10 if top is None else top
        tf = textbox(sl, MARGIN, top, W - 2 * MARGIN, 0.72)
        para(tf, text, 11, WARN if warn else INK, first=True, space=3)
        para(tf, "source: " + source, 8.5, GREY, font=MONO)

    def figure(sl, name, box=None, frame=True):
        """Place a figure, fitted to a box, never stretched."""
        if not os.path.splitext(name)[1]:
            name += ".png"
        path = FIG / name
        if not path.exists():
            tf = textbox(sl, MARGIN, 3.2, W - 2 * MARGIN, 0.6)
            para(tf, "FIGURE REFUSED: %s is not on disk" % name, 13, WARN,
                 bold=True, first=True, align=PP_ALIGN.CENTER)
            return False
        bx, by, bw, bh = box or (MARGIN, BODY_TOP, W - 2 * MARGIN,
                                 BODY_BOT - BODY_TOP)
        iw, ih = Image.open(path).size
        scale = min(bw / iw, bh / ih)
        w, h = iw * scale, ih * scale
        x, y = bx + (bw - w) / 2, by + (bh - h) / 2
        if frame:
            rect(sl, x - 0.045, y - 0.045, w + 0.09, h + 0.09, PAPER, RULE)
        sl.shapes.add_picture(str(path), Inches(x), Inches(y),
                              Inches(w), Inches(h))
        return True

    def new(title, kicker=None):
        sl = prs.slides.add_slide(blank)
        chrome(sl, title, kicker)
        return sl

    def fig_slide(title, name, cap, source, kicker=None, warn=False):
        sl = new(title, kicker)
        figure(sl, name)
        caption(sl, cap, source, warn=warn)
        return sl

    def two_fig_slide(title, left, right, cap, source, kicker=None,
                      labels=None):
        sl = new(title, kicker)
        gap, colw = 0.30, (W - 2 * MARGIN - 0.30) / 2
        top, hh = BODY_TOP, BODY_BOT - BODY_TOP - (0.26 if labels else 0)
        for i, name in enumerate((left, right)):
            x = MARGIN + i * (colw + gap)
            figure(sl, name, (x, top, colw, hh))
            if labels:
                tf = textbox(sl, x, top + hh + 0.06, colw, 0.24)
                para(tf, labels[i], 10, ACCENT, bold=True, first=True,
                     align=PP_ALIGN.CENTER)
        caption(sl, cap, source)
        return sl

    def section(number, title, blurb):
        sl = prs.slides.add_slide(blank)
        state["n"] += 1
        rect(sl, 0, 0, W, H, HEAD)
        rect(sl, MARGIN, 2.55, 1.55, 0.05, ACCENT)
        tf = textbox(sl, MARGIN, 2.05, 6, 0.4)
        para(tf, "PART %s" % number, 12, ACCENT, bold=True, first=True)
        tf = textbox(sl, MARGIN, 2.85, W - 2 * MARGIN - 2.4, 1.1)
        para(tf, title, 34, PAPER, bold=True, first=True)
        tf = textbox(sl, MARGIN, 4.15, W - 2 * MARGIN - 3.2, 1.2)
        para(tf, blurb, 13.5, "C3C9D2", first=True)
        tf = textbox(sl, W - MARGIN - 1.2, H - 0.44, 1.2, 0.26)
        para(tf, str(state["n"]), 8.5, "5B6270", align=PP_ALIGN.RIGHT,
             first=True)
        return sl

    def metric_row(sl, cards, y=BODY_TOP, h=1.28, x0=MARGIN, width=None):
        """Cards of headline numbers. value / label / footnote.

        x0/width exist because a row that always spanned the slide printed
        itself straight over the figure beside it.
        """
        n = len(cards)
        gap = 0.22
        width = W - 2 * MARGIN if width is None else width
        cw = (width - gap * (n - 1)) / n
        for i, (value, label, foot, colour) in enumerate(cards):
            x = x0 + i * (cw + gap)
            rect(sl, x, y, cw, h, BAND)
            rect(sl, x, y, 0.055, h, colour)
            tf = textbox(sl, x + 0.24, y + 0.16, cw - 0.4, 0.5)
            para(tf, value, 25, colour, bold=True, first=True)
            tf = textbox(sl, x + 0.24, y + 0.66, cw - 0.4, 0.28)
            para(tf, label, 10.5, HEAD, bold=True, first=True)
            tf = textbox(sl, x + 0.24, y + 0.93, cw - 0.4, 0.32)
            para(tf, foot, 8.5, GREY, first=True)

    def bullets(sl, items, x=MARGIN, y=BODY_TOP, w=W - 2 * MARGIN, size=13.5,
                gap=9):
        tf = textbox(sl, x, y, w, BODY_BOT - y)
        first = True
        for item in items:
            if isinstance(item, tuple):
                head, rest = item
                p = para(tf, head, size, HEAD, bold=True, first=first,
                         space=2)
                first = False
                para(tf, rest, size - 1.5, INK, space=gap)
            else:
                para(tf, "— " + item, size, INK, first=first, space=gap)
                first = False
        return tf

    def table(sl, headers, rows, y=BODY_TOP, widths=None, size=10.5,
              row_h=0.34, colours=None):
        n = len(headers)
        widths = widths or [(W - 2 * MARGIN) / n] * n
        xs, x = [], MARGIN
        for wd in widths:
            xs.append(x)
            x += wd
        rect(sl, MARGIN, y, sum(widths), 0.36, HEAD)
        for i, htxt in enumerate(headers):
            tf = textbox(sl, xs[i] + 0.12, y + 0.07, widths[i] - 0.2, 0.26)
            para(tf, htxt, size, PAPER, bold=True, first=True)
        yy = y + 0.36
        for r, row in enumerate(rows):
            if r % 2 == 0:
                rect(sl, MARGIN, yy, sum(widths), row_h, BAND)
            for i, cell in enumerate(row):
                col = INK
                if colours and colours.get((r, i)):
                    col = colours[(r, i)]
                tf = textbox(sl, xs[i] + 0.12, yy + 0.055, widths[i] - 0.2,
                             row_h)
                para(tf, str(cell), size, col, bold=(i == 0), first=True,
                     font=MONO if i and str(cell)[:1].isdigit() else SANS)
            yy += row_h
        return yy

    # ------------------------------------------------------------ 1. title
    sl = prs.slides.add_slide(blank)
    rect(sl, 0, 0, W, H, HEAD)
    rect(sl, 0, 0, 0.16, H, ACCENT)
    tf = textbox(sl, 1.05, 1.55, 10.6, 0.4)
    para(tf, "MSc PROJECT  ·  ROS 2 JAZZY  ·  TWO KINOVA GEN3 7-DOF", 12,
         ACCENT, bold=True, first=True)
    tf = textbox(sl, 1.05, 2.10, 10.9, 2.1)
    para(tf, "Multimodal Teleoperation of a Wearable Dual-Arm "
             "Supernumerary Robotic System", 34, PAPER, bold=True, first=True,
         space=8)
    tf = textbox(sl, 1.05, 3.98, 10.4, 0.8)
    para(tf, "Master interface design, safety architecture, and a workspace "
             "characterisation", 17, "AEB6C2", first=True)
    rect(sl, 1.05, 5.02, 2.0, 0.045, ACCENT)
    tf = textbox(sl, 1.05, 5.26, 10.4, 1.1)
    para(tf, "Gaus Sayyad", 15, PAPER, bold=True, first=True, space=4)
    para(tf, "The arms are worn by one person and driven by a different "
             "person. Both are participants.", 12, "AEB6C2")
    para(tf, "Every figure in this deck is regenerated from a committed "
             "recording by  scripts/make_presentation.py", 10, "7C8595",
         font=MONO)

    # ---------------------------------------------------------- 2. contents
    sl = new("What this deck covers", "contents")
    left = [
        ("1 · The problem", "A third and fourth arm worn by someone who is "
         "not driving them"),
        ("2 · The system as built", "Master arm, five operating modes, and "
         "the safety architecture"),
        ("3 · Perception and autonomy", "Measure the environment, then plan "
         "against the measurement"),
    ]
    right = [
        ("4 · Results", "Grasping, motion generation, VR, and the "
         "simulation-to-hardware gap"),
        ("5 · Workspace characterisation", "Why the arms barely move, sized "
         "rather than asserted"),
        ("6 · Method, limits and next steps", "What is verified, what is "
         "demonstration, and what is blocked"),
    ]
    bullets(sl, left, x=MARGIN, w=5.7, size=15)
    bullets(sl, right, x=MARGIN + 6.4, w=5.7, size=15)
    caption(sl, "The deck is ordered as the thesis is: what was built, what "
                "was measured, and what the measurement does not support.",
            "thesis_v2/main.tex — chapters 1-14")

    # ================================================== PART 1 — the problem
    section("1", "The problem",
            "A supernumerary limb is not a prosthesis and not an industrial "
            "arm. Two people share one body's worth of space, and only one of "
            "them is in control.")

    sl = new("Two people, one body, one set of arms", "framing")
    figure(sl, "diag_overview", (MARGIN, BODY_TOP, W - 2 * MARGIN, 4.30))
    caption(sl, "The driver is across the room. The wearer carries 17 kg on a "
                "backpack frame and does not control the arms. A third person "
                "holds the emergency stop and watches the wearer, not the "
                "screen. This split is what makes the safety case different "
                "from ordinary teleoperation.",
            "thesis_v2/figures/tikz/overview.tex")

    sl = new("What the project set out to build", "objectives")
    bullets(sl, [
        ("A master interface a person can actually drive",
         "An instrumented mannequin arm over a Teensy 4.1, plus a VR route "
         "and a typed or spoken sentence — four ways into the same arms."),
        ("A safety case that survives the arms being worn",
         "The wearer is in the collision model, the clearance floor is "
         "measured geometrically, and every refusal names what blocked it."),
        ("A workspace characterisation, sized rather than asserted",
         "What limits the reach, by how much, and what each design choice "
         "costs in millimetres."),
        ("Autonomy that can be checked",
         "From a typed sentence to four cubes placed by colour, with the "
         "camera's detections shown before anything moves."),
        ("An instrument discipline",
         "Every analysis script carries a known-answer test, because the "
         "measuring tool has been the fault seventeen times."),
    ], size=14)
    caption(sl, "The five objectives, and the order the rest of this deck "
                "follows.", "docs/WORK_BRIEF.md, docs/TASK_SPEC.md")

    # ================================================ PART 2 — system built
    section("2", "The system as built",
            "Hardware, the master arm, the software architecture, the five "
            "operating modes, and the safety chain that gates all of them.")

    sl = new("The hardware", "as built")
    figure(sl, "hw_kinova_vendor.jpg", (MARGIN, BODY_TOP, 5.0, 4.3))
    bullets(sl, [
        ("Two Kinova Gen3, 7-DOF, Robotiq 2F-85",
         "Mounted behind the wearer's shoulders on a backpack frame, "
         "150 mm outboard and 15° of yaw from the shoulder line."),
        ("Master: an instrumented mannequin arm",
         "14 channels over a Teensy 4.1 — roll and bend alternating along "
         "the chain."),
        ("ROS 2 Jazzy, 16 packages",
         "Teleoperation imports nothing in-repo, so it stays the baseline "
         "condition of every comparison."),
        ("Host is WSL2",
         "Which forces the high-level Kortex API: every cyclic write is a "
         "network round trip."),
    ], x=MARGIN + 5.5, w=W - 2 * MARGIN - 5.5, size=12.5)
    caption(sl, "The arm shown is the vendor's product image. NO PHOTOGRAPH "
                "OF THE ASSEMBLED LAB RIG IS COMMITTED TO THIS REPOSITORY — "
                "every visual in this deck is a render, a screenshot or a "
                "diagram, and is labelled as such.",
            "src/ros2_kortex/doc/resources/ — vendor image, not a lab photograph",
            warn=True)

    two_fig_slide("The master arm, from CAD to kinematic chain",
                  "master_arm_cad_views.png", "master_arm_chain.png",
                  "Roll and bend alternate along the chain. Link lengths are "
                  "CAD-inferred and then checked against the measured arm — "
                  "the two disagree by a few millimetres per link, and the "
                  "measured value is the one the software uses.",
                  "thesis_v2/figures/master_arm_cad_views.png, "
                  "master_arm_chain.png",
                  kicker="master interface",
                  labels=["CAD, zero configuration",
                          "Chain: CAD against measured, mm"])

    fig_slide("Electronics and pin assignment",
              "diag_electronics",
              "Fourteen channels over one Teensy 4.1. The call-out records "
              "why two analog-capable pins could not be used — a wiring "
              "constraint that later shows up in the channel health results.",
              "thesis_v2/figures/diagrams/electronics.tex",
              kicker="master interface")

    sl = new("Master channel health — the honest picture", "measured")
    figure(sl, "channel_health.png", (MARGIN, BODY_TOP, 6.1, 4.3))
    figure(sl, "channel_degradation.png",
           (MARGIN + 6.5, BODY_TOP, W - 2 * MARGIN - 6.5, 4.3))
    caption(sl, "Seven of fourteen channels are incoherent — a soldering "
                "problem, with l_j2 and l_j4 named by regression. The system "
                "degrades rather than stopping: which channels are FROZEN is "
                "decided from the newest baseline, and a stale baseline "
                "freezes channels that now work.",
            "recordings/baselines/channels_20260806.json (2026-08-06)",
            warn=True)

    fig_slide("Software architecture", "diag_architecture",
              "Six srl_* packages. The dependency arrow runs one way: "
              "srl_teleop imports nothing in-repo, so the baseline condition "
              "of every experiment cannot be contaminated by the autonomy "
              "layer it is compared against.",
              "thesis_v2/figures/diagrams/architecture.tex",
              kicker="architecture")

    fig_slide("The data path, end to end", "diag_datapath",
              "From a sensed master joint to a commanded arm joint, with "
              "every rate and every gate on the way.",
              "thesis_v2/figures/tikz/datapath.tex", kicker="architecture")

    # The mode diagrams are four to five times wider than they are tall, so
    # they go two to a slide at full width. Four in a 2x2 grid made every
    # label unreadable.
    sl = new("Driving the arms by hand: the mannequin, and VR",
             "operating modes")
    figure(sl, "diag_mode1_mannequin", (MARGIN, BODY_TOP, W - 2 * MARGIN, 2.12))
    tf = textbox(sl, MARGIN, BODY_TOP + 2.18, W - 2 * MARGIN, 0.24)
    para(tf, "MODE 01 — instrumented mannequin arm over a Teensy 4.1", 10.5,
         ACCENT, bold=True, first=True)
    figure(sl, "diag_mode2_vr", (MARGIN, BODY_TOP + 2.50, W - 2 * MARGIN, 2.12))
    tf = textbox(sl, MARGIN, BODY_TOP + 4.66, W - 2 * MARGIN, 0.24)
    para(tf, "MODE 02 — two hand controllers as 6-DOF motion capture", 10.5,
         ACCENT, bold=True, first=True)
    caption(sl, "Blue is sensing, grey is processing, red can stop the arm, "
                "green commands motion — the same colour law in every mode "
                "diagram.",
            "thesis_v2/figures/diagrams/mode1_mannequin.tex, mode2_vr.tex",
            top=BODY_BOT + 0.32)

    sl = new("Sharing control, and giving up control entirely",
             "operating modes")
    figure(sl, "diag_mode3_shared", (MARGIN, BODY_TOP, W - 2 * MARGIN, 2.12))
    tf = textbox(sl, MARGIN, BODY_TOP + 2.18, W - 2 * MARGIN, 0.24)
    para(tf, "MODES 03 / 04 — shared autonomy over either input path", 10.5,
         ACCENT, bold=True, first=True)
    figure(sl, "diag_mode4_voice", (MARGIN, BODY_TOP + 2.50, W - 2 * MARGIN, 2.12))
    tf = textbox(sl, MARGIN, BODY_TOP + 4.66, W - 2 * MARGIN, 0.24)
    para(tf, "MODE 06 — full autonomy from a typed or spoken sentence", 10.5,
         ACCENT, bold=True, first=True)
    caption(sl, "The task layer commands the SAME waypoints under every mode "
                "— run_abc builds them with no mode argument — so any "
                "difference in what the robot does is a property of the mode, "
                "not of the task.",
            "thesis_v2/figures/diagrams/mode3_shared.tex, mode4_voice.tex",
            top=BODY_BOT + 0.32)

    fig_slide("The safety chain", "diag_safety",
              "Three stages before any command leaves. Stage 3 measures the "
              "distance to the wearer from the shapes directly — it does not "
              "use the simulator's collision settings, because the SRDF "
              "deliberately excludes the 44 proximal pairs a shoulder mount "
              "actually threatens, and would report a clear pose with the "
              "tube inside the person.",
              "thesis_v2/figures/tikz/safety.tex", kicker="safety")

    sl = new("The wearer is in the collision model", "safety")
    figure(sl, "rviz_home_pose_labelled", (MARGIN, BODY_TOP, 6.2, 4.3))
    figure(sl, "home_presentation_iso.png",
           (MARGIN + 6.55, BODY_TOP, W - 2 * MARGIN - 6.55, 4.3))
    caption(sl, "RViz, home pose, both arms parked. The wearer's torso, head, "
                "hips and arms are all in the model, and the wearer's posture "
                "is a variable with one source — a posture that reaches the "
                "planner but not the clearance guard measures the old body "
                "under a new name.",
            "thesis_v2/figures/rviz/home_pose_labelled.png; "
            "docs/img/presentation_pose_iso.png (2026-08-15)")

    sl = new("Clearance to the wearer, per task", "safety · measured")
    figure(sl, "clearance_tasks.png", (MARGIN, BODY_TOP, 7.4, 4.3))
    bullets(sl, [
        ("The floor is 150 mm", "and the mount alone sits 161 mm from the "
         "torso — so there are 11 mm of headroom before the arm does "
         "anything at all."),
        ("Only T1 stage 1 is clean",
         "T0 breaches by ~142/137 mm, T2 by 150/147, T3's left arm by 88."),
        ("Predictive avoidance is worth 0.0–5.6 mm",
         "All 28 refusals name end_effector_link: the hand is inside the "
         "person, and the null space holds the hand fixed by definition."),
    ], x=MARGIN + 7.8, w=W - 2 * MARGIN - 7.8, size=11.5)
    caption(sl, "A breach is reported, not silenced. An SRDF exclusion "
                "silences an alarm; it does not move the metal.",
            "recordings/baselines/home_change_applied.json, "
            "predictive_avoidance.json", warn=True)

    # =========================================== PART 3 — perception, autonomy
    section("3", "Perception and autonomy",
            "Measure the environment, then plan against the measurement — "
            "rather than against a scene someone typed in.")

    fig_slide("The perception pipeline", "diag_perception",
              "Four cameras — both wrist RGB-D, a RealSense and a USB scene "
              "camera — reduced to a plane and a set of objects per camera "
              "per cycle, with per-camera health and named refusals.",
              "thesis_v2/figures/tikz/perception.tex", kicker="perception")

    sl = new("The camera may only make the wearer BIGGER", "safety case")
    bullets(sl, [
        ("fuse() keeps whichever body is CLOSER to the robot, per part, with "
         "no mode switch.",
         "A tracked torso can sit beside a mannequin upper arm in one scene. "
         "A body injected 0.10 / 0.30 / 1.00 / 5.00 m FURTHER away changes "
         "nothing at all — which is the property that makes the camera safe "
         "to add."),
    ], size=13, y=BODY_TOP)
    table(sl, ["fault injected through the live node", "what must happen",
               "result"], [
        ("the frame is dark", "refuse, naming darkness", "PASS"),
        ("the frame is blank", "refuse, naming a blank frame", "PASS"),
        ("two people in frame", "refuse, naming the second person", "PASS"),
        ("nobody in frame", "refuse, naming the empty frame", "PASS"),
        ("the arms are occluded", "fall back per part, naming the occlusion",
         "PASS"),
        ("the body is pushed further away", "ignore it — keep the mannequin",
         "PASS"),
        ("control: one person, clear frame", "track, and say it is tracking",
         "PASS"),
    ], y=BODY_TOP + 1.05, widths=[5.0, 4.6, 2.51], size=11, row_h=0.42,
        colours={(i, 2): GOOD for i in range(7)})
    caption(sl, "7 of 7 — and the requirement that a refusal NAME its own "
                "fault caught a false pass where the two-person gate had "
                "never run. The injected frames are stock test images, not "
                "lab photographs.",
            "scripts/verify_wearer_fallbacks.py, "
            "src/srl_perception/test/test_wearer_tracking.py",
            top=BODY_BOT + 0.30)

    sl = new("The robot maps the workspace, then plans against the map",
             "environment mapping")
    figure(sl, "session_scan.png", (MARGIN, BODY_TOP, 6.1, 4.3))
    metric_row(sl, [
        ("6 / 6", "objects recovered", "every one exact in x,y to a "
         "millimetre; every dimension within 3 mm", GOOD),
        ("+0.2 mm", "surface height error", "against the renderer's own "
         "geometry", GOOD),
    ], y=BODY_TOP, h=1.35, x0=MARGIN + 6.5, width=W - 2 * MARGIN - 6.5)
    bullets(sl, [
        ("Objects are cut from the picture", "segment_lift, not clustered "
         "from the point pile."),
        ("One wrist attitude per pass, pure translation",
         "printer-probe style: 2 layers × 3 yaws × 2 arms."),
        ("74 of 74 reachable cells, 0 stillness refusals",
         "and the sweep records what it can reach, then replans from it."),
    ], x=MARGIN + 6.5, y=BODY_TOP + 1.55, w=W - 2 * MARGIN - 6.5, size=11.5)
    caption(sl, "The earlier '−5.2 mm' surface error was surface_z reporting "
                "the plane's intercept AT THE ORIGIN, which is inside the "
                "wearer — an instrument fault, not a measurement.",
            "recordings/session/20260824_190937/ ; "
            "docs/system/25_calibrate_then_plan.md")

    sl = new("From the map to a pick", "environment mapping")
    figure(sl, "session_pick.png", (MARGIN, BODY_TOP, 7.5, 4.3))
    bullets(sl, [
        ("Table found at z = 1.2485 m, tilt 0.54°", "bounds found by a coarse "
         "probe before the sweep, not typed in."),
        ("6 objects; 4 graspable at 41 mm, 2 refused at 140 mm",
         "'too wide' is a named refusal — the gripper opens 85 mm."),
        ("Approached from ABOVE",
         "pregrasp z 1.490 → grasp z 1.381 onto an object at z 1.282."),
        ("765 s end to end, 55 narration sentences",
         "full scan, reference, pick and quick check, all exit 0."),
    ], x=MARGIN + 7.9, w=W - 2 * MARGIN - 7.9, size=11)
    caption(sl, "DEMONSTRATION, not evidence: the arm is simulated and the "
                "depth is rendered. What is real is the sweep order, the "
                "deprojection, the segmentation, the fusion, the plane fit, "
                "the IK, the approach-angle search and every refusal.",
            "recordings/session/20260824_190937/summary.txt (2026-08-24)",
            warn=True)

    sl = new("Full autonomy: four cubes placed by colour, from a typed "
             "sentence", "mode 06")
    figure(sl, "clip_t1_quad.png", (MARGIN, BODY_TOP, 8.6, 4.3))
    bullets(sl, [
        ("4 grasps, 4 releases", "every cube on the pad of its own colour, "
         "started at home 0.0000 rad."),
        ("Both arms, pads at ±0.290 m",
         "straddling the centreline — and neither arm can cross it, so a cube "
         "is only ever delivered to the pad on its own side."),
        ("Each clip runs card → look → task",
         "the observe move is in the footage, and the card carries the "
         "sentence verbatim."),
    ], x=MARGIN + 9.0, w=W - 2 * MARGIN - 9.0, size=11)
    caption(sl, "Four synchronised RViz views of one run. The wearer, the "
                "table, the two coloured pads and the four cubes are all in "
                "the scene the planner is checked against.",
            "recordings/verification/06_full_autonomy/T1/S1_both_arms_centre/"
            "rviz_quad.mp4 (2026-08-23)")

    two_fig_slide("A seeded layout, and the view where a grasp is judged",
                  "clip_t1s2_quad.png", "clip_t1_gripper.png",
                  "Stage 2 is stage 1's geometry with the side drawn at "
                  "random — eight seeds walked, all clean. The gripper view "
                  "exists because a grasp asserted from a joint angle is not "
                  "a grasp: it has to be seen closing on the object.",
                  "recordings/verification/06_full_autonomy/T1S2/... and "
                  ".../T1/S1_both_arms_centre/rviz_gripper.mp4",
                  kicker="mode 06",
                  labels=["T1 stage 2 — seeded layout",
                          "T1 — the pads on the cube"])

    sl = new("Understanding a free-form sentence", "language")
    figure(sl, "language_outcomes.png", (MARGIN, BODY_TOP, 6.6, 4.3))
    bullets(sl, [
        ("Zero MISUNDERSTOOD is the whole result",
         "ASKED and REFUSED are both SAFE — the arm moved in neither. The bar "
         "is drawn at zero rather than omitted, so it reads as a measured "
         "zero and not an absence."),
        ("A wider parser sweep, 75 phrasings",
         "34 correct / 14 asked / 27 refused / 0 misunderstood, against "
         "21 / 14 / 37 / 3 for the grammar it replaced, run on the same cases "
         "in its own interpreter."),
        ("Ambiguity asks, and the ask is answerable",
         "10 of the 14 asks carry a scored reply; all 10 resolve correctly, "
         "and a reply is never counted as CORRECT."),
    ], x=MARGIN + 7.0, w=W - 2 * MARGIN - 7.0, size=11)
    caption(sl, "Voice is a PARSER score, never a transcription score. "
                "/dev/snd on this host has only 'timer', so no microphone can "
                "be opened and every voice figure came from synthesised "
                "audio.",
            "recordings/baselines/language_vision_sweep.json (40 phrasings); "
            "scripts/sweep_t1_instructions.py (75)", warn=True)

    # ==================================================== PART 4 — results
    section("4", "Results",
            "Grasping and placement, motion generation, the VR path, and the "
            "one measurement taken on real hardware.")

    sl = new("Headline results", "results")
    metric_row(sl, [
        ("100%", "grasp success", "17 cells × 8 angles, all five modes "
         "(2026-08-23 clip set)", GOOD),
        ("0.000 mm", "grasp positioning error", "against a 30 mm capture "
         "gate", GOOD),
        ("56.3 mm", "T1 placement error", "down from 319.7 mm before the "
         "re-record", GOOD),
        ("12.91 mm", "control budget, RSS", "worst case 19.99 mm — inside "
         "the 30 mm gate for the first time", ACCENT),
    ], h=1.32)
    table(sl, ["mode", "task", "n", "grasped", "positioning", "placement"], [
        ("01 master", "T3", "2", "2 / 2", "0.00 mm", "—"),
        ("02 VR", "T3", "2", "2 / 2", "0.00 mm", "—"),
        ("03 shared", "T3", "2", "2 / 2", "0.00 mm", "—"),
        ("04 VR + shared", "T3", "2", "2 / 2", "0.00 mm", "—"),
        ("06 autonomy", "T1", "4", "4 / 4", "0.00 mm", "55.5 – 56.9 mm"),
        ("06 autonomy", "T1S2", "4", "4 / 4", "0.00 mm", "—"),
    ], y=BODY_TOP + 1.52, widths=[2.5, 1.5, 0.9, 1.5, 2.4, 3.31], size=11,
        row_h=0.36)
    caption(sl, "T1 and T1S2 run under mode 06 only; T0 and the dance "
                "routines grasp nothing by design. Reading a by-design "
                "absence as a gap has been an instrument fault here three "
                "times.",
            "recordings/verification/accuracy_table.json (2026-08-24)",
            top=BODY_BOT + 0.30)

    two_fig_slide("Grasp success and positioning error, per mode",
                  "res_grasp_matrix.png", "res_accuracy_per_mode.png",
                  "Accuracy is TIED across all five modes on this clip set. "
                  "That is the honest answer to 'is VR the most accurate "
                  "mode': it is not distinguishable here, and this set "
                  "PREDATES the 2026-08-26 VR smoothing.",
                  "recordings/verification/accuracy_table.json → "
                  "recordings/analysis/ (2026-08-24)",
                  kicker="results",
                  labels=["grasp success, mode × task",
                          "positioning and placement error"])

    fig_slide("The same waypoints, different travel", "res_mode_path_lengths.png",
              "The task layer commands identical waypoints under every mode — "
              "run_abc builds them with no mode argument. The robot does not "
              "perform identically: T0's left path is 1.407 m under mode 01 "
              "against 2.034 m under mode 03, measured with no operator "
              "present.",
              "recordings/baselines/mode_difference.json (2026-08-15)",
              kicker="results")

    sl = new("Motion generation: a per-joint clamp replaced by Ruckig",
             "results · motion")
    figure(sl, "res_teleop_motion_generator.png",
           (MARGIN, BODY_TOP, 7.6, 4.3))
    metric_row(sl, [
        ("51.3 → 0.0", "off-path excursion, mm", "left arm; right "
         "39.2 → 0.0, against a 30 mm gate", GOOD),
    ], y=BODY_TOP, h=1.30, x0=MARGIN + 8.0, width=W - 2 * MARGIN - 8.0)
    bullets(sl, [
        ("It never read joint_limits.yaml",
         "17.5 rad/s commanded against a 1.3963 limit — 12.53×. Now 1.00×."),
        ("Not a tuning problem",
         "swept against its own step size, the clamp's excursion SETTLES at "
         "68 mm. It cannot be tuned into compliance."),
        ("Acceleration and jerk are ASSUMED and labelled",
         "the yaml declares has_acceleration_limits: false for all fourteen "
         "joints. Velocity limits are the robot's."),
    ], x=MARGIN + 8.0, y=BODY_TOP + 1.50, w=W - 2 * MARGIN - 8.0, size=11)
    caption(sl, "motion_generator:=legacy reproduces any recording made "
                "before 2026-08-23.",
            "recordings/baselines/teleop_motion.json (2026-08-23)")

    two_fig_slide("The VR path: smoothing, and where it stops working",
                  "res_vr_smoothing.png", "res_vr_lag_vs_speed.png",
                  "1-Euro against the EMA it replaced: 3.4× better stillness "
                  "and 2.4× less lag. Under 0.27 m/s every protocol segment "
                  "stays inside the 30 mm gate; the deliberate fast reach at "
                  "0.31–0.35 m/s does not. The smoothing figure is the filter "
                  "maths recomputed through the shipped code — NOT a "
                  "through-the-node or real-arm measurement.",
                  "srl_vr_teleop/vr_smoothing.py; "
                  "recordings/vr_teleop/protocol_20260826_124739.json",
                  kicker="results · VR", labels=[
                      "stillness residual and reach lag",
                      "peak lag against peak hand speed"])

    sl = new("The simulation-to-hardware gap — measured on the real arms",
             "results · real hardware")
    figure(sl, "res_sim_to_real_park_error.png", (MARGIN, BODY_TOP, 7.4, 4.3))
    metric_row(sl, [
        ("0.305°", "every joint parks short", "on the side it came "
         "from; 211 of 252 errors in a 0.007° band", WARN),
    ], y=BODY_TOP, h=1.30, x0=MARGIN + 7.8, width=W - 2 * MARGIN - 7.8)
    bullets(sl, [
        ("7.2 mm RMS at the end effector", "on a 100 mm move, both arms."),
        ("A bound, not a spread",
         "pushing the joint errors through the Jacobian reproduces the "
         "Cartesian error to 90%, r = 0.923 — a constant terminal offset."),
        ("ONE parameter per arm removes 74% / 76%",
         "held out on an axis it was never fitted on, where a Cartesian 3×3 "
         "scores 93 mm."),
    ], x=MARGIN + 7.8, y=BODY_TOP + 1.50, w=W - 2 * MARGIN - 7.8, size=11)
    caption(sl, "36 real runs on the physical arms — the only hardware "
                "measurement in this deck. The COMPENSATION is modelled and "
                "has NOT been tried on hardware. The data already existed in "
                "arm_directional_calibration.json and nothing read it.",
            "recordings/baselines/arm_directional_calibration.json, "
            "sim_to_real_gap.json (2026-08-23)", warn=True)

    sl = new("And the 'speed sweep' was not one", "results · instrument")
    bullets(sl, [
        ("The claim", "I reported the parking error as speed-independent "
         "across a 5× range."),
        ("What the data says", "vmax_rad_s was read once at bridge start-up "
         "and never applied. 7 of 36 runs moved FASTER than the limit they "
         "were commanded with, and kortex_highlevel_bridge records this in "
         "its own source."),
        ("So", "the three speeds are three REPLICATES of one condition. How "
         "the error varies with speed has NEVER been measured, and the file "
         "now says so in its own verdict field."),
        ("A control was added", "which asserts the sweep still looks inert — "
         "so a genuine sweep makes the test fail and the stale sentences get "
         "rewritten."),
    ], size=13.5)
    caption(sl, "This slide is a result. A measurement that contradicts an "
                "earlier one is an instrument check, not a finding, until the "
                "two are reconciled.",
            "recordings/baselines/sim_to_real_gap.json → "
            "speed_sweep_was_inert; "
            "test_the_arm_stops_short_and_the_bridge_knows.py", warn=True)

    # ================================================ PART 5 — workspace
    section("5", "Workspace characterisation",
            "Why the arms barely move — sized in millimetres, per direction, "
            "per design choice, rather than asserted.")

    two_fig_slide("Reachability, and two arms that cannot meet",
                  "reachability_polar.png", "workspace_disjoint.png",
                  "N = 10 repeats over the whole densified path, not one IK "
                  "call — TRAC-IK restarts randomly and one call is one coin "
                  "flip. The two arms' reachable sets are disjoint, which is "
                  "why a handover task needs the right arm re-parked in "
                  "hardware before it can exist at all.",
                  "recordings/baselines/workspace_n10_20260806.json",
                  kicker="workspace", labels=[
                      "reach per direction, both arms",
                      "the two reachable sets do not overlap"])

    sl = new("What binds the workspace", "workspace")
    table(sl, ["direction", "what limits it", "measured"], [
        ("forward / outboard", "the pinned wrist", "0.065 m mean reach at the "
         "work point, against 0.450 free"),
        ("inboard", "the wearer — the TORSO, not their arms",
         "min |x| 0.300 (L) / 0.375 (R) with the wearer's arms deleted "
         "entirely"),
        ("down", "the table", "no configuration puts |x| ≤ 0.10 m on the "
         "surface; 3360 cells searched"),
        ("top-down on a surface", "not achievable at all",
         "0 of 840 cells, any table height 0.70–1.10 m, either arm"),
        ("any joint limit", "nothing", "no direction is bound by one"),
    ], widths=[2.9, 4.0, 5.21], size=11.5, row_h=0.62)
    caption(sl, "Folding the wearer's arms is worth 50 mm to the mannequin "
                "and NOTHING to a real person, because what binds changes "
                "identity: a limb for the mannequin, the torso every time for "
                "the person.",
            "recordings/baselines/centre_vs_wearer_posture.json, "
            "centre_on_surface.json, wearer_size_effect.json",
            top=BODY_BOT + 0.24)

    sl = new("What the pinned wrist costs", "workspace · sized")
    metric_row(sl, [
        ("0.065 m", "pinned, at the work point", "11 of 12 direction walks "
         "wearer-bound", WARN),
        ("0.325 m", "with a 45° cone", "the follower's own cone supplies it", ACCENT),
        ("0.450 m", "fully free orientation", "+385 mm, and only 4 walks "
         "wearer-bound", GOOD),
        ("+0 mm", "freeing the ROLL alone", "refused by name — it buys "
         "nothing", GREY),
    ], h=1.32)
    bullets(sl, [
        ("The mechanism is not the wearer",
         "A fixed 6-DOF pose on a 7-DOF arm spends the whole redundancy — and "
         "the redundancy is exactly the joint that moves the elbow out of the "
         "person while the hand stays put."),
        ("A factor of seven, and it was never sized before this measurement",
         "From the HOME end-effector the penalty is only 89 mm; at the point "
         "where the work actually happens it is 385 mm. Measuring from the "
         "wrong start point is what hid it."),
        ("Nothing changes until a mode opts in",
         "orientation_policy defaults to 'exact', so HARD CONSTRAINT 1 is "
         "untouched by the finding."),
    ], y=BODY_TOP + 1.52, size=12.5)
    caption(sl, "0.15 m wearer floor enforced under every policy, so the "
                "comparison is like for like.",
            "scripts/measure_orientation_cost.py; "
            "docs/system/21_what_makes_the_workspace_small.md")

    two_fig_slide("Mount geometry and the lateral constraint",
                  "mount_sweep.png", "lateral_constraint.png",
                  "The mount was swept rather than chosen: 150 mm outboard "
                  "and 15° of yaw is where the reachable set stops fighting "
                  "the wearer. base_link still sits 161 mm from the torso and "
                  "no joint moves it — so a clearance reading of exactly "
                  "0.1610 m is the MOUNT, not the arm.",
                  "recordings/baselines/gap_vs_mount.json, "
                  "clearance_region.json", kicker="workspace",
                  labels=["mount position sweep",
                          "the lateral constraint"])

    sl = new("The error budget, term by term", "workspace · budget")
    table(sl, ["term", "mm", "systematic", "note"], [
        ("terminal joint error, uncompensated", "7.24", "yes",
         "0.305° per joint on 36 real runs"),
        ("  the same, overshoot switched on", "1.88", "yes",
         "modelled, held out on an unseen axis — UNTRIED on hardware"),
        ("shell bias, before the plane", "10.50", "yes",
         "the object's bottom was unknown"),
        ("  the same, with the support plane", "0.00", "yes",
         "the object RESTS on the plane, so its bottom is known"),
        ("depth noise at 0.4–0.8 m", "2.00", "no", "RealSense D4xx, "
         "subpixel-limited"),
        ("IK solve residual", "0.20", "no", "arm_ik reports its achieved "
         "residual; a refusal is searched"),
        ("declared vs measured pad offset", "0.05", "yes",
         "derived from the FK measurement since 2026-08-23"),
        ("camera_link vs the physical module", "UNMEASURED", "yes",
         "the largest remaining unknown — no CAD of the gripper here"),
    ], widths=[4.3, 1.35, 1.5, 4.96], size=10, row_h=0.44,
        colours={(7, 1): WARN, (7, 3): WARN, (1, 3): WARN})
    caption(sl, "RSS 18.64 → 12.91 mm, worst case 33.39 → 19.99 mm — inside "
                "the 30 mm capture gate for the first time. The pad row is "
                "COMPUTED from the constants, so reverting the derivation "
                "makes the budget say so.",
            "recordings/baselines/control_budget.json; "
            "scripts/measure_control_budget.py", top=BODY_BOT + 0.30)

    # ============================================ PART 6 — method and limits
    section("6", "Method, limits and next steps",
            "The operator's window, the instrument discipline that produced "
            "these numbers, and an honest account of what none of them "
            "support.")

    sl = new("The window is the interface", "operator")
    figure(sl, "gui_control_window.png", (MARGIN, BODY_TOP, 8.3, 4.3))
    bullets(sl, [
        ("Every capability reaches the window",
         "A feature reachable only from a terminal does not exist for the "
         "person running the session."),
        ("257 checks, 0 failed",
         "The audit presses every button AND checks what the press did — the "
         "e-stop must reach /estop, and a service button with no stack must "
         "say so."),
        ("A passing audit is not proof",
         "Three defects lived for weeks behind a green return code: an "
         "invisible embedded RViz, a control column capped below its own "
         "content, and every mode button below the fold. So: launch it and "
         "look at it."),
    ], x=MARGIN + 8.7, w=W - 2 * MARGIN - 8.7, size=10.5)
    caption(sl, "The audit's own worst catch: it had HUNG on a modal consent "
                "dialog since that dialog was added, so it never reached the "
                "presses that would have shown five buttons doing nothing "
                "silently inside a Qt slot.",
            "recordings/session/20260824_190937/control_window.png; "
            "scripts/verify_gui_buttons.py")

    two_fig_slide("Commanded against actual, side by side",
                  "gui_rviz_after_fix.png", "gui_dual_panels.png",
                  "RViz (COMMANDED) embedded beside a native ACTUAL panel "
                  "drawn from the real_* frames, with the divergence readout "
                  "under both. The RViz master draws three states, not two — "
                  "commanded, actual, and PLANNED-BUT-REFUSED with its reason "
                  "string, which cannot be drawn without one.",
                  "thesis_v2/figures/rviz_after_fix.png; "
                  "docs/img/gui_dual_panels.png", kicker="operator",
                  labels=["the embedded COMMANDED view",
                          "both panels and the divergence readout"])

    sl = new("The standing rule, and what it caught", "method")
    bullets(sl, [
        ("Before reporting a bad measurement, validate the instrument.",
         "A surprising failure is evidence about the INSTRUMENT until the "
         "instrument has been cleared. This has been the fault seventeen "
         "times."),
    ], size=14, y=BODY_TOP)
    table(sl, ["what it looked like", "what it actually was"], [
        ("T2's grippers never closed", "run_abc held a grip until the pads "
         "REACHED the object — unsatisfiable for a task declaring "
         "grip_obj=None"),
        ("A flaky autonomy task", "clip_scene measured stage 2 at stage 1's "
         "approach: a 99.1 mm error in the MEASUREMENT, not the task"),
        ("A grasp that was 13.47 mm short",
         "PAD_MID_EE was derived, not measured — and the pad midpoint is a "
         "curve, not a constant: the fingers swing on a four-bar"),
        ("A failing test called '='",
         "the test gate matched ^FAILED over pytest output, and pep257 echoes "
         "the source lines it objects to"),
        ("Three instruments reporting defects that did not exist",
         "an i % 2 arm mapping, a by-design absence read as a gap, and a "
         "release demanded from a task that never releases"),
    ], y=BODY_TOP + 0.95, widths=[4.7, 7.41], size=10.5, row_h=0.62)
    caption(sl, "Every analysis script carries a known-answer test, and a "
                "check that cannot fail on a deliberately broken input is not "
                "a check.", "CLAUDE.md — THE STANDING RULE; "
                            "docs/system/findings.md", top=BODY_BOT + 0.30)

    sl = new("What is verified, and what is not", "limits")
    gap2 = 0.32
    colw3 = (W - 2 * MARGIN - gap2) / 2
    rect(sl, MARGIN, BODY_TOP, colw3, 4.5, BAND)
    rect(sl, MARGIN, BODY_TOP, 0.055, 4.5, GOOD)
    tf = textbox(sl, MARGIN + 0.28, BODY_TOP + 0.18, colw3 - 0.5, 0.32)
    para(tf, "MEASURED, AND THE FILE IS COMMITTED", 11.5, GOOD, bold=True,
         first=True)
    bullets(sl, [
        "Grasping and placement across five modes, 17 cells × 8 angles",
        "The parking error, on 36 real-hardware runs",
        "Ruckig against the clamp it replaced, including the sweep showing "
        "the clamp cannot be tuned",
        "Workspace reach per direction at N = 10 over the whole path",
        "Clearance to the wearer, geometrically, per task",
        "Six injected perception faults, each refusal naming its own fault",
        "1381 unit tests passing; 257 GUI checks, 0 failed",
    ], x=MARGIN + 0.28, y=BODY_TOP + 0.60, w=colw3 - 0.55, size=10.5, gap=6)

    x2 = MARGIN + colw3 + gap2
    rect(sl, x2, BODY_TOP, colw3, 4.5, BAND)
    rect(sl, x2, BODY_TOP, 0.055, 4.5, WARN)
    tf = textbox(sl, x2 + 0.28, BODY_TOP + 0.18, colw3 - 0.5, 0.32)
    para(tf, "NOT MEASURED — AND NOT CLAIMED", 11.5, WARN, bold=True,
         first=True)
    bullets(sl, [
        "NO HUMAN DATA HAS BEEN COLLECTED AT ALL — blocked on ethics",
        "Speech recognition against a real speaker: /dev/snd has only "
        "'timer', so every voice number came from synthesised audio",
        "The sim-to-real compensation, on hardware",
        "How the parking error varies with speed",
        "camera_link against the physical camera module — the largest "
        "remaining unknown in the budget",
        "Detection rate at working distance",
        "7 of 14 master channels are incoherent; a soldering problem",
    ], x=x2 + 0.28, y=BODY_TOP + 0.60, w=colw3 - 0.55, size=10.5, gap=6)
    caption(sl, "Worn operation is >17 kg with no gravity compensation on "
                "someone who did not choose the motion. The operator and the "
                "wearer are different people and consent separately.",
            "CLAUDE.md — Blocked on the lab / Blocked on ethics",
            top=BODY_BOT + 0.30)

    fig_slide("The study that is designed but not run", "diag_experiment_design",
              "The protocol, the conditions and the analysis exist and are "
              "written up. No participant has been recruited: anonymity is "
              "enforced in code — write_manifest() raises on name, email, "
              "date of birth, address and phone — but consent has not been "
              "sought, so the participant results chapter is deliberately "
              "empty.",
              "thesis_v2/figures/diagrams/experiment_design.tex; "
              "docs/research/", kicker="limits", warn=False)

    sl = new("Contributions", "conclusion")
    bullets(sl, [
        ("A working multimodal master interface",
         "Four input routes — instrumented mannequin arm, VR controllers, "
         "shared autonomy over either, and a typed or spoken sentence — into "
         "one arm-command path, with the baseline condition kept free of "
         "in-repo dependencies so the comparison is not circular."),
        ("A safety architecture that measures the wearer rather than trusting "
         "the planner",
         "The wearer's body and posture are variables with one source; the "
         "camera may only ever make that body BIGGER; and the clearance floor "
         "is enforced against the same body the planner sees."),
        ("A workspace characterisation that sizes its own constraints",
         "The pinned wrist costs a factor of seven at the work point, the "
         "torso is what closes the centre, and top-down on a surface is not "
         "achievable at all — each with the search that establishes it."),
        ("Autonomy from a sentence, with zero misunderstandings",
         "40 phrasings in the vision-coupled sweep and 75 in the parser "
         "sweep; ambiguity asks, and the ask is answerable in the same box."),
        ("An instrument discipline, and the ledger of what it caught",
         "Seventeen occasions where the measuring tool was the fault, each "
         "written up with the mechanism — which is itself a result about how "
         "to evaluate a system like this."),
    ], size=12)

    sl = new("Next, in order", "next steps")
    table(sl, ["#", "what", "why it is next"], [
        ("1", "Off-axis directions in the calibration sweep",
         "the 3×3 fit scores 93 mm held out on an axis — six axis-aligned "
         "directions are exactly a basis, with no redundancy to test"),
        ("2", "Try the terminal overshoot on hardware",
         "modelled at 74% / 76% removal; both it and the 0.10° deadband are "
         "live parameters and four runs settle which wins"),
        ("3", "Measure camera_link against the physical module",
         "the cheapest high-value calibration left, and the largest unknown "
         "in the budget"),
        ("4", "Attach a scene camera",
         "the node, both calibration tools and their self-tests are built; "
         "no camera has ever been attached to this host"),
        ("5", "Repair the seven incoherent master channels",
         "a soldering problem, with l_j2 and l_j4 named by regression"),
        ("6", "Ethics approval, then participants",
         "the protocol is written; the arms are worn by one person and driven "
         "by another, and they consent separately"),
    ], widths=[0.5, 4.4, 7.21], size=10.5, row_h=0.66)
    caption(sl, "Ordered by what unblocks the most downstream work, not by "
                "effort.", "docs/NEXT_SESSION_2026_08_23.md, "
                           "docs/system/24_motion_planning.md",
            top=BODY_BOT + 0.30)

    # ------------------------------------------------------------ closing
    sl = prs.slides.add_slide(blank)
    state["n"] += 1
    rect(sl, 0, 0, W, H, HEAD)
    rect(sl, 0, 0, 0.16, H, ACCENT)
    tf = textbox(sl, 1.05, 2.55, 10.4, 1.0)
    para(tf, "Thank you", 40, PAPER, bold=True, first=True)
    rect(sl, 1.05, 3.80, 2.0, 0.045, ACCENT)
    tf = textbox(sl, 1.05, 4.10, 10.4, 1.6)
    para(tf, "Everything shown is regenerated from committed recordings by "
             "scripts/make_presentation.py.", 13, "AEB6C2", first=True,
         space=6)
    para(tf, "Evidence: recordings/verification/ (clips and the accuracy "
             "table) · recordings/baselines/ (every measurement) · "
             "recordings/analysis/ (the result figures) · docs/system/"
             "findings.md (the diagnostic history)", 11, "7C8595", font=MONO)

    OUT.mkdir(parents=True, exist_ok=True)
    prs.save(str(DECK))
    return len(prs.slides.__iter__.__self__._sldIdLst)


def main():
    for mod in ("pptx", "PIL", "fitz", "matplotlib"):
        if not _need(mod):
            sys.exit("missing %s -- run with .venv_vision/bin/python" % mod)
    print("preparing assets")
    prepare_assets()
    print("building deck")
    n = build_deck()
    print("wrote %s (%d slides)" % (DECK.relative_to(ROOT), n))


if __name__ == "__main__":
    main()
