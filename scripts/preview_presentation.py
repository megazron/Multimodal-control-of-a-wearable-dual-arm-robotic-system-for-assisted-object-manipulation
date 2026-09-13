#!/usr/bin/env python3
"""Render the built deck to PNGs so it can be LOOKED AT, not just returned.

    .venv_vision/bin/python scripts/preview_presentation.py

Writes one PNG per slide into ``extras/presentation/preview/`` and a contact
sheet beside them. There is no LibreOffice on this host, so the pages are
drawn from the SAVED .pptx -- every rectangle, picture and text run is read
back out of the file that will actually be opened, rather than from the
script that wrote it. That is the point: a renderer fed by the generator's
own variables could not catch a shape placed off the slide.

Two deliberate inaccuracies, both conservative:

* The deck asks for Calibri and this host has no Calibri. Liberation Sans is
  substituted, and it is WIDER than Calibri at the same point size -- so text
  that fits in this preview fits in PowerPoint, never the other way round.
* Line breaking here is a greedy wrap. PowerPoint's is too, but its kerning
  differs slightly, so a line ending within a word-width of the box edge may
  break one word earlier or later.

Overflow is not silently drawn: any text frame whose wrapped height exceeds
its box, and any shape extending past the slide, is reported on stdout and
outlined in red on the page.
"""
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont
from pptx import Presentation
from pptx.util import Emu

ROOT = Path(__file__).resolve().parent.parent
DECK = ROOT / "docs" / "presentation" / "SRL_project_presentation.pptx"
OUT = ROOT / "docs" / "presentation" / "preview"

DPI = 110                                    # 1466 x 825 per slide
FONTS = {
    (False, False): "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
    (True, False): "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
    (False, True): "/usr/share/fonts/truetype/liberation/LiberationSans-Italic.ttf",
    (True, True): "/usr/share/fonts/truetype/liberation/LiberationSans-BoldItalic.ttf",
}
MONO = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"

# RGBColor subclasses tuple, so "#%s" % rgb raises TypeError rather than
# formatting -- caught by a bare except, that silently painted every solid
# fill white and every coloured run black. Always str() it first.
_cache = {}


def font(size_pt, bold=False, italic=False, mono=False):
    px = max(6, int(round(size_pt * DPI / 72.0)))
    key = (px, bold, italic, mono)
    if key not in _cache:
        path = MONO if mono else FONTS[(bold, italic)]
        _cache[key] = ImageFont.truetype(path, px)
    return _cache[key]


def px(emu):
    return int(round(Emu(emu).inches * DPI))


def wrap(draw, text, fnt, width):
    """Greedy wrap, matching how a text frame with word_wrap on behaves."""
    if not text:
        return [""]
    lines, line = [], ""
    for word in text.split():
        trial = (line + " " + word).strip()
        if draw.textlength(trial, font=fnt) <= width or not line:
            line = trial
        else:
            lines.append(line)
            line = word
    lines.append(line)
    return lines


def render(slide, index, problems):
    W = px(PRS.slide_width)
    H = px(PRS.slide_height)
    img = Image.new("RGB", (W, H), "white")
    d = ImageDraw.Draw(img)
    overflow_boxes = []

    for shape in slide.shapes:
        try:
            x, y = px(shape.left), px(shape.top)
            w, h = px(shape.width), px(shape.height)
        except TypeError:
            continue

        if x < -2 or y < -2 or x + w > W + 2 or y + h > H + 2:
            problems.append("slide %2d: shape past the slide edge at "
                            "(%d,%d) %dx%d" % (index, x, y, w, h))
            overflow_boxes.append((x, y, w, h))

        if shape.shape_type == 13 or shape.__class__.__name__ == "Picture":
            try:
                pic = Image.open(shape.image.blob and
                                 __import__("io").BytesIO(shape.image.blob))
                pic = pic.convert("RGB").resize((max(1, w), max(1, h)),
                                                Image.LANCZOS)
                img.paste(pic, (x, y))
            except Exception:                                  # noqa: BLE001
                d.rectangle([x, y, x + w, y + h], outline="magenta", width=2)
            continue

        if shape.has_text_frame and not shape.text_frame.text.strip() \
                and shape.fill.type is not None:
            pass

        # solid fill
        try:
            if shape.fill.type is not None and shape.fill.type == 1:
                d.rectangle([x, y, x + w, y + h],
                            fill="#" + str(shape.fill.fore_color.rgb))
        except Exception:                                      # noqa: BLE001
            pass
        try:
            if shape.line.color and shape.line.color.type is not None:
                d.rectangle([x, y, x + w, y + h],
                            outline="#" + str(shape.line.color.rgb), width=1)
        except Exception:                                      # noqa: BLE001
            pass

        if not shape.has_text_frame:
            continue

        tf = shape.text_frame
        cy = y
        for para in tf.paragraphs:
            runs = [r for r in para.runs if r.text]
            if not runs:
                cy += 4
                continue
            r0 = runs[0]
            size = r0.font.size.pt if r0.font.size else 12
            mono = (r0.font.name or "").lower().startswith("consolas")
            fnt = font(size, bool(r0.font.bold), bool(r0.font.italic), mono)
            colour = "#000000"
            try:
                if r0.font.color is not None and r0.font.color.type is not None:
                    colour = "#" + str(r0.font.color.rgb)
            except Exception:                                  # noqa: BLE001
                pass
            text = "".join(r.text for r in runs)
            lh = int(size * DPI / 72.0 * 1.22)
            for line in wrap(d, text, fnt, max(10, w)):
                tx = x
                if para.alignment is not None and "CENTER" in str(para.alignment):
                    tx = x + (w - d.textlength(line, font=fnt)) / 2
                elif para.alignment is not None and "RIGHT" in str(para.alignment):
                    tx = x + w - d.textlength(line, font=fnt)
                d.text((tx, cy), line, font=fnt, fill=colour)
                cy += lh
            cy += int((para.space_after.pt if para.space_after else 0)
                      * DPI / 72.0)

        if cy > y + h + 4:
            problems.append("slide %2d: text overflows its box by %d px "
                            "-- \"%s\"" % (index, cy - (y + h),
                                           tf.text[:52].replace("\n", " ")))
            overflow_boxes.append((x, y, w, h))

    for (x, y, w, h) in overflow_boxes:
        d.rectangle([x, y, x + w, y + h], outline="red", width=3)
    return img


def main():
    global PRS
    if not DECK.exists():
        sys.exit("no deck at %s -- run scripts/make_presentation.py first"
                 % DECK)
    PRS = Presentation(str(DECK))
    OUT.mkdir(parents=True, exist_ok=True)
    for p in OUT.glob("slide_*.png"):
        p.unlink()

    problems, pages = [], []
    for i, slide in enumerate(PRS.slides, 1):
        img = render(slide, i, problems)
        img.save(OUT / ("slide_%02d.png" % i))
        pages.append(img)

    cols = 4
    tw, th = 460, 270
    rows = (len(pages) + cols - 1) // cols
    sheet = Image.new("RGB", (cols * tw, rows * (th + 18)), "#EEEFF2")
    d = ImageDraw.Draw(sheet)
    for i, page in enumerate(pages):
        t = page.copy()
        t.thumbnail((tw - 10, th - 10))
        x, y = (i % cols) * tw, (i // cols) * (th + 18)
        sheet.paste(t, (x + 5, y + 16))
        d.text((x + 6, y + 3), "%d" % (i + 1), font=font(9, bold=True),
               fill="#333333")
    sheet.save(OUT / "contact_sheet.png")

    print("%d slide(s) -> %s" % (len(pages), OUT))
    if problems:
        print("\n%d LAYOUT PROBLEM(S):" % len(problems))
        for p in problems:
            print("  " + p)
    else:
        print("no shape off the slide, no text frame overflowing its box")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
