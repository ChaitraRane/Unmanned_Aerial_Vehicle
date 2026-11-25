"""Convert PRACTICAL_1_REPORT.md to PRACTICAL_1_REPORT.docx (headings, paragraphs, lists, tables, images)."""
import os
import re
import sys

from docx import Document
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt, RGBColor

HERE = os.path.dirname(os.path.abspath(__file__))
TOKEN = re.compile(r"(\*\*[^*]+\*\*|\*[^*]+\*|`[^`]+`)")


def runs(par, text, size=None):
    text = text.replace("\\*", "*")
    for part in TOKEN.split(text):
        if not part:
            continue
        if part.startswith("**"):
            r = par.add_run(part[2:-2]); r.bold = True
        elif part.startswith("`"):
            r = par.add_run(part[1:-1]); r.font.name = "Consolas"; r.font.color.rgb = RGBColor(0x60, 0x20, 0x20)
        elif part.startswith("*") and len(part) > 2:
            r = par.add_run(part[1:-1]); r.italic = True
        else:
            r = par.add_run(part)
        if size:
            r.font.size = Pt(size)


def shade(cell, hexcol):
    tc = cell._tc.get_or_add_tcPr()
    sh = OxmlElement("w:shd")
    sh.set(qn("w:val"), "clear"); sh.set(qn("w:color"), "auto"); sh.set(qn("w:fill"), hexcol)
    tc.append(sh)


def table(doc, rows):
    cells = [[c.strip() for c in r.strip().strip("|").split("|")] for r in rows]
    cells = [c for c in cells if not all(re.fullmatch(r":?-{2,}:?", x) for x in c)]
    t = doc.add_table(rows=len(cells), cols=len(cells[0]))
    t.style = "Table Grid"
    t.alignment = WD_TABLE_ALIGNMENT.CENTER
    for i, row in enumerate(cells):
        for j, val in enumerate(row):
            c = t.cell(i, j)
            c.paragraphs[0].text = ""
            runs(c.paragraphs[0], val, size=8.5)
            if i == 0:
                shade(c, "D9E2F3")
                for r in c.paragraphs[0].runs:
                    r.bold = True
    doc.add_paragraph()


def main(md, out):
    doc = Document()
    st = doc.styles["Normal"]; st.font.name = "Calibri"; st.font.size = Pt(10.5)
    for s in doc.sections:
        s.left_margin = s.right_margin = Cm(2.0); s.top_margin = s.bottom_margin = Cm(1.8)
    lines = open(md, encoding="utf-8").read().splitlines()
    i = 0
    while i < len(lines):
        ln = lines[i]
        if ln.startswith("|"):
            block = []
            while i < len(lines) and lines[i].startswith("|"):
                block.append(lines[i]); i += 1
            table(doc, block)
            continue
        m = re.match(r"^(#{1,3}) (.*)", ln)
        img = re.match(r"!\[(.*?)\]\((.*?)\)", ln)
        if m:
            doc.add_heading(m.group(2), level=len(m.group(1)) - 1 if len(m.group(1)) > 1 else 0)
        elif img:
            doc.add_picture(os.path.join(os.path.dirname(md), img.group(2)), width=Cm(17))
            cap = doc.add_paragraph(img.group(1)); cap.runs[0].italic = True
        elif re.match(r"^\d+\. ", ln):
            p = doc.add_paragraph(); p.paragraph_format.left_indent = Cm(0.6); p.paragraph_format.first_line_indent = Cm(-0.6)
            runs(p, ln)
        elif ln.startswith("- "):
            p = doc.add_paragraph(style="List Bullet"); runs(p, ln[2:])
        elif ln.strip():
            p = doc.add_paragraph(); runs(p, ln)
        i += 1
    doc.save(out)


if __name__ == "__main__":
    src = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "PRACTICAL_1_REPORT.md")
    main(src, os.path.splitext(src)[0] + ".docx")
