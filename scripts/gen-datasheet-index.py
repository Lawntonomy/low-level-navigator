#!/usr/bin/env python3
"""Regenerate docs/rp2350-datasheet-index.md from the datasheet's own table of
contents.

The datasheet PDF is ~1380 pages and the Read tool caps at 20 pages per call, so
an index is what makes targeted lookups possible. Only section titles and page
numbers are extracted -- factual metadata, not datasheet content. The PDF stays
the single source of truth.

Usage:  python3 scripts/gen-datasheet-index.py [path/to/datasheet.pdf]

Requires pdftotext (poppler-utils). Re-run if the datasheet is replaced with a
newer revision; TOC_LAST_PAGE and the printed->PDF offset may need adjusting.
"""

import pathlib
import re
import subprocess
import sys

DEFAULT_PDF = "docs/RP-008373-DS-2-rp2350-datasheet.pdf"
OUT = pathlib.Path("docs/rp2350-datasheet-index.md")
TOC_LAST_PAGE = 13

# Printed page numbers in the ToC are offset from PDF page numbers by the front
# matter. Verified: ToC lists "11. PIO" at printed 876; PDF page 877 renders
# with footer "876".
PDF_OFFSET = 1

ENTRY = re.compile(r"^(\s*)(.+?)\s*[.\s]{4,}\s*(\d+)\s*$")
NUMBERED = re.compile(r"^(\d+(?:\.\d+)*)\.\s+(.*)$")

# Sections worth surfacing for this project: PIO/DMA drive the encoders and
# motor PWM, and the errata list is the first place to look when hardware
# behaviour contradicts the prose.
CURATED = [
    ("PIO -- encoder capture and motor PWM", [
        "11. PIO", "11.2.1. PIO programs", "11.2.7. IRQ flags",
        "11.2.8. Interactions between state machines", "11.3. PIO assembler (pioasm)",
        "11.4. Instruction Set", "11.5.3. FIFO joining", "11.5.5. Clock Dividers",
        "11.5.6. GPIO mapping",
    ]),
    ("DMA -- encoder ring buffers", [
        "12.6. DMA", "12.13.4. DMA DREQ interface", "4.4.3. Streaming DMA interface",
    ]),
    ("GPIO", ["9. GPIO", "9.8. Processor GPIO controls (SIO)", "9.10.2. Enable a GPIO interrupt"]),
    ("Clocks, timers, watchdog", [
        "8. Clocks", "8.5. Tick generators", "12.8. System timers", "12.9. Watchdog",
        "12.9.3. Watchdog counter", "12.9.4. Control watchdog reset levels",
    ]),
    ("Multicore and cross-core state", [
        "3.1. SIO", "3.1.2. CPUID", "3.1.4. Hardware spinlocks",
        "3.1.5. Inter-processor FIFOs (Mailboxes)", "3.1.6. Doorbells",
        "2.1.3. Atomic register access",
    ]),
    ("Errata -- check before trusting the prose", ["Appendix E: Errata"]),
]


def parse_toc(pdf: str):
    text = subprocess.run(
        ["pdftotext", "-f", "1", "-l", str(TOC_LAST_PAGE), "-layout", pdf, "-"],
        capture_output=True, text=True, check=True,
    ).stdout

    rows, started = [], False
    for line in text.splitlines():
        if "Table of contents" in line:
            started = True
            continue
        if not started or not line.strip() or line.strip() == "RP2350 Datasheet":
            continue
        m = ENTRY.match(line)
        if m:
            rows.append((m.group(2).strip(), int(m.group(3))))
    return rows


def main():
    pdf = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PDF
    if not pathlib.Path(pdf).exists():
        sys.exit(f"not found: {pdf}")

    rows = parse_toc(pdf)
    if not rows:
        sys.exit("parsed 0 entries -- has the ToC layout changed?")

    by_title = {t: p for t, p in rows}
    out = []
    w = out.append

    w("# RP2350 Datasheet -- Section Index")
    w("")
    w(f"Generated from `{pathlib.Path(pdf).name}` by `scripts/gen-datasheet-index.py`.")
    w("Section titles and page numbers only -- the PDF remains the source of truth.")
    w("")
    w("## Reading the datasheet")
    w("")
    w(f"**PDF page = printed page + {PDF_OFFSET}.** Page numbers printed in the footer (and listed")
    w("in the datasheet's own ToC) run one behind the PDF page index. Both are given below.")
    w("")
    w("Two ways to read it, both available to the `rp2350-researcher` agent:")
    w("")
    w("| Method | Returns | Limit |")
    w("| --- | --- | --- |")
    w('| `Read` with `pages: "877-878"` | Rendered pages, including figures and block diagrams | 20 pages per call |')
    w("| `pdftotext -f 877 -l 878 -layout <pdf> -` | Text only, greppable across all pages | none |")
    w("")
    w("Use `Read` when a diagram, register layout, or table matters -- which for PIO and DMA")
    w("is most of the time. Use `pdftotext` piped to `grep` to locate a term across the whole")
    w("document before committing a 20-page read to it.")
    w("")
    w("## Start here")
    w("")
    for heading, titles in CURATED:
        w(f"**{heading}**")
        w("")
        for t in titles:
            p = by_title.get(t)
            if p is not None:
                w(f"- {t} — printed {p}, **PDF {p + PDF_OFFSET}**")
        w("")

    w("## Full index")
    w("")
    w("Indented by section depth. `printed` / **`PDF`**.")
    w("")
    for title, page in rows:
        m = NUMBERED.match(title)
        depth = m.group(1).count(".") if m else 1
        indent = "  " * min(depth, 4)
        w(f"{indent}- {title} — {page} / **{page + PDF_OFFSET}**")
    w("")

    OUT.write_text("\n".join(out))
    print(f"wrote {OUT} ({len(rows)} entries, {OUT.stat().st_size // 1024} KB)")


if __name__ == "__main__":
    main()
