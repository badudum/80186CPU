#!/usr/bin/env python3
"""
Render the text buffer saved by tools/screen_dump.tcl.

    quartus_stp -t tools/screen_dump.tcl
    python3 tools/screen_dump.py

Prints the 80x25 screen as text, and optionally the attribute bytes, so the
machine can be debugged with no monitor attached.

The .mif files Quartus writes back use BINARY data and run-length ranges of the
form `[0..7FF] : 20;` for repeated values -- neither of which a naive reader
handles, and both of which quietly produce nonsense if ignored.
"""
import os
import re
import sys

COLS, ROWS = 80, 25
DEFAULT_DIR = "/tmp/fpga80186_screen"


def load_mif(path):
    """Address -> value. Handles BIN/HEX radix and [a..b] ranges."""
    if not os.path.exists(path):
        sys.exit("error: %s does not exist -- run tools/screen_dump.tcl first"
                 % path)

    addr_radix = "HEX"
    data_radix = "HEX"
    out = {}
    in_content = False

    for line in open(path):
        line = line.split("--")[0].strip()
        if not line:
            continue

        m = re.match(r"ADDRESS_RADIX\s*=\s*(\w+)", line, re.I)
        if m:
            addr_radix = m.group(1).upper()
            continue
        m = re.match(r"DATA_RADIX\s*=\s*(\w+)", line, re.I)
        if m:
            data_radix = m.group(1).upper()
            continue
        if re.match(r"CONTENT\s+BEGIN", line, re.I):
            in_content = True
            continue
        if re.match(r"END\s*;", line, re.I):
            in_content = False
            continue
        if not in_content:
            continue

        abase = {"BIN": 2, "OCT": 8, "DEC": 10, "HEX": 16, "UNS": 10}.get(addr_radix, 16)
        dbase = {"BIN": 2, "OCT": 8, "DEC": 10, "HEX": 16, "UNS": 10}.get(data_radix, 16)

        # [lo..hi] : value;
        m = re.match(r"\[\s*([0-9A-Fa-f]+)\s*\.\.\s*([0-9A-Fa-f]+)\s*\]\s*:\s*([0-9A-Fa-f]+)\s*;", line)
        if m:
            lo = int(m.group(1), abase)
            hi = int(m.group(2), abase)
            v = int(m.group(3), dbase)
            for a in range(lo, hi + 1):
                out[a] = v
            continue

        # addr : value;
        m = re.match(r"([0-9A-Fa-f]+)\s*:\s*([0-9A-Fa-f]+)\s*;", line)
        if m:
            out[int(m.group(1), abase)] = int(m.group(2), dbase)

    return out


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_DIR
    lo = load_mif(os.path.join(d, "vram_lo.mif"))
    hi = load_mif(os.path.join(d, "vram_hi.mif"))

    print("read %d character cells, %d attribute cells" % (len(lo), len(hi)))
    print()
    print("     +" + "-" * COLS + "+")
    for r in range(ROWS):
        line = ""
        for c in range(COLS):
            ch = lo.get(r * COLS + c, 0x20)
            line += chr(ch) if 0x20 <= ch < 0x7F else ("." if ch else " ")
        print("  %2d |%s|" % (r, line))
    print("     +" + "-" * COLS + "+")

    # Attributes matter when the screen looks wrong: a cell with a sane
    # character but a black-on-black attribute is invisible, and one with a
    # stray character usually has a stray attribute beside it.
    if "-a" in sys.argv or "--attrs" in sys.argv:
        print()
        print("attributes (hex), first 8 rows:")
        for r in range(min(8, ROWS)):
            print("  %2d %s" % (r, " ".join("%02X" % hi.get(r * COLS + c, 0)
                                            for c in range(min(32, COLS)))))

    # Anything outside the printable range is worth calling out explicitly:
    # it is the usual signature of a buffer that was never initialised or of
    # writes landing at the wrong address.
    odd = sorted({v for v in lo.values() if not (0x20 <= v < 0x7F)})
    if odd:
        print()
        print("non-printable character codes present: %s"
              % " ".join("%02X" % v for v in odd[:16]))


if __name__ == "__main__":
    main()
