#!/usr/bin/env python3
"""Tests for tools/mkdisk.py, the large-disk builder.

Runs the tool as a subprocess so the command line is covered too, and reads
every image back with the same FAT12 walker the tool uses -- a builder that
agrees with itself proves nothing, so the checks compare against the bytes
that went in.
"""
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from fat12 import Fat12                                    # noqa: E402
from mkdisk import read_fat12                              # noqa: E402

count = 0
fails = 0


def chk(desc, got, want):
    global count, fails
    count += 1
    if got != want:
        print("FAIL %-52s got=%r want=%r" % (desc, got, want))
        fails += 1


def chk_true(desc, cond):
    chk(desc, bool(cond), True)


def make_source(path):
    """A minimal bootable floppy: the two system files plus one more."""
    fs = Fat12(2880, 18, 2)
    fs.set_boot_code(b"\x90" * 100)
    fs.add_file("IO.SYS", b"io" * 3000)
    fs.add_file("MSDOS.SYS", b"ms" * 4000)
    fs.add_file("COMMAND.COM", b"cmd" * 2000)
    open(path, "wb").write(fs.image())


def run(*args):
    return subprocess.run([sys.executable, os.path.join(HERE, "mkdisk.py")]
                          + list(args), capture_output=True, text=True)


tmp = tempfile.mkdtemp()
src = os.path.join(tmp, "src.img")
out = os.path.join(tmp, "out.img")
extra = os.path.join(tmp, "PAYLOAD.BIN")
make_source(src)

payload = bytes((i * 31 + 7) & 0xFF for i in range(300000))   # ~37 clusters
open(extra, "wb").write(payload)

r = run(src, out, "--size", "16", extra)
chk("the tool succeeded", r.returncode, 0)

img = open(out, "rb").read()
chk("the image is the requested size", len(img), 16 * 1024 * 1024)
chk("the image is bootable", img[510:512], b"\x55\xaa")
chk("the OEM name came from the source", img[3:11], open(src, "rb").read()[3:11])

files = read_fat12(img)
names = [n for n, _ in files]

# The MS-DOS boot sector checks these two by POSITION, not by name lookup, so
# anything that reorders them produces a disk that looks fine and does not
# boot.
chk("IO.SYS is the first root entry", names[0], "IO.SYS")
chk("MSDOS.SYS is the second", names[1], "MSDOS.SYS")
chk_true("the other source files came across", "COMMAND.COM" in names)
chk_true("the added file is present", "PAYLOAD.BIN" in names)

by = dict(files)
chk("a large added file round-trips byte for byte", by["PAYLOAD.BIN"], payload)
chk("IO.SYS round-trips", by["IO.SYS"], b"io" * 3000)
chk("COMMAND.COM round-trips", by["COMMAND.COM"], b"cmd" * 2000)

# ---- the guards ----
# 16 MB at floppy geometry needs 909 cylinders and this BIOS keeps the
# cylinder in CH alone, so the image would silently read the wrong tracks.
r = run(src, out, "--size", "16", "--spt", "18", "--heads", "2")
chk("floppy geometry on a 16 MB image is refused", r.returncode != 0, True)
chk_true("...and says why", "cylinder" in r.stderr.lower())

plain = os.path.join(tmp, "plain.img")
open(plain, "wb").write(b"\x00" * 65536)
r = run(plain, out, "--size", "8")
chk("a source with no boot signature is refused", r.returncode != 0, True)

nosys = os.path.join(tmp, "nosys.img")
fs = Fat12(2880, 18, 2)
fs.set_boot_code(b"\x90" * 100)
fs.add_file("README.TXT", b"nothing bootable here")
open(nosys, "wb").write(fs.image())
r = run(nosys, out, "--size", "8")
chk("a source without IO.SYS is refused", r.returncode != 0, True)

print()
print("=" * 46)
print(" mkdisk: %d checks, %d failed" % (count, fails))
print("=" * 46)
sys.exit(1 if fails else 0)
