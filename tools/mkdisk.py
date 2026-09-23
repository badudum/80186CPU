#!/usr/bin/env python3
"""Build a large bootable MS-DOS disk image from a floppy plus extra files.

    python3 tools/mkdisk.py ms-dos/disk01.img out.img --size 16 FILE...

The machine's disk lives in SDRAM and storage.sv takes any sector count, so
the 1.44 MB the boot floppy is stuck at is not a hardware limit -- it is just
the size of the image being loaded. This builds a bigger one: same MS-DOS boot
sector and system files, a BPB describing the new geometry, and whatever else
you name on the command line.

WHY NOT JUST COPY THE FLOPPY AND MAKE IT BIGGER. A FAT volume's size is baked
into its BPB, its FAT length and its root directory. Growing one means
rebuilding all three and relocating every file, which is what this does.

THREE CONSTRAINTS, none obvious, all of which produce a disk that looks right
and does not boot:

  * IO.SYS MUST BE THE FIRST ROOT ENTRY AND MSDOS.SYS THE SECOND. The MS-DOS
    boot sector does not search the directory -- it compares entry 0 against
    "IO      SYS" and entry 1 against "MSDOS   SYS" and gives up if either
    fails. That is why this writes them before anything else.

  * FAT12 HAS 4084 USABLE CLUSTER NUMBERS. Past about 2 MB the clusters have
    to get bigger rather than more numerous; fat12.py picks a size and refuses
    combinations it cannot number.

  * THE CYLINDER NUMBER MUST FIT IN EIGHT BITS. A real BIOS puts cylinder bits
    8 and 9 in CL[7:6]; this BIOS's INT 13h reads the cylinder from CH alone,
    so anything over 256 cylinders silently reads the wrong track. A 16 MB
    image at 18 sectors and 2 heads would need 909 of them. Geometry with more
    sectors and heads -- 63 x 16 is the usual hard-disk shape -- keeps the
    count small, and this refuses to build an image that would overflow.

SIXTEEN MEGABYTES IS THE CEILING, and it is not arbitrary: storage.sv forms
its SDRAM address as BASE + {lba[14:0], 9'b0}, which is fifteen bits of sector
number. 32,768 sectors is exactly what that reaches.

USING ONE OF THESE IMAGES takes three steps, and skipping any of them gives a
disk the machine half-reads:

  1. Tell the BIOS the new geometry and rebuild the ROM. INT 13h converts CHS
     to LBA with the sectors-per-track and heads it was BUILT with, so a ROM
     built for a floppy reads the wrong sectors off this disk.

         python3 tools/img2hex.py --sdram out.img   # writes rom/geometry.py
         python3 tools/gen_bios.py rom/

  2. Tell the hardware how big the device is. DISK_SECTORS is a synthesis
     parameter, so this one needs a rebuild rather than a JTAG reload:

         set_parameter -name DISK_SECTORS 32768      # in FPGA80186.qsf
         quartus_sh --flow compile FPGA80186

  3. Program and load:

         quartus_pgm -m jtag -o "p;output_files/FPGA80186.sof@2"
         quartus_stp -t tools/jtag_load.tcl out.img

If only the BIOS changed, steps 1 and 3 can skip the rebuild by pushing the
ROM in over JTAG instead:

    quartus_stp -t tools/ismce_load.tcl BIOL rom/bios.lo.mif
    quartus_stp -t tools/ismce_load.tcl BIOH rom/bios.hi.mif

NOT YET RUN ON HARDWARE. Whether MS-DOS is happy being booted from a 16 MB
volume with hard-disk geometry on what it is told is drive 0 is the open
question; the image itself is verified byte for byte by tools/test_mkdisk.py.
"""
import argparse
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fat12 import Fat12, Fat12Error                       # noqa: E402

SECTOR = 512
MAX_CYLINDERS = 256          # this BIOS keeps the cylinder in CH alone


def read_fat12(img):
    """Pull every root-directory file out of a FAT12 image, in order."""
    reserved = struct.unpack_from("<H", img, 14)[0]
    num_fats = img[16]
    root_entries = struct.unpack_from("<H", img, 17)[0]
    spf = struct.unpack_from("<H", img, 22)[0]
    spc = img[13]
    root_lba = reserved + num_fats * spf
    data_lba = root_lba + (root_entries * 32 + SECTOR - 1) // SECTOR
    fat = img[reserved * SECTOR: reserved * SECTOR + spf * SECTOR]

    def entry(c):
        o = (c * 3) // 2
        v = fat[o] | (fat[o + 1] << 8)
        return (v >> 4) if (c & 1) else (v & 0xFFF)

    out = []
    for i in range(root_entries):
        e = root_lba * SECTOR + i * 32
        if img[e] == 0x00:
            break
        if img[e] == 0xE5 or (img[e + 11] & 0x08):     # deleted, or the label
            continue
        name = img[e:e + 8].decode("latin1").rstrip()
        ext = img[e + 8:e + 11].decode("latin1").rstrip()
        size = struct.unpack_from("<I", img, e + 28)[0]
        c = struct.unpack_from("<H", img, e + 26)[0]
        blob = bytearray()
        while 2 <= c < 0xFF0 and len(blob) < size + spc * SECTOR:
            off = (data_lba + (c - 2) * spc) * SECTOR
            blob += img[off: off + spc * SECTOR]
            c = entry(c)
        out.append((name + ("." + ext if ext else ""), bytes(blob[:size])))
    return out


def pick_cluster_size(total_sectors, root_entries):
    """Smallest cluster that keeps the volume inside FAT12's 4084 numbers."""
    for spc in (1, 2, 4, 8, 16, 32, 64):
        try:
            Fat12(total_sectors, 63, 16, root_entries=root_entries,
                  sectors_per_cluster=spc)
            return spc
        except Fat12Error:
            continue
    raise Fat12Error("no cluster size makes a FAT12 volume of %d sectors"
                     % total_sectors)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("source", help="a bootable MS-DOS floppy image")
    ap.add_argument("output", help="the image to write")
    ap.add_argument("--size", type=int, default=16, help="megabytes (default 16)")
    ap.add_argument("--spt", type=int, default=63, help="sectors per track")
    ap.add_argument("--heads", type=int, default=16, help="heads")
    ap.add_argument("--root-entries", type=int, default=512)
    ap.add_argument("add", nargs="*", help="extra files to place on the volume")
    args = ap.parse_args()

    total_sectors = args.size * 1024 * 1024 // SECTOR
    cylinders = total_sectors // (args.spt * args.heads)
    if cylinders > MAX_CYLINDERS:
        ap.error("%d MB at %d sectors x %d heads needs %d cylinders; this "
                 "BIOS's INT 13h keeps the cylinder in CH and cannot address "
                 "more than %d. Raise --spt or --heads."
                 % (args.size, args.spt, args.heads, cylinders, MAX_CYLINDERS))

    src = open(args.source, "rb").read()
    if src[510:512] != b"\x55\xaa":
        ap.error("%s has no boot signature; it is not a bootable image"
                 % args.source)

    files = read_fat12(src)
    by_name = dict(files)
    for required in ("IO.SYS", "MSDOS.SYS"):
        if required not in by_name:
            ap.error("%s has no %s, so nothing built from it would boot"
                     % (args.source, required))

    spc = pick_cluster_size(total_sectors, args.root_entries)
    # Carry the source's OEM name across rather than stamping our own. It is
    # what SYS.COM does, and some DOS versions look at it.
    fs = Fat12(total_sectors, args.spt, args.heads,
               root_entries=args.root_entries, sectors_per_cluster=spc,
               oem=src[3:11].decode("latin1"))

    # The boot sector's code, with this volume's BPB. Bytes 0-2 are the jump
    # and 62 onwards is the loader; everything between is the BPB that fat12.py
    # has just computed, so only the code is carried over.
    fs.set_boot_code(src[62:510])

    # IO.SYS first, MSDOS.SYS second, before anything else can take those
    # slots. The boot sector checks them by position, not by name lookup.
    order = ["IO.SYS", "MSDOS.SYS"]
    order += [n for n, _ in files if n not in order]
    for name in order:
        fs.add_file(name, by_name[name], attrs=0x27 if name in
                    ("IO.SYS", "MSDOS.SYS") else 0x20)

    for path in args.add:
        data = open(path, "rb").read()
        fs.add_file(os.path.basename(path).upper(), data)

    img = fs.image()
    with open(args.output, "wb") as f:
        f.write(img)

    used = sum(1 for c in range(2, fs.cluster_count + 2) if fs.fat[c])
    print(fs.describe())
    print("geometry : %d cylinders x %d heads x %d sectors"
          % (cylinders, args.heads, args.spt))
    print("clusters : %d of %d bytes, %d used, %.1f MB free"
          % (fs.cluster_count, fs.cluster_bytes, used,
             (fs.cluster_count - used) * fs.cluster_bytes / 1048576.0))
    print("files    :")
    for name in order:
        print("   %-13s %8d" % (name, len(by_name[name])))
    for path in args.add:
        print("   %-13s %8d" % (os.path.basename(path).upper(),
                                os.path.getsize(path)))
    print("wrote %s: %d sectors (%d MB)"
          % (args.output, total_sectors, args.size))


if __name__ == "__main__":
    main()
