#!/usr/bin/env python3
"""
Put an existing disk image onto the board's block device.

    python3 tools/img2hex.py mydos.img

This converts a raw sector image into rom/disk.hex, which storage.sv loads into
on-chip ROM, and writes rom/geometry.py so the BIOS's INT 13h is built with the
same sectors-per-track and heads the image declares. Then:

    python3 tools/gen_bios.py rom/          # rebuild the BIOS with that geometry
    quartus_map/fit/sta/asm                 # rebuild the bitstream

TWO THINGS THAT WILL BITE YOU, both checked below.

GEOMETRY. INT 13h speaks cylinder/head/sector, and the BIOS turns that into a
flat sector number using constants compiled into it. A 1.44 MB floppy image
says 18 sectors per track and 2 heads; a 360 KB one says 9 and 2; this project
defaulted to 16 and 4. If the BIOS's arithmetic disagrees with what the image's
boot sector assumes, every single read lands on the wrong sector and the
failure looks like disk corruption rather than a configuration mistake. So the
geometry is taken FROM the image rather than assumed.

SIZE. The device is on-chip ROM inside the FPGA, and a Cyclone V 5CSEMA5 has
about 4 Mbit of it in total. Each 512-byte sector costs 4096 bits, and the rest
of the design already needs some, so roughly 700-800 sectors is the ceiling:

    360 KB floppy   720 sectors    fits, using most of what is left
    720 KB floppy  1440 sectors    DOES NOT FIT
    1.44 MB floppy 2880 sectors    DOES NOT FIT, by about 4x

An image too large to fit is reported rather than silently truncated. Getting
past that ceiling means backing the disk with the board's 64 MB SDRAM instead
of block RAM, which is a hardware change, not a tooling one.

AN .ISO IS NOT A FLOPPY IMAGE. A CD image is ISO 9660 with 2048-byte sectors,
and neither the FAT12 boot path nor a DOS boot sector can read it. Bootable
CDs carry a floppy image inside them (El Torito); that embedded image is what
you want. This script detects an ISO and says so rather than producing
something that cannot work.
"""
import os
import sys

SECTOR = 512

# Rough ceiling for the on-chip ROM backing. Not exact -- the fitter packs
# M10K blocks with some slack -- so this warns rather than refuses near the
# boundary, and refuses only where it is hopeless.
SECTORS_COMFORTABLE = 720
SECTORS_CEILING = 900


def fail(msg):
    sys.exit("error: " + msg)


def parse_bpb(b):
    """Read the BIOS parameter block. Returns None if it does not look like one."""
    def u16(o):
        return b[o] | (b[o + 1] << 8)

    if len(b) < SECTOR:
        return None
    if b[510] != 0x55 or b[511] != 0xAA:
        return None

    bpb = {
        "bytes_per_sector": u16(11),
        "sectors_per_cluster": b[13],
        "reserved": u16(14),
        "num_fats": b[16],
        "root_entries": u16(17),
        "total_sectors": u16(19),
        "media": b[21],
        "sectors_per_fat": u16(22),
        "sectors_per_track": u16(24),
        "heads": u16(26),
        "fs_type": bytes(b[54:62]).decode("ascii", "replace"),
        "oem": bytes(b[3:11]).decode("ascii", "replace"),
    }
    # A plausible BPB, not just a sector that happens to end in AA55.
    if bpb["bytes_per_sector"] not in (128, 256, 512, 1024, 2048, 4096):
        return None
    if bpb["num_fats"] not in (1, 2):
        return None
    return bpb


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__.strip())

    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    flags = set(a for a in sys.argv[1:] if a.startswith("--"))

    # With the disk backed by SDRAM there is no rom/disk.hex to write and no
    # block-RAM ceiling to respect -- the image goes over JTAG instead. But the
    # geometry still has to reach the BIOS, so this mode extracts just that.
    geom_only = "--sdram" in flags or "--geometry-only" in flags

    src = args[0]
    out = args[1] if len(args) > 1 else "rom/disk.hex"

    if not os.path.exists(src):
        fail("%s does not exist" % src)

    data = open(src, "rb").read()
    print("read %s: %d bytes (%.1f KB)" % (src, len(data), len(data) / 1024.0))

    # ---- is it a CD image? ----
    if len(data) > 0x8006 and data[0x8001:0x8006] == b"CD001":
        fail("this is an ISO 9660 CD image, not a raw disk image.\n"
             "       Its sectors are 2048 bytes and its filesystem is not FAT,\n"
             "       so neither the boot path nor a DOS boot sector can read it.\n"
             "       If it is a bootable CD it contains a floppy image (El\n"
             "       Torito) -- extract that and pass it instead. On Linux:\n"
             "         7z x image.iso '[BOOT]/*'\n"
             "       or mount it and look for a .img inside.")

    if len(data) % SECTOR:
        pad = SECTOR - (len(data) % SECTOR)
        print("  note: padding %d bytes to a whole sector" % pad)
        data += b"\0" * pad

    sectors = len(data) // SECTOR

    # ---- does it fit? ----
    print("  %d sectors of %d bytes" % (sectors, SECTOR))
    if sectors > SECTORS_CEILING and not geom_only:
        fail("%d sectors (%d KB) will not fit in the FPGA's on-chip memory.\n"
             "       The practical ceiling is around %d sectors (%d KB); this\n"
             "       image needs about %.1fx that.\n"
             "       A 1.44 MB floppy image cannot be used this way. Backing\n"
             "       the disk with the board's SDRAM instead of block RAM is\n"
             "       the way past it, and that is a hardware change."
             % (sectors, sectors // 2, SECTORS_CEILING, SECTORS_CEILING // 2,
                sectors / float(SECTORS_CEILING)))
    if sectors > SECTORS_COMFORTABLE and not geom_only:
        print("  WARNING: %d sectors is near the on-chip limit; if the fitter\n"
              "           runs out of M10K blocks, this is why." % sectors)

    # ---- geometry ----
    bpb = parse_bpb(data)
    if bpb is None:
        print("  WARNING: no usable BIOS parameter block found (no AA55 signature,\n"
              "           or the fields do not look like one). The image will be\n"
              "           written, but the BIOS geometry is left unchanged and\n"
              "           nothing here can check that it matches.")
        spt = heads = None
    else:
        print("  OEM name          : %r" % bpb["oem"])
        print("  filesystem type   : %r" % bpb["fs_type"])
        print("  bytes per sector  : %d" % bpb["bytes_per_sector"])
        print("  sectors/cluster   : %d" % bpb["sectors_per_cluster"])
        print("  reserved sectors  : %d" % bpb["reserved"])
        print("  FAT copies        : %d" % bpb["num_fats"])
        print("  root entries      : %d" % bpb["root_entries"])
        print("  total sectors     : %d" % bpb["total_sectors"])
        print("  sectors per track : %d" % bpb["sectors_per_track"])
        print("  heads             : %d" % bpb["heads"])

        if bpb["bytes_per_sector"] != SECTOR:
            fail("the image uses %d-byte sectors; this device is built for %d."
                 % (bpb["bytes_per_sector"], SECTOR))

        spt = bpb["sectors_per_track"]
        heads = bpb["heads"]
        if not spt or not heads:
            print("  WARNING: the BPB declares zero sectors-per-track or heads,\n"
                  "           so the BIOS geometry is left unchanged.")
            spt = heads = None
        elif bpb["total_sectors"] and bpb["total_sectors"] != sectors:
            print("  WARNING: the BPB says %d sectors but the file holds %d."
                  % (bpb["total_sectors"], sectors))

    # ---- emit ----
    if geom_only:
        print("  (geometry only: the image goes to SDRAM over JTAG, so no")
        print("   rom/disk.hex is written)")
    else:
        os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
        with open(out, "w") as f:
            for i in range(0, len(data), 2):
                f.write("%04x\n" % (data[i] | (data[i + 1] << 8)))
        print("wrote %s" % out)

    if spt is not None:
        geo = os.path.join(os.path.dirname(out) or ".", "geometry.py")
        with open(geo, "w") as f:
            f.write("# Written by tools/img2hex.py from %s.\n" % os.path.basename(src))
            f.write("# gen_bios.py and gen_disk.py import this so the BIOS's\n")
            f.write("# INT 13h describes the same disk the image was built for.\n")
            f.write("SPT = %d\n" % spt)
            f.write("HEADS = %d\n" % heads)
            f.write("SECTORS = %d\n" % sectors)
        print("wrote %s (SPT=%d HEADS=%d SECTORS=%d)" % (geo, spt, heads, sectors))

    print()
    print("next:")
    if geom_only:
        print("  1. python3 tools/gen_bios.py rom/    # picks up the geometry")
        print("  2. reload the BIOS (ismce_load.tcl) or rebuild the bitstream")
        print("  3. quartus_stp -t tools/jtag_load.tcl %s" % src)
    else:
        print("  1. set SECTORS in modules/storage.sv to %d" % sectors)
        print("  2. python3 tools/gen_bios.py rom/    # picks up the geometry")
        print("  3. rebuild the bitstream (quartus_map / fit / sta / asm)")


if __name__ == "__main__":
    main()
