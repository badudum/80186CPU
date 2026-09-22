#!/usr/bin/env python3
"""
Build the block-storage image: a FAT12 volume with a boot sector and a kernel.

The device is word-addressed (512-byte sectors = 256 words of 16 bits), so this
writes one 4-digit hex word per line rather than the byte-per-line format the
BIOS and font images use.

WHAT BOOTS, AND HOW. The image is a real FAT12 filesystem (see fat12.py). Its
boot sector is 8086 code that reads the BIOS parameter block at runtime, loads
the FAT and the root directory, searches for KERNEL.BIN by name, follows its
cluster chain, loads it at 1000:0000 and jumps to it. Nothing about the layout
is hardcoded: move the file, resize the volume, or change the geometry and the
same boot sector still finds it.

That matters beyond tidiness. The boot sector is an independent implementation
of the format -- written in assembly, running on the CPU under test -- reading
structures a Python program wrote. With no mtools or dosfstools on this machine
to check the image against, two implementations agreeing is the strongest
validation available, and it is a genuinely different one from the Python
reader in test_fat12.py.

GEOMETRY. The BPB records sectors-per-track and heads, and the boot sector uses
them to turn a logical sector into the CHS triple INT 13h wants. They must
match what the BIOS's INT 13h actually does, so they are passed in from the
same constants rather than guessed.

Usage:  python3 tools/gen_disk.py [sectors] [outfile]
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from asm86 import (Asm, mem,
                   AX, CX, DX, BX, SP, BP, SI, DI,
                   AL, CL, DL, BL, AH, CH, DH, BH,
                   ES, CS, SS, DS)
from fat12 import Fat12

SECTORS = int(sys.argv[1]) if len(sys.argv) > 1 else 256
OUT = sys.argv[2] if len(sys.argv) > 2 else "rom/disk.hex"

# Must agree with SPT/HEADS in gen_bios.py -- the BIOS's INT 13h and the boot
# sector's LBA-to-CHS arithmetic have to describe the same disk. Both read
# rom/geometry.py when it exists, for exactly that reason: gen_bios.py picking
# up an uploaded image's geometry while this file kept the built-in default
# left the two describing different disks, and they agreed only for sectors
# below the first head boundary, where every mapping gives the same answer.
SPT = 16
HEADS = 4

# The size of the disk the BIOS was BUILT for, which is not the size of the
# test image built here: the BIOS describes whatever image was last uploaded,
# and INT 13h AH=08 reports that geometry.
BIOS_SECTORS = 256

try:
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    "..", "rom"))
    from geometry import SPT as _SPT, HEADS as _HEADS, SECTORS as _BS  # noqa: E402
    SPT, HEADS, BIOS_SECTORS = _SPT, _HEADS, _BS
    print("geometry from rom/geometry.py: SPT=%d HEADS=%d SECTORS=%d"
          % (SPT, HEADS, BIOS_SECTORS))
except ImportError:
    pass

KERNEL_NAME = "KERNEL.BIN"
KERNEL_SEG = 0x1000            # loaded at 1000:0000

# Where the boot sector parks what it reads. Both sit above the boot sector and
# below the stack it set up at 7C00... except they do not: the stack grows DOWN
# from 7C00 and these grow up from 7E00, so they never meet.
FAT_BUF = 0x7E00
ROOT_BUF = 0x8000

# BPB fields, as absolute addresses once the sector is loaded at 0000:7C00.
BPB = 0x7C00
B_SECSIZE = BPB + 0x0B
B_CLUSTSZ = BPB + 0x0D
B_RESERVED = BPB + 0x0E
B_NUMFATS = BPB + 0x10
B_ROOTENT = BPB + 0x11
B_SPF = BPB + 0x16
B_SPT = BPB + 0x18
B_HEADS = BPB + 0x1A

# Scratch words, at fixed addresses inside the boot sector itself. It is in RAM
# once loaded, and the tail of the sector is free -- the code is checked below
# not to reach them. Fixed addresses keep every reference a plain displacement
# instead of something that has to be back-patched once the layout is known.
V_ROOT_LBA = BPB + 0x1F0
V_ROOT_SECS = BPB + 0x1F2
V_DATA_LBA = BPB + 0x1F4
V_CLUSTER = BPB + 0x1F6
SCRATCH_START = 0x1F0

ECHO_KEYS = 5                  # keystrokes the kernel echoes before halting

# KERNEL.BIN is padded to span several clusters on purpose. A file that fits in
# one cluster never makes the boot sector follow a chain, so the 12-bit FAT
# unpacking -- the part of FAT12 that is genuinely easy to get wrong -- would
# go untested in hardware. Five clusters starting at cluster 2 covers both even
# and odd entries, which take opposite halves of the byte they share.
KERNEL_SECTORS = 5
KERNEL_BYTES = KERNEL_SECTORS * 512
KERNEL_TAIL = KERNEL_BYTES - 2     # where the end-of-file marker lives
KERNEL_MAGIC = 0xC0DE


# ===========================================================================
# The boot sector
# ===========================================================================
def build_boot_code():
    """Code that sits at offset 3Eh of the boot sector, i.e. 0000:7C3E."""
    a = Asm(512, origin=0x7C3E)
    a.org(0x7C3E)

    # Entered with CS=0000, IP=7C3E and an unknown DS/ES/SS.
    a.cli()
    a.mov(AX, 0x0000)
    a.mov(DS, AX)
    a.mov(ES, AX)
    a.mov(SS, AX)
    a.mov(SP, 0x7C00)
    a.sti()
    a.cld()

    a.mov_label(SI, "m_load")
    a.call("puts")

    # ---- the FAT, at the reserved-sector count ----
    a.mov(AX, mem(disp=B_RESERVED))
    a.mov(CX, mem(disp=B_SPF))
    a.mov(BX, FAT_BUF)
    a.call("read_many")

    # ---- the root directory, after all the FAT copies ----
    a.mov(AL, mem(disp=B_NUMFATS))
    a.mov(AH, 0)
    a.mul(mem(disp=B_SPF))                 # AX = num_fats * sectors_per_fat
    a.add(AX, mem(disp=B_RESERVED))
    a.mov(mem(disp=V_ROOT_LBA), AX)

    a.mov(AX, mem(disp=B_ROOTENT))
    a.shr(AX, 4)                           # 32-byte entries, 512-byte sectors
    a.mov(mem(disp=V_ROOT_SECS), AX)

    a.mov(CX, AX)
    a.mov(AX, mem(disp=V_ROOT_LBA))
    a.mov(BX, ROOT_BUF)
    a.call("read_many")

    # ---- first data sector ----
    a.mov(AX, mem(disp=V_ROOT_LBA))
    a.add(AX, mem(disp=V_ROOT_SECS))
    a.mov(mem(disp=V_DATA_LBA), AX)

    # ---- find the file ----
    a.mov(CX, mem(disp=B_ROOTENT))
    a.mov(DI, ROOT_BUF)
    a.label("scan")
    a.push(CX)
    a.push(DI)
    a.mov_label(SI, "fname")
    a.mov(CX, 11)
    a.repe()
    a.cmpsb()
    a.pop(DI)
    a.pop(CX)
    a.jz("found")
    a.add(DI, 32)
    a.loop("scan")

    a.mov_label(SI, "m_nofile")
    a.call("puts")
    a.jmps("stop")

    a.label("found")
    a.mov(AX, mem(DI, disp=26))            # starting cluster
    a.mov(mem(disp=V_CLUSTER), AX)

    # ---- walk the chain into KERNEL_SEG:0000 ----
    a.mov(AX, KERNEL_SEG)
    a.mov(ES, AX)
    a.mov(BX, 0x0000)

    a.label("load")
    a.mov(AX, mem(disp=V_CLUSTER))
    # 0FF0 and above are the reserved and end-of-chain values.
    a.cmp(AX, 0x0FF0)
    a.jnc("loaded")
    a.add(AX, mem(disp=V_DATA_LBA))
    a.sub(AX, 2)                           # cluster 2 is the first data sector
    a.mov(CX, 1)
    a.call("read_many")
    a.add(BX, 512)
    a.mov(AX, mem(disp=V_CLUSTER))
    a.call("fat_next")
    a.mov(mem(disp=V_CLUSTER), AX)
    a.jmps("load")

    a.label("loaded")
    a.mov_label(SI, "m_go")
    a.call("puts")
    a.mov(DX, 0x0000)                      # DL = boot drive
    a.jmpf(KERNEL_SEG, 0x0000)

    a.label("stop")
    a.hlt()
    a.jmps("stop")

    # -----------------------------------------------------------------
    # fat_next -- AX = cluster, returns the next one.
    #
    # Two 12-bit entries share three bytes, so entry n starts at byte
    # n + n/2 and is taken from the low or high nibble of the byte they
    # share depending on whether n is odd. Getting only the even case right
    # works until a file is long enough to reach an odd cluster.
    # -----------------------------------------------------------------
    a.label("fat_next")
    a.push(BX)
    a.push(DX)
    a.mov(BX, AX)
    a.shr(BX, 1)
    a.add(BX, AX)                          # BX = n + n/2
    a.mov(DX, mem(BX, disp=FAT_BUF))
    a.test(AL, 1)
    a.jz("fn_even")
    a.shr(DX, 4)
    a.jmps("fn_done")
    a.label("fn_even")
    a.and_(DX, 0x0FFF)
    a.label("fn_done")
    a.mov(AX, DX)
    a.pop(DX)
    a.pop(BX)
    a.ret()

    # -----------------------------------------------------------------
    # read_many -- AX = first LBA, CX = count, ES:BX = destination.
    # -----------------------------------------------------------------
    a.label("read_many")
    a.pusha()
    a.label("rm_loop")
    a.push(CX)
    a.call("read_one")
    a.pop(CX)
    a.inc(AX)
    a.add(BX, 512)
    a.loop("rm_loop")
    a.popa()
    a.ret()

    # -----------------------------------------------------------------
    # read_one -- AX = LBA, ES:BX = destination. Converts to CHS using the
    # geometry the BPB declares, not a compiled-in assumption.
    # -----------------------------------------------------------------
    # BX is deliberately NOT saved across the INT 13h call. It is the buffer
    # offset the BIOS is handed, and a BIOS has to give it back -- real boot
    # sectors rely on that, and ours saving it privately hid the fact that ours
    # did not. read_many below walks BX forward between sectors, so if INT 13h
    # ever clobbers it again the kernel loads into the wrong place and the test
    # fails instead of the hardware.
    a.label("read_one")
    a.push(AX)
    a.push(CX)
    a.push(DX)
    a.mov(DX, 0)
    a.div(mem(disp=B_SPT))                 # AX = LBA/SPT, DX = LBA%SPT
    a.inc(DL)                              # sectors are numbered from 1
    a.mov(CL, DL)
    a.mov(DX, 0)
    a.div(mem(disp=B_HEADS))               # AX = cylinder, DX = head
    a.mov(CH, AL)
    a.mov(DH, DL)
    a.mov(DL, 0)                           # drive 0
    a.mov(AX, 0x0201)                      # AH=02 read, AL=1 sector
    a.int_(0x13)
    a.jc("rd_err")
    a.pop(DX)
    a.pop(CX)
    a.pop(AX)
    a.ret()

    a.label("rd_err")
    a.mov_label(SI, "m_readerr")
    a.call("puts")
    a.jmps("stop")

    # -----------------------------------------------------------------
    # puts -- DS:SI, NUL-terminated, through INT 10h.
    # -----------------------------------------------------------------
    a.label("puts")
    a.push(AX)
    a.push(BX)
    a.push(SI)
    a.label("ps_loop")
    a.lodsb()
    a.cmp(AL, 0)
    a.jz("ps_end")
    a.mov(AH, 0x0E)
    a.mov(BL, 0x07)
    a.int_(0x10)
    a.jmps("ps_loop")
    a.label("ps_end")
    a.pop(SI)
    a.pop(BX)
    a.pop(AX)
    a.ret()

    # ---- strings and scratch ----
    a.label("fname")
    for ch in (KERNEL_NAME.split(".")[0].ljust(8)
               + KERNEL_NAME.split(".")[1].ljust(3)):
        a.db(ord(ch))

    a.label("m_load")
    a.dz("Loading " + KERNEL_NAME + "\r\n")
    a.label("m_nofile")
    a.dz("File not found.\r\n")
    a.label("m_readerr")
    a.dz("Read error.\r\n")
    a.label("m_go")
    a.dz("Starting kernel.\r\n")

    a.resolve()

    end = a.here() - BPB
    if end > SCRATCH_START:
        raise SystemExit("boot code reaches %03X, over the scratch words at %03X"
                         % (end, SCRATCH_START))

    return bytes(a.buf[0:a.pos])


# ===========================================================================
# KERNEL.BIN -- what the boot sector loads and runs
# ===========================================================================
def build_kernel():
    a = Asm(4096, origin=0x0000)
    a.org(0x0000)

    # Entered at KERNEL_SEG:0000 by a far jump, so CS is already right. Taking
    # DS and ES from CS rather than a constant means the kernel does not care
    # where the boot sector chose to put it.
    a.mov(AX, CS)
    a.mov(DS, AX)
    a.mov(ES, AX)

    # Before announcing anything, check the LAST word of the file. It only
    # arrives if every cluster of the chain was followed and loaded in order,
    # so this turns "the kernel ran" into "the whole file is here".
    a.mov(AX, mem(disp=KERNEL_TAIL))
    a.cmp(AX, KERNEL_MAGIC)
    a.jz("k_tail_ok")
    a.mov_label(SI, "k_bad")
    a.call("k_puts")
    a.jmps("k_stop")

    a.label("k_tail_ok")
    a.mov_label(SI, "k_msg")
    a.call("k_puts")

    # Ask the BIOS what shape the disk is and leave the answer at 0000:0600
    # for the testbench. INT 13h AH=08 is the one service nothing else here
    # calls, and it reported a disk exactly one cylinder long for a long time
    # because nothing ever looked.
    a.mov(AH, 0x08)
    a.mov(DL, 0x00)
    a.int_(0x13)
    a.push(CX)
    a.push(DX)
    a.xor(AX, AX)
    a.mov(DS, AX)
    a.pop(AX)
    a.mov(mem(disp=0x0602), AX)          # DX: DH = last head, DL = drives
    a.pop(AX)
    a.mov(mem(disp=0x0600), AX)          # CX: CH = last cylinder, CL = SPT
    a.mov(AX, CS)
    a.mov(DS, AX)

    # Echo a few keystrokes, which exercises the whole keyboard chain from the
    # pins up through INT 16h -- from code that was loaded off a filesystem.
    a.mov(CX, ECHO_KEYS)
    a.label("k_loop")
    a.push(CX)
    a.mov(AH, 0x00)
    a.int_(0x16)
    a.mov(AH, 0x0E)
    a.mov(BL, 0x0B)
    a.int_(0x10)
    a.pop(CX)
    a.loop("k_loop")

    a.label("k_stop")
    a.hlt()
    a.jmps("k_stop")

    a.label("k_puts")
    a.push(AX)
    a.push(BX)
    a.push(SI)
    a.label("kp_loop")
    a.lodsb()
    a.cmp(AL, 0)
    a.jz("kp_end")
    a.mov(AH, 0x0E)
    a.mov(BL, 0x0A)
    a.int_(0x10)
    a.jmps("kp_loop")
    a.label("kp_end")
    a.pop(SI)
    a.pop(BX)
    a.pop(AX)
    a.ret()

    a.label("k_msg")
    a.dz("KERNEL.BIN loaded from FAT12 and running.\r\n")
    a.label("k_bad")
    a.dz("Cluster chain broken: tail of KERNEL.BIN is wrong.\r\n")

    a.resolve()

    if a.pos > KERNEL_TAIL:
        raise SystemExit("kernel code overruns its padding")

    img = bytearray(KERNEL_BYTES)
    img[0:a.pos] = a.buf[0:a.pos]
    # Fill the gap with a per-sector byte so a mis-ordered chain corrupts
    # something visible rather than landing on zeros either way.
    for i in range(a.pos, KERNEL_TAIL):
        img[i] = 0x41 + (i // 512)
    img[KERNEL_TAIL] = KERNEL_MAGIC & 0xFF
    img[KERNEL_TAIL + 1] = (KERNEL_MAGIC >> 8) & 0xFF
    return bytes(img)


# ===========================================================================
def main():
    fs = Fat12(total_sectors=SECTORS, sectors_per_track=SPT, heads=HEADS)

    boot = build_boot_code()
    fs.set_boot_code(boot)

    # Padding placed BEFORE the kernel so that loading the kernel has to cross
    # a head boundary and a cylinder boundary. Without it every sector the boot
    # chain touches lives on cylinder 0 head 0, where LBA-to-CHS and back is the
    # identity and a mistake in either direction is invisible. The BIOS's INT
    # 13h read the caller's head out of DH after a word MUL had already
    # overwritten DX, so every head-1 read silently returned a head-0 sector --
    # and the entire test suite passed, because nothing ever asked for head 1.
    pad_start = fs.data_start
    want_start = 2 * SPT - 2        # kernel spans ... SPT-1 | SPT ... 2*SPT ...
    pad_sectors = want_start - pad_start
    if pad_sectors > 0:
        fs.add_file("PADDING.BIN", bytes(pad_sectors * 512))

    kernel = build_kernel()
    kernel_cluster = fs.add_file(KERNEL_NAME, kernel)
    kernel_lba = fs.data_start + (kernel_cluster - 2) * fs.sectors_per_cluster
    if kernel_lba + KERNEL_SECTORS <= SPT:
        raise SystemExit("KERNEL.BIN sits entirely on head 0 (LBA %d..%d); the "
                         "boot chain would never exercise a head boundary"
                         % (kernel_lba, kernel_lba + KERNEL_SECTORS - 1))

    # A second file, so the directory search has to actually search rather than
    # matching the first entry it looks at.
    fs.add_file("README.TXT",
                b"FPGA80186 FAT12 volume.\r\n"
                b"KERNEL.BIN is loaded by the boot sector.\r\n")

    img = fs.image()

    os.makedirs(os.path.dirname(OUT) or ".", exist_ok=True)
    with open(OUT, "w") as f:
        for i in range(0, len(img), 2):
            f.write("%04x\n" % (img[i] | (img[i + 1] << 8)))

    print(fs.describe())
    print("boot code %d bytes (%d free), %s %d bytes"
          % (len(boot), 512 - 0x3E - 2 - len(boot), KERNEL_NAME, len(kernel)))
    # The geometry has to be visible to the testbenches too, or they end up
    # carrying a third copy of it that drifts from the other two. One generated
    # header, read by the simulation, keeps there being exactly one answer.
    svh = os.path.join(os.path.dirname(OUT) or ".", "geometry.svh")
    with open(svh, "w") as f:
        f.write("// Written by tools/gen_disk.py -- do not edit.\n"
                "// The disk geometry the BIOS's INT 13h and the boot sector's\n"
                "// BPB both use. Testbenches include this so there is one\n"
                "// source of truth rather than a constant per file.\n"
                "localparam int GEO_SPT     = %d;\n"
                "localparam int GEO_HEADS   = %d;\n"
                "localparam int GEO_SECTORS = %d;\n"
                "// What INT 13h AH=08 reports: the geometry of the disk the\n"
                "// BIOS was built for, which may be larger than this image.\n"
                "localparam int GEO_BIOS_CYLS = %d;\n"
                % (SPT, HEADS, SECTORS,
                   max(1, BIOS_SECTORS // (SPT * HEADS))))
    print("wrote %s" % svh)

    print("wrote %s: %d sectors (%d KB)"
          % (OUT, SECTORS, SECTORS * 512 // 1024))
    print("%s at LBA %d..%d -- heads %s"
          % (KERNEL_NAME, kernel_lba, kernel_lba + KERNEL_SECTORS - 1,
             sorted({(l // SPT) % HEADS
                     for l in range(kernel_lba, kernel_lba + KERNEL_SECTORS)})))


if __name__ == "__main__":
    main()
