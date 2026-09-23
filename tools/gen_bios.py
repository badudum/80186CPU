#!/usr/bin/env python3
"""
Build the FPGA80186 boot ROM: a BIOS with the interrupt services DOS expects.

The ROM is 16 KB, aliased through F0000-FFFFF, so the reset vector at physical
FFFF0 lands at image offset 3FF0 and the code runs at CS=F000 with IP equal to
the image offset.

SERVICES
    INT 08h  timer tick             INT 13h  disk
    INT 09h  keyboard (via type 12) INT 16h  keyboard services
    INT 10h  video                  INT 1Ah  time of day
    INT 11h  equipment              INT 19h  bootstrap
    INT 12h  memory size

TWO PLACES THIS DEVIATES FROM A PC, both forced by the hardware:

  * The 80186's own interrupt controller has FIXED vectors. Timer 0 arrives as
    type 8, which is already the PC's tick vector, so that one lines up. The
    keyboard arrives on INT0 as type 12, not type 9. Software hooks INT 09h, so
    the type-12 handler is a shim that issues `INT 09h` and then signals
    end-of-interrupt -- anything hooking 09h still sees every keystroke.

  * PS/2 keyboards send scancode set 2; PC software expects set 1. Translation
    happens here, in the type-9 handler, which turns set 2 into ASCII and
    stores it in the standard BIOS keyboard buffer at 40:1E.

Usage:  python3 tools/gen_bios.py [outdir]
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from asm86 import (Asm, mem,
                   AX, CX, DX, BX, SP, BP, SI, DI,
                   AL, CL, DL, BL, AH, CH, DH, BH,
                   ES, CS, SS, DS)

ROM_SIZE = 16384
RESET_AT = ROM_SIZE - 16          # 3FF0

# ---- BIOS data area, segment 0040 -----------------------------------------
BDA = 0x40
B_EQUIP = 0x10        # word   equipment list
B_MEMKB = 0x13        # word   conventional memory, KB
B_SHIFT = 0x17        # byte   shift/control state
B_KHEAD = 0x1A        # word   keyboard buffer head
B_KTAIL = 0x1C        # word   keyboard buffer tail
B_KBUF = 0x1E         # 16 entries x 2 bytes, 1E..3D
B_KBUF_END = 0x3E
B_MODE = 0x49         # byte   video mode
B_COLS = 0x4A         # word   columns
B_CURSOR = 0x50       # word   low = column, high = row
B_PAGE = 0x62         # byte   active page
B_TICKS = 0x6C        # dword  timer ticks since midnight
# Private scratch, above everything the PC BIOS defines.
B_MODEHW = 0x80       # byte   port 3D8 read straight back after a mode set
B_MODEREQ = 0x81      # byte   the AL of the most recent INT 10h AH=00
B_MODECNT = 0x82      # byte   how many AH=00 calls have been made
B_FAULT = 0x90        # 42 bytes: the register state at the last trap
B_PROFI = 0xBC        # byte   next slot in the profile ring
B_PROF  = 0xC0        # PROF_N entries of CS:IP, sampled by the timer tick
PROF_N  = 12
B_BREAK = 0xF0        # byte   a break (F0) prefix is pending
B_EXTEND = 0xF1       # byte   an extended (E0) prefix is pending

VIDEO_SEG = 0xB800
GFX_SEG   = 0xA000
SCREEN_COLS = 80
SCREEN_ROWS = 25

# ---- ports ----------------------------------------------------------------
P_KBD_DATA = 0x0060
P_DAC_IDX  = 0x03C8
P_DAC_DATA = 0x03C9
P_MODE     = 0x03D8       # CGA-style mode control; bit 1 selects graphics
P_CRTC_IDX = 0x03D4
P_CRTC_DAT = 0x03D5
P_STOR_DATA = 0x0320
P_STOR_LBALO = 0x0322
P_STOR_LBAHI = 0x0324
P_STOR_CMD = 0x0326

# Peripheral control block, default base FF00.
P_PCB = 0xFF00
P_EOI = P_PCB + 0x22
P_TMR_MASK = P_PCB + 0x32
P_INT0_MASK = P_PCB + 0x38
P_T0_CNT = P_PCB + 0x50
P_T0_MAXA = P_PCB + 0x52
P_T0_CTL = P_PCB + 0x56
P_T2_CNT = P_PCB + 0x60
P_T2_MAXA = P_PCB + 0x62
P_T2_CTL = P_PCB + 0x66

# Timer 0 counts timer 2 timeouts; timer 2 counts the CPU clock / 4.
# 25 MHz / 4 = 6.25 MHz, / 64 = 97656 Hz, / 5366 = 18.199 Hz -- the PC rate.
#
# THE DIVISOR IS TIED TO THE CPU CLOCK, so raising the clock without raising
# this makes the BIOS tick fast and every DOS program that measures time --
# which is every game -- run at the wrong speed, with nothing on screen to say
# why. `--clk-hz` exists so the rate lives in one place; FPGA80186.sv's CLK_HZ
# is the same number on the hardware side and the two must agree.
CLK_HZ = 25_000_000
_argv = []
_skip = False
for _i, _a in enumerate(sys.argv):
    if _skip:
        _skip = False
        continue
    if _a == "--clk-hz":
        CLK_HZ = int(sys.argv[_i + 1])
        _skip = True                 # the value is not the output directory
    elif _a.startswith("--clk-hz="):
        CLK_HZ = int(_a.split("=", 1)[1])
    else:
        _argv.append(_a)
sys.argv = _argv

T2_DIVISOR = 64
# 18.2065 Hz is the PC's tick. Round rather than truncate: at 40 MHz the exact
# quotient is 8584.6, and the floor is half a tick per second slow.
if CLK_HZ == 25_000_000:
    # Pinned to the value the working 25 MHz ROM has always used. The formula
    # below gives 5364, which is marginally MORE accurate, but changing a ROM
    # that boots DOS today for a 0.04% tick correction means the fallback
    # build is no longer the one known to work. Not worth it.
    T0_DIVISOR = 5366
else:
    T0_DIVISOR = int(round(CLK_HZ / 4.0 / T2_DIVISOR / 18.2065))
if CLK_HZ != 25_000_000:
    print("clock %0.3f MHz: T0 divisor %d (%0.4f Hz tick)"
          % (CLK_HZ / 1e6, T0_DIVISOR,
             CLK_HZ / 4.0 / T2_DIVISOR / T0_DIVISOR))

# `--fast-tick` divides the tick period by 64, which is only ever used to build
# a ROM for sim/tb_msdos.sv. MS-DOS times its startup waits in BIOS ticks -- the
# F5/F8 prompt alone is two seconds, fifty million clocks, most of an hour of
# simulation for a wait that does nothing. Ticking faster costs nothing but a
# wrong wall clock, and turns "wait, then carry on" into something a simulation
# can actually reach. It must never go into a bitstream, which is why it is a
# flag rather than the default.
FAST_TICK = "--fast-tick" in sys.argv
if FAST_TICK:
    T0_DIVISOR = max(1, T0_DIVISOR // 64)
    print("FAST TICK: T0 divisor %d -- simulation only, do NOT synthesise this"
          % T0_DIVISOR)

# Disk geometry. The device is a flat array of 512-byte sectors; INT 13h speaks
# CHS, so one has to be mapped onto the other.
#
# These MUST match the disk image. A boot sector computes CHS from the geometry
# its own BPB declares, and INT 13h here converts it back; if the two disagree
# every read lands on a different sector than the one asked for, and the
# symptom looks like a corrupt filesystem rather than a mismatched constant.
# tools/img2hex.py writes rom/geometry.py from an uploaded image's BPB so that
# cannot happen by accident; 16 x 4 is the default for the image this project
# generates itself.
SPT = 16
HEADS = 4
SECTORS = 256

try:
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    "..", "rom"))
    from geometry import SPT as _SPT, HEADS as _HEADS, SECTORS as _SECTORS  # noqa: E402
    SPT, HEADS, SECTORS = _SPT, _HEADS, _SECTORS
    print("geometry from rom/geometry.py: SPT=%d HEADS=%d SECTORS=%d"
          % (SPT, HEADS, SECTORS))
except ImportError:
    pass

# INT 13h AH=08 reports the LAST cylinder, not the count.
CYLS = max(1, SECTORS // (SPT * HEADS))

a = Asm(ROM_SIZE)


def jcc_far(cc, label):
    """A conditional branch beyond rel8 range.

    On the 8086 and 80186, Jcc is ONLY ever an 8-bit displacement -- the near
    form is a 386 addition. Anything further away has to invert the condition
    and jump over an unconditional near jump. Forgetting this is the classic
    way to write 16-bit code that assembles on a modern tool and cannot run on
    the CPU it was written for.
    """
    inverse = {"z": "nz", "nz": "z", "c": "nc", "nc": "c",
               "a": "be", "be": "a", "l": "ge", "ge": "l"}
    skip = "_over_%04X" % a.here()
    a.jcc(inverse[cc], skip)
    a.jmp(label)
    a.label(skip)


def set_ds_bda():
    """Point DS at the BIOS data area WITHOUT disturbing AX.

    The obvious two instructions destroy AH, and AH is the function code every
    one of these services dispatches on. Written that way, INT 10h read its own
    selector as 00 -- set video mode -- so `puts` cleared the whole screen once
    per character and the machine never got past its banner. Three extra bytes
    is a cheap price for that not being possible.
    """
    a.push(AX)
    a.mov(AX, BDA)
    a.mov(DS, AX)
    a.pop(AX)


# ===========================================================================
# Cold start
# ===========================================================================
a.org(0x0100)
a.label("bios_entry")
a.cli()
a.cld()

# A stack immediately below where the boot sector will be loaded.
a.mov(AX, 0x0000)
a.mov(SS, AX)
a.mov(SP, 0x7C00)

# ---- clear the BIOS data area ----
a.mov(AX, BDA)
a.mov(ES, AX)
a.mov(DI, 0x0000)
a.mov(CX, 0x80)               # 128 words = 256 bytes
a.mov(AX, 0x0000)
a.rep()
a.stosw()

set_ds_bda()
a.mov(AX, 0x0021)             # 80x25 colour, no floppies reported
a.mov(mem(disp=B_EQUIP), AX)
a.mov(AX, 640)
a.mov(mem(disp=B_MEMKB), AX)
a.mov(AX, SCREEN_COLS)
a.mov(mem(disp=B_COLS), AX)
a.mov(AX, B_KBUF)
a.mov(mem(disp=B_KHEAD), AX)
a.mov(mem(disp=B_KTAIL), AX)

# ---- interrupt vector table ----
# Every vector gets the do-nothing handler first, so an unexpected interrupt
# returns instead of running through whatever happens to be in RAM.
a.mov(AX, 0x0000)
a.mov(ES, AX)
a.mov(DI, 0x0000)
a.mov(CX, 256)
a.label("ivt_fill")
a.mov_label(AX, "int_ignore")
a.stosw()
a.mov(AX, 0xF000)
a.stosw()
a.loop("ivt_fill")


def set_vec(num, label):
    a.mov_label(AX, label)
    a.mov(mem(disp=num * 4, seg=ES), AX)
    a.mov(AX, 0xF000)
    a.mov(mem(disp=num * 4 + 2, seg=ES), AX)


set_vec(0x00, "int_divzero")
set_vec(0x06, "int_illegal")
set_vec(0x08, "int08_tick")
set_vec(0x09, "int09_kbd")
set_vec(0x0C, "int0c_kbd_hw")     # the 80186 delivers INT0 as type 12
set_vec(0x10, "int10_video")
set_vec(0x11, "int11_equip")
set_vec(0x12, "int12_memsize")
set_vec(0x13, "int13_disk")
set_vec(0x16, "int16_kbd")
set_vec(0x19, "int19_boot")
set_vec(0x1A, "int1a_time")

# ---- video ----
a.mov(AX, 0x0003)             # AH=00 set mode 3: clears the screen
a.int_(0x10)

# ---- timer: timer 2 prescales timer 0 down to the 18.2 Hz tick ----
a.mov(DX, P_T2_MAXA)
a.mov(AX, T2_DIVISOR)
a.out_dx(AX)
a.mov(DX, P_T2_CNT)
a.mov(AX, 0)
a.out_dx(AX)
a.mov(DX, P_T2_CTL)
a.mov(AX, 0xC001)             # EN | INH | CONT
a.out_dx(AX)

a.mov(DX, P_T0_MAXA)
a.mov(AX, T0_DIVISOR)
a.out_dx(AX)
a.mov(DX, P_T0_CNT)
a.mov(AX, 0)
a.out_dx(AX)
a.mov(DX, P_T0_CTL)
a.mov(AX, 0xE009)             # EN | INH | INT | prescale from timer 2 | CONT
a.out_dx(AX)

# ---- unmask the timer and the keyboard, both at priority 0 ----
a.mov(DX, P_TMR_MASK)
a.mov(AX, 0x0000)
a.out_dx(AX)
a.mov(DX, P_INT0_MASK)
a.mov(AX, 0x0000)
a.out_dx(AX)

a.sti()

a.mov_label(SI, "banner")
a.call("puts")


a.int_(0x19)                  # bootstrap; does not return
a.hlt()

# ===========================================================================
# INT 10h -- video
# ===========================================================================
a.label("int10_video")
a.push(DS)
a.push(ES)
a.pusha()
set_ds_bda()

a.cmp(AH, 0x0E)
a.jnz("i10_not_tty")
a.call("put_char")
a.jmp("i10_done")

a.label("i10_not_tty")
a.cmp(AH, 0x00)
a.jnz("i10_not_mode")
# Record what was asked for, and that it was asked at all. A guest whose
# picture never appears might be requesting a mode this BIOS does not
# implement, or might not be going through the BIOS at all -- and those two
# need completely different fixes.
a.mov(mem(disp=B_MODEREQ), AL)
a.push(AX)
a.mov(AL, mem(disp=B_MODECNT))
a.inc(AL)
a.mov(mem(disp=B_MODECNT), AL)
a.pop(AX)
# AL is the mode number. 13h is 320x200 in 256 colours; anything else is
# treated as the text mode, which is what a BIOS with one of each should do
# rather than failing on a mode it does not have.
a.cmp(AL, 0x13)
a.jnz("i10_mode_text")

a.mov(mem(disp=B_MODE), AL)       # remember 13h before AL is reused
a.mov(DX, P_MODE)
a.mov(AL, 0x02)                   # bit 1: graphics
a.out_dx(AL)
a.in_dx(AL)                       # ...and read it back, see B_MODEHW
a.mov(mem(disp=B_MODEHW), AL)
a.call("clear_gfx")
a.mov(AX, 0x0000)
a.mov(mem(disp=B_CURSOR), AX)
a.jmp("i10_done")

# B_MODEHW records what port 3D8 reads back immediately after the write. The
# BDA's mode byte is only the BIOS's opinion; this is what the video hardware
# is actually doing, and the two disagreeing is precisely the failure worth
# being able to see.
a.label("i10_mode_text")
a.mov(DX, P_MODE)
a.mov(AL, 0x00)
a.out_dx(AL)
a.in_dx(AL)
a.mov(mem(disp=B_MODEHW), AL)
a.call("clear_screen")
a.mov(AX, 0x0000)
a.mov(mem(disp=B_CURSOR), AX)
a.mov(AL, 0x03)
a.mov(mem(disp=B_MODE), AL)
a.call("sync_cursor")
a.jmp("i10_done")

a.label("i10_not_mode")
a.cmp(AH, 0x02)
a.jnz("i10_not_setcur")
# DH = row, DL = column. Stored as one word: low byte column, high byte row.
a.mov(mem(disp=B_CURSOR), DX)
a.call("sync_cursor")
a.jmp("i10_done")

a.label("i10_not_setcur")
a.cmp(AH, 0x03)
a.jnz("i10_not_getcur")
a.mov(AX, mem(disp=B_CURSOR))
# The caller's DX and CX live in the pusha frame; write them there so popa
# hands them back. pusha pushes AX CX DX BX SP BP SI DI in that order, so from
# SP the slots are DI SI BP SP BX DX CX AX at +0 +2 +4 +6 +8 +10 +12 +14.
a.mov(BP, SP)
a.mov(mem(BP, disp=10), AX)       # DX slot
a.mov(AX, 0x0607)                 # cursor scan lines, cosmetic
a.mov(mem(BP, disp=12), AX)       # CX slot
a.jmp("i10_done")

a.label("i10_not_getcur")
a.cmp(AH, 0x06)
a.jnz("i10_not_scroll")
a.call("scroll_up")
a.jmp("i10_done")

a.label("i10_not_scroll")
a.cmp(AH, 0x09)
a.jnz("i10_not_wrchar")
# AL = character, BL = attribute, CX = repeat count, written at the cursor
# without moving it.
a.call("cell_offset")
a.mov(DI, AX)
a.mov(AX, VIDEO_SEG)
a.mov(ES, AX)
a.mov(BP, SP)
a.mov(AX, mem(BP, disp=14))       # caller's AX
a.mov(BX, mem(BP, disp=8))        # caller's BX
a.mov(AH, BL)
a.mov(CX, mem(BP, disp=12))       # caller's CX
a.cmp(CX, 0)
a.jnz("i10_wr_go")
a.mov(CX, 1)
a.label("i10_wr_go")
a.rep()
a.stosw()
a.jmp("i10_done")

a.label("i10_not_wrchar")
a.cmp(AH, 0x0F)
a.jnz("i10_done")
a.mov(BP, SP)
a.mov(AL, mem(disp=B_MODE))
a.mov(AH, SCREEN_COLS)
a.mov(mem(BP, disp=14), AX)       # AX slot: AL = mode, AH = columns
a.mov(AX, 0x0000)
a.mov(mem(BP, disp=8), AX)        # BX slot: BH = page 0

a.label("i10_done")
a.popa()
a.pop(ES)
a.pop(DS)
a.iret()

# ---------------------------------------------------------------------------
# put_char -- the teletype core. AL = character, BL = attribute. DS = 40.
# ---------------------------------------------------------------------------
a.label("put_char")
a.push(AX)
a.push(BX)
a.push(CX)
a.push(DI)
a.push(ES)

a.cmp(AL, 13)
a.jnz("pc_not_cr")
a.mov(AL, 0)
a.mov(mem(disp=B_CURSOR), AL)
a.jmp("pc_done")

a.label("pc_not_cr")
a.cmp(AL, 10)
a.jnz("pc_not_lf")
a.jmp("pc_newline")

a.label("pc_not_lf")
a.cmp(AL, 8)
a.jnz("pc_normal")
a.mov(AL, mem(disp=B_CURSOR))
a.cmp(AL, 0)
a.jz("pc_done")
a.dec(AL)
a.mov(mem(disp=B_CURSOR), AL)
a.jmp("pc_done")

a.label("pc_normal")
a.push(AX)
a.call("cell_offset")
a.mov(DI, AX)
a.mov(AX, VIDEO_SEG)
a.mov(ES, AX)
a.pop(AX)
a.mov(AH, BL)
a.stosw()
# advance the column, wrapping to the next row
a.mov(AL, mem(disp=B_CURSOR))
a.inc(AL)
a.mov(mem(disp=B_CURSOR), AL)
a.cmp(AL, SCREEN_COLS)
a.jc("pc_done")
a.mov(AL, 0)
a.mov(mem(disp=B_CURSOR), AL)

a.label("pc_newline")
a.mov(AL, mem(disp=B_CURSOR + 1))
a.inc(AL)
a.mov(mem(disp=B_CURSOR + 1), AL)
a.cmp(AL, SCREEN_ROWS)
a.jc("pc_done")
a.call("scroll_up")
a.mov(AL, SCREEN_ROWS - 1)
a.mov(mem(disp=B_CURSOR + 1), AL)

a.label("pc_done")
a.call("sync_cursor")
a.pop(ES)
a.pop(DI)
a.pop(CX)
a.pop(BX)
a.pop(AX)
a.ret()

# ---------------------------------------------------------------------------
# cell_offset -- AX = byte offset of the cursor cell. DS = 40.
# ---------------------------------------------------------------------------
a.label("cell_offset")
a.push(BX)
a.mov(AL, mem(disp=B_CURSOR + 1))     # row
a.mov(BL, SCREEN_COLS)
a.mul(BL)                             # AX = row * 80
a.mov(BL, mem(disp=B_CURSOR))         # column
a.mov(BH, 0)
a.add(AX, BX)
a.shl(AX, 1)                          # two bytes per cell
a.pop(BX)
a.ret()

# ---------------------------------------------------------------------------
# sync_cursor -- push the BDA cursor into the CRTC. DS = 40.
# ---------------------------------------------------------------------------
a.label("sync_cursor")
a.push(AX)
a.push(BX)
a.push(DX)
a.call("cell_offset")
a.shr(AX, 1)                          # cells, not bytes
a.mov(BX, AX)
a.mov(DX, P_CRTC_IDX)
a.mov(AL, 0x0E)
a.out_dx(AL)
a.mov(DX, P_CRTC_DAT)
a.mov(AL, BH)
a.out_dx(AL)
a.mov(DX, P_CRTC_IDX)
a.mov(AL, 0x0F)
a.out_dx(AL)
a.mov(DX, P_CRTC_DAT)
a.mov(AL, BL)
a.out_dx(AL)
a.pop(DX)
a.pop(BX)
a.pop(AX)
a.ret()

# ---------------------------------------------------------------------------
# clear_screen / scroll_up
# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# clear_gfx -- fill the 64,000-byte graphics aperture with index 0.
#
# A mode switch that leaves the previous contents on screen shows whatever the
# last program drew, which looks like a crash. STOSW covers it in 32,000 word
# writes rather than 64,000 byte ones.
# ---------------------------------------------------------------------------
a.label("clear_gfx")
a.push(AX)
a.push(CX)
a.push(DI)
a.push(ES)
a.mov(AX, GFX_SEG)
a.mov(ES, AX)
a.mov(DI, 0x0000)
a.mov(AX, 0x0000)
a.mov(CX, 32000)
a.cld()
a.rep(); a.stosw()
a.pop(ES)
a.pop(DI)
a.pop(CX)
a.pop(AX)
a.ret()

a.label("clear_screen")
a.pusha()
a.push(ES)
a.mov(AX, VIDEO_SEG)
a.mov(ES, AX)
a.mov(DI, 0)
a.mov(CX, SCREEN_COLS * SCREEN_ROWS)
a.mov(AX, 0x0720)
a.cld()
a.rep()
a.stosw()
a.pop(ES)
a.popa()
a.ret()

a.label("scroll_up")
a.pusha()
a.push(DS)
a.push(ES)
a.mov(AX, VIDEO_SEG)
a.mov(DS, AX)
a.mov(ES, AX)
# Move rows 1..24 up one. The destination is below the source, so a forward
# copy is the correct direction for the overlap.
a.mov(SI, SCREEN_COLS * 2)
a.mov(DI, 0)
a.mov(CX, SCREEN_COLS * (SCREEN_ROWS - 1))
a.cld()
a.rep()
a.movsw()
# blank the row that scrolled in at the bottom
a.mov(DI, SCREEN_COLS * (SCREEN_ROWS - 1) * 2)
a.mov(CX, SCREEN_COLS)
a.mov(AX, 0x0720)
a.rep()
a.stosw()
a.pop(ES)
a.pop(DS)
a.popa()
a.ret()

# ---------------------------------------------------------------------------
# puts -- print the NUL-terminated string at F000:SI through INT 10h.
# ---------------------------------------------------------------------------
a.label("puts")
a.push(AX)
a.push(BX)
a.push(SI)
a.push(DS)
a.mov(AX, 0xF000)
a.mov(DS, AX)
a.label("puts_loop")
a.lodsb()
a.cmp(AL, 0)
a.jz("puts_end")
a.mov(AH, 0x0E)
a.mov(BL, 0x07)
a.int_(0x10)
a.jmps("puts_loop")
a.label("puts_end")
a.pop(DS)
a.pop(SI)
a.pop(BX)
a.pop(AX)
a.ret()

# ===========================================================================
# INT 11h / INT 12h -- equipment and memory size
# ===========================================================================
a.label("int11_equip")
a.push(DS)
set_ds_bda()
a.mov(AX, mem(disp=B_EQUIP))
a.pop(DS)
a.iret()

a.label("int12_memsize")
a.push(DS)
set_ds_bda()
a.mov(AX, mem(disp=B_MEMKB))
a.pop(DS)
a.iret()

# ===========================================================================
# INT 1Ah -- time of day
# ===========================================================================
a.label("int1a_time")
a.cmp(AH, 0x00)
a.jnz("i1a_out")
a.push(DS)
set_ds_bda()
a.mov(AX, mem(disp=B_TICKS))
a.mov(DX, AX)
a.mov(AX, mem(disp=B_TICKS + 2))
a.mov(CX, AX)
a.mov(AL, 0)                      # no midnight rollover tracking
a.pop(DS)
a.label("i1a_out")
a.iret()

# ===========================================================================
# INT 08h -- timer tick
# ===========================================================================
# The tick handler doubles as a SAMPLING PROFILER: every tick it records the
# CS:IP it interrupted into a ring at 0040:00C0. Eighteen samples a second is
# useless for tuning and perfect for the question this keeps raising -- where
# is the machine spending its time? -- because it runs at full speed on
# hardware, where simulation reaches five million cycles a minute and the board
# does twenty-five million a second. Read it with
#   quartus_stp -t tools/jtag_peek.tcl 0x0004C0 24
a.label("int08_tick")
a.push(AX)
a.push(DX)
a.push(DS)
a.push(BP)
a.push(BX)
a.mov(BP, SP)
set_ds_bda()

# [BP+0] BX  [+2] BP  [+4] DS  [+6] DX  [+8] AX  [+10] IP  [+12] CS
a.mov(BL, mem(disp=B_PROFI))
a.mov(BH, 0)
a.add(BX, BX)
a.add(BX, BX)                     # four bytes per entry
a.add(BX, B_PROF)
a.mov(AX, mem(BP, disp=10))
a.mov(mem(BX), AX)                # the interrupted IP
a.mov(AX, mem(BP, disp=12))
a.mov(mem(BX, disp=2), AX)        # ...and its CS
a.mov(AL, mem(disp=B_PROFI))
a.inc(AL)
a.cmp(AL, PROF_N)
a.jc("i08_prof_wrapped")
a.mov(AL, 0)
a.label("i08_prof_wrapped")
a.mov(mem(disp=B_PROFI), AL)

a.mov(AX, mem(disp=B_TICKS))
a.add(AX, 1)
a.mov(mem(disp=B_TICKS), AX)
a.jnz("i08_no_carry")
a.mov(AX, mem(disp=B_TICKS + 2))
a.add(AX, 1)
a.mov(mem(disp=B_TICKS + 2), AX)
a.label("i08_no_carry")
a.int_(0x1C)                      # the user tick hook
a.mov(DX, P_EOI)
a.mov(AX, 0x0000)
a.out_dx(AX)
a.pop(BX)
a.pop(BP)
a.pop(DS)
a.pop(DX)
a.pop(AX)
a.iret()

# ===========================================================================
# Type 12 -- where the 80186 actually delivers the keyboard.
# Dispatch through INT 09h so software that hooks it still runs, then EOI.
# ===========================================================================
a.label("int0c_kbd_hw")
a.push(AX)
a.push(DX)
a.int_(0x09)
a.mov(DX, P_EOI)
a.mov(AX, 0x0000)
a.out_dx(AX)
a.pop(DX)
a.pop(AX)
a.iret()

# ===========================================================================
# INT 09h -- keyboard: set 2 scancode in, ASCII in the BIOS buffer out
# ===========================================================================
a.label("int09_kbd")
a.pusha()
a.push(DS)
set_ds_bda()

a.mov(DX, P_KBD_DATA)
a.in_dx(AL)
a.mov(BL, AL)                     # keep the raw scancode

# THE BYTES HERE ARE SET 1, not set 2. keyboard_controller.sv translates on
# the way in, so a RELEASE is bit 7 of the scancode rather than a preceding F0
# -- which also means a release no longer costs an extra interrupt. E0 still
# introduces an extended key; set 1 uses that prefix too.
a.cmp(AL, 0xE0)
a.jnz("i09_not_ext")
a.mov(AL, 1)
a.mov(mem(disp=B_EXTEND), AL)
a.jmp("i09_leave")

a.label("i09_not_ext")
# Split the make code from the release bit. BL is the key from here on, and
# B_BREAK is non-zero if this was a release.
a.mov(AL, BL)
a.and_(AL, 0x80)
a.mov(mem(disp=B_BREAK), AL)
a.and_(BL, 0x7F)

# Shift keys maintain state on both press and release.
a.cmp(BL, 0x2A)                   # left shift
a.jz("i09_shift")
a.cmp(BL, 0x36)                   # right shift
a.jnz("i09_not_shift")

a.label("i09_shift")
a.mov(AL, mem(disp=B_BREAK))
a.cmp(AL, 0)
a.jnz("i09_shift_up")
a.mov(AL, mem(disp=B_SHIFT))
a.or_(AL, 0x03)
a.mov(mem(disp=B_SHIFT), AL)
a.jmp("i09_done")
a.label("i09_shift_up")
a.mov(AL, mem(disp=B_SHIFT))
a.and_(AL, 0xFC)
a.mov(mem(disp=B_SHIFT), AL)
a.jmp("i09_done")

a.label("i09_not_shift")
# A release of anything else is simply discarded.
a.mov(AL, mem(disp=B_BREAK))
a.cmp(AL, 0)
a.jnz("i09_done")

# Translate and enqueue.
a.mov(AL, BL)
a.call("translate")
a.cmp(AL, 0)
a.jz("i09_done")
a.mov(AH, BL)                     # AH = raw scancode, AL = ASCII
a.call("kbuf_put")

a.label("i09_done")
# Every path that consumed a key clears the prefixes, so a stray F0 or E0
# cannot arm them forever.
a.mov(AL, 0)
a.mov(mem(disp=B_BREAK), AL)
a.mov(mem(disp=B_EXTEND), AL)

a.label("i09_leave")
a.pop(DS)
a.popa()
a.iret()

# ---------------------------------------------------------------------------
# translate -- AL = set 2 scancode -> AL = ASCII (0 if the key has none).
# ---------------------------------------------------------------------------
a.label("translate")
a.push(BX)
a.push(CX)
a.push(DS)
a.mov(CL, mem(disp=B_SHIFT))
a.mov(BX, 0xF000)
a.mov(DS, BX)
a.test(CL, 0x03)
a.jz("xl_lower")
a.mov_label(BX, "kbd_upper")
a.jmps("xl_go")
a.label("xl_lower")
a.mov_label(BX, "kbd_lower")
a.label("xl_go")
a.and_(AL, 0x7F)
a.xlat()
a.pop(DS)
a.pop(CX)
a.pop(BX)
a.ret()

# ---------------------------------------------------------------------------
# kbuf_put -- push AX (AH scancode, AL ASCII) into the BIOS keyboard buffer.
# A full buffer drops the key, which is what a PC does. DS = 40.
# ---------------------------------------------------------------------------
a.label("kbuf_put")
a.push(BX)
a.push(CX)
a.mov(BX, mem(disp=B_KTAIL))
a.mov(CX, BX)
a.add(CX, 2)
a.cmp(CX, B_KBUF_END)
a.jc("kbp_no_wrap")
a.mov(CX, B_KBUF)
a.label("kbp_no_wrap")
a.cmp(CX, mem(disp=B_KHEAD))
a.jz("kbp_full")
a.mov(mem(BX), AX)
a.mov(mem(disp=B_KTAIL), CX)
a.label("kbp_full")
a.pop(CX)
a.pop(BX)
a.ret()

# ===========================================================================
# INT 16h -- keyboard services
# ===========================================================================
a.label("int16_kbd")
a.cmp(AH, 0x00)
a.jnz("i16_not_read")
a.jmp("i16_read")

a.label("i16_not_read")
a.cmp(AH, 0x01)
a.jnz("i16_not_peek")
a.jmp("i16_peek")

a.label("i16_not_peek")
a.cmp(AH, 0x02)
a.jnz("i16_out")
a.push(DS)
set_ds_bda()
a.mov(AL, mem(disp=B_SHIFT))
a.pop(DS)
a.label("i16_out")
a.iret()

# ---- AH=00: block until a key is available ----
a.label("i16_read")
a.push(DS)
a.push(BX)
set_ds_bda()
a.label("i16_wait")
a.mov(BX, mem(disp=B_KHEAD))
a.cmp(BX, mem(disp=B_KTAIL))
a.jnz("i16_have")
# Nothing yet. Waiting with interrupts enabled and the CPU halted is precisely
# why HLT has to wake on an interrupt; a spin loop would work too but would
# burn bus cycles fighting the very handler it is waiting for.
a.sti()
a.hlt()
a.jmps("i16_wait")

a.label("i16_have")
a.mov(AX, mem(BX))
a.add(BX, 2)
a.cmp(BX, B_KBUF_END)
a.jc("i16_no_wrap")
a.mov(BX, B_KBUF)
a.label("i16_no_wrap")
a.mov(mem(disp=B_KHEAD), BX)
a.pop(BX)
a.pop(DS)
a.iret()

# ---- AH=01: report without consuming; ZF set means the buffer is empty ----
a.label("i16_peek")
a.push(BP)
a.mov(BP, SP)
a.push(DS)
a.push(BX)
set_ds_bda()
a.mov(BX, mem(disp=B_KHEAD))
a.cmp(BX, mem(disp=B_KTAIL))
a.jz("i16_peek_empty")
a.mov(AX, mem(BX))
# Clear ZF in the FLAGS image IRET will restore. [BP+0] is the saved BP, so
# the interrupt frame starts at [BP+2]: IP, CS, FLAGS.
a.mov(BX, mem(BP, disp=6))
a.and_(BX, 0xFFBF)
a.mov(mem(BP, disp=6), BX)
a.jmps("i16_peek_out")

a.label("i16_peek_empty")
a.mov(BX, mem(BP, disp=6))
a.or_(BX, 0x0040)
a.mov(mem(BP, disp=6), BX)

a.label("i16_peek_out")
a.pop(BX)
a.pop(DS)
a.pop(BP)
a.iret()

# ===========================================================================
# INT 13h -- disk
# ===========================================================================
a.label("int13_disk")
a.cmp(AH, 0x00)
a.jnz("i13_not_reset")
a.jmp("i13_ok")

a.label("i13_not_reset")
a.cmp(AH, 0x02)
a.jnz("i13_not_read")
a.jmp("i13_read")

a.label("i13_not_read")
a.cmp(AH, 0x03)
a.jnz("i13_not_write")
a.jmp("i13_write")

a.label("i13_not_write")
a.cmp(AH, 0x08)
a.jnz("i13_not_params")
# Geometry in the shape INT 13h reports it: CH = last cylinder, CL bits 5:0 =
# sectors per track, DH = last head, DL = number of drives. Reporting the last
# cylinder as zero describes a disk one track long, which is true of no disk
# this will ever be given.
a.mov(CH, (CYLS - 1) & 0xFF)
a.mov(CL, SPT)
a.mov(DH, HEADS - 1)
a.mov(DL, 1)
a.mov(BL, 0x04)
a.jmp("i13_ok")

a.label("i13_not_params")
# Anything else fails cleanly rather than pretending to work.
a.mov(AH, 0x01)
a.jmp("i13_fail")

# BX is SAVED here, not just used as scratch. It is an INPUT to INT 13h -- the
# offset half of the ES:BX buffer -- and callers expect it back. Reaching for
# it to edit the stacked FLAGS returned the flags word in BX instead of the
# buffer pointer, and the MS-DOS boot sector's next instruction is `mov di,bx`,
# so its directory compare ran against a garbage address and it declared a
# perfectly good disk to have no system files on it. With BX pushed the frame
# moves by two: [BP+0] BX, [BP+2] BP, [BP+4] IP, [BP+6] CS, [BP+8] FLAGS.
a.label("i13_ok")
a.mov(AH, 0x00)
a.push(BP)
a.push(BX)
a.mov(BP, SP)
a.mov(BX, mem(BP, disp=8))
a.and_(BX, 0xFFFE)                # CF = 0
a.mov(mem(BP, disp=8), BX)
a.pop(BX)
a.pop(BP)
a.iret()

a.label("i13_fail")
a.push(BP)
a.push(BX)
a.mov(BP, SP)
a.mov(BX, mem(BP, disp=8))
a.or_(BX, 0x0001)                 # CF = 1
a.mov(mem(BP, disp=8), BX)
a.pop(BX)
a.pop(BP)
a.iret()

# ---- AH=03: write AL sectors from ES:BX to CHS ----
# The CHS-to-LBA arithmetic is identical to AH=02's, including taking the head
# out of DH before any MUL runs -- see the note there, that ordering was a real
# bug and it would be the same bug here.
a.label("i13_write")
a.push(DI)
a.push(SI)
a.push(CX)
a.push(DX)
a.push(BX)

a.push(AX)
a.mov(BL, DH)                     # head, captured before MUL can clobber DX
a.mov(BH, 0)
a.mov(DI, BX)
a.mov(AL, CH)
a.mov(AH, 0)
a.mov(BX, HEADS)
a.mul(BX)
a.add(AX, DI)
a.mov(BX, SPT)
a.mul(BX)
a.mov(BL, CL)
a.and_(BL, 0x3F)
a.mov(BH, 0)
a.sub(BX, 1)
a.add(AX, BX)
a.mov(SI, AX)                     # SI = LBA
a.pop(AX)

a.mov(CL, AL)                     # CL = sectors still to write
a.mov(CH, 0)
a.pop(BX)
a.push(BX)
a.mov(DI, BX)                     # ES:DI = source

a.label("i13_wnext")
a.cmp(CL, 0)
a.jz("i13_write_done")
a.call("write_sector")
a.jc("i13_write_err")
a.inc(SI)
a.dec(CL)
a.jmps("i13_wnext")

a.label("i13_write_done")
a.pop(BX)
a.pop(DX)
a.pop(CX)
a.pop(SI)
a.pop(DI)
a.mov(AH, 0x00)
a.jmp("i13_ok")

a.label("i13_write_err")
a.pop(BX)
a.pop(DX)
a.pop(CX)
a.pop(SI)
a.pop(DI)
a.mov(AH, 0x03)                   # write protected / write fault
a.jmp("i13_fail")

# ---- AH=02: read AL sectors from CHS into ES:BX ----
a.label("i13_read")
a.push(DI)
a.push(SI)
a.push(CX)
a.push(DX)
a.push(BX)

# LBA = (cylinder * HEADS + head) * SPT + (sector - 1)
#
# The head is taken out of DH FIRST, before anything else runs, because a word
# MUL writes its result to DX:AX and so destroys DH. Reading the caller's head
# after the cylinder multiply returns the high half of that product instead --
# which is zero for every cylinder 0 access, so every read on head 1 silently
# came back as head 0. That is one whole track of wrong data with no error
# reported: booting MS-DOS read the FAT where the root directory should be and
# concluded the disk had no system files on it.
a.push(AX)
a.mov(BL, DH)                     # head, captured before MUL can clobber DX
a.mov(BH, 0)
a.mov(DI, BX)
a.mov(AL, CH)
a.mov(AH, 0)
a.mov(BX, HEADS)
a.mul(BX)
a.add(AX, DI)
a.mov(BX, SPT)
a.mul(BX)
a.mov(BL, CL)
a.and_(BL, 0x3F)
a.mov(BH, 0)
a.sub(BX, 1)
a.add(AX, BX)
a.mov(SI, AX)                     # SI = LBA
a.pop(AX)

a.mov(CL, AL)                     # CL = sectors still to read
a.mov(CH, 0)
a.pop(BX)
a.push(BX)
a.mov(DI, BX)                     # ES:DI = destination

a.label("i13_next")
a.cmp(CL, 0)
a.jz("i13_read_done")
a.call("read_sector")
a.jc("i13_read_err")
a.inc(SI)
a.dec(CL)
a.jmps("i13_next")

a.label("i13_read_done")
a.pop(BX)
a.pop(DX)
a.pop(CX)
a.pop(SI)
a.pop(DI)
a.mov(AH, 0x00)
a.jmp("i13_ok")

a.label("i13_read_err")
a.pop(BX)
a.pop(DX)
a.pop(CX)
a.pop(SI)
a.pop(DI)
a.mov(AH, 0x04)                   # sector not found
a.jmp("i13_fail")

# ---------------------------------------------------------------------------
# read_sector -- SI = LBA, ES:DI = destination. CF set on failure.
# ---------------------------------------------------------------------------
a.label("read_sector")
a.push(AX)
a.push(CX)
a.push(DX)

a.mov(DX, P_STOR_LBALO)
a.mov(AX, SI)
a.out_dx(AX)
a.mov(DX, P_STOR_LBAHI)
a.mov(AX, 0)
a.out_dx(AX)
a.mov(DX, P_STOR_CMD)
a.mov(AX, 1)
a.out_dx(AX)

a.label("rs_wait")
a.mov(DX, P_STOR_CMD)
a.in_dx(AX)
a.test(AL, 0x01)                  # BUSY
a.jnz("rs_wait")
a.test(AL, 0x04)                  # ERR
a.jnz("rs_fail")

a.mov(CX, 256)
a.mov(DX, P_STOR_DATA)
a.cld()
a.label("rs_xfer")
a.in_dx(AX)
a.stosw()
a.loop("rs_xfer")

a.clc()
a.jmps("rs_out")
a.label("rs_fail")
a.stc()
a.label("rs_out")
a.pop(DX)
a.pop(CX)
a.pop(AX)
a.ret()

# ---------------------------------------------------------------------------
# write_sector -- SI = LBA, ES:DI = source. Advances DI, CF set on error.
#
# THE ORDER IS THE OPPOSITE OF A READ and it is not interchangeable: the LBA
# goes first, then the 256 words, then the command. The command is what copies
# the buffer out to memory, so data written after it would go nowhere; and
# writing the LBA is what resets the device's data index, so filling the
# buffer before setting the LBA would start part-way through the sector and
# store it rotated. See the header of modules/storage.sv.
#
# DS IS BORROWED FOR THE COPY. The source is ES:DI, and LODSW reads DS:SI, so
# DS is pointed at ES for the duration. Nothing between the two may touch a
# variable in the BIOS data area -- the segment is wrong until DS is restored.
# ---------------------------------------------------------------------------
a.label("write_sector")
a.push(AX)
a.push(CX)
a.push(DX)
a.push(SI)
a.push(DS)

a.mov(DX, P_STOR_LBALO)
a.mov(AX, SI)
a.out_dx(AX)
a.mov(DX, P_STOR_LBAHI)
a.mov(AX, 0)
a.out_dx(AX)

a.mov(AX, ES)
a.mov(DS, AX)                     # DS:SI = ES:DI for the duration
a.mov(SI, DI)
a.mov(CX, 256)
a.mov(DX, P_STOR_DATA)
a.cld()
a.label("ws_xfer")
a.lodsw()
a.out_dx(AX)
a.loop("ws_xfer")
a.mov(DI, SI)                     # leave DI past the sector, as STOSW would
a.pop(DS)

a.mov(DX, P_STOR_CMD)
a.mov(AX, 2)
a.out_dx(AX)

a.label("ws_wait")
a.mov(DX, P_STOR_CMD)
a.in_dx(AX)
a.test(AL, 0x01)                  # BUSY
a.jnz("ws_wait")
a.test(AL, 0x04)                  # ERR
a.jnz("ws_fail")

a.clc()
a.jmps("ws_out")
a.label("ws_fail")
a.stc()
a.label("ws_out")
a.pop(SI)
a.pop(DX)
a.pop(CX)
a.pop(AX)
a.ret()

# ===========================================================================
# INT 19h -- bootstrap
# ===========================================================================
a.label("int19_boot")
a.mov_label(SI, "msg_boot")
a.call("puts")

a.mov(AX, 0x0000)
a.mov(ES, AX)
a.mov(BX, 0x7C00)
a.mov(AX, 0x0201)                 # AH=02 read, AL=1 sector
a.mov(CX, 0x0001)                 # cylinder 0, sector 1
a.mov(DX, 0x0000)                 # head 0, drive 0
a.int_(0x13)
jcc_far("c", "boot_fail")

# A boot sector is only a boot sector if it says so.
a.mov(AX, 0x0000)
a.mov(ES, AX)
a.mov(AX, mem(disp=0x7DFE, seg=ES))
a.cmp(AX, 0xAA55)
jcc_far("nz", "boot_nosig")

a.mov_label(SI, "msg_boot_ok")
a.call("puts")
a.mov(DX, 0x0000)                 # DL = boot drive, as a boot sector expects
a.jmpf(0x0000, 0x7C00)

a.label("boot_nosig")
a.mov_label(SI, "msg_nosig")
a.call("puts")
a.jmps("boot_stop")

a.label("boot_fail")
a.mov_label(SI, "msg_readfail")
a.call("puts")

a.label("boot_stop")
a.hlt()
a.jmps("boot_stop")

# ===========================================================================
# Default and fault handlers
# ===========================================================================
a.label("int_ignore")
a.iret()

a.label("int_divzero")
a.mov_label(SI, "msg_divzero")
a.call("puts")
a.iret()

# Records the whole machine state at 0040:0090 before saying anything, so it
# survives the halt and can be read back over JTAG. An unimplemented
# instruction inside a guest operating system is otherwise nearly impossible to
# place: the message says it happened and nothing says where, and the machine
# is stopped so nothing can be asked afterwards. The layout, as words:
#
#   +00 ES  +02 DS  +04 DI  +06 SI  +08 BP  +0A SP  +0C BX  +0E DX
#   +10 CX  +12 AX  +14 IP  +16 CS  +18 FLAGS  +1A SS  +1C.. eight words of
#   the faulting code's own stack
#
# which is exactly the order PUSHA and the interrupt frame leave on the stack,
# so the whole record is one REP MOVSW rather than thirteen stores.
a.label("int_illegal")
a.pusha()
a.push(DS)
a.push(ES)
a.mov(BP, SP)

a.mov(AX, BDA)
a.mov(ES, AX)
a.push(SS)
a.pop(DS)                         # DS:SI walks the stack, ES:DI the BDA
a.cld()
a.mov(SI, BP)
a.mov(DI, B_FAULT)
a.mov(CX, 13)
a.rep(); a.movsw()

# SS is the one register PUSHA does not save, and without it the SP above says
# nothing -- the stack cannot be found.
a.mov(AX, SS)
a.stosw()                         # ES:DI, which REP MOVSW left in the right place

# ...and what the faulting code had on its own stack, which is where a bad
# far call or return shows up.
a.mov(SI, BP)
a.add(SI, 26)                     # 8 PUSHA words + DS + ES + IP + CS + FLAGS
a.mov(CX, 8)
a.rep(); a.movsw()

a.pop(ES)
a.pop(DS)
a.popa()
a.mov_label(SI, "msg_illegal")
a.call("puts")
a.label("illegal_halt")
a.hlt()
a.jmps("illegal_halt")

# ===========================================================================
# Scancode tables -- PS/2 set 2 to ASCII
# ===========================================================================
# The keyboard, and which scancode set this table is in.
#
# keyboard_controller.sv translates set 2 to set 1 before software ever sees a
# byte, exactly as a PC's 8042 does, so everything below is SET 1. It is
# derived from the set 2 map rather than retyped, using the same table the
# hardware is generated from -- see tools/scancodes.py for why there is only
# one copy of it.
_SET2_BASE = {
    0x1C: 'a', 0x32: 'b', 0x21: 'c', 0x23: 'd', 0x24: 'e', 0x2B: 'f',
    0x34: 'g', 0x33: 'h', 0x43: 'i', 0x3B: 'j', 0x42: 'k', 0x4B: 'l',
    0x3A: 'm', 0x31: 'n', 0x44: 'o', 0x4D: 'p', 0x15: 'q', 0x2D: 'r',
    0x1B: 's', 0x2C: 't', 0x3C: 'u', 0x2A: 'v', 0x1D: 'w', 0x22: 'x',
    0x35: 'y', 0x1A: 'z',
    0x45: '0', 0x16: '1', 0x1E: '2', 0x26: '3', 0x25: '4', 0x2E: '5',
    0x36: '6', 0x3D: '7', 0x3E: '8', 0x46: '9',
    0x29: ' ', 0x5A: '\r', 0x66: '\b', 0x0D: '\t', 0x76: '\x1b',
    0x4E: '-', 0x55: '=', 0x54: '[', 0x5B: ']', 0x5D: '\\',
    0x4C: ';', 0x52: "'", 0x41: ',', 0x49: '.', 0x4A: '/', 0x0E: '`',
}

from scancodes import SET2_TO_SET1                             # noqa: E402

SET1_BASE = {}
for _s2, _ch in _SET2_BASE.items():
    _s1 = SET2_TO_SET1.get(_s2)
    assert _s1, "set 2 %02X has no set 1 code; the tables disagree" % _s2
    assert _s1 not in SET1_BASE, "set 1 %02X claimed twice" % _s1
    SET1_BASE[_s1] = _ch

SHIFT_MAP = {
    '1': '!', '2': '@', '3': '#', '4': '$', '5': '%', '6': '^',
    '7': '&', '8': '*', '9': '(', '0': ')', '-': '_', '=': '+',
    '[': '{', ']': '}', '\\': '|', ';': ':', "'": '"',
    ',': '<', '.': '>', '/': '?', '`': '~',
}


def build_table(shifted):
    t = bytearray(128)
    for code, ch in SET1_BASE.items():
        if shifted:
            ch = ch.upper() if ch.isalpha() else SHIFT_MAP.get(ch, ch)
        t[code] = ord(ch)
    return t


a.label("kbd_lower")
for b in build_table(False):
    a.db(b)
a.label("kbd_upper")
for b in build_table(True):
    a.db(b)

# ===========================================================================
# Strings
# ===========================================================================
a.label("banner")
a.dz("FPGA80186 BIOS -- 640K, VGA text, PS/2 keyboard, block storage.\r\n")
a.label("msg_boot")
a.dz("Booting from disk 0...\r\n")
a.label("msg_boot_ok")
a.dz("Boot sector loaded, starting.\r\n")
a.label("msg_nosig")
a.dz("No bootable disk: missing AA55 signature.\r\n")
a.label("msg_readfail")
a.dz("Disk read failed.\r\n")
a.label("msg_divzero")
a.dz("Divide by zero.\r\n")
a.label("msg_illegal")
a.dz("Illegal instruction. Halted.\r\n")

code_end = a.here()

# ===========================================================================
# Reset vector
# ===========================================================================
a.org(RESET_AT)
a.jmpf(0xF000, 0x0100)

a.resolve()

# ---- emit ----
args = [a for a in sys.argv[1:] if not a.startswith("--")]

def write_mif(path, vals, width):
    """Write the .mif beside the .hex.

    THE TWO MUST NOT DRIFT. With ISMCE set, bios_rom synthesises from the .mif
    and every simulation reads the .hex, so a stale .mif means the board runs
    different code from the one the tests pass against -- silently, and with
    nothing in the build to hint at it. That cost an afternoon once: the BIOS
    gained a video mode, every test agreed it worked, and the bitstream
    contained the previous BIOS because only the .hex had been regenerated.
    Writing both here removes the opportunity.
    """
    digits = (width + 3) // 4
    with open(path, "w") as f:
        f.write("-- Written by tools/gen_bios.py. Do not edit.\n")
        f.write("-- The RTL simulates from the .hex and synthesises from this.\n")
        f.write("DEPTH = %d;\n" % len(vals))
        f.write("WIDTH = %d;\n" % width)
        f.write("ADDRESS_RADIX = HEX;\n")
        f.write("DATA_RADIX = HEX;\n")
        f.write("CONTENT BEGIN\n")
        for i, v in enumerate(vals):
            f.write("  %X : %0*X;\n" % (i, digits, v))
        f.write("END;\n")

out = args[0] if args else "rom"
os.makedirs(out, exist_ok=True)

for bank, name in ((0, "bios.lo"), (1, "bios.hi")):
    vals = [a.buf[i * 2 + bank] for i in range(ROM_SIZE // 2)]
    with open(os.path.join(out, name + ".hex"), "w") as f:
        f.write("// Boot ROM, %s byte bank.\n" % ("even" if bank == 0 else "odd"))
        f.write("// Generated by tools/gen_bios.py -- do not edit, regenerate.\n")
        for v in vals:
            f.write("%02X\n" % v)
    # ...and the .mif the ISMCE build synthesises from, always together.
    write_mif(os.path.join(out, name + ".mif"), vals, 8)

# A symbol map, because the only debugging window into a running BIOS is the
# IP the CPU stopped at. Turning 03BA back into a routine name by hand is
# tedious and error-prone.
with open(os.path.join(out, "bios.map"), "w") as f:
    f.write("# FPGA80186 BIOS symbols, CS=F000\n")
    for name, addr in sorted(a.labels.items(), key=lambda kv: kv[1]):
        f.write("%04X %s\n" % (addr, name))

# The clock rate this ROM's timer divisor was computed for, written out so the
# hardware can refuse to build against a ROM meant for a different clock. Two
# numbers in two languages in two files WILL drift, and the symptom when they
# do is not a build error -- it is every DOS program that measures time running
# at the wrong speed, which looks like a dozen other things first.
with open(os.path.join(out, "clk.svh"), "w") as f:
    f.write("// Written by tools/gen_bios.py -- do not edit.\n")
    f.write("// The CPU clock rate this BIOS's timer divisor assumes.\n")
    f.write("// FPGA80186.sv checks its own CLK_HZ against this and refuses\n")
    f.write("// to compile if they differ. Rebuild the ROM with\n")
    f.write("//     python3 tools/gen_bios.py --clk-hz <rate> rom/\n")
    f.write("localparam int ROM_CLK_HZ = %d;\n" % CLK_HZ)

used = code_end - 0x100
print("BIOS: %d bytes of code and data (%.1f%% of %d KB)"
      % (used, 100.0 * used / ROM_SIZE, ROM_SIZE // 1024))
print("entry F000:0100, reset vector at image offset %04X" % RESET_AT)
print("wrote %s/bios.{lo,hi}.{hex,mif} and %s/bios.map" % (out, out))
