#!/usr/bin/env python3
"""
Encoding tests for asm86.

Every expectation here is a known-good x86 encoding. This matters more than a
normal unit test: a wrong byte does not raise, it produces a program that runs
and misbehaves somewhere else entirely, and the CPU it runs on is also under
development -- so a bad encoding would look like a CPU bug.

Run:  python3 tools/test_asm86.py
"""
import sys
from asm86 import (Asm, mem, AsmError,
                   AX, CX, DX, BX, SP, BP, SI, DI,
                   AL, CL, DL, BL, AH, CH, DH, BH,
                   ES, CS, SS, DS)

fails = 0
count = 0


def chk(desc, build, expect):
    """build(a) emits one instruction; expect is the byte string."""
    global fails, count
    count += 1
    a = Asm(64)
    build(a)
    a.resolve()
    got = bytes(a.buf[:a.pos])
    want = bytes(expect)
    if got != want:
        print("FAIL %-32s got %s  want %s"
              % (desc, got.hex(" "), want.hex(" ")))
        fails += 1


# ---- immediates into registers ----
chk("mov ax,1234h", lambda a: a.mov(AX, 0x1234), [0xB8, 0x34, 0x12])
chk("mov di,0",     lambda a: a.mov(DI, 0x0000), [0xBF, 0x00, 0x00])
chk("mov al,42h",   lambda a: a.mov(AL, 0x42),   [0xB0, 0x42])
chk("mov bh,0FFh",  lambda a: a.mov(BH, 0xFF),   [0xB7, 0xFF])

# ---- segment registers ----
chk("mov ds,ax", lambda a: a.mov(DS, AX), [0x8E, 0xD8])
chk("mov es,ax", lambda a: a.mov(ES, AX), [0x8E, 0xC0])
chk("mov ss,ax", lambda a: a.mov(SS, AX), [0x8E, 0xD0])
chk("mov ax,es", lambda a: a.mov(AX, ES), [0x8C, 0xC0])

# ---- register to register ----
chk("mov bx,ax", lambda a: a.mov(BX, AX), [0x8B, 0xD8])
chk("mov al,bl", lambda a: a.mov(AL, BL), [0x8A, 0xC3])

# ---- direct memory ----
chk("mov ax,[1234h]", lambda a: a.mov(AX, mem(disp=0x1234)),
    [0x8B, 0x06, 0x34, 0x12])
chk("mov [1234h],ax", lambda a: a.mov(mem(disp=0x1234), AX),
    [0x89, 0x06, 0x34, 0x12])
chk("mov [1234h],al", lambda a: a.mov(mem(disp=0x1234), AL),
    [0x88, 0x06, 0x34, 0x12])

# ---- indexed ----
chk("mov [bx],al",      lambda a: a.mov(mem(BX), AL),        [0x88, 0x07])
chk("mov al,[bx+si+2]", lambda a: a.mov(AL, mem(BX, SI, 2)), [0x8A, 0x40, 0x02])
chk("mov ax,[si]",      lambda a: a.mov(AX, mem(SI)),        [0x8B, 0x04])
chk("mov ax,[di]",      lambda a: a.mov(AX, mem(DI)),        [0x8B, 0x05])
# [BP] with zero displacement must use the disp8 form: mod=00 rm=110 is the
# direct-address escape, so a naive encoder emits [0000] instead.
chk("mov al,[bp]",      lambda a: a.mov(AL, mem(BP)),        [0x8A, 0x46, 0x00])
chk("mov ax,[bp+4]",    lambda a: a.mov(AX, mem(BP, disp=4)), [0x8B, 0x46, 0x04])
chk("mov ax,[bx+100h]", lambda a: a.mov(AX, mem(BX, disp=0x100)),
    [0x8B, 0x87, 0x00, 0x01])

# ---- segment override ----
chk("es: mov [di],al", lambda a: a.mov(mem(DI, seg=ES), AL), [0x26, 0x88, 0x05])

# ---- memory immediates ----
chk("mov word [100h],1234h",
    lambda a: a.movw(mem(disp=0x100), 0x1234),
    [0xC7, 0x06, 0x00, 0x01, 0x34, 0x12])
chk("mov byte [bx],5", lambda a: a.movb(mem(BX), 5), [0xC6, 0x07, 0x05])

# ---- stack ----
chk("push ax", lambda a: a.push(AX), [0x50])
chk("pop bx",  lambda a: a.pop(BX),  [0x5B])
chk("push es", lambda a: a.push(ES), [0x06])
chk("pop ds",  lambda a: a.pop(DS),  [0x1F])
chk("pusha",   lambda a: a.pusha(),  [0x60])
chk("popa",    lambda a: a.popa(),   [0x61])

# ---- ALU ----
chk("add ax,5",   lambda a: a.add(AX, 5),  [0x05, 0x05, 0x00])
chk("add bx,5",   lambda a: a.add(BX, 5),  [0x83, 0xC3, 0x05])
chk("cmp al,0",   lambda a: a.cmp(AL, 0),  [0x3C, 0x00])
chk("cmp bl,20h", lambda a: a.cmp(BL, 0x20), [0x80, 0xFB, 0x20])
chk("sub cx,dx",  lambda a: a.sub(CX, DX), [0x2B, 0xCA])
chk("xor ax,ax",  lambda a: a.xor(AX, AX), [0x33, 0xC0])
chk("and al,7Fh", lambda a: a.and_(AL, 0x7F), [0x24, 0x7F])
chk("or bx,cx",   lambda a: a.or_(BX, CX),  [0x0B, 0xD9])
chk("cmp ax,100h", lambda a: a.cmp(AX, 0x100), [0x3D, 0x00, 0x01])

chk("inc ax", lambda a: a.inc(AX), [0x40])
chk("dec di", lambda a: a.dec(DI), [0x4F])
chk("inc al", lambda a: a.inc(AL), [0xFE, 0xC0])

chk("test al,1",  lambda a: a.test(AL, 1),  [0xA8, 0x01])
chk("test bl,80h", lambda a: a.test(BL, 0x80), [0xF6, 0xC3, 0x80])

chk("mul bx", lambda a: a.mul(BX), [0xF7, 0xE3])
chk("div bl", lambda a: a.div(BL), [0xF6, 0xF3])
chk("cbw",    lambda a: a.cbw(),   [0x98])

# ---- shifts (immediate count is an 80186 addition) ----
chk("shl al,1",  lambda a: a.shl(AL, 1),  [0xD0, 0xE0])
chk("shl ax,4",  lambda a: a.shl(AX, 4),  [0xC1, 0xE0, 0x04])
chk("shr bx,1",  lambda a: a.shr(BX, 1),  [0xD1, 0xEB])
chk("shl cl,cl", lambda a: a.shl(CL, CL), [0xD2, 0xE1])

# ---- misc ----
chk("lea si,[bx+2]", lambda a: a.lea(SI, mem(BX, disp=2)), [0x8D, 0x77, 0x02])
chk("xchg ax,bx",    lambda a: a.xchg(AX, BX), [0x93])
chk("int 10h",       lambda a: a.int_(0x10),   [0xCD, 0x10])
chk("iret",          lambda a: a.iret(),       [0xCF])
chk("retf",          lambda a: a.retf(),       [0xCB])
chk("cli",           lambda a: a.cli(),        [0xFA])
chk("stosw",         lambda a: a.stosw(),      [0xAB])
chk("out dx,ax",     lambda a: a.out_dx(AX),   [0xEF])
chk("in ax,dx",      lambda a: a.in_dx(AX),    [0xED])
chk("in al,dx",      lambda a: a.in_dx(AL),    [0xEC])
chk("xlat",          lambda a: a.xlat(),       [0xD7])


def rep_stosw(a):
    a.rep()
    a.stosw()


chk("rep stosw", rep_stosw, [0xF3, 0xAB])
chk("jmp far F000:0100", lambda a: a.jmpf(0xF000, 0x0100),
    [0xEA, 0x00, 0x01, 0x00, 0xF0])

# ---- relative branches, which is where hand-encoding goes wrong ----


def fwd_jz(a):
    a.jz("skip")          # 2 bytes
    a.nop()               # 1
    a.label("skip")
    a.nop()


chk("forward jz over one byte", fwd_jz, [0x74, 0x01, 0x90, 0x90])


def back_jmp(a):
    a.label("top")
    a.nop()
    a.jmps("top")         # target is 3 bytes back from the end of this insn


chk("backward short jmp", back_jmp, [0x90, 0xEB, 0xFD])


def near_call(a):
    a.call("f")           # 3 bytes
    a.hlt()               # 1
    a.label("f")
    a.ret()


chk("near call forward", near_call, [0xE8, 0x01, 0x00, 0xF4, 0xC3])


def loop_back(a):
    a.label("l")
    a.stosw()
    a.loop("l")


chk("loop backward", loop_back, [0xAB, 0xE2, 0xFD])

# ---- the origin must be honoured by absolute references ----


def abs_ref(a):
    a.mov(AX, 0)
    a.label("data")


a = Asm(64, origin=0xF000)
a.org(0xF000)
a.mov(AX, 0x1234)
count += 1
if a.here() != 0xF003:
    print("FAIL origin tracking: here()=%04X want F003" % a.here())
    fails += 1

# ---- errors must raise, not silently emit something wrong ----


def expect_error(desc, fn):
    global fails, count
    count += 1
    try:
        fn()
    except AsmError:
        return
    print("FAIL %s did not raise" % desc)
    fails += 1


def far_short_jump():
    a = Asm(4096)
    a.jmps("far")
    for _ in range(200):
        a.nop()
    a.label("far")
    a.resolve()


expect_error("out-of-range short jump", far_short_jump)


def undefined_label():
    a = Asm(64)
    a.jmps("nowhere")
    a.resolve()


expect_error("undefined label", undefined_label)


def bad_mode():
    a = Asm(64)
    a.mov(AX, mem(CX))          # [CX] is not a 16-bit addressing mode


expect_error("invalid addressing mode", bad_mode)

print()
print("=" * 40)
print(" asm86 encodings: %d checked, %d failed" % (count, fails))
print("=" * 40)
sys.exit(1 if fails else 0)
