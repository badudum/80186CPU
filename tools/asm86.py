#!/usr/bin/env python3
"""
A small 16-bit x86 assembler, enough to write a BIOS in.

gen_bios.py started with a handful of hand-encoded opcodes and patched jump
displacements by hand. That is fine for twenty instructions and a liability for
a thousand: the failure mode is a jump that lands one byte into the middle of
an instruction, after which the CPU faithfully executes an immediate operand as
an opcode and the symptom appears somewhere else entirely. Two of those cost
real debugging time earlier in this project.

So displacements are computed here, once, and every reference is resolved
through a fixup table that is checked at the end. An unresolved or
out-of-range reference raises rather than silently truncating.

OPERANDS
    Registers are the module-level constants AX..DI / AL..BH / ES..DS.
    Memory is mem(...), e.g.
        mem(disp=0x1234)          -> [1234h]
        mem(BX)                   -> [BX]
        mem(BP, disp=4)           -> [BP+4]
        mem(BX, SI, disp=2)       -> [BX+SI+2]
    A segment override is mem(..., seg=ES).

Only the forms this BIOS needs are implemented. Anything missing raises a
clear error rather than emitting a wrong encoding.
"""

# ---- registers -------------------------------------------------------------
# 16-bit and 8-bit register numbers share an encoding space; W in the opcode
# picks which file is meant, so the same number means AX or AL depending on
# the instruction. The classes keep them from being mixed up by accident.


class Reg:
    def __init__(self, num, size, name):
        self.num, self.size, self.name = num, size, name

    def __repr__(self):
        return self.name


AX, CX, DX, BX, SP, BP, SI, DI = (Reg(i, 16, n) for i, n in enumerate(
    "AX CX DX BX SP BP SI DI".split()))
AL, CL, DL, BL, AH, CH, DH, BH = (Reg(i, 8, n) for i, n in enumerate(
    "AL CL DL BL AH CH DH BH".split()))


class Sreg:
    def __init__(self, num, name):
        self.num, self.name = num, name

    def __repr__(self):
        return self.name


ES, CS, SS, DS = (Sreg(i, n) for i, n in enumerate("ES CS SS DS".split()))

SEG_PREFIX = {0: 0x26, 1: 0x2E, 2: 0x36, 3: 0x3E}


class Mem:
    """A 16-bit memory operand."""

    def __init__(self, base=None, index=None, disp=0, seg=None):
        self.base, self.index, self.disp, self.seg = base, index, disp, seg


def mem(base=None, index=None, disp=0, seg=None):
    return Mem(base, index, disp, seg)


# The 16-bit r/m table. Key is (base, index) by register number.
_RM_TABLE = {
    (BX.num, SI.num): 0, (BX.num, DI.num): 1,
    (BP.num, SI.num): 2, (BP.num, DI.num): 3,
    (SI.num, None): 4, (DI.num, None): 5,
    (BP.num, None): 6, (BX.num, None): 7,
}


class AsmError(Exception):
    pass


class Asm:
    def __init__(self, size, origin=0):
        self.buf = bytearray(size)
        self.size = size
        self.pos = 0
        self.origin = origin          # IP that corresponds to buf[0]
        self.labels = {}
        self.fixups = []              # (pos, kind, label, end_pos)

    # ---- placement ----
    def org(self, offset):
        self.pos = offset - self.origin
        if not 0 <= self.pos <= self.size:
            raise AsmError("org %04X outside the image" % offset)

    def here(self):
        """The IP of the current position."""
        return self.origin + self.pos

    def label(self, name):
        if name in self.labels:
            raise AsmError("duplicate label %r" % name)
        self.labels[name] = self.here()
        return name

    # ---- raw emission ----
    def db(self, *bs):
        for b in bs:
            if self.pos >= self.size:
                raise AsmError("image overflow at %04X" % self.here())
            self.buf[self.pos] = b & 0xFF
            self.pos += 1

    def dw(self, *ws):
        for w in ws:
            self.db(w & 0xFF, (w >> 8) & 0xFF)

    def dz(self, s):
        """A NUL-terminated string."""
        for ch in s.encode("ascii"):
            self.db(ch)
        self.db(0)

    # ---- references ----
    def _ref(self, kind, label):
        self.fixups.append([self.pos, kind, label, None])
        if kind == "rel8":
            self.db(0)
        else:
            self.dw(0)
        self.fixups[-1][3] = self.here()

    def resolve(self):
        for pos, kind, label, end in self.fixups:
            if label not in self.labels:
                raise AsmError("undefined label %r" % label)
            target = self.labels[label]
            if kind == "rel8":
                d = target - end
                if not -128 <= d <= 127:
                    raise AsmError("short jump to %r out of range (%d)"
                                   % (label, d))
                self.buf[pos] = d & 0xFF
            elif kind == "rel16":
                d = (target - end) & 0xFFFF
                self.buf[pos] = d & 0xFF
                self.buf[pos + 1] = (d >> 8) & 0xFF
            elif kind == "abs16":
                self.buf[pos] = target & 0xFF
                self.buf[pos + 1] = (target >> 8) & 0xFF
            else:
                raise AsmError("bad fixup kind %r" % kind)

    # ---- ModR/M ----
    def _modrm(self, reg_field, rm):
        """Emit ModR/M (+ displacement). Segment prefix must precede this."""
        if isinstance(rm, Reg):
            self.db(0xC0 | (reg_field << 3) | rm.num)
            return
        if not isinstance(rm, Mem):
            raise AsmError("bad r/m operand %r" % (rm,))

        b = rm.base.num if rm.base is not None else None
        i = rm.index.num if rm.index is not None else None
        # Allow mem(SI) as well as mem(index=SI)
        if b in (SI.num, DI.num) and i is None and rm.base in (SI, DI):
            key = (b, None)
        else:
            key = (b, i)

        if b is None and i is None:
            # Direct address: mod=00, rm=110
            self.db(0x06 | (reg_field << 3))
            self.dw(rm.disp)
            return

        if key not in _RM_TABLE:
            raise AsmError("unsupported addressing mode base=%r index=%r"
                           % (rm.base, rm.index))
        rmf = _RM_TABLE[key]
        d = rm.disp
        # [BP] with no displacement would encode as direct address, so it
        # must use the disp8 form with a zero displacement instead.
        if d == 0 and not (rmf == 6 and b == BP.num):
            self.db(0x00 | (reg_field << 3) | rmf)
        elif -128 <= d <= 127:
            self.db(0x40 | (reg_field << 3) | rmf)
            self.db(d & 0xFF)
        else:
            self.db(0x80 | (reg_field << 3) | rmf)
            self.dw(d)

    def _seg(self, rm):
        if isinstance(rm, Mem) and rm.seg is not None:
            self.db(SEG_PREFIX[rm.seg.num])

    def _w(self, op):
        """Operand size bit from a register operand."""
        return 1 if op.size == 16 else 0

    # ---- data movement ----
    def mov(self, dst, src):
        # reg, imm
        if isinstance(dst, Reg) and isinstance(src, int):
            if dst.size == 16:
                self.db(0xB8 + dst.num)
                self.dw(src)
            else:
                self.db(0xB0 + dst.num)
                self.db(src)
            return
        # sreg, reg
        if isinstance(dst, Sreg) and isinstance(src, Reg):
            self.db(0x8E)
            self._modrm(dst.num, src)
            return
        # reg, sreg
        if isinstance(dst, Reg) and isinstance(src, Sreg):
            self.db(0x8C)
            self._modrm(src.num, dst)
            return
        # mem, sreg  /  sreg, mem
        if isinstance(dst, Mem) and isinstance(src, Sreg):
            self._seg(dst)
            self.db(0x8C)
            self._modrm(src.num, dst)
            return
        if isinstance(dst, Sreg) and isinstance(src, Mem):
            self._seg(src)
            self.db(0x8E)
            self._modrm(dst.num, src)
            return
        # reg, r/m   and   r/m, reg
        if isinstance(dst, Reg) and isinstance(src, (Reg, Mem)):
            self._seg(src)
            self.db(0x8A + self._w(dst))
            self._modrm(dst.num, src)
            return
        if isinstance(dst, Mem) and isinstance(src, Reg):
            self._seg(dst)
            self.db(0x88 + self._w(src))
            self._modrm(src.num, dst)
            return
        # mem, imm
        if isinstance(dst, Mem) and isinstance(src, int):
            raise AsmError("mov mem,imm needs an explicit width -- "
                           "use movw/movb")
        raise AsmError("unsupported mov %r, %r" % (dst, src))

    def movw(self, dst, imm):
        self._seg(dst)
        self.db(0xC7)
        self._modrm(0, dst)
        self.dw(imm)

    def movb(self, dst, imm):
        self._seg(dst)
        self.db(0xC6)
        self._modrm(0, dst)
        self.db(imm)

    def mov_label(self, reg, label):
        """mov reg16, OFFSET label -- resolved once the label is placed."""
        if not isinstance(reg, Reg) or reg.size != 16:
            raise AsmError("mov_label needs a 16-bit register")
        self.db(0xB8 + reg.num)
        self._ref("abs16", label)

    def dw_label(self, label):
        """A 16-bit data word holding a label's offset."""
        self._ref("abs16", label)

    def lea(self, dst, src):
        self.db(0x8D)
        self._modrm(dst.num, src)

    def xchg(self, a, b):
        if isinstance(a, Reg) and isinstance(b, Reg) and a.size == 16:
            if a is AX:
                self.db(0x90 + b.num)
                return
            if b is AX:
                self.db(0x90 + a.num)
                return
        self.db(0x86 + (1 if a.size == 16 else 0))
        self._modrm(a.num, b)

    # ---- stack ----
    def push(self, op):
        if isinstance(op, Reg):
            self.db(0x50 + op.num)
        elif isinstance(op, Sreg):
            self.db(0x06 | (op.num << 3))
        else:
            raise AsmError("unsupported push %r" % (op,))

    def pop(self, op):
        if isinstance(op, Reg):
            self.db(0x58 + op.num)
        elif isinstance(op, Sreg):
            self.db(0x07 | (op.num << 3))
        else:
            raise AsmError("unsupported pop %r" % (op,))

    def pusha(self):
        self.db(0x60)

    def popa(self):
        self.db(0x61)

    def pushf(self):
        self.db(0x9C)

    def popf(self):
        self.db(0x9D)

    # ---- ALU ----
    _ALU = {"add": 0, "or": 1, "adc": 2, "sbb": 3,
            "and": 4, "sub": 5, "xor": 6, "cmp": 7}

    def alu(self, op, dst, src):
        code = self._ALU[op]
        if isinstance(src, int):
            if not isinstance(dst, (Reg, Mem)):
                raise AsmError("bad alu destination")
            w = 1 if (isinstance(dst, Mem) or dst.size == 16) else 0
            # The AL/AX short forms save a byte and are what a real
            # assembler picks.
            if isinstance(dst, Reg) and dst.num == 0:
                self.db((code << 3) | 0x04 | w)
                self.dw(src) if w else self.db(src)
                return
            self._seg(dst)
            if w and -128 <= src <= 127:
                self.db(0x83)           # sign-extended imm8
                self._modrm(code, dst)
                self.db(src & 0xFF)
            else:
                self.db(0x80 | w)
                self._modrm(code, dst)
                self.dw(src) if w else self.db(src)
            return
        if isinstance(dst, Reg) and isinstance(src, (Reg, Mem)):
            self._seg(src)
            self.db((code << 3) | 0x02 | self._w(dst))
            self._modrm(dst.num, src)
            return
        if isinstance(dst, Mem) and isinstance(src, Reg):
            self._seg(dst)
            self.db((code << 3) | 0x00 | self._w(src))
            self._modrm(src.num, dst)
            return
        raise AsmError("unsupported %s %r, %r" % (op, dst, src))

    def add(self, d, s): self.alu("add", d, s)
    def sub(self, d, s): self.alu("sub", d, s)
    def cmp(self, d, s): self.alu("cmp", d, s)
    def and_(self, d, s): self.alu("and", d, s)
    def or_(self, d, s): self.alu("or", d, s)
    def xor(self, d, s): self.alu("xor", d, s)
    def adc(self, d, s): self.alu("adc", d, s)

    def test(self, dst, src):
        if isinstance(src, int):
            w = 1 if (isinstance(dst, Mem) or dst.size == 16) else 0
            if isinstance(dst, Reg) and dst.num == 0:
                self.db(0xA8 | w)
                self.dw(src) if w else self.db(src)
                return
            self._seg(dst)
            self.db(0xF6 | w)
            self._modrm(0, dst)
            self.dw(src) if w else self.db(src)
            return
        self._seg(dst)
        self.db(0x84 + self._w(src))
        self._modrm(src.num, dst)

    def inc(self, op):
        if isinstance(op, Reg) and op.size == 16:
            self.db(0x40 + op.num)
        else:
            self._seg(op)
            self.db(0xFE if (isinstance(op, Reg) and op.size == 8) else 0xFF)
            self._modrm(0, op)

    def dec(self, op):
        if isinstance(op, Reg) and op.size == 16:
            self.db(0x48 + op.num)
        else:
            self._seg(op)
            self.db(0xFE if (isinstance(op, Reg) and op.size == 8) else 0xFF)
            self._modrm(1, op)

    def neg(self, op):
        self._seg(op)
        self.db(0xF6 | self._unary_w(op))
        self._modrm(3, op)

    def _unary_w(self, op):
        """Operand-size bit. A memory operand has no width of its own, so it
        is taken as a word -- the byte forms are always written with an
        explicit 8-bit register here."""
        if isinstance(op, Mem):
            return 1
        return 1 if op.size == 16 else 0

    def mul(self, op):
        self._seg(op)
        self.db(0xF6 | self._unary_w(op))
        self._modrm(4, op)

    def div(self, op):
        self._seg(op)
        self.db(0xF6 | self._unary_w(op))
        self._modrm(6, op)

    def cbw(self): self.db(0x98)
    def cwd(self): self.db(0x99)

    # ---- shifts (immediate count is an 80186 addition) ----
    def _shift(self, sub, op, count):
        w = 1 if (isinstance(op, Mem) or op.size == 16) else 0
        if count == 1:
            self.db(0xD0 | w)
            self._modrm(sub, op)
        elif isinstance(count, Reg) and count is CL:
            self.db(0xD2 | w)
            self._modrm(sub, op)
        else:
            self.db(0xC0 | w)
            self._modrm(sub, op)
            self.db(count)

    def shl(self, op, count=1): self._shift(4, op, count)
    def shr(self, op, count=1): self._shift(5, op, count)
    def sar(self, op, count=1): self._shift(7, op, count)

    # ---- control flow ----
    _CC = {"o": 0, "no": 1, "c": 2, "nc": 3, "z": 4, "nz": 5,
           "be": 6, "a": 7, "s": 8, "ns": 9, "p": 10, "np": 11,
           "l": 12, "ge": 13, "le": 14, "g": 15}

    def jcc(self, cc, label):
        self.db(0x70 | self._CC[cc])
        self._ref("rel8", label)

    def jz(self, l): self.jcc("z", l)
    def jnz(self, l): self.jcc("nz", l)
    def jc(self, l): self.jcc("c", l)
    def jnc(self, l): self.jcc("nc", l)
    def ja(self, l): self.jcc("a", l)
    def jbe(self, l): self.jcc("be", l)
    def jl(self, l): self.jcc("l", l)
    def jge(self, l): self.jcc("ge", l)

    def jmp(self, label):
        """Near jump; always 3 bytes so the range is never a surprise."""
        self.db(0xE9)
        self._ref("rel16", label)

    def jmps(self, label):
        self.db(0xEB)
        self._ref("rel8", label)

    def jmpf(self, seg, off):
        self.db(0xEA)
        self.dw(off, seg)

    def call(self, label):
        self.db(0xE8)
        self._ref("rel16", label)

    def ret(self):
        self.db(0xC3)

    def retf(self):
        self.db(0xCB)

    def iret(self):
        self.db(0xCF)

    def int_(self, n):
        if n == 3:
            self.db(0xCC)
        else:
            self.db(0xCD, n)

    def loop(self, label):
        self.db(0xE2)
        self._ref("rel8", label)

    # ---- flags / misc ----
    def cli(self): self.db(0xFA)
    def sti(self): self.db(0xFB)
    def cld(self): self.db(0xFC)
    def std(self): self.db(0xFD)
    def hlt(self): self.db(0xF4)
    def nop(self): self.db(0x90)
    def clc(self): self.db(0xF8)
    def stc(self): self.db(0xF9)

    # ---- string ----
    def movsb(self): self.db(0xA4)
    def movsw(self): self.db(0xA5)
    def stosb(self): self.db(0xAA)
    def stosw(self): self.db(0xAB)
    # INS/OUTS: 80186 additions. They move a word between an I/O port in DX
    # and ES:DI (in) or DS:SI (out), advancing the pointer, so with a REP
    # prefix one instruction does what IN/STOSW/LOOP did per word.
    def insb(self):  self.db(0x6C)
    def insw(self):  self.db(0x6D)
    def outsb(self): self.db(0x6E)
    def outsw(self): self.db(0x6F)
    def lodsb(self): self.db(0xAC)
    def lodsw(self): self.db(0xAD)
    def scasb(self): self.db(0xAE)
    def cmpsb(self): self.db(0xA6)
    def cmpsw(self): self.db(0xA7)

    # F3 is REP for the move/store forms and REPE for the compare/scan forms;
    # F2 is REPNE. The prefix byte is the same opcode either way, so the name
    # just documents which sense is meant at the call site.
    def rep(self): self.db(0xF3)
    def repe(self): self.db(0xF3)
    def repne(self): self.db(0xF2)

    # ---- I/O ----
    def in_dx(self, reg):
        self.db(0xEC if reg.size == 8 else 0xED)

    def out_dx(self, reg):
        self.db(0xEE if reg.size == 8 else 0xEF)

    # ---- xlat ----
    def xlat(self): self.db(0xD7)
