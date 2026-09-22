# 01 — Programming Model: Registers, Segments, Flags, Addressing

## Registers

Fully object-code compatible with the 8086; the register set is unchanged.

**General purpose (16-bit), individually byte-addressable as high/low halves:**

| 16-bit | High byte | Low byte | Conventional use |
|---|---|---|---|
| AX | AH | AL | Accumulator (required operand for many ops: IN/OUT, string ops, MUL/DIV) |
| BX | BH | BL | Base register (only GP register usable as a memory base, along with BP) |
| CX | CH | CL | Count register (loop/shift/rotate/string-repeat counts) |
| DX | DH | DL | Data register (IN/OUT variable-port address, MUL/DIV high half) |

**Pointer/index registers (16-bit only, no byte access):**

| Register | Use |
|---|---|
| SP | Stack pointer (offset into SS) |
| BP | Base pointer (offset into SS by default — used for stack-frame addressing) |
| SI | Source index (string ops source; DS-relative by default) |
| DI | Destination index (string ops dest; **always ES-relative**, cannot be overridden) |

**Segment registers (16-bit):**

| Register | Selects |
|---|---|
| CS | Code segment — all instruction fetches |
| SS | Stack segment — all PUSH/POP, and BP-relative memory refs (except explicit override) |
| DS | Data segment — default for most data references |
| ES | Extra segment — string-instruction destination (DI), forced, not overridable |

**Other:**
- **IP** — 16-bit instruction pointer, offset into CS of the next instruction to fetch.
- **FLAGS** — 16-bit status/control register (see below).

Register file sizing for `reg.sv`: 8× 16-bit GP/pointer regs (or 4×16-bit with H/L byte
muxing for AX/BX/CX/DX), 4× 16-bit segment regs, 1×16-bit IP, 1×16-bit FLAGS. Reset state:
CS=FFFFh, IP=0000h; other registers are architecturally undefined at reset (safe to zero
them in RTL).

## Segment:offset addressing / physical address generation

Physical (20-bit) address = `(segment_register << 4) + offset`, discarding any carry out of
bit 19. This is identical on 8086 and 80186 (AP-186 §2.1, Figure 2). The offset comes from
combinations of pointer registers, IP, and/or an immediate displacement, depending on
addressing mode.

Because of the `<<4` overlap, a given physical address is reachable through many
segment:offset pairs (e.g. `1000:0010` and `0000:1010` are both physical `00110h`) —
segments overlap rather than partition memory.

**Byte lane addressing:** memory is organized as two 512K-byte banks (even bytes on
D0–D7 / AD0–AD7, odd bytes on D8–D15 / AD8–AD15). `A0` and `BHE` together select which
byte lane(s) are enabled per bus cycle:

| BHE | A0 | Meaning |
|---|---|---|
| 0 | 0 | Word transfer (both bytes) |
| 0 | 1 | Byte transfer, upper half (D15–D8) |
| 1 | 0 | Byte transfer, lower half (D7–D0) |
| 1 | 1 | Reserved / not used |

A word access to an **odd** address costs two bus cycles (first the odd byte on D15–D8 at
the base word address, then the even byte on D7–D0 at the next word address) — this is why
word data/instructions should be word-aligned for performance, and why `memory_controller.sv`
and `biu.sv` need explicit odd-word-access splitting logic, not just a single 16-bit access
path.

## FLAGS register (standard 8086/80186 layout, unchanged)

| Bit | Flag | Name | Notes |
|---|---|---|---|
| 0 | CF | Carry Flag | Unsigned overflow/borrow |
| 2 | PF | Parity Flag | Set if low byte of result has even parity |
| 4 | AF | Auxiliary Carry | BCD carry out of bit 3 (DAA/DAS/AAA/AAS) |
| 6 | ZF | Zero Flag | |
| 7 | SF | Sign Flag | = MSB of result |
| 8 | TF | Trap Flag | Single-step interrupt (type 1) after each instruction when set |
| 9 | IF | Interrupt Enable Flag | Gates INTR (not NMI) |
| 10 | DF | Direction Flag | String ops: 0 = auto-increment SI/DI, 1 = auto-decrement |
| 11 | OF | Overflow Flag | Signed overflow |

Bits 1, 3, 5, 12–15 are reserved (fixed values on 8086: bit1=1, others=0; the 80186 datasheet
does not call out different reserved-bit behavior, so assume 8086-compatible). `ALU.sv`
already declares `cf, pf, af, zf, sf, of` outputs — good match; `execUnit.sv`/`reg.sv` need to
pack/unpack these into a 16-bit FLAGS word with the correct bit positions above, and handle
TF/IF/DF as CPU-state bits set by CLI/STI/CLD/STD/POPF rather than ALU outputs.

## Addressing modes (ModR/M-driven, unchanged from 8086)

Effective address = `[base] + [index] + [displacement]`, with base/index drawn from a fixed
set of combinations encoded in the ModR/M byte:

| Combination | Base | Index | Default segment |
|---|---|---|---|
| BX + SI | BX | SI | DS |
| BX + DI | BX | DI | DS |
| BP + SI | BP | SI | SS |
| BP + DI | BP | DI | SS |
| SI alone | — | SI | DS |
| DI alone | — | DI | DS |
| BP alone (direct if mod=00) | BP | — | SS |
| BX alone | BX | — | DS |
| Direct address (mod=00, r/m=110) | — | — | DS |

Displacement can be none, 8-bit signed, or 16-bit, per the `mod` field of ModR/M. Segment
override prefixes (2Eh CS, 36h SS, 3Eh DS, 26h ES) force a specific segment regardless of the
table above, at a cost of 2 clocks. `decode.sv` needs a ModR/M table matching this, and
`execUnit.sv` needs the base+index+disp adder plus segment-override handling. Note AP-186's
warning: an interrupt is *not* accepted between an instruction that modifies a segment
register and the very next instruction (needed so SS:SP updates stay atomic) — relevant when
building interrupt-acceptance logic in the EU/microcode sequencer.

## Memory map conventions (this project)

Per `modules/memory_controller.sv` comments, the planned map is:
- `0x00000–0x9FFFF`: conventional RAM (640 KB)
- `0xA0000–0xBFFFF`: video RAM (text mode buffer at B8000–B8FFF)
- `0xC0000–0xEFFFF`: option ROM / extended BIOS (optional)
- `0xF0000–0xFFFFF`: BIOS ROM (64 KB), reset vector at FFFF0h

This is the standard IBM PC/XT memory layout, which is what MS-DOS/BIOS code on 80186-class
hobby machines expects — consistent with targeting MS-DOS.
