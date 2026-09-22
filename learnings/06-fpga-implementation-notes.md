# 06 — FPGA Implementation Notes (Project-Specific Synthesis)

This file is my own synthesis, not a direct restatement of the datasheets — it connects the
material in files 00–05 to the actual `modules/` skeleton in this repo and flags the design
decisions and pitfalls specific to reproducing the 80186 on an FPGA (DE1-SoC target, per
`top.sv`/`FPGA80186.sv`) with an eventual MS-DOS boot goal.

## Current skeleton → datasheet concept mapping

| Module | Datasheet concept it implements |
|---|---|
| `clk_rst.sv` | Board-clock→CPU-clock (PLL) + reset synchronization. Does **not** need to model the 80186's crystal/EFI/÷2 circuitry — just needs correct reset *timing behavior* (≥4-clock stable low, synchronized deassertion) per `04-integrated-peripherals.md` §2. |
| `biu.sv` | Bus Interface Unit: T1–T4/Tw generation, ALE/RD/WR timing, address demux, prefetch queue, HOLD/HLDA. See `02-bus-interface.md`. |
| `memory_controller.sv` | Chip-select/ready-generation unit + actual memory backing (BRAM/SDRAM) + the PC-style memory map (conventional RAM / VRAM / BIOS ROM). See `04-integrated-peripherals.md` §3 for what a real UMCS/LMCS/PACS/MPCS register interface would look like if bit-exact BIOS compatibility is wanted later. |
| `eu.sv` | Execution Unit top: owns `decode`, `reg`, `ALU`, `execUnit`, `microcode`; interfaces to `biu.sv` for fetch/mem-read/mem-write. |
| `decode.sv` | ModR/M + opcode → control signals. Needs full 8086 opcode map + the 80186 additions in `03-instruction-set.md`. |
| `execUnit.sv` | Effective-address computation (base+index+disp+segment), flag updates, multicycle sequencing hookup to ALU. |
| `microcode.sv` | Per-instruction micro-op sequencing — the natural home for MUL/IMUL/DIV/IDIV iteration, PUSHA/POPA, ENTER(level>1), and REP-prefixed string ops, all of which are genuinely multicycle per `03-instruction-set.md`'s timing table. |
| `reg.sv` | GP/pointer/segment/IP/FLAGS register file. See `01-programming-model.md` for exact bit layout of FLAGS and reset values (CS=FFFFh, IP=0000h, others undefined→zero is fine). |
| `ALU.sv` | 8/16-bit ALU ops + flags. See known issues below. |
| `vga_controller.sv` | Not part of the 80186 itself — a project-added peripheral for the MS-DOS text-mode display goal (Phase 4). |

Nothing in the current skeleton yet models the **interrupt controller**, **timers**, or
**DMA unit** as distinct peripheral register blocks — see "Open design decisions" below.

## Known issue already flagged in the code: `ALU.sv` MUL/IMUL/DIV/IDIV

The current stub (`modules/ALU.sv`) does 1-cycle combinational MUL/IMUL/DIV/IDIV via `<=`
in a clocked block, which doesn't match either (a) real 80186 timing (26–67 clocks, per
`03-instruction-set.md`) or (b) synthesizable shift-add multiply hardware. Specific bugs
visible in the current MUL branch:
- `product <= product + b[count-1] ? a<<count : 0;` — operator precedence makes this
  `product <= (product + b[count-1]) ? (a<<count) : 0`, not the intended
  `product + (b[count-1] ? a<<count : 0)`. Needs explicit parens.
- `count-1` when `count==0` underflows to a large unsigned value, reading garbage from `b`
  on the first iteration.
- IMUL still does `a * b` unsigned; per `03-instruction-set.md`, IMUL needs `$signed(a) *
  $signed(b)` and both MUL/IMUL word forms need `result_hi = product[31:16]` wired up (byte
  forms only need `result`, matching `AH:AL` vs `DX:AX` placement rules from the base 8086
  ISA).
- DIV/IDIV are single-cycle `a/b`/`a/b` with no remainder output, no divide-by-zero handling
  (the `div_zero` output exists but is never driven), and no distinction between signed and
  unsigned division. These need the same multicycle restructuring, sized per the timing
  table (29–67 clocks depending on operand size/signedness).

This was already anticipated by the module's own comments (`// FIX:` markers) — worth fixing
alongside decode/microcode work rather than leaving it, since MUL/DIV correctness is a common
silent-failure source in from-scratch CPU builds.

## Bus-cycle atomicity: the single most important BIU invariant

Per `02-bus-interface.md`, a huge fraction of the 80186's bus-timing subtlety boils down to
one rule: **a bus cycle, once started, always runs to completion** — HOLD requests, DMA
requests, and even interrupt-acknowledge sequencing all only take effect at defined
cycle-boundary states, never mid-T-state. Concretely:
- An odd-address word access (2 back-to-back byte cycles) is never split by HOLD.
- A DMA transfer's fetch+deposit pair is never split by HOLD or by the other DMA channel.
- A LOCKed instruction's data cycles are never split by HOLD or DMA.
- Interrupt-acknowledge cycles (when run) are always a locked pair.

If `biu.sv`'s FSM is built so that "should I release the bus / should I service DMA / should
I run an INTA" is checked *only* at the T4→(T1|Ti) boundary, all of the above invariants come
for free. Building it any other way (e.g. checking HOLD every clock) risks subtle bugs that
only show up under specific instruction/DMA/interrupt interleavings — exactly the kind of bug
that's painful to find after the fact on real hardware waveforms.

## Prefer SRDY over ARDY for the FPGA memory controller

Since `memory_controller.sv`'s "memory" is on-chip (BRAM now, SDRAM later) rather than a real
external chip with unknown response latency, there is no genuine asynchronous ready source in
this design — `ready` can be computed synchronously. Model it as **SRDY-style** (sampled at
the start of T3/Tw, no synchronizer chain needed) rather than reproducing ARDY's 2-stage
resolve/latch synchronizer, which only exists in the real chip to handle a truly external,
clock-independent ready signal. This simplifies `biu.sv`'s wait-state logic meaningfully.

## Reset sequencing must produce a *working* boot path before any instruction executes

Per `05-interrupts-and-reset.md`, the real 80186 doesn't rely on software to make the reset
vector fetchable — UCS resets pre-programmed (top 1K, 3 wait states, factor external ready).
`cpu_top.sv`'s reset handling should treat "UCS-equivalent active over the BIOS ROM region"
as part of the reset state, not something that has to wait for a chip-select register write.
Since `memory_controller.sv` already hardcodes the BIOS ROM at 0xF0000–0xFFFFF, this project
gets this "for free" architecturally — just make sure `rst_n` deassertion doesn't leave a
window where `biu.sv` fetches from FFFF0h before `memory_controller.sv`'s `ready` logic for
that region is valid.

## Reset vector and IVT are non-negotiable for MS-DOS/BIOS compatibility

Any BIOS/bootloader/DOS code that will run on this CPU assumes, unconditionally:
- First fetch at physical **FFFF0h** (`00000h` IVT below it is untouched at power-on, but
  BIOS will populate it early).
- IVT at physical **00000h–003FFh**, 256 × 4-byte far pointers, vector = type×4.
- Divide error = type 0, single-step = type 1, NMI = type 2, breakpoint = type 3, INTO =
  type 4 — these four are inherited straight from the 8086 and any BIOS/DOS code assumes
  them.
- The PC/XT convention of hardware IRQs living at INT 08h–0Fh (via a discrete 8259, not the
  80186's own type 8/10/12 assignments) is an IBM-PC-BIOS convention, **not** an 80186
  hardware default — see the open design decision below.

## Open design decision: how literally to emulate the integrated peripherals

Real IBM PC/XT-compatible software (BIOS, MS-DOS, DOS device drivers) is written against
discrete **8259A** (PIC), **8253/8254** (PIT), and **8237** (DMA) chips at fixed I/O
addresses (8259A at 20h/21h, 8253 at 40h–43h, 8237 at 00h–0Fh, etc.), not against the
80186's *own* integrated interrupt-controller/timer/DMA register layouts (which live in the
relocatable PCB and have a different register model entirely, per `04-integrated-
peripherals.md` and `05-interrupts-and-reset.md`). A handful of concrete options, worth a
decision in `plans/` before investing RTL effort in interrupt-controller/timer/DMA modules:

1. **Emulate discrete 8259A/8253/8237 as separate peripheral modules** at PC-standard I/O
   addresses, ignore the 80186's own integrated equivalents entirely (don't even wire them
   up). Closest to "just build a PC/XT with an 80186 CPU core," maximizes off-the-shelf
   BIOS/DOS compatibility, but throws away the 80186's actual integration story.
2. **Implement the 80186's real integrated peripherals faithfully** and write a custom BIOS
   that targets *them* directly (this is what real 80186-based embedded/industrial products
   did — they were never meant to be IBM PC clones). Historically accurate, but means writing
   or heavily patching BIOS/DOS I/O routines instead of reusing PC BIOS conventions.
3. **Hybrid**: real hobbyist 80186 PC-clone projects (this appears to be one, given the VGA/
   PS2/DOS goals in `top.sv` and `memory_controller.sv` comments) typically implement the
   80186's integrated timer/interrupt-controller/chip-select logic *and* front it with a BIOS
   that remaps/reprograms it to behave like the PC-standard I/O map as much as possible, or
   add small discrete-chip-emulation shims only where BIOS/DOS is inflexible (classically:
   the 8259A's I/O addresses and IRQ vector numbers, since real DOS/BIOS interrupt handling
   is hard-coded to expect IRQ0=timer at INT 08h).

Given the stated goal ("eventually run MS-DOS"), option 3 (or a clean option 1) is the
practical path — pure option 2 means writing a DOS-compatible BIOS from scratch with no
reference implementation to lean on. This should be an explicit `plans/` entry, since it
determines whether `04-integrated-peripherals.md`'s DMA/timer/interrupt-controller register
maps are "build these as-is" or "build PC-standard equivalents instead and treat the 80186's
own versions as optional/skippable."

## Suggested build order (informed by the above)

1. **BIU + memory_controller + register file + basic ALU + decode for a minimal instruction
   subset** (MOV, ADD/SUB, jumps) — enough to execute simple test programs from BRAM and
   validate the fetch/execute loop and bus-cycle timing against the T1–T4 model.
2. **Full 8086 instruction set decode + multicycle sequencing in microcode.sv** (this is
   where the bulk of the ALU fix-ups and multicycle MUL/DIV/string-op work lands).
3. **80186-specific instructions** (PUSHA/POPA, ENTER/LEAVE, INS/OUTS, BOUND, shift-by-imm,
   IMUL-by-imm) plus the illegal-opcode/segment-overflow/shift-mask execution differences
   from `03-instruction-set.md`.
4. **Interrupt handling** (at minimum: IVT fetch, INTR/NMI acceptance rules, IRET) — needed
   before any peripheral can usefully interrupt the CPU, and needed by DOS/BIOS regardless
   of which peripheral-emulation option is chosen above.
5. **Peripheral decision execution** (section above) — timers/PIC/DMA, real or PC-standard
   equivalents, informed by whatever `plans/` decides.
6. **VGA + PS/2 (Phase 4, already stubbed in `vga_controller.sv`/`top.sv`)** — cosmetic/IO
   layer on top of a working CPU+memory+interrupt foundation.

This order defers the single biggest open architectural question (peripheral emulation
strategy) until after the CPU core itself is solid, which is where nearly all of the
correctness risk actually lives.
