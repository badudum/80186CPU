# 05 — Interrupts, Interrupt Controller, and Reset

## Interrupt vector table

Identical mechanism to the 8086: 256 possible interrupt types, each with a 4-byte pointer
(2-byte offset, then 2-byte segment) stored at physical address `type × 4`, occupying
physical **00000h–003FFh**. On an acknowledged interrupt, the CPU pushes FLAGS, CS, IP (in
that order), clears IF and TF, then loads CS:IP from the vector table entry for that type.
IRET pops IP, CS, FLAGS to return (restoring the original IF/TF state).

Fixed/reserved vectors relevant to normal operation:

| Type | Source | Notes |
|---|---|---|
| 0 | Divide error | Raised by DIV/IDIV on overflow (see `03-instruction-set.md` for the 80186's slightly wider negative-quotient allowance) |
| 1 | Single-step (TF) | Fires after each instruction when TF=1 |
| 2 | NMI | Non-maskable, edge-triggered (low→high on the NMI pin), latched, can't be disabled by IF |
| 3 | Breakpoint (INT 3) | 1-byte opcode `CCh` |
| 4 | INTO (overflow) | Only traps if OF is set |
| 5 | BOUND range exceeded | New on 80186 |
| 6 | Invalid opcode | New/expanded on 80186 (see instruction-set-differences table) |
| 7 | ESC / coprocessor-not-available | Optional trap, programmable via relocation register's ET bit |
| 8, 18, 19 | Timer 0 / Timer 1 / Timer 2 (master mode default vectors) | Fixed, non-programmable in master mode |
| 10, 11 | DMA channel 0 / 1 | Fixed in master mode |
| 12–15 | INT0–INT3 (external pins) | Fixed in master mode |

(Vector = type × 4 in all cases — e.g. type 8's vector lives at physical 20h.)

## Interrupt controller operation

The integrated controller has two mutually exclusive modes, set via the RMX bit in the
relocation register:

- **Master mode** (default after reset): the controller is the system's master interrupt
  controller and presents its output directly to the CPU core. INT0–INT3 pins are available
  as direct external interrupt inputs (or cascade-mode inputs to external 8259As).
- **iRMX 86 mode**: the controller acts as a *slave* to an external interrupt controller
  (needed for iRMX 86 OS compatibility, or when pairing with an 80130/80150 firmware chip).
  In this mode INT0–INT3 aren't used as direct inputs; instead each timer gets its own
  individual control register (rather than sharing one), and the controller must request
  service from the CPU via the INT3 pin, and respond to interrupt-acknowledge cycles from
  an *external* controller rather than vectoring the CPU directly.

For an MS-DOS-target hobby build, **master mode is almost certainly what you want** — iRMX
86 mode exists purely for Intel's own RTOS ecosystem.

### Register set (master mode; PCB offsets)

| Offset | Register | Notes |
|---|---|---|
| 3Eh | INT3 control | |
| 3Ch | INT2 control | |
| 3Ah | INT1 control | |
| 38h | INT0 control | |
| 38h (iRMX) | DMA1 control | offset overlaps depending on mode — see datasheet Fig. 49 |
| 36h | DMA1 control | |
| 34h | DMA0 control | |
| 32h | Timer control (shared, master mode) | all 3 timers share ONE control/status path in master mode |
| 30h | Interrupt Controller Status register | |
| 2Eh | Interrupt Request register | |
| 2Ch | In-Service register | |
| 2Ah | Priority Mask register | |
| 28h | Mask register | |
| 22h | EOI register | Specific-EOI in iRMX mode |
| 20h | (iRMX only) Interrupt Vector register | |

**Per-source control register** (one bit layout, shared shape for INT0–3/DMA0-1/Timer):
3 priority-level bits (0 = highest, 7 = lowest), 1 mask bit (0 = enabled, 1 = masked).
INT0/INT1 control registers additionally carry a **C** (Cascade) bit and **SFNM** (Special
Fully Nested Mode) bit; all 4 external-interrupt control registers carry an **LTM**
(level/edge-trigger select) bit.

**Request / In-Service / Mask registers**: 7 active bits each (one per interrupt source in
master mode). Request-register bits for peripheral-sourced interrupts (timers, DMA) are
read/write; bits for the 4 external pins are read-only (the pin state *is* the request —
not latched, so if the external line deasserts the request bit clears itself too, unless
edge-triggered mode is in use and armed).

**Priority Mask register**: 3 bits recording the priority level of the interrupt currently
being serviced; blocks any lower-or-equal priority source from interrupting until EOI'd.
Automatically updated by EOI (non-specific EOI: highest-priority in-service bit is found and
cleared automatically; specific EOI: caller names which bit to clear).

**Poll / Poll-Status registers**: software-polling alternative to interrupt-driven service —
reading the Poll register acknowledges the pending interrupt (sets in-service bit, updates
priority mask) exactly as a real interrupt-acknowledge cycle would, but without actually
vectoring the CPU. Not supported in iRMX 86 mode.

**Interrupt Status register**: bit 15 = DMA halt bit (auto-set on NMI, auto-cleared by
IRET, blocks all DMA activity while set — worth having `execUnit.sv`'s IRET path clear this
if the interrupt-controller model implements it), plus 3 bits indicating *which* timer
caused an interrupt (needed because all 3 timers share one control/request bit in master
mode — the ISR must read this status register to disambiguate).

### Interrupt response sequencing (master mode)

Because the integrated controller directly feeds the CPU core in master mode, **no external
interrupt-acknowledge bus cycles are ever generated** for internally-vectored sources — the
first externally-visible sign of an interrupt is the CPU's read from the interrupt vector
table in low memory. This is a meaningful difference from a plain 8086 (which always runs
two INTA bus cycles to fetch a type byte from an external 8259A): on the 80186 in master
mode, there is no "type byte" externally, the internal controller directly supplies the
vector address (type × 4) to the internal fetch logic.

Exception: if INT0/INT1 are cascade-configured (C bit set) to external 8259As, **two INTA
bus cycles are run**, using INT2/INT3 as the acknowledge pulse outputs for INT0/INT1
respectively — allowing up to 128 individually-vectored external sources via two banks of
8259As.

### Interrupt sources

- **Internal (timers, DMA)**: latched in the controller — even if the peripheral condition
  clears, the pending request remains until acknowledged.
- **External (INT0–INT3)**: NOT latched by the controller in level-triggered mode — request
  (and its request-register bit) tracks the pin live. In edge-triggered mode, a low→high
  transition is required to (re-)arm; the line must then stay low ≥1 clock before it can
  re-arm again.
- Priority: programmer-assigned per-source (3-bit level, 0=highest..7=lowest); among the 3
  timers sharing one control register, there's also a fixed internal tiebreak (timer 0 >
  timer 1 > timer 2).

### Interrupt latency

Worst case ~69 CPU clocks (dominated by the longest uninterruptible instruction, IDIV with a
segment-override prefix). Structural rules that block interrupt acceptance (relevant to
`microcode.sv`'s sequencing, since these are boundaries the sequencer must recognize):
- Never accepted mid-instruction (only between iterations of a repeated string op, or at an
  instruction boundary) — except NMI and single-step, which still only take effect between
  instructions/iterations, not truly mid-instruction.
- Never accepted between a prefix byte (segment override, REP, LOCK) and the instruction it
  prefixes.
- Never accepted between an instruction that modifies a segment register and the very next
  instruction (protects SS:SP atomicity across a segment-register load).
- Never accepted between WAIT and the following instruction *if* TEST is still active
  (WAIT re-executes after the interrupt returns, in that case).

## Reset (cross-reference — full detail in `04-integrated-peripherals.md`)

Quick summary since it's the first thing any bring-up test will exercise:
- RES active high, ≥4 stable clocks to take effect, Schmitt-triggered input.
- First fetch ~6.5 clocks after RES deasserts, at physical **FFFF0h**.
- Interrupt controller resets to: master mode, fully-nested (SFNM=0), edge-triggered
  (LTM=0), everything masked (all MSK=1), all priorities=lowest (111), no in-service/request
  bits set, priority mask = "nothing masked" (all 1s meaning no level blocks anything yet —
  don't confuse this with the MSK bits, which *do* block everything at reset).
- Because everything is masked at reset, no spurious interrupts can fire before software
  explicitly unmasks sources it wants — a real 80186 boot ROM must deliberately configure
  and unmask the interrupt controller before relying on any interrupt-driven peripheral
  (timer tick, keyboard IRQ, etc.).

## FPGA/RTL implication

An interrupt controller module (not yet present in `modules/` — likely belongs inside `eu.sv`
or as a new peripheral module alongside `memory_controller.sv`) needs: the 7-bit
request/mask/in-service register triple, the 3-bit priority-mask register, per-source
3-bit-priority+1-bit-mask control registers, EOI handling (specific + non-specific), and a
priority resolver that finds "highest-priority unmasked, requested, not-already-in-service"
source each cycle. This is genuinely more complex than the ALU or register file and deserves
its own `plans/` design pass before RTL — it's effectively a small 8259A clone plus the
extra internal-source plumbing.
