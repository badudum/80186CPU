# 04 — Integrated Peripherals: Clock Gen, Chip-Selects, DMA, Timers, Interrupt Controller

All integrated peripherals share one architectural trait worth internalizing before reading
the rest of this file: they are all controlled through 16-bit registers living in a single
**256-byte Peripheral Control Block (PCB)**, relocatable to any 256-byte boundary in memory
or I/O space via a relocation register at PCB offset FEh. On reset, the PCB defaults to I/O
addresses **FF00h–FFFFh** (relocation register = 20FFh). All PCB accesses must be **word**
accesses; the CPU automatically inserts 0 wait states for PCB accesses (1 wait state
specifically for timer registers, due to counter-element time-multiplexing) and always
ignores external READY for these addresses.

PCB register map (offsets from PCB base):

| Offset | Registers |
|---|---|
| FEh | Relocation register (ET, M/IO, RMX bits + relocation address bits 19–8) |
| A0h–A8h | Chip-select control: UMCS, LMCS, PACS, MMCS/MPCS |
| 50h–66h | Timer 0/1/2 control, count, max-count-A/B registers |
| 20h–3Eh | Interrupt controller registers |
| C0h–DAh | DMA channel 0 descriptors |
| CAh–DEh | DMA channel 1 descriptors |

## 1. Clock generator

- On-chip crystal oscillator (parallel-resonant, Pierce) or external oscillator (EFI pin
  drives X1 directly, X2 left open) at **2× the desired CPU clock** (e.g. 16 MHz crystal for
  an 8 MHz CPU) — unlike the 8086's 8284A, which needs 3× the CPU clock.
- Internal ÷2 counter produces a **50% duty cycle** CLKOUT (vs 8086/8284A's 33% duty cycle).
  All device timings reference CLKOUT.
- No PCLK output, no oscillator output pin — both present on 8284A, absent here.
- For this FPGA project, `clk_rst.sv` doesn't need to reproduce oscillator behavior at all —
  the DE1-SoC's 50 MHz board clock plus a PLL (or simple divider for early bring-up) directly
  produces `clk_cpu`, sidestepping the whole crystal/EFI story. The relevant part to
  reproduce faithfully is **reset timing**, not clock generation.

## 2. Reset behavior

- RES is active-high, Schmitt-triggered (debounce-friendly), must be held low ("stable") for
  more than 4 clocks for reset to take effect; RESET (output) stays active for at least 5
  clocks given a RES input of at least 6 clocks.
- After RES deasserts, the CPU begins fetching its **first instruction ~6.5 CPU clocks
  later**, at physical address **FFFF0h** (CS=FFFFh, IP=0000h).
- Reset state of each peripheral (all from the 80186 datasheet's explicit per-block "and
  RESET" sections — this is the authoritative list to replicate in RTL reset logic):
  - **Chip-selects**: all six memory CS outputs driven high (inactive) *except* **UCS**,
    which is programmed active for the **top 1K byte block** with **3 wait states** and
    external-ready-factored — i.e. `UMCS` register resets to `FFFBH`. This is the mechanism
    that lets the CPU actually read boot ROM immediately after reset before any software
    initialization has run. No PCS line is active until *both* PACS and MPCS have been
    accessed (a read is enough — need not be a write with a "useful" value).
  - **DMA channels**: Start/Stop bit cleared (stopped) for both channels; any in-flight
    transfer aborted.
  - **Timers**: all EN bits cleared (no counting); for timers 0/1, RIU=0 and ALT=1, which
    drives TMR OUT pins high.
  - **Interrupt controller**: SFNM=0 (fully nested), all PR (priority) bits=1 (lowest
    priority, level 111), LTM=0 (edge-triggered), all in-service bits=0, all request bits=0,
    all mask bits=1 (**everything masked**), all cascade bits=0, all priority-mask bits=1
    (no levels masked), mode=Master (non-iRMX).
  - **Relocation register**: 20FFh (PCB at I/O FF00h–FFFFh).
- Practical consequence for `cpu_top.sv`/`memory_controller.sv`: the reset FSM must bring up
  a *working* chip-select/wait-state configuration for the top-1K region without any CPU
  instruction having executed yet — this is hardwired reset state, not something software
  programs first. Software (BIOS init code) then reprograms UMCS/LMCS/PACS/MPCS once it's
  running, per the AP-186 Appendix F example (see below), and must do so **before** jumping
  out of the initial 1K-byte ROM window, or the chip-selects go inactive and the system
  hangs fetching garbage.

Reset software example (from AP-186 Appendix F, adapted) — this is the archetypal 80186
bring-up sequence and a good reference for the project's own boot ROM code:
```asm
; org 0FFFF0h (reset vector)
        jmp   far ptr initialize

; located in the top-1K ROM window so it's guaranteed selected out of reset
UMCS_reg equ 0FFA0h
LMCS_reg equ 0FFA2h
PACS_reg equ 0FFA4h
MPCS_reg equ 0FFA8h

initialize:
        mov   dx, UMCS_reg
        mov   ax, 0F800h      ; e.g. 64K upper memory, 0 wait states
        out   dx, ax
        mov   dx, LMCS_reg
        mov   ax, 07F8h       ; e.g. 32K lower memory, 0 wait states
        out   dx, ax
        mov   dx, PACS_reg
        mov   ax, 0072h       ; peripheral base 400h, 2 wait states
        out   dx, ax
        mov   dx, MPCS_reg
        mov   ax, 00BAh       ; PCS5/6 as A1/A2, peripherals in I/O space
        out   dx, ax
        jmp   far ptr monitor  ; hand off to the real boot code
```

## 3. Chip-select / ready generation unit

Six memory chip selects + seven peripheral chip selects, all active low, **never floated
during HOLD** (external bus master accessing local bus during HOLD needs its own external
chip-select logic — the 80186 provides none for it).

**Memory chip selects** (3 regions):

| Signal | Region | Bounds | Programmability |
|---|---|---|---|
| UCS | Upper memory | fixed top at FFFFFh, programmable-size block below it | size only |
| LCS | Lower memory | fixed bottom at 00000h, programmable-size block above it | size only |
| MCS0–3 | Mid-range memory | fully programmable base + size | base must be a multiple of total block size |

**Peripheral chip selects**: PCS0–6, each active for one of 7 contiguous 128-byte blocks
above a programmable base address (base may be in memory or I/O space, programmer's
choice). PCS5/PCS6 can alternatively be configured as latched A1/A2 outputs (for driving
address pins on 8-bit peripheral chips in a non-demultiplexed subsystem) — if so configured
they can't also act as chip selects.

**Ready/wait-state bits**: each of 5 register groups (upper mem, lower mem, mid-range mem,
PCS0–3, PCS4–6) has its own 2-bit wait-state count (R1,R0) plus a bit selecting whether
external ready (ARDY/SRDY) is factored in at all:

| R1 R0 | Wait states (if external ready also factored) | Wait states (external ready ignored) |
|---|---|---|
| 0 0 | 0 + external ready | 0 |
| 0 1 | 1 + external ready | 1 |
| 1 0 | 2 + external ready | 2 |
| 1 1 | 3 + external ready | 3 |

Overlapping chip-select areas are explicitly discouraged (indeterminate ready behavior
unless both overlapping areas are programmed with identical ready bits); none should overlap
the PCB's own 256-byte block either.

**FPGA implementation note**: since this design's "memory" is entirely on-chip (BRAM/SDRAM
controller) rather than discrete external ROM/RAM chips with real access-time constraints,
the chip-select unit's *primary* remaining value here is (a) reproducing the reset-state
UCS/wait-state behavior so real 80186 boot code and any bit-exact compatibility tests behave
correctly, and (b) providing the address-decode/region-select function that
`memory_controller.sv` already does via its own address ranges. Consider implementing the
*register interface* (UMCS/LMCS/PACS/MPCS at their PCB offsets, matching reset values) even
if the "wait state count" bits mostly get ignored in favor of `memory_controller.sv`'s own
`ready` logic — that keeps real 80186 BIOS/DOS init code (which pokes these registers) from
breaking, without requiring bit-accurate wait-state emulation.

## 4. DMA unit

Two independent channels, each with: 20-bit source pointer, 20-bit destination pointer,
16-bit transfer count, 16-bit control register. All pointers address the **full 1 MB space
as flat/linear (no segmentation)** — for I/O-space pointers, upper 4 bits should be
programmed to 0.

- Every DMA transfer = 2 bus cycles (fetch, then deposit), never separated by HOLD or by the
  other DMA channel; minimum 4-clock latency from DRQ assertion to the first DMA cycle
  starting (independent of wait states).
- Bus cycles run by the DMA unit are indistinguishable from CPU-initiated ones except that
  **S6 is driven high** (vs low for CPU cycles) — this is how external logic (or this
  project's memory controller, if it cares) can tell DMA accesses apart from CPU accesses.
- Transfer count register decrements by 1 per transfer (byte or word alike); count of 0
  means 65536 transfers. If the TC bit is set, ST/STOP auto-clears when count hits 0
  (optionally also raising an interrupt if INT bit set too).
- DMA requests can come from: an external pin (DRQ0/DRQ1), timer 2 timeout, or continuously
  (self-triggered/"unsynchronized" mode).
- Source/destination-synchronized modes control exact DRQ sampling semantics — source-sync
  allows back-to-back transfers; destination-sync inserts idle time so the requesting device
  can drop its request.
- No explicit DMA-acknowledge pin — the DMA unit just performs an ordinary read/write to the
  requesting device's address; if a physical DACK pulse is needed, external logic
  synthesizes it from a PCS line, gated by ALE (because addresses aren't stable when chip
  selects first go active).
- Reset: both channels' Start/Stop bit cleared; any transfer aborted.

**FPGA note**: for an MS-DOS target, standard PC/XT DOS/BIOS code assumes discrete 8237-style
DMA (used for floppy disk transfers, DRAM refresh on real XTs, etc.), not the 80186's
integrated DMA. Decide explicitly whether this project (a) emulates an 8237 as a separate
peripheral module, (b) relies on the 80186's DMA unit and adapts BIOS/driver code
accordingly, or (c) skips hardware DMA entirely for an early bring-up phase (PIO-only disk
access). This is a `plans/` decision, not something to default silently.

## 5. Timer unit

Three independent 16-bit timers, logically modeled as one physical counter element
time-multiplexed across 3 register banks (so timer operation is fully asynchronous to bus
interface T-state sequencing — a CPU register write to a timer register takes effect the
next time that timer is serviced by the shared counter element, not instantaneously).

- **Timer 0 & 1**: each has an input pin and output pin, a 16-bit count register, **two**
  max-count registers (A and B — selectable single or dual/alternating mode via the ALT
  bit), and a control register.
- **Timer 2**: no external pins; only max-count register A. Used as: a free-running
  interrupt-on-timeout timer, a prescaler feeding timer 0/1's count-up-on-timer-2-timeout
  mode, or a DMA-request source.
- Count increments on: a timer event = external pin transition (timers 0/1 only, low-to-high,
  synchronized+latched), a CPU-clock-÷4 tick (all timers, from counter-element multiplexing
  — max count rate is CPU-clock/4 = 2 MHz @ 8 MHz CPU), or timer 2 rollover (timers 0/1 only).
- Control register bits: **EN** (enable counting — writes to it only take effect if **INH**
  is also set in the same write, allowing selective update), **CONT** (continuous — auto-
  restart at cycle end vs auto-disable), **ALT** (dual max-count mode), **RTG**/**EXT**
  (input-pin function: level-gate the enable, retrigger/one-shot on transition, or count
  external events), **MC** (max-count-reached flag, sticky, must be cleared by software).
- Reads/writes to timer registers always incur **1 wait state**, deterministically, even
  though other PCB accesses incur 0 — a fixed, known latency worth modeling exactly in
  `memory_controller.sv`'s PCB-access path if bit-exact BIOS timing ever matters.
- Reset: all EN bits cleared; timers 0/1 get RIU=0, ALT=1 (TMR OUT pins driven high).
- Typical uses demonstrated in AP-186: real-time clock (timer 2 → 1 ms interrupt,
  software increments a memory tick counter), UART baud-rate generator (timer output feeds
  TxC/RxC of a serial chip), event counter (count external pulses directly in hardware).

## 6. Interrupt controller

See `05-interrupts-and-reset.md` for the full interrupt-controller register set, vector
table, and priority/masking model — it's substantial enough to warrant its own file.
