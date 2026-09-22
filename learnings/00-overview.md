# 00 — Overview: What the 80186 Is and Why It Matters Here

## Sources
- `docs/210973-001_AP-186_Introduction_to_the_80186_Microprocessor_Mar83.pdf` (Intel Application Note AP-186, March 1983) — architectural narrative, "AP-186" below.
- `docs/80186_datasheet.pdf` (Intel 80186/80188 datasheet, order #272430-002) — authoritative electrical/timing/instruction reference, "80186 datasheet" below.
- `docs/intel-8086_datasheet.pdf` — base ISA and bus model the 80186 extends, "8086 datasheet" below.

## What the 80186 is

The 80186 is Intel's follow-on to the 8086: the same base 16-bit architecture (object-code
compatible with 8086/8088 — any 8086 binary runs unmodified) but with **six major
peripheral blocks integrated onto one chip**:

1. An enhanced CPU (8086-compatible EU + BIU, faster clocks per instruction)
2. A clock generator (on-chip oscillator + divide-by-2, no external 8284A needed)
3. Two independent DMA channels
4. A programmable interrupt controller (8259A-like, cascadable)
5. Three programmable 16-bit timers
6. Programmable memory and peripheral chip-select / wait-state generation logic

The 80188 is the same die with an 8-bit external data bus (like 8088 vs 8086) — same
internal 16-bit machine, half the pins wiggling. This project targets the 80186 (16-bit bus),
per the existing `memory_controller.sv` which already assumes a 16-bit data path.

Both 80186 and 8086 share a common base architecture with the 80286; the 80186 does **not**
add memory protection/management (that's the 80286) — it adds integration and instruction
throughput.

## Why this matters for an FPGA-from-scratch implementation

Building a real 80186 (not just an 8086 core) means the FPGA design has to reproduce, at
minimum:
- The CPU's instruction set and bus-cycle behavior (this is "just" an 8086-compatible core
  with a handful of new instructions — see `03-instruction-set.md`).
- The **integrated peripherals**, because real 80186 software (including startup/BIOS code
  and DOS) can rely on the chip-select unit being present and correctly reset (e.g. `UMCS`
  auto-selecting the top 1K of memory with 3 wait states out of reset — this is how every
  real 80186 system boots). See `04-integrated-peripherals.md` and `05-interrupts-and-reset.md`.
- Correct reset vector behavior: CS=FFFFh, IP=0000h → first fetch at physical FFFF0h,
  matching this project's `memory_controller.sv` BIOS ROM region (0xF0000–0xFFFFF).

For an MS-DOS target specifically, the peripheral set matters less than get-the-CPU-right,
because DOS/BIOS on a real IBM PC/XT-class machine used **discrete** 8237 DMA, 8259 PIC,
8253 PIT chips, not the 80186's integrated ones — a "PC/XT compatible" 80186 machine (like
the ones this hobby scene builds) typically front the CPU's integrated peripherals as if they
were those discrete parts, or bypasses them and adds real 8259/8253/8237 chips instead. But
the 80186's own peripherals are still worth implementing well because (a) they're required
for the datasheet-correct reset/chip-select sequencing that gates whether the CPU can even
fetch its first instructions from ROM, and (b) some hobby 80186 PC/XT clones do drive
peripherals through the integrated units directly with custom BIOS glue. Decide the target
peripheral story in `plans/` before spending time on DMA/timer RTL — see
`06-fpga-implementation-notes.md`.

## High-level architecture: BIU vs EU

Like the 8086, the 80186 CPU is internally split into two independently-clocked,
asynchronously-cooperating units:

- **Bus Interface Unit (BIU)**: generates all bus cycles (T1–T4 + wait states), fetches
  opcode bytes into a 6-byte prefetch queue (4 bytes on 80188), demultiplexes/multiplexes the
  address/data bus, handles HOLD/HLDA, drives chip-selects and ready logic. Maps to this
  project's `modules/biu.sv` and `modules/memory_controller.sv`.
- **Execution Unit (EU)**: pulls opcode bytes out of the prefetch queue, decodes and
  executes instructions, computes effective addresses, drives the ALU, updates the register
  file and flags. Maps to `modules/eu.sv`, `modules/decode.sv`, `modules/execUnit.sv`,
  `modules/ALU.sv`, `modules/reg.sv`, `modules/microcode.sv`.

Because these units are decoupled by the prefetch queue, the BIU can keep fetching ahead
while the EU chews on a long instruction (e.g. a 67-clock IDIV), and bus-cycle wait states
matter less to real-world throughput than a naive analysis suggests (AP-186 §3.1.10) — worth
remembering when deciding how tightly to couple `biu.sv` and `eu.sv` in the RTL: they should
be genuinely separate FSMs communicating through a small queue/handshake, not fused into one
monolithic sequencer.

## Key numeric facts to remember

| Fact | Value |
|---|---|
| Address space | 1 MB (20-bit physical address), memory-mapped |
| I/O space | 64 KB |
| Physical address formula | `(segment << 4) + offset`, truncated to 20 bits (carry-out ignored) |
| Prefetch queue depth (80186) | 6 bytes |
| Reset vector (first fetch) | physical **FFFF0h** (CS=FFFFh, IP=0000h) |
| Interrupt vector table | physical **00000h**–003FFh (256 vectors × 4 bytes) |
| Minimum bus cycle | 4 T-states (T1–T4) + 0..N wait states |
| Register-to-register MOV | 2 clocks (fastest instruction) |
| Slowest instruction | IDIV, up to 67 clocks (80186) |
| Peripheral control block | 256 bytes, relocatable, defaults to I/O FF00h–FFFFh after reset |
