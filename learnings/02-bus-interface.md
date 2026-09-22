# 02 — Bus Interface: Pins, Bus Cycles, Timing, HOLD/HLDA

## Pin summary (from 80186 datasheet Table 1)

| Signal | Type | Function |
|---|---|---|
| AD0–AD15 | I/O | Multiplexed address (T1) / data (T2–T4,Tw) bus |
| A16/S3–A19/S6 | O | High address bits during T1; status bits S3–S6 during T2–T4 (S3–S5 always 0 on 80186; S6 = 0 for CPU-initiated cycle, 1 for DMA-initiated cycle) |
| BHE/S7 | O | Bus High Enable during T1 (enables D15–D8); S7 (≡BHE) during T2–T4, doesn't need latching |
| ALE/QS0 | O | Address Latch Enable (never floats); doubles as Queue Status bit 0 if RD strapped low at reset |
| RD/QSMD | I/O | Read strobe (active low); sampled at reset to select Queue Status Mode if tied to GND |
| WR/QS1 | O | Write strobe (active low); doubles as Queue Status bit 1 |
| ARDY | I | Asynchronous Ready (active high) |
| SRDY | I | Synchronous Ready (active high) |
| DT/R | O | Data Transmit/Receive (bus buffer direction) |
| DEN | O | Data Enable (bus buffer output-enable factor) |
| S0–S2 | O | Bus cycle status (see table below) |
| LOCK | O | Active low; asserted for LOCK-prefixed instruction's data cycles |
| HOLD / HLDA | I / O | Bus arbitration handshake |
| TEST | I | Polled by WAIT instruction (for 8087-style coprocessor sync) |
| NMI | I | Edge-triggered (low→high), non-maskable, type-2 interrupt |
| INT0 / INT1/SELECT / INT2/INTA0 / INT3/INTA1/IRQ | I(/O) | External interrupt inputs; mode-dependent (see `05-interrupts-and-reset.md`) |
| TMR IN0/1, TMR OUT0/1 | I / O | Timer 0/1 external pins |
| DRQ0/DRQ1 | I | DMA request lines, level-triggered, active high |
| UCS, LCS, MCS0–3 | O | Memory chip selects (Upper/Lower/Mid-range), active low, never float on HOLD |
| PCS0–4, PCS5/A1, PCS6/A2 | O | Peripheral chip selects (7 × 128-byte blocks), active low, never float on HOLD |
| RES | I | Reset input, active high, Schmitt-triggered, needs ≥4 stable clocks while low |
| RESET | O | Synchronized reset output, active high |
| X1, X2 | I / O | Crystal inputs (crystal freq = 2× CPU clock) |
| CLKOUT | O | 50%-duty CPU clock, all timings referenced to this |

BHE/A0 encoding table (80186 only — no BHE on 80188):

| BHE | A0 | Function |
|---|---|---|
| 0 | 0 | Word transfer |
| 0 | 1 | Byte transfer, upper half (D15–D8) |
| 1 | 0 | Byte transfer, lower half (D7–D0) |
| 1 | 1 | Reserved |

Status line decode (S2 S1 S0):

| S2 S1 S0 | Bus cycle |
|---|---|
| 0 0 0 | Interrupt Acknowledge |
| 0 0 1 | Read I/O |
| 0 1 0 | Write I/O |
| 0 1 1 | Halt |
| 1 0 0 | Instruction Fetch |
| 1 0 1 | Read Data from Memory |
| 1 1 0 | Write Data to Memory |
| 1 1 1 | Passive (no bus cycle) |

There is no separate memory-RD vs I/O-RD pin — a single RD covers both; if you need to
distinguish, synthesize it from S2 (low = I/O, high = memory) latched with ALE, same as the
real chip's external designers had to.

## T-state bus cycle anatomy

Every bus cycle is a minimum of **4 T-states**: T1 (address), T2/T3/Tw (data + control),
T4 (data latched, cycle ends). Idle states (Ti) fill gaps when no bus activity is needed.
Each T-state = 1 CPU clock, split into phase 1 (clock low) and phase 2 (clock high).

- **T1**: address driven on AD0–AD15 + A16/S3–A19/S6 + BHE/S7. ALE pulses high (asserted on
  the CLKOUT rising edge *before* T1 — one full half-clock earlier than the 8086 — and
  de-asserted in the middle of T1). Addresses valid ≤44 ns after start of T1 (tCLAV), remain
  valid ≥10 ns after T1 ends (tCLAX).
- **T2**: RD or WR goes active (both start at beginning of T2). Bus turns around from
  address to data.
- **T3 / Tw**: data transferred. Wait states (Tw) inserted here if READY not yet satisfied.
- **T4**: data latched by the CPU (reads) at the start of T4; RD/WR de-asserted at start of T4.

Status lines (S0–S2, and the impending BHE/S7, S6) become valid **during the T-state
immediately preceding T1** of the cycle they describe (either a T4 or a Ti) — i.e. one
T-state of lookahead is always available, which is exactly the signal `biu.sv` needs to
know "a bus cycle is coming next cycle" before it actually starts driving addresses.

`RD`/`WR` timing specifics (8 MHz part, for reference — get exact numbers from the AC
characteristics tables in the datasheet if timing-accurate synthesis targets matter):
- RD asserted at start of T2, de-asserted at start of T4.
- WR asserted at start of T2, de-asserted at start of T4. Write data is **not yet valid**
  when WR's active edge occurs (WR corresponds to 8288's "early write") — data becomes
  valid partway through T2. This matters if the FPGA memory controller samples write data on
  WR's falling edge instead of on the following clock edge.
- `tRHAV` (RD inactive → address valid for next cycle) = 85 ns min — the 80186 requires any
  directly-connected (unbuffered) device to float its output drivers within this window.

## READY / wait states

Two independent ready inputs, internally OR'd (either one satisfied terminates the wait):
- **SRDY** (synchronous): sampled at the start of every T3/Tw; must meet setup/hold to
  CLKOUT. Simpler to generate correctly in synchronous FPGA logic — **prefer SRDY over
  ARDY for an FPGA memory controller**, since the FPGA fabric is inherently synchronous and
  ARDY's asynchronous-resolution flip-flop chain (2-stage synchronizer, AP-186 Appendix B)
  is only needed for genuinely async external ready sources.
- **ARDY** (asynchronous): passes through a 2-stage synchronizer before being presented
  internally; only the active-going edge is synchronized.
- If unused, tie ARDY low and SRDY... (either can be tied to force "always ready"; consult
  chip-select ready-bit programming, see `04-integrated-peripherals.md`, for how many wait
  states are inserted vs whether external ready is even consulted).
- Any number of wait states (0 to ∞) may be inserted.
- Accesses to the integrated peripheral control block **always ignore external ready** and
  insert 0 wait states (1 wait state for timer registers specifically, due to
  counter-element multiplexing) — a fixed/known latency the FPGA design can hardcode for
  peripheral-register reads/writes.

## HALT bus cycle

Triggered by the HLT instruction. Differs from a normal bus cycle:
- RD/WR never asserted; no address/data driven.
- S0–S2 go passive (all high) during **T2** of the halt cycle — earlier than a normal cycle
  (which goes passive during T3/Tw preceding T4).
- ALE still pulses (can be used to latch "this was a HALT" from S0–S2 externally).
- Integrated peripherals (DMA, timers) keep running during HALT — a pending DMA transfer
  still executes bus cycles while the CPU is halted, and DMA latency actually *improves*
  during HALT (no CPU bus contention). Relevant if `cpu_top.sv`/`biu.sv` model HALT as a
  distinct FSM state that still services DMA-style memory_controller requests.
- An interrupt request or RESET brings the CPU out of HALT.

## HOLD / HLDA (bus arbitration)

Simple two-wire protocol (not 8086 max-mode's RQ/GT): external master asserts HOLD, CPU
finishes its current bus activity, asserts HLDA, floats AD0-15/A16-19/BHE/S7/DT-R/DEN/RD/
WR/S0-2/LOCK (**but not the chip-select lines UCS/LCS/MCSx/PCSx, and not ALE** — those never
float). When HOLD deasserts, CPU drops HLDA in a single clock and resumes.

- HOLD is internally synchronized (2 clocks minimum latency: 1 to synchronize, 1 to signal
  internal bus-hold logic), so it may be a fully asynchronous external signal.
- The CPU will not relinquish the bus mid-cycle — it always finishes the current bus cycle,
  including both halves of an odd-address word access and all cycles of a DMA transfer or a
  LOCKed transfer. Interrupt-acknowledge cycles (always run as a locked pair) are likewise
  never split by HOLD.
- Chip-select lines are **not floated** during HOLD — if an external bus master accesses the
  local bus during HOLD, discrete chip-select logic must be added externally; the 80186
  itself won't help.
- Practical implication for `biu.sv`: model bus-cycle atomicity as a first-class FSM
  invariant — HOLD (or an internal request to release the bus) must only be sampled/acted
  on at defined cycle-boundary states, never mid-T-state.

## Min mode vs max mode — does not apply to the 80186

Unlike the 8086 (which has a MN/MX strap pin selecting fundamentally different pinouts), the
80186 **always** simultaneously provides both local bus-controller outputs (RD, WR, ALE,
DEN, DT/R) *and* status outputs (S0–S2) — there is no mode pin, no loss of one signal set to
get the other. This actually simplifies the FPGA implementation versus a literal 8086 clone:
`biu.sv` never needs a min/max mode parameter — always drive both RD/WR/ALE-style local
control and S0-S2 status.

## Locked transfers (LOCK)

- Triggered by the LOCK instruction prefix. On the 80186 (unlike the 8086), LOCK does **not**
  go active immediately when the prefix is decoded — it activates at the start of T1 of the
  *first data cycle* of the locked instruction, and stays active until 3 T-states after the
  start of the last locked data cycle. This means opcode prefetching is never locked on the
  80186 (it was on the 8086).
- While LOCK is active, the CPU ignores external HOLD requests and internal DMA requests —
  neither may interrupt a locked sequence.

## Queue Status mode (optional, low priority for this project)

If RD is tied low externally during reset, the CPU repurposes ALE→QS0 and WR→QS1 as a
prefetch-queue activity monitor (used historically for the 8087 coprocessor to track queue
state). Not needed unless a coprocessor-style peripheral is added; RD has an internal
weak pull-up specifically to avoid falling into this mode by accident, so leaving RD
undriven-but-pulled during reset is safe.

## 8086 bus differences worth remembering (don't copy 8086 timing verbatim)

- 80186 ALE goes active a full clock phase earlier than 8086/8288 ALE (minimizes address
  latch propagation delay budget).
- 80186 clock has 50% duty cycle (from on-chip crystal ÷2); 8086 with 8284A has 33% duty
  cycle. If ever cross-referencing 8086 timing diagrams, note the differing clock shape.
- 80186 doesn't provide S3/S4/S5 (segment-source / interrupt-enable status) — always driven
  low. Don't build logic depending on these from an 8086 reference design.
