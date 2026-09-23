# FPGA80186

> Repository: **[badudum/80186CPU](https://github.com/badudum/80186CPU)** —
> `git clone git@github.com:badudum/80186CPU.git`
> (the Quartus project inside is named `FPGA80186`, which is what every
> `quartus_*` command below refers to.)

An Intel 80186 implemented in SystemVerilog for the **Terasic DE1-SoC**
(Cyclone V `5CSEMA5F31C6`), with the peripherals the chip integrated on-die and
enough of a PC/XT-shaped system around it to run MS-DOS.

The CPU is a full 80186 core — BIU/EU split, six-byte prefetch queue,
segment:offset addressing, the whole instruction set including string
operations with prefixes, `ENTER`/`LEAVE`/`BOUND`/`PUSHA`/`POPA`, BCD
adjustment, and hardware multiply and divide. Around it sit the 80186's own
integrated peripherals (interrupt controller, timers, DMA, chip-select unit,
peripheral control block) plus the parts a PC needs that the 80186 never had:
VGA text output, a PS/2 keyboard controller and an SDRAM controller.

**Status: it runs MS-DOS 6.22 on real hardware.** The board cold-boots its own
**BIOS**, which loads the **MS-DOS boot sector** off a 1.44 MB floppy image in
SDRAM, which loads `IO.SYS` and `MSDOS.SYS`, which load `COMMAND.COM` — all of
it Microsoft's code, unmodified, running on this core through this BIOS's
`INT 10h/13h/16h`:

```
FPGA80186 BIOS -- 640K, VGA text, PS/2 keyboard, block storage.
Booting from disk 0...
Boot sector loaded, starting.
Starting MS-DOS...

Current date is Tue 01/01/1980
Enter new date (mm-dd-yy):
```

Leave `AUTOEXEC.BAT` in place and you get the MS-DOS 6.22 Setup installer
instead, box-drawing characters and all, waiting at **ENTER=Continue**.

The project's own test system — a BIOS, a FAT12 volume and a `KERNEL.BIN`
loaded from it by name — still boots too, and is what the regression checks:

```
FPGA80186 BIOS -- 640K, VGA text, PS/2 keyboard, block storage.
Booting from disk 0...
Boot sector loaded, starting.
Loading KERNEL.BIN
Starting kernel.
KERNEL.BIN loaded from FAT12 and running.
hello
```

The last line is typed on a PS/2 keyboard. Lines 0–2 come from the ROM; lines
3–4 from the boot sector on the disk; line 5 from a *file* on the filesystem.

It synthesises, fits and meets timing at every corner: 14% of the ALMs, 8% of
the block RAM, no negative slack.

---

## Building

Everything is driven from the command line; no GUI step is required.

```sh
Q=~/intelFPGA_lite/18.1/quartus/bin
$Q/quartus_map FPGA80186 --part=5CSEMA5F31C6   # synthesis
$Q/quartus_fit FPGA80186                        # place and route
$Q/quartus_sta FPGA80186                        # timing analysis
$Q/quartus_asm FPGA80186                        # bitstream
```

The result is `output_files/FPGA80186.sof`. Program it with:

```sh
$Q/quartus_pgm -m jtag -o "p;output_files/FPGA80186.sof@2"
```

`@2` is the FPGA in the DE1-SoC's JTAG chain; the HPS is device 1.

### Results

| | |
|---|---|
| Logic | 4,048 ALMs / 32,070 — **13%** |
| Registers | 2,566 |
| Pins | 82 / 457 |
| Block memory | 29 M10K — **7%** |
| DSP | 2 (the multiplier) |
| **PLLs** | **0** |
| Worst slack | **positive at all four corners** |
| Tightest setup | `DRAM_CLK`, +2.969 ns |

That is the shipping configuration: disk in SDRAM sized for a 1.44 MB floppy,
JTAG loader and ISMCE both enabled. Block RAM drops to 7% because the disk
image no longer lives there.

The SDRAM interface is the tightest path, and both sides of it are packed into
the I/O cells (`FAST_INPUT_REGISTER` on `DRAM_DQ`, `FAST_OUTPUT_REGISTER` on
the address and command pins). Without that the fabric routing alone eats the
output window — it was the difference between +0.115 ns and +0.753 ns.

---

## Simulation

`sim/run.sh` drives the ModelSim ASE bundled with Quartus. It needs no
configuration and writes nothing into the working tree.

```sh
./sim/run.sh tb_bios      # the whole board booting the real BIOS
./sim/run.sh tb_alu       # a single module
```

There are **27 testbenches totalling 776 checks**, all passing, plus 71
assembler encoding tests, 38 filesystem tests and 15 disk-builder tests
(`python3 tools/test_asm86.py`, `tools/test_fat12.py`, `tools/test_mkdisk.py`):

| Testbench | Covers |
|---|---|
| `tb_alu` | arithmetic, logic, shifts, rotates, multiply, divide, flags |
| `tb_regfile` | word/byte register aliasing, segment registers |
| `tb_prefetch_queue` | queue fill, drain, flush |
| `tb_biu` | T-states, wait states, byte lanes, odd-address word splits |
| `tb_cpu` | core instruction execution |
| `tb_far` | far jumps and calls, segment loads |
| `tb_string` | `MOVS`/`CMPS`/`SCAS`/`LODS`/`STOS`, `REP` prefixes |
| `tb_misc` | `ENTER`/`LEAVE`/`BOUND`/`PUSHA`/`POPA`, BCD |
| `tb_interrupt` | interrupt entry, `IRET`, nesting |
| `tb_halt` | `HLT` wake rules: IF set, IF clear, NMI, return address |
| `tb_pic` | interrupt controller priority and masking |
| `tb_timer` | the three counters |
| `tb_keyboard` | PS/2 receive, scancode FIFO |
| `tb_iodecode` | the I/O bus contract, at **real BIU timing**, both byte lanes |
| `tb_crtc` | 6845 cursor registers |
| `tb_storage` | block device (ROM backend), and the FAT12 structures on it |
| `tb_storage_sdram` | the SDRAM backend, including writes, through the real stack |
| `tb_bios_sdramdisk` | the whole machine booting with the disk in SDRAM |
| `tb_vga` | sync timing, the character pipeline |
| `tb_sdram` | the controller against a protocol-checking model |
| `tb_sdram_arb` | arbitration: fairness, and one access per request |
| `tb_jtag_loader` | the image loader, including overflow detection |
| `tb_memsys` | region decode, VRAM dual port, ROM |
| `tb_dma` | both channels, bus arbitration against a running CPU |
| `tb_top` | the assembled board |
| `tb_bios` | **the real ROM image, the real font, SDRAM-backed RAM** |

`tb_bios` is the one that matters most: nothing in it is stubbed or forced. It
releases reset, waits for the machine to block in `INT 16h`, clocks five real
11-bit PS/2 frames in at the pins, and reads the result out of the text buffer.
A character arriving there has been through the keyboard FIFO, the interrupt
controller, the type-12 shim, scancode translation, the BIOS keyboard buffer,
`INT 16h` and `INT 10h`. It fails if the ROM image, the disk image, the font
image or the memory map is wrong — which the narrower tests happily tolerate.

### Booting someone else's operating system, in simulation

`tb_msdos` is not part of the regression and has no pass/fail: it boots a real
MS-DOS floppy image with a trace attached. It exists because the machine got as
far as `Starting MS-DOS...` on hardware and then executed an invalid opcode
inside the operating system, where nothing can be asked of it afterwards. It
needs an image that cannot live in this repository, so it is opt-in:

```sh
python3 -c "d=open('ms-dos/disk01.img','rb').read();
           open('ms-dos/msdos.hex','w').write(
               ''.join('%04x\n' % (d[i]|(d[i+1]<<8)) for i in range(0,len(d),2)))"
./sim/run.sh tb_msdos
```

It echoes every character the machine prints as it prints it, keeps a rolling
window of the last 512 retired instructions, and can dump a range of physical
memory to a file — which is how the bug below was found. The whole boot to the
fault is about five million cycles, a minute of wall time.

It found three things. The first two are below; the third needed different
tooling, because by then MS-DOS was running and simulation had become too slow
to reach the interesting part — see **Profiling the running machine**.

**`XCHG` with a memory operand never read memory.** The class was
missing from the table that says which instructions load their `r/m` operand, so
the store half worked and the register half handed back whatever the register
file read had left behind — a swap that only went one way. MS-DOS uses
`xchg ax, cs:[...]` to patch the far pointer it is about to call through, taking
the old value back as the address of the block it must copy first. It got
rubbish, copied from above the top of memory, and called into it. Every
existing `XCHG` test used the register-to-register form, which needs no memory
access at all and so could never see it.

**`INT 13h` AH=08 reported the last cylinder as zero**, describing a disk one
track long. Nothing had ever called it, so `tools/gen_disk.py` now has the test
kernel ask for the geometry and leave the answer at `0000:0600` for the
testbench to check.

### Profiling the running machine

`tools/jtag_prof.tcl` reads sixteen `CS:IP` samples out of a ring that
`modules/jtag_loader.sv` fills **in hardware**, one every 2^20 clocks. The CPU
is not disturbed and the guest cannot influence it.

Sampling in the BIOS's timer interrupt is the obvious approach and it does not
work. MS-DOS hooks `INT 08h` and chains to the previous handler, so the return
address the BIOS sees is MS-DOS's own chaining call — every single time. The
profile comes back perfectly consistent and is a picture of the profiler. The
hardware ring reads the CPU's `CS:IP` straight off the register file instead.

That is what found the last hang: sixteen samples, all inside a fourteen-byte
loop, which disassembled to

```
        in   al, 61h        ; the PC's system control port
        and  al, 10h        ; bit 4, the DRAM refresh toggle
        cmp  al, ah
        jz   $-6            ; spin until it changes
```

PC software calibrates delay loops against that bit, in loops with **no
timeout**, because on real hardware it cannot fail to change. We had no port
61h, an undecoded port reads a constant, and MS-DOS hung there silently and
permanently. `modules/io_decode.sv` now implements it: bit 4 toggles every
15.085 us as it does on a real PC, and bits 1:0 read back because code that
writes them reads first and puts them back.

`tb_iodecode` earns its place for the opposite reason. `tb_keyboard` drives the
register interface with `rd` held for a single clock; the BIU holds it for the
whole of T2 and T3. That gap hid a real defect — a level-sensitive FIFO pop
retired **two** scancodes per `IN AL,60h`, and since the BIU latches data at the
*end* of T3 while the first pop happened at the end of T2, the byte the CPU got
was the one *after* the one it asked for. Typing would have produced garbage on
hardware with every module test passing. `tb_iodecode` models the bus cycle
instead of the register interface, and asserts the invariant that fixes it: a
device strobe fires only in a cycle where `ready` is high.

---

## Clocking — and why 25 MHz

There is **no PLL**. `CLOCK_50` enters the chip and `clk_rst` divides it by
two; that 25 MHz clock runs the CPU, the SDRAM and the video output alike.

The CPU core itself closes timing at 50 MHz comfortably. What does not is the
**SDRAM read capture**. `DRAM_CLK` is the inverted system clock driven out
through the fabric to a pin, so it reaches the memory about 4.5 ns after the
internal clock edge; the chip then takes a further tAC (5.4 ns) to drive read
data back. At a 20 ns period that put the data 2.4 ns *past* the capture edge
— a genuine violation at all four timing corners, not a pessimistic model.

A PLL would fix it by phase-shifting `DRAM_CLK` to cancel the output delay,
which is exactly what Terasic's own SDRAM reference design does. Without one,
halving the clock is the clean answer: at 40 ns the same edge lands deep inside
the data valid window, with no change to the memory controller at all. The cost
is cheap — a real 80186 ran at 6–12.5 MHz, so 25 MHz is still two to four times
the original part.

25 MHz also happens to be what the video wants: nominal for 640x480 @ 60 Hz is
25.175 MHz, so 25.000 is 0.7% slow and refresh lands near 59.5 Hz, well inside
what any monitor tolerates.

**This was invisible until the SDRAM interface was constrained.** With no
`set_input_delay` on `DRAM_DQ` the analyser reported no margin at all for that
path and the build looked clean while the read capture was in fact missing its
edge. `FPGA80186.sdc` now constrains it.

---

## The BIOS

`tools/gen_bios.py` builds a 16 KB ROM using `tools/asm86.py`, a small 16-bit
x86 assembler written for the job. Hand-encoding opcodes and patching jump
displacements by hand cost real debugging time early on — the failure mode is a
branch landing one byte into an instruction, after which the CPU faithfully
executes an immediate as an opcode. Every reference now goes through a fixup
table that raises on an unresolved or out-of-range target, and the encodings
are covered by `tools/test_asm86.py`.

| Service | Functions |
|---|---|
| `INT 10h` | 00 set mode, 02/03 cursor, 06 scroll, 09 write char, 0E teletype, 0F get mode |
| `INT 13h` | 00 reset, 02 read sectors (CHS→LBA), 08 get parameters |
| `INT 16h` | 00 read key (blocking), 01 peek, 02 shift state |
| `INT 11h` / `INT 12h` | equipment word, memory size |
| `INT 1Ah` | 00 read tick count |
| `INT 08h` | timer tick, chains to `INT 1Ch` |
| `INT 09h` | keyboard: set 2 → ASCII into the buffer at 40:1E |
| `INT 19h` | bootstrap: load sector 0 to 0000:7C00, check AA55, jump |

Two places it deviates from a PC, both forced by the hardware:

- **The 80186's interrupt vectors are fixed.** Timer 0 arrives as type 8, which
  happens to be the PC's tick vector. The keyboard arrives on INT0 as **type
  12, not type 9** — so the type-12 handler is a shim that issues `INT 09h` and
  then signals end-of-interrupt, leaving anything that hooks `INT 09h` working.
- **PS/2 keyboards send scancode set 2**, PC software expects set 1.
  Translation to ASCII happens in the type-9 handler.

The BIOS also depends on `HLT` waking on an interrupt: `INT 16h` parks the CPU
in `STI; HLT` rather than spinning, so it is not fighting the very handler it
is waiting for.

### What a real guest expects of INT 13h

Booting someone else's operating system is a much harder test of a BIOS than
booting your own, because your own boot code is written against whatever the
BIOS happens to do. Two habits that our boot chain had, and MS-DOS does not:

- **The caller's registers come back.** `BX` is an *input* to `INT 13h` — the
  offset half of the `ES:BX` buffer — and the handler has to return it intact.
  Ours borrowed `BX` to edit the stacked `FLAGS` word on the way out, so it
  returned the flags where the buffer pointer should have been. The MS-DOS boot
  sector's next instruction is `mov di,bx`, so it compared the root directory
  against a garbage address and announced a non-system disk. Our own boot code
  saved `BX` around the call and never noticed.
- **A word `MUL` writes `DX:AX`.** The CHS-to-LBA conversion read the head out
  of `DH` *after* multiplying the cylinder, so the head was the high half of
  that product — zero for everything on cylinder 0. Every read on head 1
  silently returned a head-0 sector.

Neither showed up in simulation, because the test disk's files all sat on
cylinder 0 head 0 and its boot code saved `BX` itself. `tools/gen_disk.py` now
pads the image so `KERNEL.BIN` straddles a head *and* a cylinder boundary, and
the boot sector deliberately does **not** save `BX` across `INT 13h`, so both
paths are exercised on every run.

### Finding a fault inside a guest

`INT 06h` writes the whole machine state to `0040:0090` before it halts — every
register, `CS:IP`, `FLAGS`, `SS:SP` and eight words of the faulting code's own
stack — so it survives the halt and can be read back with
`tools/jtag_peek.tcl 0x000490`. An unimplemented instruction inside an
operating system is otherwise nearly impossible to place: the message says it
happened, nothing says where, and the machine is stopped so nothing can be
asked afterwards.

## The filesystem

The disk is a real FAT12 volume, not a blob with a known layout — a BIOS
parameter block, a FAT with 12-bit packed entries, a 112-entry root directory
of 32-byte records, and a data area addressed in clusters. `tools/fat12.py`
builds it; `tools/gen_disk.py` puts a boot sector and files in it.

The **boot sector is a real FAT12 loader**: it reads the BPB at runtime, loads
the FAT and root directory through `INT 13h`, searches for `KERNEL.BIN` by
name, walks its cluster chain and loads it at `1000:0000`. Nothing about the
layout is compiled in — resize the volume, move the file, or change the
geometry and the same boot sector still finds it.

That matters for more than tidiness. There is no `mtools` or `dosfstools` on
this machine to validate the image against, so the strongest check available is
**two independent implementations agreeing**: a Python builder writes the
structures, and 8086 assembly running on the CPU under test reads them back.
`tools/test_fat12.py` adds a third, a reader written from the on-disk format
that takes nothing but the image bytes.

`KERNEL.BIN` is padded to span **five clusters** on purpose. A file that fits
in one cluster never makes the boot sector follow a chain, so the 12-bit FAT
unpacking — where two entries share a byte and odd ones take the high nibble —
would go untested in hardware. The kernel checks its own last word before
announcing anything, so "it ran" means "the whole file arrived, in order".
Corrupting one FAT entry makes it say so instead.

## Sharing the SDRAM

`modules/sdram_arbiter.sv` sits between the memory requesters and the single
SDRAM controller port. The CPU uses it today; the block device and a JTAG
loader will take the other two ports once the disk image moves out of on-chip
ROM and into SDRAM, where it can be larger than block RAM allows.

Two details of the controller shaped it, both found by reading the controller
rather than assuming:

- **It re-accepts a held request.** `S_IDLE` starts an access whenever `rd` or
  `wr` is high — it does *not* require the request to fall in between. A
  requester holding its line one cycle too long would get a second, unasked-for
  access: a double write, or a stray read. So a requester is marked `served`
  when its ready fires and becomes eligible again only once it lets go.
- **Priority rotates.** The CPU is not a polite requester — while waiting on
  the disk it runs a polling loop that fetches from the very memory the disk is
  trying to use. Rotation bounds everyone's wait at one turn per other
  requester.

The controller's address widened from 20 bits to 24 at the same time.
Conventional memory only needs 1 MB, but the disk image is meant to live
*above* it — out of reach of any 8086 address, visible only to the block
device. 16 MB of the chip's 64 still fits inside bank 0.

## Graphics: mode 13h

320x200 at eight bits per pixel, the mode every DOS game of the era used.
`INT 10h` with `AX=0013h` switches to it; anything else switches back to text.

Three pieces:

- **`modules/framebuffer.sv`** — 64 KB of on-chip dual-port RAM at `A0000`.
  On-chip is the whole point. Scanning 320x200 out at 60 Hz needs 3.84 MB/s,
  which is the same order as everything the SDRAM controller can deliver even
  with the open-row policy below. Display alone would crowd out the CPU. Here
  scan-out costs the rest of the machine nothing: it is a second port on a
  block RAM nobody else touches. The price is 50 of 397 M10K blocks.
- **`modules/vga_dac.sv`** — the 256-entry palette, written through ports
  `3C8`/`3C9` exactly as software expects: the index once, then red, green and
  blue, with the index advancing on its own so a whole palette is 768 writes.
  Components are six bits, 0-63, as the real DAC takes them; the top two bits
  are replicated into the bottom so 63 maps to 255 rather than 252.
- **`vga_controller`** grows a second scan-out path. Each pixel is displayed
  twice horizontally and twice vertically, which is how a real VGA fits 200
  lines into a 400-line raster, and lands the image in the same 400-line window
  the text mode already uses — so the vertical centring and sync timing are
  shared rather than duplicated.

The doubling is the part worth testing: an off-by-one there produces a picture
that looks entirely plausible and is wrong everywhere. `tb_vga` checks the
first and last pixel of a row, both halves of a doubled pixel and both raster
lines of a doubled row, and `tb_bios` has the loaded kernel set the mode and
plot through the aperture so the decode, byte lanes and DAC are covered from
software down.

### The .hex and .mif must not drift

With `ISMCE` set, `bios_rom` **synthesises from `rom/bios.lo.mif`** while every
simulation reads `rom/bios.lo.hex`. Regenerating only the `.hex` leaves the
bitstream running the previous BIOS while every test agrees the new one works,
and nothing in the build hints at it. That is not hypothetical: it cost an
afternoon here, with the board booting far enough to print six lines and then
quietly not switching video mode, because the ROM in the bitstream predated the
mode-13h code. `tools/gen_bios.py` now writes both files together, so they
cannot get out of step.

## The blitter

`modules/blitter.sv` fills and copies rectangles in the framebuffer without the
CPU touching a pixel. Three operations — fill, copy, and copy skipping a
transparent key — through eight registers at I/O `0330`–`033F`, with a busy bit
to poll. **Fill runs at one clock per pixel and copy at two**, against a CPU
that needs roughly eight, and it costs 190 ALMs.

It is deliberately not a GPU: nothing is programmable. That is what the era's
hardware did and it is what 2D inner loops actually need — clearing a screen,
drawing a sprite, scrolling a window are all a fill or a copy.

**It shares the CPU's framebuffer port and the CPU always wins.** The block RAM
has two ports and both were already taken (CPU on one, video scan-out on the
other), so `memory_controller` hands the port to the CPU for any cycle it wants
the aperture and `stall` holds the blitter still. In practice a program that
has started a blit is polling the busy bit rather than writing pixels — but
that is a habit, not a guarantee.

Strides are separate from width on purpose: copying a 32×32 sprite out of a
320-wide screen needs width 32 with both steps 320, while a sprite packed in
its own bitmap needs a source step of 32. One number cannot express both, and
getting it wrong shears the image diagonally.

`tb_blitter` runs against the real `framebuffer`, uses odd offsets so the byte
lanes have to be right, and models the shared port the way `memory_controller`
wires it — the CPU genuinely takes the port during a stall. That last part
matters: with the blitter still connected during a stall, a blitter that
ignored arbitration writes the same pixel twice to the same address and no test
notices.

## The SDRAM controller keeps the row open

A row is 1024 columns of 16 bits -- 2 KB -- and it stays ACTIVE after an
access rather than being closed by auto-precharge. A second access to the same
row then costs only the column command and the CAS latency, skipping
ACTIVATE, tRCD and tRP:

| | miss | page hit |
|---|---|---|
| read | 10 clocks | **5** |
| write | 10 clocks | **3** |

Instruction fetch, stack traffic and block copies all walk consecutive
addresses, so most accesses hit. Measured on the sequential case that matters
most, reading disk sectors out of SDRAM, it is **1.77x faster** -- 42,194
clocks down to 23,781 for the same work.

The whole-machine figure is much smaller: booting to the keyboard wait went
from 3,451,369 clocks to 3,285,789, about **5%**. That is not a disappointment,
it is where the time goes. The BIOS executes from on-chip ROM and writes to an
on-chip text buffer, so most of that boot never touches SDRAM at all. The gain
lands on code running from conventional memory -- which is to say on DOS and
everything above it.

Two things to know:

- **The row is closed only when it must be**: a different row, or a refresh,
  which requires every bank precharged. A miss costs exactly what the old
  unconditional auto-precharge cost, so the floor is unchanged.
- **Interleaving two streams in different rows makes every access a miss**, so
  the arbiter's rotating priority can thrash the row when the CPU and the
  block device run together. Per-bank row tracking would fix that; `bank` is
  currently hardwired to zero.

Correctness tests pass whether or not the row stays open, so `tb_sdram`
measures the cost of a hit and a miss and fails if a hit is not at least three
clocks cheaper. Without that the optimisation could quietly stop working.

## Two disk backends

`storage.sv` takes `USE_SDRAM`, selected at the top level by `DISK_IN_SDRAM`:

| | on-chip ROM (default) | SDRAM |
|---|---|---|
| survives power-up | yes, it is in the bitstream | no, must be loaded |
| size ceiling | ~700 sectors (block RAM) | any |
| writable | no | **yes** |
| block RAM used | 157 blocks (**40%**) | 29 blocks (**7%**) |

The register interface is identical either way, so the BIOS, `INT 13h` and the
FAT12 boot sector cannot tell which is underneath — `sim/tb_bios_sdramdisk.sv`
boots the identical image both ways and checks the same seven lines appear.

With the disk in SDRAM a **1.44 MB floppy costs nothing in block RAM**, which
is what makes a real DOS image possible. The catch is volatility: nothing boots
until a loader has put an image there, so ROM stays the default until the JTAG
loader exists.

A command copies a whole sector into a buffer and the data port serves from
there, rather than reading the backend per access — SDRAM latency inside a bus
cycle `io_decode` expects to answer immediately would need wait states threaded
back to the BIU. It also makes `BUSY` real rather than a simulated delay.

## Loading things over JTAG

Two mechanisms, both over the USB-Blaster you already program with, neither
needing a rebuild.

**The disk image, into SDRAM** — `modules/jtag_loader.sv` plus
`tools/jtag_load.tcl`:

```sh
quartus_pgm -m jtag -o "p;output_files/FPGA80186.sof@2"
quartus_stp -t tools/jtag_load.tcl ms-dos/disk01.img
```

A virtual JTAG node with four instructions: set the pointer, stream words, hold
or release the CPU, read status. Words are streamed **thousands per DR scan** —
a scan per word would be correct and unusably slow, since 1.44 MB is 737,280 of
them. The CPU is held in reset for the load and released at the end, which
restarts the machine on the image just written.

JTAG cannot be stalled mid-scan, so if TCK outruns the SDRAM writer a word is
dropped. The loader latches that as an **overflow** bit and counts what it
actually wrote; the script reads both back and fails loudly rather than leaving
a silently corrupt disk. `sim/tb_jtag_loader.sv` deliberately runs TCK far too
fast to prove the detection works.

Words are framed by a **sync pattern**, not by counting bits from the start of
the scan. A virtual JTAG node does not see the host's first bit on the scan's
first shift clock: the SLD hub and the bypass register of every other SLD node
in the chain shift junk in ahead of it. A node that latches at update-DR never
notices — the junk is pushed out the far end — but a streaming node that
consumes words as they arrive has every word boundary displaced by the width of
that lead-in, and the width depends on what else is in the chain. Enabling the
memory editor added five nodes and rotated the entire image by seven bits, with
the word count still reading exactly right. So the host prefixes each scan with
sixteen zero bits and a SYNC word and the loader frames from there, and the
script now **reads words back and compares them against the file** rather than
trusting a count.

**The BIOS and font ROMs, in place** — `ISMCE=1` plus `tools/ismce_load.tcl`:

```sh
python3 tools/gen_bios.py rom/
python3 tools/hex2mif.py rom/bios.lo.hex rom/bios.lo.mif 8
quartus_stp -t tools/ismce_load.tcl BIOL rom/bios.lo.mif
```

This matters most for the BIOS: editing one line otherwise costs a full
synthesis, fit and timing run. The In-System Memory Content Editor reaches
**on-chip memory only**, so it cannot touch the disk image in SDRAM — the two
mechanisms cover different memories rather than competing.

Enabling it instantiates `altera_syncram` with `enable_runtime_mod` instead of
letting Quartus infer the RAM. Simulation still uses the inferred path and the
`.hex` images, so the sim flow keeps working without vendor libraries — but it
does mean the synthesised memories are not the ones simulated, which is the one
real cost. The two paths must therefore agree on **read latency**: the
instantiated memories are `UNREGISTERED` on the output, giving the same single
cycle the inferred `always_ff` read has. With the output register enabled they
take two, which no simulation could show — on hardware it shifted the whole
screen right by one character and printed the glyph fetched during blanking at
the start of every line.

## Booting MS-DOS 6.22

```sh
quartus_pgm -m jtag -o "p;output_files/FPGA80186.sof@2"
jtagconfig --setparam "DE-SoC [4-2]" JtagClock 6M
quartus_stp -t tools/jtag_load.tcl ms-dos/disk01.img
```

That boots the real thing: the MS-DOS boot sector, `IO.SYS`, `MSDOS.SYS`,
`COMMAND.COM`, `AUTOEXEC.BAT`, and — because Disk 1 of a 6.22 set is the Setup
disk — `SETUP.EXE`, which paints its full-screen installer and waits at
**ENTER=Continue** for a PS/2 keyboard.

Setup will not get far: it installs to a hard disk and this machine has one
floppy. Renaming `AUTOEXEC.BAT` in the image before loading it gives a bare
`A:\>` prompt instead.

The image is loaded into SDRAM over the USB-Blaster and is **not** part of the
bitstream; MS-DOS is Microsoft's, so nothing from `ms-dos/` is committed here.
Lower the JTAG clock as shown or the loader outruns the SDRAM writer — it will
tell you if it does, and the readback check will fail rather than leaving a
quietly corrupt disk.

## Disks bigger than a floppy

`tools/mkdisk.py` builds a large bootable image from a floppy plus whatever
else you want on it:

```sh
python3 tools/mkdisk.py ms-dos/disk01.img big.img --size 16 GAME.EXE DATA.WAD
```

The disk lives in SDRAM and `storage.sv` takes any sector count, so 1.44 MB was
never a hardware limit — it was the size of the image being loaded. Growing a
FAT volume means rebuilding its BPB, FAT and root directory and relocating
every file, which is what the tool does.

**Sixteen megabytes is the ceiling**, and not arbitrarily: `storage.sv` forms
its address as `BASE + {lba[14:0], 9'b0}`, and fifteen bits of sector number
reaches exactly 32,768 sectors.

Three things have to agree or the machine half-reads the disk:

1. **The BIOS's geometry.** `INT 13h` converts CHS to LBA using the
   sectors-per-track and heads it was *built* with, so a ROM built for a floppy
   reads the wrong sectors. `tools/img2hex.py --sdram` writes `rom/geometry.py`
   from the image and `gen_bios.py` picks it up.
2. **`DISK_SECTORS`**, which is a synthesis parameter and so needs a rebuild.
3. **The cylinder count.** This BIOS keeps the cylinder in `CH` alone — a real
   BIOS puts bits 8–9 in `CL[7:6]` and we ignore them — so no more than 256
   cylinders. 16 MB at floppy geometry would need 909; at 63 sectors × 16 heads
   it needs 32. The tool refuses geometry that would overflow rather than
   producing an image that silently reads the wrong tracks.

Two details that produce a disk which looks perfect and does not boot, both
handled: **`IO.SYS` must be the first root entry and `MSDOS.SYS` the second**,
because the MS-DOS boot sector compares those two slots by position rather than
searching; and **FAT12 has only 4084 usable cluster numbers**, so past about
2 MB the clusters have to grow rather than multiply.

**MS-DOS does not always believe the BPB.** For a floppy it picks a device
parameter table from the media descriptor and the format it recognises, and
uses that instead — so two fields have to match what DOS expects or it computes
a different data area from the one the volume has, reads the wrong clusters,
and reports `Bad or missing Command Interpreter`. That happens *after* booting
perfectly, because the boot sector and `IO.SYS` do read the BPB. Both are
measured, not guessed:

- **The media descriptor.** `F0` is a floppy, `F8` a fixed disk. A 16 MB volume
  built with `F8` got as far as `IO.SYS` and then jumped into zeroed memory.
- **The root directory size.** The same 2880-sector image, same geometry, same
  media, boots COMMAND.COM with 224 root entries and fails with 512. Nothing
  else changed.

Both now default to the source image's values, which is why the tool takes a
floppy to copy from rather than building a volume from nothing.

**Verified on hardware:** a 16 MB image holding all 45 files from the MS-DOS
floppy plus a 693 KB executable and an 11 MB data file boots MS-DOS 6.22 to its
prompt, with 3.1 MB free.

## Putting your own disk image on it

```sh
python3 tools/img2hex.py yourdisk.img     # -> rom/disk.hex + rom/geometry.py
# set SECTORS in modules/storage.sv to the number it reports
python3 tools/gen_bios.py rom/            # picks up the geometry
quartus_map / fit / sta / asm             # rebuild the bitstream
```

The image is **baked into the bitstream**, not uploaded at runtime — the disk
is on-chip ROM, so changing its contents means rebuilding. Three things the
tool checks, because each of them otherwise fails in a way that looks like
something else:

- **Size.** Each 512-byte sector costs 4096 bits of block RAM and the device
  has about 4 Mbit in total, so the ceiling is roughly 700–900 sectors. A
  **360 KB floppy fits**; a **720 KB or 1.44 MB floppy does not**, by up to 4x.
  Getting past that means backing the disk with the board's 64 MB SDRAM instead
  of block RAM, which is a hardware change rather than a tooling one.
- **Geometry.** `INT 13h` speaks cylinder/head/sector and the BIOS converts it
  to a flat sector number using compiled-in constants. A 1.44 MB image declares
  18 sectors per track and 2 heads; a 360 KB one declares 9 and 2; this project
  defaults to 16 and 4. If they disagree, every read lands on the wrong sector
  and it looks like a corrupt filesystem. `img2hex.py` reads the image's BPB and
  writes `rom/geometry.py`, which `gen_bios.py` then builds against.
- **An `.iso` is not a floppy image.** A CD image is ISO 9660 with 2048-byte
  sectors; neither the boot path nor a DOS boot sector can read one. A bootable
  CD carries a floppy image inside it (El Torito) — that is the part to extract.

## Memory and I/O map

The standard PC/XT layout, which is what MS-DOS expects:

| Range | Contents | Backed by |
|---|---|---|
| `00000`–`9FFFF` | conventional RAM, 640 KB | SDRAM |
| `A0000`–`AFFFF` | graphics aperture, mode 13h | on-chip dual-port RAM |
| `B8000`–`B8FFF` | colour text buffer, 80x25 | on-chip dual-port RAM |
| `C0000`–`EFFFF` | option ROM / extended BIOS | unmapped |
| `F0000`–`FFFFF` | BIOS ROM, reset vector at `FFFF0` | on-chip ROM |

The whole 1 MB lives in bank 0 of the 64 MB SDRAM, which is why `DRAM_BA` and
the upper `DRAM_ADDR` bits are constant — that is deliberate, not a bug.

I/O space:

| Port | Device |
|---|---|
| `0060`, `0064` | PS/2 keyboard data and status |
| `0061` | system control port: bit 4 is the DRAM refresh toggle |
| `03C8`, `03C9` | VGA palette DAC: index, then red/green/blue |
| `03D8` | video mode, CGA-style; bit 1 selects graphics |
| `0320`–`032F` | block storage (PC/XT hard-disk range) |
| `03D4`, `03D5` | 6845 CRTC — cursor position and visibility |
| `FF00`–`FFFF` | 80186 Peripheral Control Block (relocatable) |

The PCB holds the interrupt controller, timers, DMA channels and chip-select
registers at their architectural offsets.

Port `0061` is the odd one out: it is not a device, just the two bits PC
software expects to find there. Bit 4 toggles every 15.085 us as the DRAM
refresh signal does on a real PC, which is what timing-calibration loops watch,
and bits 1:0 are the speaker gate and data, which nothing here drives but which
read back because code that writes them reads first and puts them back. It
exists because such loops have no timeout — on real hardware the bit cannot
fail to change — so without it MS-DOS hangs there permanently.

---

## Layout

```
modules/      the RTL; FPGA80186.sv is the top level
sim/          testbenches and run.sh
tools/        asm86.py (assembler), fat12.py (filesystem),
              gen_bios.py, gen_disk.py, gen_font.py, gen_pins.py,
              img2hex.py, img2mif.py, hex2mif.py, test_asm86.py,
              test_fat12.py, jtag_load.tcl, ismce_load.tcl
rom/          generated ROM images (tracked, so builds are reproducible)
learnings/    notes taken from the 80186 datasheets and AP-186
docs/         the source PDFs
```

`tools/gen_pins.py` derives the 82 pin assignments from the DE1-SoC manual
rather than transcribing them, and refuses to emit a partial map: it checks for
conflicting pins, ports with no assignment, and the same pin used twice.

`tools/gen_bios.py` is a small two-pass assembler that produces the boot ROM.

---

## Known limitations

- **The disk is volatile.** With `DISK_IN_SDRAM` the image lives in SDRAM and
  is pushed in over the USB-Blaster after every power-up, so anything DOS
  writes is lost on reload or power-off. The on-chip ROM backend survives
  power-up but is read-only and caps out around 700 sectors — far short of a
  1.44 MB floppy. The DE1-SoC's microSD socket is wired to the **HPS, not the
  FPGA fabric** (manual Table 3-28: `HPS_SD_CLK`, `HPS_SD_CMD`,
  `HPS_SD_DATA[3:0]` on pins A16/F18/G18/C17/D17/B16), so fabric logic cannot
  reach the card without running software on the ARM or adding a GPIO
  breakout. The register interface is the shape of a real block device, so an
  SD or GPIO backend can replace it without the software above changing.
- **The BIOS is a useful subset, not a complete one.** `INT 13h` has no write
  function, there is one video page and one video mode, no `INT 15h`, and
  extended keys (the `E0` prefix) are consumed but not mapped. Enough for
  MS-DOS to boot and for its Setup program to run; not enough for arbitrary
  DOS software.
- **The filesystem is read-only in practice.** `fat12.py` can create a volume
  and add files, and the boot sector can read one, but `INT 13h` has no write
  function, so nothing changes the disk at runtime.
- **No PC-compatible peripherals beyond the essentials.** There is no 8259 at
  20h/21h, no 8253 at 40h-43h and no 8237 at 00h-0Fh — this design uses the
  80186's own integrated equivalents, at the 80186's own addresses. Port 61h
  exists only far enough to keep timing-calibration loops running. Software
  that programs those chips directly will not work.
- **One graphics mode, and it is not VGA-register compatible.** Mode 13h
  works, set through `INT 10h` as every DOS program actually does it. Real
  VGA mode setting programs a dozen sequencer, CRTC and graphics-controller
  registers and none of those exist here, so software that pokes them
  directly will not work. There are no other graphics modes.
- **Font licensing.** `rom/font.hex` is extracted from the cp850-8x16 console
  font in the Linux `kbd` package, which is GPL-2.0. Check that before
  redistributing a bitstream. The upper half is CP850, not CP437, so the
  box-drawing glyphs differ from a real PC.
- **VGA outputs are false-pathed** rather than constrained against the ADV7123
  DAC's setup/hold window. Acceptable for bring-up; revisit before trusting the
  picture at higher pixel clocks.

### On-chip memory is inferred, not instantiated IP

The on-chip memories (boot ROM x2 banks, font ROM, text buffer, keyboard FIFO,
and the disk image when it is ROM-backed) are written as plain SystemVerilog
arrays and left to Quartus. `ISMCE=1` is the exception: it instantiates
`altera_syncram` explicitly, because the in-system memory editor needs a named
instance to attach to — see **Loading things over JTAG**. That is deliberate, and the synthesis log shows it costs nothing:
inference produces `altsyncram` megafunctions — the *same* primitive the IP
catalog would instantiate — and `$readmemh` is converted into `.mif` files that
are baked into the bitstream (`Parameter INIT_FILE set to
db/FPGA80186.ram0_bios_rom_*.hdl.mif`).

Explicit IP would add generated files to the repo and break `sim/run.sh`, which
runs the bundled ModelSim with no vendor libraries compiled. Same reasoning as
the deliberate absence of a PLL.

The one place inference needed a hint is the keyboard FIFO: 8 x 8 bits is far
too small for an M10K, and Quartus was spending a whole 10 Kbit block on it
plus pass-through logic to emulate read-during-write. `ramstyle = "logic"`
puts it in flops where it belongs.

### Benign build warnings

Three synthesis warnings are expected and understood:

- `interrupt_controller.sv:196` — `irr_latched` has five bits that are reset to
  zero and never written. They exist so the vector's bit positions line up with
  the source numbering the PCB request register uses.
- `decode.sv:227` — the `casez` overlap is intentional: `90h` is listed as `NOP`
  before the `XCHG AX,reg` pattern that also covers it, and `casez` takes the
  first match.
- Pins "stuck at GND/VCC" — `VGA_SYNC_N` is tied low (sync-on-green unused),
  `DRAM_CKE` tied high, and the unused SDRAM bank/row bits as described above.
