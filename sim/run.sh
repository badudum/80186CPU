#!/usr/bin/env bash
# Run a testbench against the FPGA80186 RTL using the ModelSim ASE bundled
# with Quartus.
#
#   ./sim/run.sh              # runs tb_alu
#   ./sim/run.sh tb_alu       # same, explicitly
#
# Two quirks of the bundled ModelSim are worked around here, neither of which
# requires modifying the Quartus install or installing system packages:
#   1. The `bin/` launcher scripts look for a `linux_rh60` directory that the
#      Lite edition does not ship. The real binaries are in `linuxaloem`, so we
#      set MODEL_TECH and call them directly.
#   2. Those binaries are 32-bit and link against the old `libncurses.so.5`
#      SONAME. Arch ships ncurses 6, so we symlink the 32-bit .so.6 under the
#      name ModelSim wants (lib32-ncurses must be installed, which it is).
#
# All build output goes to a scratch directory outside the repo so the working
# tree stays clean.

set -euo pipefail

TB="${1:-tb_alu}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Per-testbench, so two runs at once cannot delete each other's
# compiled library out from under them -- which looks exactly like a
# testbench failing at random.
BUILD="${TMPDIR:-/tmp}/fpga80186_sim/$TB"

# --- locate ModelSim ------------------------------------------------------
MODEL_TECH=""
for base in "$HOME"/intelFPGA_lite/*/modelsim_ase "$HOME"/intelFPGA/*/modelsim_ase; do
    if [ -x "$base/linuxaloem/vsim" ]; then
        MODEL_TECH="$base/linuxaloem"
        break
    fi
done
if [ -z "$MODEL_TECH" ]; then
    echo "error: could not find modelsim_ase under ~/intelFPGA_lite" >&2
    echo "       (looked for */modelsim_ase/linuxaloem/vsim)" >&2
    exit 1
fi
export MODEL_TECH

# --- ncurses .so.5 shim ---------------------------------------------------
SHIM="$BUILD/libshim"
mkdir -p "$SHIM"
for cand in /usr/lib32/libncursesw.so.6 /usr/lib32/libncurses.so.6; do
    [ -f "$cand" ] && ln -sf "$cand" "$SHIM/libncurses.so.5" && break
done
if [ ! -e "$SHIM/libncurses.so.5" ]; then
    echo "error: no 32-bit ncurses found in /usr/lib32" >&2
    echo "       install lib32-ncurses (multilib) and retry" >&2
    exit 1
fi
[ -f /usr/lib32/libtinfo.so.6 ] && ln -sf /usr/lib32/libtinfo.so.6 "$SHIM/libtinfo.so.5"
export LD_LIBRARY_PATH="$SHIM:${LD_LIBRARY_PATH:-}"

# --- which RTL does this testbench need? ----------------------------------
# Kept explicit rather than globbing modules/*.sv: most modules are still
# stubs with undriven outputs, and compiling them all just adds noise.
# The package must compile before anything that references it.
PKG="$REPO/modules/cpu_pkg.sv"

case "$TB" in
    tb_alu)            RTL=("$PKG" "$REPO/modules/ALU.sv") ;;
    tb_regfile)        RTL=("$PKG" "$REPO/modules/regfile.sv") ;;
    tb_prefetch_queue) RTL=("$PKG" "$REPO/modules/prefetch_queue.sv") ;;
    tb_keyboard)       RTL=("$PKG" "$REPO/modules/keyboard_controller.sv") ;;
    tb_iodecode)       RTL=("$PKG" "$REPO/modules/io_decode.sv"
                            "$REPO/modules/keyboard_controller.sv"
                            "$REPO/modules/storage.sv"
                            "$REPO/modules/crtc.sv") ;;
    tb_crtc)           RTL=("$PKG" "$REPO/modules/crtc.sv") ;;
    tb_sdram_arb)      RTL=("$PKG" "$REPO/modules/sdram_arbiter.sv") ;;
    tb_jtag_loader)    RTL=("$PKG" "$REPO/modules/jtag_loader.sv"
                            "$REPO/modules/sdram_arbiter.sv"
                            "$REPO/modules/sdram_controller.sv"
                            "$REPO/sim/sdram_model.sv") ;;
    tb_storage_sdram)  RTL=("$PKG" "$REPO/modules/storage.sv"
                            "$REPO/modules/sdram_arbiter.sv"
                            "$REPO/modules/sdram_controller.sv"
                            "$REPO/sim/sdram_model.sv") ;;
    tb_storage)        RTL=("$PKG" "$REPO/modules/storage.sv") ;;
    tb_vga)            RTL=("$PKG" "$REPO/modules/vga_controller.sv" "$REPO/modules/font_rom.sv" "$REPO/modules/vga_dac.sv") ;;
    tb_sdram)          RTL=("$PKG" "$REPO/modules/sdram_controller.sv" "$REPO/sim/sdram_model.sv") ;;
    tb_top|tb_bios|tb_bios_sdramdisk|tb_msdos)    RTL=("$PKG"
                            "$REPO/modules/ALU.sv"
                            "$REPO/modules/regfile.sv"
                            "$REPO/modules/decode.sv"
                            "$REPO/modules/microcode.sv"
                            "$REPO/modules/execUnit.sv"
                            "$REPO/modules/eu.sv"
                            "$REPO/modules/prefetch_queue.sv"
                            "$REPO/modules/biu.sv"
                            "$REPO/modules/pcb.sv"
                            "$REPO/modules/interrupt_controller.sv"
                            "$REPO/modules/timer.sv"
                            "$REPO/modules/dma.sv"
                            "$REPO/modules/chip_select.sv"
                            "$REPO/modules/cpu_top.sv"
                            "$REPO/modules/vram.sv"
                            "$REPO/modules/framebuffer.sv"
                            "$REPO/modules/vga_dac.sv"
                            "$REPO/modules/bios_rom.sv"
                            "$REPO/modules/memory_controller.sv"
                            "$REPO/modules/io_decode.sv"
                            "$REPO/modules/keyboard_controller.sv"
                            "$REPO/modules/storage.sv"
                            "$REPO/modules/crtc.sv"
                            "$REPO/modules/font_rom.sv"
                            "$REPO/modules/vga_controller.sv"
                            "$REPO/modules/clk_rst.sv"
                            "$REPO/modules/sdram_controller.sv"
                            "$REPO/modules/sdram_arbiter.sv"
                            "$REPO/sim/sdram_model.sv"
                            "$REPO/modules/FPGA80186.sv") ;;
    tb_memsys)         RTL=("$PKG"
                            "$REPO/modules/vram.sv"
                            "$REPO/modules/framebuffer.sv"
                            "$REPO/modules/vga_dac.sv"
                            "$REPO/modules/bios_rom.sv"
                            "$REPO/modules/sdram_controller.sv"
                            "$REPO/modules/sdram_arbiter.sv"
                            "$REPO/modules/memory_controller.sv"
                            "$REPO/modules/chip_select.sv") ;;
    tb_biu)            RTL=("$PKG" "$REPO/modules/biu.sv" "$REPO/modules/prefetch_queue.sv") ;;
    tb_cpu|tb_interrupt|tb_pic|tb_timer|tb_far|tb_string|tb_misc|tb_dma|tb_halt) RTL=("$PKG"
                            "$REPO/modules/ALU.sv"
                            "$REPO/modules/regfile.sv"
                            "$REPO/modules/decode.sv"
                            "$REPO/modules/microcode.sv"
                            "$REPO/modules/execUnit.sv"
                            "$REPO/modules/eu.sv"
                            "$REPO/modules/prefetch_queue.sv"
                            "$REPO/modules/biu.sv"
                            "$REPO/modules/pcb.sv"
                            "$REPO/modules/interrupt_controller.sv"
                            "$REPO/modules/timer.sv"
                            "$REPO/modules/dma.sv"
                            "$REPO/modules/chip_select.sv"
                            "$REPO/modules/cpu_top.sv") ;;
    *)          echo "error: unknown testbench '$TB' -- add its RTL list to run.sh" >&2
                exit 1 ;;
esac

if [ ! -f "$REPO/sim/$TB.sv" ]; then
    echo "error: $REPO/sim/$TB.sv not found" >&2
    exit 1
fi

# --- build and run --------------------------------------------------------
mkdir -p "$BUILD"
cd "$BUILD"

# The ROM images are referenced by relative path so that Quartus (which
# resolves from the project directory) and the simulator agree. The simulator
# runs here, so point that same relative path at the real files.
rm -f rom
ln -s "$REPO/rom" rom

# tb_msdos reads a proprietary disk image that cannot live in the repository,
# so the directory holding it is linked the same way when it is present.
rm -f ms-dos
if [ -d "$REPO/ms-dos" ]; then ln -s "$REPO/ms-dos" ms-dos; fi

rm -rf work
"$MODEL_TECH/vlib" work > /dev/null

echo "== compiling =="
# Capture rather than pipe: piping into grep hides vlog's exit status, and a
# failed compile must not fall through to a simulation run that then reports a
# meaningless pass. Check the exit code AND scan for "** Error", since an
# internal compiler error can leave a half-built module behind.
if ! "$MODEL_TECH/vlog" -sv +incdir+"$REPO/rom" "${RTL[@]}" "$REPO/sim/$TB.sv" > vlog.log 2>&1; then
    echo "COMPILE FAILED:"
    cat vlog.log
    exit 1
fi
if grep -q '^\*\* Error' vlog.log; then
    echo "COMPILE ERRORS:"
    grep -E '^\*\* Error' vlog.log
    exit 1
fi
grep -E '^\*\* Warning' vlog.log || true

echo "== running $TB =="
# The vsim-3116 symbol warnings are noise from running a 32-bit binary here;
# they are not simulation errors.
# Do not let a non-zero vsim exit abort the script under `set -e`: the checks
# below turn it into a message that says WHY it failed.
"$MODEL_TECH/vsim" -c -do "run -all; quit" "work.$TB" > vsim.log 2>&1 || true
grep -Ev 'vsim-3116' vsim.log | sed -e 's/^# //'

# A design that COMPILES but fails to ELABORATE runs no test at all and prints
# no failure line, which reads as a pass when scanning a batch of results.
# Catch that explicitly instead of trusting the absence of bad news.
if grep -q "Error loading design" vsim.log; then
    echo "ELABORATION FAILED"
    exit 1
fi
if ! grep -q "checks:" vsim.log; then
    echo "NO TEST OUTPUT -- the testbench did not run to completion"
    exit 1
fi
