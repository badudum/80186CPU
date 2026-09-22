# ---------------------------------------------------------------------------
# FPGA80186 timing constraints (DE1-SoC, Cyclone V).
#
# There is no PLL in this design. CLOCK_50 enters the chip and clk_rst divides
# it by two; that 25 MHz clock runs the CPU, the SDRAM and the video output.
# Only the divider itself, the reset debouncer and the power-on counter remain
# in the 50 MHz domain, and nothing crosses between the two synchronously --
# the reset they produce is applied asynchronously and analysed as recovery and
# removal, not as a data path.
#
# The derived clock has to be declared, or the analyser treats it as a plain
# data path and reports nonsense for the whole design.
# ---------------------------------------------------------------------------

create_clock -name CLOCK_50 -period 20.000 [get_ports CLOCK_50]

# clk_sys = CLOCK_50 / 2 = 25 MHz. For video this is 0.7% under the nominal
# 25.175 MHz pixel clock for 640x480 @ 60 Hz, putting refresh near 59.5 Hz.
create_generated_clock -name clk_sys -source [get_ports CLOCK_50] -divide_by 2 \
    [get_registers {clk_rst:*|clk_sys_r}]

derive_clock_uncertainty

# Asynchronous or non-timing-critical I/O.
set_false_path -from [get_ports {KEY[*]}] -to *
set_false_path -from [get_ports PS2_CLK]  -to *
set_false_path -from [get_ports PS2_DAT]  -to *
set_false_path -from * -to [get_ports {LEDR[*]}]

# VGA outputs feed the ADV7123 DAC. A proper build would constrain these
# against the DAC's setup/hold window; false-pathing them is acceptable for a
# first bring-up build but should be revisited before trusting the picture at
# higher pixel clocks.
set_false_path -from * -to [get_ports {VGA_R[*]}]
set_false_path -from * -to [get_ports {VGA_G[*]}]
set_false_path -from * -to [get_ports {VGA_B[*]}]
set_false_path -from * -to [get_ports VGA_HS]
set_false_path -from * -to [get_ports VGA_VS]
set_false_path -from * -to [get_ports VGA_CLK]
set_false_path -from * -to [get_ports VGA_BLANK_N]
set_false_path -from * -to [get_ports VGA_SYNC_N]

# ---------------------------------------------------------------------------
# SDRAM interface (IS42S16320D-7TL, 32M x 16, on the DE1-SoC).
#
# DRAM_CLK is the inverted system clock -- see sdram_controller.sv, which does
# `assign dram_clk = ~clk` because there is no PLL to phase-shift with. That
# has to be DECLARED as a generated clock or the analyser treats it as ordinary
# data and times nothing against it.
#
# These delays are not decoration. Before they were added the entire memory
# interface was unconstrained, so STA reported no margin for it and the build
# looked clean while the read path was in fact missing its capture edge. That
# violation is what set the system clock at 25 MHz rather than 50.
#
# Datasheet numbers for the -7 speed grade:
#   tAC 5.4 ns   clock -> data valid    (read, worst case)
#   tOH 2.7 ns   data hold after clock  (read)
#   tSU 1.5 ns   input setup            (write/command)
#   tHD 0.8 ns   input hold             (write/command)
#
# Board trace delay is ignored; the DE1-SoC traces are short and every
# DE-series reference constraint set does the same.
# ---------------------------------------------------------------------------
create_generated_clock -name DRAM_CLK -source [get_ports CLOCK_50] \
    -divide_by 2 -invert [get_ports DRAM_CLK]

set sdram_outs [get_ports {DRAM_ADDR[*] DRAM_BA[*] DRAM_DQ[*] \
                           DRAM_CKE DRAM_CS_N DRAM_RAS_N DRAM_CAS_N \
                           DRAM_WE_N DRAM_LDQM DRAM_UDQM}]

set_output_delay -clock DRAM_CLK -max  1.5 $sdram_outs
set_output_delay -clock DRAM_CLK -min -0.8 $sdram_outs

set_input_delay  -clock DRAM_CLK -max  5.4 [get_ports {DRAM_DQ[*]}]
set_input_delay  -clock DRAM_CLK -min  2.7 [get_ports {DRAM_DQ[*]}]
