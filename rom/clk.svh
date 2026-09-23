// Written by tools/gen_bios.py -- do not edit.
// The CPU clock rate this BIOS's timer divisor assumes.
// FPGA80186.sv checks its own CLK_HZ against this and refuses
// to compile if they differ. Rebuild the ROM with
//     python3 tools/gen_bios.py --clk-hz <rate> rom/
localparam int ROM_CLK_HZ = 40000000;
