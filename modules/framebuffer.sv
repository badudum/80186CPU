// ---------------------------------------------------------------------------
// framebuffer — 64 KB linear graphics memory for mode 13h.
//
// Hierarchy: FPGA80186 -> memory_controller -> framebuffer
//            (read port goes to vga_controller)
//
// The PC puts its graphics aperture at A0000. Mode 13h is 320x200 at eight
// bits per pixel, one byte per pixel, packed left to right and top to bottom
// with no padding -- 64,000 bytes, which is why a 64 KB window covers it with
// room to spare and why the decode is a clean A0000-AFFFF rather than an
// awkward A0000-AF9FF.
//
// ON-CHIP, NOT IN SDRAM, and that is the whole point. Scanning 320x200 out at
// 60 Hz is 3.84 MB/s, and the SDRAM controller delivers about 3.3 MB/s because
// it bursts one word at a time and auto-precharges every access. Display alone
// would consume more than the whole memory system can provide, before the CPU
// fetched a single instruction. Here scan-out costs the rest of the machine
// nothing: it is a second port on a block RAM that nobody else is using.
//
// The cost is 50 of the 397 M10K blocks, which the design has in abundance.
//
// Like vram, the two ports have INDEPENDENT CLOCKS -- the CPU writes on
// clk_cpu, the video scan-out reads on clk_vga -- and the M10K handles the
// crossing. Do not add synchronisers.
//
// Organised as two byte-wide banks so a byte write touches one lane, which is
// what a program plotting single pixels does almost exclusively.
// ---------------------------------------------------------------------------

`timescale 1ns/1ns
module framebuffer #(
    parameter int AW = 15                     // words; 32768 x 16 = 64 KB
) (
    // CPU side
    input  logic            clk_cpu,
    input  logic [AW-1:0]   cpu_addr,
    input  logic [15:0]     cpu_wdata,
    input  logic            cpu_we,
    input  logic [1:0]      cpu_be,           // [0] low byte, [1] high byte
    output logic [15:0]     cpu_rdata,

    // VGA side (read only)
    input  logic            clk_vga,
    input  logic [AW-1:0]   vga_addr,
    output logic [15:0]     vga_rdata
);

    logic [7:0] ram_lo [0:(1<<AW)-1];
    logic [7:0] ram_hi [0:(1<<AW)-1];

    // Start black. Without this the first frame after power-up is whatever the
    // block RAM came up as, which looks like a fault rather than an empty
    // screen.
    initial begin
        for (int i = 0; i < (1<<AW); i++) begin
            ram_lo[i] = 8'h00;
            ram_hi[i] = 8'h00;
        end
    end

    // Plain `always`, not `always_ff`: the initial block above also writes
    // these arrays, and always_ff forbids a second driver. See vram.sv.
    always @(posedge clk_cpu) begin
        if (cpu_we && cpu_be[0]) ram_lo[cpu_addr] <= cpu_wdata[7:0];
        if (cpu_we && cpu_be[1]) ram_hi[cpu_addr] <= cpu_wdata[15:8];
        cpu_rdata <= {ram_hi[cpu_addr], ram_lo[cpu_addr]};
    end

    always_ff @(posedge clk_vga)
        vga_rdata <= {ram_hi[vga_addr], ram_lo[vga_addr]};

endmodule
