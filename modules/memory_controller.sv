// ---------------------------------------------------------------------------
// memory_controller — system memory map, address decode, and READY generation.
//
// Hierarchy: FPGA80186 -> memory_controller -> {sdram_controller, bios_rom, vram}
// Reference: learnings/01-programming-model.md (memory map),
//            learnings/02-bus-interface.md (READY, byte lanes)
// Testbench: sim/tb_memsys.sv (on-chip RAM), sim/tb_sdram.sv (the controller),
//            sim/tb_bios.sv (the whole board against an SDRAM model)
//
// Memory map (the standard IBM PC/XT layout, which is what MS-DOS expects):
//     00000-9FFFF  conventional RAM, 640 KB      -> SDRAM, or a small on-chip
//                                                   window when USE_SDRAM=0
//     A0000-BFFFF  video RAM; text buffer B8000  -> vram
//     C0000-EFFFF  option ROM / extended BIOS    -> unmapped
//     F0000-FFFFF  BIOS ROM, reset vector FFFF0  -> bios_rom
//
// USE_SDRAM picks what backs conventional RAM. With it clear, RAM is a RAM_KB
// window of on-chip M10K -- enough for test programs, nowhere near enough for
// DOS, and it keeps the design buildable and simulatable without the SDRAM
// pins. With it set, the full 640 KB comes from the board's SDRAM.
//
// READY is where the two differ and why it is generated per region rather than
// as a fixed delay: on-chip memory answers in one cycle, while SDRAM takes
// around ten and occasionally much longer when a refresh gets in first. That
// variability is exactly what the BIU's wait-state path exists to absorb.
//
// NO TRI-STATE on the CPU side: Cyclone V has no internal tri-state buffers,
// so the bus is split into wdata/rdata. Only dram_dq is a real bidirectional
// net, and that is legal because it is a chip pin.
// ---------------------------------------------------------------------------

`timescale 1ns/1ns
module memory_controller #(
    parameter int RAM_KB    = 32,     // on-chip RAM window when USE_SDRAM = 0
    parameter bit USE_SDRAM = 1'b0,
    // Power-up delay the SDRAM needs before it accepts commands. Real parts
    // want 200 us; a testbench cannot afford to simulate that.
    parameter int SDRAM_INIT_CYCLES = 5000,
    parameter bit ISMCE = 1'b0
) (
    input  logic        clk,
    input  logic        rst_n,

    // CPU bus interface (from the BIU)
    input  logic [19:0] addr,
    input  logic [15:0] wdata,
    output logic [15:0] rdata,
    input  logic        rd,
    input  logic        wr,
    input  logic        bhe,          // active low: upper lane enabled
    input  logic        a0,           // low lane disabled when high
    output logic        ready,

    // VGA read port into the text buffer
    input  logic        clk_vga,
    input  logic [10:0] vram_read_addr,
    output logic [15:0] vram_read_data,

    // ---- extra SDRAM requesters ----
    // The block device and the JTAG loader also need the memory. They live
    // outside this module, so their ports are brought out here rather than
    // moving the DRAM pins somewhere else. Addresses are 24-bit: these reach
    // ABOVE the CPU's 1 MB, which is where the disk image lives.
    // Tie rd/wr low to leave a port idle.
    input  logic [1:0]  ext_rd,
    input  logic [1:0]  ext_wr,
    input  logic [23:0] ext_addr  [2],
    input  logic [15:0] ext_wdata [2],
    input  logic [1:0]  ext_be    [2],
    output logic [1:0]  ext_ready,
    output logic [15:0] ext_rdata,

    // SDRAM chip pins (unused and driven inactive when USE_SDRAM = 0)
    output logic [12:0] dram_addr,
    output logic [1:0]  dram_ba,
    inout  wire  [15:0] dram_dq,
    output logic        dram_cke,
    output logic        dram_cs_n,
    output logic        dram_ras_n,
    output logic        dram_cas_n,
    output logic        dram_we_n,
    output logic [1:0]  dram_dqm,
    output logic        dram_clk
);

    localparam int RAM_WORDS = (RAM_KB * 1024) / 2;
    localparam int RAM_AW    = $clog2(RAM_WORDS);

    // ---- region decode ----
    logic in_ram, in_vram, in_rom;
    assign in_ram  = USE_SDRAM ? (addr < 20'hA0000)
                               : ((addr < 20'hA0000) && (addr < RAM_KB * 1024));
    assign in_vram = (addr >= 20'hB8000) && (addr < 20'hB9000);
    assign in_rom  = (addr >= 20'hF0000);

    logic [1:0] be;
    assign be = {~bhe, ~a0};

    logic [15:0] ram_q;
    logic        ram_ready;

    generate
        if (USE_SDRAM) begin : g_sdram
            // Three requesters share the one controller port: the CPU here,
            // and the two brought out above. See sdram_arbiter.sv.
            logic [2:0]  arb_rd, arb_wr, arb_ready;
            logic [23:0] arb_addr  [3];
            logic [15:0] arb_wdata [3];
            logic [1:0]  arb_be    [3];
            logic [15:0] arb_rdata;

            logic [23:0] mem_addr;
            logic [15:0] mem_wdata, mem_rdata;
            logic [1:0]  mem_be;
            logic        mem_rd, mem_wr, mem_ready;

            // Port 0 is the CPU. Its address is only 20 bits wide, so it
            // reaches the bottom 1 MB and cannot see the disk above it.
            assign arb_rd[0]    = rd && in_ram;
            assign arb_wr[0]    = wr && in_ram;
            assign arb_addr[0]  = {4'h0, addr};
            assign arb_wdata[0] = wdata;
            assign arb_be[0]    = be;

            assign arb_rd[2:1]  = ext_rd;
            assign arb_wr[2:1]  = ext_wr;
            assign arb_addr[1]  = ext_addr[0];
            assign arb_addr[2]  = ext_addr[1];
            assign arb_wdata[1] = ext_wdata[0];
            assign arb_wdata[2] = ext_wdata[1];
            assign arb_be[1]    = ext_be[0];
            assign arb_be[2]    = ext_be[1];

            assign ram_q     = arb_rdata;
            assign ram_ready = arb_ready[0];
            assign ext_ready = arb_ready[2:1];
            assign ext_rdata = arb_rdata;

            sdram_arbiter #(.NREQ(3), .AW(24)) u_arb (
                .clk       (clk),
                .rst_n     (rst_n),
                .req_rd    (arb_rd),
                .req_wr    (arb_wr),
                .req_addr  (arb_addr),
                .req_wdata (arb_wdata),
                .req_be    (arb_be),
                .req_ready (arb_ready),
                .req_rdata (arb_rdata),
                .mem_addr  (mem_addr),
                .mem_wdata (mem_wdata),
                .mem_be    (mem_be),
                .mem_rd    (mem_rd),
                .mem_wr    (mem_wr),
                .mem_rdata (mem_rdata),
                .mem_ready (mem_ready)
            );

            sdram_controller #(.INIT_CYCLES(SDRAM_INIT_CYCLES)) u_sdram (
                .clk        (clk),
                .rst_n      (rst_n),
                .addr       (mem_addr),
                .wdata      (mem_wdata),
                .rdata      (mem_rdata),
                .rd         (mem_rd),
                .wr         (mem_wr),
                .be         (mem_be),
                .ready      (mem_ready),
                .dram_addr  (dram_addr),
                .dram_ba    (dram_ba),
                .dram_dq    (dram_dq),
                .dram_cke   (dram_cke),
                .dram_cs_n  (dram_cs_n),
                .dram_ras_n (dram_ras_n),
                .dram_cas_n (dram_cas_n),
                .dram_we_n  (dram_we_n),
                .dram_dqm   (dram_dqm),
                .dram_clk   (dram_clk)
            );
        end else begin : g_onchip
            // The fallback path has no SDRAM at all, so the extra requesters
            // are answered with a permanent "never ready" rather than being
            // left floating.
            assign ext_ready = 2'b00;
            assign ext_rdata = 16'h0000;
            // Two byte-wide banks, matching the 80186's even/odd organisation,
            // so a byte write needs no read-modify-write.
            logic [7:0] ram_lo [0:RAM_WORDS-1];
            logic [7:0] ram_hi [0:RAM_WORDS-1];

            logic [RAM_AW-1:0] ram_idx;
            assign ram_idx = addr[RAM_AW:1];

            always_ff @(posedge clk) begin
                if (wr && in_ram) begin
                    if (be[0]) ram_lo[ram_idx] <= wdata[7:0];
                    if (be[1]) ram_hi[ram_idx] <= wdata[15:8];
                end
                ram_q <= {ram_hi[ram_idx], ram_lo[ram_idx]};
            end

            // One-cycle latency, matching the block RAM.
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) ram_ready <= 1'b0;
                else        ram_ready <= (rd || wr) && in_ram;
            end

            assign dram_addr  = 13'h0000;
            assign dram_ba    = 2'b00;
            assign dram_dq    = 16'hzzzz;
            assign dram_cke   = 1'b0;
            assign dram_cs_n  = 1'b1;
            assign dram_ras_n = 1'b1;
            assign dram_cas_n = 1'b1;
            assign dram_we_n  = 1'b1;
            assign dram_dqm   = 2'b11;
            assign dram_clk   = 1'b0;
        end
    endgenerate

    // ---- video RAM ----
    logic [15:0] vram_q;
    vram #(.AW(11), .ISMCE(ISMCE)) u_vram (
        .clk_cpu   (clk),
        .cpu_addr  (addr[11:1]),
        .cpu_wdata (wdata),
        .cpu_we    (wr && in_vram),
        .cpu_be    (be),
        .cpu_rdata (vram_q),
        .clk_vga   (clk_vga),
        .vga_addr  (vram_read_addr),
        .vga_rdata (vram_read_data)
    );

    // ---- boot ROM ----
    logic [15:0] rom_q;
    bios_rom #(.ROM_AW(14), .ISMCE(ISMCE)) u_rom (
        .clk   (clk),
        .addr  (addr),
        .rdata (rom_q)
    );

    // ---- read mux, one cycle behind the address ----
    logic in_ram_q, in_vram_q, in_rom_q;
    always_ff @(posedge clk) begin
        in_ram_q  <= in_ram;
        in_vram_q <= in_vram;
        in_rom_q  <= in_rom;
    end

    always_comb begin
        if      (in_rom_q)  rdata = rom_q;
        else if (in_vram_q) rdata = vram_q;
        else if (in_ram)    rdata = ram_q;     // SDRAM holds its data after ready
        else if (in_ram_q)  rdata = ram_q;
        else                rdata = 16'hFFFF;  // unmapped: traps as FF /7
    end

    // ---- ready ----
    // Conventional RAM answers on its own schedule; everything else takes one
    // cycle. Unmapped addresses still acknowledge, so a stray access cannot
    // wedge the CPU in wait states forever.
    logic other_ready;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) other_ready <= 1'b0;
        else        other_ready <= (rd || wr) && !in_ram;
    end

    assign ready = in_ram ? ram_ready : other_ready;

endmodule
