`timescale 1ns/1ns
//
// Whole-board integration test. Everything below FPGA80186 is instantiated for
// real -- clocks, CPU, memory, I/O, keyboard and video -- and a small program
// is executed out of the boot ROM.
//
// What this is for: proving the pieces connect and run together. The
// module-level testbenches already check each block's behaviour in detail;
// what only shows up here is mis-wiring, a domain crossing that was not
// thought through, or an output left floating.
//
// This is now a genuine cold boot. The program lives entirely in the boot ROM
// and the reset vector is a FAR jump, which is how a real BIOS escapes the
// sixteen bytes reachable at FFFF0 before the address wraps out of ROM.
// Conventional RAM is the board's SDRAM, so the CPU starts fetching from ROM
// while the SDRAM is still running its power-up initialisation, and the first
// write to memory stalls on READY until that finishes -- exactly the behaviour
// the wait-state path exists for.
//
module tb_top;

    logic       CLOCK_50 = 0;
    logic [0:0] KEY = 1'b1;
    logic [9:0] LEDR;
    logic [7:0] VGA_R, VGA_G, VGA_B;
    logic       VGA_HS, VGA_VS, VGA_CLK, VGA_BLANK_N, VGA_SYNC_N;
    logic       PS2_CLK = 1'b1, PS2_DAT = 1'b1;
    logic [12:0] DRAM_ADDR;
    logic [1:0]  DRAM_BA;
    wire  [15:0] DRAM_DQ;
    logic        DRAM_CKE, DRAM_CS_N, DRAM_RAS_N, DRAM_CAS_N, DRAM_WE_N;
    logic        DRAM_LDQM, DRAM_UDQM, DRAM_CLK;

    // Short debounce so the test does not spend 10 ms in reset.
    // The JTAG loader holds an Altera primitive that cannot be simulated.
    FPGA80186 #(.ENABLE_JTAG_LOADER(1'b0)) dut (.*);
    defparam dut.u_clk_rst.DEBOUNCE = 20;
    defparam dut.u_mem.SDRAM_INIT_CYCLES = 40;

    sdram_model #(.CAS_LATENCY(2)) chip (
        .dram_clk (DRAM_CLK), .dram_cke (DRAM_CKE), .dram_cs_n (DRAM_CS_N),
        .dram_ras_n (DRAM_RAS_N), .dram_cas_n (DRAM_CAS_N), .dram_we_n (DRAM_WE_N),
        .dram_addr (DRAM_ADDR), .dram_ba (DRAM_BA),
        .dram_dqm ({DRAM_UDQM, DRAM_LDQM}), .dram_dq (DRAM_DQ)
    );

    always #10 CLOCK_50 = ~CLOCK_50;        // 50 MHz

    int errors = 0, checks = 0;
    task chk(input string nm, input int got, input int exp);
        checks++;
        if (got !== exp) begin
            $display("FAIL %-38s got=%04h exp=%04h", nm, got, exp);
            errors++;
        end
    endtask

    `define AX  dut.u_cpu.u_eu.u_regfile.gpr[0]
    `define BX  dut.u_cpu.u_eu.u_regfile.gpr[3]


    int i;

    // Load a byte into the boot ROM image. bios_rom splits even and odd bytes
    // into separate banks and indexes them by addr[11:1].
    task rom_put(input int phys, input byte b);
        begin
            if (phys[0]) dut.u_mem.u_rom.g_inferred.rom_hi[phys[11:1]] = b;
            else         dut.u_mem.u_rom.g_inferred.rom_lo[phys[11:1]] = b;
        end
    endtask

    // Conventional RAM lives in the SDRAM model, indexed by word address.
    function automatic [15:0] ram_word(input int phys);
        ram_word = chip.mem[phys >> 1];
    endfunction

    int hs_falls, vs_falls;
    logic [9:0] v_before;
    logic hs_d, vs_d;
    always @(posedge VGA_CLK) begin
        hs_d <= VGA_HS; vs_d <= VGA_VS;
        if (hs_d && !VGA_HS) hs_falls++;
        if (vs_d && !VGA_VS) vs_falls++;
    end

    initial begin
        hs_falls = 0; vs_falls = 0;

        // ---- reset vector: FAR jump to F000:0100, which stays in ROM ----
        rom_put('hFFF0, 8'hEA);
        rom_put('hFFF1, 8'h00); rom_put('hFFF2, 8'h01);
        rom_put('hFFF3, 8'h00); rom_put('hFFF4, 8'hF0);

        // ---- program at F000:0100, i.e. ROM offset 100h ----
        i = 'h0100;
        rom_put(i++, 8'hBC); rom_put(i++, 8'h00); rom_put(i++, 8'h80); // MOV SP,8000h
        rom_put(i++, 8'hB8); rom_put(i++, 8'h00); rom_put(i++, 8'h00); // MOV AX,0
        rom_put(i++, 8'h8E); rom_put(i++, 8'hD8);                      // MOV DS,AX
        rom_put(i++, 8'hB8); rom_put(i++, 8'h34); rom_put(i++, 8'h12); // MOV AX,1234h
        rom_put(i++, 8'hBB); rom_put(i++, 8'h00); rom_put(i++, 8'h20); // MOV BX,2000h
        rom_put(i++, 8'h89); rom_put(i++, 8'h07);                      // MOV [BX],AX
        rom_put(i++, 8'h50);                                           // PUSH AX
        rom_put(i++, 8'h5A);                                           // POP DX
        rom_put(i++, 8'h40);                                           // INC AX
        rom_put(i++, 8'hF4);                                           // HLT

        // release reset
        repeat (10) @(negedge CLOCK_50);
        KEY[0] = 1'b0;
        repeat (100) @(negedge CLOCK_50);
        KEY[0] = 1'b1;

        // ---- the CPU should run the program and halt ----
        i = 0;
        while (!dut.halted && i < 200000) begin @(negedge CLOCK_50); i++; end
        chk("cpu halted", dut.halted, 1'b1);
        chk("AX after program", `AX, 16'h1235);
        chk("BX after program", `BX, 16'h2000);
        chk("write reached SDRAM", ram_word('h2000), 16'h1234);
        // PUSH/POP exercised the stack, which is also in SDRAM.
        chk("stack round-trip through SDRAM",
            dut.u_cpu.u_eu.u_regfile.gpr[2], 16'h1234);
        chk("SDRAM initialised", chip.initialised, 1'b1);
        chk("no SDRAM protocol errors", chip.errors, 0);

        // LED[7] mirrors halted, which is the only bring-up signal before
        // video works.
        chk("halt visible on LED", LEDR[7], 1'b1);

        // ---- video runs independently of the CPU ----
        // The CPU is halted; scan-out must keep going regardless. The line
        // counter is sampled rather than waiting for a vertical sync pulse,
        // which would need a full 420,000-pixel-clock frame -- tb_vga already
        // checks the vertical timing precisely, so all that is needed here is
        // evidence that scan-out is alive and independent of the CPU.
        hs_falls = 0;
        v_before = dut.u_vga.v_cnt;
        repeat (60000) @(negedge CLOCK_50);
        chk("horizontal sync is running", (hs_falls > 20), 1'b1);
        chk("line counter advancing",     (dut.u_vga.v_cnt !== v_before), 1'b1);
        chk("sync-on-green tied off", VGA_SYNC_N, 1'b0);

        // ---- clocking ----
        chk("VGA_CLK is driven", (VGA_CLK === 1'b0 || VGA_CLK === 1'b1), 1'b1);

        // ---- nothing floating on the board outputs ----
        chk("LED defined",   (^LEDR  !== 1'bx), 1'b1);
        chk("VGA_R defined", (^VGA_R !== 1'bx), 1'b1);
        chk("VGA_G defined", (^VGA_G !== 1'bx), 1'b1);
        chk("VGA_B defined", (^VGA_B !== 1'bx), 1'b1);
        chk("VGA_HS defined", (VGA_HS === 1'b0 || VGA_HS === 1'b1), 1'b1);
        chk("VGA_VS defined", (VGA_VS === 1'b0 || VGA_VS === 1'b1), 1'b1);
        chk("VGA_BLANK_N defined",
            (VGA_BLANK_N === 1'b0 || VGA_BLANK_N === 1'b1), 1'b1);

        $display("");
        $display("==================================");
        $display(" checks: %0d   failures: %0d", checks, errors);
        $display("==================================");
        if (errors == 0) $display("ALL TESTS PASSED");
        $finish;
    end

    initial begin
        #20000000;
        $display("FAIL global timeout (halted=%b IP=%04h)", dut.halted, dut.dbg_ip);
        $display(" checks: %0d   failures: %0d", checks, errors + 1);
        $finish;
    end

endmodule
