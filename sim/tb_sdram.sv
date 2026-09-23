`timescale 1ns/1ns
//
// SDRAM controller test, against a behavioural model that enforces protocol.
//
// The model raises an error on any READ or WRITE issued without an ACTIVE row,
// on an ACTIVE to a bank that is already open, and on a refresh while a row is
// still open -- so a controller that forgets to precharge, or that reads from a
// stale row, fails here rather than corrupting memory on real hardware.
//
module tb_sdram;

    logic        clk = 0, rst_n = 0;
    logic [23:0] addr = 0;
    logic [15:0] wdata = 0, rdata;
    logic        rd = 0, wr = 0;
    logic [1:0]  be = 2'b11;
    logic        ready;

    logic [12:0] dram_addr;
    logic [1:0]  dram_ba, dram_dqm;
    wire  [15:0] dram_dq;
    logic        dram_cke, dram_cs_n, dram_ras_n, dram_cas_n, dram_we_n, dram_clk;

    // Short init so the test does not simulate 200 us of power-up delay, and a
    // short refresh interval so refreshes actually happen during the run.
    sdram_controller #(
        .INIT_CYCLES    (20),
        .REFRESH_CYCLES (60),
        .CAS_LATENCY    (2)
    ) dut (
        .clk (clk), .rst_n (rst_n),
        .addr (addr), .wdata (wdata), .rdata (rdata),
        .rd (rd), .wr (wr), .be (be), .ready (ready),
        .dram_addr (dram_addr), .dram_ba (dram_ba), .dram_dq (dram_dq),
        .dram_cke (dram_cke), .dram_cs_n (dram_cs_n), .dram_ras_n (dram_ras_n),
        .dram_cas_n (dram_cas_n), .dram_we_n (dram_we_n), .dram_dqm (dram_dqm),
        // The memory clock is supplied rather than derived inside the
        // controller, so the real design can phase-shift it from the PLL.
        // Inverting `clk` is exactly what the controller used to do for
        // itself, so these tests still measure what they were written against.
        .dram_clk_in (~clk),
        .dram_clk (dram_clk)
    );

    sdram_model #(.CAS_LATENCY(2)) chip (
        .dram_clk (dram_clk), .dram_cke (dram_cke), .dram_cs_n (dram_cs_n),
        .dram_ras_n (dram_ras_n), .dram_cas_n (dram_cas_n), .dram_we_n (dram_we_n),
        .dram_addr (dram_addr), .dram_ba (dram_ba), .dram_dqm (dram_dqm),
        .dram_dq (dram_dq)
    );

    always #10 clk = ~clk;         // 50 MHz

    int errors = 0, checks = 0;
    task chk(input string nm, input int got, input int exp);
        checks++;
        if (got !== exp) begin
            $display("FAIL %-34s got=%04h exp=%04h", nm, got, exp);
            errors++;
        end
    endtask

    int cycles;

    task do_write(input [19:0] a, input [15:0] d, input [1:0] mask);
        begin
            @(negedge clk);
            addr = a; wdata = d; be = mask; wr = 1;
            cycles = 0;
            while (!ready && cycles < 1000) begin @(negedge clk); cycles++; end
            @(negedge clk);
            wr = 0;
            @(negedge clk);
        end
    endtask

    task do_read(input [19:0] a, output [15:0] d);
        begin
            @(negedge clk);
            addr = a; be = 2'b11; rd = 1;
            cycles = 0;
            while (!ready && cycles < 1000) begin @(negedge clk); cycles++; end
            d = rdata;
            @(negedge clk);
            rd = 0;
            @(negedge clk);
        end
    endtask

    logic [15:0] d;
    int i, before_refresh;
    int hit_min, miss_min;

    initial begin
        repeat (4) @(negedge clk);
        rst_n = 1;

        // Initialisation has to finish before anything is accepted; the model
        // flags a read or write issued before the mode register is set.
        cycles = 0;
        while (!chip.initialised && cycles < 2000) begin @(negedge clk); cycles++; end
        chk("controller completed initialisation", chip.initialised, 1'b1);
        chk("mode register: burst length 1", chip.mode_reg[2:0], 3'b000);
        chk("mode register: CAS latency 2",  chip.mode_reg[6:4], 3'b010);

        // ---- basic write / read ----
        do_write(20'h00100, 16'h1234, 2'b11);
        do_read (20'h00100, d);
        chk("word write then read", d, 16'h1234);

        do_write(20'h00102, 16'hBEEF, 2'b11);
        do_read (20'h00102, d);
        chk("second word", d, 16'hBEEF);
        do_read (20'h00100, d);
        chk("first word survived", d, 16'h1234);

        // ---- byte masking ----
        do_write(20'h00200, 16'hAAAA, 2'b11);
        do_write(20'h00200, 16'h00BB, 2'b01);      // low byte only
        do_read (20'h00200, d);
        chk("low-byte write kept the high byte", d, 16'hAABB);
        do_write(20'h00200, 16'hCC00, 2'b10);      // high byte only
        do_read (20'h00200, d);
        chk("high-byte write kept the low byte", d, 16'hCCBB);

        // ---- addresses that cross a row boundary ----
        // Columns are the low ten bits of the word address, so 800h bytes in
        // is a different row.
        do_write(20'h00000, 16'h1111, 2'b11);
        do_write(20'h00800, 16'h2222, 2'b11);      // row 1
        do_write(20'h01000, 16'h3333, 2'b11);      // row 2
        do_read (20'h00000, d); chk("row 0 readback", d, 16'h1111);
        do_read (20'h00800, d); chk("row 1 readback", d, 16'h2222);
        do_read (20'h01000, d); chk("row 2 readback", d, 16'h3333);
        do_read (20'h00800, d); chk("row 1 again after switching", d, 16'h2222);

        // ---- high addresses, near the top of the 1 MB space ----
        do_write(20'hFFFFE, 16'hF00D, 2'b11);
        do_read (20'hFFFFE, d);
        chk("top of memory", d, 16'hF00D);

        // ---- a run of sequential accesses ----
        for (i = 0; i < 16; i++)
            do_write(20'h03000 + i*2, 16'h5000 + i[15:0], 2'b11);
        for (i = 0; i < 16; i++) begin
            do_read(20'h03000 + i*2, d);
            chk("sequential readback", d, 16'h5000 + i[15:0]);
        end

        // ---- refresh keeps running ----
        before_refresh = chip.refresh_count;
        repeat (2000) @(negedge clk);
        chk("refreshes are being issued", (chip.refresh_count > before_refresh), 1'b1);

        // ...and data survives them
        do_read(20'h00100, d);
        chk("data survived refresh", d, 16'h1234);

        // ---- the open row must actually save time ----
        // Correctness tests pass whether or not the row stays open, so
        // without this the optimisation could quietly stop working and
        // nothing would notice. Minimums over several attempts, because a
        // refresh landing inside one measurement inflates it.
        hit_min  = 9999;
        miss_min = 9999;
        for (i = 0; i < 8; i++) begin
            do_read(20'h00000, d);                // may miss; opens the row
            do_read(20'h00002, d);                // same row: hit
            if (cycles < hit_min) hit_min = cycles;
            do_read(20'h00800, d);                // a different row: miss
            if (cycles < miss_min) miss_min = cycles;
        end
        $display("  read cost: %0d clocks on a page hit, %0d on a miss",
                 hit_min, miss_min);
        checks++;
        if (!(hit_min < miss_min)) begin
            $display("FAIL a page hit (%0d) is not cheaper than a miss (%0d)",
                     hit_min, miss_min);
            errors++;
        end

        // A hit skips ACTIVATE, tRCD and tRP -- around five clocks at these
        // timings. Requiring a clear margin rather than merely "faster"
        // catches a hit path that has silently grown the work back.
        checks++;
        if (!(miss_min - hit_min >= 3)) begin
            $display("FAIL page hit saves only %0d clocks, expected 3 or more",
                     miss_min - hit_min);
            errors++;
        end

        // The row must be reopened correctly after a refresh closes it.
        do_write(20'h00300, 16'hC0DE, 2'b11);
        repeat (400) @(negedge clk);              // long enough for a refresh
        do_read(20'h00300, d);
        chk("data readable after the row was closed by refresh", d, 16'hC0DE);

        // ---- the model saw no protocol violations ----
        chk("no SDRAM protocol errors", chip.errors, 0);

        $display("");
        $display("==================================");
        $display(" checks: %0d   failures: %0d", checks, errors);
        $display("==================================");
        if (errors == 0) $display("ALL TESTS PASSED");
        $finish;
    end

    initial begin
        #4000000;
        $display("FAIL global timeout");
        $display(" checks: %0d   failures: %0d", checks, errors + 1);
        $finish;
    end

endmodule
