`timescale 1ns/1ns
//
// The JTAG image loader, writing into SDRAM through the real stack.
//
// The Altera virtual JTAG node itself cannot be simulated without vendor
// libraries, so the loader is built with SIM_HOOKS and the JTAG state machine
// is driven from here instead: capture-DR, a run of shift-DR ticks, then
// update-DR, exactly as the host's scan produces them. Everything downstream
// of that -- the bit assembly, the clock crossing, the SDRAM writes and the
// status readback -- is the real thing.
//
// TCK IS A GENUINELY DIFFERENT CLOCK here, deliberately unrelated to the
// system clock, because that is what it is on hardware. The handshake between
// them is the part most likely to be subtly wrong, and the overflow test below
// deliberately runs TCK far too fast to confirm the safety net actually
// catches a writer that cannot keep up.
//
module tb_jtag_loader;

    logic clk = 0, rst_n = 0;
    always #5 clk = ~clk;                 // 100 ns/10 = system clock

    // ---- JTAG-side stimulus ----
    logic       sim_tck = 0, sim_tdi = 0, sim_tdo;
    logic [3:0] sim_ir = 4'd0;
    logic       sim_cdr = 0, sim_sdr = 0, sim_udr = 0;

    // Half-period, changed mid-test to provoke overflow.
    int tckh = 20;

    // ---- loader ----
    logic [23:0] ld_addr;
    logic [15:0] ld_wdata;
    logic [1:0]  ld_be;
    logic        ld_rd, ld_wr, cpu_hold;

    logic [2:0]  arb_rd, arb_wr, arb_ready;
    logic [23:0] arb_addr  [3];
    logic [15:0] arb_wdata [3];
    logic [1:0]  arb_be    [3];
    logic [15:0] arb_rdata;

    jtag_loader #(.SIM_HOOKS(1'b1)) dut (
        .clk (clk), .rst_n (rst_n),
        .mem_addr (ld_addr), .mem_wdata (ld_wdata), .mem_be (ld_be),
        .mem_rd (ld_rd), .mem_wr (ld_wr),
        .mem_rdata (arb_rdata), .mem_ready (arb_ready[2]),
        .cpu_hold (cpu_hold),
        .cpu_pc   (32'h0),
        .sim_tck (sim_tck), .sim_tdi (sim_tdi), .sim_ir (sim_ir),
        .sim_cdr (sim_cdr), .sim_sdr (sim_sdr), .sim_udr (sim_udr),
        .sim_tdo (sim_tdo)
    );

    // The loader sits on arbiter port 2, as it does in memory_controller.
    assign arb_rd = {ld_rd, 2'b00};
    assign arb_wr = {ld_wr, 2'b00};
    always_comb begin
        arb_addr[0] = 24'h0; arb_wdata[0] = 16'h0; arb_be[0] = 2'b11;
        arb_addr[1] = 24'h0; arb_wdata[1] = 16'h0; arb_be[1] = 2'b11;
        arb_addr[2] = ld_addr; arb_wdata[2] = ld_wdata; arb_be[2] = ld_be;
    end

    logic [23:0] mem_addr;
    logic [15:0] mem_wdata, mem_rdata;
    logic [1:0]  mem_be;
    logic        mem_rd, mem_wr, mem_ready;

    sdram_arbiter #(.NREQ(3), .AW(24)) u_arb (
        .clk (clk), .rst_n (rst_n),
        .req_rd (arb_rd), .req_wr (arb_wr),
        .req_addr (arb_addr), .req_wdata (arb_wdata), .req_be (arb_be),
        .req_ready (arb_ready), .req_rdata (arb_rdata),
        .mem_addr (mem_addr), .mem_wdata (mem_wdata), .mem_be (mem_be),
        .mem_rd (mem_rd), .mem_wr (mem_wr),
        .mem_rdata (mem_rdata), .mem_ready (mem_ready)
    );

    logic [12:0] dram_addr;
    logic [1:0]  dram_ba, dram_dqm;
    wire  [15:0] dram_dq;
    logic        dram_cke, dram_cs_n, dram_ras_n, dram_cas_n, dram_we_n, dram_clk;

    sdram_controller #(.INIT_CYCLES(40)) u_sdram (
        .clk (clk), .rst_n (rst_n),
        .addr (mem_addr), .wdata (mem_wdata), .rdata (mem_rdata),
        .rd (mem_rd), .wr (mem_wr), .be (mem_be), .ready (mem_ready),
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
        .dram_addr (dram_addr), .dram_ba (dram_ba),
        .dram_dqm (dram_dqm), .dram_dq (dram_dq)
    );

    function automatic int unsigned midx(input int unsigned byte_addr);
        midx = (byte_addr >> 1) & 21'h1FFFFF;
    endfunction

    int errors = 0, checks = 0;
    task chk(input string nm, input int got, input int exp);
        checks++;
        if (got !== exp) begin
            $display("FAIL %-48s got=%0h exp=%0h", nm, got, exp);
            errors++;
        end
    endtask

    localparam logic [3:0] IR_ADDR   = 4'd1;
    localparam logic [3:0] IR_DATA   = 4'd2;
    localparam logic [3:0] IR_CTRL   = 4'd3;
    localparam logic [3:0] IR_STATUS = 4'd4;
    localparam logic [3:0] IR_PEEK   = 4'd5;
    localparam logic [3:0] IR_PEEKD  = 4'd6;
    localparam logic [15:0] SYNC     = 16'hB2C1;

    // How many junk bits the SLD hub and the other nodes in the chain push in
    // ahead of the host's data. On hardware with the in-system memory editor
    // enabled this was measured at seven; the point of the test is that the
    // framing must not care what it is, so the tests below sweep it.
    int lead_in = 7;

    // ---- the JTAG scan sequence ----
    task automatic tck_tick;
        begin
            #(tckh) sim_tck = 1'b1;
            #(tckh) sim_tck = 1'b0;
        end
    endtask

    task automatic scan_begin(input [3:0] ir);
        begin
            // A few idle ticks first. Real JTAG walks the TAP state machine
            // through Select-DR-Scan and friends before reaching Capture-DR,
            // so TCK is always running for several cycles beforehand -- which
            // is what lets the status synchronisers settle. Ticking capture
            // straight away is a fiction that made the word count read short.
            sim_ir = ir; sim_sdr = 0; sim_udr = 0; sim_cdr = 0;
            repeat (4) tck_tick();
            sim_cdr = 1;
            tck_tick();
            sim_cdr = 0; sim_sdr = 1;
        end
    endtask

    task automatic scan_bit(input logic b);
        begin
            sim_tdi = b;
            tck_tick();
        end
    endtask

    task automatic scan_end;
        begin
            sim_sdr = 0; sim_udr = 1;
            tck_tick();
            sim_udr = 0;
        end
    endtask

    task automatic set_addr(input [23:0] a);
        begin
            scan_begin(IR_ADDR);
            for (int i = 0; i < 24; i++) scan_bit(a[i]);
            scan_end();
        end
    endtask

    task automatic set_ctrl(input [7:0] c);
        begin
            scan_begin(IR_CTRL);
            for (int i = 0; i < 8; i++) scan_bit(c[i]);
            scan_end();
        end
    endtask

    // One scan carrying many words, which is how the host actually streams.
    //
    // The scan opens the way tools/jtag_load.tcl opens it: `lead_in` bits of
    // whatever the chain had in it, then sixteen zeros, then SYNC, then the
    // data. The junk is the part that matters -- without it this testbench
    // passes against framing that is hopelessly broken on real hardware, which
    // is exactly what happened.
    task automatic send_words(input int n, input [15:0] first);
        begin
            scan_begin(IR_DATA);
            for (int i = 0; i < lead_in; i++) scan_bit($random);
            for (int i = 0; i < 16; i++) scan_bit(1'b0);
            for (int i = 0; i < 16; i++) scan_bit((SYNC >> i) & 1'b1);
            for (int w = 0; w < n; w++)
                for (int i = 0; i < 16; i++) scan_bit(((first + w) >> i) & 1'b1);
            scan_end();
        end
    endtask

    // Read {seq, data} without disturbing the pointer.
    task automatic peek_data(output [7:0] sq, output [15:0] v);
        logic [31:0] t;
        begin
            scan_begin(IR_PEEKD);
            for (int i = 0; i < 32; i++) begin
                t[i] = sim_tdo;
                tck_tick();
            end
            scan_end();
            v  = t[15:0];
            sq = t[23:16];
        end
    endtask

    // Ask for the word at the pointer and wait for it, rather than assuming it
    // will have arrived by the next scan. Whether it has is a race between TCK
    // and the system clock, and on hardware that race was observed going both
    // ways within one session -- readback that is sometimes one scan stale and
    // sometimes not is worse than useless for debugging.
    task automatic peek_word(output [15:0] v);
        logic [7:0] sq0, sq;
        logic [15:0] d;
        int guard;
        begin
            peek_data(sq0, d);
            scan_begin(IR_PEEK);                 // request; advances the pointer
            for (int i = 0; i < 32; i++) tck_tick();
            scan_end();
            guard = 0;
            forever begin
                peek_data(sq, d);
                if (sq !== sq0) break;
                guard++;
                if (guard > 20) begin
                    $display("FAIL: peek never completed");
                    errors++;
                    break;
                end
            end
            v = d;
        end
    endtask

    task automatic read_status(output [31:0] v);
        begin
            scan_begin(IR_STATUS);
            for (int i = 0; i < 32; i++) begin
                v[i] = sim_tdo;
                tck_tick();
            end
            scan_end();
        end
    endtask

    logic [31:0] st;
    logic [15:0] pv;
    int i, mism;

    initial begin
        repeat (4) @(negedge clk);
        rst_n = 1;
        repeat (300) @(negedge clk);       // SDRAM power-up

        // ---- the CPU hold ----
        chk("CPU not held out of reset", cpu_hold, 1'b0);
        set_ctrl(8'h01);
        repeat (10) @(negedge clk);
        chk("CTRL asserts the CPU hold", cpu_hold, 1'b1);

        // ---- write a run of words ----
        set_addr(24'h100000);
        send_words(64, 16'hA000);
        repeat (400) @(negedge clk);       // let the writer drain

        mism = 0;
        for (i = 0; i < 64; i++)
            if (chip.mem[midx(24'h100000 + i*2)] !== 16'hA000 + i[15:0]) mism++;
        chk("64 words landed in SDRAM", mism, 0);
        chk("the pointer auto-incremented",
            chip.mem[midx(24'h100000 + 63*2)], 16'hA03F);
        chk("nothing was written past the end",
            chip.mem[midx(24'h100000 + 64*2)], 16'h0000);

        // ---- the status readback ----
        read_status(st);
        chk("status reports every word written", st[23:0], 64);
        chk("status reports no overflow", st[31], 1'b0);

        // ---- a second run at a different address ----
        set_addr(24'h180000);
        send_words(32, 16'h5500);
        repeat (300) @(negedge clk);

        mism = 0;
        for (i = 0; i < 32; i++)
            if (chip.mem[midx(24'h180000 + i*2)] !== 16'h5500 + i[15:0]) mism++;
        chk("a second run landed at its own address", mism, 0);
        chk("the first run is undisturbed",
            chip.mem[midx(24'h100000)], 16'hA000);

        read_status(st);
        chk("the counter restarted with the address", st[23:0], 32);

        // ---- release the CPU ----
        set_ctrl(8'h00);
        repeat (10) @(negedge clk);
        chk("CTRL releases the CPU hold", cpu_hold, 1'b0);

        // ---- OVERFLOW: TCK far faster than the writer can drain ----
        // A word every 16 TCK ticks against an SDRAM write of roughly fifteen
        // system clocks. At this ratio the writer cannot keep up, and the
        // point of the test is that this is DETECTED rather than silently
        // corrupting the image.
        tckh = 1;
        set_addr(24'h140000);
        send_words(64, 16'h1200);
        repeat (600) @(negedge clk);

        tckh = 20;                          // slow down so status reads cleanly
        read_status(st);
        chk("overflow was detected", st[31], 1'b1);
        checks++;
        if (st[23:0] >= 64) begin
            $display("FAIL overflow was flagged but every word still arrived (%0d)",
                     st[23:0]);
            errors++;
        end else begin
            $display("  overflow run: %0d of 64 words written, flag set", st[23:0]);
        end

        // ---- and the loader recovers ----
        set_addr(24'h1C0000);
        send_words(16, 16'h7700);
        repeat (300) @(negedge clk);
        mism = 0;
        for (i = 0; i < 16; i++)
            if (chip.mem[midx(24'h1C0000 + i*2)] !== 16'h7700 + i[15:0]) mism++;
        chk("the loader works again after an overflow", mism, 0);
        read_status(st);
        chk("and the overflow flag cleared with the new address", st[31], 1'b0);

        // ---- reading memory back ----
        // Without this the loader is write-only and a wrong image can only be
        // diagnosed by what the CPU does with it.
        set_addr(24'h100000);
        peek_word(pv);
        chk("PEEK returned the first word written", pv, 16'hA000);
        peek_word(pv);
        chk("PEEK advanced to the second word", pv, 16'hA001);

        // ---- framing must not depend on the width of the chain's lead-in ----
        // This is the regression for the bug that made every word of a loaded
        // image come back rotated by seven bits. The old framing counted bits
        // from the first shift clock of the scan, so it was correct only when
        // the lead-in happened to be zero -- which is exactly what a testbench
        // that drives the node directly produces, and why it passed while the
        // hardware was thoroughly broken.
        for (int li = 0; li <= 9; li++) begin
            lead_in = li;
            set_addr(24'h180000);
            send_words(8, 16'hC300);
            repeat (300) @(negedge clk);
            mism = 0;
            for (i = 0; i < 8; i++)
                if (chip.mem[midx(24'h180000 + i*2)] !== 16'hC300 + i[15:0]) mism++;
            chk($sformatf("framing survives a %0d-bit lead-in", li), mism, 0);
        end
        lead_in = 7;

        chk("no SDRAM protocol errors", chip.errors, 0);

        $display("");
        $display("==================================");
        $display(" checks: %0d   failures: %0d", checks, errors);
        $display("==================================");
        if (errors == 0) $display("ALL TESTS PASSED");
        $finish;
    end

    initial begin
        #50000000;
        $display("FAIL global timeout");
        $display(" checks: %0d   failures: %0d", checks, errors + 1);
        $finish;
    end

endmodule
