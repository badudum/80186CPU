`timescale 1ns/1ns
//
// The block device backed by SDRAM rather than on-chip ROM.
//
// This is the whole stack the real design uses: storage -> sdram_arbiter ->
// sdram_controller -> a protocol-checking SDRAM model. Nothing is stubbed
// between the register interface and the memory chip pins, so a mistake in the
// address arithmetic, the arbiter handshake or the controller shows up here.
//
// The arbiter is instantiated with THREE ports and the device on port 1, which
// is where memory_controller actually puts it. Testing it on port 0 would have
// worked even if the index mapping were wrong.
//
// The model is preloaded directly, which is exactly what the JTAG loader will
// do on hardware: SDRAM is volatile, so something has to put an image there
// before the machine can boot from it.
//
// WRITES ARE THE POINT. The ROM backend cannot have them, so INT 13h has no
// write function today. Here a sector is read, modified through the data port,
// written back, and read again -- and the model's memory is checked directly
// to confirm the data really reached the chip rather than just the buffer.
//
module tb_storage_sdram;

    localparam int  SECTORS = 64;
    localparam logic [23:0] BASE = 24'h100000;

    // Byte address -> index in the model's array. The controller splits a word
    // address into col = [9:0] and row = [22:10], and the model indexes by
    // {row, col}, so the two collapse back to word_addr[20:0].
    function automatic int unsigned midx(input int unsigned byte_addr);
        midx = (byte_addr >> 1) & 21'h1FFFFF;
    endfunction

    logic clk = 0, rst_n = 0;

    // ---- device register interface ----
    logic        sel = 0, rd = 0, wr = 0;
    logic [2:0]  reg_sel = 3'd0;
    logic [15:0] wdata = 16'h0000, rdata;

    // ---- device SDRAM port ----
    logic [23:0] st_addr;
    logic [15:0] st_wdata;
    logic [1:0]  st_be;
    logic        st_rd, st_wr;

    // ---- arbiter signals, declared before the device that reads them ----
    logic [2:0]  arb_rd, arb_wr, arb_ready;
    logic [23:0] arb_addr  [3];
    logic [15:0] arb_wdata [3];
    logic [1:0]  arb_be    [3];
    logic [15:0] arb_rdata;

    storage #(.SECTORS(SECTORS), .USE_SDRAM(1'b1), .BASE(BASE)) dut (
        .clk (clk), .rst_n (rst_n),
        .sel (sel), .reg_sel (reg_sel), .rd (rd), .wr (wr),
        .wdata (wdata), .rdata (rdata),
        .mem_addr (st_addr), .mem_wdata (st_wdata), .mem_be (st_be),
        .mem_rd (st_rd), .mem_wr (st_wr),
        .mem_rdata (arb_rdata), .mem_ready (arb_ready[1])
    );

    // ---- arbiter, device on port 1 as in memory_controller ----
    assign arb_rd    = {1'b0, st_rd, 1'b0};
    assign arb_wr    = {1'b0, st_wr, 1'b0};
    always_comb begin
        arb_addr[0]  = 24'h000000; arb_wdata[0] = 16'h0000; arb_be[0] = 2'b11;
        arb_addr[1]  = st_addr;    arb_wdata[1] = st_wdata; arb_be[1] = st_be;
        arb_addr[2]  = 24'h000000; arb_wdata[2] = 16'h0000; arb_be[2] = 2'b11;
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
        .dram_clk (dram_clk)
    );

    sdram_model #(.CAS_LATENCY(2)) chip (
        .dram_clk (dram_clk), .dram_cke (dram_cke), .dram_cs_n (dram_cs_n),
        .dram_ras_n (dram_ras_n), .dram_cas_n (dram_cas_n), .dram_we_n (dram_we_n),
        .dram_addr (dram_addr), .dram_ba (dram_ba),
        .dram_dqm (dram_dqm), .dram_dq (dram_dq)
    );

    always #5 clk = ~clk;

    localparam logic [2:0] R_DATA   = 3'd0;
    localparam logic [2:0] R_LBA_LO = 3'd1;
    localparam logic [2:0] R_LBA_HI = 3'd2;
    localparam logic [2:0] R_CMD    = 3'd3;

    int errors = 0, checks = 0;
    task chk(input string nm, input int got, input int exp);
        checks++;
        if (got !== exp) begin
            $display("FAIL %-46s got=%04h exp=%04h", nm, got, exp);
            errors++;
        end
    endtask

    task reg_write(input [2:0] r, input [15:0] v);
        begin
            @(negedge clk); sel = 1; reg_sel = r; wdata = v;
            @(negedge clk); wr = 1;
            @(negedge clk); wr = 0; sel = 0;
            @(negedge clk);
        end
    endtask

    task reg_read(input [2:0] r, output [15:0] v);
        begin
            @(negedge clk); sel = 1; reg_sel = r;
            @(negedge clk); rd = 1; v = rdata;
            @(negedge clk); rd = 0; sel = 0;
            @(negedge clk);
        end
    endtask

    int busy_polls;
    task issue(input [23:0] lba, input [15:0] cmd);
        logic [15:0] st;
        begin
            reg_write(R_LBA_LO, lba[15:0]);
            reg_write(R_LBA_HI, {8'h00, lba[23:16]});
            reg_write(R_CMD, cmd);
            busy_polls = 0;
            reg_read(R_CMD, st);
            while (st[0]) begin
                busy_polls++;
                reg_read(R_CMD, st);
            end
        end
    endtask

    logic [15:0] v, st;
    int i, s, mism;

    initial begin
        // ---- preload, as the JTAG loader will ----
        // Every sector carries its own number so a wrong LBA is obvious.
        for (s = 0; s < SECTORS; s++)
            for (i = 0; i < 256; i++)
                chip.mem[midx(BASE + s*512 + i*2)] = 16'((s << 8) | (i & 8'hFF));

        repeat (4) @(negedge clk);
        rst_n = 1;
        // let the controller finish its power-up sequence
        repeat (300) @(negedge clk);

        // ---- read sector 0 ----
        issue(24'd0, 16'h0001);
        chk("BUSY was observed during the transfer", (busy_polls > 0), 1'b1);
        reg_read(R_CMD, st);
        chk("read completed without error", st[2], 1'b0);
        chk("data is on offer", st[1], 1'b1);

        mism = 0;
        for (i = 0; i < 256; i++) begin
            reg_read(R_DATA, v);
            if (v !== 16'((0 << 8) | i)) mism++;
        end
        chk("sector 0 came back intact", mism, 0);

        reg_read(R_CMD, st);
        chk("data withdrawn after 256 words", st[1], 1'b0);

        // ---- a different sector really is different ----
        issue(24'd5, 16'h0001);
        reg_read(R_DATA, v); chk("sector 5 word 0", v, 16'h0500);
        reg_read(R_DATA, v); chk("sector 5 word 1", v, 16'h0501);

        issue(SECTORS - 1, 16'h0001);
        reg_read(R_DATA, v); chk("last sector word 0", v, 16'h3F00);

        // ---- WRITE: read, modify, write back ----
        issue(24'd7, 16'h0001);
        reg_read(R_DATA, v); chk("sector 7 before the write", v, 16'h0700);

        // Refill the whole buffer through the data port, then flush it.
        issue(24'd7, 16'h0001);
        for (i = 0; i < 256; i++) reg_write(R_DATA, 16'hC000 + i[15:0]);
        issue(24'd7, 16'h0002);
        reg_read(R_CMD, st);
        chk("write completed without error", st[2], 1'b0);
        chk("a write leaves no data on offer", st[1], 1'b0);

        // Did it actually reach the chip, or only the buffer?
        chk("word 0 reached SDRAM", chip.mem[midx(BASE + 7*512)],      16'hC000);
        chk("word 1 reached SDRAM", chip.mem[midx(BASE + 7*512 + 2)],  16'hC001);
        chk("word 255 reached SDRAM", chip.mem[midx(BASE + 7*512 + 510)], 16'hC0FF);

        // Neighbouring sectors must be untouched.
        chk("sector 6 undisturbed", chip.mem[midx(BASE + 6*512)], 16'h0600);
        chk("sector 8 undisturbed", chip.mem[midx(BASE + 8*512)], 16'h0800);

        // ---- and read it back through the device ----
        issue(24'd7, 16'h0001);
        mism = 0;
        for (i = 0; i < 256; i++) begin
            reg_read(R_DATA, v);
            if (v !== 16'hC000 + i[15:0]) mism++;
        end
        chk("the written sector reads back", mism, 0);

        // ---- out of range is refused ----
        issue(SECTORS, 16'h0001);
        reg_read(R_CMD, st);
        chk("out-of-range read sets ERR", st[2], 1'b1);
        chk("out-of-range read offers nothing", st[1], 1'b0);

        // A refused write must not have touched memory either.
        issue(SECTORS + 4, 16'h0002);
        reg_read(R_CMD, st);
        chk("out-of-range write sets ERR", st[2], 1'b1);

        // ---- recovery ----
        issue(24'd5, 16'h0001);
        reg_read(R_CMD, st);
        chk("a valid command clears ERR", st[2], 1'b0);
        reg_read(R_DATA, v); chk("and reads correctly again", v, 16'h0500);

        chk("no SDRAM protocol errors", chip.errors, 0);

        $display("");
        $display("==================================");
        $display(" checks: %0d   failures: %0d", checks, errors);
        $display("==================================");
        if (errors == 0) $display("ALL TESTS PASSED");
        $finish;
    end

    initial begin
        #20000000;
        $display("FAIL global timeout");
        $display(" checks: %0d   failures: %0d", checks, errors + 1);
        $finish;
    end

endmodule
