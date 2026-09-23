`timescale 1ns/1ns
//
// The blitter, against the real framebuffer.
//
// Nothing here is modelled: the framebuffer is framebuffer.sv, so the byte
// lanes, the registered read and the write enables are the shipped ones. That
// matters because the lane arithmetic -- a byte offset picking a word and a
// half -- is exactly the kind of thing a testbench model gets wrong in the
// same way the design does.
//
// The CPU side of the framebuffer is shared, so this also drives `stall` to
// prove the blitter yields the port and resumes without losing or repeating a
// pixel. A blitter that dropped a write under contention would produce a
// picture with occasional wrong pixels -- the hardest kind of fault to see.
//
module tb_blitter;

    logic clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    logic        reg_sel = 0, reg_rd = 0, reg_wr = 0;
    logic [2:0]  reg_num = 0;
    logic [15:0] reg_wdata = 0, reg_rdata;
    logic        stall = 0, busy;

    logic [14:0] fb_addr;
    logic [15:0] fb_wdata, fb_rdata;
    logic [1:0]  fb_be;
    logic        fb_we;

    blitter dut (
        .clk (clk), .rst_n (rst_n),
        .reg_sel (reg_sel), .reg_num (reg_num), .reg_rd (reg_rd),
        .reg_wr (reg_wr), .reg_wdata (reg_wdata), .reg_rdata (reg_rdata),
        .stall (stall),
        .fb_addr (fb_addr), .fb_wdata (fb_wdata), .fb_be (fb_be),
        .fb_we (fb_we), .fb_rdata (fb_rdata),
        .busy (busy)
    );

    // The shared port, wired exactly as memory_controller wires it: when the
    // CPU wants the aperture it TAKES the port, and the blitter's outputs are
    // not connected to anything that cycle. Modelling `stall` as a mere signal
    // while leaving the blitter connected makes the arbitration untestable --
    // a blitter that ignored stall would write the same pixel twice to the
    // same address and nothing would notice.
    logic        cpu_we    = 1'b0;
    logic [14:0] cpu_addr  = 15'h7FF0;        // scratch, far from any test
    logic [15:0] cpu_wdata = 16'hDEAD;

    framebuffer #(.AW(15)) fb (
        .clk_cpu   (clk),
        .cpu_addr  (stall ? cpu_addr  : fb_addr),
        .cpu_wdata (stall ? cpu_wdata : fb_wdata),
        .cpu_we    (stall ? cpu_we    : fb_we),
        .cpu_be    (stall ? 2'b11     : fb_be),
        .cpu_rdata (fb_rdata),
        .clk_vga   (clk),
        .vga_addr  (15'd0),
        .vga_rdata ()
    );

    int errors = 0, checks = 0;
    task chk(input string nm, input int got, input int exp);
        checks++;
        if (got !== exp) begin
            $display("FAIL %-46s got=%04h exp=%04h", nm, got, exp);
            errors++;
        end
    endtask

    localparam logic [2:0] R_DST = 0, R_SRC = 1, R_WIDTH = 2, R_HEIGHT = 3,
                           R_DSTSTEP = 4, R_SRCSTEP = 5, R_COLOUR = 6,
                           R_CMD = 7;

    task automatic wr_reg(input [2:0] n, input [15:0] v);
        begin
            @(negedge clk);
            reg_sel = 1; reg_num = n; reg_wdata = v; reg_wr = 1;
            @(negedge clk);
            reg_wr = 0; reg_sel = 0;
        end
    endtask

    // Read a pixel straight out of the framebuffer's arrays, so the check does
    // not depend on the blitter being able to read it back.
    function automatic logic [7:0] pix(input int off);
        pix = off[0] ? fb.ram_hi[off >> 1] : fb.ram_lo[off >> 1];
    endfunction

    task automatic put(input int off, input [7:0] v);
        begin
            if (off[0]) fb.ram_hi[off >> 1] = v;
            else        fb.ram_lo[off >> 1] = v;
        end
    endtask

    int cyc;
    task automatic run_op(input [1:0] o);
        begin
            cyc = 0;
            wr_reg(R_CMD, {13'd0, o, 1'b1});
            while (busy && cyc < 200000) begin @(negedge clk); cyc++; end
        end
    endtask

    int i, x, y, bad;

    initial begin
        repeat (4) @(negedge clk);
        rst_n = 1;
        repeat (2) @(negedge clk);

        for (i = 0; i < 65536; i++) put(i, 8'h00);

        // ---- fill ----
        // Deliberately an ODD destination and an odd width, because the byte
        // lane is chosen by the low address bit and an even-only test would
        // pass against a blitter that ignored it entirely.
        wr_reg(R_DST,     16'd0321);
        wr_reg(R_WIDTH,   16'd5);
        wr_reg(R_HEIGHT,  16'd3);
        wr_reg(R_DSTSTEP, 16'd320);
        wr_reg(R_COLOUR,  16'h00AB);
        run_op(2'd0);
        chk("fill finished", busy, 1'b0);

        bad = 0;
        for (y = 0; y < 3; y++)
            for (x = 0; x < 5; x++)
                if (pix(321 + y * 320 + x) !== 8'hAB) bad++;
        chk("fill wrote every pixel of the rectangle", bad, 0);
        chk("fill left the pixel before it alone",  pix(320), 8'h00);
        chk("fill left the pixel after it alone",   pix(326), 8'h00);
        chk("fill did not spill onto the next row", pix(321 + 3 * 320), 8'h00);

        // ---- copy ----
        for (i = 0; i < 64; i++) put(4096 + i, 8'h40 + i[7:0]);
        wr_reg(R_SRC,     16'd4096);
        wr_reg(R_DST,     16'd8193);          // odd again: lanes must differ
        wr_reg(R_WIDTH,   16'd8);
        wr_reg(R_HEIGHT,  16'd4);
        wr_reg(R_SRCSTEP, 16'd8);             // a packed 8-wide bitmap...
        wr_reg(R_DSTSTEP, 16'd320);           // ...into a 320-wide screen
        run_op(2'd1);

        bad = 0;
        for (y = 0; y < 4; y++)
            for (x = 0; x < 8; x++)
                if (pix(8193 + y * 320 + x) !== 8'h40 + y * 8 + x) bad++;
        chk("copy moved every pixel, lanes and strides included", bad, 0);
        chk("copy did not disturb the source", pix(4096), 8'h40);

        // ---- transparent copy ----
        // The key must be skipped, not written as some other colour: whatever
        // was underneath has to survive.
        for (i = 0; i < 8; i++) put(4096 + i, (i % 2) ? 8'hFF : 8'h11);
        for (i = 0; i < 8; i++) put(12288 + i, 8'h77);
        wr_reg(R_SRC,     16'd4096);
        wr_reg(R_DST,     16'd12288);
        wr_reg(R_WIDTH,   16'd8);
        wr_reg(R_HEIGHT,  16'd1);
        wr_reg(R_COLOUR,  16'hFF00);          // high byte = the key
        run_op(2'd2);

        bad = 0;
        for (i = 0; i < 8; i++)
            if (pix(12288 + i) !== ((i % 2) ? 8'h77 : 8'h11)) bad++;
        chk("transparent copy skipped the key and kept the background", bad, 0);

        // ---- the CPU wins the port ----
        // Held off for a while, the blitter must produce exactly the same
        // result -- no pixel lost, none written twice.
        for (i = 0; i < 64; i++) put(16384 + i, 8'h00);
        wr_reg(R_DST,     16'd16384);
        wr_reg(R_WIDTH,   16'd16);
        wr_reg(R_HEIGHT,  16'd2);
        wr_reg(R_DSTSTEP, 16'd16);
        wr_reg(R_COLOUR,  16'h005A);
        fork
            run_op(2'd0);
            begin
                repeat (3) @(negedge clk);
                repeat (12) begin
                    stall = 1; cpu_we = 1; @(negedge clk);
                    stall = 0; cpu_we = 0; @(negedge clk);
                end
            end
        join
        bad = 0;
        for (i = 0; i < 32; i++) if (pix(16384 + i) !== 8'h5A) bad++;
        chk("a stalled blit still fills the whole rectangle", bad, 0);
        chk("and wrote nothing past it", pix(16384 + 32), 8'h00);

        // ---- a zero-sized rectangle does nothing ----
        put(20000, 8'h99);
        wr_reg(R_DST,    16'd20000);
        wr_reg(R_WIDTH,  16'd0);
        wr_reg(R_HEIGHT, 16'd4);
        wr_reg(R_COLOUR, 16'h00EE);
        run_op(2'd0);
        chk("a zero width draws nothing", pix(20000), 8'h99);
        chk("...and does not leave the blitter busy", busy, 1'b0);

        // ---- it is actually fast ----
        // A fill is one clock a pixel. The CPU needs about eight, so this is
        // the number the whole exercise is for.
        wr_reg(R_DST,     16'd24576);
        wr_reg(R_WIDTH,   16'd256);
        wr_reg(R_HEIGHT,  16'd16);
        wr_reg(R_DSTSTEP, 16'd256);
        wr_reg(R_COLOUR,  16'h0033);
        run_op(2'd0);
        $display("  fill of 4096 pixels took %0d clocks (%0.2f per pixel)",
                 cyc, cyc / 4096.0);
        checks++;
        if (cyc > 4096 + 64) begin
            $display("FAIL fill took %0d clocks for 4096 pixels", cyc);
            errors++;
        end

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
