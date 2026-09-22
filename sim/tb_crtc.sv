`timescale 1ns/1ns
//
// 6845 CRTC cursor registers.
//
// Small, but it is what stands between the BIOS and a cursor that follows the
// text it prints, so the things checked here are the ones INT 10h depends on:
// the index/data pair actually indexes, the 11-bit address is assembled from
// two byte registers in the right order, and bit 5 of register 0A turns the
// cursor off rather than on.
//
module tb_crtc;

    logic       clk = 0, rst_n = 0;
    logic       sel = 0, port = 0, rd = 0, wr = 0;
    logic [7:0] wdata = 8'h00, rdata;
    logic        cursor_en;
    logic [10:0] cursor_addr;

    crtc dut (
        .clk (clk), .rst_n (rst_n),
        .sel (sel), .port (port), .rd (rd), .wr (wr),
        .wdata (wdata), .rdata (rdata),
        .cursor_en (cursor_en), .cursor_addr (cursor_addr)
    );

    always #5 clk = ~clk;

    int errors = 0, checks = 0;
    task chk(input string nm, input int got, input int exp);
        checks++;
        if (got !== exp) begin
            $display("FAIL %-42s got=%04h exp=%04h", nm, got, exp);
            errors++;
        end
    endtask

    // `wr` is a single-cycle strobe, as io_decode delivers it.
    task wr_reg(input logic p, input [7:0] v);
        begin
            @(negedge clk); sel = 1; port = p; wdata = v;
            @(negedge clk); wr = 1;
            @(negedge clk); wr = 0; sel = 0;
            @(negedge clk);
        end
    endtask

    task rd_reg(input logic p, output [7:0] v);
        begin
            @(negedge clk); sel = 1; port = p;
            @(negedge clk); rd = 1; v = rdata;
            @(negedge clk); rd = 0; sel = 0;
            @(negedge clk);
        end
    endtask

    // The idiom BIOS code uses: index to 3D4, value to 3D5.
    task set_reg(input [4:0] idx, input [7:0] v);
        begin
            wr_reg(1'b0, {3'b000, idx});
            wr_reg(1'b1, v);
        end
    endtask

    logic [7:0] v;

    initial begin
        repeat (4) @(negedge clk);
        rst_n = 1;
        @(negedge clk);

        // ---- power-up state ----
        // A visible cursor at cell 0 before any software runs is the only
        // proof the video path is alive when the font has not loaded.
        chk("cursor enabled out of reset", cursor_en, 1'b1);
        chk("cursor at cell 0 out of reset", cursor_addr, 11'd0);

        // ---- the index register indexes ----
        wr_reg(1'b0, 8'h0E);
        rd_reg(1'b0, v);
        chk("index reads back", v, 8'h0E);

        // ---- an 11-bit address out of two byte registers ----
        // Row 5, column 10 on an 80-column screen is cell 410 = 019Ah.
        set_reg(5'h0E, 8'h01);          // high
        set_reg(5'h0F, 8'h9A);          // low
        chk("cursor address assembled", cursor_addr, 11'h19A);

        // Writing only the low byte must move the cursor within the row.
        set_reg(5'h0F, 8'h9B);
        chk("low byte alone moves the cursor", cursor_addr, 11'h19B);

        // The last cell of an 80x25 screen is 1999 = 7CFh.
        set_reg(5'h0E, 8'h07);
        set_reg(5'h0F, 8'hCF);
        chk("last cell of the screen", cursor_addr, 11'h7CF);

        // ---- the address is 11 bits, not 14 ----
        // A larger value must truncate rather than wrap somewhere surprising.
        set_reg(5'h0E, 8'hFF);
        set_reg(5'h0F, 8'hFF);
        chk("address truncates to 11 bits", cursor_addr, 11'h7FF);

        // ---- bit 5 of register 0A turns the cursor OFF ----
        set_reg(5'h0A, 8'h20);
        chk("bit 5 disables the cursor", cursor_en, 1'b0);
        set_reg(5'h0A, 8'h0E);
        chk("clearing bit 5 re-enables it", cursor_en, 1'b1);

        // ---- registers read back through the data port ----
        set_reg(5'h0A, 8'h06);
        wr_reg(1'b0, 8'h0A);
        rd_reg(1'b1, v);
        chk("cursor start reads back", v, 8'h06);

        // ---- a full CRTC table write must not fault ----
        // Real code writes all sixteen registers. The timing ones do nothing
        // here, but they must be accepted and must not disturb the cursor.
        set_reg(5'h0E, 8'h00);
        set_reg(5'h0F, 8'h50);          // cell 80 = row 1, column 0
        for (int i = 0; i < 10; i++) set_reg(i[4:0], 8'h55);
        set_reg(5'h0C, 8'h00);
        set_reg(5'h0D, 8'h00);
        chk("timing registers left the cursor alone", cursor_addr, 11'd80);
        chk("timing registers left it enabled", cursor_en, 1'b1);

        $display("");
        $display("==================================");
        $display(" checks: %0d   failures: %0d", checks, errors);
        $display("==================================");
        if (errors == 0) $display("ALL TESTS PASSED");
        $finish;
    end

    initial begin
        #500000;
        $display("FAIL global timeout");
        $display(" checks: %0d   failures: %0d", checks, errors + 1);
        $finish;
    end

endmodule
