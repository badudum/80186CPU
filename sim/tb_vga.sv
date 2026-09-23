`timescale 1ns/1ns
//
// VGA text-mode controller test.
//
// Two things are checked. First the raw 640x480@60 sync timing, which is what
// a monitor locks to. Second, and more importantly, that a character written
// into the text buffer renders at the correct pixel position -- the pixel
// pipeline runs a character group ahead of the display to cover VRAM and font
// ROM latency, and if that lookahead is off by one the entire screen shifts
// sideways by a character. That failure still produces a stable, plausible
// picture, so it is exactly the kind of bug that only a positional check
// catches.
//
module tb_vga;

    logic        clk = 0, rst_n = 0;
    logic [10:0] vram_addr;
    logic [15:0] vram_data;
    logic        mode_gfx = 0;
    logic [14:0] fb_addr;
    logic [15:0] fb_data;
    logic [7:0]  pal_index;
    logic [17:0] pal_rgb;
    logic        dac_wr = 0, dac_port = 0;
    logic [7:0]  dac_wdata = 0;
    logic        cursor_en = 0;
    logic [10:0] cursor_addr = 0;
    logic [7:0]  vga_r, vga_g, vga_b;
    logic        vga_hs, vga_vs, vga_blank_n, vga_sync_n, vga_clk;

    vga_controller dut (
        .clk_vga     (clk),
        .rst_n       (rst_n),
        .vram_addr   (vram_addr),
        .vram_data   (vram_data),
        .mode_gfx    (mode_gfx),
        .fb_addr     (fb_addr),
        .fb_data     (fb_data),
        .pal_index   (pal_index),
        .pal_rgb     (pal_rgb),
        .cursor_en   (cursor_en),
        .cursor_addr (cursor_addr),
        .vga_r (vga_r), .vga_g (vga_g), .vga_b (vga_b),
        .vga_hs (vga_hs), .vga_vs (vga_vs),
        .vga_blank_n (vga_blank_n), .vga_sync_n (vga_sync_n),
        .vga_clk (vga_clk)
    );

    always #5 clk = ~clk;

    // Text buffer model with the same one-cycle read latency as vram.sv.
    logic [15:0] vram_mem [0:2047];
    always_ff @(posedge clk) vram_data <= vram_mem[vram_addr];

    // Framebuffer model, same one-cycle read latency as framebuffer.sv.
    logic [15:0] fb_mem [0:32767];
    always_ff @(posedge clk) fb_data <= fb_mem[fb_addr];

    // The real palette DAC, so the index-to-colour path is the shipped one
    // rather than a testbench approximation of it.
    vga_dac u_dac (
        .clk_cpu   (clk), .rst_n (rst_n),
        .dac_wr    (dac_wr), .dac_port (dac_port), .dac_wdata (dac_wdata),
        .dac_rdata (), .clk_vga (clk),
        .pal_index (pal_index), .pal_rgb (pal_rgb)
    );

    // Write one palette entry the way software does: index to 3C8, then three
    // components to 3C9.
    task automatic set_colour(input [7:0] idx, input [5:0] r,
                              input [5:0] g, input [5:0] b);
        begin
            @(negedge clk); dac_port = 1'b0; dac_wdata = idx;      dac_wr = 1;
            @(negedge clk); dac_port = 1'b1; dac_wdata = {2'b0, r};
            @(negedge clk);                  dac_wdata = {2'b0, g};
            @(negedge clk);                  dac_wdata = {2'b0, b};
            @(negedge clk); dac_wr = 0;
        end
    endtask

    int errors = 0, checks = 0;
    task chk(input string nm, input int got, input int exp);
        checks++;
        if (got !== exp) begin
            $display("FAIL %-40s got=%02h exp=%02h", nm, got, exp);
            errors++;
        end
    endtask

    // All waiting and sampling happens on the NEGEDGE. Reading dut.h_cnt right
    // after a posedge races the non-blocking update of the counter, so the
    // loop would exit a cycle late and every sample would be skewed by one
    // pixel. Mid-cycle the counters and the combinational outputs derived from
    // them are both stable.
    task wait_h(input int hh);
        begin while (dut.h_cnt != hh) @(negedge clk); end
    endtask

    task wait_v(input int vv);
        begin while (dut.v_cnt != vv) @(negedge clk); end
    endtask

    task sample_at(input int hh, input int vv, output [7:0] r);
        begin
            while (!((dut.h_cnt == hh) && (dut.v_cnt == vv))) @(negedge clk);
            r = vga_r;
        end
    endtask

    localparam int TEXT_TOP = 40;
    localparam int V_TOT    = 525;
    localparam int TEXT_BOT = TEXT_TOP + 400;
    localparam [7:0] FG = 8'hAA;   // attribute 7 foreground: light grey
    localparam [7:0] BG = 8'h00;   // attribute 0 background: black

    logic [7:0] px;
    int i;

    // Sample every character cell of the text area and confirm only `lit` is
    // on. Two pixels per cell: the first, where a character-wide misplacement
    // shows, and the fifth, where a sub-character one does.
    // Sample all three channels; the graphics tests care about colour, not
    // just whether a pixel is lit.
    task automatic sample_rgb(input int hh, input int vv,
                              output [7:0] r, output [7:0] g, output [7:0] b);
        begin
            while (!((dut.h_cnt == hh) && (dut.v_cnt == vv))) @(negedge clk);
            r = vga_r; g = vga_g; b = vga_b;
        end
    endtask

    // One byte per pixel, packed left to right and top to bottom, so two
    // horizontally adjacent pixels share a word.
    task automatic set_pixel(input int col, input int row, input [7:0] idx);
        int off;
        begin
            off = row * 320 + col;
            if (off[0]) fb_mem[off >> 1][15:8] = idx;
            else        fb_mem[off >> 1][7:0]  = idx;
        end
    endtask

    task automatic chk_rgb(input string nm, input int hh, input int vv,
                           input [7:0] er, input [7:0] eg, input [7:0] eb);
        logic [7:0] r, g, b;
        begin
            sample_rgb(hh, vv, r, g, b);
            checks++;
            if (r !== er || g !== eg || b !== eb) begin
                $display("FAIL %-44s got=%02h%02h%02h exp=%02h%02h%02h",
                         nm, r, g, b, er, eg, eb);
                errors++;
            end
        end
    endtask

    task automatic sweep(input string nm, input int lit);
        int row, col, bad;
        logic [7:0] a, b;
        begin
            bad = 0;
            // Start from a fresh frame so the buffer rewrite above is fully in
            // effect before anything is sampled.
            wait_v(V_TOT - 1);
            wait_v(0);
            for (row = 0; row < 25; row++)
                for (col = 0; col < 80; col++) begin
                    sample_at(col*8 + 0, TEXT_TOP + row*16 + 3, a);
                    sample_at(col*8 + 4, TEXT_TOP + row*16 + 3, b);
                    if ((row*80 + col) == lit) begin
                        if (a !== FG || b !== FG) bad++;
                    end else begin
                        if (a !== BG || b !== BG) begin
                            if (bad < 6)
                                $display("  ghost at row %0d col %0d: %02h %02h",
                                         row, col, a, b);
                            bad++;
                        end
                    end
                end
            chk($sformatf("frame is clean with only %s lit", nm), bad, 0);
        end
    endtask

    initial begin
        // Blank the buffer, then place two characters on the top text row.
        for (i = 0; i < 2048; i++) vram_mem[i] = 16'h0720;    // space, grey on black
        vram_mem[0] = 16'h0741;     // 'A' at row 0, col 0
        vram_mem[1] = 16'h0742;     // 'B' at row 0, col 1
        vram_mem[80] = 16'h0743;    // 'C' at row 1, col 0

        // Synthetic font: distinctive, easy to predict per pixel.
        //   41h 'A' -> 10000001 on every scanline
        //   42h 'B' -> 11111111
        //   43h 'C' -> 00011000
        //   20h ' ' -> 00000000
        for (i = 0; i < 16; i++) begin
            dut.u_font.g_inferred.rom[{8'h41, i[3:0]}] = 8'b10000001;
            dut.u_font.g_inferred.rom[{8'h42, i[3:0]}] = 8'b11111111;
            dut.u_font.g_inferred.rom[{8'h43, i[3:0]}] = 8'b00011000;
            dut.u_font.g_inferred.rom[{8'h20, i[3:0]}] = 8'b00000000;
        end

        repeat (4) @(negedge clk);
        rst_n = 1;

        // ---------------- sync timing ----------------
        // horizontal sync occupies 656..751, active low
        wait_h(655);
        chk("hs inactive before the pulse", vga_hs, 1'b1);
        @(negedge clk);
        chk("hs asserted at 656", vga_hs, 1'b0);
        wait_h(751);
        chk("hs still asserted at 751", vga_hs, 1'b0);
        @(negedge clk);
        chk("hs released at 752", vga_hs, 1'b1);

        // blanking
        while (!((dut.h_cnt == 639) && (dut.v_cnt == 100))) @(negedge clk);
        chk("blank_n high at last visible pixel", vga_blank_n, 1'b1);
        @(negedge clk);
        chk("blank_n low once past 640", vga_blank_n, 1'b0);

        // vertical sync occupies lines 490..491, active low
        wait_v(489);
        chk("vs inactive before the pulse", vga_vs, 1'b1);
        wait_v(490);
        chk("vs asserted on line 490", vga_vs, 1'b0);
        wait_v(492);
        chk("vs released on line 492", vga_vs, 1'b1);

        // nothing is drawn above the centred text area
        sample_at(0, 10, px);
        chk("border above the text area is blank", px, BG);

        // ---------------- pixel pipeline placement ----------------
        // 'A' is 10000001: lit at column 0 and column 7 of its cell only.
        sample_at(0, TEXT_TOP, px);
        chk("char 0 pixel 0 lit", px, FG);
        sample_at(1, TEXT_TOP, px);
        chk("char 0 pixel 1 dark", px, BG);
        sample_at(6, TEXT_TOP, px);
        chk("char 0 pixel 6 dark", px, BG);
        sample_at(7, TEXT_TOP, px);
        chk("char 0 pixel 7 lit", px, FG);

        // 'B' is all ones, and must start exactly at pixel 8 -- this is the
        // check that fails if the lookahead is off by a character.
        sample_at(8,  TEXT_TOP, px);
        chk("char 1 pixel 0 lit", px, FG);
        sample_at(15, TEXT_TOP, px);
        chk("char 1 pixel 7 lit", px, FG);

        // column 2 onwards is spaces
        sample_at(16, TEXT_TOP, px);
        chk("char 2 is blank", px, BG);
        sample_at(23, TEXT_TOP, px);
        chk("char 2 stays blank", px, BG);

        // the same row renders identically on its other scanlines
        sample_at(0, TEXT_TOP + 5, px);
        chk("row 0 repeats on scanline 5", px, FG);
        sample_at(1, TEXT_TOP + 5, px);
        chk("row 0 scanline 5 pixel 1 dark", px, BG);

        // ---------------- second text row ----------------
        // 'C' is 00011000: lit only at columns 3 and 4.
        sample_at(2, TEXT_TOP + 16, px);
        chk("row 1 char 0 pixel 2 dark", px, BG);
        sample_at(3, TEXT_TOP + 16, px);
        chk("row 1 char 0 pixel 3 lit", px, FG);
        sample_at(4, TEXT_TOP + 16, px);
        chk("row 1 char 0 pixel 4 lit", px, FG);
        sample_at(5, TEXT_TOP + 16, px);
        chk("row 1 char 0 pixel 5 dark", px, BG);

        // ---------------- cursor ----------------
        // Force the blink phase on rather than waiting sixteen frames.
        cursor_en   = 1'b1;
        cursor_addr = 11'd2;                   // row 0, column 2 (a space)
        dut.blink_cnt = 5'b10000;

        sample_at(16, TEXT_TOP + 14, px);
        chk("cursor lights its cell on line 14", px, FG);
        sample_at(16, TEXT_TOP + 5, px);
        chk("cursor absent on upper scanlines", px, BG);
        // A different blank cell on the same scanline must stay dark, which is
        // what actually proves the cursor is confined -- checking a cell that
        // holds a lit glyph would pass whether the cursor leaked or not.
        sample_at(24, TEXT_TOP + 14, px);
        chk("cursor confined to its own cell", px, BG);

        dut.blink_cnt = 5'b00000;
        sample_at(16, TEXT_TOP + 14, px);
        chk("cursor dark on the off blink phase", px, BG);

        // ---------------- ghosting sweep ----------------
        // One lit cell, everything else a space, and then EVERY cell of the
        // frame is sampled. A stray character that the text buffer does not
        // contain has to come from a cell being fetched or displayed where it
        // does not belong, and a positional check of a single character cannot
        // see that. The lit cell is put last as well as first, because a fetch
        // that runs ahead of the display wraps past the end of the buffer and
        // can ghost into the top of the next frame.
        for (i = 0; i < 2048; i++) vram_mem[i] = 16'h0720;
        vram_mem[0] = 16'h0742;
        sweep("cell 0", 0);

        for (i = 0; i < 2048; i++) vram_mem[i] = 16'h0720;
        vram_mem[24*80 + 79] = 16'h0742;
        sweep("last cell", 24*80 + 79);

        // ---------------- graphics mode ----------------
        // Mode 13h is 320x200 shown as 640x400: every pixel twice as wide and
        // twice as tall, in the same 400-line window the text mode uses. The
        // doubling is the part worth checking -- an off-by-one there gives a
        // picture that looks plausible and is wrong everywhere.
        for (i = 0; i < 32768; i++) fb_mem[i] = 16'h0000;

        set_colour(8'd1, 6'd63, 6'd0,  6'd0 );      // pure red
        set_colour(8'd2, 6'd0,  6'd63, 6'd0 );      // pure green
        set_colour(8'd3, 6'd0,  6'd0,  6'd63);      // pure blue
        set_colour(8'd4, 6'd21, 6'd42, 6'd63);      // checks the 6->8 expansion

        set_pixel(0,   0,   8'd1);
        set_pixel(1,   0,   8'd2);
        set_pixel(319, 0,   8'd3);
        set_pixel(0,   199, 8'd4);

        mode_gfx = 1'b1;
        wait_v(V_TOT - 1);
        wait_v(0);

        chk_rgb("gfx pixel 0 is its palette colour",     0, TEXT_TOP,       8'hFF, 8'h00, 8'h00);
        chk_rgb("gfx pixel 0 is two dots wide",          1, TEXT_TOP,       8'hFF, 8'h00, 8'h00);
        chk_rgb("gfx pixel 0 is two lines tall",         0, TEXT_TOP + 1,   8'hFF, 8'h00, 8'h00);
        chk_rgb("gfx pixel 1 starts at dot 2",           2, TEXT_TOP,       8'h00, 8'hFF, 8'h00);
        chk_rgb("gfx pixel 1 ends at dot 3",             3, TEXT_TOP,       8'h00, 8'hFF, 8'h00);
        chk_rgb("gfx pixel 2 is background",             4, TEXT_TOP,       8'h00, 8'h00, 8'h00);
        chk_rgb("gfx last column reaches dot 638",     638, TEXT_TOP,       8'h00, 8'h00, 8'hFF);
        chk_rgb("gfx last column reaches dot 639",     639, TEXT_TOP,       8'h00, 8'h00, 8'hFF);
        chk_rgb("gfx row 1 is not row 0",                0, TEXT_TOP + 2,   8'h00, 8'h00, 8'h00);
        chk_rgb("gfx last row starts at line 398",       0, TEXT_TOP + 398, 8'h55, 8'hAA, 8'hFF);
        chk_rgb("gfx last row ends at line 399",         0, TEXT_TOP + 399, 8'h55, 8'hAA, 8'hFF);
        chk_rgb("gfx border above the image is black",   0, TEXT_TOP - 1,   8'h00, 8'h00, 8'h00);
        chk_rgb("gfx border below the image is black",   0, TEXT_BOT,       8'h00, 8'h00, 8'h00);

        // A palette write must show without touching the framebuffer: that is
        // how a game fades the screen.
        set_colour(8'd1, 6'd0, 6'd63, 6'd63);
        wait_v(V_TOT - 1);
        wait_v(0);
        chk_rgb("reloading the palette recolours the image", 0, TEXT_TOP,
                8'h00, 8'hFF, 8'hFF);

        // Back to text: the character generator must still work afterwards.
        mode_gfx = 1'b0;
        for (i = 0; i < 2048; i++) vram_mem[i] = 16'h0720;
        vram_mem[0] = 16'h0742;
        sweep("text after graphics", 0);

        $display("");
        $display("==================================");
        $display(" checks: %0d   failures: %0d", checks, errors);
        $display("==================================");
        if (errors == 0) $display("ALL TESTS PASSED");
        $finish;
    end

    // The graphics tests and three whole-frame sweeps are several frames of
    // simulated time each; the old budget was sized for the positional checks
    // alone.
    initial begin
        #200000000;
        $display("FAIL global timeout");
        $display(" checks: %0d   failures: %0d", checks, errors + 1);
        $finish;
    end

endmodule
