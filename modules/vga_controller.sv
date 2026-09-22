// ---------------------------------------------------------------------------
// vga_controller — 80x25 text-mode video output (project addition).
//
// Hierarchy: FPGA80186 -> vga_controller -> font_rom
// Reference: DE1-SoC User Manual (VGA pins, ADV7123 DAC)
// Testbench: sim/tb_vga.sv
//
// MODE: 640x480 @ 60 Hz, 25.175 MHz pixel clock. That is the easiest mode to
// get a monitor to accept, and with an 8x16 font it gives 80 columns of 30
// rows. PC text mode is 80x25, so the 400 used lines are centred with a
// 40-line border top and bottom rather than stretching the buffer.
//
//   horizontal: 640 visible, 16 front porch, 96 sync, 48 back porch = 800
//   vertical:   480 visible, 10 front porch,  2 sync, 33 back porch = 525
//   both sync pulses are ACTIVE LOW in this mode
//
// PIXEL PIPELINE: this is the part that has to be right. Both the VRAM and the
// font ROM register their outputs, so a character cannot be fetched in the
// same 8-pixel group it is displayed in -- it must be fetched during the
// PREVIOUS group. The fetch address therefore runs 8 pixels ahead of the
// display position, which also handles the line wrap naturally: during the
// last 8 pixels of a scanline the lookahead rolls into the next scanline, so
// column 0 of every line is fetched before it is needed, including the first
// line of the frame.
//
//   phase 0   fetch address changes (combinational from the lookahead)
//   phase 1   VRAM data valid -> latch character and attribute
//   phase 2   font address valid from the latched character
//   phase 3   font data valid -> latch the glyph row
//   phase 7   load the shift register; the glyph displays over the next group
//
// Getting this wrong shifts the whole screen sideways by a character, which is
// the classic symptom of an under-pipelined text-mode controller.
//
// ADV7123 NOTES: VGA_BLANK_N must be low throughout blanking or the monitor
// shows nothing, which is the usual "it doesn't work" cause on this board.
// VGA_SYNC_N is tied low because sync-on-green is unused. VGA_CLK clocks the
// DAC and is driven with the pixel clock.
//
// NOT IMPLEMENTED: graphics modes (they need far more VRAM than fits on-chip),
// the 9th column that real VGA text mode stretches to for box-drawing glyphs,
// and blink-as-blink for attribute bit 7 -- that bit is treated as bright
// background, which is the other standard interpretation and avoids a second
// blink timer.
// ---------------------------------------------------------------------------

`timescale 1ns/1ns
module vga_controller #(
    // Passed straight to font_rom. It must name a real file: an empty default
    // here silently overrides font_rom's own default and the screen renders
    // blank while every other check still passes.
    parameter FONT_FILE = "rom/font.hex",
    parameter bit ISMCE = 1'b0
) (
    input  logic        clk_vga,
    input  logic        rst_n,

    // text buffer read port (independent clock domain inside vram)
    output logic [10:0] vram_addr,
    input  logic [15:0] vram_data,

    // hardware cursor, linear cell position
    input  logic        cursor_en,
    input  logic [10:0] cursor_addr,

    // VGA output
    output logic [7:0]  vga_r,
    output logic [7:0]  vga_g,
    output logic [7:0]  vga_b,
    output logic        vga_hs,
    output logic        vga_vs,
    output logic        vga_blank_n,
    output logic        vga_sync_n,
    output logic        vga_clk
);

    // ---- 640x480 @ 60 Hz ----
    localparam int H_VIS = 640, H_FP = 16, H_SYNC = 96, H_BP = 48;
    localparam int H_TOT = H_VIS + H_FP + H_SYNC + H_BP;      // 800
    localparam int V_VIS = 480, V_FP = 10, V_SYNC = 2,  V_BP = 33;
    localparam int V_TOT = V_VIS + V_FP + V_SYNC + V_BP;      // 525

    localparam int TEXT_TOP  = 40;                            // centres 25 rows
    localparam int TEXT_ROWS = 25;
    localparam int TEXT_COLS = 80;
    localparam int TEXT_BOT  = TEXT_TOP + TEXT_ROWS * 16;     // 440

    logic [9:0] h_cnt, v_cnt;

    always_ff @(posedge clk_vga or negedge rst_n) begin
        if (!rst_n) begin
            h_cnt <= 10'd0;
            v_cnt <= 10'd0;
        end else if (h_cnt == H_TOT - 1) begin
            h_cnt <= 10'd0;
            v_cnt <= (v_cnt == V_TOT - 1) ? 10'd0 : (v_cnt + 10'd1);
        end else begin
            h_cnt <= h_cnt + 10'd1;
        end
    end

    logic h_active, v_active, active;
    assign h_active = (h_cnt < H_VIS);
    assign v_active = (v_cnt < V_VIS);
    assign active   = h_active && v_active;

    assign vga_hs = ~((h_cnt >= H_VIS + H_FP) && (h_cnt < H_VIS + H_FP + H_SYNC));
    assign vga_vs = ~((v_cnt >= V_VIS + V_FP) && (v_cnt < V_VIS + V_FP + V_SYNC));
    assign vga_blank_n = active;
    assign vga_sync_n  = 1'b0;
    assign vga_clk     = clk_vga;

    // ---- fetch position, one character group ahead of the display ----
    logic [10:0] h_look;
    logic        wraps;
    logic [9:0]  f_h, f_v;

    assign h_look = {1'b0, h_cnt} + 11'd8;
    assign wraps  = (h_look >= H_TOT);
    assign f_h    = wraps ? (h_look[9:0] - H_TOT[9:0]) : h_look[9:0];
    assign f_v    = wraps ? ((v_cnt == V_TOT - 1) ? 10'd0 : (v_cnt + 10'd1)) : v_cnt;

    logic [6:0] f_col;
    logic [9:0] f_rel;
    logic [4:0] f_row;
    logic [3:0] f_line;
    logic       f_in_text;

    assign f_col     = f_h[9:3];
    assign f_rel     = f_v - TEXT_TOP[9:0];
    assign f_in_text = (f_v >= TEXT_TOP) && (f_v < TEXT_BOT) && (f_col < TEXT_COLS);
    assign f_row     = f_rel[8:4];
    assign f_line    = f_rel[3:0];

    // row * 80 == row*64 + row*16, so no multiplier is needed
    logic [10:0] row_base;
    assign row_base  = {f_row, 6'd0} + {2'b00, f_row, 4'd0};
    assign vram_addr = f_in_text ? (row_base + {4'd0, f_col}) : 11'd0;

    // ---- pipeline ----
    logic [2:0]  phase;
    assign phase = h_cnt[2:0];

    logic [7:0]  char_r, attr_r;
    logic [3:0]  line_r;
    logic        in_text_r;
    logic [10:0] cell_r;
    logic [7:0]  glyph_r;

    logic [7:0]  shreg;
    logic [7:0]  attr_cur;
    logic        in_text_cur;
    logic [10:0] cell_cur;

    logic [7:0]  font_pixels;

    font_rom #(.INIT_FILE(FONT_FILE), .ISMCE(ISMCE)) u_font (
        .clk       (clk_vga),
        .char_code (char_r),
        .scanline  (line_r),
        .pixels    (font_pixels)
    );

    always_ff @(posedge clk_vga or negedge rst_n) begin
        if (!rst_n) begin
            char_r      <= 8'h00;
            attr_r      <= 8'h07;
            line_r      <= 4'd0;
            in_text_r   <= 1'b0;
            cell_r      <= 11'd0;
            glyph_r     <= 8'h00;
            shreg       <= 8'h00;
            attr_cur    <= 8'h07;
            in_text_cur <= 1'b0;
            cell_cur    <= 11'd0;
        end else begin
            // VRAM data for the lookahead address is valid here.
            if (phase == 3'd1) begin
                char_r    <= vram_data[7:0];
                attr_r    <= vram_data[15:8];
                line_r    <= f_line;
                in_text_r <= f_in_text;
                cell_r    <= row_base + {4'd0, f_col};
            end

            // Font data for that character is valid here.
            if (phase == 3'd3) glyph_r <= font_pixels;

            // Hand the whole group over to the display side.
            if (phase == 3'd7) begin
                shreg       <= glyph_r;
                attr_cur    <= attr_r;
                in_text_cur <= in_text_r;
                cell_cur    <= cell_r;
            end else begin
                shreg <= {shreg[6:0], 1'b0};
            end
        end
    end

    // ---- cursor ----
    // Blinks at roughly 2 Hz off the frame rate, and occupies the bottom two
    // scanlines of its cell, which is the usual underline shape.
    logic [4:0] blink_cnt;
    logic       frame_tick;
    assign frame_tick = (h_cnt == H_TOT - 1) && (v_cnt == V_TOT - 1);

    always_ff @(posedge clk_vga or negedge rst_n) begin
        if (!rst_n)          blink_cnt <= 5'd0;
        else if (frame_tick) blink_cnt <= blink_cnt + 5'd1;
    end

    logic [3:0] disp_line;
    logic [9:0] disp_rel;
    assign disp_rel  = v_cnt - TEXT_TOP[9:0];
    assign disp_line = disp_rel[3:0];

    logic cursor_here;
    assign cursor_here = cursor_en && in_text_cur && blink_cnt[4] &&
                         (cell_cur == cursor_addr) && (disp_line >= 4'd14);

    // ---- colour ----
    logic pixel_on;
    assign pixel_on = (shreg[7] | cursor_here) && in_text_cur;

    logic [3:0] colour_idx;
    assign colour_idx = pixel_on ? attr_cur[3:0] : {1'b0, attr_cur[6:4]};

    // Standard CGA palette.
    function automatic logic [23:0] palette (input logic [3:0] idx);
        case (idx)
            4'h0: palette = 24'h000000;   4'h1: palette = 24'h0000AA;
            4'h2: palette = 24'h00AA00;   4'h3: palette = 24'h00AAAA;
            4'h4: palette = 24'hAA0000;   4'h5: palette = 24'hAA00AA;
            4'h6: palette = 24'hAA5500;   4'h7: palette = 24'hAAAAAA;
            4'h8: palette = 24'h555555;   4'h9: palette = 24'h5555FF;
            4'hA: palette = 24'h55FF55;   4'hB: palette = 24'h55FFFF;
            4'hC: palette = 24'hFF5555;   4'hD: palette = 24'hFF55FF;
            4'hE: palette = 24'hFFFF55;   default: palette = 24'hFFFFFF;
        endcase
    endfunction

    logic [23:0] rgb;
    assign rgb = palette(colour_idx);

    assign vga_r = active ? rgb[23:16] : 8'h00;
    assign vga_g = active ? rgb[15:8]  : 8'h00;
    assign vga_b = active ? rgb[7:0]   : 8'h00;

endmodule
