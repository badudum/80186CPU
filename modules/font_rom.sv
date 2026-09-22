// ---------------------------------------------------------------------------
// font_rom — 8x16 character glyph ROM for text mode.
//
// Hierarchy: FPGA80186 -> vga_controller -> font_rom
//
// 256 characters x 16 scanlines x 8 pixels = 4096 bytes, addressed by
// {char_code, scanline} and returning the 8-pixel bitmap for that row of that
// glyph. One M10K block holds it comfortably.
//
// The glyph image is rom/font.hex, extracted by tools/gen_font.py from the
// cp850-8x16 console font in the Linux kbd package (GPL-2.0). ASCII 32-126 is
// identical to the PC font; the upper half is CP850 rather than CP437, so the
// box-drawing glyphs differ. Check the licence before redistributing a
// bitstream built from it.
//
// $readmemh is called unconditionally. An earlier version skipped it when
// INIT_FILE was empty, but comparing a string parameter is a simulation-only
// construct -- Quartus rejects it outright ("INIT_FILE has an aggregate
// value"), which is the sort of thing only a real synthesis run catches.
//
// One cycle of read latency. vga_controller's pixel pipeline accounts for it.
// ---------------------------------------------------------------------------

`timescale 1ns/1ns
module font_rom #(
    parameter INIT_FILE = "rom/font.hex",
    // With ISMCE set the glyph data can be rewritten over JTAG in a running
    // board, without a rebuild. It needs an explicitly instantiated
    // altera_syncram and a .mif, so the contents exist in two forms -- keep
    // them in step with tools/hex2mif.py.
    parameter bit ISMCE     = 1'b0,
    parameter     MIF_FILE  = "rom/font.mif",
    parameter     ISMCE_ID  = "FONT"
) (
    input  logic        clk,
    input  logic [7:0]  char_code,
    input  logic [3:0]  scanline,
    output logic [7:0]  pixels
);

    logic [11:0] addr;
    assign addr = {char_code, scanline};

    generate
        if (ISMCE) begin : g_ismce
            // enable_runtime_mod is what makes this visible to the In-System
            // Memory Content Editor; instance_name is how it is identified
            // there. Everything else matches the inferred version below.
            // UNREGISTERED, not CLOCK0. The output register would add a
            // SECOND cycle of read latency -- address register plus output
            // register -- while the inferred memory below, which is what every
            // simulation uses, has one. Getting that wrong makes the ISMCE
            // build behave differently from the build the tests pass against,
            // and the difference is invisible until it is on a monitor: the
            // font ROM a cycle late shifted the whole screen right by one
            // character and printed the glyph fetched during blanking, cell 0,
            // at the start of every line.
            altera_syncram #(
                .operation_mode          ("ROM"),
                .width_a                 (8),
                .widthad_a               (12),
                .numwords_a              (4096),
                .outdata_reg_a           ("UNREGISTERED"),
                .init_file               (MIF_FILE),
                .enable_runtime_mod      ("YES"),
                .instance_name           (ISMCE_ID),
                .lpm_type                ("altera_syncram")
            ) u_rom (
                .clock0    (clk),
                .address_a (addr),
                .q_a       (pixels),
                .data_a    (8'h00),
                .wren_a    (1'b0)
            );
        end else begin : g_inferred
            logic [7:0] rom [0:4095];
            initial $readmemh(INIT_FILE, rom);
            always_ff @(posedge clk) pixels <= rom[addr];
        end
    endgenerate

endmodule
