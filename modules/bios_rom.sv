// ---------------------------------------------------------------------------
// bios_rom — boot ROM mapped at the top of memory.
//
// Hierarchy: FPGA80186 -> memory_controller -> bios_rom
// Reference: learnings/04-integrated-peripherals.md §2 (reset behaviour)
//
// The very first bus cycle the CPU ever runs is an instruction fetch from
// physical FFFF0, which lands here. If this is not returning sane data the
// instant reset releases, nothing else in the system can be debugged.
//
// SIZE: 16 KB rather than the full 64 KB the region can hold. The ROM is
// ALIASED, repeating through the whole F0000-FFFFF window, so FFFF0 always
// lands in the last 16 bytes of the image whatever its size -- with ROM_AW=14
// that is image offset 3FF0, which is where gen_bios.py puts the reset vector.
//
// It was 4 KB while the boot code was a banner and a halt. A BIOS with INT 10h,
// 13h, 16h, the interrupt handlers and a scancode table does not fit in that.
// 16 KB costs 131,072 bits of M10K, about 3% of the device.
//
// The image comes from rom/bios.lo.hex and rom/bios.hi.hex, produced by
// tools/gen_bios.py. Two files because the even and odd bytes live in separate
// banks, which is what lets a byte access take just the lane it needs.
//
// The paths are separate parameters rather than one base name with string
// concatenation: building a filename from a string parameter is a
// simulation-only construct that Quartus will not synthesise.
// ---------------------------------------------------------------------------

`timescale 1ns/1ns
module bios_rom #(
    parameter int ROM_AW  = 14,               // bytes = 2**ROM_AW
    parameter     INIT_LO = "rom/bios.lo.hex",
    parameter     INIT_HI = "rom/bios.hi.hex",
    // With ISMCE set, the BIOS can be rewritten over JTAG in a running board.
    // This is the memory where that matters most: changing BIOS code otherwise
    // means a full synthesis, fit and timing run for every edit.
    parameter bit ISMCE   = 1'b0,
    parameter     MIF_LO  = "rom/bios.lo.mif",
    parameter     MIF_HI  = "rom/bios.hi.mif"
) (
    input  logic        clk,
    input  logic [19:0] addr,
    output logic [15:0] rdata
);

    localparam int WORDS = (1 << ROM_AW) / 2;

    // Two byte-wide banks, matching the 80186's even/odd bus organisation, so
    // a byte read takes the lane it needs without a read-modify-write.
    logic [ROM_AW-2:0] widx;
    assign widx = addr[ROM_AW-1:1];

    generate
        if (ISMCE) begin : g_ismce
            // Two instances, one per byte bank, so each appears separately in
            // the editor and can be reloaded on its own.
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
                .operation_mode     ("ROM"),
                .width_a            (8),
                .widthad_a          (ROM_AW-1),
                .numwords_a         (WORDS),
                .outdata_reg_a      ("UNREGISTERED"),
                .init_file          (MIF_LO),
                .enable_runtime_mod ("YES"),
                .instance_name      ("BIOL"),
                .lpm_type           ("altera_syncram")
            ) u_lo (
                .clock0 (clk), .address_a (widx), .q_a (rdata[7:0]),
                .data_a (8'h00), .wren_a (1'b0)
            );

            altera_syncram #(
                .operation_mode     ("ROM"),
                .width_a            (8),
                .widthad_a          (ROM_AW-1),
                .numwords_a         (WORDS),
                .outdata_reg_a      ("UNREGISTERED"),
                .init_file          (MIF_HI),
                .enable_runtime_mod ("YES"),
                .instance_name      ("BIOH"),
                .lpm_type           ("altera_syncram")
            ) u_hi (
                .clock0 (clk), .address_a (widx), .q_a (rdata[15:8]),
                .data_a (8'h00), .wren_a (1'b0)
            );
        end else begin : g_inferred
            logic [7:0] rom_lo [0:WORDS-1];
            logic [7:0] rom_hi [0:WORDS-1];

            initial begin
                $readmemh(INIT_LO, rom_lo);
                $readmemh(INIT_HI, rom_hi);
            end

            always_ff @(posedge clk)
                rdata <= {rom_hi[widx], rom_lo[widx]};
        end
    endgenerate

endmodule
