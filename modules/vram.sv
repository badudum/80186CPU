// ---------------------------------------------------------------------------
// vram — dual-port text-mode video RAM.
//
// Hierarchy: FPGA80186 -> memory_controller -> vram
//            (read port goes to vga_controller)
//
// The PC text buffer lives at physical B8000: 80x25 cells of two bytes each
// (character, attribute), so 4000 bytes. DOS and the BIOS write straight to
// this buffer, so its addressing has to match the PC convention exactly.
//
// The two ports have INDEPENDENT CLOCKS -- the CPU writes on clk_cpu, the
// video scan-out reads on clk_vga. That is exactly what a true dual-port block
// RAM is for, and it is why no clock-domain-crossing logic appears here: the
// M10K handles it. Do not add synchronisers around this.
//
// A CPU write racing a VGA read of the same cell produces a momentary glitch
// on one character of one frame, which is invisible and not worth arbitrating.
// ---------------------------------------------------------------------------

`timescale 1ns/1ns
module vram #(
    parameter int AW = 11,                    // words; 2048 covers 80x25
    // With ISMCE set the text buffer can be READ BACK over JTAG, so the screen
    // can be dumped with no monitor attached -- the only way to see what the
    // machine printed when debugging it remotely.
    parameter bit ISMCE   = 1'b0,
    parameter     MIF_LO  = "rom/vram_lo.mif",
    parameter     MIF_HI  = "rom/vram_hi.mif"
) (
    // CPU side
    input  logic            clk_cpu,
    input  logic [AW-1:0]   cpu_addr,
    input  logic [15:0]     cpu_wdata,
    input  logic            cpu_we,
    input  logic [1:0]      cpu_be,           // [0] low byte, [1] high byte
    output logic [15:0]     cpu_rdata,

    // VGA side (read only)
    input  logic            clk_vga,
    input  logic [AW-1:0]   vga_addr,
    output logic [15:0]     vga_rdata
);

    logic [7:0] ram_lo [0:(1<<AW)-1];
    logic [7:0] ram_hi [0:(1<<AW)-1];

    // Start as spaces with a light-grey-on-black attribute so the screen is
    // blank rather than full of garbage before the BIOS clears it.
    initial begin
        for (int i = 0; i < (1<<AW); i++) begin
            ram_lo[i] = 8'h20;   // space
            ram_hi[i] = 8'h07;   // attribute
        end
    end

    // Plain `always`, not `always_ff`: the initial block above also writes
    // these arrays to pre-fill the screen, and always_ff forbids a second
    // driver. Quartus honours initial blocks as memory initialisation.
    always @(posedge clk_cpu) begin
        if (cpu_we && cpu_be[0]) ram_lo[cpu_addr] <= cpu_wdata[7:0];
        if (cpu_we && cpu_be[1]) ram_hi[cpu_addr] <= cpu_wdata[15:8];
        cpu_rdata <= {ram_hi[cpu_addr], ram_lo[cpu_addr]};
    end

    always_ff @(posedge clk_vga)
        vga_rdata <= {ram_hi[vga_addr], ram_lo[vga_addr]};
    // ---- ISMCE shadow ----
    // The buffer itself CANNOT be made editor-visible: it is a true dual-port
    // RAM (CPU read/write on one port, video scan-out on the other), both
    // physical ports are in use, and the editor needs one of its own --
    // Quartus rejects it outright with "Cannot enable In-System Memory Content
    // Editor with BIDIR_DUAL_PORT mode RAM".
    //
    // So this is a write-only copy, fed by exactly the same CPU signals and
    // read by nothing in the design. It costs four M10K blocks and exists only
    // so the screen can be dumped over JTAG with no monitor attached.
    //
    // It shows what the CPU WROTE. If the real buffer ever disagreed with it
    // the fault would be in the buffer, and this would not reveal that -- but
    // for "what did the machine print", it is exactly the right answer.
    generate
        if (ISMCE) begin : g_shadow
            altera_syncram #(
                .operation_mode ("SINGLE_PORT"),
                .width_a (8), .widthad_a (AW), .numwords_a (1<<AW),
                .outdata_reg_a ("CLOCK0"),
                .init_file (MIF_LO),
                .read_during_write_mode_port_a ("DONT_CARE"),
                .enable_runtime_mod ("YES"), .instance_name ("VRML"),
                .lpm_type ("altera_syncram")
            ) u_shadow_lo (
                .clock0 (clk_cpu), .address_a (cpu_addr),
                .data_a (cpu_wdata[7:0]), .wren_a (cpu_we && cpu_be[0]), .q_a ()
            );

            altera_syncram #(
                .operation_mode ("SINGLE_PORT"),
                .width_a (8), .widthad_a (AW), .numwords_a (1<<AW),
                .outdata_reg_a ("CLOCK0"),
                .init_file (MIF_HI),
                .read_during_write_mode_port_a ("DONT_CARE"),
                .enable_runtime_mod ("YES"), .instance_name ("VRMH"),
                .lpm_type ("altera_syncram")
            ) u_shadow_hi (
                .clock0 (clk_cpu), .address_a (cpu_addr),
                .data_a (cpu_wdata[15:8]), .wren_a (cpu_we && cpu_be[1]), .q_a ()
            );
        end
    endgenerate

endmodule
