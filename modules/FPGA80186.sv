// ---------------------------------------------------------------------------
// FPGA80186 — board top level (DE1-SoC, Cyclone V 5CSEMA5F31C6).
// Module name must stay FPGA80186 (TOP_LEVEL_ENTITY in FPGA80186.qsf).
//
// CANONICAL MODULE HIERARCHY (other modules' headers refer back to this):
//
//   FPGA80186            board top: pins, clock/reset, system interconnect
//   +- clk_rst           CLOCK_50 straight through + /2 pixel clock, reset sync
//   +- cpu_top           "the 80186 chip" (everything inside the real package)
//   |  +- biu            bus interface unit: T1-T4 cycles, prefetch
//   |  |  +- prefetch_queue    6-byte instruction queue
//   |  +- eu             execution unit
//   |  |  +- decode      opcode + ModR/M -> control word
//   |  |  +- microcode   ModR/M addressing-mode table
//   |  |  +- execUnit    the instruction sequencer
//   |  |  +- ALU         arithmetic/logic/shift + multicycle MUL/DIV
//   |  |  +- regfile     GP, pointer, segment, IP, FLAGS
//   |  +- pcb            peripheral control block window (I/O FF00-FFFF)
//   |     +- interrupt_controller
//   |     +- timer
//   |     +- dma         (still a stub)
//   |     +- chip_select
//   +- memory_controller system memory map + READY
//   |  +- bios_rom       boot ROM, aliased through F0000-FFFFF
//   |  +- vram           dual-port text buffer at B8000
//   +- io_decode         I/O port map -> keyboard, storage
//   +- keyboard_controller   PS/2 receiver
//   +- vga_controller    640x480 text mode, or 320x200x8 graphics
//   +- vga_dac           the 256-colour palette behind mode 13h
//   +- blitter           rectangle fill and copy in the framebuffer
//      +- font_rom       8x16 glyphs
//
// BUS ROUTING: the CPU's memory and I/O address spaces are separate and
// overlap numerically, so cycles are steered by the io_cycle status bit, never
// by address alone. Accesses to the peripheral control block never appear here
// at all -- cpu_top answers those internally, exactly as the real chip does.
//
// WHAT IS NOT WIRED: dma is still a stub and is not instantiated. Conventional
// RAM is the board's SDRAM, so the whole 640 KB region is backed.
//
// PIN ASSIGNMENTS: FPGA80186.qsf carries no set_location_assignment entries
// yet, so every pin below is unassigned. They must come from the DE1-SoC User
// Manual before a bitstream will do anything on real hardware.
// ---------------------------------------------------------------------------

`timescale 1ns/1ns
module FPGA80186 #(
    // Where the disk image lives.
    //   0  on-chip ROM from rom/disk.hex. Survives power-up, capped at about
    //      700 sectors by block RAM, and read-only.
    //   1  a window of SDRAM above 1 MB. Any size and writable, but volatile:
    //      nothing boots until a loader has put an image there. Flipping this
    //      before that loader exists gives a machine with a blank disk.
    parameter bit DISK_IN_SDRAM = 1'b0,
    parameter int DISK_SECTORS  = 256,
    // The JTAG image loader instantiates an Altera virtual JTAG node, which
    // cannot be simulated without vendor libraries. Testbenches turn it off;
    // a real build leaves it on.
    parameter bit ENABLE_JTAG_LOADER = 1'b1,
    // Makes the BIOS and font ROMs visible to the In-System Memory Content
    // Editor, so their contents can be replaced over JTAG without a rebuild.
    // Off for simulation, which reads the .hex images directly.
    parameter bit ISMCE = 1'b0
) (
    // Board clock and reset
    input  logic        CLOCK_50,
    input  logic [0:0]  KEY,
    // The board calls these LEDR, and the pin assignments are generated from
    // the manual by name, so the port name has to match.
    output logic [9:0]  LEDR,
    // VGA, through the board's ADV7123 DAC
    output logic [7:0]  VGA_R,
    output logic [7:0]  VGA_G,
    output logic [7:0]  VGA_B,
    output logic        VGA_HS,
    output logic        VGA_VS,
    output logic        VGA_CLK,
    output logic        VGA_BLANK_N,
    output logic        VGA_SYNC_N,
    // PS/2 keyboard
    input  logic        PS2_CLK,
    input  logic        PS2_DAT,
    // SDRAM (64 MB on the FPGA side of the board)
    output logic [12:0] DRAM_ADDR,
    output logic [1:0]  DRAM_BA,
    inout  wire  [15:0] DRAM_DQ,
    output logic        DRAM_CKE,
    output logic        DRAM_CS_N,
    output logic        DRAM_RAS_N,
    output logic        DRAM_CAS_N,
    output logic        DRAM_WE_N,
    output logic        DRAM_LDQM,
    output logic        DRAM_UDQM,
    output logic        DRAM_CLK
);

    logic [1:0] dram_dqm;
    assign {DRAM_UDQM, DRAM_LDQM} = dram_dqm;

    // ---- clocks and reset ----
    logic clk_cpu, clk_vga, rst_n, rst_vga_n;

    clk_rst u_clk_rst (
        .clk_board  (CLOCK_50),
        .rst_btn_n  (KEY[0]),
        .clk_cpu    (clk_cpu),
        .clk_vga    (clk_vga),
        .rst_n      (rst_n),
        .rst_vga_n  (rst_vga_n)
    );

    // ---- CPU bus ----
    logic [19:0] addr;
    logic [15:0] cpu_dout, cpu_din;
    logic        rd, wr, io_cycle, bhe, a0, ale;
    logic [2:0]  s;
    logic        ready;

    logic        halted, dbg_int_taken;
    logic [15:0] dbg_ip, dbg_cs, dbg_flags;

    // ---- graphics: framebuffer scan-out and the palette DAC ----
    logic        dac_wr, dac_port, mode_gfx;
    logic [7:0]  dac_wdata, dac_rdata;
    logic [14:0] fb_read_addr;
    logic [15:0] fb_read_data;
    logic [7:0]  pal_index;
    logic [17:0] pal_rgb;

    // ---- blitter ----
    logic        pic_eoi;
    logic        blit_sel, blit_rd, blit_wr, blit_we, blit_stall, blit_busy;
    logic [2:0]  blit_reg;
    logic [15:0] blit_wdata, blit_rdata, blit_fb_wdata, blit_fb_rdata;
    logic [14:0] blit_fb_addr;
    logic [1:0]  blit_fb_be;
    logic [7:0]  dbg_int_type;

    logic        kbd_irq;

    // ---- the JTAG image loader's signals ----
    logic [23:0] ld_maddr;
    logic [15:0] ld_mwdata;
    logic [1:0]  ld_mbe;
    logic        ld_mrd, ld_mwr, cpu_hold;

    // The CPU alone is held while an image is being written. Memory, the
    // arbiter and the SDRAM controller must keep running -- they are what the
    // loader writes through.
    logic cpu_rst_n;
    assign cpu_rst_n = rst_n && !cpu_hold;

    cpu_top u_cpu (
        .clk           (clk_cpu),
        .rst_n         (cpu_rst_n),
        .addr          (addr),
        .dout          (cpu_dout),
        .din           (cpu_din),
        .rd            (rd),
        .wr            (wr),
        .io_cycle      (io_cycle),
        .bhe           (bhe),
        .a0            (a0),
        .ale           (ale),
        .s             (s),
        .ready         (ready),
        .nmi           (1'b0),
        // The keyboard is the only external interrupt source so far. In master
        // mode INT0 is a fixed type 12, so a BIOS must put its keyboard
        // handler there rather than at the PC's traditional IRQ1 vector.
        .int0          (kbd_irq),
        .int1          (1'b0),
        .int2          (1'b0),
        .int3          (1'b0),
        // Nothing on this board drives a DMA request pin; transfers are
        // started by software in unsynchronised mode, or by timer 2.
        .drq0          (1'b0),
        .drq1          (1'b0),
        .intr_req      (1'b0),
        .intr_type     (8'h00),
        .intr_ack      (),
        .halted        (halted),
        .ext_eoi       (pic_eoi),
        .dbg_ip        (dbg_ip),
        .dbg_cs        (dbg_cs),
        .dbg_flags     (dbg_flags),
        .dbg_int_type  (dbg_int_type),
        .dbg_int_taken (dbg_int_taken)
    );

    // ---- memory side ----
    logic [15:0] mem_rdata;
    logic        mem_ready;
    logic [10:0] vram_addr;
    logic [15:0] vram_data;

    // Conventional RAM now comes from the board's SDRAM, so the full 640 KB
    // region is backed rather than a 32 KB on-chip window.
    // SDRAM port 0 belongs to the block device; port 1 is reserved for the
    // JTAG loader and stays idle until that exists. When the disk is ROM-backed
    // the device never asserts its request, so this costs nothing.
    logic [1:0]  ext_rd, ext_wr, ext_ready;
    logic [23:0] ext_addr  [2];
    logic [15:0] ext_wdata [2];
    logic [1:0]  ext_be    [2];
    logic [15:0] ext_rdata;

    logic [23:0] disk_maddr;
    logic [15:0] disk_mwdata;
    logic [1:0]  disk_mbe;
    logic        disk_mrd, disk_mwr;

    assign ext_rd = {ld_mrd, disk_mrd};
    assign ext_wr = {ld_mwr, disk_mwr};
    always_comb begin
        ext_addr[0]  = disk_maddr;
        ext_wdata[0] = disk_mwdata;
        ext_be[0]    = disk_mbe;
        ext_addr[1]  = ld_maddr;
        ext_wdata[1] = ld_mwdata;
        ext_be[1]    = ld_mbe;
    end

    generate
        if (ENABLE_JTAG_LOADER) begin : g_loader
            jtag_loader #(.INSTANCE_ID(0), .SIM_HOOKS(1'b0)) u_loader (
                .clk       (clk_cpu),
                .rst_n     (rst_n),
                .mem_addr  (ld_maddr),
                .mem_wdata (ld_mwdata),
                .mem_be    (ld_mbe),
                .mem_rd    (ld_mrd),
                .mem_wr    (ld_mwr),
                .mem_rdata (ext_rdata),
                .mem_ready (ext_ready[1]),
                .cpu_hold  (cpu_hold),
                .cpu_pc    ({dbg_cs, dbg_ip}),
                .sim_tck   (1'b0),
                .sim_tdi   (1'b0),
                .sim_ir    (4'd0),
                .sim_cdr   (1'b0),
                .sim_sdr   (1'b0),
                .sim_udr   (1'b0),
                .sim_tdo   ()
            );
        end else begin : g_no_loader
            assign ld_maddr  = 24'h000000;
            assign ld_mwdata = 16'h0000;
            assign ld_mbe    = 2'b11;
            assign ld_mrd    = 1'b0;
            assign ld_mwr    = 1'b0;
            assign cpu_hold  = 1'b0;
        end
    endgenerate


    memory_controller #(.USE_SDRAM(1'b1), .ISMCE(ISMCE)) u_mem (
        .clk            (clk_cpu),
        .rst_n          (rst_n),
        .addr           (addr),
        .wdata          (cpu_dout),
        .rdata          (mem_rdata),
        .rd             (rd && !io_cycle),
        .wr             (wr && !io_cycle),
        .bhe            (bhe),
        .a0             (a0),
        .ready          (mem_ready),
        .clk_vga        (clk_vga),
        .vram_read_addr (vram_addr),
        .vram_read_data (vram_data),
        .fb_read_addr   (fb_read_addr),
        .fb_read_data   (fb_read_data),
        .blit_addr      (blit_fb_addr),
        .blit_wdata     (blit_fb_wdata),
        .blit_be        (blit_fb_be),
        .blit_we        (blit_we),
        .blit_rdata     (blit_fb_rdata),
        .blit_stall     (blit_stall),
        .ext_rd         (ext_rd),
        .ext_wr         (ext_wr),
        .ext_addr       (ext_addr),
        .ext_wdata      (ext_wdata),
        .ext_be         (ext_be),
        .ext_ready      (ext_ready),
        .ext_rdata      (ext_rdata),
        .dram_addr      (DRAM_ADDR),
        .dram_ba        (DRAM_BA),
        .dram_dq        (DRAM_DQ),
        .dram_cke       (DRAM_CKE),
        .dram_cs_n      (DRAM_CS_N),
        .dram_ras_n     (DRAM_RAS_N),
        .dram_cas_n     (DRAM_CAS_N),
        .dram_we_n      (DRAM_WE_N),
        .dram_dqm       (dram_dqm),
        .dram_clk       (DRAM_CLK)
    );

    // ---- I/O side ----
    logic [15:0] io_rdata;
    logic        io_ready;
    logic        kbd_sel, kbd_port, kbd_rd, kbd_wr;
    logic [7:0]  kbd_wdata, kbd_rdata;
    logic        crtc_sel, crtc_port, crtc_rd, crtc_wr;
    logic [7:0]  crtc_wdata, crtc_rdata;
    logic        cursor_en;
    logic [10:0] cursor_addr;
    logic        stor_sel, stor_rd, stor_wr;
    logic [2:0]  stor_reg;
    logic [15:0] stor_wdata, stor_rdata;

    blitter u_blit (
        .clk       (clk_cpu),
        .rst_n     (rst_n),
        .reg_sel   (blit_sel),
        .reg_num   (blit_reg),
        .reg_rd    (blit_rd),
        .reg_wr    (blit_wr),
        .reg_wdata (blit_wdata),
        .reg_rdata (blit_rdata),
        .stall     (blit_stall),
        .fb_addr   (blit_fb_addr),
        .fb_wdata  (blit_fb_wdata),
        .fb_be     (blit_fb_be),
        .fb_we     (blit_we),
        .fb_rdata  (blit_fb_rdata),
        .busy      (blit_busy)
    );

    vga_dac u_dac (
        .clk_cpu   (clk_cpu),
        .rst_n     (rst_n),
        .dac_wr    (dac_wr),
        .dac_port  (dac_port),
        .dac_wdata (dac_wdata),
        .dac_rdata (dac_rdata),
        .clk_vga   (clk_vga),
        .pal_index (pal_index),
        .pal_rgb   (pal_rgb)
    );

    io_decode u_io (
        .clk       (clk_cpu),
        .rst_n     (rst_n),
        .io_addr   (addr[15:0]),
        .io_rd     (rd && io_cycle),
        .io_wr     (wr && io_cycle),
        .wdata     (cpu_dout),
        .rdata     (io_rdata),
        .ready     (io_ready),
        .kbd_sel   (kbd_sel),
        .kbd_port  (kbd_port),
        .kbd_rd    (kbd_rd),
        .kbd_wr    (kbd_wr),
        .kbd_wdata (kbd_wdata),
        .kbd_rdata (kbd_rdata),
        .crtc_sel   (crtc_sel),
        .crtc_port  (crtc_port),
        .crtc_rd    (crtc_rd),
        .crtc_wr    (crtc_wr),
        .crtc_wdata (crtc_wdata),
        .crtc_rdata (crtc_rdata),
        .dac_wr     (dac_wr),
        .dac_port   (dac_port),
        .dac_wdata  (dac_wdata),
        .dac_rdata  (dac_rdata),
        .mode_gfx   (mode_gfx),
        .pic_eoi    (pic_eoi),
        .blit_sel   (blit_sel),
        .blit_reg   (blit_reg),
        .blit_rd    (blit_rd),
        .blit_wr    (blit_wr),
        .blit_wdata (blit_wdata),
        .blit_rdata (blit_rdata),
        .stor_sel   (stor_sel),
        .stor_reg   (stor_reg),
        .stor_rd    (stor_rd),
        .stor_wr    (stor_wr),
        .stor_wdata (stor_wdata),
        .stor_rdata (stor_rdata)
    );

    // Cursor controller. Software moves the cursor through this; see crtc.sv.
    crtc u_crtc (
        .clk         (clk_cpu),
        .rst_n       (rst_n),
        .sel         (crtc_sel),
        .port        (crtc_port),
        .rd          (crtc_rd),
        .wr          (crtc_wr),
        .wdata       (crtc_wdata),
        .rdata       (crtc_rdata),
        .cursor_en   (cursor_en),
        .cursor_addr (cursor_addr)
    );

    // Block storage. See storage.sv for why this is ROM-backed rather than
    // an SD card: the DE1-SoC's microSD socket is wired to the HPS.
    storage #(
        .SECTORS   (DISK_SECTORS),
        .USE_SDRAM (DISK_IN_SDRAM)
    ) u_storage (
        .clk       (clk_cpu),
        .rst_n     (rst_n),
        .sel       (stor_sel),
        .reg_sel   (stor_reg),
        .rd        (stor_rd),
        .wr        (stor_wr),
        .wdata     (stor_wdata),
        .rdata     (stor_rdata),
        .mem_addr  (disk_maddr),
        .mem_wdata (disk_mwdata),
        .mem_be    (disk_mbe),
        .mem_rd    (disk_mrd),
        .mem_wr    (disk_mwr),
        .mem_rdata (ext_rdata),
        .mem_ready (ext_ready[0])
    );

    // Steer responses by the cycle type, not by address: the memory and I/O
    // spaces overlap numerically.
    assign cpu_din = io_cycle ? io_rdata : mem_rdata;
    assign ready   = io_cycle ? io_ready : mem_ready;

    keyboard_controller u_kbd (
        .clk        (clk_cpu),
        .rst_n      (rst_n),
        .ps2_clk    (PS2_CLK),
        .ps2_dat    (PS2_DAT),
        .sel        (kbd_sel),
        .port       (kbd_port),
        .rd         (kbd_rd),
        .wr         (kbd_wr),
        .wdata      (kbd_wdata),
        .rdata      (kbd_rdata),
        .data_avail (),
        .irq        (kbd_irq)
    );

    // ---- video ----
    // The cursor comes from the CRTC, which powers up enabled at cell 0: a
    // correctly working video path then shows a blinking cursor even before a
    // font image exists, and with no font loaded every glyph is blank, so the
    // cursor is the only visible proof that scan-out and the text buffer are
    // alive. Software moves it afterwards through ports 3D4/3D5.
    //
    // No clock-domain crossing here despite the two names: clk_cpu and clk_vga
    // are the same 25 MHz net (see clk_rst.sv), so the CRTC's registers are
    // read by the video side synchronously.
    vga_controller #(.ISMCE(ISMCE)) u_vga (
        .clk_vga     (clk_vga),
        .rst_n       (rst_vga_n),
        .vram_addr   (vram_addr),
        .vram_data   (vram_data),
        .mode_gfx    (mode_gfx),
        .fb_addr     (fb_read_addr),
        .fb_data     (fb_read_data),
        .pal_index   (pal_index),
        .pal_rgb     (pal_rgb),
        .cursor_en   (cursor_en),
        .cursor_addr (cursor_addr),
        .vga_r       (VGA_R),
        .vga_g       (VGA_G),
        .vga_b       (VGA_B),
        .vga_hs      (VGA_HS),
        .vga_vs      (VGA_VS),
        .vga_blank_n (VGA_BLANK_N),
        .vga_sync_n  (VGA_SYNC_N),
        .vga_clk     (VGA_CLK)
    );

    // ---- bring-up visibility ----
    // Before video works these LEDs are the only window into the CPU, so they
    // show whether it halted and roughly where it is executing.
    assign LEDR = {2'b00, halted, dbg_int_taken, dbg_ip[5:0]};

endmodule
