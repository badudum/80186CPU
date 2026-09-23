// ---------------------------------------------------------------------------
// io_decode — I/O-space port map: routes IN/OUT cycles to peripherals.
//
// Hierarchy: FPGA80186 -> io_decode -> {keyboard_controller, storage}
// Reference: learnings/02-bus-interface.md (I/O bus cycles)
//
// The 80186 has a 64 KB I/O space entirely separate from its 1 MB memory
// space. They overlap numerically, so a cycle can only be classified by the
// CPU's status lines -- never by address alone. This module owns the I/O side
// of that split.
//
// Division of labour: the 256-byte peripheral control block (the 80186's OWN
// integrated peripherals, at I/O FF00-FFFF by default) is decoded INSIDE
// cpu_top by pcb.sv, because on real silicon those accesses never left the
// chip. Everything external to the CPU is decoded here.
//
// ALWAYS TERMINATE THE CYCLE: `ready` is asserted even for ports nothing
// claims. An errant OUT to a nonexistent device must not wedge the CPU in wait
// states forever, which is a classic and very confusing bring-up hang.
// Unclaimed ports read FFFF, matching what a real PC bus floats to.
//
// SINGLE-SHOT STROBES. This is the subtle one. The BIU asserts RD/WR for the
// WHOLE of T2 and T3 -- longer still with wait states -- so a device strobe
// derived directly from them is high for several clocks. Any device with a
// side effect on access (a FIFO that pops, a counter that advances, a
// write-to-clear bit) would then perform it two or more times per bus cycle.
//
// Worse than the repeat is the timing: the BIU latches read data at the END of
// T3, so a FIFO that popped at the end of T2 would be presenting the NEXT
// entry by the time the CPU sampled. One `IN AL,60h` would return the byte
// after the one it asked for and discard two.
//
// So kbd_rd/kbd_wr are one-cycle pulses aligned with `ready`, which is exactly
// when the transfer completes. Devices behind this module may treat them as
// "the access happened, once" and drive their read data combinationally from
// `sel`/`port`, which stays stable for the whole cycle. pcb.sv solves the same
// problem for the CPU's internal peripherals with its `active` flag.
//
// PC-COMPATIBILITY NOTE: standard PC software expects an 8259 at 20h/21h, an
// 8253 at 40h-43h and an 8237 at 00h-0Fh. This design uses the 80186's own
// integrated equivalents instead, so those port numbers are deliberately
// unclaimed. If PC BIOS/DOS code is ever run unmodified, shims at those
// addresses would be added here -- see learnings/06-fpga-implementation-notes.md.
// ---------------------------------------------------------------------------

`timescale 1ns/1ns
module io_decode #(
    // Clocks per DRAM-refresh half period. A real PC toggles port 61h bit 4
    // every 15.085 us; at 25 MHz that is 377 clocks.
    parameter int REFRESH_DIV = 377
) (
    input  logic        clk,
    input  logic        rst_n,

    // CPU I/O bus (only driven during I/O-space cycles)
    input  logic [15:0] io_addr,
    input  logic        io_rd,
    input  logic        io_wr,
    input  logic [15:0] wdata,
    output logic [15:0] rdata,
    output logic        ready,

    // keyboard controller
    output logic        kbd_sel,
    output logic        kbd_port,     // 0 = data (60h), 1 = status (64h)
    output logic        kbd_rd,
    output logic        kbd_wr,
    output logic [7:0]  kbd_wdata,
    input  logic [7:0]  kbd_rdata,

    // 6845 CRTC (cursor control) at 3D4/3D5
    output logic        crtc_sel,
    output logic        crtc_port,    // 0 = index (3D4), 1 = data (3D5)
    output logic        crtc_rd,
    output logic        crtc_wr,
    output logic [7:0]  crtc_wdata,
    input  logic [7:0]  crtc_rdata,

    // VGA palette DAC at 3C8/3C9
    output logic        dac_wr,
    output logic        dac_port,     // 0 = 3C8 index, 1 = 3C9 data
    output logic [7:0]  dac_wdata,
    input  logic [7:0]  dac_rdata,

    // Video mode, CGA-style: bit 1 selects graphics
    output logic        mode_gfx,

    // 8259 shim: a one-cycle pulse when software ends an interrupt the PC way
    output logic        pic_eoi,

    // 8253 programmable interval timer at 40h-43h
    output logic        pit_sel,
    output logic [1:0]  pit_port,
    output logic        pit_rd,
    output logic        pit_wr,
    output logic [7:0]  pit_wdata,
    input  logic [7:0]  pit_rdata,

    // blitter
    output logic        blit_sel,
    output logic [2:0]  blit_reg,     // (port - 0330h) >> 1
    output logic        blit_rd,
    output logic        blit_wr,
    output logic [15:0] blit_wdata,
    input  logic [15:0] blit_rdata,

    // block storage
    output logic        stor_sel,
    output logic [2:0]  stor_reg,     // (port - 0320h) >> 1
    output logic        stor_rd,
    output logic        stor_wr,
    output logic [15:0] stor_wdata,
    input  logic [15:0] stor_rdata
);

    localparam logic [15:0] PORT_KBD_DATA = 16'h0060;
    localparam logic [15:0] PORT_KBD_STAT = 16'h0064;
    localparam logic [15:0] PORT_PPI_B    = 16'h0061;
    localparam logic [15:0] PORT_DAC_IDX  = 16'h03C8;
    localparam logic [15:0] PORT_DAC_DATA = 16'h03C9;
    localparam logic [15:0] PORT_MODE     = 16'h03D8;
    localparam logic [15:0] PORT_PIC_CMD  = 16'h0020;
    localparam logic [15:0] PORT_PIC_MASK = 16'h0021;
    localparam logic [13:0] PORT_PIT_PAGE  = 14'h0010;   // 0040-0043

    // Block storage occupies 0320-032F, the PC/XT hard-disk controller range.
    localparam logic [11:0] PORT_STOR_PAGE = 12'h032;
    // The blitter sits at 0330-033F, next to the block device.
    localparam logic [11:0] PORT_BLIT_PAGE = 12'h033;

    // BYTE LANES. A byte-wide device on a 16-bit bus has to answer on the lane
    // the CPU is listening to. The BIU puts an odd-address byte on D15-D8 and
    // an even-address byte on D7-D0 (biu.sv, cur_dout), so the port address
    // selects the lane in BOTH directions. Every byte port here was even until
    // the CRTC arrived at 3D5, which made this visible.
    logic [7:0] byte_wdata;
    assign byte_wdata = io_addr[0] ? wdata[15:8] : wdata[7:0];

    // ---- port 61h, the PC's "system control port" ----
    //
    // Only two things here matter. Bit 4 is the DRAM REFRESH TOGGLE: on a real
    // PC it flips every 15.085 us, and PC software uses it as a free running
    // fine-grained clock to calibrate delay loops. Bits 1:0 gate the speaker,
    // which nothing on this board drives, but they read back because code that
    // writes them usually reads first and puts them back.
    //
    // Without the toggle bit the machine hangs, silently and permanently, in
    // code that has no timeout because on real hardware the bit cannot fail to
    // change. MS-DOS's startup calibration is exactly such a loop: `in al,61h;
    // and al,10h; cmp al,ah; jz $-4`. It spins forever on a port that reads a
    // constant, which is what an undecoded port does.
    logic [$clog2(REFRESH_DIV)-1:0] refresh_cnt;
    logic                           refresh_tog;
    logic [1:0]                     spk_bits;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            refresh_cnt <= '0;
            refresh_tog <= 1'b0;
        end else if (refresh_cnt == REFRESH_DIV[$clog2(REFRESH_DIV)-1:0] - 1) begin
            refresh_cnt <= '0;
            refresh_tog <= ~refresh_tog;
        end else begin
            refresh_cnt <= refresh_cnt + 1'b1;
        end
    end

    logic hit_ppi;
    assign hit_ppi = (io_addr == PORT_PPI_B);

    logic [7:0] ppi_rdata;
    assign ppi_rdata = {3'b000, refresh_tog, 2'b00, spk_bits};

    // ---- video mode select, port 3D8 ----
    // The CGA mode control register, of which only bit 1 -- graphics rather
    // than text -- is implemented. This design is not VGA register
    // compatible: real mode 13h is set by programming a dozen sequencer,
    // CRTC and graphics-controller registers, and nothing here models them.
    // The BIOS's INT 10h AH=00 writes this instead, so software that sets a
    // mode the way everything actually does -- through the BIOS -- works, and
    // software that pokes VGA registers directly does not.
    logic hit_mode;
    assign hit_mode = (io_addr == PORT_MODE);

    logic hit_dac;
    assign hit_dac  = (io_addr == PORT_DAC_IDX) || (io_addr == PORT_DAC_DATA);
    assign dac_port = (io_addr == PORT_DAC_DATA);

    // ---- 8259 shim at 20h/21h ----
    // Not an 8259, and not pretending to be one. PC software ends an
    // interrupt by writing OCW2 with the EOI bit to port 20h; this design's
    // interrupts come from the 80186's own controller, which takes its EOI at
    // FF22 instead. A guest that does it the PC way therefore leaves the
    // in-service bit set forever and never receives another interrupt --
    // including the timer, so its clock stops and it waits for time that
    // never passes. Doom8088 hangs on a black screen for exactly that reason.
    //
    // So a write to 20h with the EOI bit set is turned into a pulse that
    // clears the 80186 controller's highest-priority in-service bit, which is
    // what its own EOI register does. The interrupt mask at 21h is accepted
    // and read back but not acted on: ignoring a mask can only deliver
    // interrupts a guest expected to be able to receive, which is the safe
    // direction to be wrong in.
    logic hit_pit;
    assign hit_pit  = (io_addr[15:2] == PORT_PIT_PAGE);
    assign pit_port = io_addr[1:0];

    logic hit_pic_cmd, hit_pic_mask;
    assign hit_pic_cmd  = (io_addr == PORT_PIC_CMD);
    assign hit_pic_mask = (io_addr == PORT_PIC_MASK);

    logic [7:0] pic_mask_r;

    logic hit_kbd, hit_stor, hit_crtc, hit_blit;
    assign hit_kbd  = (io_addr == PORT_KBD_DATA) || (io_addr == PORT_KBD_STAT);
    assign kbd_port = (io_addr == PORT_KBD_STAT);

    assign hit_stor = (io_addr[15:4] == PORT_STOR_PAGE);
    assign stor_reg = io_addr[3:1];

    assign hit_blit = (io_addr[15:4] == PORT_BLIT_PAGE);
    assign blit_reg = io_addr[3:1];

    // 6845 CRTC, the PC's cursor controller: 3D4 index, 3D5 data.
    assign hit_crtc  = (io_addr == 16'h03D4) || (io_addr == 16'h03D5);
    assign crtc_port = io_addr[0];

    // One-cycle access strobe, aligned with the completion of the transfer.
    // `ready` is registered from io_rd/io_wr, so its RISING edge is the single
    // clock during which the CPU latches read data -- see the header.
    logic ready_d, access_strobe;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) ready_d <= 1'b0;
        else        ready_d <= ready;
    end
    assign access_strobe = ready && !ready_d;

    assign kbd_sel   = hit_kbd;
    assign kbd_rd    = hit_kbd && io_rd && access_strobe;
    assign kbd_wr    = hit_kbd && io_wr && access_strobe;
    assign kbd_wdata = byte_wdata;

    assign crtc_sel   = hit_crtc;
    assign crtc_rd    = hit_crtc && io_rd && access_strobe;
    assign crtc_wr    = hit_crtc && io_wr && access_strobe;
    assign crtc_wdata = byte_wdata;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)                              spk_bits <= 2'b00;
        else if (hit_ppi && io_wr && access_strobe) spk_bits <= byte_wdata[1:0];
    end

    assign dac_wr     = hit_dac && io_wr && access_strobe;
    assign dac_wdata  = byte_wdata;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)                                  mode_gfx <= 1'b0;
        else if (hit_mode && io_wr && access_strobe) mode_gfx <= byte_wdata[1];
    end

    // OCW2 bit 5 is the EOI request; bit 4 clear distinguishes OCW2 from ICW1.
    assign pic_eoi = hit_pic_cmd && io_wr && access_strobe
                     && byte_wdata[5] && !byte_wdata[4];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)                                      pic_mask_r <= 8'h00;
        else if (hit_pic_mask && io_wr && access_strobe) pic_mask_r <= byte_wdata;
    end

    assign pit_sel   = hit_pit;
    assign pit_rd    = hit_pit && io_rd && access_strobe;
    assign pit_wr    = hit_pit && io_wr && access_strobe;
    assign pit_wdata = byte_wdata;

    assign blit_sel   = hit_blit;
    assign blit_rd    = hit_blit && io_rd && access_strobe;
    assign blit_wr    = hit_blit && io_wr && access_strobe;
    assign blit_wdata = wdata;

    assign stor_sel   = hit_stor;
    assign stor_rd    = hit_stor && io_rd && access_strobe;
    assign stor_wr    = hit_stor && io_wr && access_strobe;
    assign stor_wdata = wdata;

    // Byte devices place their answer on the lane matching the port address;
    // storage is genuinely 16-bit and its registers are all even.
    always_comb begin
        if (hit_kbd)
            rdata = io_addr[0] ? {kbd_rdata, 8'h00} : {8'h00, kbd_rdata};
        else if (hit_crtc)
            rdata = io_addr[0] ? {crtc_rdata, 8'h00} : {8'h00, crtc_rdata};
        else if (hit_dac)
            rdata = io_addr[0] ? {dac_rdata, 8'h00} : {8'h00, dac_rdata};
        else if (hit_mode)
            rdata = io_addr[0] ? {6'b0, mode_gfx, 9'b0}
                               : {14'b0, mode_gfx, 1'b0};
        else if (hit_ppi)
            rdata = io_addr[0] ? {ppi_rdata, 8'h00} : {8'h00, ppi_rdata};
        else if (hit_stor)
            rdata = stor_rdata;
        else if (hit_blit)
            rdata = blit_rdata;
        else if (hit_pit)
            rdata = io_addr[0] ? {pit_rdata, 8'h00} : {8'h00, pit_rdata};
        else if (hit_pic_mask)
            rdata = io_addr[0] ? {pic_mask_r, 8'h00} : {8'h00, pic_mask_r};
        else if (hit_pic_cmd)
            rdata = 16'h0000;
        else
            rdata = 16'hFFFF;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) ready <= 1'b0;
        else        ready <= io_rd || io_wr;
    end

endmodule
