// ---------------------------------------------------------------------------
// crtc — the cursor half of a 6845 CRT controller.
//
// Hierarchy: FPGA80186 -> io_decode -> crtc
// Testbench: sim/tb_crtc.sv
//
// The video timing in vga_controller is fixed, so none of the 6845's timing
// registers mean anything here. What software genuinely needs from this chip
// is the CURSOR, and it needs it at the addresses it expects: PC code sets the
// cursor by writing an index to 3D4 and a value to 3D5, and that is what the
// BIOS's INT 10h services below it are built on. Without it the cursor is
// pinned wherever the RTL hardwired it, and a DOS prompt has nowhere to blink.
//
// REGISTERS IMPLEMENTED
//   0A  cursor start   bit 5 = cursor OFF, bits 4:0 = first scanline
//   0B  cursor end     last scanline
//   0E  cursor address high
//   0F  cursor address low
//
// Registers 00-09 and 0C-0D are accepted and stored so that code which writes
// a full CRTC table does not fault, but nothing reads them back out into the
// video path. Writing them cannot change the display, which is the honest
// behaviour for a controller whose timing is not programmable.
//
// THE 11-BIT ADDRESS. A real 6845 holds a 14-bit address; an 80x25 screen only
// needs 11 bits, so the top of the high byte is dropped. Software that writes
// a larger value gets it truncated rather than wrapping the cursor somewhere
// unexpected.
//
// SCANLINE RANGE IS NOT HONOURED. The cursor shape in vga_controller is fixed
// at the bottom two scanlines, so registers 0A/0B only take effect through bit
// 5 (on/off). A block cursor set by software will still render as an
// underline.
// ---------------------------------------------------------------------------

`timescale 1ns/1ns
module crtc (
    input  logic        clk,
    input  logic        rst_n,

    // register interface (address decode done by io_decode).
    // `rd`/`wr` are SINGLE-CYCLE strobes -- see the contract in io_decode.sv.
    input  logic        sel,
    input  logic        port,          // 0 = index (3D4), 1 = data (3D5)
    input  logic        rd,
    input  logic        wr,
    input  logic [7:0]  wdata,
    output logic [7:0]  rdata,

    // -> vga_controller
    output logic        cursor_en,
    output logic [10:0] cursor_addr
);

    localparam logic [4:0] R_CURSOR_START = 5'h0A;
    localparam logic [4:0] R_CURSOR_END   = 5'h0B;
    localparam logic [4:0] R_CURSOR_HI    = 5'h0E;
    localparam logic [4:0] R_CURSOR_LO    = 5'h0F;

    logic [4:0]  index;
    logic [7:0]  regs [0:31];

    assign cursor_en   = ~regs[R_CURSOR_START][5];
    assign cursor_addr = {regs[R_CURSOR_HI][2:0], regs[R_CURSOR_LO]};

    always_comb begin
        if (port == 1'b0) rdata = {3'b000, index};
        else              rdata = regs[index];
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            index <= 5'h00;
            for (int i = 0; i < 32; i++) regs[i] <= 8'h00;
            // Power up with the cursor ON at cell 0. A blinking cursor in the
            // corner is the cheapest proof that the whole video path is alive
            // before any software has run -- if the font ROM failed to load,
            // every glyph is blank and the cursor is the only thing visible.
            regs[R_CURSOR_START] <= 8'h0E;   // visible, start line 14
            regs[R_CURSOR_END]   <= 8'h0F;
        end else if (sel && wr) begin
            if (port == 1'b0) index          <= wdata[4:0];
            else              regs[index]    <= wdata;
        end
    end

endmodule
