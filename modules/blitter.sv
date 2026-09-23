// ---------------------------------------------------------------------------
// blitter — fills and copies rectangles in the graphics framebuffer.
//
// Hierarchy: FPGA80186 -> blitter
//            (register interface from io_decode, pixels through
//             memory_controller's framebuffer port)
// Testbench: sim/tb_blitter.sv
//
// WHY THIS IS THE FIRST ACCELERATOR TO BUILD. Every 2D operation a program
// does -- clearing the screen, drawing a sprite, scrolling a window -- is a
// rectangle fill or a rectangle copy. The CPU does them one byte at a time
// through a 16-bit bus at roughly eight clocks a pixel. This does a fill in
// one clock a pixel and a copy in two, against on-chip memory that nothing
// else is using, so it costs the rest of the machine nothing.
//
// It is deliberately NOT a GPU. There is no programmable anything: three
// fixed operations, eight registers, and a busy bit. That is what the era's
// hardware did, and it is what the inner loops actually need.
//
// SHARING THE FRAMEBUFFER PORT. The framebuffer is a true dual-port block RAM
// with both ports already spoken for -- the CPU writes one, the video scan-out
// reads the other -- so this cannot have a port of its own. It shares the CPU
// side, and the CPU WINS every time: `stall` holds the blitter still for any
// cycle the CPU touches the aperture. In practice that costs nothing, because
// a program that has started a blit is polling the busy bit rather than
// writing pixels, but "in practice" is not a guarantee and the arbitration is
// cheap.
//
// THE REGISTERS, at I/O 0330-033F (this design is not PC register compatible;
// see the port map in the README):
//
//   0330  DST      destination byte offset into the aperture
//   0332  SRC      source byte offset (copies only)
//   0334  WIDTH    pixels per row
//   0336  HEIGHT   rows
//   0338  DSTSTEP  bytes from one destination row to the next
//   033A  SRCSTEP  bytes from one source row to the next
//   033C  COLOUR   low byte = fill colour, high byte = transparent key
//   033E  CMD      write: bit 0 starts, bits 2:1 select the operation
//                  read:  bit 0 is busy
//
// Operations: 0 fill, 1 copy, 2 copy skipping pixels equal to the key.
//
// STRIDES ARE SEPARATE from width on purpose. Copying a 32x32 sprite out of a
// 320-wide screen into a 320-wide screen needs width 32 and both steps 320; a
// sprite packed 32 wide in its own little bitmap needs SRCSTEP 32. One number
// cannot express both, and getting it wrong produces a picture that is
// diagonally sheared -- recognisable once seen, baffling the first time.
//
// A WIDTH OR HEIGHT OF ZERO DOES NOTHING, rather than wrapping the counter
// round and writing 65,536 rows over the whole aperture. Clipping code
// produces empty rectangles constantly.
// ---------------------------------------------------------------------------

`timescale 1ns/1ns
module blitter (
    input  logic        clk,
    input  logic        rst_n,

    // register interface
    input  logic        reg_sel,
    input  logic [2:0]  reg_num,          // (port - 0330h) >> 1
    input  logic        reg_rd,
    input  logic        reg_wr,
    input  logic [15:0] reg_wdata,
    output logic [15:0] reg_rdata,

    // framebuffer port, shared with the CPU
    input  logic        stall,            // the CPU is using the port this cycle
    output logic [14:0] fb_addr,
    output logic [15:0] fb_wdata,
    output logic [1:0]  fb_be,
    output logic        fb_we,
    input  logic [15:0] fb_rdata,

    output logic        busy
);

    localparam logic [2:0] R_DST     = 3'd0;
    localparam logic [2:0] R_SRC     = 3'd1;
    localparam logic [2:0] R_WIDTH   = 3'd2;
    localparam logic [2:0] R_HEIGHT  = 3'd3;
    localparam logic [2:0] R_DSTSTEP = 3'd4;
    localparam logic [2:0] R_SRCSTEP = 3'd5;
    localparam logic [2:0] R_COLOUR  = 3'd6;
    localparam logic [2:0] R_CMD     = 3'd7;

    localparam logic [1:0] OP_FILL   = 2'd0;
    localparam logic [1:0] OP_COPY   = 2'd1;
    localparam logic [1:0] OP_TCOPY  = 2'd2;

    localparam logic [1:0] S_IDLE = 2'd0;
    localparam logic [1:0] S_WR   = 2'd1;
    localparam logic [1:0] S_RD   = 2'd2;
    localparam logic [1:0] S_RDW  = 2'd3;

    logic [15:0] r_dst, r_src, r_width, r_height, r_dststep, r_srcstep, r_colour;
    logic [1:0]  op;

    logic [1:0]  state;
    logic [15:0] dst_row, src_row;        // start of the row being drawn
    logic [15:0] dst_cur, src_cur;        // the pixel within it
    logic [15:0] x, y;
    logic [7:0]  pixel;                   // what is about to be written

    assign busy = (state != S_IDLE);

    always_comb begin
        case (reg_num)
            R_DST:     reg_rdata = r_dst;
            R_SRC:     reg_rdata = r_src;
            R_WIDTH:   reg_rdata = r_width;
            R_HEIGHT:  reg_rdata = r_height;
            R_DSTSTEP: reg_rdata = r_dststep;
            R_SRCSTEP: reg_rdata = r_srcstep;
            R_COLOUR:  reg_rdata = r_colour;
            default:   reg_rdata = {15'h0000, busy};
        endcase
    end

    // A byte offset picks a word and a lane. Writing the byte into BOTH lanes
    // means the byte enable alone decides where it lands, with no shifting.
    logic [15:0] addr_byte;
    assign addr_byte = (state == S_RD || state == S_RDW) ? src_cur : dst_cur;
    assign fb_addr   = addr_byte[15:1];
    assign fb_wdata  = {pixel, pixel};
    assign fb_be     = dst_cur[0] ? 2'b10 : 2'b01;

    logic [7:0] src_byte;
    assign src_byte = src_cur[0] ? fb_rdata[15:8] : fb_rdata[7:0];

    // Skipped pixels still cost their cycles -- only the write is suppressed.
    logic transparent;
    assign transparent = (op == OP_TCOPY) && (src_byte == r_colour[15:8]);

    assign fb_we = (state == S_WR) && !stall && !transparent;

    // True on the cycle a pixel is finished, which is what advances the walk.
    logic step;
    assign step = (state == S_WR) && !stall;

    logic last_x, last_y;
    assign last_x = (x + 16'd1 >= r_width);
    assign last_y = (y + 16'd1 >= r_height);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_dst <= 16'h0000; r_src <= 16'h0000;
            r_width <= 16'h0000; r_height <= 16'h0000;
            r_dststep <= 16'h0000; r_srcstep <= 16'h0000;
            r_colour <= 16'h0000;
            op <= OP_FILL;
            state <= S_IDLE;
            dst_row <= 16'h0000; src_row <= 16'h0000;
            dst_cur <= 16'h0000; src_cur <= 16'h0000;
            x <= 16'h0000; y <= 16'h0000;
            pixel <= 8'h00;
        end else begin
            if (reg_sel && reg_wr && !busy) begin
                case (reg_num)
                    R_DST:     r_dst     <= reg_wdata;
                    R_SRC:     r_src     <= reg_wdata;
                    R_WIDTH:   r_width   <= reg_wdata;
                    R_HEIGHT:  r_height  <= reg_wdata;
                    R_DSTSTEP: r_dststep <= reg_wdata;
                    R_SRCSTEP: r_srcstep <= reg_wdata;
                    R_COLOUR:  r_colour  <= reg_wdata;
                    default: begin
                        // Starting with nothing to draw must do nothing at
                        // all; clipping produces empty rectangles constantly.
                        if (reg_wdata[0] && (r_width != 16'd0)
                                         && (r_height != 16'd0)) begin
                            op      <= reg_wdata[2:1];
                            dst_row <= r_dst;  dst_cur <= r_dst;
                            src_row <= r_src;  src_cur <= r_src;
                            x <= 16'd0;        y <= 16'd0;
                            pixel <= r_colour[7:0];
                            state <= (reg_wdata[2:1] == OP_FILL) ? S_WR : S_RD;
                        end
                    end
                endcase
            end

            case (state)
                S_IDLE: ;

                // The framebuffer read is registered, so the address goes out
                // in one cycle and the data arrives in the next.
                S_RD:  if (!stall) state <= S_RDW;
                S_RDW: begin
                    pixel <= src_byte;
                    state <= S_WR;
                end

                S_WR: if (!stall) begin
                    if (last_x && last_y) begin
                        state <= S_IDLE;
                    end else begin
                        if (last_x) begin
                            x       <= 16'd0;
                            y       <= y + 16'd1;
                            dst_row <= dst_row + r_dststep;
                            src_row <= src_row + r_srcstep;
                            dst_cur <= dst_row + r_dststep;
                            src_cur <= src_row + r_srcstep;
                        end else begin
                            x       <= x + 16'd1;
                            dst_cur <= dst_cur + 16'd1;
                            src_cur <= src_cur + 16'd1;
                        end
                        state <= (op == OP_FILL) ? S_WR : S_RD;
                    end
                end

                default: state <= S_IDLE;
            endcase

            if (step && (op == OP_FILL)) pixel <= r_colour[7:0];
        end
    end

endmodule
