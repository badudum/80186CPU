// ---------------------------------------------------------------------------
// vga_dac — the 256-entry colour palette behind mode 13h.
//
// Hierarchy: FPGA80186 -> vga_dac
//            (write side from io_decode, read side to vga_controller)
//
// In mode 13h a framebuffer byte is not a colour, it is an INDEX. This holds
// what those 256 indices mean. Without it every image is a greyscale ramp at
// best, because nothing tells the display that index 47 is a particular brown.
//
// PORTS, as the PC has them:
//   3C8  write the index that the next colour will be loaded into
//   3C9  write one component; three writes give red, green, blue in order and
//        the index then advances on its own
//
// That auto-advance is not a convenience, it is the interface: software loads
// a whole palette by writing 3C8 once and then 768 bytes to 3C9. Doom does
// exactly this, and so does every other DOS game.
//
// SIX BITS PER CHANNEL is what the real DAC takes, and what software writes:
// values run 0-63, not 0-255. Feeding them to an eight-bit output unshifted
// makes the whole picture dark by a factor of four, which looks like a broken
// palette rather than a scaling mistake. The top two bits are replicated into
// the bottom so that 63 maps to 255 rather than 252.
//
// The palette is read on clk_vga and written on clk_cpu. As with vram, that is
// what a true dual-port block RAM is for; no synchronisers belong here. A
// write landing in the same frame as a read shows one wrong pixel on one
// frame, which is invisible and is exactly what real hardware does unless
// software waits for vertical blank.
// ---------------------------------------------------------------------------

`timescale 1ns/1ns
module vga_dac (
    // CPU side
    input  logic        clk_cpu,
    input  logic        rst_n,
    input  logic        dac_wr,
    input  logic        dac_port,      // 0 = 3C8 index, 1 = 3C9 data
    input  logic [7:0]  dac_wdata,
    output logic [7:0]  dac_rdata,

    // VGA side
    input  logic        clk_vga,
    input  logic [7:0]  pal_index,
    output logic [17:0] pal_rgb        // {r[5:0], g[5:0], b[5:0]}
);

    logic [17:0] pal [0:255];

    // A sensible default so a mode switch without a palette load is not a
    // black screen: the standard VGA arrangement of sixteen EGA colours, a
    // greyscale ramp, then a colour cube. Software that cares overwrites it.
    initial begin
        for (int i = 0; i < 256; i++) pal[i] = 18'd0;
        // The sixteen CGA/EGA colours, in the 0-63 scale the DAC uses.
        pal[0]  = {6'd0,  6'd0,  6'd0 };   pal[1]  = {6'd0,  6'd0,  6'd42};
        pal[2]  = {6'd0,  6'd42, 6'd0 };   pal[3]  = {6'd0,  6'd42, 6'd42};
        pal[4]  = {6'd42, 6'd0,  6'd0 };   pal[5]  = {6'd42, 6'd0,  6'd42};
        pal[6]  = {6'd42, 6'd21, 6'd0 };   pal[7]  = {6'd42, 6'd42, 6'd42};
        pal[8]  = {6'd21, 6'd21, 6'd21};   pal[9]  = {6'd21, 6'd21, 6'd63};
        pal[10] = {6'd21, 6'd63, 6'd21};   pal[11] = {6'd21, 6'd63, 6'd63};
        pal[12] = {6'd63, 6'd21, 6'd21};   pal[13] = {6'd63, 6'd21, 6'd63};
        pal[14] = {6'd63, 6'd63, 6'd21};   pal[15] = {6'd63, 6'd63, 6'd63};
        // 16-31: a greyscale ramp, as the VGA default has it.
        for (int i = 0; i < 16; i++)
            pal[16 + i] = {6'(i * 4), 6'(i * 4), 6'(i * 4)};
        // 32-255: a coarse colour cube, so an unloaded palette still shows
        // structure rather than black.
        for (int i = 0; i < 224; i++)
            pal[32 + i] = {6'((i / 36) * 12), 6'(((i / 6) % 6) * 12),
                           6'((i % 6) * 12)};
    end

    // ---- CPU side: the 3C8/3C9 state machine ----
    logic [7:0] wr_index;
    logic [1:0] wr_phase;               // 0 = red, 1 = green, 2 = blue
    logic [5:0] hold_r, hold_g;

    assign dac_rdata = dac_port ? {2'b00, (wr_phase == 2'd0) ? hold_r
                                        : (wr_phase == 2'd1) ? hold_g : 6'd0}
                                : wr_index;

    always @(posedge clk_cpu or negedge rst_n) begin
        if (!rst_n) begin
            wr_index <= 8'h00;
            wr_phase <= 2'd0;
            hold_r   <= 6'd0;
            hold_g   <= 6'd0;
        end else if (dac_wr) begin
            if (!dac_port) begin
                // Writing the index always restarts at the red component.
                wr_index <= dac_wdata;
                wr_phase <= 2'd0;
            end else begin
                case (wr_phase)
                    2'd0: begin hold_r <= dac_wdata[5:0]; wr_phase <= 2'd1; end
                    2'd1: begin hold_g <= dac_wdata[5:0]; wr_phase <= 2'd2; end
                    default: begin
                        pal[wr_index] <= {hold_r, hold_g, dac_wdata[5:0]};
                        wr_index      <= wr_index + 8'd1;
                        wr_phase      <= 2'd0;
                    end
                endcase
            end
        end
    end

    // ---- VGA side ----
    always_ff @(posedge clk_vga) pal_rgb <= pal[pal_index];

endmodule
