// ---------------------------------------------------------------------------
// timer — 80186 integrated timer unit (timers 0, 1, 2).
//
// Hierarchy: cpu_top -> pcb -> timer
// Reference: learnings/04-integrated-peripherals.md §5
// Testbench: sim/tb_timer.sv
//
// Three 16-bit timers. Timers 0 and 1 have an input pin, an output pin and two
// max-count registers; timer 2 has neither pin and only one max-count
// register, and doubles as a prescaler for the other two and as a DMA trigger.
//
// REGISTER MAP (PCB offsets)
//   50h T0 count   52h T0 max A   54h T0 max B   56h T0 control
//   58h T1 count   5Ah T1 max A   5Ch T1 max B   5Eh T1 control
//   60h T2 count   62h T2 max A   --             66h T2 control
//
// CONTROL REGISTER BITS
//   15 EN    enable counting
//   14 INH   write-inhibit: EN only changes when INH is set in the SAME write,
//            which is what lets software update other bits without disturbing
//            whether the timer is running
//   13 INT   raise an interrupt when max count is reached
//   12 RIU   read-only: which max-count register is in use (0 = A, 1 = B)
//    5 MC    max count reached; sticky, cleared by software
//    4 RTG   retrigger on input transition (NOT IMPLEMENTED)
//    3 P     prescaler: count timer 2 timeouts instead of the internal tick
//    2 EXT   count external transitions on TMRIN instead of the internal tick
//    1 ALT   alternate between max-count A and B (timers 0 and 1 only)
//    0 CONT  continuous; when clear the timer disables itself after one cycle
//
// The bit POSITIONS above are this implementation's reading of the standard
// 80186 layout; learnings/04 names the bits but does not record their
// positions. Verify against docs/80186_datasheet.pdf before running software
// written for real hardware. The offsets are firmer -- they follow the
// documented 50h-66h range and the natural four-register-per-timer spacing.
//
// TICK RATE: the internal count source is the CPU clock divided by 4, matching
// the real part (2 MHz at an 8 MHz CPU clock). That divider is what sets the
// achievable interrupt periods, so a DOS-style 18.2 Hz tick comes from
// programming a large max count rather than from anything here.
//
// NOT IMPLEMENTED: RTG (retrigger/one-shot input mode), and the real chip's
// time-multiplexing of one physical counter across three register banks --
// here all three count independently, which is observably identical except for
// the one-wait-state access timing the multiplexing caused.
// ---------------------------------------------------------------------------

`timescale 1ns/1ns
module timer
    import cpu_pkg::*;
(
    input  logic        clk,
    input  logic        rst_n,

    // ---- PCB bus ----
    input  logic        sel,
    input  logic [7:1]  pcb_off,
    input  logic [15:0] pcb_wdata,
    input  logic        pcb_we,
    input  logic        pcb_re,
    output logic [15:0] pcb_rdata,

    // ---- timer 0/1 external pins ----
    input  logic        tmrin0,
    input  logic        tmrin1,
    output logic        tmrout0,
    output logic        tmrout1,

    // ---- interrupt requests ----
    output logic        timer0_irq,
    output logic        timer1_irq,
    output logic        timer2_irq,

    // ---- timer 2 timeout as a DMA trigger ----
    output logic        timer2_dma_req
);

    localparam int B_EN = 15, B_INH = 14, B_INT = 13, B_RIU = 12;
    localparam int B_MC = 5, B_RTG = 4, B_P = 3, B_EXT = 2, B_ALT = 1, B_CONT = 0;

    logic [15:0] count [0:2];
    logic [15:0] maxa  [0:2];
    logic [15:0] maxb  [0:2];
    logic [15:0] ctrl  [0:2];
    logic        riu   [0:2];

    // CPU clock / 4 internal tick.
    logic [1:0] presc;
    logic       tick;
    assign tick = (presc == 2'd3);
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) presc <= 2'd0;
        else        presc <= presc + 2'd1;
    end

    // TMRIN edge detection (timers 0 and 1 only).
    logic tin0_s, tin0_p, tin1_s, tin1_p;
    logic tin0_edge, tin1_edge;
    assign tin0_edge = tin0_s && !tin0_p;
    assign tin1_edge = tin1_s && !tin1_p;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tin0_s <= 1'b0; tin0_p <= 1'b0;
            tin1_s <= 1'b0; tin1_p <= 1'b0;
        end else begin
            tin0_s <= tmrin0; tin0_p <= tin0_s;
            tin1_s <= tmrin1; tin1_p <= tin1_s;
        end
    end

    // Timer 2 rollover drives the prescaler input of timers 0 and 1, so it is
    // computed before them.
    logic [2:0] expire;          // max count reached this cycle
    logic       t2_expire;
    assign t2_expire = expire[2];

    // Per-timer count enable.
    logic [2:0] do_count;
    always_comb begin
        for (int i = 0; i < 3; i++) begin
            if (!ctrl[i][B_EN]) do_count[i] = 1'b0;
            else if (i == 2)    do_count[i] = tick;          // timer 2: internal only
            else if (ctrl[i][B_EXT]) do_count[i] = (i == 0) ? tin0_edge : tin1_edge;
            else if (ctrl[i][B_P])   do_count[i] = t2_expire; // prescaled by timer 2
            else                     do_count[i] = tick;
        end
    end

    // Target max count for each timer this cycle.
    logic [15:0] target [0:2];
    always_comb begin
        for (int i = 0; i < 3; i++)
            target[i] = (ctrl[i][B_ALT] && riu[i] && (i != 2)) ? maxb[i] : maxa[i];
    end

    always_comb begin
        for (int i = 0; i < 3; i++)
            expire[i] = do_count[i] && ((count[i] + 16'd1) >= target[i]) && (target[i] != 16'd0);
    end

    assign timer0_irq     = expire[0] && ctrl[0][B_INT];
    assign timer1_irq     = expire[1] && ctrl[1][B_INT];
    assign timer2_irq     = expire[2] && ctrl[2][B_INT];
    assign timer2_dma_req = expire[2];

    // In alternating mode the output tracks which max-count register is in
    // use, giving a square wave whose halves are maxA and maxB. Otherwise it
    // sits high and pulses low on expiry, matching the reset state (ALT=1,
    // RIU=0) leaving the pins high.
    assign tmrout0 = ctrl[0][B_ALT] ? ~riu[0] : ~expire[0];
    assign tmrout1 = ctrl[1][B_ALT] ? ~riu[1] : ~expire[1];

    // ---- register access ----
    logic [7:0] off;
    assign off = {pcb_off, 1'b0};

    always_comb begin
        case (off)
            8'h50:   pcb_rdata = count[0];
            8'h52:   pcb_rdata = maxa[0];
            8'h54:   pcb_rdata = maxb[0];
            8'h56:   pcb_rdata = {ctrl[0][15:13], riu[0], ctrl[0][11:0]};
            8'h58:   pcb_rdata = count[1];
            8'h5A:   pcb_rdata = maxa[1];
            8'h5C:   pcb_rdata = maxb[1];
            8'h5E:   pcb_rdata = {ctrl[1][15:13], riu[1], ctrl[1][11:0]};
            8'h60:   pcb_rdata = count[2];
            8'h62:   pcb_rdata = maxa[2];
            8'h66:   pcb_rdata = {ctrl[2][15:13], riu[2], ctrl[2][11:0]};
            default: pcb_rdata = 16'h0000;
        endcase
    end

    logic       wr_ctrl;
    logic [1:0] wr_idx;
    always_comb begin
        wr_ctrl = 1'b0;
        wr_idx  = 2'd0;
        case (off)
            8'h56: begin wr_ctrl = 1'b1; wr_idx = 2'd0; end
            8'h5E: begin wr_ctrl = 1'b1; wr_idx = 2'd1; end
            8'h66: begin wr_ctrl = 1'b1; wr_idx = 2'd2; end
            default: ;
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 3; i++) begin
                count[i] <= 16'h0000;
                maxa[i]  <= 16'h0000;
                maxb[i]  <= 16'h0000;
                // Reset clears EN (nothing counts) and sets ALT, which with
                // RIU=0 leaves the TMROUT pins high.
                ctrl[i]  <= 16'h0002;
                riu[i]   <= 1'b0;
            end
        end else begin
            // ---- counting ----
            for (int i = 0; i < 3; i++) begin
                if (expire[i]) begin
                    count[i] <= 16'h0000;
                    ctrl[i][B_MC] <= 1'b1;                 // sticky, software clears
                    if (ctrl[i][B_ALT] && (i != 2)) riu[i] <= ~riu[i];
                    // A non-continuous timer stops itself after one full cycle.
                    // In alternating mode that means after the B half.
                    if (!ctrl[i][B_CONT]) begin
                        if (!ctrl[i][B_ALT] || riu[i]) ctrl[i][B_EN] <= 1'b0;
                    end
                end else if (do_count[i]) begin
                    count[i] <= count[i] + 16'd1;
                end
            end

            // ---- register writes ----
            if (sel && pcb_we) begin
                case (off)
                    8'h50: count[0] <= pcb_wdata;
                    8'h52: maxa[0]  <= pcb_wdata;
                    8'h54: maxb[0]  <= pcb_wdata;
                    8'h58: count[1] <= pcb_wdata;
                    8'h5A: maxa[1]  <= pcb_wdata;
                    8'h5C: maxb[1]  <= pcb_wdata;
                    8'h60: count[2] <= pcb_wdata;
                    8'h62: maxa[2]  <= pcb_wdata;
                    default: ;
                endcase

                if (wr_ctrl) begin
                    // Everything except EN updates unconditionally. EN only
                    // changes if INH is set in this same write -- that is the
                    // whole point of INH, and it is why software can safely
                    // clear MC without accidentally stopping the timer.
                    ctrl[wr_idx][13:0] <= pcb_wdata[13:0];
                    if (pcb_wdata[B_INH]) ctrl[wr_idx][B_EN] <= pcb_wdata[B_EN];
                    ctrl[wr_idx][B_INH] <= 1'b0;
                end
            end
        end
    end

endmodule
