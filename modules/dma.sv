// ---------------------------------------------------------------------------
// dma — 80186 integrated DMA unit (2 independent channels).
//
// Hierarchy: cpu_top -> pcb -> dma
// Reference: learnings/04-integrated-peripherals.md §4
// Testbench: sim/tb_dma.sv
//
// Each channel has a 20-bit source pointer, a 20-bit destination pointer, a
// 16-bit transfer count and a control register. Pointers address the full 1 MB
// space linearly -- there is no segmentation here, which is why they are 20
// bits rather than 16.
//
// REGISTER MAP (PCB offsets). Each channel occupies sixteen bytes:
//   C0h/D0h  source pointer, low 16 bits
//   C2h/D2h  source pointer, high 4 bits (in bits 3:0)
//   C4h/D4h  destination pointer, low 16 bits
//   C6h/D6h  destination pointer, high 4 bits
//   C8h/D8h  transfer count
//   CAh/DAh  control
// learnings/04 records the block as "C0h-DAh / CAh-DEh", which overlaps and
// looks like an extraction artifact; the sixteen-byte-per-channel layout above
// is the standard one and spans exactly that range. Worth confirming against
// docs/80186_datasheet.pdf before running real 80186 software.
//
// CONTROL REGISTER BITS
//   15 DST M/IO   14 DST DEC   13 DST INC
//   12 SRC M/IO   11 SRC DEC   10 SRC INC
//    9 TC          8 INT        7:6 SYN      5 P
//    4 TDRQ        2 CHG        1 ST/STOP    0 B/W
// As with the timer, ST/STOP only changes when CHG is set in the same write,
// so software can adjust other bits without accidentally starting or stopping
// a channel.
//
// EVERY TRANSFER IS TWO BUS CYCLES -- fetch, then deposit -- and the pair is
// ATOMIC: bus_req stays asserted across both, so nothing can interleave
// between them. That is the same bus-cycle-atomicity rule the BIU is built
// around. bus_req then drops between transfers, which is what lets the CPU
// make progress rather than being starved by an unsynchronised channel.
//
// NOT IMPLEMENTED: the destination-synchronised idle gap, and the priority bit
// (channel 0 always wins). Neither matters until something actually contends.
// ---------------------------------------------------------------------------

`timescale 1ns/1ns
module dma
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

    // ---- request sources ----
    input  logic        drq0,
    input  logic        drq1,
    input  logic        timer2_dma_req,

    // ---- bus mastering, through the BIU ----
    // Two separate signals, and the distinction matters: bus_hold keeps
    // OWNERSHIP across the whole fetch/deposit pair so nothing can interleave
    // between them, while bus_req asks for one individual cycle and must fall
    // in between, because that is how the BIU's request handshake ends a
    // transfer. Driving both from one signal deadlocks: the BIU never sees the
    // request drop, so it never starts the second cycle.
    output logic        bus_hold,
    output logic        bus_req,
    input  logic        bus_gnt,
    output logic [19:0] bus_addr,
    output logic        bus_wr,
    output logic        bus_io,
    output logic        bus_word,
    output logic [15:0] bus_wdata,
    input  logic [15:0] bus_rdata,
    input  logic        bus_done,
    output logic        dma_active,      // the S6 equivalent

    // ---- interrupts ----
    output logic        dma0_irq,
    output logic        dma1_irq
);

    localparam int B_DST_MIO = 15, B_DST_DEC = 14, B_DST_INC = 13;
    localparam int B_SRC_MIO = 12, B_SRC_DEC = 11, B_SRC_INC = 10;
    localparam int B_TC = 9, B_INT = 8, B_P = 5, B_TDRQ = 4;
    localparam int B_CHG = 2, B_ST = 1, B_BW = 0;

    logic [19:0] src   [0:1];
    logic [19:0] dst   [0:1];
    logic [15:0] count [0:1];
    logic [15:0] ctrl  [0:1];

    logic [1:0] irq_r;
    assign dma0_irq = irq_r[0];
    assign dma1_irq = irq_r[1];

    // ---- which channels want to run ----
    logic [1:0] drq;
    assign drq = {drq1, drq0};

    logic [1:0] wants;
    always_comb begin
        for (int c = 0; c < 2; c++) begin
            if (!ctrl[c][B_ST]) begin
                wants[c] = 1'b0;
            end else if (ctrl[c][7:6] == 2'b00) begin
                // Unsynchronised: run continuously until the count expires.
                wants[c] = 1'b1;
            end else begin
                wants[c] = drq[c] || (ctrl[c][B_TDRQ] && timer2_dma_req);
            end
        end
    end

    // Channel 0 wins a tie; the programmable priority bit is not implemented.
    logic       any_want;
    logic       chan;
    assign any_want = |wants;
    assign chan     = wants[0] ? 1'b0 : 1'b1;

    localparam logic [2:0] S_IDLE    = 3'd0;
    localparam logic [2:0] S_FETCH   = 3'd1;
    localparam logic [2:0] S_GAP     = 3'd2;
    localparam logic [2:0] S_DEPOSIT = 3'd3;
    localparam logic [2:0] S_UPDATE  = 3'd4;

    logic [2:0]  state;
    logic        cur;                 // channel currently being serviced
    logic [15:0] data_r;

    logic [19:0] step;
    assign step = ctrl[cur][B_BW] ? 20'd2 : 20'd1;   // B/W set means word

    // Ownership spans the whole transfer; the per-cycle request drops during
    // S_GAP so the BIU can retire the fetch and accept the deposit.
    assign bus_hold   = (state == S_FETCH) || (state == S_GAP) ||
                        (state == S_DEPOSIT);
    assign bus_req    = (state == S_FETCH) || (state == S_DEPOSIT);
    assign dma_active = bus_hold;

    always_comb begin
        bus_addr  = 20'h00000;
        bus_wr    = 1'b0;
        bus_io    = 1'b0;
        bus_word  = ctrl[cur][B_BW];
        bus_wdata = data_r;
        case (state)
            S_FETCH: begin
                bus_addr = src[cur];
                bus_io   = ~ctrl[cur][B_SRC_MIO];
            end
            S_DEPOSIT: begin
                bus_addr = dst[cur];
                bus_io   = ~ctrl[cur][B_DST_MIO];
                bus_wr   = 1'b1;
            end
            default: ;
        endcase
    end

    // ---- register access ----
    logic [7:0] off;
    assign off = {pcb_off, 1'b0};

    logic       reg_chan;
    logic [3:0] reg_idx;
    assign reg_chan = off[4];              // C0-CF is channel 0, D0-DF channel 1
    assign reg_idx  = off[3:0];

    always_comb begin
        case (reg_idx)
            4'h0:    pcb_rdata = src[reg_chan][15:0];
            4'h2:    pcb_rdata = {12'h000, src[reg_chan][19:16]};
            4'h4:    pcb_rdata = dst[reg_chan][15:0];
            4'h6:    pcb_rdata = {12'h000, dst[reg_chan][19:16]};
            4'h8:    pcb_rdata = count[reg_chan];
            4'hA:    pcb_rdata = ctrl[reg_chan];
            default: pcb_rdata = 16'h0000;
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int c = 0; c < 2; c++) begin
                src[c]   <= 20'h00000;
                dst[c]   <= 20'h00000;
                count[c] <= 16'h0000;
                // Reset clears Start/Stop on both channels and aborts any
                // transfer in progress.
                ctrl[c]  <= 16'h0000;
            end
            state  <= S_IDLE;
            cur    <= 1'b0;
            data_r <= 16'h0000;
            irq_r  <= 2'b00;
        end else begin
            irq_r <= 2'b00;

            // ---- transfer engine ----
            case (state)
                S_IDLE: begin
                    if (any_want) begin
                        cur   <= chan;
                        state <= S_FETCH;
                    end
                end

                S_FETCH: begin
                    if (bus_gnt && bus_done) begin
                        data_r <= bus_rdata;
                        state  <= S_GAP;
                    end
                end

                // The BIU needs its request line to fall between transfers.
                S_GAP: state <= S_DEPOSIT;

                S_DEPOSIT: begin
                    if (bus_gnt && bus_done) state <= S_UPDATE;
                end

                S_UPDATE: begin
                    if (ctrl[cur][B_SRC_INC])      src[cur] <= src[cur] + step;
                    else if (ctrl[cur][B_SRC_DEC]) src[cur] <= src[cur] - step;
                    if (ctrl[cur][B_DST_INC])      dst[cur] <= dst[cur] + step;
                    else if (ctrl[cur][B_DST_DEC]) dst[cur] <= dst[cur] - step;

                    count[cur] <= count[cur] - 16'd1;

                    // A count of one means this was the last transfer. Zero
                    // programmed means 65536, which falls out of the wrap.
                    if (count[cur] == 16'd1) begin
                        if (ctrl[cur][B_TC])  ctrl[cur][B_ST] <= 1'b0;
                        if (ctrl[cur][B_INT]) irq_r[cur]      <= 1'b1;
                    end
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase

            // ---- register writes ----
            if (sel && pcb_we) begin
                case (reg_idx)
                    4'h0: src[reg_chan][15:0]   <= pcb_wdata;
                    4'h2: src[reg_chan][19:16]  <= pcb_wdata[3:0];
                    4'h4: dst[reg_chan][15:0]   <= pcb_wdata;
                    4'h6: dst[reg_chan][19:16]  <= pcb_wdata[3:0];
                    4'h8: count[reg_chan]       <= pcb_wdata;
                    4'hA: begin
                        // Start/Stop only moves when CHG is set in the same
                        // write, so other bits can be updated safely while a
                        // channel is running.
                        ctrl[reg_chan] <= {pcb_wdata[15:2], 1'b0, pcb_wdata[0]};
                        if (pcb_wdata[B_CHG]) ctrl[reg_chan][B_ST] <= pcb_wdata[B_ST];
                        else                  ctrl[reg_chan][B_ST] <= ctrl[reg_chan][B_ST];
                    end
                    default: ;
                endcase
            end
        end
    end

endmodule
