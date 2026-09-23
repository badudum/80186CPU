// ---------------------------------------------------------------------------
// interrupt_controller — 80186 integrated interrupt controller, master mode.
//
// Hierarchy: cpu_top -> pcb -> interrupt_controller
// Reference: learnings/05-interrupts-and-reset.md
// Testbench: sim/tb_pic.sv
//
// Master mode is the reset default and the only mode implemented; iRMX 86
// slave mode exists purely for Intel's own RTOS and is out of scope.
//
// In master mode the controller feeds the CPU core directly and NO external
// interrupt-acknowledge bus cycles are ever run -- the vector is handed to the
// core internally. That is a real difference from a plain 8086, which always
// runs two INTA cycles to read a type byte from an external 8259A.
//
// REGISTER MAP (PCB offsets, all word accesses)
//   22h  EOI                 write: end of interrupt
//   28h  Mask                1 = masked; all 1s at reset
//   2Ah  Priority Mask       3 bits; 7 at reset (blocks nothing)
//   2Ch  In-Service          one bit per source
//   2Eh  Interrupt Request   one bit per source
//   30h  Status              which timer fired
//   32h  Timer control       all three timers share one control register
//   34h  DMA 0 control
//   36h  DMA 1 control
//   38h  INT0 control    3Ah INT1    3Ch INT2    3Eh INT3
// Each control register: bits 2:0 = priority (0 highest), bit 3 = mask.
//
// SOURCE BIT ASSIGNMENT: the register offsets above come from the datasheet
// summary, but that summary does not record which bit of the mask/request/
// in-service registers belongs to which source. The assignment below is this
// implementation's own and should be checked against docs/80186_datasheet.pdf
// before running software written for real 80186 hardware:
//   bit 0 Timer   bit 1 DMA0   bit 2 DMA1
//   bit 3 INT0    bit 4 INT1   bit 5 INT2   bit 6 INT3
//
// VECTORS are fixed in master mode: timers 8/18/19, DMA 10/11, INT0-3 12-15.
//
// LATCHING follows the datasheet: internal sources (timers, DMA) latch, so a
// pulse is never lost, while the external INT0-3 inputs are level-sensitive
// and track their pin. A device driving one of those must hold it until the
// handler clears the condition. Edge-triggered mode (the LTM bit) is not
// implemented.
//
// PRIORITY is fully nested: a source is eligible only if it is requested,
// unmasked, not already in service, and strictly higher priority (lower
// number) than anything currently in service. EOI clears the highest-priority
// in-service bit; specific EOI is not implemented.
// ---------------------------------------------------------------------------

`timescale 1ns/1ns
module interrupt_controller
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

    // A non-specific EOI issued from OUTSIDE the peripheral control block.
    // PC software ends an interrupt by writing to an 8259 at port 20h, not to
    // this controller's own register, and a guest that does so leaves the
    // in-service bit set here forever -- which stops every later interrupt,
    // including the timer. io_decode turns that write into this pulse.
    input  logic        ext_eoi,

    // ---- external interrupt pins ----
    input  logic        int0,
    input  logic        int1,
    input  logic        int2,
    input  logic        int3,

    // ---- internal peripheral requests ----
    input  logic        timer0_irq,
    input  logic        timer1_irq,
    input  logic        timer2_irq,
    input  logic        dma0_irq,
    input  logic        dma1_irq,

    // ---- to the CPU core ----
    output logic        intr,
    output logic [7:0]  vector_type,
    input  logic        iack
);

    localparam int NSRC = 7;
    localparam int S_TIMER = 0, S_DMA0 = 1, S_DMA1 = 2;
    localparam int S_INT0  = 3, S_INT1 = 4, S_INT2 = 5, S_INT3 = 6;

    logic [NSRC-1:0] mask_r, isr_r, irr_latched;
    logic [2:0]      pri_r   [0:NSRC-1];
    logic [2:0]      prio_mask_r;
    logic            t0_lat, t1_lat, t2_lat;

    // ---- request assembly ----
    // Internal sources latch; external pins are level-sensitive.
    logic [NSRC-1:0] request;
    always_comb begin
        request            = {NSRC{1'b0}};
        request[S_TIMER]   = t0_lat | t1_lat | t2_lat;
        request[S_DMA0]    = irr_latched[S_DMA0];
        request[S_DMA1]    = irr_latched[S_DMA1];
        request[S_INT0]    = int0;
        request[S_INT1]    = int1;
        request[S_INT2]    = int2;
        request[S_INT3]    = int3;
    end

    // ---- fully-nested priority resolution ----
    // Nothing in service is represented as priority 8, so every source with a
    // 3-bit priority is strictly higher and can get through.
    logic [3:0] isr_pri;
    always_comb begin
        isr_pri = 4'd8;
        for (int i = 0; i < NSRC; i++)
            if (isr_r[i] && ({1'b0, pri_r[i]} < isr_pri)) isr_pri = {1'b0, pri_r[i]};
    end

    logic [NSRC-1:0] eligible;
    always_comb begin
        for (int i = 0; i < NSRC; i++)
            eligible[i] = request[i] && !mask_r[i] && !isr_r[i] &&
                          ({1'b0, pri_r[i]} < isr_pri) &&
                          (pri_r[i] <= prio_mask_r);
    end

    logic [2:0] winner;
    logic       any_eligible;
    always_comb begin
        winner       = 3'd0;
        any_eligible = 1'b0;
        for (int i = NSRC-1; i >= 0; i--)
            if (eligible[i]) begin
                // Scanning downwards means a lower index wins a priority tie,
                // which matches the datasheet's fixed internal ordering.
                if (!any_eligible || (pri_r[i] <= pri_r[winner])) begin
                    winner       = i[2:0];
                    any_eligible = 1'b1;
                end
            end
    end

    assign intr = any_eligible;

    // Vector numbers are fixed in master mode. All three timers share one
    // source bit, so the status register exists to tell them apart; the
    // vector follows the same fixed tiebreak (timer 0 > 1 > 2).
    always_comb begin
        case (winner)
            S_DMA0:  vector_type = 8'd10;
            S_DMA1:  vector_type = 8'd11;
            S_INT0:  vector_type = 8'd12;
            S_INT1:  vector_type = 8'd13;
            S_INT2:  vector_type = 8'd14;
            S_INT3:  vector_type = 8'd15;
            default: vector_type = t0_lat ? 8'd8 : (t1_lat ? 8'd18 : 8'd19);
        endcase
    end

    // ---- register reads ----
    logic [7:0] off;
    assign off = {pcb_off, 1'b0};

    always_comb begin
        case (off)
            8'h28:   pcb_rdata = {9'h0, mask_r};
            8'h2A:   pcb_rdata = {13'h0, prio_mask_r};
            8'h2C:   pcb_rdata = {9'h0, isr_r};
            8'h2E:   pcb_rdata = {9'h0, request};
            8'h30:   pcb_rdata = {13'h0, t2_lat, t1_lat, t0_lat};
            8'h32:   pcb_rdata = {12'h0, mask_r[S_TIMER], pri_r[S_TIMER]};
            8'h34:   pcb_rdata = {12'h0, mask_r[S_DMA0],  pri_r[S_DMA0]};
            8'h36:   pcb_rdata = {12'h0, mask_r[S_DMA1],  pri_r[S_DMA1]};
            8'h38:   pcb_rdata = {12'h0, mask_r[S_INT0],  pri_r[S_INT0]};
            8'h3A:   pcb_rdata = {12'h0, mask_r[S_INT1],  pri_r[S_INT1]};
            8'h3C:   pcb_rdata = {12'h0, mask_r[S_INT2],  pri_r[S_INT2]};
            8'h3E:   pcb_rdata = {12'h0, mask_r[S_INT3],  pri_r[S_INT3]};
            default: pcb_rdata = 16'h0000;
        endcase
    end

    // Highest-priority in-service bit, for non-specific EOI.
    logic [2:0] eoi_target;
    logic       eoi_valid;
    always_comb begin
        eoi_target = 3'd0;
        eoi_valid  = 1'b0;
        for (int i = NSRC-1; i >= 0; i--)
            if (isr_r[i]) begin
                if (!eoi_valid || (pri_r[i] <= pri_r[eoi_target])) begin
                    eoi_target = i[2:0];
                    eoi_valid  = 1'b1;
                end
            end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Everything masked, lowest priority, nothing in service. A boot
            // ROM must deliberately unmask what it wants, which is why no
            // spurious interrupt can fire before software is ready.
            mask_r      <= {NSRC{1'b1}};
            isr_r       <= {NSRC{1'b0}};
            irr_latched <= {NSRC{1'b0}};
            prio_mask_r <= 3'd7;
            t0_lat      <= 1'b0;
            t1_lat      <= 1'b0;
            t2_lat      <= 1'b0;
            for (int i = 0; i < NSRC; i++) pri_r[i] <= 3'd7;
        end else begin
            // Latch internal sources.
            if (timer0_irq) t0_lat <= 1'b1;
            if (timer1_irq) t1_lat <= 1'b1;
            if (timer2_irq) t2_lat <= 1'b1;
            if (dma0_irq)   irr_latched[S_DMA0] <= 1'b1;
            if (dma1_irq)   irr_latched[S_DMA1] <= 1'b1;

            // ---- CPU accepted the interrupt we were presenting ----
            if (iack) begin
                isr_r[winner] <= 1'b1;
                case (winner)
                    S_TIMER: begin
                        // Clear only the timer that supplied the vector.
                        if      (t0_lat) t0_lat <= 1'b0;
                        else if (t1_lat) t1_lat <= 1'b0;
                        else             t2_lat <= 1'b0;
                    end
                    S_DMA0: irr_latched[S_DMA0] <= 1'b0;
                    S_DMA1: irr_latched[S_DMA1] <= 1'b0;
                    default: ;   // external pins are level-sensitive, nothing to clear
                endcase
            end

            // An EOI from the 8259 shim clears the same bit the controller's
            // own EOI register would.
            if (ext_eoi && eoi_valid) isr_r[eoi_target] <= 1'b0;

            // ---- register writes ----
            if (sel && pcb_we) begin
                case (off)
                    8'h22: if (eoi_valid) isr_r[eoi_target] <= 1'b0;   // non-specific EOI
                    8'h28: mask_r      <= pcb_wdata[NSRC-1:0];
                    8'h2A: prio_mask_r <= pcb_wdata[2:0];
                    8'h2C: isr_r       <= pcb_wdata[NSRC-1:0];
                    8'h2E: begin
                        // Only the latched (internal) request bits are writable.
                        irr_latched[S_DMA0] <= pcb_wdata[S_DMA0];
                        irr_latched[S_DMA1] <= pcb_wdata[S_DMA1];
                    end
                    8'h32: begin mask_r[S_TIMER] <= pcb_wdata[3]; pri_r[S_TIMER] <= pcb_wdata[2:0]; end
                    8'h34: begin mask_r[S_DMA0]  <= pcb_wdata[3]; pri_r[S_DMA0]  <= pcb_wdata[2:0]; end
                    8'h36: begin mask_r[S_DMA1]  <= pcb_wdata[3]; pri_r[S_DMA1]  <= pcb_wdata[2:0]; end
                    8'h38: begin mask_r[S_INT0]  <= pcb_wdata[3]; pri_r[S_INT0]  <= pcb_wdata[2:0]; end
                    8'h3A: begin mask_r[S_INT1]  <= pcb_wdata[3]; pri_r[S_INT1]  <= pcb_wdata[2:0]; end
                    8'h3C: begin mask_r[S_INT2]  <= pcb_wdata[3]; pri_r[S_INT2]  <= pcb_wdata[2:0]; end
                    8'h3E: begin mask_r[S_INT3]  <= pcb_wdata[3]; pri_r[S_INT3]  <= pcb_wdata[2:0]; end
                    default: ;
                endcase
            end
        end
    end

endmodule
