// ---------------------------------------------------------------------------
// chip_select — 80186 chip-select / ready-generation unit.
//
// Hierarchy: cpu_top -> pcb -> chip_select
// Reference: learnings/04-integrated-peripherals.md §3
// Testbench: sim/tb_chipsel.sv
//
// REGISTER MAP (PCB offsets, per the AP-186 reset example in learnings/04)
//   A0h UMCS   upper memory: block ending at FFFFF, programmable size
//   A2h LMCS   lower memory: block starting at 00000, programmable size
//   A4h PACS   peripheral chip selects PCS0-3 base + ready
//   A8h MPCS   mid-range / PCS4-6 configuration + ready
//
// REGISTER FORMAT
//   bits 15:6  address bits A19-A10 (block start for UMCS, block end for LMCS,
//              base for PACS)
//   bit  2     R2: 1 = ignore external READY entirely
//   bits 1:0   R1,R0: 0-3 wait states
// UMCS resets to FFFBh, which decodes as: start address 3FFh -> FFC00, i.e.
// the top 1 KB, with 3 wait states and external ready factored in. That reset
// value is the mechanism that makes the reset vector at FFFF0 fetchable before
// any software has run, and it is the single most important thing in this
// module.
//
// The 15:6 / 2:0 split above is confirmed by the reset value decoding exactly
// as documented (FFFB -> top 1 KB, 3 wait states). The MPCS bit fields that
// select mid-range block size and whether peripherals live in memory or I/O
// space are NOT pinned down by learnings/04, so mid-range MCS decode and the
// PCS5/6-as-A1/A2 alternate function are left unimplemented rather than
// guessed. Check docs/80186_datasheet.pdf before relying on them.
//
// WHAT THIS ACTUALLY DOES ON AN FPGA: the chip-select output pins drive
// nothing, because memory here is on-chip rather than discrete parts needing
// select lines. What matters is (a) the register interface, so 80186 boot code
// that programs UMCS/LMCS/PACS/MPCS behaves, and (b) the wait-state outputs,
// which are real information the BIU could consume. The decode is implemented
// for fidelity and for the day external parts hang off the GPIO headers.
// ---------------------------------------------------------------------------

`timescale 1ns/1ns
module chip_select
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

    // ---- address being decoded ----
    input  logic [19:0] addr,
    input  logic        io_space,

    // ---- chip-select outputs (active low) ----
    output logic        ucs_n,
    output logic        lcs_n,
    output logic [3:0]  mcs_n,
    output logic [6:0]  pcs_n,

    // ---- wait-state info for the decoded region ----
    output logic [1:0]  wait_cnt,
    output logic        use_ext_ready
);

    logic [15:0] umcs, lmcs, pacs, mpcs;

    // No PCS line goes active until both PACS and MPCS have been accessed at
    // least once -- a documented quirk that stops peripheral selects glitching
    // before software has configured where they live.
    logic pacs_seen, mpcs_seen;

    logic [7:0] off;
    assign off = {pcb_off, 1'b0};

    always_comb begin
        case (off)
            8'hA0:   pcb_rdata = umcs;
            8'hA2:   pcb_rdata = lmcs;
            8'hA4:   pcb_rdata = pacs;
            8'hA8:   pcb_rdata = mpcs;
            default: pcb_rdata = 16'h0000;
        endcase
    end

    // ---- region decode ----
    logic [9:0] a_hi;
    assign a_hi = addr[19:10];

    logic in_ucs, in_lcs, in_pcs;
    logic [2:0] pcs_idx;

    // UCS covers from its programmed start up to the top of memory.
    assign in_ucs = !io_space && (a_hi >= umcs[15:6]);
    // LCS covers from zero up to its programmed end.
    assign in_lcs = !io_space && (a_hi <= lmcs[15:6]);

    // PCS0-6 are seven contiguous 128-byte blocks above the PACS base. The
    // base lives in I/O space by default, which is where the PCB itself sits.
    logic [19:0] pacs_base;
    assign pacs_base = {pacs[15:6], 10'h000};

    logic [19:0] pcs_delta;
    assign pcs_delta = addr - pacs_base;

    always_comb begin
        in_pcs  = 1'b0;
        pcs_idx = 3'd0;
        if (pacs_seen && mpcs_seen && (addr >= pacs_base) && (pcs_delta < 20'd896)) begin
            in_pcs  = 1'b1;
            pcs_idx = pcs_delta[9:7];     // 128 bytes per block
        end
    end

    assign ucs_n = !in_ucs;
    assign lcs_n = !in_lcs;
    assign mcs_n = 4'hF;                  // mid-range decode not implemented
    always_comb begin
        pcs_n = 7'h7F;
        if (in_pcs) pcs_n[pcs_idx] = 1'b0;
    end

    // Wait states follow whichever region matched. UCS wins when regions
    // overlap, which the datasheet discourages anyway.
    always_comb begin
        if (in_ucs) begin
            wait_cnt      = umcs[1:0];
            use_ext_ready = ~umcs[2];
        end else if (in_lcs) begin
            wait_cnt      = lmcs[1:0];
            use_ext_ready = ~lmcs[2];
        end else if (in_pcs) begin
            wait_cnt      = (pcs_idx <= 3'd3) ? pacs[1:0] : mpcs[1:0];
            use_ext_ready = (pcs_idx <= 3'd3) ? ~pacs[2]  : ~mpcs[2];
        end else begin
            wait_cnt      = 2'd0;
            use_ext_ready = 1'b1;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Top 1 KB, 3 wait states, external ready factored in. This is
            // what makes FFFF0 fetchable out of reset.
            umcs      <= 16'hFFFB;
            lmcs      <= 16'h003B;
            pacs      <= 16'h007B;
            mpcs      <= 16'h00BB;
            pacs_seen <= 1'b0;
            mpcs_seen <= 1'b0;
        end else if (sel && (pcb_we || pcb_re)) begin
            // A read is enough to arm the PCS lines, per the datasheet.
            case (off)
                8'hA4: pacs_seen <= 1'b1;
                8'hA8: mpcs_seen <= 1'b1;
                default: ;
            endcase
            if (pcb_we) begin
                case (off)
                    8'hA0: umcs <= pcb_wdata;
                    8'hA2: lmcs <= pcb_wdata;
                    8'hA4: pacs <= pcb_wdata;
                    8'hA8: mpcs <= pcb_wdata;
                    default: ;
                endcase
            end
        end
    end

endmodule
