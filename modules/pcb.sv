// ---------------------------------------------------------------------------
// pcb — Peripheral Control Block: the 256-byte relocatable register window.
//
// Hierarchy: cpu_top -> pcb -> {interrupt_controller, timer, dma, chip_select}
// Reference: learnings/04-integrated-peripherals.md
// Testbench: sim/tb_pic.sv (exercised through real IN/OUT instructions)
//
// The PCB holds every integrated peripheral's registers in one 256-byte block
// that can be relocated anywhere in memory or I/O space. It resets to I/O
// FF00-FFFF (relocation register 20FFh), which is why the only instruction
// form that can reach it is OUT DX,AX / IN AX,DX -- the 8-bit immediate port
// forms cannot express a port number above FFh.
//
// Accesses here never reach the external bus. cpu_top routes a matching
// request to this module instead of the BIU, which is exactly what the real
// chip does: PCB cycles are answered internally, always ignore external READY,
// and complete with no wait states.
//
// RELOCATION REGISTER BIT LAYOUT is taken from the summary in learnings/04,
// which records the reset value (20FFh) and that it holds "ET, M/IO, RMX bits
// + relocation address bits 19-8". The exact bit positions for ET and RMX are
// NOT pinned down by that summary, so only the two that matter here are
// decoded: bits 11-0 as address bits 19-8, and bit 12 as memory-vs-I/O. Those
// two are the ones the reset value constrains (20FFh -> I/O, base FF00h) and
// they are what the address decode below needs. Verify ET and RMX against
// docs/80186_datasheet.pdf before implementing the ESC trap or iRMX mode.
// ---------------------------------------------------------------------------

`timescale 1ns/1ns
module pcb
    import cpu_pkg::*;
(
    input  logic        clk,
    input  logic        rst_n,

    // ---- CPU request (routed here by cpu_top when `hit` is set) ----
    input  logic        req,
    input  logic        req_wr,
    input  logic        req_io,
    input  logic [19:0] req_addr,
    input  logic [15:0] req_wdata,
    output logic [15:0] req_rdata,
    output logic        req_done,
    output logic        hit,          // combinational: address is inside the PCB

    // ---- shared sub-peripheral bus ----
    output logic [7:1]  pcb_off,
    output logic [15:0] pcb_wdata,
    output logic        pcb_we,
    output logic        pcb_re,

    output logic        ic_sel,
    output logic        timer_sel,
    output logic        dma_sel,
    output logic        cs_sel,

    input  logic [15:0] ic_rdata,
    input  logic [15:0] timer_rdata,
    input  logic [15:0] dma_rdata,
    input  logic [15:0] cs_rdata
);

    localparam logic [7:0] OFF_RELOC = 8'hFE;

    logic [15:0] reloc;
    logic        m_io;                 // 0 = PCB lives in I/O space, 1 = memory
    assign m_io = reloc[12];

    logic [7:0] offset;
    assign offset = req_addr[7:0];

    // Address match. In I/O space only the low 16 address bits exist, so the
    // comparison is against bits 15-8 of the programmed base.
    always_comb begin
        if (req_io && !m_io)      hit = (req_addr[15:8] == reloc[7:0]);
        else if (!req_io && m_io) hit = (req_addr[19:8] == reloc[11:0]);
        else                      hit = 1'b0;
    end

    // Sub-range decode. Boundaries come from learnings/04; the DMA range there
    // looks like it may carry an OCR artifact, so it is treated as one block
    // C0-DF rather than two overlapping ones.
    logic in_ic, in_timer, in_cs, in_dma, in_reloc;
    assign in_ic    = (offset >= 8'h20) && (offset <= 8'h3F);
    assign in_timer = (offset >= 8'h50) && (offset <= 8'h67);
    assign in_cs    = (offset >= 8'hA0) && (offset <= 8'hA9);
    assign in_dma   = (offset >= 8'hC0) && (offset <= 8'hDF);
    assign in_reloc = (offset == OFF_RELOC);

    // `active` makes the access a single-shot: the request line stays high
    // until req_done is seen, and without this the write would repeat.
    logic active, done_r;
    logic [15:0] rdata_r;

    logic access;
    assign access = req && hit && !active;

    assign pcb_off   = offset[7:1];
    assign pcb_wdata = req_wdata;
    assign pcb_we    = access &&  req_wr;
    assign pcb_re    = access && !req_wr;
    assign ic_sel    = access && in_ic;
    assign timer_sel = access && in_timer;
    assign dma_sel   = access && in_dma;
    assign cs_sel    = access && in_cs;

    assign req_done  = done_r;
    assign req_rdata = rdata_r;

    logic [15:0] read_mux;
    always_comb begin
        if      (in_reloc) read_mux = reloc;
        else if (in_ic)    read_mux = ic_rdata;
        else if (in_timer) read_mux = timer_rdata;
        else if (in_dma)   read_mux = dma_rdata;
        else if (in_cs)    read_mux = cs_rdata;
        else               read_mux = 16'h0000;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reloc   <= 16'h20FF;       // I/O space, base FF00h
            active  <= 1'b0;
            done_r  <= 1'b0;
            rdata_r <= 16'h0000;
        end else begin
            done_r <= 1'b0;
            if (access) begin
                active  <= 1'b1;
                done_r  <= 1'b1;       // zero wait states, always
                rdata_r <= read_mux;
                if (req_wr && in_reloc) reloc <= req_wdata;
            end
            if (!req) active <= 1'b0;
        end
    end

endmodule
