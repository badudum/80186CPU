// ---------------------------------------------------------------------------
// cpu_top — the 80186 "chip": everything that was inside the real package.
//
// Hierarchy: FPGA80186 -> cpu_top -> {biu, eu, pcb -> {interrupt_controller,
//                                     timer, dma, chip_select}}
// Reference: learnings/00-overview.md (BIU/EU split),
//            learnings/04-integrated-peripherals.md (PCB)
//
// The boundary is deliberate: the integrated peripherals live INSIDE this
// module because they were inside the real chip, while memory_controller, VGA
// and keyboard live outside in FPGA80186.sv because they were external
// components on a real 80186 board.
//
// PCB REQUEST INTERCEPT: accesses to the peripheral control block (I/O
// FF00-FFFF by default) are answered internally and never reach the external
// bus, which is what the real chip does. The intercept sits between the EU's
// request port and the BIU rather than inside the BIU, so the BIU's bus-cycle
// FSM stays concerned only with real bus cycles.
//
// INTERRUPT SOURCES: the integrated controller drives the core directly, and
// in master mode no external interrupt-acknowledge cycles are run. The
// external intr_req/intr_type inputs are kept alongside it -- they let a
// testbench inject a vector directly, and leave room for an external PIC
// later. The integrated controller takes precedence when both are asserted.
//
// DMA ARBITRATION: the DMA unit is a real bus master. It asks for the bus, and
// ownership changes hands only while the BIU is between cycles -- granting
// mid-cycle would split a transfer in half. While the DMA owns the bus the
// EU's request is held off rather than cancelled, so the CPU simply stalls for
// a few cycles, which is what cycle stealing is.
//
// NOT IMPLEMENTED: HOLD/HLDA (there is no external bus master), LOCK, and HALT
// bus cycles.
// ---------------------------------------------------------------------------

`timescale 1ns/1ns
module cpu_top
    import cpu_pkg::*;
(
    input  logic        clk,
    input  logic        rst_n,

    // ---- external bus ----
    output logic [19:0] addr,
    output logic [15:0] dout,
    input  logic [15:0] din,
    output logic        rd,
    output logic        wr,
    output logic        io_cycle,
    output logic        bhe,
    output logic        a0,
    output logic        ale,
    output logic [2:0]  s,
    input  logic        ready,

    // ---- interrupt pins ----
    input  logic        nmi,
    input  logic        int0,
    input  logic        int1,
    input  logic        int2,
    input  logic        int3,

    // ---- DMA request pins ----
    input  logic        drq0,
    input  logic        drq1,

    // direct vector injection (testbench / future external PIC)
    input  logic        intr_req,
    input  logic [7:0]  intr_type,
    output logic        intr_ack,

    // ---- status / debug ----
    output logic        halted,
    // An end-of-interrupt issued to the 8259 shim at port 20h, which lives
    // outside this module because a real 80186 board's PC-compatible glue
    // would have been external too.
    input  logic        ext_eoi,

    // A timer interrupt from the PC-compatible 8253. Once software has
    // programmed that chip it OWNS the tick, because it reprogrammed the rate
    // and is counting on getting it; until then the 80186's own timer drives
    // the interrupt exactly as before.
    input  logic        ext_tick,
    input  logic        ext_tick_en,

    output logic [15:0] dbg_ip,
    output logic [15:0] dbg_cs,
    output logic [15:0] dbg_flags,
    output logic [7:0]  dbg_int_type,
    output logic        dbg_int_taken
);

    logic [7:0]  fetch_data;
    logic        fetch_valid, fetch_set;
    logic [2:0]  fetch_pop_n;          // bytes the EU retires this cycle
    logic [7:0]  fetch_peek [0:5];     // queue contents, oldest first
    logic [3:0]  fetch_count;
    logic [19:0] fetch_addr;

    // EU request port
    logic        req, req_wr, req_io, req_word;
    logic [19:0] req_addr;
    logic [15:0] req_wdata, req_rdata;
    logic        req_done;

    // ---- PCB intercept ----
    logic        pcb_hit, pcb_done;
    logic [15:0] pcb_rdata;
    logic        biu_done;
    logic [15:0] biu_rdata;

    // (the PCB/DMA response muxing appears below, once the arbitration
    //  signals it depends on have been declared)

    // PCB sub-peripheral bus
    logic [7:1]  pcb_off;
    logic [15:0] pcb_wdata;
    logic        pcb_we, pcb_re;
    logic        ic_sel, timer_sel, dma_sel, cs_sel;
    logic [15:0] ic_rdata, timer_rdata, dma_rdata, cs_rdata;

    // ---- interrupt plumbing ----
    logic        ic_intr, core_intr, core_iack, ic_iack;
    logic [7:0]  ic_vector, core_vector;

    assign core_intr   = ic_intr | intr_req;
    assign core_vector = ic_intr ? ic_vector : intr_type;
    // Only tell the controller it was acknowledged if it was the source.
    assign ic_iack     = core_iack & ic_intr;
    assign intr_ack    = core_iack & ~ic_intr;

    logic timer0_irq, timer1_irq, timer2_irq, timer2_dma_req;
    logic dma0_irq, dma1_irq;

    // ---- bus arbitration between the EU and the DMA ----
    logic        bus_idle;
    logic        dma_bus_hold, dma_bus_req, dma_owns;
    logic [19:0] dma_addr;
    logic        dma_wr, dma_io, dma_word;
    logic [15:0] dma_wdata;
    logic        dma_active;

    // Ownership only changes at an idle boundary, and the DMA holds its
    // request across a whole fetch/deposit pair so the two cannot be split.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)              dma_owns <= 1'b0;
        else if (!dma_owns)      dma_owns <= dma_bus_hold && bus_idle;
        else if (!dma_bus_hold)  dma_owns <= 1'b0;
    end

    logic        biu_req, biu_req_wr, biu_req_io, biu_req_word;
    logic [19:0] biu_req_addr;
    logic [15:0] biu_req_wdata, biu_req_rdata;
    logic        biu_req_done;

    // A DMA request that has not been granted yet stops the BIU accepting new
    // CPU work, so the bus drains to idle and the handover can happen.
    // Reserve the bus for the whole DMA transfer, not just its individual
    // cycles. Releasing it during the gap between the fetch and the deposit
    // would let a prefetch slip in between them, which breaks the atomicity
    // the transfer is supposed to have.
    logic biu_hold_req;
    assign biu_hold_req = dma_bus_hold && !(dma_owns && dma_bus_req);

    // One dead cycle whenever ownership changes. The BIU latches "this request
    // has already been started" and only clears it when its request line goes
    // low; without a gap the incoming master's request looks like the outgoing
    // master's still-pending one, and the BIU refuses to start it -- which
    // deadlocks both of them.
    logic dma_owns_d, arb_gap;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) dma_owns_d <= 1'b0;
        else        dma_owns_d <= dma_owns;
    end
    assign arb_gap = (dma_owns != dma_owns_d);

    assign biu_req       = arb_gap ? 1'b0
                         : (dma_owns ? dma_bus_req : (req && !pcb_hit));
    assign biu_req_wr    = dma_owns ? dma_wr      : req_wr;
    assign biu_req_io    = dma_owns ? dma_io      : req_io;
    assign biu_req_word  = dma_owns ? dma_word    : req_word;
    assign biu_req_addr  = dma_owns ? dma_addr    : req_addr;
    assign biu_req_wdata = dma_owns ? dma_wdata   : req_wdata;

    // The EU sees no acknowledge while the DMA holds the bus, so it waits
    // rather than mistaking a DMA cycle's completion for its own.
    assign biu_rdata = biu_req_rdata;
    assign biu_done  = dma_owns ? 1'b0 : biu_req_done;
    assign req_rdata = pcb_hit ? pcb_rdata : biu_rdata;
    assign req_done  = pcb_hit ? pcb_done  : biu_done;

    biu u_biu (
        .clk         (clk),
        .rst_n       (rst_n),
        .addr        (addr),
        .dout        (dout),
        .din         (din),
        .rd          (rd),
        .wr          (wr),
        .io_cycle    (io_cycle),
        .bhe         (bhe),
        .a0          (a0),
        .ale         (ale),
        .s           (s),
        .ready       (ready),
        .fetch_addr  (fetch_addr),
        .fetch_set   (fetch_set),
        .fetch_data  (fetch_data),
        .fetch_valid (fetch_valid),
        .fetch_pop_n (fetch_pop_n),
        .fetch_peek  (fetch_peek),
        .fetch_count (fetch_count),
        .req         (biu_req),
        .req_wr      (biu_req_wr),
        .req_io      (biu_req_io),
        .req_word    (biu_req_word),
        .req_addr    (biu_req_addr),
        .req_wdata   (biu_req_wdata),
        .req_rdata   (biu_req_rdata),
        .req_done    (biu_req_done),
        .bus_idle    (bus_idle),
        .hold_req    (biu_hold_req)
    );

    pcb u_pcb (
        .clk         (clk),
        .rst_n       (rst_n),
        .req         (req),
        .req_wr      (req_wr),
        .req_io      (req_io),
        .req_addr    (req_addr),
        .req_wdata   (req_wdata),
        .req_rdata   (pcb_rdata),
        .req_done    (pcb_done),
        .hit         (pcb_hit),
        .pcb_off     (pcb_off),
        .pcb_wdata   (pcb_wdata),
        .pcb_we      (pcb_we),
        .pcb_re      (pcb_re),
        .ic_sel      (ic_sel),
        .timer_sel   (timer_sel),
        .dma_sel     (dma_sel),
        .cs_sel      (cs_sel),
        .ic_rdata    (ic_rdata),
        .timer_rdata (timer_rdata),
        .dma_rdata   (dma_rdata),
        .cs_rdata    (cs_rdata)
    );

    interrupt_controller u_pic (
        .clk         (clk),
        .rst_n       (rst_n),
        .ext_eoi     (ext_eoi),
        .sel         (ic_sel),
        .pcb_off     (pcb_off),
        .pcb_wdata   (pcb_wdata),
        .pcb_we      (pcb_we),
        .pcb_re      (pcb_re),
        .pcb_rdata   (ic_rdata),
        .int0        (int0),
        .int1        (int1),
        .int2        (int2),
        .int3        (int3),
        .timer0_irq  (ext_tick_en ? ext_tick : timer0_irq),
        .timer1_irq  (timer1_irq),
        .timer2_irq  (timer2_irq),
        .dma0_irq    (dma0_irq),
        .dma1_irq    (dma1_irq),
        .intr        (ic_intr),
        .vector_type (ic_vector),
        .iack        (ic_iack)
    );

    timer u_timer (
        .clk            (clk),
        .rst_n          (rst_n),
        .sel            (timer_sel),
        .pcb_off        (pcb_off),
        .pcb_wdata      (pcb_wdata),
        .pcb_we         (pcb_we),
        .pcb_re         (pcb_re),
        .pcb_rdata      (timer_rdata),
        .tmrin0         (1'b0),
        .tmrin1         (1'b0),
        .tmrout0        (),
        .tmrout1        (),
        .timer0_irq     (timer0_irq),
        .timer1_irq     (timer1_irq),
        .timer2_irq     (timer2_irq),
        .timer2_dma_req (timer2_dma_req)
    );

    dma u_dma (
        .clk            (clk),
        .rst_n          (rst_n),
        .sel            (dma_sel),
        .pcb_off        (pcb_off),
        .pcb_wdata      (pcb_wdata),
        .pcb_we         (pcb_we),
        .pcb_re         (pcb_re),
        .pcb_rdata      (dma_rdata),
        .drq0           (drq0),
        .drq1           (drq1),
        .timer2_dma_req (timer2_dma_req),
        .bus_hold       (dma_bus_hold),
        .bus_req        (dma_bus_req),
        .bus_gnt        (dma_owns),
        .bus_addr       (dma_addr),
        .bus_wr         (dma_wr),
        .bus_io         (dma_io),
        .bus_word       (dma_word),
        .bus_wdata      (dma_wdata),
        .bus_rdata      (biu_req_rdata),
        .bus_done       (biu_req_done),
        .dma_active     (dma_active),
        .dma0_irq       (dma0_irq),
        .dma1_irq       (dma1_irq)
    );

    chip_select u_cs (
        .clk           (clk),
        .rst_n         (rst_n),
        .sel           (cs_sel),
        .pcb_off       (pcb_off),
        .pcb_wdata     (pcb_wdata),
        .pcb_we        (pcb_we),
        .pcb_re        (pcb_re),
        .pcb_rdata     (cs_rdata),
        .addr          (addr),
        .io_space      (io_cycle),
        .ucs_n         (),
        .lcs_n         (),
        .mcs_n         (),
        .pcs_n         (),
        .wait_cnt      (),
        .use_ext_ready ()
    );

    eu u_eu (
        .clk           (clk),
        .rst_n         (rst_n),
        .fetch_data    (fetch_data),
        .fetch_valid   (fetch_valid),
        .fetch_pop_n   (fetch_pop_n),
        .fetch_peek    (fetch_peek),
        .fetch_count   (fetch_count),
        .fetch_set     (fetch_set),
        .fetch_addr    (fetch_addr),
        .req           (req),
        .req_wr        (req_wr),
        .req_io        (req_io),
        .req_word      (req_word),
        .req_addr      (req_addr),
        .req_wdata     (req_wdata),
        .req_rdata     (req_rdata),
        .req_done      (req_done),
        .nmi           (nmi),
        .intr_req      (core_intr),
        .intr_type     (core_vector),
        .intr_ack      (core_iack),
        .halted        (halted),
        .dbg_ip        (dbg_ip),
        .dbg_cs        (dbg_cs),
        .dbg_flags     (dbg_flags),
        .dbg_int_type  (dbg_int_type),
        .dbg_int_taken (dbg_int_taken)
    );

endmodule
