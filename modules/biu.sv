// ---------------------------------------------------------------------------
// biu — Bus Interface Unit: owns every bus cycle the CPU runs.
//
// Hierarchy: cpu_top -> biu -> prefetch_queue
// Reference: learnings/02-bus-interface.md
// Testbench: sim/tb_biu.sv
//
// THE INVARIANT THAT SHAPES THIS MODULE:
//   A bus cycle, once started, always runs to completion. Arbitration between
//   fetch and data cycles happens ONLY in the idle state, never mid-cycle.
//   Building it this way means an odd-address word access can never be split
//   by something else, and it is what HOLD/DMA support will hook into later.
//
// BUS CYCLE: minimum 4 T-states. T1 drives the address and pulses ALE, T2
// asserts RD or WR, T3 transfers data (repeating as Tw while READY is low),
// T4 ends the cycle. READY is sampled SRDY-style -- synchronously, at T3 --
// because this design's memory is on-chip and has no genuinely asynchronous
// ready source (see learnings/06-fpga-implementation-notes.md).
//
// ARBITRATION: data cycles beat prefetch, because a stalled instruction must
// not wait behind speculative fetch. The second half of a split word access
// outranks both, so the two halves stay adjacent.
//
// ODD-ADDRESS WORD ACCESS: the 80186 data bus is two byte lanes selected by
// BHE and A0. A word at an odd address straddles them, so it becomes two byte
// cycles -- upper lane at the odd address, then lower lane at the next. This
// is not an optimization to skip: unaligned word access is legal and common
// in x86 code.
//
// DMA shares this bus. cpu_top hands ownership over only while bus_idle is
// high, which is the same idle-state arbitration point above -- so a DMA
// transfer can never land in the middle of a CPU cycle.
//
// NOT IMPLEMENTED YET: HOLD/HLDA (no external bus master exists), LOCK, HALT
// cycles, queue-status mode. The idle-state arbitration point is where each of
// them belongs.
//   Interrupt acknowledge cycles are genuinely not needed: in master mode the
//   80186's internal controller supplies the vector directly and no INTA bus
//   cycle is ever run (learnings/05-interrupts-and-reset.md).
// ---------------------------------------------------------------------------

`timescale 1ns/1ns
module biu (
    input  logic        clk,
    input  logic        rst_n,

    // ---- external bus ----
    output logic [19:0] addr,
    output logic [15:0] dout,
    input  logic [15:0] din,
    output logic        rd,
    output logic        wr,
    output logic        io_cycle,     // 1 = I/O space, 0 = memory space
    output logic        bhe,          // active low: upper lane (D15-D8) enabled
    output logic        a0,           // low address bit / lower-lane select
    output logic        ale,
    output logic [2:0]  s,            // bus status, see learnings/02
    input  logic        ready,

    // ---- instruction fetch, to the EU ----
    input  logic [19:0] fetch_addr,   // restart address, loaded on fetch_set
    input  logic        fetch_set,    // redirect + flush the queue
    output logic [7:0]  fetch_data,
    output logic        fetch_valid,
    // The sequencer asks for a whole instruction's worth at a time now, and
    // looks ahead with `peek` to decide how much that is.
    input  logic [2:0]  fetch_pop_n,
    output logic [7:0]  fetch_peek [0:5],
    output logic [3:0]  fetch_count,

    // ---- data access, from the EU ----
    input  logic        req,          // hold high until req_done
    input  logic        req_wr,
    input  logic        req_io,
    input  logic        req_word,
    input  logic [19:0] req_addr,
    input  logic [15:0] req_wdata,
    output logic [15:0] req_rdata,
    output logic        req_done,     // one-cycle pulse

    // Arbitration point. High only between bus cycles, which is the one place
    // ownership of the bus may change hands -- granting anywhere else would
    // split a cycle in half.
    output logic        bus_idle,

    // Another master wants the bus. While this is high the BIU starts no new
    // cycles -- not data, and crucially not prefetch either -- so that it
    // drains to idle and the grant can actually land. Without it the BIU keeps
    // prefetching whenever the queue has room, idle never comes around, and
    // the waiting master is starved indefinitely. The second half of a split
    // access is exempt: that pair must complete.
    input  logic        hold_req
);

    localparam logic [2:0] ST_IDLE = 3'd0;
    localparam logic [2:0] ST_T1   = 3'd1;
    localparam logic [2:0] ST_T2   = 3'd2;
    localparam logic [2:0] ST_T3   = 3'd3;
    localparam logic [2:0] ST_T4   = 3'd4;

    // Bus status encodings (S2 S1 S0).
    localparam logic [2:0] S_IO_RD   = 3'b001;
    localparam logic [2:0] S_IO_WR   = 3'b010;
    localparam logic [2:0] S_FETCH   = 3'b100;
    localparam logic [2:0] S_MEM_RD  = 3'b101;
    localparam logic [2:0] S_MEM_WR  = 3'b110;
    localparam logic [2:0] S_PASSIVE = 3'b111;

    logic [2:0]  state;

    // Current bus cycle description.
    logic        cur_fetch;      // this cycle is an instruction fetch
    logic        cur_wr, cur_io;
    logic [19:0] cur_addr;
    logic        cur_bhe, cur_a0;
    logic [15:0] cur_dout;
    logic        cur_fetch_odd;  // fetch started at an odd address: keep high byte only

    logic [19:0] fetch_ptr;

    // Split (odd-address) word access bookkeeping.
    logic        split_second;   // the pending cycle is the second half
    logic        split_active;   // a second half is owed
    logic [7:0]  split_lo;       // first half of a split read

    logic        req_taken;      // current req already started; wait for req to drop
    logic [15:0] rdata_r;
    logic        done_r;

    // Latched copy of the request: the second half of a split access still
    // needs the ORIGINAL write data, and cur_dout gets overwritten per cycle.
    logic        xfer_word;
    logic [15:0] xfer_wdata;

    // Sticky "throw this fetch away". A redirect cannot abort a bus cycle
    // that has already started, and the data may not arrive until several
    // cycles after the redirect pulse, so remembering the intent is not
    // optional -- gating only on fetch_set itself lets stale bytes through.
    logic        fetch_discard;

    // Prefetch queue wiring.
    logic        pq_write_en, pq_write_word, pq_space;
    logic [15:0] pq_write_data;

    prefetch_queue u_pq (
        .clk             (clk),
        .rst_n           (rst_n),
        .write_en        (pq_write_en),
        .write_data      (pq_write_data),
        .write_word      (pq_write_word),
        .space_available (pq_space),
        .pop_n           (fetch_pop_n),
        .peek            (fetch_peek),
        .pop_data        (fetch_data),
        .pop_valid       (fetch_valid),
        .flush           (fetch_set),
        .count           (fetch_count)
    );

    // =====================================================================
    // Arbitration (idle state only)
    // =====================================================================
    logic start_split, start_data, start_fetch;

    assign start_split = split_active;
    assign start_data  = !split_active && req && !req_taken && !hold_req;
    assign start_fetch = !split_active && !(req && !req_taken) && pq_space &&
                         !fetch_set && !hold_req;

    // =====================================================================
    // Bus outputs
    // =====================================================================
    assign addr     = cur_addr;
    assign dout     = cur_dout;
    assign io_cycle = cur_io;
    assign bhe      = cur_bhe;
    assign a0       = cur_a0;

    // RD/WR assert at the start of T2 and drop at the start of T4.
    assign rd = ((state == ST_T2) || (state == ST_T3)) && !cur_wr;
    assign wr = ((state == ST_T2) || (state == ST_T3)) &&  cur_wr;

    assign ale = (state == ST_T1);

    assign bus_idle = (state == ST_IDLE) && !split_active;

    always_comb begin
        if (state == ST_IDLE) s = S_PASSIVE;
        else if (cur_fetch)   s = S_FETCH;
        else if (cur_io)      s = cur_wr ? S_IO_WR  : S_IO_RD;
        else                  s = cur_wr ? S_MEM_WR : S_MEM_RD;
    end

    // Queue fill happens on the same edge that ends T3. `fetch_set` gates it:
    // a flush during an in-flight fetch cannot abort the bus cycle, so the
    // data is discarded on arrival instead.
    assign pq_write_en   = (state == ST_T3) && ready && cur_fetch
                           && !fetch_set && !fetch_discard;
    assign pq_write_word = !cur_fetch_odd;
    assign pq_write_data = cur_fetch_odd ? {8'h00, din[15:8]} : din;

    assign req_rdata = rdata_r;
    assign req_done  = done_r;

    // =====================================================================
    // Sequencer
    // =====================================================================
    logic [15:0] assembled;

    always_comb begin
        // Byte reads take whichever lane the address selects.
        if (split_second)      assembled = {din[7:0], split_lo};
        else if (!xfer_word)   assembled = cur_a0 ? {8'h00, din[15:8]} : {8'h00, din[7:0]};
        else                   assembled = din;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= ST_IDLE;
            cur_fetch     <= 1'b0;
            cur_wr        <= 1'b0;
            cur_io        <= 1'b0;
            cur_addr      <= 20'h00000;
            cur_bhe       <= 1'b1;
            cur_a0        <= 1'b0;
            cur_dout      <= 16'h0000;
            cur_fetch_odd <= 1'b0;
            fetch_ptr     <= 20'hFFFF0;
            split_second  <= 1'b0;
            split_active  <= 1'b0;
            split_lo      <= 8'h00;
            req_taken     <= 1'b0;
            rdata_r       <= 16'h0000;
            done_r        <= 1'b0;
            xfer_word     <= 1'b0;
            xfer_wdata    <= 16'h0000;
            fetch_discard <= 1'b0;
        end else begin
            done_r <= 1'b0;

            if (!req) req_taken <= 1'b0;

            // A redirect reloads the fetch pointer. The queue flush itself is
            // wired straight to prefetch_queue.
            if (fetch_set) fetch_ptr <= fetch_addr;

            // Mark an in-flight fetch for discard. By T4 the data has already
            // been pushed, but the queue flush clears it, so only T1..T3 need
            // covering here.
            if (fetch_set && cur_fetch && (state != ST_IDLE) && (state != ST_T4))
                fetch_discard <= 1'b1;
            else if (state == ST_T4)
                fetch_discard <= 1'b0;

            case (state)
                ST_IDLE: begin
                    if (start_split) begin
                        // Second half of an odd word: lower lane at addr+1.
                        cur_fetch     <= 1'b0;
                        cur_fetch_odd <= 1'b0;
                        cur_addr      <= cur_addr + 20'd1;
                        cur_bhe       <= 1'b1;
                        cur_a0        <= 1'b0;
                        cur_dout      <= {8'h00, xfer_wdata[15:8]};
                        split_second  <= 1'b1;
                        split_active  <= 1'b0;
                        state         <= ST_T1;
                    end else if (start_data) begin
                        cur_fetch     <= 1'b0;
                        cur_fetch_odd <= 1'b0;
                        cur_wr        <= req_wr;
                        cur_io        <= req_io;
                        cur_addr      <= req_addr;
                        req_taken     <= 1'b1;
                        split_second  <= 1'b0;
                        xfer_word     <= req_word;
                        xfer_wdata    <= req_wdata;
                        if (req_word && req_addr[0]) begin
                            // Straddles both lanes: the word's LOW byte lives
                            // at the odd address, so it goes out on D15-D8.
                            cur_bhe      <= 1'b0;
                            cur_a0       <= 1'b1;
                            cur_dout     <= {req_wdata[7:0], 8'h00};
                            split_active <= 1'b1;
                        end else if (req_word) begin
                            cur_bhe      <= 1'b0;
                            cur_a0       <= 1'b0;
                            cur_dout     <= req_wdata;
                            split_active <= 1'b0;
                        end else begin
                            cur_bhe      <= req_addr[0] ? 1'b0 : 1'b1;
                            cur_a0       <= req_addr[0];
                            cur_dout     <= req_addr[0] ? {req_wdata[7:0], 8'h00}
                                                        : {8'h00, req_wdata[7:0]};
                            split_active <= 1'b0;
                        end
                        state <= ST_T1;
                    end else if (start_fetch) begin
                        cur_fetch     <= 1'b1;
                        cur_wr        <= 1'b0;
                        cur_io        <= 1'b0;
                        // Always fetch the aligned word containing fetch_ptr.
                        cur_addr      <= {fetch_ptr[19:1], 1'b0};
                        cur_bhe       <= 1'b0;
                        cur_a0        <= 1'b0;
                        cur_fetch_odd <= fetch_ptr[0];
                        split_second  <= 1'b0;
                        state         <= ST_T1;
                    end
                end

                ST_T1: state <= ST_T2;

                ST_T2: state <= ST_T3;

                // Stays here as Tw while READY is low.
                ST_T3: begin
                    if (ready) begin
                        if (cur_fetch) begin
                            // Queue push is combinational on this same edge.
                            // Must not advance a pointer that a redirect has
                            // just reloaded, or the new stream starts skewed.
                            if (!fetch_set && !fetch_discard)
                                fetch_ptr <= cur_fetch_odd ? (fetch_ptr + 20'd1)
                                                           : (fetch_ptr + 20'd2);
                        end else if (!cur_wr) begin
                            if (split_active) split_lo <= din[15:8];
                            else              rdata_r  <= assembled;
                        end
                        state <= ST_T4;
                    end
                end

                ST_T4: begin
                    // A data transfer is complete unless a second half is owed.
                    if (!cur_fetch && !split_active) done_r <= 1'b1;
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
