// ---------------------------------------------------------------------------
// prefetch_queue — 6-byte instruction prefetch queue (the BIU/EU decoupler).
//
// Hierarchy: cpu_top -> biu -> prefetch_queue
// Reference: learnings/00-overview.md (BIU/EU decoupling)
// Testbench: sim/tb_prefetch_queue.sv
//
// The 80186 queue is 6 bytes deep (the 80188 has 4). It exists so the BIU can
// run ahead fetching while the EU is busy, which is why a 67-clock IDIV costs
// almost no fetch bandwidth.
//
// The BIU fills a WORD at a time (one bus cycle returns two bytes) but the EU
// pops a BYTE at a time, so the two sides move at different granularity. Fill
// and pop may both happen in the same cycle.
//
// `write_word` = 0 pushes only the low byte. That happens after a control
// transfer to an ODD address: the first bus cycle of the new instruction
// stream returns a word whose low half belongs to the previous (odd) address,
// so only the high byte is wanted -- the BIU passes that byte in the low
// position with write_word = 0.
//
// FLUSH must be honored on every taken jump, call, return, interrupt entry and
// IRET. A stale queue after a control transfer executes speculatively fetched
// bytes, which is the most destructive bug this module can have. Note the BIU
// cannot abort a bus cycle already in flight (bus cycles are atomic) -- it
// must instead discard the data when it lands, which it does by simply not
// asserting write_en for it.
// ---------------------------------------------------------------------------

`timescale 1ns/1ns
module prefetch_queue (
    input  logic        clk,
    input  logic        rst_n,

    // Fill side (from biu)
    input  logic        write_en,
    input  logic [15:0] write_data,
    input  logic        write_word,      // 1 = push both bytes, 0 = low byte only
    output logic        space_available, // room for a full word

    // Pop side (to eu)
    // CONSUMPTION IS BY COUNT, NOT BY PULSE. The EU used to take one byte
    // per cycle, which meant an instruction already sitting whole in the
    // queue still cost a cycle per byte to read in -- opcode, ModR/M,
    // displacement, immediate, one state each. Measured on the MS-DOS boot
    // that was 3.5 million cycles, 17.7% of all of them, spent fetching
    // bytes that had already arrived. `peek` exposes the queue contents so a
    // whole instruction can be examined at once, and `pop_n` retires all of
    // it in a single cycle.
    input  logic [2:0]  pop_n,          // bytes to consume this cycle, 0..DEPTH
    output logic [7:0]  peek [0:5],     // peek[i] = byte i ahead of the head
    output logic [7:0]  pop_data,       // == peek[0], kept for one-byte users
    output logic        pop_valid,

    // Control
    input  logic        flush,
    output logic [3:0]  count
);

    localparam int DEPTH = 6;

    logic [7:0] q [0:DEPTH-1];
    logic [2:0] head, tail;
    logic [3:0] cnt;

    logic [2:0] do_pop;
    logic [1:0] push_n;

    // Clamped, so asking for more than is present consumes only what is
    // there. A decoder that mispredicts an instruction's length must not be
    // able to run the head past the tail.
    assign do_pop = ({1'b0, pop_n} > cnt) ? cnt[2:0] : pop_n;
    always_comb begin
        if (!write_en)      push_n = 2'd0;
        else if (write_word) push_n = 2'd2;
        else                 push_n = 2'd1;
    end

    assign pop_valid = (cnt != 4'd0);
    assign pop_data  = peek[0];
    assign count     = cnt;

    // Byte i ahead of the head, wrapped. Bytes past `cnt` are not valid and
    // the reader is expected to check `count` before believing them.
    logic [3:0] peek_idx [0:5];
    always_comb begin
        for (int i = 0; i < DEPTH; i++) begin
            peek_idx[i] = {1'b0, head} + i[3:0];
            if (peek_idx[i] >= DEPTH) peek_idx[i] = peek_idx[i] - DEPTH[3:0];
            peek[i] = q[peek_idx[i][2:0]];
        end
    end

    // Conservative by one: the BIU only starts a fetch cycle when a whole word
    // will fit on arrival, and a bus cycle takes at least 4 clocks, so leaving
    // a little headroom costs nothing and avoids an overflow corner case.
    assign space_available = (cnt <= 4'd4);

    // Wrapped pointer arithmetic, precomputed into signals rather than done
    // with a function called in index position. `q[wrap(tail,1)] <= ...` is
    // legal SystemVerilog but crashes ModelSim 10.5b with an internal
    // compiler error, so the indices are plain signals here.
    logic [3:0] tail_sum;
    logic [2:0] tail_p1, tail_next, head_next;

    assign tail_sum  = {1'b0, tail} + {2'b0, push_n};
    assign tail_next = (tail_sum >= DEPTH) ? (tail_sum[2:0] - DEPTH[2:0]) : tail_sum[2:0];
    assign tail_p1   = (tail == DEPTH-1) ? 3'd0 : (tail + 3'd1);
    // The head can now move by more than one, so it wraps by subtraction
    // rather than by a compare against DEPTH-1.
    logic [3:0] head_sum;
    assign head_sum  = {1'b0, head} + {1'b0, do_pop};
    assign head_next = (head_sum >= DEPTH) ? (head_sum[2:0] - DEPTH[2:0])
                                           : head_sum[2:0];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            head <= 3'd0;
            tail <= 3'd0;
            cnt  <= 4'd0;
        end else if (flush) begin
            // Flush wins over any concurrent fill or pop.
            head <= 3'd0;
            tail <= 3'd0;
            cnt  <= 4'd0;
        end else begin
            if (push_n != 2'd0) begin
                q[tail] <= write_data[7:0];
                if (push_n == 2'd2) q[tail_p1] <= write_data[15:8];
                tail <= tail_next;
            end

            if (do_pop != 3'd0) head <= head_next;

            cnt <= cnt + {2'b0, push_n} - {1'b0, do_pop};
        end
    end

endmodule
