// ---------------------------------------------------------------------------
// bpred — branch history table with 2-bit saturating counters, plus the
// target each branch went to.
//
// Hierarchy: cpu_top -> eu -> execUnit -> bpred
// Testbench: sim/tb_bpred.sv
//
// WHY THIS IS WORTH BUILDING, measured rather than assumed:
//
//     FETCH_OP: 77,317 cycles, 37,528 with an empty queue (48.5%),
//               36,842 of those just after a flush
//
// Nearly half of all the cycles the sequencer spends waiting for an opcode
// are spent with NOTHING in the queue, and 98% of those follow a branch
// throwing the queue away. That is about 6% of every cycle the machine runs,
// waiting for the first byte at a branch target to come back from memory.
//
// An earlier estimate in this project put a predictor at ~2%. That was
// reasoned from a model -- "a taken branch costs a queue refill against a
// high CPI" -- rather than from the measurement above, and it was several
// times too low. The cost is not a refill, it is a full memory round trip
// with the execution unit stalled.
//
// LOOKED UP BY INSTRUCTION ADDRESS, NOT FETCH ADDRESS. On x86 you cannot tell
// at fetch time which byte begins an instruction without decoding from a
// known boundary, which is why real designs carry predecode bits in the
// instruction cache. This design does not need any of that: the sequencer
// knows instr_start_ip exactly when it starts an instruction, and that is
// early enough to redirect the fetch several cycles before the branch
// resolves.
//
// WHY 2-BIT COUNTERS. A single bit mispredicts twice per loop: once on the
// exit, and again on the next entry, because the exit flipped it. Two bits
// with saturation need two disagreements to change the prediction, so a loop
// that runs many times and exits once costs one mispredict instead of two.
// That is the whole reason the standard is two bits and not one.
//
// AN ENTRY IS ALLOCATED ONLY ON A TAKEN BRANCH. A not-taken branch needs no
// entry: the default with no entry is "not taken", which is already right,
// and allocating for it would evict a branch that does need one.
//
// A WRONG PREDICTION IS ONLY A PERFORMANCE COST. The execution unit resolves
// every branch itself and redirects if the queue holds the wrong path, which
// is exactly what it does today for every taken branch. So a mispredict costs
// what every taken branch costs now, and correctness never depends on the
// prediction being right.
// ---------------------------------------------------------------------------

`timescale 1ns/1ns
module bpred #(
    // 256 entries is 1 M10K and covers far more branches than a DOS-era
    // program has hot ones.
    parameter int ENTRIES = 256
) (
    input  logic        clk,
    input  logic        rst_n,

    // ---- lookup, when an instruction starts ----
    input  logic [15:0] q_ip,
    output logic        p_valid,      // this address is a known branch
    output logic        p_taken,      // ...and the counter says take it
    output logic [15:0] p_target,

    // ---- update, when a branch resolves ----
    input  logic        u_valid,      // a conditional branch just resolved
    input  logic [15:0] u_ip,
    input  logic        u_taken,
    input  logic [15:0] u_target
);

    localparam int IDX_W = $clog2(ENTRIES);
    localparam int TAG_W = 16 - IDX_W;

    // A TAG IS NOT OPTIONAL HERE, even though a mispredict is harmless. Two
    // branches sharing an index would otherwise hand each other's TARGET
    // across, turning every alternation into a guaranteed mispredict -- worse
    // than not predicting at all.
    logic                  ent_valid [0:ENTRIES-1];
    logic [TAG_W-1:0]      ent_tag   [0:ENTRIES-1];
    logic [1:0]            ent_ctr   [0:ENTRIES-1];
    logic [15:0]           ent_tgt   [0:ENTRIES-1];

    logic [IDX_W-1:0] q_idx, u_idx;
    logic [TAG_W-1:0] q_tag, u_tag;

    assign q_idx = q_ip[IDX_W-1:0];
    assign q_tag = q_ip[15:IDX_W];
    assign u_idx = u_ip[IDX_W-1:0];
    assign u_tag = u_ip[15:IDX_W];

    // Combinational lookup. The sequencer needs the answer in the same cycle
    // it starts the instruction, because that is the cycle in which it can
    // still redirect the fetch.
    always_comb begin
        p_valid  = ent_valid[q_idx] && (ent_tag[q_idx] == q_tag);
        // Bit 1 of the counter IS the prediction: 2 and 3 are taken, 0 and 1
        // are not.
        p_taken  = p_valid && ent_ctr[q_idx][1];
        p_target = ent_tgt[q_idx];
    end

    logic u_hit;
    assign u_hit = ent_valid[u_idx] && (ent_tag[u_idx] == u_tag);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < ENTRIES; i++) begin
                ent_valid[i] <= 1'b0;
                ent_tag[i]   <= '0;
                ent_ctr[i]   <= 2'b01;      // weakly not taken
                ent_tgt[i]   <= 16'h0000;
            end
        end else if (u_valid) begin
            if (u_hit) begin
                // Saturate rather than wrap. Wrapping would turn a strongly
                // taken branch into a strongly not-taken one on a single
                // disagreement, which is the opposite of the point.
                if (u_taken) begin
                    if (ent_ctr[u_idx] != 2'b11) ent_ctr[u_idx] <= ent_ctr[u_idx] + 2'd1;
                    ent_tgt[u_idx] <= u_target;
                end else begin
                    if (ent_ctr[u_idx] != 2'b00) ent_ctr[u_idx] <= ent_ctr[u_idx] - 2'd1;
                end
            end else if (u_taken) begin
                // Allocate, replacing whatever shared the index. Weakly
                // taken, not strongly: one observation is not enough to
                // justify two mispredicts before it can change its mind.
                ent_valid[u_idx] <= 1'b1;
                ent_tag[u_idx]   <= u_tag;
                ent_ctr[u_idx]   <= 2'b10;
                ent_tgt[u_idx]   <= u_target;
            end
        end
    end

endmodule
