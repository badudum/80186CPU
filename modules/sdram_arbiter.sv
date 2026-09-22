// ---------------------------------------------------------------------------
// sdram_arbiter — shares one SDRAM controller between several requesters.
//
// Hierarchy: memory_controller -> sdram_arbiter -> sdram_controller
// Testbench: sim/tb_sdram_arb.sv
//
// The SDRAM controller has exactly one request port and no notion of a queue.
// That was fine while the CPU was the only thing talking to memory. It stops
// being fine as soon as the disk image lives in SDRAM too, because then the
// block device and the CPU want the same port at the same time -- and later so
// does the JTAG loader that fills the disk in the first place.
//
// FIXED PRIORITY WOULD STARVE SOMETHING. The CPU is not a polite requester: it
// fetches instructions continuously, and the code it runs while waiting for the
// disk is a polling loop that fetches from the very memory the disk is trying
// to use. So priority ROTATES: after a requester is served it drops to the back
// of the queue. Nobody waits longer than one turn per other requester.
//
// ZERO ADDED LATENCY. The grant is combinational when the arbiter is idle, so a
// CPU access with no competition costs exactly what it did before this module
// existed. Only the cycle-by-cycle winner is registered, to hold the selection
// steady for the length of a transfer.
//
// ONE ACCESS PER REQUEST. This is the subtle part. The controller starts a new
// access whenever rd or wr is high and it is back in S_IDLE -- it does not
// require the request to fall in between. A requester that holds its line up
// for a cycle too long would therefore get a second, unasked-for access, which
// for a write means writing twice and for a read means a stray bus cycle. So a
// requester is marked `served` when its ready fires and becomes eligible again
// only once it has actually dropped its request.
//
// THE CONTRACT for a requester:
//   - hold addr/wdata/be/rd/wr steady until you see your own `ready`
//   - `ready` is a single-cycle pulse, and `rdata` is valid in that cycle and
//     held afterwards until the next read completes
//   - drop rd/wr after ready, before asking for anything else
// ---------------------------------------------------------------------------

`timescale 1ns/1ns
module sdram_arbiter #(
    parameter int NREQ = 3,
    parameter int AW   = 24
) (
    input  logic clk,
    input  logic rst_n,

    // ---- requester side ----
    input  logic [NREQ-1:0] req_rd,
    input  logic [NREQ-1:0] req_wr,
    input  logic [AW-1:0]   req_addr  [NREQ],
    input  logic [15:0]     req_wdata [NREQ],
    input  logic [1:0]      req_be    [NREQ],
    output logic [NREQ-1:0] req_ready,
    output logic [15:0]     req_rdata,          // broadcast; only the winner cares

    // ---- controller side ----
    output logic [AW-1:0]   mem_addr,
    output logic [15:0]     mem_wdata,
    output logic [1:0]      mem_be,
    output logic            mem_rd,
    output logic            mem_wr,
    input  logic [15:0]     mem_rdata,
    input  logic            mem_ready
);

    localparam int IDW = (NREQ > 1) ? $clog2(NREQ) : 1;

    logic [NREQ-1:0]  asking;      // wants the bus
    logic [NREQ-1:0]  served;      // already got its ready, waiting to let go
    logic [NREQ-1:0]  eligible;

    assign asking   = req_rd | req_wr;
    assign eligible = asking & ~served;

    // ---- rotating priority ----
    // `rotate` is whoever gets first refusal this round. Scanning from there
    // and taking the first eligible requester gives round-robin fairness
    // without a counter per port.
    logic [IDW-1:0] rotate;
    logic [IDW-1:0] winner;
    logic           found;

    always_comb begin
        int unsigned s, idx;
        winner = '0;
        found  = 1'b0;
        for (int k = 0; k < NREQ; k++) begin
            // (rotate + k) mod NREQ, without a variable modulo
            s   = rotate + k;
            idx = (s >= NREQ) ? (s - NREQ) : s;
            if (!found && eligible[idx]) begin
                winner = idx[IDW-1:0];
                found  = 1'b1;
            end
        end
    end

    // ---- the grant ----
    logic           active;
    logic [IDW-1:0] cur;

    // While idle the winner drives the controller directly, so an uncontended
    // access is not delayed by a cycle of arbitration.
    logic [IDW-1:0] sel;
    logic           sel_valid;
    assign sel       = active ? cur : winner;
    assign sel_valid = active ? 1'b1 : found;

    assign mem_addr  = req_addr[sel];
    assign mem_wdata = req_wdata[sel];
    assign mem_be    = req_be[sel];
    assign mem_rd    = sel_valid && req_rd[sel] && !served[sel];
    assign mem_wr    = sel_valid && req_wr[sel] && !served[sel];

    assign req_rdata = mem_rdata;

    // The completion goes only to whoever asked for it.
    always_comb begin
        req_ready = '0;
        if (sel_valid && mem_ready) req_ready[sel] = 1'b1;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active <= 1'b0;
            cur    <= '0;
            rotate <= '0;
            served <= '0;
        end else begin
            // A requester becomes eligible again once it lets go.
            for (int i = 0; i < NREQ; i++)
                if (!asking[i]) served[i] <= 1'b0;

            if (!active) begin
                if (found) begin
                    active <= 1'b1;
                    cur    <= winner;
                    // A single-cycle access can complete in the same cycle it
                    // was granted, so the finish is handled here too.
                    if (mem_ready) begin
                        active      <= 1'b0;
                        served[winner] <= 1'b1;
                        rotate      <= (winner == NREQ - 1) ? '0
                                                            : (winner + 1'b1);
                    end
                end
            end else if (mem_ready) begin
                active      <= 1'b0;
                served[cur] <= 1'b1;
                // Whoever just went drops to the back of the queue.
                rotate      <= (cur == NREQ - 1) ? '0 : (cur + 1'b1);
            end
        end
    end

endmodule
