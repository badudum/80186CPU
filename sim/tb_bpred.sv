`timescale 1ns/1ns
//
// The branch history table, exercised against the behaviours that actually
// decide whether a predictor is worth having.
//
// The important cases are not "does it remember a target" but the ones where
// a naive predictor goes wrong: a loop that exits and is re-entered, two
// branches sharing an index, and a counter that must saturate rather than
// wrap. Each is checked explicitly below.
//
module tb_bpred;

    localparam int ENTRIES = 256;

    logic        clk = 0, rst_n = 0;
    logic [15:0] q_ip = 16'h0000;
    logic        p_valid, p_taken;
    logic [15:0] p_target;
    logic        u_valid = 0;
    logic [15:0] u_ip = 16'h0000;
    logic        u_taken = 0;
    logic [15:0] u_target = 16'h0000;

    bpred #(.ENTRIES(ENTRIES)) dut (.*);

    always #5 clk = ~clk;

    int errors = 0, checks = 0;
    task chk(input string nm, input int got, input int exp);
        checks++;
        if (got !== exp) begin
            $display("FAIL %-52s got=%0d exp=%0d", nm, got, exp);
            errors++;
        end
    endtask

    // Resolve a branch: tell the table what actually happened.
    task automatic resolve(input [15:0] ip, input logic taken, input [15:0] tgt);
        begin
            @(negedge clk);
            u_valid = 1; u_ip = ip; u_taken = taken; u_target = tgt;
            @(negedge clk);
            u_valid = 0;
        end
    endtask

    // Ask what it predicts for an address.
    task automatic ask(input [15:0] ip);
        begin
            q_ip = ip;
            #1;
        end
    endtask

    initial begin
        repeat (3) @(negedge clk);
        rst_n = 1;
        @(negedge clk);

        // ---- an address nobody has seen is not a branch ----
        ask(16'h1000);
        chk("an unseen address is not predicted", p_valid, 0);
        chk("...and is predicted not taken",      p_taken, 0);

        // ---- one taken branch allocates ----
        resolve(16'h1000, 1'b1, 16'h2000);
        ask(16'h1000);
        chk("a taken branch is now known",   p_valid, 1);
        chk("...and predicted taken",        p_taken, 1);
        chk("...to the right target",        p_target, 16'h2000);

        // ---- a not-taken branch does NOT allocate ----
        // The default with no entry is "not taken", which is already right,
        // and allocating would evict a branch that needs the slot.
        ask(16'h1004);
        chk("a fresh address starts unknown", p_valid, 0);
        resolve(16'h1004, 1'b0, 16'h0000);
        ask(16'h1004);
        chk("a not-taken branch allocated nothing", p_valid, 0);

        // ---- the counter saturates upward ----
        resolve(16'h1000, 1'b1, 16'h2000);
        resolve(16'h1000, 1'b1, 16'h2000);
        resolve(16'h1000, 1'b1, 16'h2000);
        ask(16'h1000);
        chk("still taken after many agreements", p_taken, 1);

        // ---- THE REASON FOR TWO BITS ----
        // A strongly taken branch that is not taken ONCE -- a loop exiting --
        // must still be predicted taken next time. A one-bit predictor flips
        // here and then mispredicts the next entry to the loop as well, so a
        // loop costs two mispredicts per execution instead of one.
        resolve(16'h1000, 1'b0, 16'h0000);
        ask(16'h1000);
        chk("one disagreement does NOT flip the prediction", p_taken, 1);

        // Two in a row does flip it.
        resolve(16'h1000, 1'b0, 16'h0000);
        ask(16'h1000);
        chk("two disagreements flip it", p_taken, 0);

        // ...and it saturates downward rather than wrapping. Wrapping would
        // turn strongly-not-taken into strongly-taken on one disagreement.
        resolve(16'h1000, 1'b0, 16'h0000);
        resolve(16'h1000, 1'b0, 16'h0000);
        resolve(16'h1000, 1'b0, 16'h0000);
        ask(16'h1000);
        chk("counter saturated down, did not wrap", p_taken, 0);

        // One taken brings it back to weakly taken but not yet predicting.
        resolve(16'h1000, 1'b1, 16'h2000);
        ask(16'h1000);
        chk("one agreement is not yet enough to predict taken", p_taken, 0);
        resolve(16'h1000, 1'b1, 16'h2000);
        ask(16'h1000);
        chk("two agreements predict taken again", p_taken, 1);

        // ---- the target follows the branch ----
        // A branch whose target changes -- an indirect jump, or a relative
        // branch reached from a different segment -- must not keep handing
        // out the old one.
        resolve(16'h1000, 1'b1, 16'h3333);
        ask(16'h1000);
        chk("the target is updated on a taken resolve", p_target, 16'h3333);

        // ---- two branches sharing an index ----
        // ENTRIES apart, so they collide. Without a tag the second would
        // inherit the first's TARGET, turning every alternation into a
        // guaranteed mispredict -- worse than not predicting at all.
        resolve(16'h1000, 1'b1, 16'h3333);
        resolve(16'h1000 + ENTRIES, 1'b1, 16'h4444);
        ask(16'h1000 + ENTRIES);
        chk("the aliasing branch has its own target", p_target, 16'h4444);
        ask(16'h1000);
        chk("the displaced branch is no longer claimed", p_valid, 0);
        chk("...so it is predicted not taken rather than to a wrong target",
            p_taken, 0);

        // ---- a fresh allocation starts weakly taken ----
        // Strongly taken would take two disagreements to unlearn a branch
        // seen exactly once.
        resolve(16'h5000, 1'b1, 16'h6000);
        ask(16'h5000);
        chk("a new branch is predicted taken", p_taken, 1);
        resolve(16'h5000, 1'b0, 16'h0000);
        ask(16'h5000);
        chk("...and one disagreement is enough to stop", p_taken, 0);

        $display("");
        $display("==================================");
        $display(" checks: %0d   failures: %0d", checks, errors);
        $display("==================================");
        if (errors == 0) $display("ALL TESTS PASSED");
        $finish;
    end

    initial begin
        #200000;
        $display("FAIL global timeout");
        $display(" checks: %0d   failures: %0d", checks, errors + 1);
        $finish;
    end

endmodule
