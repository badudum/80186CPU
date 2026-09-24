`timescale 1ns/1ns
module tb_prefetch_queue;

    logic        clk = 0, rst_n = 0;
    logic        write_en = 0, write_word = 1, pop = 0, flush = 0;
    // The queue consumes by count now. `pop` stays as this testbench's
    // one-byte shorthand so the existing cases read the same; the multi-byte
    // path gets its own case at the end.
    logic [2:0]  pop_n;
    logic [7:0]  peek [0:5];
    assign pop_n = pop ? 3'd1 : 3'd0;
    logic [15:0] write_data = 0;
    logic        space_available, pop_valid;
    logic [7:0]  pop_data;
    logic [3:0]  count;

    prefetch_queue dut (.*);

    always #5 clk = ~clk;

    int errors = 0, checks = 0;

    task chk(input string nm, input int got, input int exp);
        checks++;
        if (got !== exp) begin
            $display("FAIL %-34s got=%0h exp=%0h", nm, got, exp);
            errors++;
        end
    endtask

    task push(input [15:0] d, input w);
        begin
            @(negedge clk);
            write_data = d; write_word = w; write_en = 1;
            @(negedge clk);
            write_en = 0;
        end
    endtask

    // Pop one byte and return it (sampled before the clock edge that removes it)
    task popb(output [7:0] b);
        begin
            @(negedge clk);
            b = pop_data;
            pop = 1;
            @(negedge clk);
            pop = 0;
        end
    endtask

    logic [7:0] got;

    initial begin
        repeat (3) @(negedge clk);
        chk("reset count", count, 0);
        chk("reset pop_valid", pop_valid, 0);
        chk("reset space", space_available, 1);
        rst_n = 1;
        @(negedge clk);

        // ---- word push, byte pops, little-endian order ----
        push(16'hBBAA, 1);
        chk("count after word push", count, 2);
        chk("pop_valid", pop_valid, 1);

        popb(got); chk("first byte is low half",  got, 8'hAA);
        popb(got); chk("second byte is high half", got, 8'hBB);
        chk("empty again", count, 0);
        chk("pop_valid low when empty", pop_valid, 0);

        // ---- byte-only push (jump to odd address) ----
        push(16'h0055, 0);
        chk("count after byte push", count, 1);
        popb(got); chk("byte-only value", got, 8'h55);
        chk("empty", count, 0);

        // ---- fill to capacity, verify wraparound ordering ----
        push(16'h1100, 1);
        push(16'h3322, 1);
        push(16'h5544, 1);
        chk("full count", count, 6);
        chk("no space when full", space_available, 0);

        popb(got); chk("fifo[0]", got, 8'h00);
        popb(got); chk("fifo[1]", got, 8'h11);
        popb(got); chk("fifo[2]", got, 8'h22);
        chk("count after 3 pops", count, 3);
        chk("space available again", space_available, 1);

        // push across the wrap point while partially drained
        push(16'h7766, 1);
        chk("count after wrap push", count, 5);
        popb(got); chk("fifo[3]", got, 8'h33);
        popb(got); chk("fifo[4]", got, 8'h44);
        popb(got); chk("fifo[5]", got, 8'h55);
        popb(got); chk("wrapped byte low",  got, 8'h66);
        popb(got); chk("wrapped byte high", got, 8'h77);
        chk("drained", count, 0);

        // ---- simultaneous push and pop ----
        push(16'hDDCC, 1);
        @(negedge clk);
        write_data = 16'hFFEE; write_word = 1; write_en = 1;
        pop = 1;                       // pop CC in the same cycle
        @(negedge clk);
        write_en = 0; pop = 0;
        #1;
        chk("count after simultaneous push/pop", count, 3);  // 2 - 1 + 2
        popb(got); chk("remaining DD", got, 8'hDD);
        popb(got); chk("then EE", got, 8'hEE);
        popb(got); chk("then FF", got, 8'hFF);
        chk("empty after drain", count, 0);

        // ---- flush ----
        push(16'h2211, 1);
        push(16'h4433, 1);
        chk("count before flush", count, 4);
        @(negedge clk);
        flush = 1;
        @(negedge clk);
        flush = 0;
        #1;
        chk("count after flush", count, 0);
        chk("pop_valid after flush", pop_valid, 0);

        // queue is usable again and starts clean after a flush
        push(16'h9988, 1);
        popb(got); chk("post-flush first byte", got, 8'h88);
        popb(got); chk("post-flush second byte", got, 8'h99);

        // ---- flush must win over a concurrent push ----
        @(negedge clk);
        write_data = 16'hAAAA; write_word = 1; write_en = 1;
        flush = 1;
        @(negedge clk);
        write_en = 0; flush = 0;
        #1;
        chk("flush beats concurrent push", count, 0);

        // ---- popping an empty queue must not underflow ----
        @(negedge clk);
        pop = 1;
        @(negedge clk);
        pop = 0;
        #1;
        chk("no underflow on empty pop", count, 0);

        // ---- reading a whole instruction at once ----
        // The reason this interface exists: an instruction already sitting
        // in the queue used to cost a cycle per byte to read in. `peek` has
        // to show the bytes in order from the head, and `pop_n` has to
        // retire all of them in one cycle.
        flush = 1; @(negedge clk); flush = 0; @(negedge clk);
        push(16'hBBAA, 1'b1);            // AA BB
        push(16'hDDCC, 1'b1);            // CC DD
        push(16'hFFEE, 1'b1);            // EE FF
        #1;
        chk("six bytes queued", count, 6);
        chk("peek[0]", peek[0], 8'hAA);
        chk("peek[1]", peek[1], 8'hBB);
        chk("peek[2]", peek[2], 8'hCC);
        chk("peek[3]", peek[3], 8'hDD);
        chk("peek[4]", peek[4], 8'hEE);
        chk("peek[5]", peek[5], 8'hFF);
        chk("pop_data still tracks peek[0]", pop_data, 8'hAA);

        // Retire four bytes in ONE cycle, as a four-byte instruction would.
        @(negedge clk);
        pop = 0; force pop_n = 3'd4;
        @(negedge clk);
        release pop_n;
        #1;
        chk("four bytes went in one cycle", count, 2);
        chk("head landed on the fifth byte", peek[0], 8'hEE);
        chk("...and the sixth follows it",   peek[1], 8'hFF);

        // Asking for more than is present must consume only what is there:
        // a decoder that gets an instruction's length wrong must not be able
        // to run the head past the tail.
        @(negedge clk);
        force pop_n = 3'd6;
        @(negedge clk);
        release pop_n;
        #1;
        chk("over-long pop is clamped, not wrapped", count, 0);

        // The head wraps mid-buffer, so a multi-byte pop has to wrap too.
        push(16'h2211, 1'b1);
        push(16'h4433, 1'b1);
        #1;
        chk("refilled after wrap", count, 4);
        chk("first byte after wrap", peek[0], 8'h11);
        chk("fourth byte after wrap", peek[3], 8'h44);
        @(negedge clk);
        force pop_n = 3'd3;
        @(negedge clk);
        release pop_n;
        #1;
        chk("three consumed across the wrap", count, 1);
        chk("the remaining byte is the last one", peek[0], 8'h44);

        $display("");
        $display("==================================");
        $display(" checks: %0d   failures: %0d", checks, errors);
        $display("==================================");
        if (errors == 0) $display("ALL TESTS PASSED");
        $finish;
    end

endmodule
