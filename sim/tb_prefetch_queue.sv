`timescale 1ns/1ns
module tb_prefetch_queue;

    logic        clk = 0, rst_n = 0;
    logic        write_en = 0, write_word = 1, pop = 0, flush = 0;
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

        $display("");
        $display("==================================");
        $display(" checks: %0d   failures: %0d", checks, errors);
        $display("==================================");
        if (errors == 0) $display("ALL TESTS PASSED");
        $finish;
    end

endmodule
