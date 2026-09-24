`timescale 1ns/1ns
module tb_biu;

    logic        clk = 0, rst_n = 0;
    logic [19:0] addr;
    logic [15:0] dout, din;
    logic        rd, wr, io_cycle, bhe, a0, ale;
    logic [2:0]  s;
    logic        ready;
    logic [19:0] fetch_addr = 20'h00000;
    logic        fetch_set = 0;
    logic [7:0]  fetch_data;
    logic        fetch_valid, fetch_pop = 0;
    // The queue consumes by count now; this testbench still thinks in single
    // bytes, so `fetch_pop` stays as its shorthand.
    logic [2:0]  fetch_pop_n;
    logic [7:0]  fetch_peek [0:5];
    logic [3:0]  fetch_count;
    assign fetch_pop_n = fetch_pop ? 3'd1 : 3'd0;
    logic        req = 0, req_wr = 0, req_io = 0, req_word = 0;
    logic [19:0] req_addr = 0;
    logic [15:0] req_wdata = 0, req_rdata;
    logic        req_done;
    logic        bus_idle;
    logic        hold_req = 1'b0;   // no competing bus master in this test

    biu dut (.*);

    always #5 clk = ~clk;

    // ---- byte-addressable memory model with real lane behaviour ----
    // Lower lane (D7-D0) is the even byte and is enabled when a0 = 0.
    // Upper lane (D15-D8) is the odd byte and is enabled when bhe = 0.
    logic [7:0] mem [0:'hFFFF];
    logic [15:0] even_a, odd_a;
    assign even_a = {addr[15:1], 1'b0};
    assign odd_a  = {addr[15:1], 1'b1};

    assign din = {mem[odd_a], mem[even_a]};

    // Wait-state injection: hold READY low for `waits` cycles of each T3.
    int waits = 0;
    int wait_cnt = 0;
    always_ff @(posedge clk) begin
        if (rd || wr) begin
            if (wait_cnt < waits) wait_cnt <= wait_cnt + 1;
        end else wait_cnt <= 0;
    end
    assign ready = (wait_cnt >= waits);

    // Plain `always`, not `always_ff`: the initial block also writes mem to
    // set up test data, and always_ff forbids a second driver.
    always @(posedge clk) begin
        if (wr && ready) begin
            if (!a0)  mem[even_a] <= dout[7:0];
            if (!bhe) mem[odd_a]  <= dout[15:8];
        end
    end

    int errors = 0, checks = 0;
    task chk(input string nm, input int got, input int exp);
        checks++;
        if (got !== exp) begin
            $display("FAIL %-36s got=%0h exp=%0h", nm, got, exp);
            errors++;
        end
    endtask

    task do_read(input [19:0] a, input w, input io, output [15:0] d);
        begin
            @(negedge clk);
            req_addr = a; req_word = w; req_io = io; req_wr = 0; req = 1;
            while (!req_done) @(negedge clk);
            d = req_rdata;
            @(negedge clk);
            req = 0;
            @(negedge clk);
        end
    endtask

    task do_write(input [19:0] a, input w, input [15:0] d);
        begin
            @(negedge clk);
            req_addr = a; req_word = w; req_io = 0; req_wr = 1; req_wdata = d; req = 1;
            while (!req_done) @(negedge clk);
            @(negedge clk);
            req = 0;
            @(negedge clk);
        end
    endtask

    task redirect(input [19:0] a);
        begin
            @(negedge clk);
            fetch_addr = a; fetch_set = 1;
            @(negedge clk);
            fetch_set = 0;
        end
    endtask

    task get_byte(output [7:0] b);
        begin
            while (!fetch_valid) @(negedge clk);
            b = fetch_data;
            fetch_pop = 1;
            @(negedge clk);
            fetch_pop = 0;
        end
    endtask

    logic [15:0] d;
    logic [7:0]  b;
    int i;

    initial begin
        // program bytes at 00100h
        mem['h100] = 8'h11; mem['h101] = 8'h22; mem['h102] = 8'h33;
        mem['h103] = 8'h44; mem['h104] = 8'h55; mem['h105] = 8'h66;
        // data
        mem['h200] = 8'hAA; mem['h201] = 8'hBB;
        mem['h202] = 8'hCC; mem['h203] = 8'hDD;
        for (i = 'h300; i < 'h310; i++) mem[i] = 8'h00;

        repeat (3) @(negedge clk);
        rst_n = 1;
        @(negedge clk);

        // ---------------- sequential instruction fetch ----------------
        redirect(20'h00100);
        get_byte(b); chk("fetch byte 0", b, 8'h11);
        get_byte(b); chk("fetch byte 1", b, 8'h22);
        get_byte(b); chk("fetch byte 2", b, 8'h33);
        get_byte(b); chk("fetch byte 3", b, 8'h44);
        get_byte(b); chk("fetch byte 4", b, 8'h55);
        get_byte(b); chk("fetch byte 5", b, 8'h66);

        // ---------------- redirect to an ODD address ----------------
        // The first fetch reads the aligned word and keeps only the high byte.
        redirect(20'h00101);
        get_byte(b); chk("odd redirect byte 0", b, 8'h22);
        get_byte(b); chk("odd redirect byte 1", b, 8'h33);
        get_byte(b); chk("odd redirect byte 2", b, 8'h44);

        // ---------------- aligned word read ----------------
        do_read(20'h00200, 1, 0, d);
        chk("word read @200", d, 16'hBBAA);

        // ---------------- byte reads, both lanes ----------------
        do_read(20'h00200, 0, 0, d);
        chk("byte read @200 (even lane)", d, 16'h00AA);
        do_read(20'h00201, 0, 0, d);
        chk("byte read @201 (odd lane)", d, 16'h00BB);
        do_read(20'h00203, 0, 0, d);
        chk("byte read @203 (odd lane)", d, 16'h00DD);

        // ---------------- ODD-ADDRESS word read (split cycle) ----------------
        do_read(20'h00201, 1, 0, d);
        chk("odd word read @201", d, 16'hCCBB);

        // ---------------- writes ----------------
        do_write(20'h00300, 1, 16'h3412);
        chk("aligned word write lo", mem['h300], 8'h12);
        chk("aligned word write hi", mem['h301], 8'h34);

        do_write(20'h00302, 0, 16'h0099);
        chk("byte write even lane", mem['h302], 8'h99);
        chk("neighbour untouched", mem['h303], 8'h00);

        do_write(20'h00305, 0, 16'h0077);
        chk("byte write odd lane", mem['h305], 8'h77);
        chk("neighbour untouched", mem['h304], 8'h00);

        // ODD-ADDRESS word write: low byte to the odd address, high byte next
        do_write(20'h00307, 1, 16'hBEEF);
        chk("odd word write low byte",  mem['h307], 8'hEF);
        chk("odd word write high byte", mem['h308], 8'hBE);
        chk("byte before untouched",    mem['h306], 8'h00);

        // read it back through the split-read path
        do_read(20'h00307, 1, 0, d);
        chk("odd word read-back", d, 16'hBEEF);

        // ---------------- I/O cycles use a distinct status ----------------
        fork
            begin
                do_read(20'h00200, 0, 1, d);
            end
            begin
                @(posedge rd);
                #1 chk("io_cycle asserted for I/O read", io_cycle, 1);
                chk("status = I/O read", s, 3'b001);
            end
        join

        // ---------------- wait states ----------------
        waits = 3;
        do_read(20'h00200, 1, 0, d);
        chk("word read with 3 wait states", d, 16'hBBAA);
        do_write(20'h0030A, 1, 16'hF00D);
        chk("write with wait states lo", mem['h30A], 8'h0D);
        chk("write with wait states hi", mem['h30B], 8'hF0);
        waits = 0;

        // ---------------- fetch still works after data traffic ----------------
        redirect(20'h00100);
        get_byte(b); chk("fetch resumes after data", b, 8'h11);
        get_byte(b); chk("fetch resumes after data 2", b, 8'h22);

        // ---------------- flush discards stale prefetch ----------------
        redirect(20'h00100);
        repeat (20) @(negedge clk);       // let the queue fill
        redirect(20'h00104);              // redirect before consuming it
        get_byte(b); chk("flush then refetch", b, 8'h55);
        get_byte(b); chk("flush then refetch 2", b, 8'h66);

        $display("");
        $display("==================================");
        $display(" checks: %0d   failures: %0d", checks, errors);
        $display("==================================");
        if (errors == 0) $display("ALL TESTS PASSED");
        $finish;
    end

    // safety net so a hang fails loudly instead of running forever
    initial begin
        #200000;
        $display("FAIL timeout -- simulation did not complete");
        $display(" checks: %0d   failures: %0d", checks, errors + 1);
        $finish;
    end

endmodule
