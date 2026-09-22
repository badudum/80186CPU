`timescale 1ns/1ns
//
// SDRAM arbiter.
//
// The model below deliberately reproduces the two habits of the real
// sdram_controller that make arbitration non-trivial:
//
//   1. It starts an access whenever rd or wr is high and it is idle. It does
//      NOT require the request to fall in between. So a requester that holds
//      its line one cycle too long gets a second, unasked-for access -- a
//      double write, or a stray read. The arbiter has to prevent that.
//
//   2. It periodically becomes unavailable for a while, the way a refresh
//      cycle makes the real one unavailable, and refresh outranks requests.
//      Arbitration has to survive the controller simply not answering.
//
// What is checked is not just "data arrives" but the two properties that only
// show up under contention: every requester eventually gets served however
// greedy its neighbours are, and each request produces exactly one access.
//
module tb_sdram_arb;

    localparam int NREQ = 3;
    localparam int AW   = 24;

    logic clk = 0, rst_n = 0;

    logic [NREQ-1:0] req_rd = '0, req_wr = '0, req_ready;
    logic [AW-1:0]   req_addr  [NREQ];
    logic [15:0]     req_wdata [NREQ];
    logic [1:0]      req_be    [NREQ];
    logic [15:0]     req_rdata;

    logic [AW-1:0] mem_addr;
    logic [15:0]   mem_wdata, mem_rdata;
    logic [1:0]    mem_be;
    logic          mem_rd, mem_wr, mem_ready;

    sdram_arbiter #(.NREQ(NREQ), .AW(AW)) dut (
        .clk (clk), .rst_n (rst_n),
        .req_rd (req_rd), .req_wr (req_wr),
        .req_addr (req_addr), .req_wdata (req_wdata), .req_be (req_be),
        .req_ready (req_ready), .req_rdata (req_rdata),
        .mem_addr (mem_addr), .mem_wdata (mem_wdata), .mem_be (mem_be),
        .mem_rd (mem_rd), .mem_wr (mem_wr),
        .mem_rdata (mem_rdata), .mem_ready (mem_ready)
    );

    always #5 clk = ~clk;

    int cycles = 0;
    always @(posedge clk) cycles++;

    // ---- a controller model with the real one's habits ----
    localparam int LATENCY = 4;

    // Wide enough that the test's address ranges do not alias onto each
    // other -- a 12-bit window folded 001000h and 003000h together and made
    // the model look like an arbiter bug.
    logic [15:0] store [0:16383];       // word-addressed window
    int  state;                         // 0 idle, 1 busy, 2 recover, 3 refresh
    int  cnt;
    int  refresh_gap;
    int  accesses;                      // how many the memory actually performed

    logic [AW-1:0] lat_addr;
    logic [15:0]   lat_wdata;
    logic [1:0]    lat_be;
    logic          lat_wr;

    // The index is computed into a signal rather than called inline in the
    // subscript: ModelSim ASE 10.5b throws an internal compiler error on a
    // function call in an array-index position.
    logic [13:0] cur_widx, lat_widx;
    assign cur_widx = (mem_addr >> 1) & 14'h3FFF;

    // Plain `always`, not `always_ff`: the initial block below also clears
    // `store`, and always_ff forbids a second driver.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= 0; cnt <= 0; mem_ready <= 1'b0;
            accesses <= 0; refresh_gap <= 40; mem_rdata <= 16'h0000;
        end else begin
            mem_ready <= 1'b0;
            refresh_gap <= refresh_gap - 1;

            case (state)
                0: begin
                    if (refresh_gap <= 0) begin
                        // Refresh outranks a pending access, as in the real one.
                        state <= 3; cnt <= 6; refresh_gap <= 40;
                    end else if (mem_rd || mem_wr) begin
                        lat_addr  <= mem_addr;
                        lat_widx  <= cur_widx;
                        lat_wdata <= mem_wdata;
                        lat_be    <= mem_be;
                        lat_wr    <= mem_wr;
                        cnt       <= LATENCY;
                        state     <= 1;
                        accesses  <= accesses + 1;
                    end
                end
                1: begin
                    if (cnt == 0) begin
                        if (lat_wr) begin
                            if (lat_be[0]) store[lat_widx][7:0]  <= lat_wdata[7:0];
                            if (lat_be[1]) store[lat_widx][15:8] <= lat_wdata[15:8];
                        end else begin
                            mem_rdata <= store[lat_widx];
                        end
                        mem_ready <= 1'b1;
                        cnt   <= 3;
                        state <= 2;
                    end else cnt <= cnt - 1;
                end
                2: if (cnt == 0) state <= 0; else cnt <= cnt - 1;
                3: if (cnt == 0) state <= 0; else cnt <= cnt - 1;
            endcase
        end
    end

    int errors = 0, checks = 0;
    task chk(input string nm, input int got, input int exp);
        checks++;
        if (got !== exp) begin
            $display("FAIL %-46s got=%0d exp=%0d", nm, got, exp);
            errors++;
        end
    endtask

    // ---- requester drivers ----
    int served_count [NREQ];

    task automatic do_write(input int id, input [AW-1:0] a, input [15:0] d);
        begin
            @(negedge clk);
            req_addr[id]  = a;
            req_wdata[id] = d;
            req_be[id]    = 2'b11;
            req_wr[id]    = 1'b1;
            do @(negedge clk); while (!req_ready[id]);
            req_wr[id] = 1'b0;
            served_count[id]++;
            @(negedge clk);
        end
    endtask

    task automatic do_read(input int id, input [AW-1:0] a, output [15:0] d);
        begin
            @(negedge clk);
            req_addr[id] = a;
            req_be[id]   = 2'b11;
            req_rd[id]   = 1'b1;
            do @(negedge clk); while (!req_ready[id]);
            d = req_rdata;
            req_rd[id] = 1'b0;
            served_count[id]++;
            @(negedge clk);
        end
    endtask

    logic [15:0] v;
    int i, n_before;
    int wait_cycles;
    logic stop_greedy = 0;
    int done0, done1, done2;

    initial begin
        for (i = 0; i < NREQ; i++) begin
            req_addr[i] = '0; req_wdata[i] = '0; req_be[i] = 2'b11;
            served_count[i] = 0;
        end
        for (i = 0; i < 16384; i++) store[i] = 16'h0000;

        repeat (4) @(negedge clk);
        rst_n = 1;
        @(negedge clk);

        // ---------------- one requester, nothing to contend with ----------------
        do_write(0, 24'h000100, 16'hBEEF);
        do_read (0, 24'h000100, v);
        chk("uncontended write then read", v, 16'hBEEF);

        do_write(1, 24'h000200, 16'h1234);
        do_read (1, 24'h000200, v);
        chk("a second requester works too", v, 16'h1234);

        // ---------------- one access per request ----------------
        // The memory counts what it actually performed; it must match.
        n_before = accesses;
        do_write(2, 24'h000300, 16'hAAAA);
        chk("one request produced one access", accesses - n_before, 1);

        n_before = accesses;
        do_read(2, 24'h000300, v);
        chk("one read produced one access", accesses - n_before, 1);
        chk("and returned the right data", v, 16'hAAAA);

        // ---------------- a requester that HOLDS its line ----------------
        // This is the case the controller alone gets wrong. Keeping wr high
        // well past ready must not write a second time.
        n_before = accesses;
        @(negedge clk);
        req_addr[0] = 24'h000400; req_wdata[0] = 16'h5555;
        req_be[0] = 2'b11; req_wr[0] = 1'b1;
        do @(negedge clk); while (!req_ready[0]);
        // ... and hold it up for a long time afterwards
        repeat (60) @(negedge clk);
        req_wr[0] = 1'b0;
        @(negedge clk);
        chk("a held request still only accesses once", accesses - n_before, 1);

        // ---------------- three requesters at once ----------------
        // Each hammers its own region; interleaving must not mix them up.
        fork
            begin : p0
                logic [15:0] r;
                for (int k = 0; k < 24; k++) do_write(0, 24'h001000 + k*2, 16'h1000 + k);
                for (int k = 0; k < 24; k++) begin
                    do_read(0, 24'h001000 + k*2, r);
                    if (r !== 16'h1000 + k) begin
                        $display("FAIL requester 0 read %04h at %0d, expected %04h",
                                 r, k, 16'h1000 + k);
                        errors++;
                    end
                end
                done0 = 1;
            end
            begin : p1
                logic [15:0] r;
                for (int k = 0; k < 24; k++) do_write(1, 24'h002000 + k*2, 16'h2000 + k);
                for (int k = 0; k < 24; k++) begin
                    do_read(1, 24'h002000 + k*2, r);
                    if (r !== 16'h2000 + k) begin
                        $display("FAIL requester 1 read %04h at %0d, expected %04h",
                                 r, k, 16'h2000 + k);
                        errors++;
                    end
                end
                done1 = 1;
            end
            begin : p2
                logic [15:0] r;
                for (int k = 0; k < 24; k++) do_write(2, 24'h003000 + k*2, 16'h3000 + k);
                for (int k = 0; k < 24; k++) begin
                    do_read(2, 24'h003000 + k*2, r);
                    if (r !== 16'h3000 + k) begin
                        $display("FAIL requester 2 read %04h at %0d, expected %04h",
                                 r, k, 16'h3000 + k);
                        errors++;
                    end
                end
                done2 = 1;
            end
        join
        checks++;    // the three loops above check their own data
        chk("all three finished", done0 + done1 + done2, 3);
        chk("requester 0 completed every access", served_count[0] >= 48, 1);
        chk("requester 1 completed every access", served_count[1] >= 48, 1);
        chk("requester 2 completed every access", served_count[2] >= 48, 1);

        // ---------------- no starvation under greedy neighbours ----------------
        // TWO greedy requesters, not one. With a single greedy neighbour the
        // `served` flag alone is enough: it must let go to become eligible
        // again, and the victim slips in during that gap. It takes two of them
        // taking turns to actually squeeze a third out, which is the case
        // rotation exists for -- and the reason the first version of this test
        // passed with fixed priority and proved nothing.
        stop_greedy = 0;
        wait_cycles = 0;
        fork
            begin : greedy0
                while (!stop_greedy) do_write(0, 24'h000500, 16'h0001);
            end
            begin : greedy1
                while (!stop_greedy) do_write(1, 24'h000600, 16'h0002);
            end
            begin : victim
                logic [15:0] r;
                int t0;
                repeat (40) @(negedge clk);       // let the greedy pair get going
                t0 = cycles;
                do_read(2, 24'h000300, r);
                wait_cycles = cycles - t0;
                chk("the squeezed-out requester got its data", r, 16'hAAAA);
                stop_greedy = 1;
            end
        join

        $display("");
        $display("  requester 2 waited %0d cycles behind two greedy neighbours",
                 wait_cycles);
        $display("");
        // Rotation bounds this at roughly one turn per other requester. Fixed
        // priority lets the other two hand off to each other indefinitely.
        checks++;
        if (wait_cycles > 120) begin
            $display("FAIL requester 2 waited %0d cycles -- it is being starved",
                     wait_cycles);
            errors++;
        end

        req_rd = '0;
        req_wr = '0;
        @(negedge clk);

        $display("");
        $display("==================================");
        $display(" checks: %0d   failures: %0d", checks, errors);
        $display("==================================");
        if (errors == 0) $display("ALL TESTS PASSED");
        $finish;
    end

    initial begin
        #4000000;
        $display("FAIL global timeout");
        $display(" checks: %0d   failures: %0d", checks, errors + 1);
        $finish;
    end

endmodule
