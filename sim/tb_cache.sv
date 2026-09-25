`timescale 1ns/1ns
//
// The cache, against a memory model that answers slowly enough for a hit and
// a miss to be told apart by eye.
//
// The driver below mimics the BIU's actual bus timing rather than just
// wiggling rd: address in T1, rd in T2, wait for ready, drop in T4. That
// matters, because the cache's whole claim to a free hit rests on the address
// being stable a cycle BEFORE rd arrives. A testbench that asserted both
// together would still pass while the real machine lost a cycle on every
// access, so the timing is part of what is being tested -- `t_read` returns
// the cycle count and the checks below assert on it.
//
// What is checked:
//   - a miss returns the right word, and the line around it is then resident
//   - a hit costs the 4-T-state minimum, a miss visibly more
//   - the critical word comes back first, so a miss costs about what an
//     uncached read costs rather than a whole line
//   - writes reach memory, and a write that hits updates the cached copy,
//     including a single-byte write
//   - a write that misses does not allocate
//   - two addresses that share an index evict each other and stay correct
//   - a foreign write below 1 MB invalidates; one above 1 MB does not
//
module tb_cache;

    localparam int KB = 2, LINE_WORDS = 4;

    logic        clk = 0, rst_n = 0;
    logic [19:0] c_addr = 0;
    logic [15:0] c_wdata = 0, c_rdata;
    logic        c_rd = 0, c_wr = 0, c_ready;
    logic [1:0]  c_be = 2'b11;

    logic [23:0] m_addr;
    logic [15:0] m_wdata, m_rdata;
    logic        m_rd, m_wr, m_ready;
    logic [1:0]  m_be;

    logic        snoop_wr = 0;
    logic [23:0] snoop_addr = 0;
    logic        stat_hit, stat_miss;
    logic        stat_pf_start, stat_pf_done, stat_pf_abort;
    int pf_starts = 0, pf_dones = 0, pf_aborts = 0;
    always @(posedge clk) begin
        if (stat_pf_start) pf_starts++;
        if (stat_pf_done)  pf_dones++;
        if (stat_pf_abort) pf_aborts++;
    end

    // PREFETCH is forced ON here. It is OFF by default because measuring it
    // on the MS-DOS boot showed it losing -- CPI 18.36 to 19.81 -- but the
    // logic still has to be correct for the day the memory system can carry
    // it, and three real bugs in it were only found by these checks plus a
    // full-system boot.
    cache #(.KB(KB), .LINE_WORDS(LINE_WORDS), .PREFETCH(1'b1)) dut (.*);

    always #5 clk = ~clk;

    // ---- a deliberately slow memory ----
    // LAT is the number of wait cycles, chosen so that a miss and a hit are
    // separated by more than one cycle in every measurement below.
    localparam int LAT = 6;
    logic [7:0] mem [0:'h1FFFF];
    int         lat_cnt = 0;
    logic       busy = 0;
    int         mem_writes = 0;

    // Plain `always`: the initial block below also fills `mem`, and always_ff
    // forbids a second driver.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy <= 0; lat_cnt <= 0; m_ready <= 0; m_rdata <= 0;
        end else begin
            m_ready <= 1'b0;
            if (!busy && (m_rd || m_wr)) begin
                busy <= 1'b1; lat_cnt <= LAT;
            end else if (busy) begin
                if (lat_cnt > 1) lat_cnt <= lat_cnt - 1;
                else begin
                    busy    <= 1'b0;
                    m_ready <= 1'b1;
                    if (m_rd)
                        m_rdata <= {mem[m_addr + 1], mem[m_addr]};
                    if (m_wr) begin
                        if (m_be[0]) mem[m_addr]     <= m_wdata[7:0];
                        if (m_be[1]) mem[m_addr + 1] <= m_wdata[15:8];
                        mem_writes <= mem_writes + 1;
                    end
                end
            end
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
    task chkh(input string nm, input int got, input int exp);
        checks++;
        if (got !== exp) begin
            $display("FAIL %-46s got=%04h exp=%04h", nm, got, exp);
            errors++;
        end
    endtask

    // ---- a bus cycle shaped like the BIU's ----
    // T1 drives the address, T2 raises rd, ready is sampled from T3 onwards,
    // T4 drops everything. Returns the T1-to-ready cycle count: 3 is the
    // 4-T-state minimum an 80186 can achieve.
    int cyc;
    task automatic t_read(input int a, output logic [15:0] d, output int cycles);
        begin
            @(negedge clk);  c_addr = a[19:0]; c_be = 2'b11;     // T1
            @(negedge clk);  c_rd = 1'b1;                        // T2
            cycles = 1;
            forever begin
                @(posedge clk);
                if (c_ready) break;
                @(negedge clk);
                cycles++;
            end
            d = c_rdata;
            cycles++;
            @(negedge clk);  c_rd = 1'b0;                        // T4
            @(negedge clk);
        end
    endtask

    task automatic t_write(input int a, input logic [15:0] d, input logic [1:0] be);
        begin
            @(negedge clk);  c_addr = a[19:0]; c_wdata = d; c_be = be;
            @(negedge clk);  c_wr = 1'b1;
            forever begin
                @(posedge clk);
                if (c_ready) break;
                @(negedge clk);
            end
            @(negedge clk);  c_wr = 1'b0; c_be = 2'b11;
            @(negedge clk);
        end
    endtask

    // A miss releases the CPU as soon as ITS word arrives and fills the rest
    // of the line behind it, so for a short while afterwards the line is not
    // yet resident. Measurements of hit latency have to let that finish
    // first, or they measure the tail of the previous miss. `t_fill` does
    // both halves: fetch, settle, and hand back the latency of a genuinely
    // resident re-read.
    task automatic settle;
        begin repeat (60) @(negedge clk); end
    endtask

    task automatic t_fill(input int a);
        logic [15:0] dd; int cc;
        begin
            t_read(a, dd, cc);
            settle();
        end
    endtask

    task automatic flush(input int a);
        begin
            @(negedge clk); snoop_addr = a[23:0]; snoop_wr = 1'b1;
            @(negedge clk); snoop_wr = 1'b0;
            @(negedge clk);
        end
    endtask

    logic [15:0] d;
    int i, miss_cycles, hit_cycles, c2;

    initial begin
        for (i = 0; i < 'h1FFFF; i++) mem[i] = i[7:0];
        // Recognisable words: address 2*n holds n + 1000h.
        for (i = 0; i < 'hFFF0; i++) begin
            mem[i * 2]     = (i + 'h1000) & 'hFF;
            mem[i * 2 + 1] = ((i + 'h1000) >> 8) & 'hFF;
        end

        repeat (4) @(negedge clk);
        rst_n = 1;
        repeat (4) @(negedge clk);

        // ---- a cold read ----
        t_read('h00100, d, miss_cycles);
        chkh("cold read returns the right word", d, 16'h1080);
        checks++;
        if (miss_cycles <= 3) begin
            $display("FAIL a cold read was not a miss (%0d cycles)", miss_cycles);
            errors++;
        end

        // The rest of the line has to have landed behind it, so the three
        // neighbouring words are now resident once the fill has finished.
        settle();
        t_read('h00102, d, hit_cycles);
        chkh("next word in the line", d, 16'h1081);
        chk("...and it hit (4-T-state minimum)", hit_cycles, 3);

        t_read('h00104, d, c2);
        chkh("third word in the line", d, 16'h1082);
        chk("...also a hit", c2, 3);

        t_read('h00106, d, c2);
        chkh("fourth word in the line", d, 16'h1083);
        chk("...also a hit", c2, 3);

        // ---- and the original address is still there ----
        t_read('h00100, d, c2);
        chkh("the word that caused the miss is cached", d, 16'h1080);
        chk("...and now hits", c2, 3);

        // ---- critical word first ----
        // Asking for the LAST word of a fresh line must cost about what
        // asking for the first one did. If the fill were in line order this
        // would be a whole line longer.
        t_read('h0020E, d, c2);
        chkh("last word of a cold line", d, 16'h1107);
        checks++;
        if (c2 > miss_cycles + 1) begin
            $display("FAIL critical word not fetched first: %0d cycles vs %0d",
                     c2, miss_cycles);
            errors++;
        end
        settle();
        t_read('h00208, d, c2);
        chkh("...and the rest of that line filled behind it", d, 16'h1104);
        chk("...as a hit", c2, 3);

        // ---- a request that lands DURING a background fill ----
        // This is the common case, not an edge case: the prefetch queue asks
        // for the next word a few cycles after the last one arrived, which is
        // while the rest of the line is still being fetched. It must wait and
        // then be answered from the line, not started as a second fill of the
        // same line and not answered with whatever was there before.
        t_read('h00300, d, c2);          // cold: fill starts
        t_read('h00302, d, c2);          // arrives mid-fill
        chkh("a read during a background fill gets the right word", d, 16'h1181);
        settle();
        t_read('h00302, d, c2);
        chk("...and the line really was cached, not refetched twice", c2, 3);

        // ---- writes ----
        mem_writes = 0;
        t_write('h00100, 16'hBEEF, 2'b11);
        chk("a write reached memory", mem_writes, 1);
        chkh("memory took the low byte",  mem['h00100], 8'hEF);
        chkh("memory took the high byte", mem['h00101], 8'hBE);

        t_read('h00100, d, c2);
        chkh("a write that hit updated the cached copy", d, 16'hBEEF);
        chk("...and the read still hits", c2, 3);

        // a single byte, into a line that is resident
        t_write('h00102, 16'h00A5, 2'b01);
        t_read('h00102, d, c2);
        chkh("a byte write updated only the low byte", d, 16'h10A5);
        chk("...and still hits", c2, 3);

        // a write that misses must not pull the line in
        t_write('h04000, 16'hCAFE, 2'b11);
        chkh("a missing write still reached memory", mem['h04000], 8'hFE);
        t_read('h04000, d, c2);
        chkh("...returns the written word", d, 16'hCAFE);
        checks++;
        if (c2 <= 3) begin
            $display("FAIL a write allocated a line it should not have");
            errors++;
        end

        // ---- two addresses sharing an index ----
        // KB*1024 bytes of data, so addresses that far apart land on the same
        // line and evict each other.
        t_fill('h00700);
        t_fill('h00700 + KB * 1024);
        t_read('h00700 + KB * 1024, d, c2);
        chkh("the aliasing address reads its own data", d,
             16'h1000 + ('h00700 + KB * 1024) / 2);
        t_read('h00700, d, c2);
        chkh("...and the first one is correct again after eviction", d, 16'h1380);
        checks++;
        if (c2 <= 3) begin
            $display("FAIL an evicted line still hit -- tag compare is wrong");
            errors++;
        end

        // ---- the one-cycle window where a fill completes ----
        // A re-read that arrives just as a fill finishes used to be answered
        // from a stale registered tag: the evicted line's tag was still in
        // the lookup register while the valid bit had already been set for
        // its replacement, so asking for the EVICTED address compared equal
        // and got the new line's data. It needs three things to line up --
        // two addresses sharing an index, the second evicting the first, and
        // the re-read landing in exactly the cycle the fill completes -- so
        // the delay is swept rather than guessed at. Without the sweep this
        // passes or fails depending on the memory latency, which is not a
        // property anyone should have to think about to keep the test honest.
        for (i = 0; i <= 10; i++) begin
            flush('h000000);                      // start from an empty cache
            t_fill('h00900);                      // line A resident
            t_read('h01100, d, c2);               // evicts it, fill still running
            repeat (i) @(negedge clk);
            t_read('h00900, d, c2);               // lands near the fill's end
            checks++;
            if (d !== 16'h1480) begin
                $display("FAIL stale lookup answered an evicted address (delay %0d): got=%04h exp=1480", i, d);
                errors++;
            end
        end

        // ---- snooping ----
        t_fill('h00500);
        t_read('h00500, d, c2);
        chk("resident before the snoop", c2, 3);
        flush('h000400);                 // a foreign write below 1 MB
        t_read('h00500, d, c2);
        checks++;
        if (c2 <= 3) begin
            $display("FAIL a foreign write below 1 MB did not invalidate");
            errors++;
        end
        chkh("...and the refetched word is right", d, 16'h1280);

        settle();
        t_read('h00500, d, c2);
        chk("resident again", c2, 3);
        flush('h100000);                 // the disk image, which cannot alias
        t_read('h00500, d, c2);
        chk("a write above 1 MB left the cache alone", c2, 3);

        // ---- next-line prefetch ----
        // The line after a demand miss should arrive without being asked
        // for, so a sequential walk stops missing after the first line.
        flush('h000000);
        settle();
        t_read('h01000, d, c2);                 // cold miss on one line
        chk("the missing line was fetched", d, 16'h1000 + 'h01000/2);
        settle();                               // let the prefetch run
        t_read('h01008, d, c2);                 // the NEXT line along
        chk("the next line arrived without being asked for", c2, 3);
        chkh("...and its data is right", d, 16'h1000 + 'h01008/2);

        // It must not chain. Prefetching off a prefetch would run ahead of
        // the program and evict lines that were actually wanted.
        settle();
        t_read('h01010, d, c2);                 // two lines on
        checks++;
        if (c2 <= 3) begin
            $display("FAIL prefetch chained past the line after the miss");
            errors++;
        end

        // ---- a demand access must never wait for a prefetch ----
        // THE POINT OF THE WHOLE DESIGN. The memory path is blocking, so a
        // prefetch holding the port while the CPU wants it would make the
        // machine slower rather than faster.
        //
        // The wait for is_pf is what makes this test mean anything. An
        // earlier version just issued two reads back to back and measured 26
        // cycles against 10 -- but that delay was the PREVIOUS demand fill
        // still finishing in the background, which critical-word-first does
        // by design and has nothing to do with prefetching. Waiting until a
        // prefetch is genuinely in flight is the only way to attribute the
        // delay to the right thing.
        flush('h000000);
        settle();
        t_read('h02000, d, miss_cycles);        // reference: a plain miss
        settle();

        flush('h000000);
        settle();
        t_read('h03000, d, c2);                 // miss; queues a prefetch
        i = 0;
        while (!dut.is_pf && i < 400) begin @(negedge clk); i++; end
        checks++;
        if (!dut.is_pf) begin
            $display("FAIL no prefetch was ever started after a miss");
            errors++;
        end
        repeat (3) @(negedge clk);              // land inside the fill
        t_read('h05000, d, c2);                 // demand, mid-prefetch
        chkh("the demand read got its own data", d, 16'h1000 + 'h05000/2);
        checks++;
        if (c2 > miss_cycles + LAT + 6) begin
            $display("FAIL a demand read waited for a prefetch: %0d vs %0d",
                     c2, miss_cycles);
            errors++;
        end
        checks++;
        if (pf_aborts == 0) begin
            $display("FAIL the prefetch was not abandoned for the demand");
            errors++;
        end

        // An abandoned prefetch must not leave a half-filled line valid.
        settle();
        t_read('h03008, d, c2);
        chkh("an abandoned line is refetched correctly", d,
             16'h1000 + 'h03008/2);

        // ---- a prefetch must not corrupt the line it displaces ----
        // The cache is direct mapped, so a prefetched line shares its index
        // with whatever is already resident there, and the fill writes into
        // the data array straight away. If the displaced line is not
        // invalidated first, its tag and valid bit still claim it is good
        // while its data has been overwritten -- and the cache serves the
        // corruption as a hit. Addresses 2 KB apart share an index here.
        flush('h000000);
        settle();
        // 6000 and 6800 are 2 KB apart so they share an index, and neither
        // has been written by an earlier case -- 4000 has, which is how the
        // first version of this check came to expect the wrong value.
        t_fill('h06000);                        // resident at index 0
        t_read('h06000, d, c2);
        chk("the line is resident before the prefetch", c2, 3);

        // A miss whose NEXT line lands on the same index with a different tag.
        t_read('h067F8, d, c2);
        settle();                               // let the prefetch complete

        t_read('h06000, d, c2);
        chkh("the displaced line is not served corrupted", d,
             16'h1000 + 'h06000/2);

        // ...and the case that actually reaches the bug: read the displaced
        // line WHILE the prefetch is filling over it. The fill writes the
        // data array immediately but only fixes the tag at the end, so in
        // between the line still claims to be the old one. Aborting there
        // and serving the read gives a hit on a line whose data has been
        // half replaced. Completing the prefetch first hides this, which is
        // why the check above passes either way.
        flush('h000000);
        settle();
        t_fill('h06000);
        t_read('h067F8, d, c2);                 // queues a prefetch of 6800
        i = 0;
        while (!dut.is_pf && i < 400) begin @(negedge clk); i++; end
        repeat (4) @(negedge clk);              // let it write a word or two
        t_read('h06000, d, c2);                 // the displaced line
        chkh("no corrupt hit on a line a prefetch was overwriting", d,
             16'h1000 + 'h06000/2);

        // ---- a prefetch must never answer the CPU ----
        // fill_ack forwards the first word of a fill straight to the CPU and
        // asserts c_ready. That is the critical-word-first path and it is
        // right for a demand miss. For a prefetch nobody asked for the word,
        // so firing it hands a waiting read data from an address it never
        // requested -- and the machine then executes whatever was
        // prefetched. Every other check in this file passed while that was
        // broken; only booting DOS caught it.
        //
        // The read is issued as early as possible after the prefetch starts,
        // so it is still waiting when the prefetch's FIRST word lands.
        flush('h000000);
        settle();
        t_read('h07000, d, c2);                 // miss; queues a prefetch
        i = 0;
        while (!dut.is_pf && i < 400) begin @(negedge clk); i++; end
        t_read('h0A000, d, c2);                 // unrelated address, at once
        chkh("a waiting read is not answered with prefetch data", d,
             16'h1000 + 'h0A000/2);

        // ---- the counters ----
        checks++;
        if (stat_hit === 1'bx || stat_miss === 1'bx) begin
            $display("FAIL the statistics outputs are undriven");
            errors++;
        end

        $display("");
        $display("==================================");
        $display(" checks: %0d   failures: %0d", checks, errors);
        $display("==================================");
        if (errors == 0) $display("ALL TESTS PASSED");
        $finish;
    end

    initial begin
        #500000;
        $display("FAIL global timeout");
        $display(" checks: %0d   failures: %0d", checks, errors + 1);
        $finish;
    end

endmodule
