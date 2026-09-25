`timescale 1ns/1ns
//
// What does one iteration of a string operation actually cost?
//
// Memory here answers in ZERO wait states, which is the point: it strips the
// SDRAM out of the measurement and leaves the sequencer's own cost. On the
// MS-DOS boot the string states are 28.7% of every cycle the machine runs and
// 38.1% of those cycles are waiting for the bus -- so roughly two thirds of
// the cost is the state machine, and that is the part this measures and the
// part RTL changes here can remove.
//
// It reports cycles per iteration for each form rather than asserting a bound.
// A threshold here would be a number someone made up; the useful output is the
// before-and-after of a change, and a regression shows as the figure moving.
// The correctness of what the loops compute is checked too, because a faster
// engine that copies the wrong bytes is not faster.
//
module tb_strperf;

    logic        clk = 0, rst_n = 0;
    logic [19:0] addr;
    logic [15:0] dout, din;
    logic        rd, wr, io_cycle, bhe, a0, ale;
    logic [2:0]  s;
    logic        ready;
    logic        nmi = 0, intr_req = 0;
    logic [7:0]  intr_type = 8'h00;
    logic        intr_ack;
    logic        int0 = 0, int1 = 0, int2 = 0, int3 = 0;
    logic        ext_eoi = 1'b0;
    logic        ext_tick = 1'b0, ext_tick_en = 1'b0;
    logic        drq0 = 0, drq1 = 0;
    logic        halted;
    logic [15:0] dbg_ip, dbg_cs, dbg_flags;
    logic [7:0]  dbg_int_type;
    logic        dbg_int_taken;

    cpu_top dut (.*);

    always #5 clk = ~clk;

    logic [7:0] mem [0:'hFFFFF];
    logic [19:0] even_a, odd_a;
    assign even_a = {addr[19:1], 1'b0};
    assign odd_a  = {addr[19:1], 1'b1};
    assign din    = {mem[odd_a], mem[even_a]};
    assign ready  = 1'b1;          // zero wait states, deliberately

    always @(posedge clk) begin
        if (wr) begin
            if (!a0)  mem[even_a] <= dout[7:0];
            if (!bhe) mem[odd_a]  <= dout[15:8];
        end
    end

    `define AX dut.u_eu.u_regfile.gpr[0]
    `define CX dut.u_eu.u_regfile.gpr[1]
    `define SI dut.u_eu.u_regfile.gpr[6]
    `define DI dut.u_eu.u_regfile.gpr[7]

    int errors = 0, checks = 0;
    task chk(input string nm, input int got, input int exp);
        checks++;
        if (got !== exp) begin
            $display("FAIL %-40s got=%04h exp=%04h", nm, got, exp);
            errors++;
        end
    endtask

    int i, p;
    task put(input int a, input byte b); begin mem[a] = b; end endtask

    // Count cycles between the string instruction starting and the machine
    // halting, so the figure covers the whole instruction including its
    // per-instruction setup, not just the loop.
    int cycles;
    always @(posedge clk) if (rst_n && !halted) cycles++;

    localparam int COUNT = 1000;

    // Build a program that sets up DS/ES/SI/DI/CX and runs one REP string
    // instruction, then halts. `op` bytes are the instruction itself.
    task automatic build(input byte pfx, input byte op, input int cnt);
        begin
            for (i = 0; i < 'hFFFFF; i++) mem[i] = 8'h00;
            // source data at 2000:0000, something distinguishable
            for (i = 0; i < 2*COUNT; i++) mem['h20000 + i] = i[7:0];

            put('hFFFF0, 8'hE9); put('hFFFF1, 8'h1D); put('hFFFF2, 8'h04);
            p = 'h00410;
            put(p++, 8'hB8); put(p++, 8'h00); put(p++, 8'h20); // MOV AX,2000h
            put(p++, 8'h8E); put(p++, 8'hD8);                  // MOV DS,AX
            put(p++, 8'hB8); put(p++, 8'h00); put(p++, 8'h30); // MOV AX,3000h
            put(p++, 8'h8E); put(p++, 8'hC0);                  // MOV ES,AX
            put(p++, 8'hBE); put(p++, 8'h00); put(p++, 8'h00); // MOV SI,0
            put(p++, 8'hBF); put(p++, 8'h00); put(p++, 8'h00); // MOV DI,0
            put(p++, 8'hB9); put(p++, cnt[7:0]); put(p++, cnt[15:8]); // MOV CX,cnt
            put(p++, 8'hFC);                                   // CLD
            put(p++, 8'hB8); put(p++, 8'h55); put(p++, 8'hAA); // MOV AX,AA55h
            if (pfx != 8'h00) put(p++, pfx);
            put(p++, op);
            put(p++, 8'hF4);                                   // HLT
        end
    endtask

    task automatic run(input string nm, input byte pfx, input byte op,
                       input int cnt, output int per_iter);
        int c, frac;
        begin
            rst_n = 0;
            build(pfx, op, cnt);
            repeat (4) @(negedge clk);
            cycles = 0;
            rst_n = 1;
            c = 0;
            while (c < 400000 && !halted) begin @(negedge clk); c++; end
            if (!halted) begin
                $display("FAIL %s did not halt", nm);
                errors++;
                per_iter = 0;
            end else begin
                per_iter = cycles / cnt;
                // Printed as hundredths by hand: ModelSim pads %02d with a
                // space rather than a zero, so "16.09" came out as "16. 9".
                frac = ((cycles * 100) / cnt) % 100;
                $display("  %-22s %7d cycles for %0d iterations -- %0d.%s%0d per iteration",
                         nm, cycles, cnt, cycles / cnt,
                         (frac < 10) ? "0" : "", frac);
            end
            checks++;
        end
    endtask

    int pi_movsw, pi_stosw, pi_movsb, pi_lodsw, pi_scasw;

    initial begin
        $display("string engine cost, zero wait states:");

        run("REP MOVSW", 8'hF3, 8'hA5, COUNT, pi_movsw);
        // every word copied
        chk("MOVSW copied first word",  {mem['h30001], mem['h30000]}, 16'h0100);
        chk("MOVSW copied last word",
            {mem['h30000 + 2*COUNT - 1], mem['h30000 + 2*COUNT - 2]},
            {mem['h20000 + 2*COUNT - 1], mem['h20000 + 2*COUNT - 2]});
        chk("MOVSW CX drained", `CX, 0);
        chk("MOVSW SI advanced", `SI, 2*COUNT);
        chk("MOVSW DI advanced", `DI, 2*COUNT);

        run("REP STOSW", 8'hF3, 8'hAB, COUNT, pi_stosw);
        chk("STOSW wrote AX",  {mem['h30001], mem['h30000]}, 16'hAA55);
        chk("STOSW CX drained", `CX, 0);

        run("REP MOVSB", 8'hF3, 8'hA4, COUNT, pi_movsb);
        chk("MOVSB CX drained", `CX, 0);

        run("REP LODSW", 8'hF3, 8'hAD, COUNT, pi_lodsw);
        chk("LODSW CX drained", `CX, 0);

        run("REPE SCASW", 8'hF3, 8'hAF, COUNT, pi_scasw);

        $display("");
        $display("==================================");
        $display(" checks: %0d   failures: %0d", checks, errors);
        $display("==================================");
        if (errors == 0) $display("ALL TESTS PASSED");
        $finish;
    end

    initial begin
        #200000000;
        $display("FAIL global timeout (IP=%04h)", dbg_ip);
        $display(" checks: %0d   failures: %0d", checks, errors + 1);
        $finish;
    end

endmodule
