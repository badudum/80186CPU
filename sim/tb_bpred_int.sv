`timescale 1ns/1ns
//
// Interrupts landing on a speculated branch.
//
// THE INVARIANT: an interrupt arriving at ANY cycle must not change what the
// program computes. It may change how long the program takes; it may not
// change the answer. That is the whole contract, and it is exactly the
// contract speculation is most likely to break, because a prediction is state
// carried across an instruction boundary -- the same boundary an interrupt is
// accepted at.
//
// WHY A SWEEP RATHER THAN A CASE. The dangerous window is a handful of cycles
// wide: the cycle a branch is captured and predicted, the cycles between that
// and the branch resolving, and the cycle the misprediction is recovered on.
// Hand-picking an offset tests whichever cycle the author happened to think
// of. Firing at every offset in turn tests all of them, including the ones
// nobody would think of, and it does it without needing to know which cycle is
// which.
//
// tb_interrupt already covers whether an interrupt is TAKEN correctly. This
// covers whether taking one is INVISIBLE, which is a different question and
// the one that matters once a branch can be in flight.
//
// The workload is deliberately branch- and call-heavy: a hot backward
// conditional branch, which is what trains a two-bit counter to predict taken,
// wrapped around nested CALL/RET, which is the one place the return address
// and the predicted target can disagree.
//
module tb_bpred_int;

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
    assign ready  = 1'b1;

    always @(posedge clk) begin
        if (wr) begin
            if (!a0)  mem[even_a] <= dout[7:0];
            if (!bhe) mem[odd_a]  <= dout[15:8];
        end
    end

    `define AX dut.u_eu.u_regfile.gpr[0]
    `define CX dut.u_eu.u_regfile.gpr[1]
    `define BX dut.u_eu.u_regfile.gpr[3]
    `define SP dut.u_eu.u_regfile.gpr[4]
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
    task vec(input int typ, input int off, input int seg);
        begin
            mem[typ*4+0] = off[7:0];   mem[typ*4+1] = off[15:8];
            mem[typ*4+2] = seg[7:0];   mem[typ*4+3] = seg[15:8];
        end
    endtask

    // CS=FFFF, so physical = FFFF0 + offset (mod 1 MB): offset 0420 is
    // physical 00410, just clear of the vector table.
    localparam int ITERS = 100;
    // BX ends as 100+99+...+1.
    localparam int EXPECT_BX = ITERS * (ITERS + 1) / 2;

    task automatic load_program;
        begin
            for (i = 0; i < 'hFFFFF; i++) mem[i] = 8'h00;

            vec('h40, 'h0500, 'hFFFF);           // first IRQ
            vec('h41, 'h0520, 'hFFFF);           // the one that nests inside it

            // reset vector: JMP near to 0420
            put('hFFFF0, 8'hE9); put('hFFFF1, 8'h1D); put('hFFFF2, 8'h04);

            p = 'h00410;
            put(p++, 8'hBC); put(p++, 8'h00); put(p++, 8'h80); // 0420 MOV SP,8000h
            put(p++, 8'hBB); put(p++, 8'h00); put(p++, 8'h00); // 0423 MOV BX,0
            put(p++, 8'hB9); put(p++, 8'h64); put(p++, 8'h00); // 0426 MOV CX,100
            put(p++, 8'hFB);                                   // 0429 STI
            // 042A CALL 0440 (rel16 = 0440-042D = 0013)
            put(p++, 8'hE8); put(p++, 8'h13); put(p++, 8'h00);
            put(p++, 8'h49);                                   // 042D DEC CX
            put(p++, 8'h75); put(p++, 8'hFA);                  // 042E JNZ -6 -> 042A
            put(p++, 8'hFA);                                   // 0430 CLI
            put(p++, 8'hF4);                                   // 0431 HLT

            // Outer at 0440: CALL Inner (rel16 = 0450-0443 = 000D), then RET
            p = 'h00430;
            put(p++, 8'hE8); put(p++, 8'h0D); put(p++, 8'h00);
            put(p++, 8'hC3);

            // Inner at 0450: ADD BX,CX ; RET
            p = 'h00440;
            put(p++, 8'h01); put(p++, 8'hCB);
            put(p++, 8'hC3);

            // Handler A at 0500: STI ; INC DI ; a short delay loop ; IRET.
            //
            // THE STI IS THE POINT. An interrupt is entered with IF clear, so
            // without it a second request simply waits and nothing nests. With
            // it there is a window, several instructions wide, in which the
            // machine is inside one handler and can take another -- which is
            // what a keypress during the timer tick does on the real board,
            // and what a timer-only workload never produces.
            //
            // The delay loop widens that window so the sweep can land a second
            // request inside it, and is itself a hot backward conditional
            // branch, so the nesting happens around a branch being predicted.
            p = 'h004F0;
            put(p++, 8'hFB);                                   // 0500 STI
            put(p++, 8'h47);                                   // 0501 INC DI
            put(p++, 8'h51);                                   // 0502 PUSH CX
            put(p++, 8'hB9); put(p++, 8'h08); put(p++, 8'h00); // 0503 MOV CX,8
            put(p++, 8'h49);                                   // 0506 DEC CX
            put(p++, 8'h75); put(p++, 8'hFD);                  // 0507 JNZ -3 -> 0506
            put(p++, 8'h59);                                   // 0509 POP CX
            put(p++, 8'hCF);                                   // 050A IRET

            // Handler B at 0520: INC SI ; IRET. Touches nothing else.
            p = 'h00510;
            put(p++, 8'h46); put(p++, 8'hCF);
        end
    endtask

    task put(input int a, input byte b); begin mem[a] = b; end endtask

    // Run the program once, optionally firing one IRQ `at` cycles after reset
    // is released. at < 0 means no interrupt at all.
    // Run the program once. `at` fires IRQ 40h that many cycles after reset is
    // released; `at2`, if >= 0, fires IRQ 41h at that cycle, which is how the
    // nested case is produced. Either may be negative for "do not fire".
    task automatic run_once(input int at, input int at2,
                            output int got_bx, output int got_cx,
                            output int got_di, output int got_si,
                            output int got_sp, output bit did_halt);
        int c;
        bit want_a, want_b;
        begin
            rst_n = 0;
            intr_req = 1'b0;
            intr_type = 8'h40;
            load_program();
            repeat (4) @(negedge clk);
            rst_n = 1;

            c = 0;
            did_halt = 0;
            want_a = (at  >= 0);
            want_b = (at2 >= 0);
            while (c < 60000 && !halted) begin
                @(negedge clk);
                c++;
                // One request line, two sources: raise A first, then B once A
                // has been acknowledged, so the second lands while the first
                // handler is running rather than replacing it.
                if (want_a && c == at) begin
                    intr_type = 8'h40; intr_req = 1'b1;
                end else if (want_b && c == at2 && !intr_req) begin
                    intr_type = 8'h41; intr_req = 1'b1;
                end
                if (intr_req && intr_ack) begin
                    intr_req = 1'b0;
                    if (intr_type == 8'h40) want_a = 1'b0; else want_b = 1'b0;
                end
            end
            did_halt = halted;
            intr_req = 1'b0;
            got_bx = `BX; got_cx = `CX; got_di = `DI; got_si = `SI; got_sp = `SP;
        end
    endtask

    int bx, cx, di, si, sp;
    bit hlt;
    int taken_count = 0, nested_count = 0;

    initial begin
        // ---- the reference run, with no interrupt at all ----
        run_once(-1, -1, bx, cx, di, si, sp, hlt);
        chk("reference: halted",        hlt, 1'b1);
        chk("reference: BX",            bx,  EXPECT_BX);
        chk("reference: CX",            cx,  0);
        chk("reference: SP balanced",   sp,  16'h8000);
        chk("reference: no IRQ ran",    di,  0);

        if (errors != 0) begin
            $display("reference run is already wrong -- sweep would be meaningless");
        end else begin
            // ---- the sweep ----
            // Every cycle offset across a stretch of the loop. The loop body
            // is short, so a few hundred offsets covers the branch being
            // predicted, resolved and recovered many times over, at every
            // phase relative to the interrupt.
            for (int at = 40; at <= 640; at++) begin
                run_once(at, -1, bx, cx, di, si, sp, hlt);
                if (di != 0) taken_count++;
                if (!hlt || bx != EXPECT_BX || cx != 0 || sp != 16'h8000) begin
                    checks++;
                    errors++;
                    $display("FAIL IRQ at cycle %0d: halted=%0b BX=%04h (exp %04h) CX=%04h SP=%04h DI=%04h",
                             at, hlt, bx, EXPECT_BX, cx, sp, di);
                    if (errors > 8) begin
                        $display("  (stopping after 8 failures)");
                        break;
                    end
                end
            end
            checks++;   // the sweep itself counts as one check when it passes
            $display("  swept 601 interrupt offsets; %0d of them were taken",
                     taken_count);

            // ---- the nested sweep ----
            // A second request lands a few cycles after the first, inside
            // handler A's STI window, so the machine takes an interrupt while
            // already in one. Several separations are tried because the window
            // moves with the first interrupt's own timing.
            for (int at = 60; at <= 360; at++) begin
                for (int gap = 4; gap <= 16; gap += 4) begin
                    run_once(at, at + gap, bx, cx, di, si, sp, hlt);
                    if (si != 0) nested_count++;
                    if (!hlt || bx != EXPECT_BX || cx != 0 || sp != 16'h8000) begin
                        checks++;
                        errors++;
                        $display("FAIL nested IRQ at %0d + %0d: halted=%0b BX=%04h (exp %04h) CX=%04h SP=%04h DI=%04h SI=%04h",
                                 at, gap, hlt, bx, EXPECT_BX, cx, sp, di, si);
                        if (errors > 8) begin
                            $display("  (stopping after 8 failures)");
                            break;
                        end
                    end
                end
                if (errors > 8) break;
            end
            checks++;
            $display("  swept nested pairs; %0d of them actually nested",
                     nested_count);
            if (nested_count == 0) begin
                $display("FAIL no nested interrupt was ever delivered");
                errors++;
            end
            // If no interrupt was ever actually delivered the sweep proved
            // nothing, so that is a failure in its own right.
            if (taken_count == 0) begin
                $display("FAIL the sweep never delivered an interrupt");
                errors++;
            end
        end

        $display("");
        $display("==================================");
        $display(" checks: %0d   failures: %0d", checks, errors);
        $display("==================================");
        if (errors == 0) $display("ALL TESTS PASSED");
        $finish;
    end

    initial begin
        #900000000;
        $display("FAIL global timeout (IP=%04h)", dbg_ip);
        $display(" checks: %0d   failures: %0d", checks, errors + 1);
        $finish;
    end

endmodule
