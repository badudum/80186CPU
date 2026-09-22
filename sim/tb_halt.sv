`timescale 1ns/1ns
//
// HLT and its wake-up rules.
//
// A halted 80186 leaves HALT when an interrupt is actually SERVICED, not
// merely requested. That distinction is the whole point of this test:
//
//   1. STI; HLT + a maskable request  -> wakes, runs the handler, resumes
//      at the instruction AFTER the HLT.
//   2. CLI; HLT + a maskable request  -> stays halted indefinitely. This is
//      what makes `CLI; HLT` the standard way to stop a machine dead.
//   3. CLI; HLT + NMI                 -> wakes anyway, because NMI ignores IF.
//
// The return address matters as much as the wake-up. Every x86 pushes the
// address of the instruction FOLLOWING the HLT, so IRET resumes past it. If it
// pushed the HLT's own address instead the CPU would halt again the moment the
// handler returned -- an idle loop would appear to work under a repeating timer
// tick and deadlock under a one-shot. Both handlers here read the pushed IP
// straight off the stack and hand it back for checking.
//
// LAYOUT NOTE: the interrupt vector table occupies physical 00000-003FF, so
// both the program and its handlers live above it. With CS=FFFF, physical =
// FFFF0 + offset (mod 1 MB), so offset 0420 lands at 00410 -- just clear.
//
module tb_halt;

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

    // Counting acknowledges is how the "stayed masked" check is made without
    // the program having to cooperate: a maskable request left asserted across
    // the whole run must be acknowledged exactly once.
    int ack_count = 0;
    always @(posedge clk) if (rst_n && intr_ack) ack_count <= ack_count + 1;

    int errors = 0, checks = 0;
    task chk(input string nm, input int got, input int exp);
        checks++;
        if (got !== exp) begin
            $display("FAIL %-40s got=%04h exp=%04h", nm, got, exp);
            errors++;
        end
    endtask

    `define AX dut.u_eu.u_regfile.gpr[0]
    `define CX dut.u_eu.u_regfile.gpr[1]
    `define DX dut.u_eu.u_regfile.gpr[2]
    `define BX dut.u_eu.u_regfile.gpr[3]
    `define SP dut.u_eu.u_regfile.gpr[4]
    `define SI dut.u_eu.u_regfile.gpr[6]
    `define DI dut.u_eu.u_regfile.gpr[7]

    int i, p;
    task put(input int a, input byte b); begin mem[a] = b; end endtask
    task vec(input int typ, input int off, input int seg);
        begin
            mem[typ*4+0] = off[7:0];   mem[typ*4+1] = off[15:8];
            mem[typ*4+2] = seg[7:0];   mem[typ*4+3] = seg[15:8];
        end
    endtask

    initial begin
        for (i = 0; i < 'hFFFFF; i++) mem[i] = 8'h00;

        vec('h02, 'h0520, 'hFFFF);   // NMI
        vec('h06, 'h0540, 'hFFFF);   // illegal opcode -> HLT, a safety net
        vec('h40, 'h0500, 'hFFFF);   // maskable hardware IRQ

        // reset vector: JMP near to offset 0420
        put('hFFFF0, 8'hE9); put('hFFFF1, 8'h1D); put('hFFFF2, 8'h04);

        // ---- main program at offset 0420 == physical 00410 ----
        p = 'h00410;
        put(p++, 8'hBC); put(p++, 8'h00); put(p++, 8'h80); // 0420 MOV SP,8000h
        put(p++, 8'hBB); put(p++, 8'h00); put(p++, 8'h00); // 0423 MOV BX,0
        put(p++, 8'hB9); put(p++, 8'h00); put(p++, 8'h00); // 0426 MOV CX,0
        put(p++, 8'hBE); put(p++, 8'h00); put(p++, 8'h00); // 0429 MOV SI,0
        put(p++, 8'hBF); put(p++, 8'h00); put(p++, 8'h00); // 042C MOV DI,0
        put(p++, 8'hFB);                                   // 042F STI
        put(p++, 8'hF4);                                   // 0430 HLT   <- #1
        put(p++, 8'hBE); put(p++, 8'h34); put(p++, 8'h12); // 0431 MOV SI,1234h
        put(p++, 8'hFA);                                   // 0434 CLI
        put(p++, 8'hF4);                                   // 0435 HLT   <- #2
        put(p++, 8'hBF); put(p++, 8'h78); put(p++, 8'h56); // 0436 MOV DI,5678h
        put(p++, 8'hF4);                                   // 0439 HLT   <- final

        // ---- maskable handler at offset 0500 == physical 004F0 ----
        // Reads the pushed return IP out of the stack frame and leaves it in
        // AX. After PUSH BP / MOV BP,SP the frame is:
        //   [BP+0] saved BP   [BP+2] return IP   [BP+4] CS   [BP+6] FLAGS
        p = 'h004F0;
        put(p++, 8'h55);                                   // PUSH BP
        put(p++, 8'h89); put(p++, 8'hE5);                  // MOV BP,SP
        put(p++, 8'h8B); put(p++, 8'h46); put(p++, 8'h02); // MOV AX,[BP+2]
        put(p++, 8'hBB); put(p++, 8'h11); put(p++, 8'h11); // MOV BX,1111h
        put(p++, 8'h5D);                                   // POP BP
        put(p++, 8'hCF);                                   // IRET

        // ---- NMI handler at offset 0520 == physical 00510 ----
        p = 'h00510;
        put(p++, 8'h55);                                   // PUSH BP
        put(p++, 8'h89); put(p++, 8'hE5);                  // MOV BP,SP
        put(p++, 8'h8B); put(p++, 8'h56); put(p++, 8'h02); // MOV DX,[BP+2]
        put(p++, 8'hB9); put(p++, 8'h22); put(p++, 8'h22); // MOV CX,2222h
        put(p++, 8'h5D);                                   // POP BP
        put(p++, 8'hCF);                                   // IRET

        put('h00530, 8'hF4);                               // offset 0540: HLT

        repeat (4) @(negedge clk);
        rst_n = 1;

        // ================= 1. wake on a maskable interrupt =================
        i = 0;
        while (!halted && i < 8000) begin @(negedge clk); i++; end
        chk("reached the first HLT", halted, 1'b1);
        chk("IF is set at the first HLT", dbg_flags[9], 1'b1);
        chk("nothing ran before the wake", `SI, 16'h0000);

        @(negedge clk);
        intr_type = 8'h40;
        intr_req  = 1'b1;

        i = 0;
        while (!intr_ack && i < 8000) begin @(negedge clk); i++; end
        chk("halted CPU acknowledged the interrupt", intr_ack, 1'b1);

        // Drop the request on acknowledge, which is what a real controller
        // does on EOI. Holding it would be taken again the instant IRET
        // restored IF, and the CPU would never reach the next instruction --
        // correct behaviour for a level input, but not what is under test.
        @(negedge clk);
        intr_req = 1'b0;

        i = 0;
        while (`BX !== 16'h1111 && i < 8000) begin @(negedge clk); i++; end
        chk("wake ran the handler", `BX, 16'h1111);
        chk("pushed return address is past the HLT", `AX, 16'h0431);

        i = 0;
        while (`SI !== 16'h1234 && i < 8000) begin @(negedge clk); i++; end
        chk("resumed at the instruction after HLT", `SI, 16'h1234);

        // ============== 2. a maskable request must NOT wake it ==============
        // The CPU runs CLI and halts again with the request still asserted.
        i = 0;
        while (!halted && i < 8000) begin @(negedge clk); i++; end
        chk("reached the second HLT", halted, 1'b1);
        chk("IF is clear after CLI", dbg_flags[9], 1'b0);

        // Re-assert, and this time leave it asserted: with IF clear it must be
        // ignored no matter how long it is held.
        @(negedge clk);
        intr_req = 1'b1;
        repeat (2000) @(negedge clk);
        chk("still halted with IF clear", halted, 1'b1);
        chk("masked request was not acknowledged", ack_count, 1);
        chk("nothing past the second HLT ran", `DI, 16'h0000);

        // ===================== 3. NMI wakes it anyway =====================
        // Drop the maskable request first: the NMI handler's IRET restores
        // IF=0, so it would stay masked anyway, but leaving it asserted would
        // make a failure here ambiguous between the two causes.
        @(negedge clk); intr_req = 1'b0;
        @(negedge clk); nmi = 1'b1;
        repeat (4) @(negedge clk);
        nmi = 1'b0;

        i = 0;
        while (`CX !== 16'h2222 && i < 8000) begin @(negedge clk); i++; end
        chk("NMI woke the halted CPU despite IF", `CX, 16'h2222);
        chk("NMI return address is past the HLT", `DX, 16'h0436);

        i = 0;
        while (`DI !== 16'h5678 && i < 8000) begin @(negedge clk); i++; end
        chk("resumed after the NMI wake", `DI, 16'h5678);

        // ===================== final HLT stays halted =====================
        i = 0;
        while (!halted && i < 8000) begin @(negedge clk); i++; end
        chk("reached the final HLT", halted, 1'b1);
        repeat (2000) @(negedge clk);
        chk("final HLT is not disturbed", halted, 1'b1);

        chk("interrupt was taken exactly once", ack_count, 1);
        chk("no unexpected trap", dbg_int_taken, 1'b0);
        chk("SP balanced after both IRETs", `SP, 16'h8000);

        $display("");
        $display("==================================");
        $display(" checks: %0d   failures: %0d", checks, errors);
        $display("==================================");
        if (errors == 0) $display("ALL TESTS PASSED");
        $finish;
    end

    initial begin
        #4000000;
        $display("FAIL global timeout (IP=%04h halted=%b int_type=%02h)",
                 dbg_ip, halted, dbg_int_type);
        $display(" checks: %0d   failures: %0d", checks, errors + 1);
        $finish;
    end

endmodule
