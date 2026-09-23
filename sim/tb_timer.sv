`timescale 1ns/1ns
//
// Timer test, driven by software on the core. This is the path a DOS tick
// needs end to end: program the timer through the PCB, unmask the timer source
// in the interrupt controller, and let repeated expiries vector the CPU into a
// handler that counts them.
//
// Timer 2 is used because it needs no external pins. Its master-mode vector is
// type 19, so the handler goes in IVT slot 19 (physical 4Ch).
//
module tb_timer;

    logic        clk = 0, rst_n = 0;
    logic [19:0] addr;
    logic [15:0] dout, din;
    logic        rd, wr, io_cycle, bhe, a0, ale;
    logic [2:0]  s;
    logic        ready;
    logic        nmi = 0;
    logic        int0 = 0, int1 = 0, int2 = 0, int3 = 0;
    logic        drq0 = 0, drq1 = 0;
    logic        intr_req = 0;
    logic [7:0]  intr_type = 8'h00;
    logic        intr_ack;
    logic        halted;
    logic [15:0] dbg_ip, dbg_cs, dbg_flags;
    logic        ext_eoi = 1'b0;   // the 8259 shim's EOI; unused here
    logic        ext_tick = 1'b0, ext_tick_en = 1'b0;
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

    int errors = 0, checks = 0;
    task chk(input string nm, input int got, input int exp);
        checks++;
        if (got !== exp) begin
            $display("FAIL %-34s got=%04h exp=%04h", nm, got, exp);
            errors++;
        end
    endtask
    task chk_ge(input string nm, input int got, input int minv);
        checks++;
        if (!(got >= minv)) begin
            $display("FAIL %-34s got=%0d expected >= %0d", nm, got, minv);
            errors++;
        end
    endtask

    `define BX dut.u_eu.u_regfile.gpr[3]
    `define SP dut.u_eu.u_regfile.gpr[4]

    int i, p;
    task put(input int a, input byte b); begin mem[a] = b; end endtask
    task vec(input int typ, input int off, input int seg);
        begin
            mem[typ*4+0] = off[7:0];  mem[typ*4+1] = off[15:8];
            mem[typ*4+2] = seg[7:0];  mem[typ*4+3] = seg[15:8];
        end
    endtask

    initial begin
        for (i = 0; i < 'hFFFFF; i++) mem[i] = 8'h00;

        vec(19,    'h0500, 'hFFFF);   // timer 2 interrupt
        vec('h06,  'h0600, 'hFFFF);   // illegal opcode -> HLT, so gaps show up

        put('hFFFF0, 8'hE9); put('hFFFF1, 8'h1D); put('hFFFF2, 8'h04);

        // ---- main program, offset 0420 == physical 00410 ----
        p = 'h00410;
        put(p++, 8'hBC); put(p++, 8'h00); put(p++, 8'h80); // 0420 MOV SP,8000h
        put(p++, 8'hBB); put(p++, 8'h00); put(p++, 8'h00); // 0423 MOV BX,0
        // timer 2 max count A = 0040h
        put(p++, 8'hBA); put(p++, 8'h62); put(p++, 8'hFF); // 0426 MOV DX,FF62h
        put(p++, 8'hB8); put(p++, 8'h40); put(p++, 8'h00); // 0429 MOV AX,0040h
        put(p++, 8'hEF);                                   // 042C OUT DX,AX
        // unmask the timer source in the interrupt controller, priority 0
        put(p++, 8'hBA); put(p++, 8'h32); put(p++, 8'hFF); // 042D MOV DX,FF32h
        put(p++, 8'hB8); put(p++, 8'h00); put(p++, 8'h00); // 0430 MOV AX,0000h
        put(p++, 8'hEF);                                   // 0433 OUT DX,AX
        // enable timer 2: EN | INH | INT | CONT
        put(p++, 8'hBA); put(p++, 8'h66); put(p++, 8'hFF); // 0434 MOV DX,FF66h
        put(p++, 8'hB8); put(p++, 8'h01); put(p++, 8'hE0); // 0437 MOV AX,E001h
        put(p++, 8'hEF);                                   // 043A OUT DX,AX
        put(p++, 8'hFB);                                   // 043B STI
        // wait until three ticks have been counted
        put(p++, 8'h83); put(p++, 8'hFB); put(p++, 8'h03); // 043C CMP BX,3
        put(p++, 8'h7C); put(p++, 8'hFB);                  // 043F JL -5 -> 043C
        put(p++, 8'hF4);                                   // 0441 HLT

        // ---- timer handler, offset 0500 == physical 004F0 ----
        p = 'h004F0;
        put(p++, 8'h43);                                   // 0500 INC BX
        put(p++, 8'hBA); put(p++, 8'h22); put(p++, 8'hFF); // 0501 MOV DX,FF22h
        put(p++, 8'hB8); put(p++, 8'h00); put(p++, 8'h00); // 0504 MOV AX,0
        put(p++, 8'hEF);                                   // 0507 OUT DX,AX   (EOI)
        put(p++, 8'hCF);                                   // 0508 IRET

        put('h005F0, 8'hF4);                               // illegal -> HLT

        repeat (4) @(negedge clk);
        rst_n = 1;

        i = 0;
        while (!halted && i < 60000) begin @(negedge clk); i++; end
        chk("cpu reached HLT", halted, 1'b1);

        chk_ge("timer produced repeated ticks", `BX, 3);
        chk("timer 2 supplied vector 19", dbg_int_type, 8'd19);
        chk("SP balanced after handlers", `SP, 16'h8000);

        // timer state, read through the hierarchy
        chk("timer 2 max count programmed", dut.u_timer.maxa[2], 16'h0040);
        chk("timer 2 still enabled (CONT)", dut.u_timer.ctrl[2][15], 1'b1);
        chk("timer 2 MC flag set",          dut.u_timer.ctrl[2][5],  1'b1);
        chk("INH does not stick",           dut.u_timer.ctrl[2][14], 1'b0);
        chk("timers 0 and 1 left disabled", dut.u_timer.ctrl[0][15], 1'b0);
        chk("EOI cleared in-service",       dut.u_pic.isr_r, 7'h00);

        $display("");
        $display("==================================");
        $display(" checks: %0d   failures: %0d", checks, errors);
        $display("==================================");
        if (errors == 0) $display("ALL TESTS PASSED");
        $finish;
    end

    initial begin
        #4000000;
        $display("FAIL global timeout (IP=%04h BX=%04h)", dbg_ip, `BX);
        $display(" checks: %0d   failures: %0d", checks, errors + 1);
        $finish;
    end

endmodule
