`timescale 1ns/1ns
//
// Integrated interrupt controller test, driven entirely by software running on
// the core. The program reaches the controller through real OUT DX,AX / IN
// AX,DX instructions to the peripheral control block at I/O FF00-FFFF, so this
// exercises the whole chain: IN/OUT decode -> PCB address match -> register
// write -> priority resolution -> vector to the CPU -> handler -> EOI -> IRET.
//
// It checks specifically that:
//   - every source is masked at reset, so an asserted INT0 does nothing
//   - unmasking it through its control register lets the interrupt through
//   - the controller supplies the fixed master-mode vector (INT0 = type 12)
//   - the handler can re-mask and EOI through the same register window
//   - reading a PCB register back returns what software wrote
//
module tb_pic;

    logic        clk = 0, rst_n = 0;
    logic [19:0] addr;
    logic [15:0] dout, din;
    logic        rd, wr, io_cycle, bhe, a0, ale;
    logic [2:0]  s;
    logic        ready;
    logic        nmi = 0;
    logic        int0 = 0, int1 = 0, int2 = 0, int3 = 0;
    logic        ext_eoi = 1'b0;
    logic        drq0 = 0, drq1 = 0;
    logic        intr_req = 0;
    logic [7:0]  intr_type = 8'h00;
    logic        intr_ack;
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

    int errors = 0, checks = 0;
    task chk(input string nm, input int got, input int exp);
        checks++;
        if (got !== exp) begin
            $display("FAIL %-34s got=%04h exp=%04h", nm, got, exp);
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

        vec('h0C, 'h0500, 'hFFFF);   // INT0 -> type 12, master-mode fixed vector
        vec('h06, 'h0600, 'hFFFF);   // illegal opcode -> a bare HLT, so a decode
                                     // gap stops visibly instead of running zeros

        // reset vector: JMP near to offset 0420
        put('hFFFF0, 8'hE9); put('hFFFF1, 8'h1D); put('hFFFF2, 8'h04);

        // ---- main program, offset 0420 == physical 00410 ----
        p = 'h00410;
        put(p++, 8'hBC); put(p++, 8'h00); put(p++, 8'h80); // 0420 MOV SP,8000h
        put(p++, 8'hBB); put(p++, 8'h00); put(p++, 8'h00); // 0423 MOV BX,0
        put(p++, 8'hFB);                                   // 0426 STI
        put(p++, 8'hB9); put(p++, 8'h40); put(p++, 8'h00); // 0427 MOV CX,0040h
        put(p++, 8'hE2); put(p++, 8'hFE);                  // 042A LOOP self (delay)
        put(p++, 8'hBA); put(p++, 8'h38); put(p++, 8'hFF); // 042C MOV DX,FF38h
        put(p++, 8'hB8); put(p++, 8'h00); put(p++, 8'h00); // 042F MOV AX,0 (pri 0, unmasked)
        put(p++, 8'hEF);                                   // 0432 OUT DX,AX
        put(p++, 8'h83); put(p++, 8'hFB); put(p++, 8'h00); // 0433 CMP BX,0
        put(p++, 8'h74); put(p++, 8'hFB);                  // 0436 JZ -5 -> 0433
        put(p++, 8'hBA); put(p++, 8'h28); put(p++, 8'hFF); // 0438 MOV DX,FF28h (mask reg)
        put(p++, 8'hED);                                   // 043B IN AX,DX
        put(p++, 8'hA3); put(p++, 8'h00); put(p++, 8'h60); // 043C MOV [6000h],AX
        put(p++, 8'hF4);                                   // 043F HLT

        // ---- INT0 handler, offset 0500 == physical 004F0 ----
        p = 'h004F0;
        put(p++, 8'hBB); put(p++, 8'h11); put(p++, 8'h11); // 0500 MOV BX,1111h
        put(p++, 8'hBA); put(p++, 8'h38); put(p++, 8'hFF); // 0503 MOV DX,FF38h
        put(p++, 8'hB8); put(p++, 8'h08); put(p++, 8'h00); // 0506 MOV AX,8 (mask bit set)
        put(p++, 8'hEF);                                   // 0509 OUT DX,AX
        put(p++, 8'hBA); put(p++, 8'h22); put(p++, 8'hFF); // 050A MOV DX,FF22h (EOI)
        put(p++, 8'hB8); put(p++, 8'h00); put(p++, 8'h00); // 050D MOV AX,0
        put(p++, 8'hEF);                                   // 0510 OUT DX,AX
        put(p++, 8'hCF);                                   // 0511 IRET

        // illegal-opcode handler: just stop
        put('h005F0, 8'hF4);                               // 0600 HLT

        repeat (4) @(negedge clk);
        rst_n = 1;

        // INT0 is asserted from the very start, while every source is still
        // masked by the reset state. Nothing may happen until software unmasks.
        int0 = 1'b1;

        repeat (400) @(negedge clk);
        chk("masked at reset: no handler yet", `BX, 16'h0000);
        chk("no interrupt taken while masked", dbg_int_taken, 1'b0);

        // software unmasks INT0; the controller should now vector the core
        i = 0;
        while (`BX !== 16'h1111 && i < 20000) begin @(negedge clk); i++; end
        chk("handler ran after unmasking", `BX, 16'h1111);
        chk("controller supplied type 12", dbg_int_type, 8'd12);

        // INT0 stays asserted; the handler re-masked it, so it must not retrigger
        i = 0;
        while (!halted && i < 20000) begin @(negedge clk); i++; end
        chk("cpu reached HLT", halted, 1'b1);

        // Mask register read back through the PCB: reset is all seven sources
        // masked (7Fh); software cleared INT0's bit and the handler set it
        // again, so it should read 7Fh once more.
        chk("mask register read back", mem['h6000], 8'h7F);
        chk("mask register high byte",  mem['h6001], 8'h00);

        chk("SP balanced", `SP, 16'h8000);

        // in-service bit was cleared by the EOI the handler issued
        chk("EOI cleared in-service", dut.u_pic.isr_r, 7'h00);

        // ---- an EOI arriving from the 8259 shim ----
        // PC software ends an interrupt by writing port 20h, not this
        // controller's own register. Without somewhere for that write to go,
        // the in-service bit stays set and NOTHING is ever delivered again --
        // the timer included, so a guest's clock stops and it waits for time
        // that never comes. Force a bit in service and clear it the PC way.
        @(negedge clk);
        force dut.u_pic.isr_r = 7'b0001000;
        @(negedge clk);
        release dut.u_pic.isr_r;
        chk("a bit is in service before the shim's EOI",
            dut.u_pic.isr_r, 7'b0001000);

        @(negedge clk);
        ext_eoi = 1'b1;
        @(negedge clk);
        ext_eoi = 1'b0;
        @(negedge clk);
        chk("an EOI from port 20h cleared it", dut.u_pic.isr_r, 7'h00);

        // ...and it must not clear anything when nothing is in service, nor
        // disturb the mask.
        @(negedge clk);
        ext_eoi = 1'b1;
        @(negedge clk);
        ext_eoi = 1'b0;
        @(negedge clk);
        chk("a stray EOI with nothing in service is harmless",
            dut.u_pic.isr_r, 7'h00);
        chk("...and left the mask alone", dut.u_pic.mask_r, 7'h7F);

        $display("");
        $display("==================================");
        $display(" checks: %0d   failures: %0d", checks, errors);
        $display("==================================");
        if (errors == 0) $display("ALL TESTS PASSED");
        $finish;
    end

    initial begin
        #3000000;
        $display("FAIL global timeout (IP=%04h int_type=%02h)", dbg_ip, dbg_int_type);
        $display(" checks: %0d   failures: %0d", checks, errors + 1);
        $finish;
    end

endmodule
