`timescale 1ns/1ns
//
// Far transfers and segment-register loads.
//
// This is the test that was impossible before: the whole program lives in the
// ROM region and runs there. Reset starts at FFFF0 with CS=FFFF, which leaves
// only sixteen reachable bytes before the address wraps out of ROM; a FAR jump
// reloads CS and escapes that, which is exactly what a real BIOS does as its
// first instruction.
//
// It also exercises MOV to and from a segment register -- without which
// software cannot address anything outside the reset segments. Here DS is
// loaded with B800 so the program can write the text buffer at B8000, which is
// well beyond what a zero DS can reach.
//
module tb_far;

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
            $display("FAIL %-36s got=%04h exp=%04h", nm, got, exp);
            errors++;
        end
    endtask

    `define AX dut.u_eu.u_regfile.gpr[0]
    `define CX dut.u_eu.u_regfile.gpr[1]
    `define BX dut.u_eu.u_regfile.gpr[3]
    `define SP dut.u_eu.u_regfile.gpr[4]
    `define BP dut.u_eu.u_regfile.gpr[5]
    `define SI dut.u_eu.u_regfile.gpr[6]
    `define DI dut.u_eu.u_regfile.gpr[7]
    `define CS dut.u_eu.u_regfile.sreg[1]
    `define DS dut.u_eu.u_regfile.sreg[3]

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

        vec('h06, 'h0300, 'hF000);     // illegal opcode -> a bare HLT

        // ---- reset vector: FAR jump to F000:0100 ----
        // EA, then the offset word, then the segment word.
        put('hFFFF0, 8'hEA);
        put('hFFFF1, 8'h00); put('hFFFF2, 8'h01);   // offset 0100
        put('hFFFF3, 8'h00); put('hFFFF4, 8'hF0);   // segment F000

        // ---- main program at F000:0100 == physical F0100 ----
        p = 'hF0100;
        put(p++, 8'hBC); put(p++, 8'h00); put(p++, 8'h80); // 0100 MOV SP,8000h
        put(p++, 8'hB8); put(p++, 8'h00); put(p++, 8'hB8); // 0103 MOV AX,B800h
        put(p++, 8'h8E); put(p++, 8'hD8);                  // 0106 MOV DS,AX
        put(p++, 8'hB8); put(p++, 8'h41); put(p++, 8'h07); // 0108 MOV AX,0741h
        put(p++, 8'hBB); put(p++, 8'h00); put(p++, 8'h00); // 010B MOV BX,0
        put(p++, 8'h89); put(p++, 8'h07);                  // 010E MOV [BX],AX -> B8000
        put(p++, 8'h8C); put(p++, 8'hD9);                  // 0110 MOV CX,DS
        put(p++, 8'h9A); put(p++, 8'h00); put(p++, 8'h02);
                         put(p++, 8'h00); put(p++, 8'hF0); // 0112 CALL FAR F000:0200
        put(p++, 8'hBE); put(p++, 8'h34); put(p++, 8'h12); // 0117 MOV SI,1234h
        put(p++, 8'h9A); put(p++, 8'h10); put(p++, 8'h02);
                         put(p++, 8'h00); put(p++, 8'hF0); // 011A CALL FAR F000:0210
        put(p++, 8'hF4);                                   // 011F HLT

        // ---- subroutine at F000:0200, returns with RETF ----
        p = 'hF0200;
        put(p++, 8'hBF); put(p++, 8'h78); put(p++, 8'h56); // MOV DI,5678h
        put(p++, 8'hCB);                                   // RETF

        // ---- subroutine at F000:0210, returns with RETF 4 ----
        p = 'hF0210;
        put(p++, 8'hBD); put(p++, 8'h99); put(p++, 8'h99); // MOV BP,9999h
        put(p++, 8'hCA); put(p++, 8'h04); put(p++, 8'h00); // RETF 4

        put('hF0300, 8'hF4);                               // illegal -> HLT

        repeat (4) @(negedge clk);
        rst_n = 1;

        i = 0;
        while (!halted && i < 40000) begin @(negedge clk); i++; end
        chk("cpu halted", halted, 1'b1);
        chk("halted at the intended HLT", dbg_ip, 16'h011F);

        // ---- the far jump out of the reset vector ----
        chk("CS loaded by the far jump", `CS, 16'hF000);

        // ---- segment register load and store ----
        chk("DS loaded by MOV DS,AX", `DS, 16'hB800);
        chk("MOV CX,DS read it back", `CX, 16'hB800);

        // ---- a write through the newly loaded DS ----
        // BX=0 with DS=B800 addresses physical B8000, far outside anything a
        // zero DS could reach.
        chk("write through DS, low byte",  mem['hB8000], 8'h41);
        chk("write through DS, high byte", mem['hB8001], 8'h07);

        // ---- far call and RETF ----
        chk("far call reached its target", `DI, 16'h5678);
        chk("RETF returned to the caller", `SI, 16'h1234);
        chk("second far call ran",         `BP, 16'h9999);

        // CALL FAR pushes CS then IP, so IP sits at the lower address. Both
        // calls use the same stack slots, so what survives is the second
        // call's return address (011F), not the first's.
        chk("pushed return IP low",  mem['h7FFC], 8'h1F);
        chk("pushed return IP high", mem['h7FFD], 8'h01);
        chk("pushed CS low",         mem['h7FFE], 8'h00);
        chk("pushed CS high",        mem['h7FFF], 8'hF0);

        // First call/RETF balances exactly; the second uses RETF 4, which
        // discards four more bytes of arguments.
        chk("SP reflects RETF imm16", `SP, 16'h8004);

        $display("");
        $display("==================================");
        $display(" checks: %0d   failures: %0d", checks, errors);
        $display("==================================");
        if (errors == 0) $display("ALL TESTS PASSED");
        $finish;
    end

    initial begin
        #2000000;
        $display("FAIL global timeout (IP=%04h CS=%04h int=%02h)",
                 dbg_ip, `CS, dbg_int_type);
        $display(" checks: %0d   failures: %0d", checks, errors + 1);
        $finish;
    end

endmodule
