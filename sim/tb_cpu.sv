`timescale 1ns/1ns
//
// Whole-core test: assembles a small program by hand, runs it to HLT, then
// checks the architectural state it should have produced.
//
// Reset puts CS=FFFF and IP=0000, so execution begins at physical FFFF0. The
// program starts with a near JMP to IP=0100, which lands at physical
// FFFF0 + 0100 = 000F0 after the 20-bit address wrap -- the same trick a real
// boot ROM uses to escape the handful of bytes at the top of memory.
//
module tb_cpu;

    logic        clk = 0, rst_n = 0;
    logic [19:0] addr;
    logic [15:0] dout, din;
    logic        rd, wr, io_cycle, bhe, a0, ale;
    logic [2:0]  s;
    logic        ready;
    logic        halted;
    logic [15:0] dbg_ip, dbg_cs, dbg_flags;
    logic [7:0]  dbg_int_type;
    logic        dbg_int_taken;
    logic        nmi = 1'b0;
    logic        intr_req = 1'b0;
    logic [7:0]  intr_type = 8'h00;
    logic        intr_ack;
    logic        int0 = 0, int1 = 0, int2 = 0, int3 = 0;
    logic        drq0 = 0, drq1 = 0;

    cpu_top dut (.*);

    always #5 clk = ~clk;

    // ---- 1 MB byte-addressable memory with real lane behaviour ----
    logic [7:0] mem [0:'hFFFFF];
    logic [19:0] even_a, odd_a;
    assign even_a = {addr[19:1], 1'b0};
    assign odd_a  = {addr[19:1], 1'b1};
    assign din    = {mem[odd_a], mem[even_a]};

    // Intermittent READY rather than a constant 1. Real memory on this board
    // (SDRAM) will not answer in zero wait states, and running the whole
    // program against a stalling bus is a much stronger test of the core than
    // running it against an idealised one -- every bus cycle in every
    // instruction gets exercised with wait states inserted.
    logic [2:0] ready_ctr = 3'd0;
    always @(posedge clk) ready_ctr <= ready_ctr + 3'd1;
    assign ready = (ready_ctr >= 3'd2);

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
            $display("FAIL %-28s got=%04h exp=%04h", nm, got, exp);
            errors++;
        end
    endtask

    // Register-file shortcuts (hierarchical, for observation only).
    `define AX dut.u_eu.u_regfile.gpr[0]
    `define CX dut.u_eu.u_regfile.gpr[1]
    `define DX dut.u_eu.u_regfile.gpr[2]
    `define BX dut.u_eu.u_regfile.gpr[3]
    `define SP dut.u_eu.u_regfile.gpr[4]
    `define BP dut.u_eu.u_regfile.gpr[5]
    `define SI dut.u_eu.u_regfile.gpr[6]
    `define DI dut.u_eu.u_regfile.gpr[7]

    int i;
    int p;

    task put(input int a, input byte b);
        begin mem[a] = b; end
    endtask

    initial begin
        for (i = 0; i < 'hFFFFF; i++) mem[i] = 8'h00;

        // ---- reset vector: JMP near to IP=0100 ----
        // At IP=0, next IP = 3, so rel16 = 0100 - 3 = 00FD.
        put('hFFFF0, 8'hE9); put('hFFFF1, 8'hFD); put('hFFFF2, 8'h00);

        // ---- main program, physical 000F0 == CS:0100 ----
        p = 'h000F0;
        // 0100  MOV AX,1234h
        put(p++, 8'hB8); put(p++, 8'h34); put(p++, 8'h12);
        // 0103  ADD AX,1111h          -> AX = 2345h
        put(p++, 8'h05); put(p++, 8'h11); put(p++, 8'h11);
        // 0106  MOV BX,3000h
        put(p++, 8'hBB); put(p++, 8'h00); put(p++, 8'h30);
        // 0109  MOV [BX],AX           -> mem[3000] = 2345h
        put(p++, 8'h89); put(p++, 8'h07);
        // 010B  MOV CX,[BX]           -> CX = 2345h
        put(p++, 8'h8B); put(p++, 8'h0F);
        // 010D  MOV AX,0000h
        put(p++, 8'hB8); put(p++, 8'h00); put(p++, 8'h00);
        // 0110  INC AX  / INC AX / DEC AX  -> AX = 1
        put(p++, 8'h40); put(p++, 8'h40); put(p++, 8'h48);
        // 0113  MOV DX,0005h
        put(p++, 8'hBA); put(p++, 8'h05); put(p++, 8'h00);
        // 0116  MOV SP,8000h
        put(p++, 8'hBC); put(p++, 8'h00); put(p++, 8'h80);
        // 0119  PUSH DX
        put(p++, 8'h52);
        // 011A  POP SI                -> SI = 5
        put(p++, 8'h5E);
        // 011B  MOV CX,0003h
        put(p++, 8'hB9); put(p++, 8'h03); put(p++, 8'h00);
        // 011E  INC AX                (loop body)
        put(p++, 8'h40);
        // 011F  LOOP 011E             rel8 = -3
        put(p++, 8'hE2); put(p++, 8'hFD);
        // 0121  CMP AX,0004h
        put(p++, 8'h3D); put(p++, 8'h04); put(p++, 8'h00);
        // 0124  JZ +1  -> 0127
        put(p++, 8'h74); put(p++, 8'h01);
        // 0126  HLT                   (failure path: reached only if JZ wrong)
        put(p++, 8'hF4);
        // 0127  NOP
        put(p++, 8'h90);
        // 0128  CALL +4 -> 012F
        put(p++, 8'hE8); put(p++, 8'h04); put(p++, 8'h00);
        // 012B  MOV SI,1111h          (runs after RET)
        put(p++, 8'hBE); put(p++, 8'h11); put(p++, 8'h11);
        // 012E  HLT                   (successful end)
        put(p++, 8'hF4);
        // 012F  INC BX                (subroutine)
        put(p++, 8'h43);
        // 0130  RET
        put(p++, 8'hC3);

        repeat (4) @(negedge clk);
        rst_n = 1;

        // run until the CPU halts
        i = 0;
        while (!halted && i < 20000) begin
            @(negedge clk);
            i++;
        end

        if (!halted) begin
            $display("FAIL cpu did not halt within %0d cycles (IP=%04h)", i, dbg_ip);
            errors++;
        end else begin
            $display("halted after %0d cycles, IP=%04h", i, dbg_ip);
        end

        // dbg_ip is the architectural IP, which is written at retire. HLT
        // halts without retiring, so IP still holds the value the previous
        // instruction left -- which is the address of the HLT itself. The
        // failure-path HLT is at 0126, the successful one at 012E.
        chk("halted at the right HLT", dbg_ip, 16'h012E);

        chk("AX after loop",      `AX, 16'h0004);
        chk("BX incremented",     `BX, 16'h3001);
        chk("CX drained by LOOP", `CX, 16'h0000);
        chk("DX preserved",       `DX, 16'h0005);
        chk("SI from MOV imm",    `SI, 16'h1111);
        chk("SP restored",        `SP, 16'h8000);

        chk("memory store low",   mem['h3000], 8'h45);
        chk("memory store high",  mem['h3001], 8'h23);

        chk("return address on stack lo", mem['h7FFE], 8'h2B);
        chk("return address on stack hi", mem['h7FFF], 8'h01);

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
