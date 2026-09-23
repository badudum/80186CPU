`timescale 1ns/1ns
//
// Interrupt test. Exercises, in one program:
//   - software INT n, and IRET returning to the right place
//   - a hardware interrupt request with IF set (taken)
//   - a hardware request with IF clear (correctly ignored)
//   - NMI, which must be taken even with IF clear
//   - the divide-error trap raised by the ALU
//
// LAYOUT NOTE: the interrupt vector table occupies physical 00000-003FF, so
// both the program and its handlers have to live above that. With CS=FFFF,
// physical = FFFF0 + offset (mod 1 MB), so an offset of 0420 lands at 00410 --
// just clear of the table.
//
module tb_interrupt;

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

    int errors = 0, checks = 0;
    task chk(input string nm, input int got, input int exp);
        checks++;
        if (got !== exp) begin
            $display("FAIL %-32s got=%04h exp=%04h", nm, got, exp);
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

    // Write one IVT entry: offset then segment, at physical type*4.
    task vec(input int typ, input int off, input int seg);
        begin
            mem[typ*4+0] = off[7:0];   mem[typ*4+1] = off[15:8];
            mem[typ*4+2] = seg[7:0];   mem[typ*4+3] = seg[15:8];
        end
    endtask

    initial begin
        for (i = 0; i < 'hFFFFF; i++) mem[i] = 8'h00;

        // ---- interrupt vector table ----
        vec('h00, 'h0530, 'hFFFF);   // divide error
        vec('h02, 'h0520, 'hFFFF);   // NMI
        vec('h20, 'h0500, 'hFFFF);   // software INT 20h
        vec('h40, 'h0510, 'hFFFF);   // hardware IRQ, type 40h
        vec('h50, 'h0540, 'hFFFF);   // masked IRQ, must never run

        // ---- reset vector: JMP near to offset 0420 ----
        put('hFFFF0, 8'hE9); put('hFFFF1, 8'h1D); put('hFFFF2, 8'h04);

        // ---- main program at offset 0420 == physical 00410 ----
        p = 'h00410;
        put(p++, 8'hBC); put(p++, 8'h00); put(p++, 8'h80); // 0420 MOV SP,8000h
        put(p++, 8'hBB); put(p++, 8'h00); put(p++, 8'h00); // 0423 MOV BX,0
        put(p++, 8'hB9); put(p++, 8'h00); put(p++, 8'h00); // 0426 MOV CX,0
        put(p++, 8'hBA); put(p++, 8'h00); put(p++, 8'h00); // 0429 MOV DX,0
        put(p++, 8'hFB);                                   // 042C STI
        put(p++, 8'hCD); put(p++, 8'h20);                  // 042D INT 20h
        put(p++, 8'h83); put(p++, 8'hF9); put(p++, 8'h00); // 042F CMP CX,0
        put(p++, 8'h74); put(p++, 8'hFB);                  // 0432 JZ -5 -> 042F
        put(p++, 8'hFA);                                   // 0434 CLI
        put(p++, 8'h83); put(p++, 8'hFA); put(p++, 8'h00); // 0435 CMP DX,0
        put(p++, 8'h74); put(p++, 8'hFB);                  // 0438 JZ -5 -> 0435
        put(p++, 8'hBE); put(p++, 8'h33); put(p++, 8'h33); // 043A MOV SI,3333h
        put(p++, 8'hB8); put(p++, 8'h0A); put(p++, 8'h00); // 043D MOV AX,000Ah
        put(p++, 8'hBD); put(p++, 8'h00); put(p++, 8'h00); // 0440 MOV BP,0
        put(p++, 8'hF7); put(p++, 8'hF5);                  // 0443 DIV BP  (by zero)
        put(p++, 8'hF4);                                   // 0445 HLT

        // ---- handlers, each "MOV reg,imm ; IRET" ----
        p = 'h004F0;                                       // offset 0500
        put(p++, 8'hBB); put(p++, 8'h11); put(p++, 8'h11); put(p++, 8'hCF);
        p = 'h00500;                                       // offset 0510
        put(p++, 8'hB9); put(p++, 8'h22); put(p++, 8'h22); put(p++, 8'hCF);
        p = 'h00510;                                       // offset 0520
        put(p++, 8'hBA); put(p++, 8'h44); put(p++, 8'h44); put(p++, 8'hCF);
        p = 'h00520;                                       // offset 0530
        put(p++, 8'hBF); put(p++, 8'h55); put(p++, 8'h55); put(p++, 8'hCF);
        p = 'h00530;                                       // offset 0540 (never)
        put(p++, 8'hBE); put(p++, 8'hAD); put(p++, 8'hDE); put(p++, 8'hCF);

        repeat (4) @(negedge clk);
        rst_n = 1;

        // ---- 1. software INT 20h ----
        i = 0;
        while (`BX !== 16'h1111 && i < 4000) begin @(negedge clk); i++; end
        chk("software INT ran handler", `BX, 16'h1111);

        // ---- 2. hardware IRQ with IF set ----
        @(negedge clk);
        intr_type = 8'h40;
        intr_req  = 1'b1;
        i = 0;
        while (!intr_ack && i < 4000) begin @(negedge clk); i++; end
        chk("controller saw the acknowledge", intr_ack, 1'b1);
        @(negedge clk);
        intr_req = 1'b0;

        i = 0;
        while (`CX !== 16'h2222 && i < 4000) begin @(negedge clk); i++; end
        chk("hardware IRQ ran handler", `CX, 16'h2222);

        // ---- 3. a request that must stay masked (CPU runs CLI next) ----
        // Wait for the CLI to have executed: the CPU is then spinning on DX.
        repeat (200) @(negedge clk);
        chk("IF is clear after CLI", dbg_flags[9], 1'b0);
        intr_type = 8'h50;
        intr_req  = 1'b1;
        repeat (400) @(negedge clk);
        chk("masked IRQ was not taken", `SI, 16'h0000);

        // ---- 4. NMI, which ignores IF ----
        @(negedge clk); nmi = 1'b1;
        repeat (4) @(negedge clk);
        nmi = 1'b0;

        i = 0;
        while (`DX !== 16'h4444 && i < 4000) begin @(negedge clk); i++; end
        chk("NMI ran despite IF clear", `DX, 16'h4444);

        // ---- 5. divide-error trap ----
        i = 0;
        while (!halted && i < 8000) begin @(negedge clk); i++; end
        chk("cpu halted", halted, 1'b1);
        chk("divide error ran handler", `DI, 16'h5555);

        // the masked request was still asserted the whole time
        chk("masked IRQ never ran", `SI, 16'h3333);
        intr_req = 1'b0;

        // ---- stack discipline ----
        chk("SP balanced after all IRETs", `SP, 16'h8000);
        chk("pushed CS low",  mem['h7FFC], 8'hFF);
        chk("pushed CS high", mem['h7FFD], 8'hFF);

        $display("");
        $display("==================================");
        $display(" checks: %0d   failures: %0d", checks, errors);
        $display("==================================");
        if (errors == 0) $display("ALL TESTS PASSED");
        $finish;
    end

    initial begin
        #2000000;
        $display("FAIL global timeout (IP=%04h int_type=%02h)", dbg_ip, dbg_int_type);
        $display(" checks: %0d   failures: %0d", checks, errors + 1);
        $finish;
    end

endmodule
