`timescale 1ns/1ns
//
// String operations and instruction prefixes.
//
// One program exercises REP MOVSB, REP STOSW, LODSB, REPNE SCASB, REPE CMPSB,
// a backward copy with the direction flag set, a segment-override prefix, and
// finally a long REP MOVSB that the testbench interrupts part-way through.
//
// That last one matters most. A repeated string operation is the only place an
// interrupt may be taken INSIDE an instruction, and on return it has to resume
// exactly where it left off -- which works only if SI, DI and CX are
// architecturally correct at every iteration boundary and the pushed return
// address points at the first prefix rather than the opcode.
//
module tb_string;

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
            $display("FAIL %-38s got=%04h exp=%04h", nm, got, exp);
            errors++;
        end
    endtask

    `define CX dut.u_eu.u_regfile.gpr[1]
    `define SP dut.u_eu.u_regfile.gpr[4]
    `define BP dut.u_eu.u_regfile.gpr[5]
    `define SI dut.u_eu.u_regfile.gpr[6]
    `define DI dut.u_eu.u_regfile.gpr[7]

    // How many times the long REP writes into its destination. A copy is
    // IDEMPOTENT, so a REP that restarts from the beginning after an interrupt
    // leaves exactly the same bytes behind as one that resumes -- the final
    // state cannot tell them apart, and a restart is not a cosmetic
    // difference: with a periodic interrupt a long enough copy would never
    // finish. Counting the writes can tell them apart.
    // Counted on the EDGE of wr, not per cycle: the BIU drives wr through both
    // T2 and T3, so a per-cycle count reads exactly double and looks like the
    // copy ran twice.
    int  long_writes = 0;
    logic prev_wr_l = 1'b0;
    always @(posedge clk) begin
        if (wr && !prev_wr_l && !io_cycle && addr >= 'h7000 && addr < 'h7040)
            long_writes++;
        prev_wr_l <= wr;
    end

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

        // source data: eight distinctive bytes, then a longer run
        put('h1000, 8'h11); put('h1001, 8'h22); put('h1002, 8'h33); put('h1003, 8'h44);
        put('h1004, 8'h55); put('h1005, 8'h66); put('h1006, 8'h77); put('h1007, 8'h88);
        for (i = 8; i < 64; i++) put('h1000 + i, 8'hA0 + i[7:0]);

        vec('h40, 'h0300, 'hF000);     // the interrupt used during the long REP
        vec('h06, 'h0310, 'hF000);     // illegal opcode -> HLT

        // reset vector: far jump to F000:0100
        put('hFFFF0, 8'hEA);
        put('hFFFF1, 8'h00); put('hFFFF2, 8'h01);
        put('hFFFF3, 8'h00); put('hFFFF4, 8'hF0);

        p = 'hF0100;
        put(p++, 8'hBC); put(p++, 8'h00); put(p++, 8'h80); // 0100 MOV SP,8000h
        put(p++, 8'hB8); put(p++, 8'h00); put(p++, 8'h00); // 0103 MOV AX,0
        put(p++, 8'h8E); put(p++, 8'hD8);                  // 0106 MOV DS,AX
        put(p++, 8'h8E); put(p++, 8'hC0);                  // 0108 MOV ES,AX
        put(p++, 8'hFC);                                   // 010A CLD
        // REP MOVSB: copy eight bytes 1000 -> 2000
        put(p++, 8'hBE); put(p++, 8'h00); put(p++, 8'h10); // 010B MOV SI,1000h
        put(p++, 8'hBF); put(p++, 8'h00); put(p++, 8'h20); // 010E MOV DI,2000h
        put(p++, 8'hB9); put(p++, 8'h08); put(p++, 8'h00); // 0111 MOV CX,8
        put(p++, 8'hF3); put(p++, 8'hA4);                  // 0114 REP MOVSB
        // REP STOSW: fill four words at 3000
        put(p++, 8'hB8); put(p++, 8'hAA); put(p++, 8'hAA); // 0116 MOV AX,AAAAh
        put(p++, 8'hBF); put(p++, 8'h00); put(p++, 8'h30); // 0119 MOV DI,3000h
        put(p++, 8'hB9); put(p++, 8'h04); put(p++, 8'h00); // 011C MOV CX,4
        put(p++, 8'hF3); put(p++, 8'hAB);                  // 011F REP STOSW
        // LODSB: AL <- [1000], AH must be left alone
        put(p++, 8'hBE); put(p++, 8'h00); put(p++, 8'h10); // 0121 MOV SI,1000h
        put(p++, 8'hAC);                                   // 0124 LODSB
        put(p++, 8'hA3); put(p++, 8'h00); put(p++, 8'h40); // 0125 MOV [4000h],AX
        // REPNE SCASB: find 55h in the copy at 2000
        put(p++, 8'hBF); put(p++, 8'h00); put(p++, 8'h20); // 0128 MOV DI,2000h
        put(p++, 8'hB9); put(p++, 8'h08); put(p++, 8'h00); // 012B MOV CX,8
        put(p++, 8'hB8); put(p++, 8'h55); put(p++, 8'h00); // 012E MOV AX,0055h
        put(p++, 8'hF2); put(p++, 8'hAE);                  // 0131 REPNE SCASB
        put(p++, 8'h89); put(p++, 8'h3E); put(p++, 8'h10); put(p++, 8'h40); // 0133 MOV [4010h],DI
        put(p++, 8'h89); put(p++, 8'h0E); put(p++, 8'h12); put(p++, 8'h40); // 0137 MOV [4012h],CX
        // REPE CMPSB: the two blocks are identical, so it should run to CX=0
        put(p++, 8'hBE); put(p++, 8'h00); put(p++, 8'h10); // 013B MOV SI,1000h
        put(p++, 8'hBF); put(p++, 8'h00); put(p++, 8'h20); // 013E MOV DI,2000h
        put(p++, 8'hB9); put(p++, 8'h08); put(p++, 8'h00); // 0141 MOV CX,8
        put(p++, 8'hF3); put(p++, 8'hA6);                  // 0144 REPE CMPSB
        put(p++, 8'h89); put(p++, 8'h0E); put(p++, 8'h14); put(p++, 8'h40); // 0146 MOV [4014h],CX
        // backward copy with DF set
        put(p++, 8'hFD);                                   // 014A STD
        put(p++, 8'hBE); put(p++, 8'h07); put(p++, 8'h10); // 014B MOV SI,1007h
        put(p++, 8'hBF); put(p++, 8'h07); put(p++, 8'h50); // 014E MOV DI,5007h
        put(p++, 8'hB9); put(p++, 8'h08); put(p++, 8'h00); // 0151 MOV CX,8
        put(p++, 8'hF3); put(p++, 8'hA4);                  // 0154 REP MOVSB
        put(p++, 8'hFC);                                   // 0156 CLD
        // segment override: ES: MOV [BX],AX writes through ES, not DS
        put(p++, 8'hB8); put(p++, 8'h00); put(p++, 8'h06); // 0157 MOV AX,0600h
        put(p++, 8'h8E); put(p++, 8'hC0);                  // 015A MOV ES,AX
        put(p++, 8'hBB); put(p++, 8'h00); put(p++, 8'h08); // 015C MOV BX,0800h
        put(p++, 8'h26); put(p++, 8'h89); put(p++, 8'h07); // 015F ES: MOV [BX],AX
        put(p++, 8'hB8); put(p++, 8'h00); put(p++, 8'h00); // 0162 MOV AX,0
        put(p++, 8'h8E); put(p++, 8'hC0);                  // 0165 MOV ES,AX
        // INS and OUTS, the 80186's own string forms. The testbench memory
        // answers I/O cycles as well as memory ones, so a port number doubles
        // as an address here and the transfer is observable.
        put(p++, 8'hBE); put(p++, 8'h00); put(p++, 8'h10); // 0167 MOV SI,1000h
        put(p++, 8'hBA); put(p++, 8'h00); put(p++, 8'h90); // 016A MOV DX,9000h
        put(p++, 8'h6E);                                   // 016D OUTSB
        put(p++, 8'hBF); put(p++, 8'h00); put(p++, 8'h91); // 016E MOV DI,9100h
        put(p++, 8'h6C);                                   // 0171 INSB
        // a long REP the testbench will interrupt part-way through
        put(p++, 8'hBE); put(p++, 8'h00); put(p++, 8'h10); // 0172 MOV SI,1000h
        put(p++, 8'hBF); put(p++, 8'h00); put(p++, 8'h70); // 016A MOV DI,7000h
        put(p++, 8'hB9); put(p++, 8'h40); put(p++, 8'h00); // 016D MOV CX,64
        put(p++, 8'hFB);                                   // 0170 STI
        put(p++, 8'hF3); put(p++, 8'hA4);                  // 0171 REP MOVSB
        put(p++, 8'hFA);                                   // 0173 CLI
        put(p++, 8'hF4);                                   // 0174 HLT

        // interrupt handler: count it and return
        put('hF0300, 8'h45);                               // INC BP
        put('hF0301, 8'hCF);                               // IRET
        put('hF0310, 8'hF4);                               // illegal -> HLT

        repeat (4) @(negedge clk);
        rst_n = 1;

        // Interrupt the long REP once it is under way.
        //
        // WATCHES THE ENGINE'S OWN DI, NOT THE ARCHITECTURAL ONE. The string
        // engine keeps SI/DI/CX in its own registers while it loops and
        // commits them to the register file on the way out, so the
        // architectural DI does not move mid-instruction and a trigger
        // watching it would never fire. It would not fail, either -- it would
        // wait out the loop and then interrupt nothing, and every check below
        // about the interrupted copy would pass while testing an
        // uninterrupted one. The engine's live value is what "under way"
        // actually means.
        i = 0;
        while (!((dut.u_eu.u_exec.di_r >= 16'h7004) &&
                 (dut.u_eu.u_exec.di_r <  16'h7030)) && (i < 300000)) begin
            @(negedge clk); i++;
        end
        intr_type = 8'h40;
        intr_req  = 1'b1;
        i = 0;
        while (!intr_ack && i < 20000) begin @(negedge clk); i++; end
        chk("interrupt accepted during REP", intr_ack, 1'b1);
        @(negedge clk);
        intr_req = 1'b0;

        i = 0;
        while (!halted && i < 300000) begin @(negedge clk); i++; end
        chk("cpu halted", halted, 1'b1);
        chk("halted at the intended HLT", dbg_ip, 16'h017F);

        // ---- REP MOVSB ----
        chk("MOVSB byte 0", mem['h2000], 8'h11);
        chk("MOVSB byte 3", mem['h2003], 8'h44);
        chk("MOVSB byte 7", mem['h2007], 8'h88);
        chk("MOVSB did not overrun", mem['h2008], 8'h00);

        // ---- REP STOSW ----
        chk("STOSW word 0 low",  mem['h3000], 8'hAA);
        chk("STOSW word 0 high", mem['h3001], 8'hAA);
        chk("STOSW word 3 high", mem['h3007], 8'hAA);
        chk("STOSW did not overrun", mem['h3008], 8'h00);

        // ---- LODSB: loads AL only, leaves AH untouched ----
        chk("LODSB loaded AL", mem['h4000], 8'h11);
        chk("LODSB left AH alone", mem['h4001], 8'hAA);

        // ---- REPNE SCASB: 55h sits at offset 4, so DI stops at 2005 ----
        chk("SCASB stopped past the match", {mem['h4011], mem['h4010]}, 16'h2005);
        chk("SCASB left CX at 3",           {mem['h4013], mem['h4012]}, 16'h0003);

        // ---- REPE CMPSB over identical blocks runs to completion ----
        chk("CMPSB ran to CX=0", {mem['h4015], mem['h4014]}, 16'h0000);

        // ---- backward copy ----
        chk("backward copy byte 0", mem['h5000], 8'h11);
        chk("backward copy byte 7", mem['h5007], 8'h88);
        chk("backward copy did not run past", mem['h4FFF], 8'h00);

        // ---- segment override ----
        chk("override wrote through ES, low",  mem['h6800], 8'h00);
        chk("override wrote through ES, high", mem['h6801], 8'h06);
        chk("override did not write through DS", mem['h0800], 8'h00);

        // ---- INS / OUTS ----
        chk("OUTSB sent the byte to the port", mem['h9000], 8'h11);
        chk("INSB stored the byte at ES:DI",   mem['h9100], 8'h11);

        // ---- the interrupted REP still completed correctly ----
        chk("interrupt fired during REP", (`BP != 16'h0000), 1'b1);
        chk("long copy byte 0",  mem['h7000], 8'h11);
        chk("long copy byte 4",  mem['h7004], 8'h55);
        chk("long copy byte 8",  mem['h7008], 8'hA8);
        chk("long copy last byte", mem['h703F], 8'hDF);
        chk("long copy did not overrun", mem['h7040], 8'h00);
        chk("CX drained", `CX, 16'h0000);
        chk("SI advanced by the full count", `SI, 16'h1040);
        chk("DI advanced by the full count", `DI, 16'h7040);
        // 64 bytes, written once each. More than that means the interrupt
        // made the instruction start over rather than carry on, which the
        // copied bytes themselves cannot show.
        chk("interrupted REP resumed, did not restart", long_writes, 64);

        chk("SP balanced", `SP, 16'h8000);

        $display("");
        $display("==================================");
        $display(" checks: %0d   failures: %0d", checks, errors);
        $display("==================================");
        if (errors == 0) $display("ALL TESTS PASSED");
        $finish;
    end

    initial begin
        #8000000;
        $display("FAIL global timeout (IP=%04h CX=%04h DI=%04h)", dbg_ip, `CX, `DI);
        $display(" checks: %0d   failures: %0d", checks, errors + 1);
        $finish;
    end

endmodule
