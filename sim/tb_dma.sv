`timescale 1ns/1ns
//
// DMA controller test, driven by software on the core.
//
// The program sets up channel 0 through the peripheral control block and
// starts an unsynchronised memory-to-memory block move, then polls until the
// transfer-complete interrupt fires. That exercises the whole path: PCB
// register writes, the DMA's own transfer engine, bus arbitration against the
// CPU, and the completion interrupt through the interrupt controller.
//
// What makes this more than a data-movement check is that the CPU keeps
// executing throughout. The DMA and the CPU are contending for the same bus,
// and the arbiter is only allowed to hand it over between bus cycles -- so if
// a DMA transfer could land in the middle of a CPU cycle, the CPU's own memory
// accesses would come back wrong.
//
module tb_dma;

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

    `define BX dut.u_eu.u_regfile.gpr[3]
    `define SP dut.u_eu.u_regfile.gpr[4]

    // The invariant: ownership of the bus may only change hands while the BIU
    // is between cycles. If it ever flips while a cycle is in progress, that
    // cycle has been split in half.
    int  split_violations;
    logic owns_prev, idle_prev;
    always @(posedge clk) begin
        if (rst_n) begin
            // Only the RISING edge matters: the bus may only be taken while
            // the BIU is between cycles. Releasing it as the final cycle
            // retires is normal.
            if (dut.dma_owns && !owns_prev && !idle_prev) split_violations++;
            owns_prev <= dut.dma_owns;
            idle_prev <= dut.u_biu.bus_idle;
        end else begin
            owns_prev <= 1'b0;
            idle_prev <= 1'b1;
        end
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
        split_violations = 0;
        for (i = 0; i < 'hFFFFF; i++) mem[i] = 8'h00;

        // source block: 16 distinctive words at 2000h
        for (i = 0; i < 16; i++) begin
            put('h2000 + i*2,     8'h10 + i[7:0]);
            put('h2000 + i*2 + 1, 8'hA0 + i[7:0]);
        end

        vec('h0A, 'h0500, 'hF000);   // DMA channel 0 completion -> type 10
        vec('h06, 'h0590, 'hF000);   // illegal opcode -> HLT

        // reset vector: far jump to F000:0100
        put('hFFFF0, 8'hEA);
        put('hFFFF1, 8'h00); put('hFFFF2, 8'h01);
        put('hFFFF3, 8'h00); put('hFFFF4, 8'hF0);

        p = 'hF0100;
        put(p++, 8'hBC); put(p++, 8'h00); put(p++, 8'h80); // MOV SP,8000h
        put(p++, 8'hB8); put(p++, 8'h00); put(p++, 8'h00); // MOV AX,0
        put(p++, 8'h8E); put(p++, 8'hD8);                  // MOV DS,AX
        put(p++, 8'hBB); put(p++, 8'h00); put(p++, 8'h00); // MOV BX,0

        // ---- program DMA channel 0 ----
        // source pointer = 02000h
        put(p++, 8'hBA); put(p++, 8'hC0); put(p++, 8'hFF); // MOV DX,FFC0h
        put(p++, 8'hB8); put(p++, 8'h00); put(p++, 8'h20); // MOV AX,2000h
        put(p++, 8'hEF);                                   // OUT DX,AX
        put(p++, 8'hBA); put(p++, 8'hC2); put(p++, 8'hFF); // MOV DX,FFC2h
        put(p++, 8'hB8); put(p++, 8'h00); put(p++, 8'h00); // MOV AX,0
        put(p++, 8'hEF);                                   // OUT DX,AX
        // destination pointer = 03000h
        put(p++, 8'hBA); put(p++, 8'hC4); put(p++, 8'hFF); // MOV DX,FFC4h
        put(p++, 8'hB8); put(p++, 8'h00); put(p++, 8'h30); // MOV AX,3000h
        put(p++, 8'hEF);                                   // OUT DX,AX
        put(p++, 8'hBA); put(p++, 8'hC6); put(p++, 8'hFF); // MOV DX,FFC6h
        put(p++, 8'hB8); put(p++, 8'h00); put(p++, 8'h00); // MOV AX,0
        put(p++, 8'hEF);                                   // OUT DX,AX
        // transfer count = 16 words
        put(p++, 8'hBA); put(p++, 8'hC8); put(p++, 8'hFF); // MOV DX,FFC8h
        put(p++, 8'hB8); put(p++, 8'h10); put(p++, 8'h00); // MOV AX,0010h
        put(p++, 8'hEF);                                   // OUT DX,AX
        // unmask the DMA 0 source in the interrupt controller, priority 0
        put(p++, 8'hBA); put(p++, 8'h34); put(p++, 8'hFF); // MOV DX,FF34h
        put(p++, 8'hB8); put(p++, 8'h00); put(p++, 8'h00); // MOV AX,0
        put(p++, 8'hEF);                                   // OUT DX,AX
        put(p++, 8'hFB);                                   // STI
        // control: DST M/IO + DST INC + SRC M/IO + SRC INC + TC + INT
        //          + CHG + ST + word  = B706h
        put(p++, 8'hBA); put(p++, 8'hCA); put(p++, 8'hFF); // MOV DX,FFCAh
        put(p++, 8'hB8); put(p++, 8'h07); put(p++, 8'hB7); // MOV AX,B707h
        put(p++, 8'hEF);                                   // OUT DX,AX

        // ---- keep the CPU busy on the same bus while the DMA runs ----
        // Each pass writes a known value and reads it back; if a DMA transfer
        // ever split one of these cycles the readback would be wrong.
        put(p++, 8'hB9); put(p++, 8'h00); put(p++, 8'h01); // MOV CX,0100h
        // loop:
        put(p++, 8'hB8); put(p++, 8'h5A); put(p++, 8'hA5); // MOV AX,A55Ah
        put(p++, 8'hBF); put(p++, 8'h00); put(p++, 8'h40); // MOV DI,4000h
        put(p++, 8'h89); put(p++, 8'h05);                  // MOV [DI],AX
        put(p++, 8'h8B); put(p++, 8'h15);                  // MOV DX,[DI]
        put(p++, 8'h39); put(p++, 8'hC2);                  // CMP DX,AX
        put(p++, 8'h74); put(p++, 8'h01);                  // JZ +1
        put(p++, 8'h43);                                   // INC BX  (corruption seen)
        // Loop back to 0140h. From the next IP (0151h) that is -17, not -15:
        // getting this wrong lands mid-instruction inside the MOV above, and
        // the CPU will faithfully execute the immediate's bytes as opcodes.
        put(p++, 8'hE2); put(p++, 8'hEF);                  // LOOP -17 -> 0140h
        put(p++, 8'hF4);                                   // HLT

        // ---- DMA completion handler ----
        // It must preserve the registers it touches. The interrupt lands at an
        // arbitrary instruction boundary in the loop below, so a handler that
        // clobbered AX or DX would corrupt the comparison the loop is making
        // and look exactly like a split bus cycle.
        p = 'hF0500;
        put(p++, 8'h50);                                   // PUSH AX
        put(p++, 8'h52);                                   // PUSH DX
        put(p++, 8'hBE); put(p++, 8'h99); put(p++, 8'h99); // MOV SI,9999h
        put(p++, 8'hBA); put(p++, 8'h22); put(p++, 8'hFF); // MOV DX,FF22h
        put(p++, 8'hB8); put(p++, 8'h00); put(p++, 8'h00); // MOV AX,0
        put(p++, 8'hEF);                                   // OUT DX,AX   (EOI)
        put(p++, 8'h5A);                                   // POP DX
        put(p++, 8'h58);                                   // POP AX
        put(p++, 8'hCF);                                   // IRET

        put('hF0590, 8'hF4);                               // illegal -> HLT

        repeat (4) @(negedge clk);
        rst_n = 1;

        i = 0;
        while (!halted && i < 400000) begin @(negedge clk); i++; end
        chk("cpu halted", halted, 1'b1);
        chk("no unexpected trap", dbg_int_taken, 1'b0);

        // ---- the block was moved ----
        chk("DMA moved word 0 low",   mem['h3000], 8'h10);
        chk("DMA moved word 0 high",  mem['h3001], 8'hA0);
        chk("DMA moved word 7",       mem['h300E], 8'h17);
        chk("DMA moved word 15 low",  mem['h301E], 8'h1F);
        chk("DMA moved word 15 high", mem['h301F], 8'hAF);
        chk("DMA did not overrun",    mem['h3020], 8'h00);

        // ---- pointers and count ended where they should ----
        chk("source pointer advanced",      dut.u_dma.src[0],   20'h02020);
        chk("destination pointer advanced", dut.u_dma.dst[0],   20'h03020);
        chk("count drained",                dut.u_dma.count[0], 16'h0000);
        chk("TC cleared Start/Stop",        dut.u_dma.ctrl[0][1], 1'b0);

        // ---- the completion interrupt reached a handler ----
        chk("completion interrupt vectored", dut.u_eu.u_regfile.gpr[6], 16'h9999);
        chk("controller supplied type 10",   dbg_int_type, 8'd10);
        chk("EOI cleared in-service",        dut.u_pic.isr_r, 7'h00);

        // ---- the CPU's own accesses were never corrupted ----
        chk("no CPU bus cycle was split", `BX, 16'h0000);
        chk("arbiter never granted mid-cycle", split_violations, 0);

        chk("SP balanced", `SP, 16'h8000);

        $display("");
        $display("==================================");
        $display(" checks: %0d   failures: %0d", checks, errors);
        $display("==================================");
        if (errors == 0) $display("ALL TESTS PASSED");
        $finish;
    end

    initial begin
        #12000000;
        $display("FAIL global timeout (IP=%04h count=%04h BX=%04h)",
                 dbg_ip, dut.u_dma.count[0], `BX);
        $display(" checks: %0d   failures: %0d", checks, errors + 1);
        $finish;
    end

endmodule
