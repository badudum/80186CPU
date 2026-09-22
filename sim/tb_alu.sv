`timescale 1ns/1ns
module tb_alu;

    logic        clk = 0, rst_n = 0;
    logic [5:0]  alu_op;
    logic        word, start;
    logic [15:0] a, a_hi, b;
    logic        cf_in;
    logic [4:0]  shift_cnt;
    logic [15:0] result, result_hi;
    logic        div_zero, byte_ok, cf, pf, af, zf, sf, of, busy;

    ALU dut (.*);

    always #5 clk = ~clk;

    int errors = 0;
    int checks = 0;

    localparam [5:0] ADD=6'b000000, SUB=6'b000001, CMP=6'b000010, INC=6'b000011,
                     DEC=6'b000100, NEG=6'b000101, AND=6'b000110, OR=6'b000111,
                     XOR=6'b001000, NOT=6'b001001, TEST=6'b001010, SHL=6'b001011,
                     SHR=6'b001100, SAR=6'b001101, ROL=6'b001110, ROR=6'b001111,
                     RCL=6'b010000, RCR=6'b010001, MUL=6'b010010, IMUL=6'b010011,
                     DIV=6'b010100, IDIV=6'b010101, ADC=6'b010110, SBB=6'b010111;

    task chk(input string nm, input int got, input int exp);
        checks++;
        if (got !== exp) begin
            $display("FAIL %-34s got=%0h exp=%0h", nm, got, exp);
            errors++;
        end
    endtask

    // Combinational op: apply inputs, settle, compare
    task comb_op(input [5:0] op, input w, input [15:0] aa, input [15:0] bb,
                 input cfi, input [4:0] cnt);
        begin
            alu_op = op; word = w; a = aa; b = bb; cf_in = cfi; shift_cnt = cnt;
            a_hi = 16'h0000; start = 1'b0;
            #1;
        end
    endtask

    // Multicycle op: pulse start, wait for busy to clear
    task mc_op(input [5:0] op, input w, input [15:0] ahi, input [15:0] aa, input [15:0] bb);
        begin
            @(negedge clk);
            alu_op = op; word = w; a_hi = ahi; a = aa; b = bb;
            cf_in = 1'b0; shift_cnt = 5'd0; start = 1'b1;
            @(negedge clk);
            start = 1'b0;
            while (busy) @(negedge clk);
            #1;
        end
    endtask

    initial begin
        alu_op = ADD; word = 1; a = 0; a_hi = 0; b = 0; cf_in = 0; shift_cnt = 0; start = 0;
        repeat (3) @(negedge clk);
        rst_n = 1;
        @(negedge clk);

        // ---------------- ADD / ADC ----------------
        comb_op(ADD, 1, 16'h7FFF, 16'h0001, 0, 0);
        chk("ADD w 7FFF+1 result", result, 16'h8000);
        chk("ADD w 7FFF+1 cf", cf, 0);
        chk("ADD w 7FFF+1 of", of, 1);   // signed overflow
        chk("ADD w 7FFF+1 sf", sf, 1);
        chk("ADD w 7FFF+1 zf", zf, 0);
        chk("ADD w 7FFF+1 af", af, 1);   // F+1 carries out of bit 3
        chk("ADD w 7FFF+1 pf", pf, 1);   // low byte 00 -> even parity

        comb_op(ADD, 1, 16'hFFFF, 16'h0001, 0, 0);
        chk("ADD w FFFF+1 result", result, 16'h0000);
        chk("ADD w FFFF+1 cf", cf, 1);
        chk("ADD w FFFF+1 zf", zf, 1);
        chk("ADD w FFFF+1 of", of, 0);

        comb_op(ADD, 0, 16'h00FF, 16'h0001, 0, 0);
        chk("ADD b FF+1 result", result[7:0], 8'h00);
        chk("ADD b FF+1 cf", cf, 1);
        chk("ADD b FF+1 zf", zf, 1);
        chk("ADD b FF+1 of", of, 0);
        chk("ADD b FF+1 af", af, 1);

        comb_op(ADD, 0, 16'h007F, 16'h0001, 0, 0);
        chk("ADD b 7F+1 of", of, 1);     // byte signed overflow
        chk("ADD b 7F+1 sf", sf, 1);
        chk("ADD b 7F+1 cf", cf, 0);

        comb_op(ADC, 1, 16'h0001, 16'h0001, 1, 0);
        chk("ADC w 1+1+1 result", result, 16'h0003);

        // ---------------- SUB / SBB / CMP ----------------
        comb_op(SUB, 1, 16'h0000, 16'h0001, 0, 0);
        chk("SUB w 0-1 result", result, 16'hFFFF);
        chk("SUB w 0-1 cf", cf, 1);      // borrow
        chk("SUB w 0-1 sf", sf, 1);
        chk("SUB w 0-1 of", of, 0);
        chk("SUB w 0-1 af", af, 1);

        comb_op(SUB, 1, 16'h8000, 16'h0001, 0, 0);
        chk("SUB w 8000-1 result", result, 16'h7FFF);
        chk("SUB w 8000-1 of", of, 1);   // signed underflow

        comb_op(SBB, 1, 16'h0005, 16'h0002, 1, 0);
        chk("SBB w 5-2-1 result", result, 16'h0002);

        comb_op(CMP, 1, 16'h1234, 16'h1234, 0, 0);
        chk("CMP w equal zf", zf, 1);

        // ---------------- INC / DEC / NEG ----------------
        comb_op(INC, 1, 16'h7FFF, 16'h0000, 0, 0);
        chk("INC w 7FFF result", result, 16'h8000);
        chk("INC w 7FFF of", of, 1);
        chk("INC w 7FFF af", af, 1);

        comb_op(DEC, 1, 16'h8000, 16'h0000, 0, 0);
        chk("DEC w 8000 result", result, 16'h7FFF);
        chk("DEC w 8000 of", of, 1);

        comb_op(NEG, 1, 16'h0001, 16'h0000, 0, 0);
        chk("NEG w 1 result", result, 16'hFFFF);
        chk("NEG w 1 cf", cf, 1);
        comb_op(NEG, 1, 16'h0000, 16'h0000, 0, 0);
        chk("NEG w 0 cf", cf, 0);
        comb_op(NEG, 1, 16'h8000, 16'h0000, 0, 0);
        chk("NEG w 8000 of", of, 1);

        // ---------------- logic ----------------
        comb_op(AND, 1, 16'hF0F0, 16'h0FF0, 0, 0);
        chk("AND w result", result, 16'h00F0);
        chk("AND w cf", cf, 0);
        chk("AND w of", of, 0);

        comb_op(OR, 1, 16'hF000, 16'h000F, 0, 0);
        chk("OR w result", result, 16'hF00F);

        comb_op(XOR, 1, 16'hFFFF, 16'h0F0F, 0, 0);
        chk("XOR w result", result, 16'hF0F0);

        comb_op(NOT, 1, 16'h0F0F, 16'h0000, 0, 0);
        chk("NOT w result", result, 16'hF0F0);
        comb_op(NOT, 0, 16'h000F, 16'h0000, 0, 0);
        chk("NOT b result", result[7:0], 8'hF0);

        // ---------------- shifts ----------------
        comb_op(SHL, 1, 16'h8000, 16'h0000, 0, 1);
        chk("SHL w 8000<<1 result", result, 16'h0000);
        chk("SHL w 8000<<1 cf", cf, 1);
        chk("SHL w 8000<<1 of", of, 1);  // cf ^ msb(result)
        chk("SHL w 8000<<1 zf", zf, 1);

        comb_op(SHL, 1, 16'h0001, 16'h0000, 0, 4);
        chk("SHL w 1<<4 result", result, 16'h0010);
        chk("SHL w 1<<4 cf", cf, 0);

        comb_op(SHL, 0, 16'h0080, 16'h0000, 0, 1);
        chk("SHL b 80<<1 result", result[7:0], 8'h00);
        chk("SHL b 80<<1 cf", cf, 1);

        comb_op(SHR, 1, 16'h0001, 16'h0000, 0, 1);
        chk("SHR w 1>>1 result", result, 16'h0000);
        chk("SHR w 1>>1 cf", cf, 1);

        comb_op(SHR, 1, 16'h8000, 16'h0000, 0, 1);
        chk("SHR w 8000>>1 result", result, 16'h4000);
        chk("SHR w 8000>>1 cf", cf, 0);
        chk("SHR w 8000>>1 of", of, 1);  // of = msb of original

        comb_op(SHR, 0, 16'h0081, 16'h0000, 0, 1);
        chk("SHR b 81>>1 result", result[7:0], 8'h40);
        chk("SHR b 81>>1 cf", cf, 1);

        comb_op(SAR, 1, 16'h8000, 16'h0000, 0, 1);
        chk("SAR w 8000>>1 result", result, 16'hC000);
        chk("SAR w 8000>>1 cf", cf, 0);
        chk("SAR w 8000>>1 of", of, 0);

        comb_op(SAR, 0, 16'h0080, 16'h0000, 0, 1);
        chk("SAR b 80>>1 result", result[7:0], 8'hC0);

        comb_op(SAR, 1, 16'hFFFF, 16'h0000, 0, 4);
        chk("SAR w FFFF>>4 result", result, 16'hFFFF);

        // shift by zero leaves the operand alone
        comb_op(SHL, 1, 16'h1234, 16'h0000, 1, 0);
        chk("SHL w by0 result", result, 16'h1234);
        chk("SHL w by0 cf held", cf, 1);

        // ---------------- rotates ----------------
        comb_op(ROL, 1, 16'h8001, 16'h0000, 0, 1);
        chk("ROL w 8001 result", result, 16'h0003);
        chk("ROL w 8001 cf", cf, 1);

        comb_op(ROL, 0, 16'h0081, 16'h0000, 0, 1);
        chk("ROL b 81 result", result[7:0], 8'h03);
        chk("ROL b 81 cf", cf, 1);

        comb_op(ROR, 1, 16'h0001, 16'h0000, 0, 1);
        chk("ROR w 0001 result", result, 16'h8000);
        chk("ROR w 0001 cf", cf, 1);

        comb_op(ROR, 0, 16'h0001, 16'h0000, 0, 1);
        chk("ROR b 01 result", result[7:0], 8'h80);

        // rotate by full width is identity
        comb_op(ROL, 1, 16'h1234, 16'h0000, 0, 16);
        chk("ROL w by16 identity", result, 16'h1234);
        comb_op(ROL, 0, 16'h0034, 16'h0000, 0, 8);
        chk("ROL b by8 identity", result[7:0], 8'h34);

        comb_op(RCL, 1, 16'h8000, 16'h0000, 0, 1);
        chk("RCL w 8000 cf0 result", result, 16'h0000);
        chk("RCL w 8000 cf0 cf", cf, 1);

        comb_op(RCL, 1, 16'h0000, 16'h0000, 1, 1);
        chk("RCL w 0000 cf1 result", result, 16'h0001);
        chk("RCL w 0000 cf1 cf", cf, 0);

        comb_op(RCR, 1, 16'h0001, 16'h0000, 1, 1);
        chk("RCR w 0001 cf1 result", result, 16'h8000);
        chk("RCR w 0001 cf1 cf", cf, 1);

        comb_op(RCL, 0, 16'h0080, 16'h0000, 0, 1);
        chk("RCL b 80 cf0 result", result[7:0], 8'h00);
        chk("RCL b 80 cf0 cf", cf, 1);

        // 17-step RCL word / 9-step RCL byte are identity
        comb_op(RCL, 1, 16'h1234, 16'h0000, 0, 17);
        chk("RCL w by17 identity", result, 16'h1234);
        comb_op(RCL, 0, 16'h0034, 16'h0000, 0, 9);
        chk("RCL b by9 identity", result[7:0], 8'h34);

        // ---------------- MUL / IMUL ----------------
        mc_op(MUL, 1, 16'h0000, 16'h1234, 16'h0002);
        chk("MUL w 1234*2 lo", result, 16'h2468);
        chk("MUL w 1234*2 hi", result_hi, 16'h0000);
        chk("MUL w 1234*2 cf", cf, 0);
        chk("MUL w 1234*2 of", of, 0);

        mc_op(MUL, 1, 16'h0000, 16'hFFFF, 16'hFFFF);
        chk("MUL w FFFF*FFFF lo", result, 16'h0001);
        chk("MUL w FFFF*FFFF hi", result_hi, 16'hFFFE);
        chk("MUL w FFFF*FFFF cf", cf, 1);

        mc_op(MUL, 0, 16'h0000, 16'h00FF, 16'h00FF);
        chk("MUL b FF*FF result", result, 16'hFE01);
        chk("MUL b FF*FF cf", cf, 1);

        mc_op(MUL, 0, 16'h0000, 16'h0002, 16'h0003);
        chk("MUL b 2*3 result", result, 16'h0006);
        chk("MUL b 2*3 cf", cf, 0);

        mc_op(IMUL, 1, 16'h0000, 16'hFFFF, 16'h0002);
        chk("IMUL w -1*2 lo", result, 16'hFFFE);
        chk("IMUL w -1*2 hi", result_hi, 16'hFFFF);
        chk("IMUL w -1*2 cf", cf, 0);   // hi is sign-extension of lo

        mc_op(IMUL, 1, 16'h0000, 16'h1000, 16'h1000);
        chk("IMUL w 1000*1000 lo", result, 16'h0000);
        chk("IMUL w 1000*1000 hi", result_hi, 16'h0100);
        chk("IMUL w 1000*1000 cf", cf, 1);

        mc_op(IMUL, 0, 16'h0000, 16'h00FF, 16'h0002);
        chk("IMUL b -1*2 result", result, 16'hFFFE);
        chk("IMUL b -1*2 cf", cf, 0);

        // ---------------- DIV / IDIV ----------------
        mc_op(DIV, 1, 16'h0000, 16'h0064, 16'h0007);
        chk("DIV w 100/7 quot", result, 16'd14);
        chk("DIV w 100/7 rem", result_hi, 16'd2);
        chk("DIV w 100/7 dz", div_zero, 0);
        chk("DIV w 100/7 fits", byte_ok, 1);

        mc_op(DIV, 1, 16'h0001, 16'h0000, 16'h0002);
        chk("DIV w 10000h/2 quot", result, 16'h8000);
        chk("DIV w 10000h/2 fits", byte_ok, 1);

        mc_op(DIV, 1, 16'h0000, 16'hFFFF, 16'h0001);
        chk("DIV w FFFF/1 quot", result, 16'hFFFF);
        chk("DIV w FFFF/1 rem", result_hi, 16'h0000);

        mc_op(DIV, 0, 16'h0000, 16'h0064, 16'h0007);
        chk("DIV b 100/7 quot", result[7:0], 8'd14);
        chk("DIV b 100/7 rem", result_hi[7:0], 8'd2);

        // quotient overflow: 10000h/1 does not fit 16 bits
        mc_op(DIV, 1, 16'h0001, 16'h0000, 16'h0001);
        chk("DIV w overflow flagged", byte_ok, 0);

        // byte overflow: AX=1000h / 1 = 1000h does not fit AL
        mc_op(DIV, 0, 16'h0000, 16'h1000, 16'h0001);
        chk("DIV b overflow flagged", byte_ok, 0);

        mc_op(DIV, 1, 16'h0000, 16'h0064, 16'h0000);
        chk("DIV w by zero", div_zero, 1);

        mc_op(DIV, 0, 16'h0000, 16'h0064, 16'h0000);
        chk("DIV b by zero", div_zero, 1);

        // signed divide
        mc_op(IDIV, 1, 16'hFFFF, 16'hFF9C, 16'h0007);   // -100 / 7
        chk("IDIV w -100/7 quot", result, 16'hFFF2);    // -14
        chk("IDIV w -100/7 rem", result_hi, 16'hFFFE);  // -2

        mc_op(IDIV, 1, 16'h0000, 16'h0064, 16'hFFF9);   // 100 / -7
        chk("IDIV w 100/-7 quot", result, 16'hFFF2);    // -14
        chk("IDIV w 100/-7 rem", result_hi, 16'h0002);  // +2

        mc_op(IDIV, 1, 16'hFFFF, 16'hFF9C, 16'hFFF9);   // -100 / -7
        chk("IDIV w -100/-7 quot", result, 16'd14);
        chk("IDIV w -100/-7 rem", result_hi, 16'hFFFE);

        mc_op(IDIV, 0, 16'h0000, 16'hFF9C, 16'h0007);   // AX=-100 / 7 (byte)
        chk("IDIV b -100/7 quot", result[7:0], 8'hF2);  // -14
        chk("IDIV b -100/7 rem", result_hi[7:0], 8'hFE);

        // the 80186 permits the most-negative quotient (8000h / 80h)
        mc_op(IDIV, 0, 16'h0000, 16'hFF80, 16'h0001);   // -128 / 1 = -128
        chk("IDIV b -128/1 quot", result[7:0], 8'h80);
        chk("IDIV b -128/1 permitted", byte_ok, 1);

        mc_op(IDIV, 0, 16'h0000, 16'h0080, 16'h0001);   // +128 / 1 overflows AL
        chk("IDIV b +128/1 overflow", byte_ok, 0);

        // busy must be low for a combinational op
        comb_op(ADD, 1, 16'h0001, 16'h0001, 0, 0);
        chk("busy low for comb op", busy, 0);

        $display("");
        $display("==================================");
        $display(" checks: %0d   failures: %0d", checks, errors);
        $display("==================================");
        if (errors == 0) $display("ALL TESTS PASSED");
        $finish;
    end

endmodule
