`timescale 1ns/1ns
//
// Instruction length decode, against hand-worked x86 encodings.
//
// Length is the one thing in this module that must never be wrong. A wrong
// length does not corrupt one instruction -- it moves the head of the queue
// to the wrong byte, so every instruction after it decodes from garbage. The
// cases below are therefore chosen to cover each way the length can vary:
// no ModR/M, a ModR/M with each mod encoding, 8- and 16-bit immediates, and
// prefixes stacked ahead of all of it.
//
module tb_decode_len;

    logic [7:0] peek [0:5];
    logic [3:0] count;
    logic       valid;
    logic [2:0] len, n_prefix;
    logic       op_has_modrm, rm_is_mem;
    logic [2:0] op_imm_bytes;
    logic [15:0] disp, imm, imm2;
    logic        rep_z;
    logic [7:0]  op_byte_o, modrm_byte_o;
    logic       has_rep, has_seg_ovr;
    logic [1:0] seg_ovr;

    decode_len dut (.*);

    int errors = 0, checks = 0;

    task automatic put(input int n, input int b0, input int b1, input int b2,
                       input int b3, input int b4, input int b5);
        begin
            peek[0] = b0[7:0]; peek[1] = b1[7:0]; peek[2] = b2[7:0];
            peek[3] = b3[7:0]; peek[4] = b4[7:0]; peek[5] = b5[7:0];
            count   = n[3:0];
            #1;
        end
    endtask

    task automatic want(input string nm, input int n,
                        input int b0, input int b1, input int b2,
                        input int b3, input int b4, input int b5,
                        input int exp_len, input int exp_pfx);
        begin
            put(n, b0, b1, b2, b3, b4, b5);
            checks++;
            if (!valid) begin
                $display("FAIL %-38s not valid with %0d bytes", nm, n);
                errors++;
            end else if (len !== exp_len[2:0] || n_prefix !== exp_pfx[2:0]) begin
                $display("FAIL %-38s len=%0d pfx=%0d  exp len=%0d pfx=%0d",
                         nm, len, n_prefix, exp_len, exp_pfx);
                errors++;
            end
        end
    endtask

    task chk(input string nm, input int got, input int exp);
        begin
            checks++;
            if (got !== exp) begin
                $display("FAIL %-38s got=%0d exp=%0d", nm, got, exp);
                errors++;
            end
        end
    endtask

    initial begin
        // ---- no ModR/M ----
        want("NOP",                     1, 'h90,0,0,0,0,0,            1, 0);
        want("PUSH AX",                 1, 'h50,0,0,0,0,0,            1, 0);
        want("MOV AL,imm8",             2, 'hB0,'h12,0,0,0,0,         2, 0);
        want("MOV AX,imm16",            3, 'hB8,'h34,'h12,0,0,0,      3, 0);
        // The SAME instruction with an immediate whose low byte looks like a
        // ModR/M with mod=01. decode.sv will happily report disp_bytes=1 for
        // it, and an implementation that trusts that field without checking
        // has_modrm returns 4. This encoding is what the real instruction
        // stream hit; the 34/12 one above passes either way.
        want("MOV AX,0040 (imm looks like ModR/M)",
                                        3, 'hB8,'h40,'h00,0,0,0,      3, 0);
        want("MOV AX,0080 (mod=10 shape)",
                                        3, 'hB8,'h80,'h00,0,0,0,      3, 0);
        want("MOV CX,imm16 after a prefix",
                                        4, 'h2E,'hB9,'h40,'h00,0,0,   4, 1);
        want("MOV AL,40 (imm8, no ModR/M)",
                                        2, 'hB0,'h40,0,0,0,0,         2, 0);
        want("JMP rel8",                2, 'hEB,'h10,0,0,0,0,         2, 0);
        want("JMP rel16",               3, 'hE9,'h34,'h12,0,0,0,      3, 0);
        want("INT imm8",                2, 'hCD,'h21,0,0,0,0,         2, 0);
        want("RET",                     1, 'hC3,0,0,0,0,0,            1, 0);

        // ---- ModR/M, each mod encoding ----
        // mod=11 register direct: no displacement.
        want("MOV AX,BX (mod=11)",      2, 'h8B,'hC3,0,0,0,0,         2, 0);
        // mod=00 memory, no displacement.
        want("MOV AX,[BX] (mod=00)",    2, 'h8B,'h07,0,0,0,0,         2, 0);
        // mod=00 rm=110 is the special case: a 16-bit direct address.
        want("MOV AX,[addr16]",         4, 'h8B,'h06,'h34,'h12,0,0,   4, 0);
        // mod=01 one displacement byte.
        want("MOV AX,[BX+d8]",          3, 'h8B,'h47,'h04,0,0,0,      3, 0);
        // mod=10 two displacement bytes.
        want("MOV AX,[BX+d16]",         4, 'h8B,'h87,'h34,'h12,0,0,   4, 0);

        // ---- ModR/M together with an immediate ----
        want("MOV [BX],imm16",          4, 'hC7,'h07,'h34,'h12,0,0,   4, 0);
        want("MOV [BX+d8],imm16",       5, 'hC7,'h47,'h04,'h34,'h12,0,5, 0);
        want("ADD [BX+d16],imm16",      6, 'h81,'h87,'h34,'h12,'h78,'h56, 6, 0);

        // ---- prefixes stack ahead of everything ----
        want("ES: MOV AX,[BX]",         3, 'h26,'h8B,'h07,0,0,0,      3, 1);
        want("CS: MOV AX,[BX+d16]",     5, 'h2E,'h8B,'h87,'h34,'h12,0,5, 1);
        want("REP MOVSW",               2, 'hF3,'hA5,0,0,0,0,         2, 1);
        want("ES: REP MOVSW",           3, 'h26,'hF3,'hA5,0,0,0,      3, 2);
        want("LOCK ADD [BX],AX",        3, 'hF0,'h01,'h07,0,0,0,      3, 1);

        // ---- the prefix flags come out with the length ----
        put(3, 'h26, 'hF3, 'hA5, 0, 0, 0);
        chk("ES: REP -- rep seen",      has_rep, 1);
        chk("ES: REP -- override seen", has_seg_ovr, 1);
        chk("ES: REP -- segment is ES", seg_ovr, 0);   // SR_ES

        put(2, 'h36, 'h90, 0, 0, 0, 0);
        chk("SS: override seen",        has_seg_ovr, 1);
        chk("SS: segment is SS",        seg_ovr, 2);   // SR_SS
        chk("SS: no rep",               has_rep, 0);

        put(2, 'hF3, 'hA5, 0, 0, 0, 0);                // REP MOVSW
        chk("F3 is REPE",  rep_z, 1);
        chk("opcode past the prefix", op_byte_o, 8'hA5);
        put(2, 'hF2, 'hAE, 0, 0, 0, 0);                // REPNE SCASB
        chk("F2 is REPNE", rep_z, 0);
        put(3, 'h26, 'h8B, 'h07, 0, 0, 0);             // ES: MOV AX,[BX]
        chk("opcode past a segment override", op_byte_o, 8'h8B);
        chk("ModR/M after that opcode", modrm_byte_o, 8'h07);

        // A byte that merely looks like a prefix in the middle of an
        // instruction is not one: the scan must stop at the first non-prefix.
        put(3, 'h8B, 'hF3, 'hA5, 0, 0, 0);             // MOV SI,BX
        chk("F3 after an opcode is not a prefix", n_prefix, 0);
        chk("...and the length is just the MOV",  len, 2);

        // ---- not enough bytes yet ----
        // THIS IS THE CASE THAT PREVENTS DESYNCHRONISATION. A length that is
        // not yet determined must report itself as unknown rather than guess,
        // because a caller acting on a guess moves the queue head to the
        // wrong byte and every instruction after it is garbage.
        put(1, 'h8B, 0, 0, 0, 0, 0);                   // opcode only, ModR/M missing
        chk("ModR/M not yet arrived -> not valid", valid, 0);

        put(2, 'h8B, 'h87, 0, 0, 0, 0);                // needs 2 displacement bytes
        chk("displacement not yet arrived -> not valid", valid, 0);

        put(4, 'hC7, 'h47, 'h04, 'h34, 0, 0);          // immediate half there
        chk("immediate half arrived -> not valid", valid, 0);

        put(5, 'hC7, 'h47, 'h04, 'h34, 'h12, 0);
        chk("...valid once the last byte lands", valid, 1);
        chk("...and the length is five", len, 5);

        put(0, 0, 0, 0, 0, 0, 0);
        chk("empty queue -> not valid", valid, 0);

        // ---- the assembled fields, not just the length ----
        // The sequencer will take these directly, so the byte offsets and
        // the sign extension have to be right as well as the length.
        put(4, 'h8B,'h47,'hFC,0,0,0);           // MOV AX,[BX-4]
        chk("disp8 is sign-extended", disp, 16'hFFFC);
        put(4, 'h8B,'h47,'h04,0,0,0);           // MOV AX,[BX+4]
        chk("positive disp8", disp, 16'h0004);
        put(4, 'h8B,'h87,'h34,'h12,0,0);        // MOV AX,[BX+1234]
        chk("disp16 little-endian", disp, 16'h1234);
        put(3, 'hB8,'h34,'h12,0,0,0);           // MOV AX,1234
        chk("imm16 little-endian", imm, 16'h1234);
        put(2, 'hB0,'h80,0,0,0,0);              // MOV AL,80 -- no sign extend
        chk("imm8 into a byte op is not extended", imm, 16'h0080);
        put(3, 'h83,'hC0,'hFF,0,0,0);           // ADD AX,-1 (sign-extended)
        chk("imm8 sign-extended for a word op", imm, 16'hFFFF);
        put(5, 'hC7,'h47,'h04,'h34,'h12,0);     // MOV [BX+4],1234
        chk("displacement and immediate together: disp", disp, 16'h0004);
        chk("...and the immediate after it", imm, 16'h1234);
        put(6, 'h81,'h87,'h34,'h12,'h78,'h56);  // ADD [BX+1234],5678
        chk("disp16 then imm16: disp", disp, 16'h1234);
        chk("disp16 then imm16: imm",  imm,  16'h5678);
        put(5, 'hEA,'h00,'h10,'h00,'hF0,0);     // JMP FAR F000:1000
        chk("far pointer offset", imm, 16'h1000);
        chk("far pointer segment", imm2, 16'hF000);
        put(2, 'h8B,'hC3,0,0,0,0);              // MOV AX,BX (register form)
        chk("register operand is not memory", rm_is_mem, 0);
        put(2, 'h8B,'h07,0,0,0,0);              // MOV AX,[BX]
        chk("memory operand is memory", rm_is_mem, 1);

        // ---- the encodings the shadow run flagged ----
        $display("  probe: opcode -> has_modrm disp_bytes imm_bytes len");
        put(6, 'hFC,0,0,0,0,0);
        $display("    FC CLD      %0d %0d %0d  len=%0d",
                 op_has_modrm, dut.d_disp_bytes, op_imm_bytes, len);
        put(6, 'h50,0,0,0,0,0);
        $display("    50 PUSH AX  %0d %0d %0d  len=%0d",
                 op_has_modrm, dut.d_disp_bytes, op_imm_bytes, len);
        put(6, 'h58,0,0,0,0,0);
        $display("    58 POP AX   %0d %0d %0d  len=%0d",
                 op_has_modrm, dut.d_disp_bytes, op_imm_bytes, len);
        put(6, 'hAB,0,0,0,0,0);
        $display("    AB STOSW    %0d %0d %0d  len=%0d",
                 op_has_modrm, dut.d_disp_bytes, op_imm_bytes, len);
        put(6, 'hB8,'h40,'h00,0,0,0);
        $display("    B8 MOV AX   %0d %0d %0d  len=%0d",
                 op_has_modrm, dut.d_disp_bytes, op_imm_bytes, len);

        $display("");
        $display("==================================");
        $display(" checks: %0d   failures: %0d", checks, errors);
        $display("==================================");
        if (errors == 0) $display("ALL TESTS PASSED");
        $finish;
    end

endmodule
