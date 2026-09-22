// ---------------------------------------------------------------------------
// decode — opcode + ModR/M -> control word. Purely combinational.
//
// Hierarchy: cpu_top -> eu -> decode
// Reference: learnings/03-instruction-set.md, learnings/01-programming-model.md
// Testbench: sim/tb_decode.sv
//
// Instruction format: [prefixes] opcode [ModR/M] [displacement] [immediate].
// There is no SIB byte on 8086/80186 -- that is a 386 addition.
//
// Anything not implemented decodes as C_ILLEGAL, which the sequencer turns
// into the 80186's illegal-instruction trap. That is deliberate: an
// unimplemented opcode must fail loudly rather than fall through to whatever
// the default control word happens to do. Silently executing the wrong
// instruction is far harder to debug than stopping on the spot.
//
// PREFIXES are not decoded here. They are recognised directly from the fetched
// byte inside execUnit's fetch loop, because a prefix must never become the
// opcode this module sees.
//
// NOT YET DECODED (all fall through to C_ILLEGAL): the indirect far forms of
// CALL and JMP (FF /3 and FF /5); PUSH/POP of segment registers; LES/LDS;
// XLAT; the BCD adjust instructions; and the remaining 80186 additions
// (PUSHA/POPA/ENTER/LEAVE/BOUND/IMUL-imm). The opcode space for each is left
// unclaimed so adding them later is additive.
// ---------------------------------------------------------------------------

`timescale 1ns/1ns
module decode
    import cpu_pkg::*;
(
    input  logic [7:0]  opcode,
    input  logic [7:0]  modrm,        // only meaningful when has_modrm

    output logic [5:0]  iclass,
    output logic        has_modrm,
    output logic        word_op,      // w bit: 1 = 16-bit operation
    output logic        dir_to_reg,   // d bit: 1 = reg <- r/m, 0 = r/m <- reg
    output logic [5:0]  alu_op,
    output logic [2:0]  imm_bytes,    // immediate length in bytes: 0,1,2,3,4
    output logic        imm_sext,     // sign-extend an 8-bit immediate to 16
    output logic [2:0]  op_reg,       // register encoded in the opcode byte
    output logic [3:0]  cond,         // condition selector for Jcc / LOOP

    // ModR/M breakdown (valid when has_modrm)
    output logic [1:0]  mod_field,
    output logic [2:0]  reg_field,
    output logic [2:0]  rm_field,
    output logic [1:0]  disp_bytes,
    output logic        rm_is_mem
);

    assign mod_field = modrm[7:6];
    assign reg_field = modrm[5:3];
    assign rm_field  = modrm[2:0];
    assign rm_is_mem = (modrm[7:6] != 2'b11);

    always_comb begin
        case (modrm[7:6])
            2'b00:   disp_bytes = (modrm[2:0] == 3'b110) ? 2'd2 : 2'd0; // direct address
            2'b01:   disp_bytes = 2'd1;                                 // sign-extended
            2'b10:   disp_bytes = 2'd2;
            default: disp_bytes = 2'd0;                                 // register operand
        endcase
    end

    // ALU selector carried in opcode[5:3] for the 00-3F block and in the
    // ModR/M reg field for group 1.
    function automatic logic [5:0] alu_from_sel (input logic [2:0] sel);
        case (sel)
            3'd0:    alu_from_sel = ALU_ADD;
            3'd1:    alu_from_sel = ALU_OR;
            3'd2:    alu_from_sel = ALU_ADC;
            3'd3:    alu_from_sel = ALU_SBB;
            3'd4:    alu_from_sel = ALU_AND;
            3'd5:    alu_from_sel = ALU_SUB;
            3'd6:    alu_from_sel = ALU_XOR;
            default: alu_from_sel = ALU_CMP;
        endcase
    endfunction

    // Shift/rotate selector from the ModR/M reg field. 110 is SAL, which is
    // the same operation as SHL.
    function automatic logic [5:0] shift_from_sel (input logic [2:0] sel);
        case (sel)
            3'd0:    shift_from_sel = ALU_ROL;
            3'd1:    shift_from_sel = ALU_ROR;
            3'd2:    shift_from_sel = ALU_RCL;
            3'd3:    shift_from_sel = ALU_RCR;
            3'd4:    shift_from_sel = ALU_SHL;
            3'd5:    shift_from_sel = ALU_SHR;
            3'd6:    shift_from_sel = ALU_SHL;
            default: shift_from_sel = ALU_SAR;
        endcase
    endfunction

    always_comb begin
        iclass     = C_ILLEGAL;
        has_modrm  = 1'b0;
        word_op    = opcode[0];
        dir_to_reg = opcode[1];
        alu_op     = ALU_ADD;
        imm_bytes  = 3'd0;
        imm_sext   = 1'b0;
        op_reg     = opcode[2:0];
        cond       = opcode[3:0];

        casez (opcode)
            // ---- the 00-3F ALU block ----
            // Within each group of 8, the low three opcode bits select the
            // form: 0-3 are the ModR/M forms, 4-5 are accumulator+immediate,
            // and 6-7 are PUSH/POP of a segment register (not implemented, so
            // they stay C_ILLEGAL).
            8'b00??_????: begin
                alu_op = alu_from_sel(opcode[5:3]);
                if (opcode[2:0] <= 3'd3) begin
                    iclass    = C_ALU_RM;
                    has_modrm = 1'b1;
                end else if (opcode[2:0] <= 3'd5) begin
                    iclass    = C_ALU_ACC;
                    word_op   = opcode[0];
                    imm_bytes = opcode[0] ? 3'd2 : 3'd1;
                end else if (opcode[2:0] == 3'd6) begin
                    // 06/0E/16/1E: PUSH ES/CS/SS/DS. (26/2E/36/3E are the
                    // segment-override prefixes and never reach decode.)
                    iclass     = C_SREG_STK;
                    word_op    = 1'b1;
                    dir_to_reg = 1'b0;
                    op_reg     = {1'b0, opcode[4:3]};
                end else if (opcode[5]) begin
                    // 27 DAA, 2F DAS, 37 AAA, 3F AAS all sit at r/m = 7 in the
                    // upper half of this block.
                    iclass = C_BCD;
                    cond   = {1'b0, opcode[5:3]};
                end else if (opcode[4:3] != 2'b01) begin
                    // 07/17/1F: POP ES/SS/DS. 0F would be POP CS, which the
                    // 80186 turned into an illegal-instruction trap.
                    iclass     = C_SREG_STK;
                    word_op    = 1'b1;
                    dir_to_reg = 1'b1;
                    op_reg     = {1'b0, opcode[4:3]};
                end
            end

            // ---- 80186 additions: PUSHA/POPA/BOUND/PUSH imm/IMUL imm ----
            8'b0110_0000: begin iclass = C_PUSHA; word_op = 1'b1; end
            8'b0110_0001: begin iclass = C_POPA;  word_op = 1'b1; end
            8'b0110_0010: begin iclass = C_BOUND; has_modrm = 1'b1; word_op = 1'b1; end
            // 68 takes a word, 6A a sign-extended byte.
            8'b0110_1000: begin iclass = C_PUSH_IMM; word_op = 1'b1; imm_bytes = 3'd2; end
            8'b0110_1010: begin iclass = C_PUSH_IMM; word_op = 1'b1; imm_bytes = 3'd1;
                                imm_sext = 1'b1; end
            // 69/6B: three-operand signed multiply by an immediate.
            8'b0110_1001: begin iclass = C_IMUL_IMM; has_modrm = 1'b1; word_op = 1'b1;
                                alu_op = ALU_IMUL; imm_bytes = 3'd2; end
            8'b0110_1011: begin iclass = C_IMUL_IMM; has_modrm = 1'b1; word_op = 1'b1;
                                alu_op = ALU_IMUL; imm_bytes = 3'd1; imm_sext = 1'b1; end

            // ---- INC/DEC reg ----
            8'b0100_????: begin
                iclass  = C_INCDEC_R;
                word_op = 1'b1;                 // always 16-bit
                op_reg  = opcode[2:0];
                alu_op  = opcode[3] ? ALU_DEC : ALU_INC;
            end

            // ---- PUSH/POP reg ----
            8'b0101_0???: begin iclass = C_PUSH_R; word_op = 1'b1; op_reg = opcode[2:0]; end
            8'b0101_1???: begin iclass = C_POP_R;  word_op = 1'b1; op_reg = opcode[2:0]; end

            // ---- Jcc short ----
            8'b0111_????: begin
                iclass    = C_JCC;
                imm_bytes = 3'd1;
                imm_sext  = 1'b1;
                cond      = opcode[3:0];
            end

            // ---- group 1: ALU r/m,imm ----
            8'b1000_00??: begin
                if (opcode[1:0] != 2'b10) begin  // 82 is an undocumented alias
                    iclass    = C_GRP1;
                    has_modrm = 1'b1;
                    word_op   = opcode[0];
                    alu_op    = alu_from_sel(modrm[5:3]);
                    // 81 takes a full word, 80 and 83 take one byte; 83
                    // sign-extends that byte to 16 bits.
                    imm_bytes = (opcode[1:0] == 2'b01) ? 3'd2 : 3'd1;
                    imm_sext  = (opcode[1:0] == 2'b11);
                end
            end

            // ---- TEST r/m,reg and XCHG r/m,reg ----
            8'b1000_010?: begin iclass = C_TEST_RM; has_modrm = 1'b1; alu_op = ALU_TEST; end
            8'b1000_011?: begin iclass = C_XCHG_RM; has_modrm = 1'b1; end

            // ---- MOV r/m,reg ----
            8'b1000_10??: begin iclass = C_MOV_RM; has_modrm = 1'b1; end

            // ---- MOV to/from a segment register ----
            // 8C stores a segment register into r/m, 8E loads one from r/m.
            // opcode[1] separates them, which is the same d-bit position the
            // rest of the MOV family uses.
            8'b1000_11?0: begin
                iclass    = C_MOV_SREG;
                has_modrm = 1'b1;
                word_op   = 1'b1;           // always 16-bit
            end

            // ---- LEA ----
            8'b1000_1101: begin iclass = C_LEA; has_modrm = 1'b1; word_op = 1'b1; end

            // ---- POP r/m ----
            8'b1000_1111: begin iclass = C_POP_RM; has_modrm = 1'b1; word_op = 1'b1; end

            // ---- sign extension, flag transfers, WAIT ----
            8'b1001_1000: begin iclass = C_SIGNEXT; cond = 4'd0; end   // CBW
            8'b1001_1001: begin iclass = C_SIGNEXT; cond = 4'd1; end   // CWD
            8'b1001_1011: iclass = C_NOP;                              // WAIT: no coprocessor
            8'b1001_1100: begin iclass = C_FLAGSTK; dir_to_reg = 1'b0; word_op = 1'b1; end // PUSHF
            8'b1001_1101: begin iclass = C_FLAGSTK; dir_to_reg = 1'b1; word_op = 1'b1; end // POPF
            8'b1001_1110: begin iclass = C_AHFLAGS; cond = 4'd0; end   // SAHF
            8'b1001_1111: begin iclass = C_AHFLAGS; cond = 4'd1; end   // LAHF

            // ---- NOP (which is XCHG AX,AX) and XCHG AX,reg ----
            8'b1001_0000: iclass = C_NOP;
            8'b1001_0???: begin iclass = C_XCHG_R; word_op = 1'b1; op_reg = opcode[2:0]; end

            // ---- MOV acc,[addr] / [addr],acc ----
            8'b1010_00??: begin
                iclass     = C_MOV_ACC;
                word_op    = opcode[0];
                dir_to_reg = ~opcode[1];        // A0/A1 load acc, A2/A3 store acc
                imm_bytes  = 3'd2;              // the direct address
            end

            // ---- string operations ----
            // No ModR/M and no immediate: the operands are always DS:SI and
            // ES:DI, with the destination segment fixed at ES.
            8'b1010_010?: begin iclass = C_MOVS; word_op = opcode[0]; end
            8'b1010_011?: begin iclass = C_CMPS; word_op = opcode[0]; alu_op = ALU_CMP; end
            8'b1010_101?: begin iclass = C_STOS; word_op = opcode[0]; end
            8'b1010_110?: begin iclass = C_LODS; word_op = opcode[0]; end
            8'b1010_111?: begin iclass = C_SCAS; word_op = opcode[0]; alu_op = ALU_CMP; end
            8'b0110_110?: begin iclass = C_INS;  word_op = opcode[0]; end
            8'b0110_111?: begin iclass = C_OUTS; word_op = opcode[0]; end

            // ---- TEST acc,imm ----
            8'b1010_100?: begin
                iclass    = C_TEST_ACC;
                alu_op    = ALU_TEST;
                imm_bytes = opcode[0] ? 3'd2 : 3'd1;
            end

            // ---- MOV reg,imm ----
            8'b1011_????: begin
                iclass    = C_MOV_IMM_R;
                word_op   = opcode[3];
                op_reg    = opcode[2:0];
                imm_bytes = opcode[3] ? 3'd2 : 3'd1;
            end

            // ---- RET near ----
            8'b1100_0010: begin iclass = C_RET_NEAR; imm_bytes = 3'd2; end
            8'b1100_0011: begin iclass = C_RET_NEAR; imm_bytes = 3'd0; end

            // ---- MOV r/m,imm ----
            8'b1100_011?: begin
                iclass    = C_MOV_RM_I;
                has_modrm = 1'b1;
                word_op   = opcode[0];
                imm_bytes = opcode[0] ? 3'd2 : 3'd1;
            end

            // ---- LES / LDS: load a far pointer into a segment and a register ----
            8'b1100_0100: begin iclass = C_LES_LDS; has_modrm = 1'b1; word_op = 1'b1;
                                op_reg = {1'b0, SR_ES}; end
            8'b1100_0101: begin iclass = C_LES_LDS; has_modrm = 1'b1; word_op = 1'b1;
                                op_reg = {1'b0, SR_DS}; end

            // ---- ENTER / LEAVE (80186) ----
            // ENTER carries a 16-bit frame size followed by an 8-bit nesting
            // level, which is why three immediate bytes are needed.
            8'b1100_1000: begin iclass = C_ENTER; word_op = 1'b1; imm_bytes = 3'd3; end
            8'b1100_1001: begin iclass = C_LEAVE; word_op = 1'b1; end

            // ---- ASCII adjust for multiply/divide ----
            8'b1101_0100: begin iclass = C_ASCII; cond = 4'd0; imm_bytes = 3'd1; end // AAM
            8'b1101_0101: begin iclass = C_ASCII; cond = 4'd1; imm_bytes = 3'd1; end // AAD

            // ---- XLAT ----
            8'b1101_0111: begin iclass = C_XLAT; word_op = 1'b0; end

            // ---- software interrupts and return-from-interrupt ----
            8'b1100_1100: iclass = C_INT3;
            8'b1100_1101: begin iclass = C_INT_IMM; imm_bytes = 3'd1; end
            8'b1100_1110: iclass = C_INTO;
            8'b1100_1111: iclass = C_IRET;

            // ---- shifts/rotates ----
            // C0/C1 take an immediate count (an 80186 addition);
            // D0/D1 shift by 1; D2/D3 shift by CL.
            8'b1100_000?: begin
                iclass    = C_GRP2;
                has_modrm = 1'b1;
                word_op   = opcode[0];
                alu_op    = shift_from_sel(modrm[5:3]);
                imm_bytes = 3'd1;
                cond      = 4'd2;               // count source: immediate
            end
            8'b1101_00??: begin
                iclass    = C_GRP2;
                has_modrm = 1'b1;
                word_op   = opcode[0];
                alu_op    = shift_from_sel(modrm[5:3]);
                cond      = opcode[1] ? 4'd1 : 4'd0;  // 1 = CL, 0 = literal 1
            end

            // ---- LOOP / LOOPZ / LOOPNZ / JCXZ ----
            8'b1110_00??: begin
                iclass    = C_LOOP;
                imm_bytes = 3'd1;
                imm_sext  = 1'b1;
                cond      = {2'b00, opcode[1:0]};
            end

            // ---- IN / OUT ----
            // E4-E7 take an 8-bit port number as an immediate; EC-EF take a
            // 16-bit port in DX, which is the only form that can reach the
            // peripheral control block up at FF00-FFFF.
            8'b1110_01??: begin
                iclass    = opcode[1] ? C_OUT : C_IN;
                word_op   = opcode[0];
                imm_bytes = 3'd1;
                cond      = 4'd0;              // port comes from the immediate
            end
            8'b1110_11??: begin
                iclass  = opcode[1] ? C_OUT : C_IN;
                word_op = opcode[0];
                cond    = 4'd1;                // port comes from DX
            end

            // ---- CALL / JMP near, JMP short ----
            8'b1110_1000: begin iclass = C_CALL_NEAR; imm_bytes = 3'd2; end
            8'b1110_1001: begin iclass = C_JMP_NEAR;  imm_bytes = 3'd2; end
            8'b1110_1011: begin iclass = C_JMP_SHORT; imm_bytes = 3'd1; imm_sext = 1'b1; end

            // ---- far transfers ----
            // EA and 9A carry a full ptr16:16 -- offset then segment -- which
            // is why imm_bytes needs an encoding for four.
            8'b1110_1010: begin iclass = C_JMP_FAR;  imm_bytes = 3'd4; end
            8'b1001_1010: begin iclass = C_CALL_FAR; imm_bytes = 3'd4; end
            8'b1100_1011: begin iclass = C_RETF;     imm_bytes = 3'd0; end
            8'b1100_1010: begin iclass = C_RETF;     imm_bytes = 3'd2; end

            // ---- HLT / CMC ----
            8'b1111_0100: iclass = C_HLT;
            8'b1111_0101: begin iclass = C_FLAGOP; cond = 4'd0; end   // CMC

            // ---- group 3: TEST/NOT/NEG/MUL/IMUL/DIV/IDIV ----
            8'b1111_011?: begin
                iclass    = C_GRP3;
                has_modrm = 1'b1;
                word_op   = opcode[0];
                case (modrm[5:3])
                    3'd0, 3'd1: begin alu_op = ALU_TEST; imm_bytes = opcode[0] ? 3'd2 : 3'd1; end
                    3'd2:       alu_op = ALU_NOT;
                    3'd3:       alu_op = ALU_NEG;
                    3'd4:       alu_op = ALU_MUL;
                    3'd5:       alu_op = ALU_IMUL;
                    3'd6:       alu_op = ALU_DIV;
                    default:    alu_op = ALU_IDIV;
                endcase
            end

            // ---- flag operations ----
            8'b1111_1000: begin iclass = C_FLAGOP; cond = 4'd1; end   // CLC
            8'b1111_1001: begin iclass = C_FLAGOP; cond = 4'd2; end   // STC
            8'b1111_1010: begin iclass = C_FLAGOP; cond = 4'd3; end   // CLI
            8'b1111_1011: begin iclass = C_FLAGOP; cond = 4'd4; end   // STI
            8'b1111_1100: begin iclass = C_FLAGOP; cond = 4'd5; end   // CLD
            8'b1111_1101: begin iclass = C_FLAGOP; cond = 4'd6; end   // STD

            // ---- group 4/5: INC/DEC/CALL/JMP/PUSH r/m ----
            8'b1111_111?: begin
                has_modrm = 1'b1;
                word_op   = opcode[0];
                // FE/FF with reg field 111 is an illegal-instruction trap on
                // the 80186 (the 8086 ignored it) -- see learnings/03.
                // FE/FF with reg field 111 is an illegal-instruction trap on
                // the 80186. FE only permits INC and DEC; the indirect
                // transfer and push forms are word-only.
                if ((modrm[5:3] == 3'b111) ||
                    (!opcode[0] && (modrm[5:3] > 3'b001))) begin
                    iclass = C_ILLEGAL;
                end else begin
                    iclass = C_GRP5;
                    case (modrm[5:3])
                        3'd0:    alu_op = ALU_INC;
                        3'd1:    alu_op = ALU_DEC;
                        default: alu_op = ALU_ADD;   // CALL/JMP/PUSH forms
                    endcase
                    if (modrm[5:3] > 3'b001) word_op = 1'b1;
                end
            end

            default: iclass = C_ILLEGAL;
        endcase
    end

    // Note: 0Fh (POP CS on the 8086), 63-67h and F1h are illegal-instruction
    // traps on the 80186 rather than merely unimplemented. They already land
    // on C_ILLEGAL here because nothing claims them. When interrupt support
    // exists, C_ILLEGAL should raise interrupt type 6 instead of halting, at
    // which point these become correct 80186 behaviour for free.

endmodule
