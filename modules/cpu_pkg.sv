// ---------------------------------------------------------------------------
// cpu_pkg — constants shared across the CPU core.
//
// Anything that two modules must agree on numerically lives here. The ALU
// operation codes are the motivating case: decode.sv emits them and ALU.sv
// consumes them, and if the two ever drifted apart the result would be a CPU
// that quietly computes the wrong thing. Same for the FLAGS bit positions,
// which regfile.sv packs and execUnit.sv builds masks against.
//
// Add to this package rather than redeclaring a constant locally.
// ---------------------------------------------------------------------------

`ifndef CPU_PKG_SV
`define CPU_PKG_SV

package cpu_pkg;

    // ---- ALU operations (consumed by ALU.sv) ----
    localparam logic [5:0] ALU_ADD  = 6'b000000;
    localparam logic [5:0] ALU_SUB  = 6'b000001;
    localparam logic [5:0] ALU_CMP  = 6'b000010;
    localparam logic [5:0] ALU_INC  = 6'b000011;
    localparam logic [5:0] ALU_DEC  = 6'b000100;
    localparam logic [5:0] ALU_NEG  = 6'b000101;
    localparam logic [5:0] ALU_AND  = 6'b000110;
    localparam logic [5:0] ALU_OR   = 6'b000111;
    localparam logic [5:0] ALU_XOR  = 6'b001000;
    localparam logic [5:0] ALU_NOT  = 6'b001001;
    localparam logic [5:0] ALU_TEST = 6'b001010;
    localparam logic [5:0] ALU_SHL  = 6'b001011;
    localparam logic [5:0] ALU_SHR  = 6'b001100;
    localparam logic [5:0] ALU_SAR  = 6'b001101;
    localparam logic [5:0] ALU_ROL  = 6'b001110;
    localparam logic [5:0] ALU_ROR  = 6'b001111;
    localparam logic [5:0] ALU_RCL  = 6'b010000;
    localparam logic [5:0] ALU_RCR  = 6'b010001;
    localparam logic [5:0] ALU_MUL  = 6'b010010;
    localparam logic [5:0] ALU_IMUL = 6'b010011;
    localparam logic [5:0] ALU_DIV  = 6'b010100;
    localparam logic [5:0] ALU_IDIV = 6'b010101;
    localparam logic [5:0] ALU_ADC  = 6'b010110;
    localparam logic [5:0] ALU_SBB  = 6'b010111;

    // ---- FLAGS bit positions (architectural, do not renumber) ----
    localparam int F_CF = 0;
    localparam int F_PF = 2;
    localparam int F_AF = 4;
    localparam int F_ZF = 6;
    localparam int F_SF = 7;
    localparam int F_TF = 8;
    localparam int F_IF = 9;
    localparam int F_DF = 10;
    localparam int F_OF = 11;

    // Common flag write masks. x86 does not update every flag on every
    // instruction, so execUnit selects one of these per instruction class.
    localparam logic [15:0] FM_NONE   = 16'h0000;
    localparam logic [15:0] FM_ALL    = 16'h08D5;  // OF SF ZF AF PF CF
    localparam logic [15:0] FM_NOCF   = 16'h08D4;  // same minus CF (INC/DEC)
    localparam logic [15:0] FM_LOGIC  = 16'h08D5;  // CF/OF forced 0 by the ALU
    localparam logic [15:0] FM_ROT    = 16'h0801;  // rotates touch only CF, OF
    localparam logic [15:0] FM_CF     = 16'h0001;
    localparam logic [15:0] FM_IF     = 16'h0200;
    localparam logic [15:0] FM_DF     = 16'h0400;
    localparam logic [15:0] FM_IFTF   = 16'h0300;  // cleared on interrupt entry
    localparam logic [15:0] FM_POPF   = 16'h0FD5;  // every defined flag

    // ---- register encodings (x86 ModR/M reg field) ----
    localparam logic [2:0] R_AX = 3'd0, R_CX = 3'd1, R_DX = 3'd2, R_BX = 3'd3;
    localparam logic [2:0] R_SP = 3'd4, R_BP = 3'd5, R_SI = 3'd6, R_DI = 3'd7;
    localparam logic [2:0] R_AL = 3'd0, R_CL = 3'd1, R_DL = 3'd2, R_BL = 3'd3;
    localparam logic [2:0] R_AH = 3'd4, R_CH = 3'd5, R_DH = 3'd6, R_BH = 3'd7;

    localparam logic [1:0] SR_ES = 2'd0, SR_CS = 2'd1, SR_SS = 2'd2, SR_DS = 2'd3;

    // ---- instruction classes emitted by decode.sv ----
    localparam logic [5:0] C_ILLEGAL   = 6'd0;   // unimplemented or invalid opcode
    localparam logic [5:0] C_ALU_RM    = 6'd1;   // ALU r/m,reg and reg,r/m
    localparam logic [5:0] C_ALU_ACC   = 6'd2;   // ALU acc,imm
    localparam logic [5:0] C_GRP1      = 6'd3;   // 80/81/83: ALU r/m,imm
    localparam logic [5:0] C_MOV_RM    = 6'd4;   // 88-8B
    localparam logic [5:0] C_MOV_IMM_R = 6'd5;   // B0-BF
    localparam logic [5:0] C_MOV_RM_I  = 6'd6;   // C6/C7
    localparam logic [5:0] C_MOV_ACC   = 6'd7;   // A0-A3
    localparam logic [5:0] C_INCDEC_R  = 6'd8;   // 40-4F
    localparam logic [5:0] C_PUSH_R    = 6'd9;   // 50-57
    localparam logic [5:0] C_POP_R     = 6'd10;  // 58-5F
    localparam logic [5:0] C_JCC       = 6'd11;  // 70-7F
    localparam logic [5:0] C_JMP_SHORT = 6'd12;  // EB
    localparam logic [5:0] C_JMP_NEAR  = 6'd13;  // E9
    localparam logic [5:0] C_CALL_NEAR = 6'd14;  // E8
    localparam logic [5:0] C_RET_NEAR  = 6'd15;  // C3 / C2
    localparam logic [5:0] C_LOOP      = 6'd16;  // E0-E3
    localparam logic [5:0] C_GRP2      = 6'd17;  // shifts/rotates
    localparam logic [5:0] C_GRP3      = 6'd18;  // F6/F7
    localparam logic [5:0] C_GRP5      = 6'd19;  // FE/FF
    localparam logic [5:0] C_FLAGOP    = 6'd20;  // CLC/STC/CMC/CLI/STI/CLD/STD
    localparam logic [5:0] C_XCHG_R    = 6'd21;  // 90-97
    localparam logic [5:0] C_NOP       = 6'd22;
    localparam logic [5:0] C_HLT       = 6'd23;
    localparam logic [5:0] C_LEA       = 6'd24;  // 8D
    localparam logic [5:0] C_TEST_RM   = 6'd25;  // 84/85
    localparam logic [5:0] C_TEST_ACC  = 6'd26;  // A8/A9
    localparam logic [5:0] C_XCHG_RM   = 6'd27;  // 86/87
    localparam logic [5:0] C_INT_IMM   = 6'd28;  // CD ib
    localparam logic [5:0] C_INT3      = 6'd29;  // CC
    localparam logic [5:0] C_INTO      = 6'd30;  // CE
    localparam logic [5:0] C_IRET      = 6'd31;  // CF
    localparam logic [5:0] C_IN        = 6'd32;  // E4/E5 (imm8 port), EC/ED (DX)
    localparam logic [5:0] C_OUT       = 6'd33;  // E6/E7 (imm8 port), EE/EF (DX)
    localparam logic [5:0] C_MOV_SREG  = 6'd34;  // 8C (store sreg), 8E (load sreg)
    localparam logic [5:0] C_JMP_FAR   = 6'd35;  // EA ptr16:16
    localparam logic [5:0] C_CALL_FAR  = 6'd36;  // 9A ptr16:16
    localparam logic [5:0] C_RETF      = 6'd37;  // CB, CA imm16
    // String operations. All share one engine in execUnit; these only select
    // which of load/store/compare/adjust it performs per iteration.
    localparam logic [5:0] C_MOVS      = 6'd38;  // A4/A5
    localparam logic [5:0] C_CMPS      = 6'd39;  // A6/A7
    localparam logic [5:0] C_STOS      = 6'd40;  // AA/AB
    localparam logic [5:0] C_LODS      = 6'd41;  // AC/AD
    localparam logic [5:0] C_SCAS      = 6'd42;  // AE/AF
    localparam logic [5:0] C_INS       = 6'd43;  // 6C/6D  (80186 addition)
    localparam logic [5:0] C_OUTS      = 6'd44;  // 6E/6F  (80186 addition)
    localparam logic [5:0] C_SREG_STK  = 6'd45;  // 06/0E/16/1E push, 07/17/1F pop
    localparam logic [5:0] C_FLAGSTK   = 6'd46;  // 9C PUSHF, 9D POPF
    localparam logic [5:0] C_AHFLAGS   = 6'd47;  // 9E SAHF, 9F LAHF
    localparam logic [5:0] C_SIGNEXT   = 6'd48;  // 98 CBW, 99 CWD
    localparam logic [5:0] C_BCD       = 6'd49;  // 27 DAA, 2F DAS, 37 AAA, 3F AAS
    localparam logic [5:0] C_ASCII     = 6'd50;  // D4 AAM, D5 AAD
    localparam logic [5:0] C_XLAT      = 6'd51;  // D7
    localparam logic [5:0] C_LES_LDS   = 6'd52;  // C4 LES, C5 LDS
    localparam logic [5:0] C_POP_RM    = 6'd53;  // 8F /0
    localparam logic [5:0] C_PUSH_IMM  = 6'd54;  // 68 imm16, 6A imm8 (80186)
    localparam logic [5:0] C_PUSHA     = 6'd55;  // 60 (80186)
    localparam logic [5:0] C_POPA      = 6'd56;  // 61 (80186)
    localparam logic [5:0] C_ENTER     = 6'd57;  // C8 (80186)
    localparam logic [5:0] C_LEAVE     = 6'd58;  // C9 (80186)
    localparam logic [5:0] C_BOUND     = 6'd59;  // 62 (80186)
    localparam logic [5:0] C_IMUL_IMM  = 6'd60;  // 69/6B (80186)

    // ---- fixed interrupt types (learnings/05-interrupts-and-reset.md) ----
    // The interrupt vector table lives at physical 00000-003FF; the vector for
    // a given type is at physical (type * 4): offset word first, then segment.
    localparam logic [7:0] INT_DIV_ERR = 8'd0;
    localparam logic [7:0] INT_SINGLE  = 8'd1;
    localparam logic [7:0] INT_NMI     = 8'd2;
    localparam logic [7:0] INT_BREAK   = 8'd3;
    localparam logic [7:0] INT_OVERFLW = 8'd4;
    localparam logic [7:0] INT_BOUND   = 8'd5;
    localparam logic [7:0] INT_ILLEGAL = 8'd6;
    localparam logic [7:0] INT_ESC     = 8'd7;

endpackage

`endif
