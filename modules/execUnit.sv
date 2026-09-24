// ---------------------------------------------------------------------------
// execUnit — the instruction sequencer: fetch, decode dispatch, execute, retire.
//
// Hierarchy: cpu_top -> eu -> execUnit
// Reference: learnings/01-programming-model.md, learnings/03-instruction-set.md
// Testbench: sim/tb_cpu.sv (runs real programs through the whole core)
//
// This module owns all sequencing state. decode.sv and microcode.sv are pure
// combinational lookups and eu.sv is structural wiring; the FSM is here.
//
// SHAPE: one instruction at a time, no overlap. Bytes are pulled from the
// BIU's prefetch queue, which is what decouples fetch from execution -- the
// BIU runs ahead on its own and this module pops when it needs a byte.
//
// IP OWNERSHIP: ip_next is authoritative and advances by one per byte popped.
// It is written to the register file at retire so the architectural IP is
// observable and CALL can push a correct return address. The BIU keeps a
// separate prefetch pointer; on a control transfer this module asserts
// fetch_set to redirect and flush it.
//
// REGISTER PORT PRESSURE shapes several states. The register file has two read
// ports, but some instructions want three values at once -- word DIV needs AX,
// DX and the divisor. Those get a preparatory state (S_PREP then S_LOAD2) that
// latches operands across two cycles, rather than growing a third read port
// for one instruction.
//
// INTERRUPTS are implemented: hardware (NMI and INTR), software (INT n, INT3,
// INTO), the divide-error and illegal-opcode traps, and IRET. Entry pushes
// FLAGS, CS, IP in that order, clears IF and TF, and loads CS:IP from the
// vector table at physical (type * 4). A hardware interrupt is accepted only
// at an instruction boundary, checked in S_FETCH_OP before the opcode is
// consumed. NMI is edge-triggered, latched, and ignores IF.
//
// BUS HANDSHAKE: the BIU requires `req` to fall between transfers, so every
// multi-word sequence here passes through a gap state with no request
// asserted. Holding `req` high across two reads hangs the core.
//
// FAR TRANSFERS and SEGMENT LOADS are implemented: JMP FAR and CALL FAR
// (direct ptr16:16), RETF and RETF imm16, and MOV to and from a segment
// register. A far call shares the interrupt-entry push sequence, entering it
// one step in so it pushes CS and IP but not FLAGS; RETF shares the IRET pop
// sequence, stopping one word early. Loading a segment register shadows the
// next instruction from interrupts, which is what keeps an SS:SP update
// atomic.
//
// PREFIXES are consumed in S_FETCH_OP, which loops until it sees a real
// opcode. Segment overrides replace the addressing mode's default segment;
// REP/REPNE arm the string engine; LOCK is recognised and discarded, since
// this design has no second bus master for it to lock against. An interrupt is
// never accepted between a prefix and its instruction.
//
// STRING OPERATIONS share one engine (S_STR_*). Per iteration it optionally
// reads a source, reads a destination, writes a destination, compares, and
// adjusts SI/DI/CX -- which combination is selected by the instruction class.
// SI, DI, CX and AX are written back EVERY iteration even when unchanged,
// which costs a cycle each and buys the property that matters: the
// architectural registers are always correct at an iteration boundary, so a
// repeated operation can be interrupted there and resumed. On such an
// interrupt the address pushed is that of the first prefix, so the whole
// instruction re-executes (the 8086 pushed the last prefix and got
// multi-prefix cases wrong; the 80186 fixed it).
//
// Anything decode.sv cannot place reports as C_ILLEGAL, which raises the
// type-6 trap rather than halting -- so a vector-6 handler must exist, and
// dbg_int_type shows the type when one does not.
// ---------------------------------------------------------------------------

`timescale 1ns/1ns
module execUnit
    import cpu_pkg::*;
(
    input  logic        clk,
    input  logic        rst_n,

    // ---- instruction bytes, from the BIU prefetch queue ----
    input  logic [7:0]  fetch_data,
    input  logic        fetch_valid,
    // Bytes to retire this cycle. One for the byte-at-a-time path, more when
    // the whole instruction is already in the queue.
    output logic [2:0]  fetch_pop_n,
    input  logic [7:0]  fetch_peek [0:5],
    input  logic [3:0]  fetch_count,
    output logic        fetch_set,
    output logic [19:0] fetch_addr,

    // ---- data bus requests, to the BIU ----
    output logic        req,
    output logic        req_wr,
    output logic        req_io,
    output logic        req_word,
    output logic [19:0] req_addr,
    output logic [15:0] req_wdata,
    input  logic [15:0] req_rdata,
    input  logic        req_done,

    // ---- decoder (combinational) ----
    output logic [7:0]  dec_opcode,
    output logic [7:0]  dec_modrm,
    input  logic [5:0]  iclass,
    input  logic        has_modrm,
    input  logic        word_op,
    input  logic        dir_to_reg,
    input  logic [5:0]  alu_op,
    input  logic [2:0]  imm_bytes,
    input  logic        imm_sext,
    input  logic [2:0]  op_reg,
    input  logic [3:0]  cond,
    input  logic [1:0]  mod_field,
    input  logic [2:0]  reg_field,
    input  logic [2:0]  rm_field,
    input  logic [1:0]  disp_bytes,
    input  logic        rm_is_mem,

    // ---- addressing-mode table ----
    output logic [2:0]  amode_rm,
    output logic [1:0]  amode_mod,
    input  logic        use_base,
    input  logic [2:0]  base_reg,
    input  logic        use_index,
    input  logic [2:0]  index_reg,
    input  logic [1:0]  def_seg,
    input  logic        direct_addr,

    // ---- register file ----
    output logic [2:0]  rd0_sel,
    output logic        rd0_word,
    input  logic [15:0] rd0_data,
    output logic [2:0]  rd1_sel,
    output logic        rd1_word,
    input  logic [15:0] rd1_data,
    output logic [2:0]  rf_wr_sel,
    output logic        rf_wr_word,
    output logic        rf_wr_en,
    output logic [15:0] rf_wr_data,
    output logic [1:0]  sreg_rd_sel,
    input  logic [15:0] sreg_rd_data,
    input  logic [15:0] cs,
    output logic        ip_we,
    output logic [15:0] ip_wdata,
    input  logic [15:0] flags,
    output logic [15:0] flags_wdata,
    output logic [15:0] flags_wmask,

    // ---- ALU ----
    output logic [5:0]  alu_sel,
    output logic        alu_word,
    output logic        alu_start,
    output logic [15:0] alu_a,
    output logic [15:0] alu_a_hi,
    output logic [15:0] alu_b,
    output logic        alu_cf_in,
    output logic [4:0]  alu_shift,
    input  logic [15:0] alu_result,
    input  logic [15:0] alu_result_hi,
    input  logic        alu_cf,
    input  logic        alu_pf,
    input  logic        alu_af,
    input  logic        alu_zf,
    input  logic        alu_sf,
    input  logic        alu_of,
    input  logic        alu_busy,
    input  logic        alu_div_zero,
    input  logic        alu_byte_ok,

    // ---- segment register writes (interrupt entry and IRET load CS) ----
    output logic [1:0]  sreg_wr_sel,
    output logic        sreg_wr_en,
    output logic [15:0] sreg_wr_data,

    // ---- interrupts ----
    input  logic        nmi,          // edge-triggered, non-maskable, type 2
    input  logic        intr_req,     // level, gated by IF
    input  logic [7:0]  intr_type,    // vector supplied by the controller
    output logic        intr_ack,     // one-cycle pulse on acceptance

    output logic        halted,
    output logic [7:0]  dbg_int_type, // last interrupt/trap taken
    output logic        dbg_int_taken
);

    localparam logic [5:0] S_START     = 6'd0;
    localparam logic [5:0] S_FETCH_OP  = 6'd1;
    localparam logic [5:0] S_MODRM     = 6'd2;
    localparam logic [5:0] S_DISP      = 6'd3;
    localparam logic [5:0] S_IMM       = 6'd4;
    localparam logic [5:0] S_EA        = 6'd5;
    localparam logic [5:0] S_PREP      = 6'd6;
    localparam logic [5:0] S_LOAD2     = 6'd7;
    localparam logic [5:0] S_LOAD      = 6'd8;
    localparam logic [5:0] S_EXEC      = 6'd9;
    localparam logic [5:0] S_LOOP_DEC  = 6'd10;
    localparam logic [5:0] S_ALU_WAIT  = 6'd11;
    localparam logic [5:0] S_STORE     = 6'd12;
    localparam logic [5:0] S_PUSH      = 6'd13;
    localparam logic [5:0] S_POP       = 6'd14;
    localparam logic [5:0] S_SP_UPD    = 6'd15;
    localparam logic [5:0] S_WB        = 6'd16;
    localparam logic [5:0] S_WB_HI     = 6'd17;
    localparam logic [5:0] S_RETIRE    = 6'd18;
    localparam logic [5:0] S_REDIRECT  = 6'd19;
    localparam logic [5:0] S_HALT      = 6'd20;
    // interrupt entry: push FLAGS/CS/IP, read the vector, load CS:IP
    localparam logic [5:0] S_INT_PREP  = 6'd21;
    localparam logic [5:0] S_INT_SETUP = 6'd22;
    localparam logic [5:0] S_INT_WR    = 6'd23;
    localparam logic [5:0] S_INT_SP    = 6'd24;
    localparam logic [5:0] S_INT_RDLO  = 6'd25;
    localparam logic [5:0] S_INT_RDHI  = 6'd26;
    localparam logic [5:0] S_INT_APPLY = 6'd27;
    // IRET: pop IP, CS, FLAGS
    localparam logic [5:0] S_IRET_PREP = 6'd28;
    localparam logic [5:0] S_IRET_RD   = 6'd29;
    localparam logic [5:0] S_IRET_APPL = 6'd30;
    // The BIU requires `req` to fall between transfers (its req_taken latch),
    // so any sequence of back-to-back bus cycles needs a gap state where no
    // request is asserted. Holding req high across two reads simply hangs.
    localparam logic [5:0] S_INT_GAP   = 6'd31;
    localparam logic [5:0] S_IRET_GAP  = 6'd32;
    localparam logic [5:0] S_IO_RD     = 6'd33;
    localparam logic [5:0] S_IO_WR     = 6'd34;
    localparam logic [5:0] S_FAR_APPLY = 6'd35;
    localparam logic [5:0] S_SREG_WB   = 6'd36;
    // String engine. One iteration walks CHECK -> RD1 -> RD2 -> WR -> EXEC ->
    // WB, skipping whichever steps the particular operation does not need.
    localparam logic [5:0] S_STR_PREP1 = 6'd37;
    localparam logic [5:0] S_STR_PREP2 = 6'd38;
    localparam logic [5:0] S_STR_CHECK = 6'd39;
    localparam logic [5:0] S_STR_RD1   = 6'd40;
    localparam logic [5:0] S_STR_G1    = 6'd41;
    localparam logic [5:0] S_STR_RD2   = 6'd42;
    localparam logic [5:0] S_STR_G2    = 6'd43;
    localparam logic [5:0] S_STR_WR    = 6'd44;
    localparam logic [5:0] S_STR_G3    = 6'd45;
    localparam logic [5:0] S_STR_EXEC  = 6'd46;
    localparam logic [5:0] S_STR_WB    = 6'd47;
    localparam logic [5:0] S_GET_SP    = 6'd48;
    localparam logic [5:0] S_PUSHA_RD  = 6'd49;
    localparam logic [5:0] S_PUSHA_WR  = 6'd50;
    localparam logic [5:0] S_POPA_RD   = 6'd51;
    localparam logic [5:0] S_POPA_WB   = 6'd52;
    localparam logic [5:0] S_MEM_RD2   = 6'd53;
    localparam logic [5:0] S_GAP       = 6'd54;
    localparam logic [5:0] S_ASCII_W   = 6'd55;
    localparam logic [5:0] S_ENTER_LP  = 6'd56;
    localparam logic [5:0] S_ENTER_FIN = 6'd57;
    localparam logic [5:0] S_BOUND_CHK = 6'd58;
    localparam logic [5:0] S_GAP2      = 6'd59;

    logic [5:0]  state;
    logic [7:0]  opcode_r, modrm_r;
    logic [15:0] disp_r, imm_r;
    logic [15:0] ip_next;
    logic [15:0] ea_r;
    logic [1:0]  seg_r;
    logic [15:0] rm_val, reg_val, acc_val, dx_val;
    logic [15:0] res_val, res_hi;
    logic [15:0] flags_val_r, flags_mask_r;
    logic [2:0]  wb_reg, wb_hi_reg;
    logic        wb_word, wb_en, wb_hi_en, wb_hi_word;
    logic        do_jump;
    logic [15:0] jump_target;
    logic [15:0] push_val;
    logic        call_mode;
    logic [15:0] sp_new;
    logic        second_byte;
    logic [1:0]  imm_idx;         // which immediate byte is being fetched
    logic [15:0] imm2_r;          // second immediate word: the segment of a ptr16:16
    logic [15:0] sreg_val;        // segment register read back for MOV r/m,sreg
    logic [15:0] far_cs;          // destination segment of a far transfer
    logic        seq_far;         // the push/pop sequence is a far call/ret, not an interrupt
    logic        block_int_once;  // suppress one interrupt check after a segment load
    logic [15:0] port_r;          // I/O port for IN/OUT
    logic        pop_to_flags;    // POPF: the popped word goes to FLAGS
    logic        pop_to_sreg;     // POP sreg / LES / LDS
    logic        pop_to_rm;       // POP r/m
    logic [2:0]  ga_idx;          // PUSHA/POPA register index
    logic [15:0] sp_at_entry;     // SP before PUSHA / ENTER started
    logic [15:0] mem2_val;        // second word of a far pointer or BOUND pair
    logic [15:0] enter_level;
    logic [15:0] bp_val;

    // ---- prefixes ----
    // Prefixes are consumed inside S_FETCH_OP, which loops until it sees a
    // real opcode. instr_start_ip remembers where the FIRST prefix was: an
    // interrupt during a repeated string operation must push that address, so
    // the whole instruction including its prefixes re-executes on return. The
    // 8086 pushed the address of the last prefix instead and got multi-prefix
    // cases wrong; the 80186 fixed it.
    logic        prefix_seen;
    logic        seg_ovr_en;
    logic [1:0]  seg_ovr;
    logic        rep_en;
    logic        rep_z;            // F3 = repeat while equal, F2 = while not equal
    logic [15:0] instr_start_ip;

    // ---- string engine ----
    logic [15:0] si_r, di_r, cx_r, ax_r;
    logic [15:0] str_src, str_dst;
    logic [1:0]  str_wb_step;

    // ---- interrupt state ----
    logic [7:0]  int_type_r;
    logic [1:0]  int_step;
    logic [15:0] int_ret_ip, int_new_cs, int_new_flags;
    logic        nmi_sync, nmi_prev, nmi_pending;
    logic        int_taken_r;
    // Distinguishes an externally-requested interrupt (which the controller
    // must be told was accepted) from NMI, a software INT, or a trap.
    logic        int_from_intr;

    assign dbg_int_type  = int_type_r;
    assign dbg_int_taken = int_taken_r;

    // A hardware interrupt may be accepted only at an instruction boundary.
    // NMI outranks INTR and ignores IF; INTR is gated by IF. The other
    // acceptance rules from learnings/05 (never between a prefix and its
    // instruction, never after a segment-register load) are not needed yet:
    // prefixes are not implemented and no instruction writes a segment
    // register except interrupt entry and IRET, which are not interruptible.
    logic hw_int_ready;
    assign hw_int_ready = nmi_pending || (intr_req && flags[F_IF]);

    // Prefix recognition works on the raw fetched byte rather than through
    // decode, because decode looks at opcode_r and a prefix must never become
    // opcode_r.
    // ---- how much of the instruction is already in the queue? ----
    // decode_len looks at the queue directly rather than at the registered
    // opcode, so the sequencer can know the shape of the instruction it is
    // ABOUT to read rather than the one it just read. See decode_len.sv.
    logic       dl_valid;
    logic [2:0] dl_len, dl_npfx;
    logic       dl_rep, dl_segovr_en;
    logic [1:0] dl_segovr;

    logic       dl_has_modrm;
    logic [2:0] dl_imm_bytes;

    decode_len u_dlen (
        .peek         (fetch_peek),
        .count        (fetch_count),
        .valid        (dl_valid),
        .len          (dl_len),
        .n_prefix     (dl_npfx),
        .op_has_modrm (dl_has_modrm),
        .op_imm_bytes (dl_imm_bytes),
        .has_rep      (dl_rep),
        .has_seg_ovr  (dl_segovr_en),
        .seg_ovr      (dl_segovr)
    );

    // The byte-at-a-time path still drives a single-byte pop; the fast path
    // below overrides it with the whole instruction's length.
    logic       fetch_pop;


    logic       fetch_is_seg_ovr, fetch_is_rep, fetch_is_lock, fetch_is_prefix;
    logic [1:0] fetch_seg;
    always_comb begin
        fetch_is_seg_ovr = (fetch_data == 8'h26) || (fetch_data == 8'h2E) ||
                           (fetch_data == 8'h36) || (fetch_data == 8'h3E);
        fetch_is_rep     = (fetch_data == 8'hF2) || (fetch_data == 8'hF3);
        // LOCK is recognised and discarded: this design has no other bus
        // master, so there is nothing for it to lock against.
        fetch_is_lock    = (fetch_data == 8'hF0);
        fetch_is_prefix  = fetch_is_seg_ovr || fetch_is_rep || fetch_is_lock;
        case (fetch_data)
            8'h26:   fetch_seg = SR_ES;
            8'h2E:   fetch_seg = SR_CS;
            8'h36:   fetch_seg = SR_SS;
            default: fetch_seg = SR_DS;
        endcase
    end

    // String operation shape. These pick which steps of the shared engine run.
    logic str_rd_src, str_rd_dst, str_wr_dst;
    logic str_io_rd, str_io_wr, str_cmp, str_to_ax, str_adj_si, str_adj_di;

    assign str_rd_src = (iclass == C_MOVS) || (iclass == C_CMPS) ||
                        (iclass == C_LODS) || (iclass == C_OUTS);
    assign str_rd_dst = (iclass == C_CMPS) || (iclass == C_SCAS);
    assign str_wr_dst = (iclass == C_MOVS) || (iclass == C_STOS) ||
                        (iclass == C_INS);
    assign str_io_rd  = (iclass == C_INS);
    assign str_io_wr  = (iclass == C_OUTS);
    assign str_cmp    = (iclass == C_CMPS) || (iclass == C_SCAS);
    assign str_to_ax  = (iclass == C_LODS);
    assign str_adj_si = (iclass == C_MOVS) || (iclass == C_CMPS) ||
                        (iclass == C_LODS) || (iclass == C_OUTS);
    assign str_adj_di = (iclass == C_MOVS) || (iclass == C_CMPS) ||
                        (iclass == C_STOS) || (iclass == C_SCAS) ||
                        (iclass == C_INS);

    // Decimal and ASCII adjust. These operate on AL (and AH for AAA/AAS) and
    // are the only instructions that consume the auxiliary-carry flag, which
    // is why the ALU bothers to produce it.
    logic [7:0] bcd_al, bcd_ah;
    logic       bcd_cf, bcd_af;
    logic       lo_gt9;
    assign lo_gt9 = (acc_val[3:0] > 4'd9);

    always_comb begin
        bcd_al = acc_val[7:0];
        bcd_ah = acc_val[15:8];
        bcd_cf = flags[F_CF];
        bcd_af = flags[F_AF];

        case (cond[2:0])
            3'd4: begin                                   // DAA
                if (lo_gt9 || flags[F_AF]) begin
                    bcd_al = acc_val[7:0] + 8'h06;
                    bcd_af = 1'b1;
                end else bcd_af = 1'b0;
                if ((acc_val[7:0] > 8'h99) || flags[F_CF]) begin
                    bcd_al = bcd_al + 8'h60;
                    bcd_cf = 1'b1;
                end else bcd_cf = 1'b0;
            end
            3'd5: begin                                   // DAS
                if (lo_gt9 || flags[F_AF]) begin
                    bcd_al = acc_val[7:0] - 8'h06;
                    bcd_af = 1'b1;
                end else bcd_af = 1'b0;
                if ((acc_val[7:0] > 8'h99) || flags[F_CF]) begin
                    bcd_al = bcd_al - 8'h60;
                    bcd_cf = 1'b1;
                end else bcd_cf = 1'b0;
            end
            3'd6: begin                                   // AAA
                if (lo_gt9 || flags[F_AF]) begin
                    bcd_al = (acc_val[7:0] + 8'h06) & 8'h0F;
                    bcd_ah = acc_val[15:8] + 8'h01;
                    bcd_af = 1'b1;
                    bcd_cf = 1'b1;
                end else begin
                    bcd_al = acc_val[7:0] & 8'h0F;
                    bcd_af = 1'b0;
                    bcd_cf = 1'b0;
                end
            end
            default: begin                                // AAS
                if (lo_gt9 || flags[F_AF]) begin
                    bcd_al = (acc_val[7:0] - 8'h06) & 8'h0F;
                    bcd_ah = acc_val[15:8] - 8'h01;
                    bcd_af = 1'b1;
                    bcd_cf = 1'b1;
                end else begin
                    bcd_al = acc_val[7:0] & 8'h0F;
                    bcd_af = 1'b0;
                    bcd_cf = 1'b0;
                end
            end
        endcase
    end

    logic [15:0] bcd_flags;
    always_comb begin
        bcd_flags        = 16'h0000;
        bcd_flags[F_CF]  = bcd_cf;
        bcd_flags[F_AF]  = bcd_af;
        bcd_flags[F_ZF]  = (bcd_al == 8'h00);
        bcd_flags[F_SF]  = bcd_al[7];
        bcd_flags[F_PF]  = ~^bcd_al;
    end

    // Direction flag: forward when DF is clear. Byte ops step by one, word by two.
    logic [15:0] str_delta;
    assign str_delta = flags[F_DF] ? (word_op ? 16'hFFFE : 16'hFFFF)
                                   : (word_op ? 16'h0002 : 16'h0001);

    assign dec_opcode = opcode_r;
    assign dec_modrm  = modrm_r;
    assign amode_rm   = rm_field;
    assign amode_mod  = mod_field;
    assign halted     = (state == S_HALT);

    logic [19:0] fetch_phys, ea_phys;
    assign fetch_phys = ({4'h0, cs} << 4) + {4'h0, ip_next};
    assign ea_phys    = ({4'h0, sreg_rd_data} << 4) + {4'h0, ea_r};

    logic rm_mem;
    assign rm_mem = has_modrm && rm_is_mem;

    // Index of the last immediate byte. Up to four are supported: the far
    // transfers carry a full ptr16:16, and ENTER carries a word plus a byte.
    logic [1:0] imm_last;
    always_comb begin
        case (imm_bytes)
            3'd4:    imm_last = 2'd3;
            3'd3:    imm_last = 2'd2;
            3'd2:    imm_last = 2'd1;
            default: imm_last = 2'd0;
        endcase
    end

    // A 16-bit immediate is two states today, one per byte, even when both
    // bytes are sitting in the queue. imm_take2 spots that case so S_IMM can
    // retire the pair in one cycle; everything else still goes a byte at a
    // time, including the far-pointer forms with four immediate bytes.
    logic imm_take2;
    assign imm_take2 = (state == S_IMM) && (imm_idx == 2'd0)
                       && (imm_last == 2'd1) && (fetch_count >= 4'd2);

    assign fetch_pop_n = imm_take2 ? 3'd2 : (fetch_pop ? 3'd1 : 3'd0);

    logic is_muldiv, is_grp3_test;
    assign is_muldiv    = (alu_op == ALU_MUL) || (alu_op == ALU_IMUL) ||
                          (alu_op == ALU_DIV) || (alu_op == ALU_IDIV);
    assign is_grp3_test = (iclass == C_GRP3) && (alu_op == ALU_TEST);

    // Which side of a ModR/M instruction is the destination.
    logic dst_is_rm, needs_rm_value;
    always_comb begin
        case (iclass)
            C_ALU_RM, C_MOV_RM:    dst_is_rm = ~dir_to_reg;
            C_MOV_SREG:            dst_is_rm = ~dir_to_reg;
            C_GRP1, C_GRP2, C_GRP3, C_MOV_RM_I: dst_is_rm = 1'b1;
            C_GRP5:            dst_is_rm = (reg_field <= 3'd1);   // only INC/DEC write back
            default:               dst_is_rm = 1'b0;
        endcase
    end

    always_comb begin
        case (iclass)
            C_LES_LDS, C_BOUND: needs_rm_value = 1'b0;  // the EA is the operand
            C_IMUL_IMM:        needs_rm_value = 1'b1;
            C_MOV_RM:          needs_rm_value = dir_to_reg;
            C_MOV_SREG:        needs_rm_value = dir_to_reg;   // 8E reads r/m
            C_MOV_RM_I, C_LEA: needs_rm_value = 1'b0;
            // XCHG both reads and writes its r/m operand. Leaving it out of
            // this list let the STORE half work while the register half got
            // whatever the register-file read had left in rm_val -- a swap
            // that only went one way. Nothing caught it because XCHG was only
            // ever tested register-to-register, where no load is needed.
            C_ALU_RM, C_GRP1, C_GRP2, C_GRP3, C_GRP5,
            C_TEST_RM, C_XCHG_RM: needs_rm_value = 1'b1;
            default:           needs_rm_value = 1'b0;
        endcase
    end

    logic uses_stack;
    assign uses_stack = (iclass == C_PUSH_R) || (iclass == C_POP_R) ||
                        (iclass == C_CALL_NEAR) || (iclass == C_RET_NEAR) ||
                        (iclass == C_CALL_FAR)  || (iclass == C_RETF) ||
                        (iclass == C_FLAGSTK)   || (iclass == C_SREG_STK) ||
                        (iclass == C_PUSH_IMM)  || (iclass == C_POP_RM) ||
                        (iclass == C_PUSHA)     || (iclass == C_POPA) ||
                        (iclass == C_ENTER)     || (iclass == C_LEAVE);

    // ---- condition evaluation ----
    logic f_cf, f_pf, f_zf, f_sf, f_of;
    assign f_cf = flags[F_CF];
    assign f_pf = flags[F_PF];
    assign f_zf = flags[F_ZF];
    assign f_sf = flags[F_SF];
    assign f_of = flags[F_OF];

    logic cond_true;
    always_comb begin
        case (cond[3:1])
            3'd0: cond_true = f_of;
            3'd1: cond_true = f_cf;
            3'd2: cond_true = f_zf;
            3'd3: cond_true = f_cf | f_zf;
            3'd4: cond_true = f_sf;
            3'd5: cond_true = f_pf;
            3'd6: cond_true = f_sf ^ f_of;
            default: cond_true = (f_sf ^ f_of) | f_zf;
        endcase
        if (cond[0]) cond_true = ~cond_true;   // odd Jcc opcodes are the negation
    end

    // LOOP family: E0 LOOPNZ, E1 LOOPZ, E2 LOOP, E3 JCXZ.
    logic [15:0] cx_dec;
    logic        loop_taken;
    assign cx_dec = reg_val - 16'd1;
    always_comb begin
        case (cond[1:0])
            2'd0:    loop_taken = (cx_dec != 16'h0000) && !f_zf;
            2'd1:    loop_taken = (cx_dec != 16'h0000) &&  f_zf;
            2'd2:    loop_taken = (cx_dec != 16'h0000);
            default: loop_taken = (reg_val == 16'h0000);   // JCXZ, no decrement
        endcase
    end

    // ---- flag mask per ALU operation ----
    logic [15:0] mask_for_op;
    always_comb begin
        case (alu_op)
            ALU_NOT:                  mask_for_op = FM_NONE;
            ALU_INC, ALU_DEC:         mask_for_op = FM_NOCF;
            ALU_ROL, ALU_ROR,
            ALU_RCL, ALU_RCR:         mask_for_op = FM_ROT;
            ALU_MUL, ALU_IMUL:        mask_for_op = 16'h0801;
            ALU_DIV, ALU_IDIV:        mask_for_op = FM_NONE;
            default:                  mask_for_op = FM_ALL;
        endcase
    end

    logic [15:0] alu_flag_bits;
    always_comb begin
        alu_flag_bits       = 16'h0000;
        alu_flag_bits[F_CF] = alu_cf;
        alu_flag_bits[F_PF] = alu_pf;
        alu_flag_bits[F_AF] = alu_af;
        alu_flag_bits[F_ZF] = alu_zf;
        alu_flag_bits[F_SF] = alu_sf;
        alu_flag_bits[F_OF] = alu_of;
    end

    // =====================================================================
    // Combinational outputs
    // =====================================================================
    always_comb begin
        fetch_pop   = 1'b0;
        fetch_set   = 1'b0;
        fetch_addr  = fetch_phys;
        req         = 1'b0;
        req_wr      = 1'b0;
        req_io      = 1'b0;
        req_word    = word_op;
        req_addr    = ea_phys;
        req_wdata   = res_val;
        rd0_sel     = reg_field;
        rd0_word    = word_op;
        rd1_sel     = rm_field;
        rd1_word    = word_op;
        rf_wr_sel   = wb_reg;
        rf_wr_word  = wb_word;
        rf_wr_en    = 1'b0;
        rf_wr_data  = res_val;
        sreg_rd_sel = seg_r;
        ip_we       = 1'b0;
        ip_wdata    = ip_next;
        flags_wdata = flags_val_r;
        flags_wmask = FM_NONE;
        alu_sel      = alu_op;
        alu_word     = word_op;
        if (iclass == C_ASCII) begin
            alu_sel  = cond[0] ? ALU_MUL : ALU_DIV;   // AAD multiplies, AAM divides
            alu_word = 1'b0;
        end
        if (iclass == C_IMUL_IMM) alu_sel = ALU_IMUL;
        alu_start    = 1'b0;
        alu_cf_in    = f_cf;
        sreg_wr_sel  = SR_CS;
        sreg_wr_en   = 1'b0;
        sreg_wr_data = int_new_cs;
        intr_ack     = 1'b0;

        // ---- ALU operand routing ----
        case (iclass)
            C_ALU_RM:   begin
                alu_a = dir_to_reg ? reg_val : rm_val;
                alu_b = dir_to_reg ? rm_val  : reg_val;
            end
            C_GRP1:     begin alu_a = rm_val;  alu_b = imm_r;   end
            C_ALU_ACC:  begin alu_a = acc_val; alu_b = imm_r;   end
            C_TEST_RM:  begin alu_a = rm_val;  alu_b = reg_val; end
            C_TEST_ACC: begin alu_a = acc_val; alu_b = imm_r;   end
            C_INCDEC_R: begin alu_a = reg_val; alu_b = 16'h0000; end
            // CMPS computes (DS:SI) - (ES:DI); SCAS computes AX - (ES:DI).
            C_IMUL_IMM: begin alu_a = rm_val;  alu_b = imm_r;   end
            // AAM divides AL by the base; AAD multiplies AH by it. Both use
            // the byte-width ALU, which takes its operand from the low half,
            // so AH has to be moved down for AAD.
            C_ASCII:    begin
                alu_a = cond[0] ? {8'h00, acc_val[15:8]} : {8'h00, acc_val[7:0]};
                alu_b = {8'h00, imm_r[7:0]};
            end
            C_CMPS:     begin alu_a = str_src; alu_b = str_dst; end
            C_SCAS:     begin alu_a = ax_r;    alu_b = str_dst; end
            C_GRP3:     begin
                if (is_muldiv)         begin alu_a = acc_val; alu_b = rm_val; end
                else if (is_grp3_test) begin alu_a = rm_val;  alu_b = imm_r;  end
                else                   begin alu_a = rm_val;  alu_b = 16'h0000; end
            end
            default:    begin alu_a = rm_val;  alu_b = 16'h0000; end
        endcase
        alu_a_hi = dx_val;

        // ---- shift count ----
        // decode puts the count source in `cond`: 0 = literal 1, 1 = CL,
        // 2 = immediate byte.
        case (cond[1:0])
            2'd1:    alu_shift = reg_val[4:0];   // CL, latched in S_PREP
            2'd2:    alu_shift = imm_r[4:0];
            default: alu_shift = 5'd1;
        endcase

        // ---- read-port steering ----
        case (state)
            S_EA: begin
                rd0_sel = base_reg;  rd0_word = 1'b1;
                rd1_sel = index_reg; rd1_word = 1'b1;
            end
            S_PREP: begin
                case (iclass)
                    C_ALU_ACC, C_TEST_ACC, C_MOV_ACC: begin
                        rd0_sel = R_AX; rd0_word = word_op;
                    end
                    C_BCD, C_SIGNEXT, C_AHFLAGS, C_ASCII: begin
                        rd0_sel = R_AX; rd0_word = 1'b1;
                    end
                    C_XLAT: begin
                        rd0_sel = R_AX; rd0_word = 1'b1;
                        rd1_sel = R_BX; rd1_word = 1'b1;
                    end
                    C_XCHG_R: begin
                        rd0_sel = op_reg; rd0_word = 1'b1;
                        rd1_sel = R_AX;   rd1_word = 1'b1;
                    end
                    C_FLAGSTK, C_PUSH_IMM, C_SREG_STK, C_POP_RM,
                    C_PUSHA, C_POPA: begin
                        rd1_sel = R_SP; rd1_word = 1'b1;
                    end
                    C_ENTER, C_LEAVE: begin
                        rd0_sel = R_BP; rd0_word = 1'b1;
                        rd1_sel = R_SP; rd1_word = 1'b1;
                    end
                    C_GRP3: begin
                        rd0_sel = R_AX; rd0_word = word_op;
                        rd1_sel = R_DX; rd1_word = 1'b1;
                    end
                    C_INCDEC_R, C_PUSH_R: begin
                        rd0_sel = op_reg; rd0_word = 1'b1;
                        rd1_sel = R_SP;   rd1_word = 1'b1;
                    end
                    C_POP_R, C_CALL_NEAR, C_RET_NEAR: begin
                        rd1_sel = R_SP; rd1_word = 1'b1;
                    end
                    C_LOOP: begin
                        rd0_sel = R_CX; rd0_word = 1'b1;
                    end
                    C_IN, C_OUT, C_INS, C_OUTS: begin
                        rd0_sel = R_AX; rd0_word = word_op;
                        rd1_sel = R_DX; rd1_word = 1'b1;
                    end
                    C_CALL_FAR, C_RETF: begin
                        rd1_sel = R_SP; rd1_word = 1'b1;
                    end
                    C_GRP2: begin
                        rd0_sel = R_CX; rd0_word = 1'b1;   // for the by-CL form
                    end
                    default: ;
                endcase
            end
            S_PUSH, S_POP, S_SP_UPD,
            S_INT_PREP, S_IRET_PREP: begin
                rd1_sel = R_SP; rd1_word = 1'b1;
            end
            // PUSHA walks the register file in encoding order, so the index
            // doubles as the register number.
            S_PUSHA_RD: begin
                rd0_sel = ga_idx; rd0_word = 1'b1;
            end
            S_GET_SP: begin
                rd1_sel = R_SP; rd1_word = 1'b1;
            end
            // The string engine needs four registers and there are two read
            // ports, so they are latched across two cycles.
            S_STR_PREP1: begin
                rd0_sel = R_SI; rd0_word = 1'b1;
                rd1_sel = R_DI; rd1_word = 1'b1;
            end
            S_STR_PREP2: begin
                rd0_sel = R_CX; rd0_word = 1'b1;
                rd1_sel = R_AX; rd1_word = word_op;
            end
            default: ;
        endcase

        // For MOV r/m,sreg the source is a segment register, so the segment
        // read port is borrowed here. The EA's own segment is only needed
        // later, in S_STORE, so there is no conflict.
        if ((state == S_PREP) && (iclass == C_MOV_SREG) && !dir_to_reg)
            sreg_rd_sel = reg_field[1:0];
        if ((state == S_PREP) && (iclass == C_SREG_STK) && !dir_to_reg)
            sreg_rd_sel = op_reg[1:0];

        // ---- state-driven outputs ----
        case (state)
            S_START: begin
                fetch_set  = 1'b1;
                fetch_addr = ({4'h0, cs} << 4);
            end

            // Pop only when the byte is actually consumed. S_MODRM and S_DISP
            // are entered even when the instruction has no ModR/M or no
            // displacement, and popping there would silently discard the next
            // byte of the instruction stream.
            S_FETCH_OP: fetch_pop = fetch_valid;
            S_MODRM:    fetch_pop = fetch_valid && has_modrm;
            S_DISP:     fetch_pop = fetch_valid && (disp_bytes != 2'd0);
            S_IMM:      fetch_pop = fetch_valid;

            S_LOAD: begin
                req      = 1'b1;
                req_addr = ea_phys;
            end

            S_EXEC: alu_start = (is_muldiv && (iclass == C_GRP3)) ||
                                (iclass == C_ASCII) || (iclass == C_IMUL_IMM);

            S_STORE: begin
                req       = 1'b1;
                req_wr    = 1'b1;
                req_addr  = ea_phys;
                req_wdata = res_val;
            end

            S_PUSH: begin
                req         = 1'b1;
                req_wr      = 1'b1;
                req_word    = 1'b1;
                sreg_rd_sel = SR_SS;
                req_addr    = ({4'h0, sreg_rd_data} << 4) + {4'h0, ea_r};
                req_wdata   = push_val;
            end

            S_POP: begin
                req         = 1'b1;
                req_word    = 1'b1;
                sreg_rd_sel = SR_SS;
                req_addr    = ({4'h0, sreg_rd_data} << 4) + {4'h0, ea_r};
            end

            S_IO_RD: begin
                req      = 1'b1;
                req_io   = 1'b1;
                req_addr = {4'h0, port_r};
            end

            S_IO_WR: begin
                req       = 1'b1;
                req_wr    = 1'b1;
                req_io    = 1'b1;
                req_addr  = {4'h0, port_r};
                req_wdata = res_val;
            end

            S_PUSHA_WR: begin
                req         = 1'b1;
                req_wr      = 1'b1;
                req_word    = 1'b1;
                sreg_rd_sel = SR_SS;
                req_addr    = ({4'h0, sreg_rd_data} << 4) + {4'h0, ea_r};
                req_wdata   = push_val;
            end

            S_POPA_RD: begin
                req         = 1'b1;
                req_word    = 1'b1;
                sreg_rd_sel = SR_SS;
                req_addr    = ({4'h0, sreg_rd_data} << 4) + {4'h0, ea_r};
            end

            // POPA restores in reverse order and throws away the stored SP.
            S_POPA_WB: begin
                if (ga_idx != 3'd3) begin
                    rf_wr_en   = 1'b1;
                    rf_wr_sel  = 3'd7 - ga_idx;
                    rf_wr_word = 1'b1;
                    rf_wr_data = res_val;
                end
            end

            // Second memory word: ENTER's display copy reads the caller's
            // frame through SS, everything else uses the instruction's own
            // segment.
            S_MEM_RD2: begin
                req      = 1'b1;
                req_word = 1'b1;
                if (iclass == C_ENTER) begin
                    sreg_rd_sel = SR_SS;
                    req_addr    = ({4'h0, sreg_rd_data} << 4) + {4'h0, ea_r};
                end else begin
                    req_addr = ea_phys;
                end
            end

            S_SP_UPD: begin
                rf_wr_sel  = R_SP;
                rf_wr_word = 1'b1;
                rf_wr_data = sp_new;
                rf_wr_en   = 1'b1;
            end

            S_WB: begin
                rf_wr_en   = wb_en;
                rf_wr_sel  = wb_reg;
                rf_wr_word = wb_word;
                rf_wr_data = res_val;
            end

            S_WB_HI: begin
                rf_wr_en   = 1'b1;
                rf_wr_sel  = wb_hi_reg;
                rf_wr_word = wb_hi_word;
                rf_wr_data = res_hi;
            end

            S_RETIRE: begin
                ip_we       = 1'b1;
                ip_wdata    = do_jump ? jump_target : ip_next;
                flags_wmask = flags_mask_r;
                flags_wdata = flags_val_r;
            end

            S_REDIRECT: begin
                fetch_set  = 1'b1;
                fetch_addr = ({4'h0, cs} << 4) + {4'h0, jump_target};
                // Keep the architectural IP mirror correct. Far returns and
                // IRET reach here without passing through S_RETIRE, so without
                // this the register file's IP would go stale after every one
                // of them -- and that value is what dbg_ip reports to the
                // board LEDs.
                ip_we      = 1'b1;
                ip_wdata   = jump_target;
            end

            // Far transfers load CS here; S_REDIRECT then sees the updated
            // value when it computes the fetch address.
            S_FAR_APPLY: begin
                sreg_wr_sel  = SR_CS;
                sreg_wr_data = far_cs;
                sreg_wr_en   = 1'b1;
            end

            S_SREG_WB: begin
                sreg_wr_sel  = (iclass == C_MOV_SREG) ? reg_field[1:0]
                                                      : op_reg[1:0];
                sreg_wr_data = rm_val;
                sreg_wr_en   = 1'b1;
            end

            // Source read: memory at DS:SI (overridable), or an I/O port for INS.
            S_STR_RD1: begin
                req      = 1'b1;
                req_word = word_op;
                if (str_io_rd) begin
                    req_io   = 1'b1;
                    req_addr = {4'h0, rm_val};        // port number, latched from DX
                end else begin
                    sreg_rd_sel = seg_ovr_en ? seg_ovr : SR_DS;
                    req_addr    = ({4'h0, sreg_rd_data} << 4) + {4'h0, si_r};
                end
            end

            // Destination read, for the compare forms. Always ES:DI, which is
            // not overridable.
            S_STR_RD2: begin
                req         = 1'b1;
                req_word    = word_op;
                sreg_rd_sel = SR_ES;
                req_addr    = ({4'h0, sreg_rd_data} << 4) + {4'h0, di_r};
            end

            // Destination write: ES:DI, or an I/O port for OUTS.
            S_STR_WR: begin
                req      = 1'b1;
                req_wr   = 1'b1;
                req_word = word_op;
                if (str_io_wr) begin
                    req_io    = 1'b1;
                    req_addr  = {4'h0, rm_val};
                    req_wdata = str_src;
                end else begin
                    sreg_rd_sel = SR_ES;
                    req_addr    = ({4'h0, sreg_rd_data} << 4) + {4'h0, di_r};
                    req_wdata   = (iclass == C_STOS) ? ax_r : str_src;
                end
            end

            // SI, DI, CX and AX are all written back every iteration. Writing
            // an unchanged value costs a cycle and nothing else, and it means
            // the architectural registers are always correct at the top of the
            // loop -- which is what makes the operation safely interruptible.
            S_STR_WB: begin
                rf_wr_en   = 1'b1;
                rf_wr_word = 1'b1;
                case (str_wb_step)
                    2'd0: begin rf_wr_sel = R_SI; rf_wr_data = si_r; end
                    2'd1: begin rf_wr_sel = R_DI; rf_wr_data = di_r; end
                    2'd2: begin rf_wr_sel = R_CX; rf_wr_data = cx_r; end
                    default: begin
                        rf_wr_sel  = R_AX;
                        rf_wr_word = word_op;
                        rf_wr_data = ax_r;
                    end
                endcase
            end

            // ---- interrupt entry ----
            S_INT_WR: begin
                req         = 1'b1;
                req_wr      = 1'b1;
                req_word    = 1'b1;
                sreg_rd_sel = SR_SS;
                req_addr    = ({4'h0, sreg_rd_data} << 4) + {4'h0, ea_r};
                req_wdata   = push_val;
            end

            S_INT_PREP: intr_ack = int_from_intr;

            S_INT_SP: begin
                rf_wr_sel  = R_SP;
                rf_wr_word = 1'b1;
                rf_wr_data = sp_new;
                rf_wr_en   = 1'b1;
            end

            // The vector table is at physical 00000-003FF, addressed absolutely
            // rather than through a segment register.
            S_INT_RDLO: begin
                req      = 1'b1;
                req_word = 1'b1;
                req_addr = {10'h000, int_type_r, 2'b00};
            end

            S_INT_RDHI: begin
                req      = 1'b1;
                req_word = 1'b1;
                req_addr = {10'h000, int_type_r, 2'b00} + 20'd2;
            end

            S_INT_APPLY: begin
                sreg_wr_sel  = SR_CS;
                sreg_wr_data = int_new_cs;
                sreg_wr_en   = 1'b1;
                // Interrupt entry clears IF and TF so the handler runs with
                // further maskable interrupts disabled and no single-step.
                flags_wmask  = FM_IFTF;
                flags_wdata  = 16'h0000;
            end

            // ---- IRET ----
            S_IRET_RD: begin
                req         = 1'b1;
                req_word    = 1'b1;
                sreg_rd_sel = SR_SS;
                req_addr    = ({4'h0, sreg_rd_data} << 4) + {4'h0, ea_r};
            end

            S_IRET_APPL: begin
                // SP, CS and FLAGS go to three independent register-file
                // ports, so all three land in the same cycle.
                rf_wr_sel    = R_SP;
                rf_wr_word   = 1'b1;
                rf_wr_data   = sp_new;
                rf_wr_en     = 1'b1;
                sreg_wr_sel  = SR_CS;
                sreg_wr_data = int_new_cs;
                sreg_wr_en   = 1'b1;
                flags_wmask  = seq_far ? FM_NONE : FM_POPF;
                flags_wdata  = int_new_flags;
            end

            default: ;
        endcase
    end

    // =====================================================================
    // Sequencer
    // =====================================================================
    logic [15:0] rel_target;
    assign rel_target = ip_next + imm_r;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_START;
            opcode_r     <= 8'h90;
            modrm_r      <= 8'h00;
            disp_r       <= 16'h0000;
            imm_r        <= 16'h0000;
            ip_next      <= 16'h0000;
            ea_r         <= 16'h0000;
            seg_r        <= SR_DS;
            rm_val       <= 16'h0000;
            reg_val      <= 16'h0000;
            acc_val      <= 16'h0000;
            dx_val       <= 16'h0000;
            res_val      <= 16'h0000;
            res_hi       <= 16'h0000;
            flags_val_r  <= 16'h0000;
            flags_mask_r <= FM_NONE;
            wb_reg       <= 3'd0;
            wb_hi_reg    <= R_DX;
            wb_hi_word   <= 1'b1;
            wb_word      <= 1'b1;
            wb_en        <= 1'b0;
            wb_hi_en     <= 1'b0;
            do_jump      <= 1'b0;
            jump_target  <= 16'h0000;
            push_val     <= 16'h0000;
            call_mode    <= 1'b0;
            sp_new       <= 16'h0000;
            second_byte  <= 1'b0;
            imm_idx      <= 2'd0;
            imm2_r       <= 16'h0000;
            sreg_val     <= 16'h0000;
            far_cs       <= 16'h0000;
            seq_far      <= 1'b0;
            block_int_once <= 1'b0;
            prefix_seen  <= 1'b0;
            seg_ovr_en   <= 1'b0;
            seg_ovr      <= SR_DS;
            rep_en       <= 1'b0;
            rep_z        <= 1'b1;
            instr_start_ip <= 16'h0000;
            pop_to_flags <= 1'b0;
            pop_to_sreg  <= 1'b0;
            pop_to_rm    <= 1'b0;
            ga_idx       <= 3'd0;
            sp_at_entry  <= 16'h0000;
            mem2_val     <= 16'h0000;
            enter_level  <= 16'h0000;
            bp_val       <= 16'h0000;
            wb_hi_word   <= 1'b1;
            si_r         <= 16'h0000;
            di_r         <= 16'h0000;
            cx_r         <= 16'h0000;
            ax_r         <= 16'h0000;
            str_src      <= 16'h0000;
            str_dst      <= 16'h0000;
            str_wb_step  <= 2'd0;
            port_r       <= 16'h0000;
            int_type_r   <= 8'h00;
            int_step     <= 2'd0;
            int_ret_ip   <= 16'h0000;
            int_new_cs   <= 16'h0000;
            int_new_flags<= 16'h0000;
            nmi_sync     <= 1'b0;
            nmi_prev     <= 1'b0;
            nmi_pending   <= 1'b0;
            int_taken_r   <= 1'b0;
            int_from_intr <= 1'b0;
        end else begin
            // NMI is edge-triggered (low->high) and latched, so a pulse is
            // never missed even though it is only acted on at an instruction
            // boundary. Synchronized first because it is an external pin.
            nmi_sync <= nmi;
            nmi_prev <= nmi_sync;
            if (nmi_sync && !nmi_prev) nmi_pending <= 1'b1;

            case (state)
                S_START: begin
                    ip_next <= 16'h0000;
                    state   <= S_FETCH_OP;
                end

                // ---------- instruction byte fetch ----------
                S_FETCH_OP: begin
                    disp_r       <= 16'h0000;
                    imm_r        <= 16'h0000;
                    second_byte  <= 1'b0;
                    do_jump      <= 1'b0;
                    wb_en        <= 1'b0;
                    wb_hi_en     <= 1'b0;
                    call_mode    <= 1'b0;
                    pop_to_flags <= 1'b0;
                    pop_to_sreg  <= 1'b0;
                    pop_to_rm    <= 1'b0;
                    flags_mask_r <= FM_NONE;
                    seg_r        <= SR_DS;
                    int_taken_r  <= 1'b0;

                    // An instruction boundary is the only place a hardware
                    // interrupt may be accepted. Checked before the opcode is
                    // consumed so the pending instruction is not half-started.
                    int_from_intr <= 1'b0;
                    imm_idx       <= 2'd0;
                    seq_far       <= 1'b0;
                    // Only reset prefix state when starting a fresh
                    // instruction; mid-scan these must persist.
                    if (!prefix_seen) begin
                        seg_ovr_en     <= 1'b0;
                        rep_en         <= 1'b0;
                        instr_start_ip <= ip_next;
                    end
                    // An interrupt may not be accepted between an instruction
                    // that loads a segment register and the one after it. That
                    // is what keeps an SS:SP pair updating atomically -- taking
                    // an interrupt in between would push onto a half-updated
                    // stack.
                    // Prefixes are consumed here, looping until a real
                    // opcode arrives. An interrupt may not be accepted between
                    // a prefix and its instruction, so the check below is
                    // skipped once any prefix has been seen.
                    if (fetch_valid && fetch_is_prefix) begin
                        ip_next     <= ip_next + 16'd1;
                        prefix_seen <= 1'b1;
                        if (fetch_is_seg_ovr) begin
                            seg_ovr_en <= 1'b1;
                            seg_ovr    <= fetch_seg;
                        end else if (fetch_is_rep) begin
                            rep_en <= 1'b1;
                            rep_z  <= (fetch_data == 8'hF3);
                        end
                        // LOCK is accepted and ignored.
                    end else if (block_int_once || prefix_seen) begin
                        if (block_int_once) block_int_once <= 1'b0;
                        if (fetch_valid) begin
                            opcode_r <= fetch_data;
                            ip_next  <= ip_next + 16'd1;
                            state    <= S_MODRM;
                        end
                    end else if (hw_int_ready) begin
                        int_ret_ip <= ip_next;
                        if (nmi_pending) begin
                            int_type_r  <= INT_NMI;
                            nmi_pending <= 1'b0;
                        end else begin
                            int_type_r    <= intr_type;
                            int_from_intr <= 1'b1;
                        end
                        state <= S_INT_PREP;
                    end else if (fetch_valid) begin
                        opcode_r <= fetch_data;
                        ip_next  <= ip_next + 16'd1;
                        // SKIP S_MODRM WHEN THERE IS NO ModR/M BYTE. That
                        // state would only transition, and every instruction
                        // was paying for it: 1,077,357 cycles across 968,049
                        // instructions on the MS-DOS boot.
                        //
                        // The decision has to come from the byte being read,
                        // not from `has_modrm`, which is decoded from the
                        // REGISTERED opcode and so still describes the
                        // previous instruction. decode_len looks at the queue
                        // directly, which is the whole reason it is here.
                        //
                        // prefix_seen is cleared here because S_MODRM is
                        // where that normally happens; skipping the state
                        // without clearing it leaves the next instruction
                        // believing it is still scanning prefixes.
                        if (!dl_has_modrm) begin
                            prefix_seen <= 1'b0;
                            state <= (dl_imm_bytes != 3'd0) ? S_IMM : S_PREP;
                        end else begin
                            state <= S_MODRM;
                        end
                    end
                end

                S_MODRM: begin
                    prefix_seen <= 1'b0;
                    if (has_modrm) begin
                        if (fetch_valid) begin
                            modrm_r <= fetch_data;
                            ip_next <= ip_next + 16'd1;
                            state   <= S_DISP;
                        end
                    end else begin
                        state <= (imm_bytes != 2'd0) ? S_IMM : S_PREP;
                    end
                end

                S_DISP: begin
                    if (disp_bytes == 2'd0) begin
                        state <= (imm_bytes != 2'd0) ? S_IMM
                               : (rm_mem ? S_EA : S_PREP);
                    end else if (fetch_valid) begin
                        ip_next <= ip_next + 16'd1;
                        if (!second_byte) begin
                            disp_r <= (disp_bytes == 2'd1)
                                        ? {{8{fetch_data[7]}}, fetch_data}
                                        : {8'h00, fetch_data};
                            if (disp_bytes == 2'd1)
                                state <= (imm_bytes != 2'd0) ? S_IMM
                                       : (rm_mem ? S_EA : S_PREP);
                            else
                                second_byte <= 1'b1;
                        end else begin
                            disp_r[15:8] <= fetch_data;
                            second_byte  <= 1'b0;
                            state <= (imm_bytes != 2'd0) ? S_IMM
                                   : (rm_mem ? S_EA : S_PREP);
                        end
                    end
                end

                S_IMM: begin
                    if (imm_take2) begin
                        // Both bytes at once. The byte order is the same one
                        // the sequential path builds: first byte low, second
                        // byte high.
                        imm_r   <= {fetch_peek[1], fetch_peek[0]};
                        ip_next <= ip_next + 16'd2;
                        imm_idx <= 2'd0;
                        state   <= rm_mem ? S_EA : S_PREP;
                    end else if (fetch_valid) begin
                        ip_next <= ip_next + 16'd1;
                        case (imm_idx)
                            2'd0: imm_r <= imm_sext ? {{8{fetch_data[7]}}, fetch_data}
                                                    : {8'h00, fetch_data};
                            2'd1: imm_r[15:8]  <= fetch_data;
                            2'd2: imm2_r[7:0]  <= fetch_data;   // ptr16:16 segment
                            default: imm2_r[15:8] <= fetch_data;
                        endcase
                        if (imm_idx == imm_last) begin
                            imm_idx <= 2'd0;
                            state   <= rm_mem ? S_EA : S_PREP;
                        end else begin
                            imm_idx <= imm_idx + 2'd1;
                        end
                    end
                end

                // ---------- effective address ----------
                S_EA: begin
                    ea_r  <= (use_base  ? rd0_data : 16'h0000) +
                             (use_index ? rd1_data : 16'h0000) +
                             disp_r;
                    // A segment-override prefix replaces the addressing mode's
                    // default segment.
                    seg_r <= seg_ovr_en ? seg_ovr : def_seg;
                    state <= S_PREP;
                end

                // ---------- latch operands that are not the ModR/M source ----
                S_PREP: begin
                    reg_val  <= rd0_data;
                    acc_val  <= rd0_data;
                    sreg_val <= sreg_rd_data;
                    if (uses_stack)         sp_new <= rd1_data;
                    else if (iclass == C_GRP3) dx_val <= rd1_data;
                    else                    rm_val <= rd1_data;

                    if (iclass == C_MOV_ACC) begin
                        // The "immediate" is a direct address for this form.
                        ea_r  <= imm_r;
                        seg_r <= seg_ovr_en ? seg_ovr : SR_DS;
                        state <= dir_to_reg ? S_LOAD : S_EXEC;
                    end else if (iclass == C_GRP3) begin
                        state <= S_LOAD2;       // still need the ModR/M operand
                    end else if (needs_rm_value && rm_mem) begin
                        state <= S_LOAD;
                    end else begin
                        state <= S_EXEC;
                    end
                end

                // Third operand read for GRP3, which already used both ports
                // on AX and DX.
                S_LOAD2: begin
                    rm_val <= rd1_data;
                    state  <= rm_mem ? S_LOAD : S_EXEC;
                end

                S_LOAD: begin
                    if (req_done) begin
                        rm_val <= req_rdata;
                        if (iclass == C_XLAT) begin
                            res_val <= {8'h00, req_rdata[7:0]};
                            wb_reg  <= R_AL;
                            wb_word <= 1'b0;
                            wb_en   <= 1'b1;
                            state   <= S_WB;
                        end else if ((iclass == C_LES_LDS) || (iclass == C_BOUND) ||
                                     ((iclass == C_GRP5) &&
                                      ((reg_field == 3'd3) || (reg_field == 3'd5)))) begin
                            ea_r  <= ea_r + 16'd2;
                            state <= S_GAP2;
                        end else begin
                            state <= S_EXEC;
                        end
                    end
                end

                S_GAP2: state <= S_MEM_RD2;

                // ---------- execute ----------
                S_EXEC: begin
                    state        <= S_WB;
                    wb_en        <= 1'b0;
                    wb_hi_en     <= 1'b0;
                    flags_mask_r <= FM_NONE;

                    case (iclass)
                        C_NOP: ;
                        C_HLT: state <= S_HALT;

                        // Unimplemented and genuinely-invalid opcodes both
                        // raise the 80186's illegal-instruction trap. A BIOS
                        // must install a type-6 handler; dbg_int_type makes it
                        // visible in simulation when one is not there.
                        C_ILLEGAL: begin
                            int_type_r <= INT_ILLEGAL;
                            int_ret_ip <= ip_next;
                            state      <= S_INT_PREP;
                        end

                        C_INT3: begin
                            int_type_r <= INT_BREAK;
                            int_ret_ip <= ip_next;
                            state      <= S_INT_PREP;
                        end

                        C_INT_IMM: begin
                            int_type_r <= imm_r[7:0];
                            int_ret_ip <= ip_next;
                            state      <= S_INT_PREP;
                        end

                        C_INTO: begin
                            // Only traps when OF is set; otherwise a no-op.
                            if (f_of) begin
                                int_type_r <= INT_OVERFLW;
                                int_ret_ip <= ip_next;
                                state      <= S_INT_PREP;
                            end
                        end

                        C_IRET: state <= S_IRET_PREP;

                        C_MOVS, C_CMPS, C_STOS, C_LODS,
                        C_SCAS, C_INS,  C_OUTS: state <= S_STR_PREP1;

                        C_XLAT: begin
                            // AL indexes a table based at DS:BX.
                            ea_r  <= rm_val + {8'h00, acc_val[7:0]};
                            seg_r <= seg_ovr_en ? seg_ovr : SR_DS;
                            state <= S_LOAD;
                        end

                        C_LES_LDS, C_BOUND: begin
                            // Both take a memory operand that is a PAIR of
                            // words, so the first is read here and the second
                            // from ea_r+2.
                            state <= S_LOAD;
                        end

                        C_IMUL_IMM, C_ASCII: state <= S_ALU_WAIT;

                        C_FLAGSTK: begin
                            if (dir_to_reg) begin              // POPF
                                ea_r         <= sp_new;
                                pop_to_flags <= 1'b1;
                                state        <= S_POP;
                            end else begin                     // PUSHF
                                push_val <= flags;
                                ea_r     <= sp_new - 16'd2;
                                sp_new   <= sp_new - 16'd2;
                                state    <= S_PUSH;
                            end
                        end

                        C_SREG_STK: begin
                            if (dir_to_reg) begin              // POP sreg
                                ea_r        <= sp_new;
                                pop_to_sreg <= 1'b1;
                                state       <= S_POP;
                            end else begin                     // PUSH sreg
                                push_val <= sreg_val;
                                ea_r     <= sp_new - 16'd2;
                                sp_new   <= sp_new - 16'd2;
                                state    <= S_PUSH;
                            end
                        end

                        C_PUSH_IMM: begin
                            push_val <= imm_r;
                            ea_r     <= sp_new - 16'd2;
                            sp_new   <= sp_new - 16'd2;
                            state    <= S_PUSH;
                        end

                        C_POP_RM: begin
                            mem2_val  <= ea_r;         // hold the destination
                            ea_r      <= sp_new;       // ea_r now addresses the stack
                            pop_to_rm <= 1'b1;
                            state     <= S_POP;
                        end

                        C_PUSHA: begin
                            // The SP that gets pushed is its value BEFORE the
                            // sequence started, not the running one.
                            sp_at_entry <= sp_new;
                            ga_idx      <= 3'd0;
                            state       <= S_PUSHA_RD;
                        end

                        C_POPA: begin
                            ga_idx <= 3'd0;
                            ea_r   <= sp_new;
                            state  <= S_POPA_RD;
                        end

                        C_LEAVE: begin
                            // SP := BP, then BP := pop(). The new SP is simply
                            // the old BP, so the pop reads from there.
                            ea_r        <= reg_val;
                            sp_new      <= reg_val;
                            wb_reg      <= R_BP;
                            wb_word     <= 1'b1;
                            state       <= S_POP;
                        end

                        C_ENTER: begin
                            push_val    <= reg_val;            // push BP
                            ea_r        <= sp_new - 16'd2;
                            sp_new      <= sp_new - 16'd2;
                            sp_at_entry <= sp_new - 16'd2;     // the frame pointer
                            bp_val      <= reg_val;
                            enter_level <= {8'h00, imm2_r[7:0]};
                            state       <= S_PUSH;
                        end

                        C_SIGNEXT: begin
                            // CBW widens AL into AX; CWD widens AX into DX:AX,
                            // so only DX is written.
                            if (cond[0]) begin
                                res_val <= {16{acc_val[15]}};
                                wb_reg  <= R_DX;
                            end else begin
                                res_val <= {{8{acc_val[7]}}, acc_val[7:0]};
                                wb_reg  <= R_AX;
                            end
                            wb_word <= 1'b1;
                            wb_en   <= 1'b1;
                        end

                        C_AHFLAGS: begin
                            if (cond[0]) begin                 // LAHF
                                res_val <= {8'h00, flags[7:0]};
                                wb_reg  <= R_AH;
                                wb_word <= 1'b0;
                                wb_en   <= 1'b1;
                            end else begin                     // SAHF
                                // Only the low byte of FLAGS is transferable.
                                flags_val_r  <= {8'h00, acc_val[15:8]};
                                flags_mask_r <= 16'h00D5;
                            end
                        end

                        C_BCD: begin
                            res_val      <= {bcd_ah, bcd_al};
                            wb_reg       <= R_AX;
                            wb_word      <= 1'b1;
                            wb_en        <= 1'b1;
                            flags_val_r  <= bcd_flags;
                            flags_mask_r <= 16'h00D5;
                        end

                        C_XCHG_R: begin
                            // AX and the named register swap, which needs two
                            // register writes and so two writeback cycles.
                            res_val    <= rm_val;              // old AX
                            wb_reg     <= op_reg;
                            wb_word    <= 1'b1;
                            wb_en      <= 1'b1;
                            res_hi     <= reg_val;             // old op_reg
                            wb_hi_reg  <= R_AX;
                            wb_hi_word <= 1'b1;
                            wb_hi_en   <= 1'b1;
                        end

                        C_XCHG_RM: begin
                            res_val    <= reg_val;             // goes to r/m
                            res_hi     <= rm_val;              // goes to reg
                            wb_hi_reg  <= reg_field;
                            wb_hi_word <= word_op;
                            wb_hi_en   <= 1'b1;
                            if (rm_mem) begin
                                state <= S_STORE;
                            end else begin
                                wb_reg  <= rm_field;
                                wb_word <= word_op;
                                wb_en   <= 1'b1;
                            end
                        end

                        C_MOV_SREG: begin
                            if (dir_to_reg) begin
                                state <= S_SREG_WB;          // 8E: sreg <- r/m
                            end else begin
                                res_val <= sreg_val;         // 8C: r/m <- sreg
                                if (rm_mem) state <= S_STORE;
                                else begin
                                    wb_reg  <= rm_field;
                                    wb_word <= 1'b1;
                                    wb_en   <= 1'b1;
                                end
                            end
                        end

                        C_JMP_FAR: begin
                            far_cs      <= imm2_r;
                            jump_target <= imm_r;
                            state       <= S_FAR_APPLY;
                        end

                        C_CALL_FAR: begin
                            // Pushes CS then IP -- the same sequence interrupt
                            // entry uses, minus the FLAGS push, so it shares
                            // that state machine starting one step in.
                            far_cs      <= imm2_r;
                            jump_target <= imm_r;
                            int_ret_ip  <= ip_next;
                            seq_far     <= 1'b1;
                            state       <= S_INT_PREP;
                        end

                        C_RETF: begin
                            seq_far <= 1'b1;
                            state   <= S_IRET_PREP;
                        end

                        C_IN: begin
                            port_r <= cond[0] ? rm_val : imm_r;
                            state  <= S_IO_RD;
                        end

                        C_OUT: begin
                            port_r  <= cond[0] ? rm_val : imm_r;
                            res_val <= acc_val;
                            state   <= S_IO_WR;
                        end

                        C_FLAGOP: begin
                            flags_val_r <= 16'h0000;
                            case (cond)
                                4'd0: begin flags_mask_r <= FM_CF; flags_val_r[F_CF] <= ~f_cf; end
                                4'd1: begin flags_mask_r <= FM_CF; flags_val_r[F_CF] <= 1'b0; end
                                4'd2: begin flags_mask_r <= FM_CF; flags_val_r[F_CF] <= 1'b1; end
                                4'd3: begin flags_mask_r <= FM_IF; flags_val_r[F_IF] <= 1'b0; end
                                4'd4: begin flags_mask_r <= FM_IF; flags_val_r[F_IF] <= 1'b1; end
                                4'd5: begin flags_mask_r <= FM_DF; flags_val_r[F_DF] <= 1'b0; end
                                default: begin flags_mask_r <= FM_DF; flags_val_r[F_DF] <= 1'b1; end
                            endcase
                        end

                        C_MOV_IMM_R: begin
                            res_val <= imm_r;
                            wb_reg  <= op_reg;
                            wb_word <= word_op;
                            wb_en   <= 1'b1;
                        end

                        C_MOV_RM: begin
                            if (dir_to_reg) begin
                                res_val <= rm_val;
                                wb_reg  <= reg_field;
                                wb_word <= word_op;
                                wb_en   <= 1'b1;
                            end else begin
                                res_val <= reg_val;
                                if (rm_mem) state <= S_STORE;
                                else begin
                                    wb_reg  <= rm_field;
                                    wb_word <= word_op;
                                    wb_en   <= 1'b1;
                                end
                            end
                        end

                        C_MOV_RM_I: begin
                            res_val <= imm_r;
                            if (rm_mem) state <= S_STORE;
                            else begin
                                wb_reg  <= rm_field;
                                wb_word <= word_op;
                                wb_en   <= 1'b1;
                            end
                        end

                        C_MOV_ACC: begin
                            if (dir_to_reg) begin
                                res_val <= rm_val;
                                wb_reg  <= R_AX;
                                wb_word <= word_op;
                                wb_en   <= 1'b1;
                            end else begin
                                res_val <= acc_val;
                                state   <= S_STORE;
                            end
                        end

                        C_LEA: begin
                            res_val <= ea_r;
                            wb_reg  <= reg_field;
                            wb_word <= 1'b1;
                            wb_en   <= 1'b1;
                        end

                        C_GRP5: begin
                            case (reg_field)
                                3'd0, 3'd1: begin              // INC / DEC r/m
                                    res_val      <= alu_result;
                                    flags_val_r  <= alu_flag_bits;
                                    flags_mask_r <= FM_NOCF;
                                    if (rm_mem) state <= S_STORE;
                                    else begin
                                        wb_reg  <= rm_field;
                                        wb_word <= word_op;
                                        wb_en   <= 1'b1;
                                    end
                                end
                                3'd2, 3'd6: begin              // CALL near / PUSH r/m
                                    state <= S_GET_SP;
                                end
                                3'd3: begin                    // CALL far indirect
                                    far_cs      <= mem2_val;
                                    jump_target <= rm_val;
                                    int_ret_ip  <= ip_next;
                                    seq_far     <= 1'b1;
                                    state       <= S_INT_PREP;
                                end
                                3'd4: begin                    // JMP near indirect
                                    do_jump     <= 1'b1;
                                    jump_target <= rm_val;
                                end
                                default: begin                 // JMP far indirect
                                    far_cs      <= mem2_val;
                                    jump_target <= rm_val;
                                    state       <= S_FAR_APPLY;
                                end
                            endcase
                        end

                        C_ALU_RM, C_GRP1, C_ALU_ACC,
                        C_TEST_RM, C_TEST_ACC, C_GRP2, C_GRP3: begin
                            if (is_muldiv && (iclass == C_GRP3)) begin
                                state <= S_ALU_WAIT;
                            end else begin
                                res_val      <= alu_result;
                                res_hi       <= alu_result_hi;
                                flags_val_r  <= alu_flag_bits;
                                flags_mask_r <= mask_for_op;

                                if ((alu_op == ALU_CMP) || (alu_op == ALU_TEST)) begin
                                    wb_en <= 1'b0;             // result discarded
                                end else if (dst_is_rm && rm_mem) begin
                                    state <= S_STORE;
                                end else if (dst_is_rm) begin
                                    wb_reg  <= rm_field;
                                    wb_word <= word_op;
                                    wb_en   <= 1'b1;
                                end else if (iclass == C_ALU_ACC) begin
                                    wb_reg  <= R_AX;
                                    wb_word <= word_op;
                                    wb_en   <= 1'b1;
                                end else begin
                                    wb_reg  <= reg_field;
                                    wb_word <= word_op;
                                    wb_en   <= 1'b1;
                                end
                            end
                        end

                        C_INCDEC_R: begin
                            res_val      <= alu_result;
                            flags_val_r  <= alu_flag_bits;
                            flags_mask_r <= FM_NOCF;
                            wb_reg       <= op_reg;
                            wb_word      <= 1'b1;
                            wb_en        <= 1'b1;
                        end

                        C_PUSH_R: begin
                            push_val <= reg_val;
                            ea_r     <= sp_new - 16'd2;
                            sp_new   <= sp_new - 16'd2;
                            state    <= S_PUSH;
                        end

                        C_POP_R: begin
                            ea_r    <= sp_new;
                            wb_reg  <= op_reg;
                            wb_word <= 1'b1;
                            state   <= S_POP;
                        end

                        C_JMP_SHORT, C_JMP_NEAR: begin
                            do_jump     <= 1'b1;
                            jump_target <= rel_target;
                        end

                        C_JCC: begin
                            if (cond_true) begin
                                do_jump     <= 1'b1;
                                jump_target <= rel_target;
                            end
                        end

                        C_LOOP: state <= S_LOOP_DEC;

                        C_CALL_NEAR: begin
                            push_val    <= ip_next;      // return address
                            ea_r        <= sp_new - 16'd2;
                            sp_new      <= sp_new - 16'd2;
                            call_mode   <= 1'b1;
                            do_jump     <= 1'b1;
                            jump_target <= rel_target;
                            state       <= S_PUSH;
                        end

                        C_RET_NEAR: begin
                            ea_r      <= sp_new;
                            call_mode <= 1'b1;
                            state     <= S_POP;
                        end

                        default: state <= S_HALT;
                    endcase
                end

                S_LOOP_DEC: begin
                    if (cond[1:0] != 2'd3) begin       // JCXZ does not decrement
                        res_val <= cx_dec;
                        wb_reg  <= R_CX;
                        wb_word <= 1'b1;
                        wb_en   <= 1'b1;
                    end
                    if (loop_taken) begin
                        do_jump     <= 1'b1;
                        jump_target <= rel_target;
                    end
                    state <= S_WB;
                end

                S_ALU_WAIT: begin
                    if (!alu_busy && (iclass == C_ASCII)) begin
                        if (cond[0]) begin
                            // AAD: AL = AH*base + AL, AH = 0
                            res_val <= {8'h00, alu_result[7:0] + acc_val[7:0]};
                        end else begin
                            // AAM: AH = AL/base, AL = AL%base
                            res_val <= {alu_result[7:0], alu_result_hi[7:0]};
                        end
                        wb_reg       <= R_AX;
                        wb_word      <= 1'b1;
                        wb_en        <= 1'b1;
                        flags_val_r  <= alu_flag_bits;
                        flags_mask_r <= 16'h00D5;
                        state        <= S_WB;
                    end else if (!alu_busy && (iclass == C_IMUL_IMM)) begin
                        // Three-operand form: the result is truncated to 16
                        // bits and goes to the register field.
                        res_val      <= alu_result;
                        wb_reg       <= reg_field;
                        wb_word      <= 1'b1;
                        wb_en        <= 1'b1;
                        flags_val_r  <= alu_flag_bits;
                        flags_mask_r <= 16'h0801;
                        state        <= S_WB;
                    end else if (!alu_busy && ((alu_op == ALU_DIV) || (alu_op == ALU_IDIV)) &&
                        (alu_div_zero || !alu_byte_ok)) begin
                        // Divide by zero, or a quotient too large for the
                        // destination: both raise the type-0 divide error, and
                        // neither writes a result back.
                        int_type_r <= INT_DIV_ERR;
                        int_ret_ip <= ip_next;
                        state      <= S_INT_PREP;
                    end else if (!alu_busy) begin
                        // BOTH halves of AX are written, byte forms included.
                        // A byte MUL produces a SIXTEEN-bit product in AX, and
                        // a byte DIV puts the quotient in AL and the remainder
                        // in AH -- neither is a byte-wide result, even though
                        // the operand was one. Writing only AL here left AH
                        // holding whatever the caller had in it, which is the
                        // sort of fault that survives every ALU test (the ALU
                        // computes the right answer) and only shows up when
                        // real code multiplies with a dirty AH.
                        if ((alu_op == ALU_DIV) || (alu_op == ALU_IDIV)) begin
                            // Quotient low, remainder high.
                            res_val <= word_op ? alu_result
                                     : {alu_result_hi[7:0], alu_result[7:0]};
                        end else begin
                            // The ALU already places a byte product's full 16
                            // bits in `result`, with `result_hi` zero.
                            res_val <= alu_result;
                        end
                        res_hi       <= alu_result_hi;
                        flags_val_r  <= alu_flag_bits;
                        flags_mask_r <= mask_for_op;
                        wb_reg       <= R_AX;
                        wb_word      <= 1'b1;
                        wb_hi_reg    <= R_DX;
                        wb_hi_word   <= 1'b1;
                        wb_en        <= 1'b1;
                        // DX takes the high half only for the word forms; a
                        // byte form has already put everything in AX.
                        wb_hi_en     <= word_op;
                        state        <= S_WB;
                    end
                end

                // ---------- memory / stack ----------
                S_STORE: if (req_done) state <= wb_hi_en ? S_WB_HI : S_RETIRE;

                S_PUSH: begin
                    if (req_done)
                        state <= (iclass == C_ENTER) ? S_ENTER_LP : S_SP_UPD;
                end

                S_POP: begin
                    if (req_done) begin
                        if (call_mode) begin
                            do_jump     <= 1'b1;
                            jump_target <= req_rdata;
                            sp_new      <= sp_new + 16'd2 + imm_r;
                        end else if (pop_to_flags) begin
                            flags_val_r  <= req_rdata;
                            flags_mask_r <= FM_POPF;
                            sp_new       <= sp_new + 16'd2;
                        end else if (pop_to_sreg) begin
                            rm_val <= req_rdata;         // S_SREG_WB writes this
                            sp_new <= sp_new + 16'd2;
                        end else begin
                            res_val <= req_rdata;
                            wb_en   <= 1'b1;
                            sp_new  <= sp_new + 16'd2;
                            if (pop_to_rm && !rm_mem) begin
                                wb_reg  <= rm_field;
                                wb_word <= 1'b1;
                            end
                        end
                        state <= S_SP_UPD;
                    end
                end

                S_SP_UPD: begin
                    if (pop_to_sreg)               state <= S_SREG_WB;
                    else if (pop_to_rm && rm_mem) begin
                        ea_r  <= mem2_val;             // restore the destination
                        state <= S_STORE;
                    end
                    else if (iclass == C_PUSHA)    state <= S_RETIRE;
                    else if (iclass == C_POPA)     state <= S_RETIRE;
                    else                           state <= wb_en ? S_WB : S_RETIRE;
                end

                // ---------- writeback ----------
                S_WB: begin
                    if (iclass == C_LES_LDS) state <= S_SREG_WB;
                    else                     state <= wb_hi_en ? S_WB_HI : S_RETIRE;
                end

                S_WB_HI: state <= S_RETIRE;

                S_RETIRE: state <= do_jump ? S_REDIRECT : S_FETCH_OP;

                S_REDIRECT: begin
                    ip_next <= jump_target;
                    state   <= S_FETCH_OP;
                end

                // =================================================
                // Interrupt entry
                // Pushes FLAGS, then CS, then IP -- that order is
                // architectural, because IRET pops them back in reverse.
                // =================================================
                S_INT_PREP: begin
                    sp_new      <= rd1_data;      // current SP
                    // A far call skips the FLAGS push, so it enters the same
                    // sequence at step 1.
                    int_step    <= seq_far ? 2'd1 : 2'd0;
                    int_taken_r <= ~seq_far;
                    state       <= S_INT_SETUP;
                end

                S_INT_SETUP: begin
                    ea_r   <= sp_new - 16'd2;
                    sp_new <= sp_new - 16'd2;
                    case (int_step)
                        2'd0:    push_val <= flags;
                        2'd1:    push_val <= cs;
                        default: push_val <= int_ret_ip;
                    endcase
                    state <= S_INT_WR;
                end

                S_INT_WR: begin
                    if (req_done) begin
                        if (int_step == 2'd2) state <= S_INT_SP;
                        else begin
                            int_step <= int_step + 2'd1;
                            state    <= S_INT_SETUP;
                        end
                    end
                end

                S_INT_SP: state <= seq_far ? S_FAR_APPLY : S_INT_RDLO;

                S_INT_RDLO: begin
                    if (req_done) begin
                        jump_target <= req_rdata;    // handler offset
                        state       <= S_INT_GAP;
                    end
                end

                S_INT_GAP: state <= S_INT_RDHI;

                S_INT_RDHI: begin
                    if (req_done) begin
                        int_new_cs <= req_rdata;     // handler segment
                        state      <= S_INT_APPLY;
                    end
                end

                // CS and the flag clears are applied here; S_REDIRECT then
                // sees the updated CS when it computes the fetch address.
                S_INT_APPLY: begin
                    do_jump <= 1'b1;
                    state   <= S_REDIRECT;
                end

                // =================================================
                // IRET: pop IP, then CS, then FLAGS
                // =================================================
                S_IRET_PREP: begin
                    sp_new   <= rd1_data;
                    ea_r     <= rd1_data;
                    int_step <= 2'd0;
                    state    <= S_IRET_RD;
                end

                S_IRET_RD: begin
                    if (req_done) begin
                        ea_r <= ea_r + 16'd2;
                        case (int_step)
                            2'd0: jump_target   <= req_rdata;
                            2'd1: int_new_cs    <= req_rdata;
                            default: int_new_flags <= req_rdata;
                        endcase
                        // RETF pops two words, IRET pops three. RETF imm16 also
                        // discards that many argument bytes from the stack.
                        if (int_step == (seq_far ? 2'd1 : 2'd2)) begin
                            sp_new <= sp_new + 16'd2 + (seq_far ? imm_r : 16'd0);
                            state  <= S_IRET_APPL;
                        end else begin
                            sp_new   <= sp_new + 16'd2;
                            int_step <= int_step + 2'd1;
                            state    <= S_IRET_GAP;
                        end
                    end
                end

                S_IRET_GAP: state <= S_IRET_RD;

                S_IO_RD: begin
                    if (req_done) begin
                        res_val <= req_rdata;
                        wb_reg  <= R_AX;
                        wb_word <= word_op;
                        wb_en   <= 1'b1;
                        state   <= S_WB;
                    end
                end

                S_IO_WR: if (req_done) state <= S_RETIRE;

                S_IRET_APPL: begin
                    do_jump <= 1'b1;
                    state   <= S_REDIRECT;
                end

                // ---------- string engine ----------
                S_STR_PREP1: begin
                    si_r  <= rd0_data;
                    di_r  <= rd1_data;
                    state <= S_STR_PREP2;
                end

                S_STR_PREP2: begin
                    cx_r        <= rd0_data;
                    ax_r        <= rd1_data;
                    str_wb_step <= 2'd0;
                    state       <= S_STR_CHECK;
                end

                S_STR_CHECK: begin
                    if (rep_en && (cx_r == 16'h0000)) begin
                        // REP with CX already zero does nothing at all.
                        state <= S_RETIRE;
                    end else if (rep_en && hw_int_ready) begin
                        // Interruptible between iterations -- this is the only
                        // place an interrupt may be taken inside an
                        // instruction. The pushed address is that of the FIRST
                        // prefix, so the whole thing resumes correctly.
                        do_jump     <= 1'b1;
                        jump_target <= instr_start_ip;
                        state       <= S_RETIRE;
                    end else if (str_rd_src || str_io_rd) begin
                        state <= S_STR_RD1;
                    end else if (str_rd_dst) begin
                        state <= S_STR_RD2;
                    end else begin
                        state <= S_STR_WR;
                    end
                end

                S_STR_RD1: begin
                    if (req_done) begin
                        str_src <= req_rdata;
                        state   <= S_STR_G1;
                    end
                end

                S_STR_G1: state <= str_rd_dst ? S_STR_RD2 :
                                   (str_wr_dst || str_io_wr) ? S_STR_WR : S_STR_EXEC;

                S_STR_RD2: begin
                    if (req_done) begin
                        str_dst <= req_rdata;
                        state   <= S_STR_G2;
                    end
                end

                S_STR_G2: state <= (str_wr_dst || str_io_wr) ? S_STR_WR : S_STR_EXEC;

                S_STR_WR: if (req_done) state <= S_STR_G3;

                S_STR_G3: state <= S_STR_EXEC;

                S_STR_EXEC: begin
                    // Compare forms publish flags; LODS delivers to AX.
                    if (str_cmp) begin
                        flags_val_r  <= alu_flag_bits;
                        flags_mask_r <= FM_ALL;
                    end
                    if (str_to_ax) ax_r <= str_src;

                    if (str_adj_si) si_r <= si_r + str_delta;
                    if (str_adj_di) di_r <= di_r + str_delta;
                    if (rep_en)     cx_r <= cx_r - 16'd1;

                    str_wb_step <= 2'd0;
                    state       <= S_STR_WB;
                end

                S_STR_WB: begin
                    if (str_wb_step == 2'd3) begin
                        // Decide whether to go round again. A plain string op
                        // runs once. REP repeats while CX is non-zero; for the
                        // compare forms it additionally stops as soon as the
                        // zero flag stops matching the prefix (F3 repeats while
                        // equal, F2 while not equal).
                        if (rep_en && (cx_r != 16'h0000) &&
                            (!str_cmp || (flags_val_r[F_ZF] == rep_z)))
                            state <= S_STR_CHECK;
                        else
                            state <= S_RETIRE;
                    end else begin
                        str_wb_step <= str_wb_step + 2'd1;
                    end
                end

                // PUSHA pushes AX CX DX BX SP BP SI DI in that order, where
                // the SP value is the one from before the first push.
                S_PUSHA_RD: begin
                    push_val <= (ga_idx == 3'd4) ? sp_at_entry : rd0_data;
                    ea_r     <= sp_new - 16'd2;
                    sp_new   <= sp_new - 16'd2;
                    state    <= S_PUSHA_WR;
                end

                S_PUSHA_WR: begin
                    if (req_done) begin
                        if (ga_idx == 3'd7) state <= S_SP_UPD;
                        else begin
                            ga_idx <= ga_idx + 3'd1;
                            state  <= S_PUSHA_RD;
                        end
                    end
                end

                // POPA reverses it: DI SI BP (discard) BX DX CX AX.
                S_POPA_RD: begin
                    if (req_done) begin
                        res_val <= req_rdata;
                        ea_r    <= ea_r + 16'd2;
                        sp_new  <= sp_new + 16'd2;
                        state   <= S_POPA_WB;
                    end
                end

                S_POPA_WB: begin
                    if (ga_idx == 3'd7) state <= S_SP_UPD;
                    else begin
                        ga_idx <= ga_idx + 3'd1;
                        state  <= S_POPA_RD;
                    end
                end

                // ENTER's display copy: level-1 words are copied from the
                // caller's frame before the new frame pointer is pushed.
                S_ENTER_LP: begin
                    if (enter_level > 16'd1) begin
                        enter_level <= enter_level - 16'd1;
                        bp_val      <= bp_val - 16'd2;
                        ea_r        <= bp_val - 16'd2;
                        state       <= S_MEM_RD2;
                    end else if (enter_level == 16'd1) begin
                        enter_level <= 16'd0;
                        push_val    <= sp_at_entry;
                        ea_r        <= sp_new - 16'd2;
                        sp_new      <= sp_new - 16'd2;
                        state       <= S_PUSH;
                    end else begin
                        state <= S_ENTER_FIN;
                    end
                end

                // GRP5's CALL-near and PUSH forms need SP, which S_PREP did
                // not latch because those share a class with INC/DEC.
                S_GET_SP: begin
                    sp_new <= rd1_data - 16'd2;
                    ea_r   <= rd1_data - 16'd2;
                    if (reg_field == 3'd2) begin
                        push_val    <= ip_next;
                        do_jump     <= 1'b1;
                        jump_target <= rm_val;
                    end else begin
                        push_val <= rm_val;
                    end
                    state <= S_PUSH;
                end

                S_MEM_RD2: begin
                    if (req_done) begin
                        mem2_val <= req_rdata;
                        if (iclass == C_ENTER) begin
                            push_val <= req_rdata;
                            ea_r     <= sp_new - 16'd2;
                            sp_new   <= sp_new - 16'd2;
                            state    <= S_PUSH;
                        end else if (iclass == C_LES_LDS) begin
                            // Low word goes to the register, high word to the
                            // segment register named by the opcode.
                            res_val <= rm_val;
                            wb_reg  <= reg_field;
                            wb_word <= 1'b1;
                            wb_en   <= 1'b1;
                            state   <= S_GAP;
                        end else if (iclass == C_GRP5) begin
                            state <= S_EXEC;       // now both words are in hand
                        end else begin
                            state <= S_BOUND_CHK;
                        end
                    end
                end

                // LES/LDS: write the register, then the segment register.
                S_GAP: begin
                    rm_val <= mem2_val;          // S_SREG_WB writes this
                    state  <= S_WB;
                end

                // BOUND traps when the index lies outside [lower, upper].
                // The endpoints are in range.
                S_BOUND_CHK: begin
                    if (($signed(reg_val) < $signed(rm_val)) ||
                        ($signed(reg_val) > $signed(mem2_val))) begin
                        int_type_r <= INT_BOUND;
                        int_ret_ip <= ip_next;
                        state      <= S_INT_PREP;
                    end else begin
                        state <= S_RETIRE;
                    end
                end

                S_ENTER_FIN: begin
                    // BP takes the frame pointer and SP drops by the frame size.
                    res_val <= sp_at_entry;
                    wb_reg  <= R_BP;
                    wb_word <= 1'b1;
                    wb_en   <= 1'b1;
                    sp_new  <= sp_at_entry - imm_r;
                    state   <= S_SP_UPD;
                end

                S_FAR_APPLY: begin
                    do_jump <= 1'b1;
                    state   <= S_REDIRECT;
                end

                S_SREG_WB: begin
                    // Loading a segment register shadows the next instruction
                    // from interrupts.
                    block_int_once <= 1'b1;
                    // Writing CS changes where instructions come from, so the
                    // prefetch stream has to be restarted at the same offset
                    // in the new segment.
                    if (reg_field[1:0] == SR_CS) begin
                        do_jump     <= 1'b1;
                        jump_target <= ip_next;
                    end
                    state <= S_RETIRE;
                end

                // A halted 80186 leaves HALT when an interrupt is actually
                // SERVICED, not merely requested. hw_int_ready already encodes
                // that rule exactly: NMI wakes it regardless of IF, while a
                // maskable request with IF clear leaves it halted forever,
                // which is what makes `CLI; HLT` the standard way to stop a
                // machine dead.
                //
                // ip_next was advanced past the HLT opcode back in S_FETCH_OP
                // and nothing since has touched it, so the pushed return
                // address is the instruction AFTER the HLT and IRET resumes
                // there. That is what every x86 does, and what an idle loop --
                // `STI; HLT` waiting for the timer tick -- depends on.
                S_HALT: begin
                    if (hw_int_ready) begin
                        int_ret_ip <= ip_next;
                        if (nmi_pending) begin
                            int_type_r  <= INT_NMI;
                            nmi_pending <= 1'b0;
                        end else begin
                            int_type_r    <= intr_type;
                            int_from_intr <= 1'b1;
                        end
                        state <= S_INT_PREP;
                    end
                end

                default: state <= S_HALT;
            endcase
        end
    end

endmodule
