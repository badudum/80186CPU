// ---------------------------------------------------------------------------
// decode_len — how many bytes is the instruction at the head of the queue?
//
// Hierarchy: cpu_top -> eu -> execUnit -> decode_len
// Testbench: sim/tb_decode_len.sv, and a shadow check in sim/tb_msdos.sv
//
// WHY THIS EXISTS. The execution unit consumed the instruction stream one
// byte per cycle: a state for the opcode, another for the ModR/M, another for
// each displacement byte, another for each immediate byte. Measured on the
// MS-DOS boot those four states cost 4,545,164 cycles, of which only
// 1,030,953 was waiting for the bus -- so 3.5 million cycles, 17.7% of every
// cycle the machine ran, went on reading bytes that had ALREADY ARRIVED and
// were sitting in the prefetch queue.
//
// Knowing the whole length up front is what collapses that: the queue can
// hand over the entire instruction in one go (prefetch_queue's `pop_n`) and
// the sequencer can start at the work rather than at the reading.
//
// LENGTH DECODE IS THE HARD PART OF x86 and the reason this is a module
// rather than an expression. The length depends on the opcode, then on the
// ModR/M mod and rm fields, then on whether that opcode takes an 8- or
// 16-bit immediate, and prefixes stack ahead of all of it. Getting it wrong
// does not produce a wrong answer in one instruction -- it DESYNCHRONISES
// THE STREAM, and everything after it decodes from the wrong byte.
//
// So this does not re-derive any of it. decode.sv already computes has_modrm,
// disp_bytes and imm_bytes, and this instantiates the same module the
// sequencer uses rather than keeping a second copy of those rules that could
// drift. The prefix set is the one execUnit recognises, for the same reason.
//
// `valid` is the other half of not desynchronising: the length is only
// meaningful once enough bytes are present to have determined it. A ModR/M
// that has not arrived yet cannot tell you how many displacement bytes
// follow, so the answer is "not yet" rather than a guess.
// ---------------------------------------------------------------------------

`timescale 1ns/1ns
module decode_len
    import cpu_pkg::*;
(
    // The queue, oldest byte first, and how many of them are real.
    input  logic [7:0]  peek [0:5],
    input  logic [3:0]  count,

    output logic        valid,        // enough bytes present to know the length
    output logic [2:0]  len,          // total bytes, prefixes included
    output logic [2:0]  n_prefix,     // how many of those are prefixes
    // The shape of the instruction ABOUT to be read, as opposed to the
    // registered opcode, which still describes the previous one. This is
    // what lets the sequencer skip states that would only transition.
    output logic        op_has_modrm,
    output logic [2:0]  op_imm_bytes,
    output logic        has_rep,
    output logic        has_seg_ovr,
    output logic [1:0]  seg_ovr,

    // The instruction's fields, assembled from the queue. Extracting them
    // here rather than in the sequencer keeps the byte offsets in one place,
    // next to the length calculation that defines them -- the two have to
    // agree, and a second copy of the offset arithmetic is exactly the kind
    // of thing that drifts.
    output logic        rm_is_mem,
    output logic        rep_z,        // F3 (REPE) rather than F2 (REPNE)
    output logic [7:0]  op_byte_o,    // the opcode, past any prefixes
    output logic [7:0]  modrm_byte_o,
    output logic [15:0] disp,
    output logic [15:0] imm,
    output logic [15:0] imm2      // ptr16:16 segment half
);

    // ---- prefixes ----
    // Scanned rather than looped so the whole thing stays combinational and
    // flat. Three is enough for anything this core executes: a segment
    // override, a repeat and a lock.
    localparam int MAXPFX = 3;

    logic is_pfx [0:MAXPFX-1];
    logic is_seg [0:MAXPFX-1];
    logic is_rep [0:MAXPFX-1];

    function automatic logic pfx_seg(input logic [7:0] b);
        pfx_seg = (b == 8'h26) || (b == 8'h2E) || (b == 8'h36) || (b == 8'h3E);
    endfunction
    function automatic logic pfx_rep(input logic [7:0] b);
        pfx_rep = (b == 8'hF2) || (b == 8'hF3);
    endfunction
    function automatic logic pfx_any(input logic [7:0] b);
        pfx_any = pfx_seg(b) || pfx_rep(b) || (b == 8'hF0);   // F0 = LOCK
    endfunction

    // A prefix only counts if the byte before it was one too, so the scan
    // stops at the first non-prefix.
    always_comb begin
        is_pfx[0] = pfx_any(peek[0]);
        is_pfx[1] = is_pfx[0] && pfx_any(peek[1]);
        is_pfx[2] = is_pfx[1] && pfx_any(peek[2]);
        for (int i = 0; i < MAXPFX; i++) begin
            is_seg[i] = is_pfx[i] && pfx_seg(peek[i]);
            is_rep[i] = is_pfx[i] && pfx_rep(peek[i]);
        end
    end

    always_comb begin
        n_prefix = 3'd0;
        for (int i = 0; i < MAXPFX; i++) if (is_pfx[i]) n_prefix = n_prefix + 3'd1;
    end

    assign has_rep     = is_rep[0] || is_rep[1] || is_rep[2];
    assign has_seg_ovr = is_seg[0] || is_seg[1] || is_seg[2];

    // The LAST segment override wins, which is what a real 8086 does.
    always_comb begin
        seg_ovr = SR_DS;
        for (int i = 0; i < MAXPFX; i++)
            if (is_seg[i]) begin
                case (peek[i])
                    8'h26:   seg_ovr = SR_ES;
                    8'h2E:   seg_ovr = SR_CS;
                    8'h36:   seg_ovr = SR_SS;
                    default: seg_ovr = SR_DS;
                endcase
            end
    end

    // ---- opcode and ModR/M, at whatever offset the prefixes left them ----
    logic [7:0] op_byte, modrm_byte;
    assign op_byte    = peek[n_prefix];
    assign modrm_byte = peek[(n_prefix + 3'd1 > 3'd5) ? 3'd5 : n_prefix + 3'd1];

    logic       d_has_modrm, d_word, d_dir, d_sext, d_rm_mem;
    logic [5:0] d_iclass, d_alu;
    logic [2:0] d_imm_bytes, d_opreg, d_regf, d_rmf;
    logic [3:0] d_cond;
    logic [1:0] d_mod, d_disp_bytes;

    // The same decoder the sequencer uses. Not a copy of its rules.
    decode u_dec (
        .opcode     (op_byte),
        .modrm      (modrm_byte),
        .iclass     (d_iclass),
        .has_modrm  (d_has_modrm),
        .word_op    (d_word),
        .dir_to_reg (d_dir),
        .alu_op     (d_alu),
        .imm_bytes  (d_imm_bytes),
        .imm_sext   (d_sext),
        .op_reg     (d_opreg),
        .cond       (d_cond),
        .mod_field  (d_mod),
        .reg_field  (d_regf),
        .rm_field   (d_rmf),
        .disp_bytes (d_disp_bytes),
        .rm_is_mem  (d_rm_mem)
    );

    // ---- length, and whether we are entitled to believe it ----
    logic [3:0] need_for_len;      // bytes required before the length is known
    logic [3:0] total;

    // DISP_BYTES IS ONLY MEANINGFUL WHEN THERE IS A ModR/M. decode.sv derives
    // it from the mod field of whatever byte it is handed, and for an opcode
    // with no ModR/M that byte is the immediate, or the next instruction.
    // MOV AX,imm16 (B8) is the case that caught this: `B8 40 00` decodes the
    // 40 as mod=01 and invents a displacement byte, giving length 4 for a
    // three-byte instruction. The hand-written test used `B8 34 12`, and 34
    // decodes as mod=00 with no displacement, so it passed -- the bug is
    // DATA-DEPENDENT, which is why it took the real instruction stream to
    // find it. Every don't-care field out of decode.sv needs this treatment.
    logic [1:0] eff_disp;
    assign eff_disp = d_has_modrm ? d_disp_bytes : 2'd0;

    assign need_for_len = {1'b0, n_prefix} + (d_has_modrm ? 4'd2 : 4'd1);
    assign total        = {1'b0, n_prefix} + 4'd1
                        + (d_has_modrm ? 4'd1 : 4'd0)
                        + {2'b0, eff_disp}
                        + {1'b0, d_imm_bytes};

    // Two separate conditions, and conflating them is a bug waiting to
    // happen: the length is KNOWN once the opcode and any ModR/M have
    // arrived, but the instruction is only CONSUMABLE once all of it has.
    // A caller that pops on "known" would run past the tail.
    assign op_has_modrm = d_has_modrm;
    assign op_imm_bytes = d_imm_bytes;
    assign rm_is_mem    = d_rm_mem;
    assign op_byte_o    = op_byte;
    assign modrm_byte_o = modrm_byte;

    // REPE and REPNE differ only in which way the string comparison exits.
    // The last repeat prefix wins, as with segment overrides.
    always_comb begin
        rep_z = 1'b0;
        for (int i = 0; i < MAXPFX; i++)
            if (is_rep[i]) rep_z = (peek[i] == 8'hF3);
    end

    // ---- field extraction ----
    // Offsets follow the same layout the length is built from: prefixes,
    // opcode, ModR/M, displacement, immediate.
    logic [3:0] disp_at, imm_at;
    assign disp_at = {1'b0, n_prefix} + 4'd1 + (d_has_modrm ? 4'd1 : 4'd0);
    assign imm_at  = disp_at + {2'b0, eff_disp};

    function automatic logic [7:0] at(input logic [3:0] i);
        at = (i > 4'd5) ? 8'h00 : peek[i[2:0]];
    endfunction

    // The bytes are pulled out into named signals rather than bit-selecting
    // the function result directly: `at(i)[7]` is legal SystemVerilog and
    // ModelSim accepts it, but Quartus rejects a bit-select applied to a
    // function call. Naming them is clearer anyway.
    // always_comb, NOT continuous assigns. `at()` reads the `peek` array,
    // which is not one of its arguments, so `assign b = at(i)` is sensitive
    // only to `i` -- when new bytes arrive and the offset happens to be
    // unchanged, the byte goes stale and the instruction decodes with the
    // wrong displacement or immediate. always_comb infers sensitivity from
    // everything read, the array included.
    //
    // This is a SIMULATION-ONLY failure: synthesis builds the combinational
    // logic regardless of sensitivity lists, so the board booted perfectly
    // while tb_top and tb_bios both failed. That divergence is what made it
    // look like a cache bug for several rounds.
    logic [7:0] disp_b0, disp_b1, imm_b0, imm_b1, imm_b2, imm_b3;
    always_comb begin
        disp_b0 = at(disp_at);
        disp_b1 = at(disp_at + 4'd1);
        imm_b0  = at(imm_at);
        imm_b1  = at(imm_at + 4'd1);
        imm_b2  = at(imm_at + 4'd2);
        imm_b3  = at(imm_at + 4'd3);
    end

    always_comb begin
        // A one-byte displacement is SIGNED; a two-byte one is taken whole.
        case (eff_disp)
            2'd1:    disp = {{8{disp_b0[7]}}, disp_b0};
            2'd2:    disp = {disp_b1, disp_b0};
            default: disp = 16'h0000;
        endcase

        // imm_sext is the same distinction for immediates: an 8-bit
        // immediate on a 16-bit operation is sign-extended.
        case (d_imm_bytes)
            3'd1:    imm = d_sext ? {{8{imm_b0[7]}}, imm_b0}
                                  : {8'h00, imm_b0};
            3'd2,
            3'd4:    imm = {imm_b1, imm_b0};
            default: imm = 16'h0000;
        endcase

        // Only the ptr16:16 forms have a second immediate.
        imm2 = (d_imm_bytes == 3'd4) ? {imm_b3, imm_b2} : 16'h0000;
    end

    assign valid = (count >= need_for_len) && (count >= total) && (total <= 4'd6);
    assign len   = total[2:0];

endmodule
