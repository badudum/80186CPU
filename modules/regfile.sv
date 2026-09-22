// ---------------------------------------------------------------------------
// regfile — 80186 register file (GP, pointer, segment, IP, FLAGS).
//
// Hierarchy: cpu_top -> eu -> regfile
// Reference: learnings/01-programming-model.md
// Testbench: sim/tb_regfile.sv  (run with ./sim/run.sh tb_regfile)
//
// Named `regfile`, not `reg`: `reg` is a reserved Verilog/SystemVerilog
// keyword and `module reg` is illegal, so Quartus rejects it outright. Keep
// the name away from keywords if this file is ever split or renamed again.
//
// REGISTER ENCODINGS -- these are the x86 ModR/M `reg` field values, not an
// arbitrary local numbering. decode.sv can pass the raw field straight in.
//
//   word (rd*_word = 1)      byte (rd*_word = 0)      segment
//   0 AX   4 SP              0 AL   4 AH              0 ES
//   1 CX   5 BP              1 CL   5 CH              1 CS
//   2 DX   6 SI              2 DL   6 DH              2 SS
//   3 BX   7 DI              3 BL   7 BH              3 DS
//
// Note the byte encoding is NOT "low half of the word register with the same
// number": index[1:0] picks AX/CX/DX/BX and index[2] picks the high half. So
// byte 4 is AH, not the low byte of SP. SP/BP/SI/DI have no byte access at
// all, which is why they are unreachable through the byte encoding.
//
// FLAGS BIT LAYOUT (bit positions are architectural, do not renumber)
//   0 CF | 2 PF | 4 AF | 6 ZF | 7 SF | 8 TF | 9 IF | 10 DF | 11 OF
//   Bit 1 always reads 1; bits 3, 5 and 12-15 always read 0. Those reserved
//   bits have no storage here, so they cannot be corrupted by a POPF/IRET
//   that supplies garbage in them.
//
// FLAG WRITES ARE MASKED, and that is the whole point: x86 does not update
// every flag on every instruction. The caller supplies flags_wdata plus
// flags_wmask, where a 1 bit means "update this flag". This is what lets
// execUnit implement "INC does not touch CF", "rotates touch only CF and OF"
// and "NOT touches nothing" without this module needing to know which
// instruction is executing. Useful masks:
//     16'h08D5  all six ALU flags (OF SF ZF AF PF CF)
//     16'h0200  IF only            (CLI / STI)
//     16'h0400  DF only            (CLD / STD)
//     16'h0300  IF and TF          (cleared on interrupt entry)
//     16'h0FD5  every defined flag (POPF / IRET)
//
// READ TIMING: all read ports are combinational, so a read in the same cycle
// as a write to the same register returns the OLD value. The EU reads
// operands and writes results in different micro-op steps, so this is not a
// hazard -- but do not build a micro-op that relies on same-cycle forwarding.
//
// TODO:
//   [x] FLAGS bit positions, register encodings and the instruction classes
//       now live in cpu_pkg.sv, which decode.sv, execUnit.sv and this module
//       all import, so none of them hardcode matching numbers.
//   [ ] CS/IP have dedicated always-on read outputs because instruction fetch
//       runs concurrently with execution. If SP access ever contends on the
//       general read ports during push/pop sequencing, give it the same
//       treatment rather than adding a third general read port.
//   [ ] `seg_written` pulses whenever a segment register is written, for the
//       interrupt-acceptance rule that blocks interrupts for one instruction
//       after a segment load (keeps SS:SP atomic, see cpu_top.sv). The
//       one-instruction delay itself belongs in the EU, not here.
// ---------------------------------------------------------------------------

`timescale 1ns/1ns
module regfile (
    input  logic        clk,
    input  logic        rst_n,

    // ---- general-purpose read ports (combinational) ----
    input  logic [2:0]  rd0_sel,
    input  logic        rd0_word,     // 1 = 16-bit register, 0 = 8-bit register
    output logic [15:0] rd0_data,     // byte reads are zero-extended
    input  logic [2:0]  rd1_sel,
    input  logic        rd1_word,
    output logic [15:0] rd1_data,

    // ---- general-purpose write port ----
    input  logic [2:0]  wr_sel,
    input  logic        wr_word,
    input  logic        wr_en,
    input  logic [15:0] wr_data,      // byte writes use wr_data[7:0]

    // ---- segment registers ----
    input  logic [1:0]  sreg_rd_sel,
    output logic [15:0] sreg_rd_data,
    input  logic [1:0]  sreg_wr_sel,
    input  logic        sreg_wr_en,
    input  logic [15:0] sreg_wr_data,
    output logic        seg_written,  // pulses on any segment write

    // ---- always-available CS:IP (instruction fetch runs concurrently) ----
    output logic [15:0] cs,
    output logic [15:0] ip,
    input  logic        ip_we,
    input  logic [15:0] ip_wdata,

    // ---- FLAGS ----
    output logic [15:0] flags,
    input  logic [15:0] flags_wdata,
    input  logic [15:0] flags_wmask
);

    // Word-register indices (also the ModR/M reg field encoding).
    localparam logic [2:0] R_AX = 3'd0, R_CX = 3'd1, R_DX = 3'd2, R_BX = 3'd3;
    localparam logic [2:0] R_SP = 3'd4, R_BP = 3'd5, R_SI = 3'd6, R_DI = 3'd7;

    // Segment indices.
    localparam logic [1:0] S_ES = 2'd0, S_CS = 2'd1, S_SS = 2'd2, S_DS = 2'd3;

    // FLAGS bit positions (shared with execUnit's mask building).
    localparam int B_CF = cpu_pkg::F_CF, B_PF = cpu_pkg::F_PF;
    localparam int B_AF = cpu_pkg::F_AF, B_ZF = cpu_pkg::F_ZF;
    localparam int B_SF = cpu_pkg::F_SF, B_TF = cpu_pkg::F_TF;
    localparam int B_IF = cpu_pkg::F_IF, B_DF = cpu_pkg::F_DF;
    localparam int B_OF = cpu_pkg::F_OF;

    logic [15:0] gpr  [0:7];
    logic [15:0] sreg [0:3];
    logic [15:0] ip_r;

    logic f_cf, f_pf, f_af, f_zf, f_sf, f_tf, f_if, f_df, f_of;

    // =====================================================================
    // Reads
    // =====================================================================
    // Byte encoding: [1:0] selects AX/CX/DX/BX, [2] selects the high half.
    //
    // These are always_comb, NOT `assign rdN_data = read_port(...)`. A function
    // called from a continuous assignment is only sensitive to its ARGUMENTS,
    // so `gpr` -- read inside the function but not passed in -- would not
    // retrigger the assignment. The read port then holds a stale value until
    // the select happens to change. Synthesis infers the correct mux either
    // way, so this shows up only in simulation, which is the worst kind of
    // bug to leave in. always_comb is sensitive to everything read inside it,
    // including through called functions.
    // Byte registers alias the low four words: AL/AH -> AX, CL/CH -> CX, and so
    // on, so bit 2 of the encoding picks the half, not the word. The index is
    // computed OUTSIDE the function and passed in: a function-local used as an
    // array index makes Quartus emit a spurious "index expression is not wide
    // enough" warning even at the correct width, while the identical
    // module-level signal on the write side does not.
    logic [2:0] rd0_bidx, rd1_bidx;
    assign rd0_bidx = {1'b0, rd0_sel[1:0]};
    assign rd1_bidx = {1'b0, rd1_sel[1:0]};

    function automatic [15:0] read_port (input logic [2:0] sel,
                                         input logic [2:0] bidx,
                                         input logic       is_word);
        if (is_word)
            read_port = gpr[sel];
        else
            read_port = sel[2] ? {8'h00, gpr[bidx][15:8]}
                               : {8'h00, gpr[bidx][7:0]};
    endfunction

    always_comb rd0_data = read_port(rd0_sel, rd0_bidx, rd0_word);
    always_comb rd1_data = read_port(rd1_sel, rd1_bidx, rd1_word);

    assign sreg_rd_data = sreg[sreg_rd_sel];
    assign cs           = sreg[S_CS];
    assign ip           = ip_r;

    assign flags = {4'b0000,          // 15:12 reserved, read as 0
                    f_of,             // 11
                    f_df,             // 10
                    f_if,             //  9
                    f_tf,             //  8
                    f_sf,             //  7
                    f_zf,             //  6
                    1'b0,             //  5 reserved
                    f_af,             //  4
                    1'b0,             //  3 reserved
                    f_pf,             //  2
                    1'b1,             //  1 always reads 1
                    f_cf};            //  0

    assign seg_written = sreg_wr_en;

    // =====================================================================
    // Write-value merge
    // =====================================================================
    // A byte write has to preserve the other half of the register. The merge
    // happens here so the sequential block below has a single, whole-word
    // assignment target, rather than assigning a part-select of an array
    // element (`gpr[i][15:8] <= ...`) in one branch and a different
    // part-select in another. Same result, but one obvious assignment is
    // easier to read and to trust than two conditional partial ones.
    logic [2:0]  wr_idx;
    logic [15:0] wr_value;

    always_comb begin
        wr_idx = wr_word ? wr_sel : {1'b0, wr_sel[1:0]};
        // Merging a byte write needs the *other* half of the same word, which
        // always lives at the byte-aliased index regardless of wr_word.
        if (wr_word)
            wr_value = wr_data;
        else if (wr_sel[2])
            wr_value = {wr_data[7:0], gpr[wr_idx][7:0]};   // high half
        else
            wr_value = {gpr[wr_idx][15:8], wr_data[7:0]};  // low half
    end

    // =====================================================================
    // Writes
    // =====================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gpr[R_AX] <= 16'h0000;
            gpr[R_CX] <= 16'h0000;
            gpr[R_DX] <= 16'h0000;
            gpr[R_BX] <= 16'h0000;
            gpr[R_SP] <= 16'h0000;
            gpr[R_BP] <= 16'h0000;
            gpr[R_SI] <= 16'h0000;
            gpr[R_DI] <= 16'h0000;

            // CS=FFFF and IP=0000 are required: together they produce the
            // first instruction fetch at physical FFFF0h.
            sreg[S_ES] <= 16'h0000;
            sreg[S_CS] <= 16'hFFFF;
            sreg[S_SS] <= 16'h0000;
            sreg[S_DS] <= 16'h0000;
            ip_r       <= 16'h0000;

            f_cf <= 1'b0; f_pf <= 1'b0; f_af <= 1'b0;
            f_zf <= 1'b0; f_sf <= 1'b0; f_tf <= 1'b0;
            f_if <= 1'b0; f_df <= 1'b0; f_of <= 1'b0;
        end else begin
            if (wr_en) gpr[wr_idx] <= wr_value;

            if (sreg_wr_en) sreg[sreg_wr_sel] <= sreg_wr_data;

            if (ip_we) ip_r <= ip_wdata;

            if (flags_wmask[B_CF]) f_cf <= flags_wdata[B_CF];
            if (flags_wmask[B_PF]) f_pf <= flags_wdata[B_PF];
            if (flags_wmask[B_AF]) f_af <= flags_wdata[B_AF];
            if (flags_wmask[B_ZF]) f_zf <= flags_wdata[B_ZF];
            if (flags_wmask[B_SF]) f_sf <= flags_wdata[B_SF];
            if (flags_wmask[B_TF]) f_tf <= flags_wdata[B_TF];
            if (flags_wmask[B_IF]) f_if <= flags_wdata[B_IF];
            if (flags_wmask[B_DF]) f_df <= flags_wdata[B_DF];
            if (flags_wmask[B_OF]) f_of <= flags_wdata[B_OF];
        end
    end

endmodule
