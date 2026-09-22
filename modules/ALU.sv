// ---------------------------------------------------------------------------
// ALU — arithmetic, logic, shifts/rotates, and multicycle multiply/divide.
//
// Hierarchy: cpu_top -> eu -> ALU
// Reference: learnings/03-instruction-set.md (timing + signedness),
//            learnings/01-programming-model.md (FLAGS bit semantics)
//
// STRUCTURE
//   Simple ops (add/sub/logic/shift/rotate) are COMBINATIONAL: drive alu_op,
//   a, b and shift_cnt, and `result` plus the flags are valid in the same
//   cycle. `busy` stays low and `start` is ignored.
//
//   MUL/IMUL/DIV/IDIV are MULTICYCLE and use a start/busy handshake:
//     - pulse `start` for one cycle with alu_op set
//     - `busy` is high from that same cycle until the result is valid
//     - read `result` / `result_hi` on the first cycle `busy` is low again
//   Multiply takes 2 cycles; divide takes ~18 (byte) or ~34 (word).
//
// MULTIPLY USES `*`, NOT A SHIFT-ADD LOOP
//   An earlier draft here iterated a shift-add multiplier and had four
//   separate bugs in ~5 lines. Cyclone V has hardened DSP multiplier blocks,
//   so `*` infers a 16x16 multiply essentially for free and cannot get the
//   algorithm wrong. Divide still has to iterate (there is no hardware
//   divider), so the restoring-division loop below is genuine sequential
//   logic.
//   The cost is cycle-shape fidelity: the real 80186 takes 26-43 clocks for
//   MUL. Nothing observable depends on that, and if cycle-accurate timing is
//   ever wanted, microcode.sv can simply stall for the datasheet count rather
//   than the ALU faking it. See learnings/03-instruction-set.md for the table.
//
// FLAGS: WHICH OPS DEFINE WHICH
//   This module always drives all six flag outputs, but x86 does NOT update
//   every flag on every instruction. The EU must apply a per-instruction write
//   mask (see execUnit.sv) -- notably:
//     - NOT affects no flags at all
//     - INC/DEC leave CF unchanged
//     - rotates (ROL/ROR/RCL/RCR) affect only CF and OF, not ZF/SF/PF
//     - a shift/rotate with count 0 affects no flags
//     - OF is architecturally defined only for shift/rotate count == 1
//     - MUL/IMUL define CF and OF; ZF/SF/PF are undefined
//     - DIV/IDIV leave all flags undefined
//   Where a flag is undefined or unchanged this module drives a harmless
//   value (usually the current cf_in or 0) rather than X.
//
// PORT NOTE: `byte_ok` had no driver and no documented meaning. It is defined
//   here as "the quotient fits the destination width", i.e. the divide did not
//   overflow, so a complete divide error is (div_zero || !byte_ok). The name
//   is poor for that -- `quot_fits` or `div_ovf` would be clearer -- but it is
//   kept so nothing already written against it breaks.
//
// TODO:
//   [x] Decimal/ASCII adjust (DAA, DAS, AAA, AAS, AAM, AAD). RESOLVED: these
//       are sequenced in execUnit.sv rather than added as ALU ops. The ALU
//       already produces AF and CF correctly, which is all those instructions
//       consume, so no new datapath was needed -- AAM and AAD reuse the
//       existing divide and multiply. See C_BCD and C_ASCII in cpu_pkg.sv.
//   [x] Verified in simulation: 131 self-checking assertions covering every
//       op, both widths, and the flag corner cases (signed overflow, AF,
//       parity, shift/rotate CF+OF, rotate-by-full-width identity, signed
//       divide sign rules, divide-by-zero, quotient overflow). All passing.
//       Run with the ModelSim bundled in Quartus -- see the note below.
//   [ ] Extend that testbench as ops are added, and cross-check against a
//       known-good 8086 emulator if one is handy. Flag corner cases are the
//       classic silent-failure area in a from-scratch x86.
//   [ ] Divide timing is ~18/34 cycles vs the datasheet's 29/38 (DIV) and
//       50/67 (IDIV). Close enough to be plausible, but not exact.
//
// RUNNING THE TESTBENCH
//     ./sim/run.sh
//   The testbench is sim/tb_alu.sv. The script drives the ModelSim ASE that
//   ships with Quartus, working around the two things that stop it running
//   out of the box (a missing `linux_rh60` launcher path and the 32-bit
//   binaries wanting the old libncurses.so.5 SONAME) without modifying the
//   Quartus install. Details are in sim/run.sh. Build output goes to a
//   scratch directory, not into the repo.
// ---------------------------------------------------------------------------

`timescale 1ns/1ns
module ALU (
    input  logic        clk,
    input  logic        rst_n,
    input  logic [5:0]  alu_op,
    input  logic        word,        // 1 = 16-bit operation, 0 = 8-bit
    input  logic        start,       // pulse to begin a multicycle op
    input  logic [15:0] a,
    input  logic [15:0] a_hi,        // high half of the dividend (DX) for word DIV/IDIV
    input  logic [15:0] b,
    input  logic        cf_in,
    input  logic [4:0]  shift_cnt,
    output logic [15:0] result,
    output logic [15:0] result_hi,   // MUL high half (DX) / DIV remainder
    output logic        div_zero,
    output logic        byte_ok,     // quotient fits destination (see header)
    output logic        cf,
    output logic        pf,
    output logic        af,
    output logic        zf,
    output logic        sf,
    output logic        of,
    output logic        busy
);

    // ---- operation encodings ----
    // Values live in cpu_pkg so decode.sv emits exactly what this module
    // decodes. These are short local aliases for readability below.
    localparam logic [5:0] ADD  = cpu_pkg::ALU_ADD;
    localparam logic [5:0] SUB  = cpu_pkg::ALU_SUB;
    localparam logic [5:0] CMP  = cpu_pkg::ALU_CMP;
    localparam logic [5:0] INC  = cpu_pkg::ALU_INC;
    localparam logic [5:0] DEC  = cpu_pkg::ALU_DEC;
    localparam logic [5:0] NEG  = cpu_pkg::ALU_NEG;
    localparam logic [5:0] AND  = cpu_pkg::ALU_AND;
    localparam logic [5:0] OR   = cpu_pkg::ALU_OR;
    localparam logic [5:0] XOR  = cpu_pkg::ALU_XOR;
    localparam logic [5:0] NOT  = cpu_pkg::ALU_NOT;
    localparam logic [5:0] TEST = cpu_pkg::ALU_TEST;
    localparam logic [5:0] SHL  = cpu_pkg::ALU_SHL;
    localparam logic [5:0] SHR  = cpu_pkg::ALU_SHR;
    localparam logic [5:0] SAR  = cpu_pkg::ALU_SAR;
    localparam logic [5:0] ROL  = cpu_pkg::ALU_ROL;
    localparam logic [5:0] ROR  = cpu_pkg::ALU_ROR;
    localparam logic [5:0] RCL  = cpu_pkg::ALU_RCL;
    localparam logic [5:0] RCR  = cpu_pkg::ALU_RCR;
    localparam logic [5:0] MUL  = cpu_pkg::ALU_MUL;
    localparam logic [5:0] IMUL = cpu_pkg::ALU_IMUL;
    localparam logic [5:0] DIV  = cpu_pkg::ALU_DIV;
    localparam logic [5:0] IDIV = cpu_pkg::ALU_IDIV;
    localparam logic [5:0] ADC  = cpu_pkg::ALU_ADC;
    localparam logic [5:0] SBB  = cpu_pkg::ALU_SBB;

    localparam logic [1:0] S_IDLE = 2'd0;
    localparam logic [1:0] S_DIV  = 2'd1;
    localparam logic [1:0] S_DONE = 2'd2;

    // =====================================================================
    // Operand normalization
    // Byte ops zero the upper half so carry/zero detection can key off a
    // single bit position selected by `word`.
    // =====================================================================
    logic [15:0] a_op, b_op;
    assign a_op = word ? a : {8'h00, a[7:0]};
    assign b_op = word ? b : {8'h00, b[7:0]};

    logic sign_a, sign_b;
    assign sign_a = word ? a_op[15] : a_op[7];
    assign sign_b = word ? b_op[15] : b_op[7];

    // =====================================================================
    // Add / subtract
    // =====================================================================
    logic cin;
    always_comb begin
        case (alu_op)
            ADC, SBB: cin = cf_in;
            default:  cin = 1'b0;
        endcase
    end

    logic [16:0] sum17, dif17, inc17, dec17, neg17;
    assign sum17 = {1'b0, a_op} + {1'b0, b_op} + {16'b0, cin};
    assign dif17 = {1'b0, a_op} - {1'b0, b_op} - {16'b0, cin};
    assign inc17 = {1'b0, a_op} + 17'd1;
    assign dec17 = {1'b0, a_op} - 17'd1;
    assign neg17 = 17'd0 - {1'b0, a_op};

    logic sign_sum, sign_dif, sign_inc, sign_dec;
    assign sign_sum = word ? sum17[15] : sum17[7];
    assign sign_dif = word ? dif17[15] : dif17[7];
    assign sign_inc = word ? inc17[15] : inc17[7];
    assign sign_dec = word ? dec17[15] : dec17[7];

    // Auxiliary carry is the carry out of bit 3 (BCD adjust support).
    logic [4:0] af_sum, af_dif;
    assign af_sum = {1'b0, a_op[3:0]} + {1'b0, b_op[3:0]} + {4'b0, cin};
    assign af_dif = {1'b0, a_op[3:0]} - {1'b0, b_op[3:0]} - {4'b0, cin};

    // =====================================================================
    // Shifts and rotates
    // =====================================================================
    // Left shift extended wide enough that the last bit shifted out lands
    // just above the MSB, which is exactly what CF must capture.
    logic [32:0] shl_ext;
    assign shl_ext = word ? ({17'h0, a_op}      << shift_cnt)
                          : ({25'h0, a_op[7:0]} << shift_cnt);

    // Right shifts pad on the right by one bit so the last bit shifted out
    // ends up in bit 0.
    logic [16:0] shr_ext, sar_ext;
    assign shr_ext = word ? ({a_op, 1'b0}                    >> shift_cnt)
                          : ({8'h00, a_op[7:0], 1'b0}        >> shift_cnt);
    assign sar_ext = word ? ($signed({a_op, 1'b0})                     >>> shift_cnt)
                          : ($signed({{8{a_op[7]}}, a_op[7:0], 1'b0})  >>> shift_cnt);

    // Rotate counts reduced modulo the operand width. Modulo 16 and 8 are
    // just bit slices; 17 and 9 need conditional subtraction (shift_cnt is
    // already masked to 5 bits by the port width, matching the 80186's
    // mod-32 rule).
    logic [4:0] n16, n8, n17, n9;
    assign n16 = {1'b0, shift_cnt[3:0]};
    assign n8  = {2'b0, shift_cnt[2:0]};
    assign n17 = (shift_cnt >= 5'd17) ? (shift_cnt - 5'd17) : shift_cnt;
    always_comb begin
        if      (shift_cnt >= 5'd27) n9 = shift_cnt - 5'd27;
        else if (shift_cnt >= 5'd18) n9 = shift_cnt - 5'd18;
        else if (shift_cnt >= 5'd9)  n9 = shift_cnt - 5'd9;
        else                         n9 = shift_cnt;
    end

    logic [15:0] rol_w, ror_w;
    logic [7:0]  rol_b, ror_b;
    assign rol_w = (a_op << n16) | (a_op >> (6'd16 - {1'b0, n16}));
    assign ror_w = (a_op >> n16) | (a_op << (6'd16 - {1'b0, n16}));
    assign rol_b = (a_op[7:0] << n8) | (a_op[7:0] >> (6'd8 - {1'b0, n8}));
    assign ror_b = (a_op[7:0] >> n8) | (a_op[7:0] << (6'd8 - {1'b0, n8}));

    // Rotate through carry operates on a (width+1)-bit value that includes CF.
    logic [16:0] rcl_w, rcr_w;
    logic [8:0]  rcl_b, rcr_b;
    assign rcl_w = ({cf_in, a_op} << n17) | ({cf_in, a_op} >> (6'd17 - {1'b0, n17}));
    assign rcr_w = ({cf_in, a_op} >> n17) | ({cf_in, a_op} << (6'd17 - {1'b0, n17}));
    assign rcl_b = ({cf_in, a_op[7:0]} << n9) | ({cf_in, a_op[7:0]} >> (6'd9 - {1'b0, n9}));
    assign rcr_b = ({cf_in, a_op[7:0]} >> n9) | ({cf_in, a_op[7:0]} << (6'd9 - {1'b0, n9}));

    // =====================================================================
    // Multiply (DSP-inferred)
    // =====================================================================
    logic [31:0] prod_u;
    assign prod_u = {16'h0, a_op} * {16'h0, b_op};

    logic signed [15:0] a_sx, b_sx;
    assign a_sx = word ? $signed(a_op) : $signed({{8{a_op[7]}}, a_op[7:0]});
    assign b_sx = word ? $signed(b_op) : $signed({{8{b_op[7]}}, b_op[7:0]});

    logic signed [31:0] prod_s;
    assign prod_s = a_sx * b_sx;

    // =====================================================================
    // Divide operand preparation (magnitudes + signs)
    // =====================================================================
    logic is_mul_op, is_div_op, is_signed_div, is_multicycle;
    assign is_mul_op     = (alu_op == MUL) || (alu_op == IMUL);
    assign is_div_op     = (alu_op == DIV) || (alu_op == IDIV);
    assign is_signed_div = (alu_op == IDIV);
    assign is_multicycle = is_mul_op || is_div_op;

    logic sign_dvd, sign_dsr, divisor_zero;
    assign sign_dvd     = word ? a_hi[15] : a[15];
    assign sign_dsr     = word ? b[15]    : b[7];
    assign divisor_zero = word ? (b == 16'h0000) : (b[7:0] == 8'h00);

    logic [31:0] dvd_nat, dvd_abs, dvd_aligned;
    assign dvd_nat = word ? {a_hi, a}
                          : (is_signed_div ? {{16{a[15]}}, a} : {16'h0000, a});
    assign dvd_abs = (is_signed_div && sign_dvd) ? (~dvd_nat + 32'd1) : dvd_nat;
    // Left-align so the iteration can always consume bit 31 first.
    assign dvd_aligned = word ? dvd_abs : {dvd_abs[15:0], 16'h0000};

    logic [15:0] dsr_nat, dsr_abs;
    assign dsr_nat = word ? b
                          : (is_signed_div ? {{8{b[7]}}, b[7:0]} : {8'h00, b[7:0]});
    assign dsr_abs = (is_signed_div && sign_dsr) ? (~dsr_nat + 16'd1) : dsr_nat;

    // =====================================================================
    // Multicycle sequencer
    // =====================================================================
    logic [1:0]  state;
    logic [15:0] mul_lo, mul_hi;
    logic [31:0] dvd_reg, quo_reg;
    logic [16:0] rem_reg;
    logic [15:0] dsr_reg;
    logic [5:0]  cnt_reg;
    logic        q_neg, r_neg, dz_reg, sdiv_reg, dword_reg;

    // One restoring-division step: pull the next dividend bit into the
    // remainder, subtract the divisor if it fits.
    logic [16:0] rem_shifted;
    logic        sub_fits;
    assign rem_shifted = {rem_reg[15:0], dvd_reg[31]};
    assign sub_fits    = (rem_shifted >= {1'b0, dsr_reg});

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            mul_lo    <= 16'h0000;
            mul_hi    <= 16'h0000;
            dvd_reg   <= 32'h0;
            quo_reg   <= 32'h0;
            rem_reg   <= 17'h0;
            dsr_reg   <= 16'h0000;
            cnt_reg   <= 6'd0;
            q_neg     <= 1'b0;
            r_neg     <= 1'b0;
            dz_reg    <= 1'b0;
            sdiv_reg  <= 1'b0;
            dword_reg <= 1'b0;
        end else begin
            case (state)
                S_IDLE: begin
                    if (start && is_mul_op) begin
                        // Byte forms leave the full 16-bit product in `result`
                        // (it lands in AX), so the high half is only used for
                        // word operations.
                        if (alu_op == IMUL) begin
                            mul_lo <= prod_s[15:0];
                            mul_hi <= word ? prod_s[31:16] : 16'h0000;
                        end else begin
                            mul_lo <= prod_u[15:0];
                            mul_hi <= word ? prod_u[31:16] : 16'h0000;
                        end
                        dz_reg <= 1'b0;
                        state  <= S_DONE;
                    end else if (start && is_div_op) begin
                        sdiv_reg  <= is_signed_div;
                        dword_reg <= word;
                        if (divisor_zero) begin
                            dz_reg  <= 1'b1;
                            quo_reg <= 32'h0;
                            rem_reg <= 17'h0;
                            state   <= S_DONE;
                        end else begin
                            dz_reg  <= 1'b0;
                            dvd_reg <= dvd_aligned;
                            dsr_reg <= dsr_abs;
                            rem_reg <= 17'h0;
                            quo_reg <= 32'h0;
                            cnt_reg <= word ? 6'd32 : 6'd16;
                            q_neg   <= is_signed_div && (sign_dvd ^ sign_dsr);
                            r_neg   <= is_signed_div && sign_dvd;
                            state   <= S_DIV;
                        end
                    end
                end

                S_DIV: begin
                    rem_reg <= sub_fits ? (rem_shifted - {1'b0, dsr_reg}) : rem_shifted;
                    quo_reg <= {quo_reg[30:0], sub_fits};
                    dvd_reg <= {dvd_reg[30:0], 1'b0};
                    cnt_reg <= cnt_reg - 6'd1;
                    if (cnt_reg == 6'd1) state <= S_DONE;
                end

                S_DONE: state <= S_IDLE;

                default: state <= S_IDLE;
            endcase
        end
    end

    assign busy = (state != S_IDLE) || (start && is_multicycle);

    // =====================================================================
    // Divide result: reapply signs, check that the quotient fits
    // =====================================================================
    logic [15:0] quo_signed, rem_signed;
    always_comb begin
        if (dword_reg) begin
            quo_signed = q_neg ? (~quo_reg[15:0] + 16'd1) : quo_reg[15:0];
            rem_signed = r_neg ? (~rem_reg[15:0] + 16'd1) : rem_reg[15:0];
        end else begin
            quo_signed = q_neg ? {8'h00, (~quo_reg[7:0] + 8'd1)} : {8'h00, quo_reg[7:0]};
            rem_signed = r_neg ? {8'h00, (~rem_reg[7:0] + 8'd1)} : {8'h00, rem_reg[7:0]};
        end
    end

    // The 80186 widened the signed range by one versus the 8086: the
    // most-negative value (8000h / 80h) is a permitted quotient.
    logic quot_overflow;
    always_comb begin
        if (sdiv_reg) begin
            if (dword_reg)
                quot_overflow = (quo_reg[31:16] != 16'h0000) ||
                                (q_neg ? (quo_reg[15:0] > 16'h8000)
                                       : (quo_reg[15:0] > 16'h7FFF));
            else
                quot_overflow = (quo_reg[31:8] != 24'h000000) ||
                                (q_neg ? (quo_reg[7:0] > 8'h80)
                                       : (quo_reg[7:0] > 8'h7F));
        end else begin
            quot_overflow = dword_reg ? (quo_reg[31:16] != 16'h0000)
                                      : (quo_reg[31:8]  != 24'h000000);
        end
    end

    assign div_zero = dz_reg;
    assign byte_ok  = (is_div_op && !dz_reg) ? !quot_overflow : 1'b1;

    // =====================================================================
    // Combinational result + flags for the single-cycle operations
    // =====================================================================
    logic [15:0] comb_result;
    logic        cf_c, of_c, af_c;

    always_comb begin
        comb_result = a_op;
        cf_c        = cf_in;   // "unchanged" default; EU masks where undefined
        of_c        = 1'b0;
        af_c        = 1'b0;

        case (alu_op)
            ADD, ADC: begin
                comb_result = sum17[15:0];
                cf_c        = word ? sum17[16] : sum17[8];
                af_c        = af_sum[4];
                of_c        = (sign_a == sign_b) && (sign_sum != sign_a);
            end

            SUB, SBB, CMP: begin
                comb_result = dif17[15:0];
                cf_c        = word ? dif17[16] : dif17[8];
                af_c        = af_dif[4];
                of_c        = (sign_a != sign_b) && (sign_dif != sign_a);
            end

            INC: begin
                comb_result = inc17[15:0];
                af_c        = (a_op[3:0] == 4'hF);
                of_c        = (sign_a == 1'b0) && (sign_inc == 1'b1);
            end

            DEC: begin
                comb_result = dec17[15:0];
                af_c        = (a_op[3:0] == 4'h0);
                of_c        = (sign_a == 1'b1) && (sign_dec == 1'b0);
            end

            NEG: begin
                comb_result = neg17[15:0];
                cf_c        = (a_op != 16'h0000);
                af_c        = (a_op[3:0] != 4'h0);
                // Only the most-negative value cannot be negated.
                of_c        = word ? (a_op == 16'h8000) : (a_op[7:0] == 8'h80);
            end

            AND, TEST: begin
                comb_result = a_op & b_op;
                cf_c        = 1'b0;
                of_c        = 1'b0;
            end

            OR: begin
                comb_result = a_op | b_op;
                cf_c        = 1'b0;
                of_c        = 1'b0;
            end

            XOR: begin
                comb_result = a_op ^ b_op;
                cf_c        = 1'b0;
                of_c        = 1'b0;
            end

            NOT: begin
                // NOT affects no flags; defaults above already hold CF.
                comb_result = word ? ~a_op : {8'h00, ~a_op[7:0]};
            end

            SHL: begin
                comb_result = word ? shl_ext[15:0] : {8'h00, shl_ext[7:0]};
                if (shift_cnt != 5'd0) begin
                    cf_c = word ? shl_ext[16] : shl_ext[8];
                    of_c = cf_c ^ (word ? shl_ext[15] : shl_ext[7]);
                end
            end

            SHR: begin
                comb_result = word ? shr_ext[16:1] : {8'h00, shr_ext[8:1]};
                if (shift_cnt != 5'd0) begin
                    cf_c = shr_ext[0];
                    of_c = sign_a;   // defined for count 1: MSB of the original
                end
            end

            SAR: begin
                comb_result = word ? sar_ext[16:1] : {8'h00, sar_ext[8:1]};
                if (shift_cnt != 5'd0) begin
                    cf_c = sar_ext[0];
                    of_c = 1'b0;
                end
            end

            ROL: begin
                comb_result = word ? rol_w : {8'h00, rol_b};
                if (shift_cnt != 5'd0) begin
                    cf_c = word ? rol_w[0] : rol_b[0];
                    of_c = cf_c ^ (word ? rol_w[15] : rol_b[7]);
                end
            end

            ROR: begin
                comb_result = word ? ror_w : {8'h00, ror_b};
                if (shift_cnt != 5'd0) begin
                    cf_c = word ? ror_w[15] : ror_b[7];
                    of_c = word ? (ror_w[15] ^ ror_w[14]) : (ror_b[7] ^ ror_b[6]);
                end
            end

            RCL: begin
                comb_result = word ? rcl_w[15:0] : {8'h00, rcl_b[7:0]};
                if (shift_cnt != 5'd0) begin
                    cf_c = word ? rcl_w[16] : rcl_b[8];
                    of_c = cf_c ^ (word ? rcl_w[15] : rcl_b[7]);
                end
            end

            RCR: begin
                comb_result = word ? rcr_w[15:0] : {8'h00, rcr_b[7:0]};
                if (shift_cnt != 5'd0) begin
                    cf_c = word ? rcr_w[16] : rcr_b[8];
                    of_c = word ? (rcr_w[15] ^ rcr_w[14]) : (rcr_b[7] ^ rcr_b[6]);
                end
            end

            default: begin
                comb_result = a_op;
            end
        endcase
    end

    // =====================================================================
    // Output muxing
    // =====================================================================
    always_comb begin
        if (is_mul_op) begin
            result    = mul_lo;
            result_hi = mul_hi;
        end else if (is_div_op) begin
            result    = quo_signed;
            result_hi = rem_signed;
        end else begin
            result    = comb_result;
            result_hi = 16'h0000;
        end
    end

    // Zero/sign/parity come from whichever result is selected. MUL/IMUL
    // define CF and OF from whether the high half is significant.
    logic [15:0] res_masked;
    assign res_masked = word ? result : {8'h00, result[7:0]};

    assign zf = (res_masked == 16'h0000);
    assign sf = word ? result[15] : result[7];
    assign pf = ~^result[7:0];          // parity of the low byte only, always

    always_comb begin
        if (alu_op == MUL) begin
            cf = word ? (mul_hi != 16'h0000) : (mul_lo[15:8] != 8'h00);
            of = cf;
            af = 1'b0;
        end else if (alu_op == IMUL) begin
            cf = word ? (mul_hi != {16{mul_lo[15]}})
                      : (mul_lo[15:8] != {8{mul_lo[7]}});
            of = cf;
            af = 1'b0;
        end else if (is_div_op) begin
            cf = cf_in;   // undefined after a divide; hold rather than drive X
            of = 1'b0;
            af = 1'b0;
        end else begin
            cf = cf_c;
            of = of_c;
            af = af_c;
        end
    end

endmodule
