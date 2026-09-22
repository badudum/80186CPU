// ---------------------------------------------------------------------------
// microcode — ModR/M addressing-mode table. Purely combinational, no state.
//
// Hierarchy: cpu_top -> eu -> microcode
// Reference: learnings/01-programming-model.md (addressing modes table)
//
// Maps the ModR/M r/m field to the registers that form the effective address
// and to the segment that applies by default. The 8086/80186 has a fixed set
// of eight base+index combinations -- there is no general scaled-index
// addressing, that arrived with the 386.
//
//   r/m   address          default segment
//   000   BX + SI          DS
//   001   BX + DI          DS
//   010   BP + SI          SS
//   011   BP + DI          SS
//   100   SI               DS
//   101   DI               DS
//   110   BP               SS   (but a direct address, in DS, when mod = 00)
//   111   BX               DS
//
// The rule behind the segment column is that anything involving BP is
// stack-relative, because BP exists to address stack frames.
//
// NOTE ON SCOPE: the sequencer does not implement segment-override prefixes
// yet, so `def_seg` is currently always the segment used. When overrides are
// added they replace this value (except for string destinations via DI, which
// are forced to ES and cannot be overridden).
// ---------------------------------------------------------------------------

`timescale 1ns/1ns
module microcode
    import cpu_pkg::*;
(
    input  logic [2:0] rm_field,
    input  logic [1:0] mod_field,

    output logic       use_base,
    output logic [2:0] base_reg,
    output logic       use_index,
    output logic [2:0] index_reg,
    output logic [1:0] def_seg,
    output logic       direct        // mod=00 + r/m=110: displacement only
);

    assign direct = (mod_field == 2'b00) && (rm_field == 3'b110);

    always_comb begin
        use_base  = 1'b0;
        use_index = 1'b0;
        base_reg  = R_BX;
        index_reg = R_SI;
        def_seg   = SR_DS;

        case (rm_field)
            3'b000: begin use_base=1; base_reg=R_BX; use_index=1; index_reg=R_SI; def_seg=SR_DS; end
            3'b001: begin use_base=1; base_reg=R_BX; use_index=1; index_reg=R_DI; def_seg=SR_DS; end
            3'b010: begin use_base=1; base_reg=R_BP; use_index=1; index_reg=R_SI; def_seg=SR_SS; end
            3'b011: begin use_base=1; base_reg=R_BP; use_index=1; index_reg=R_DI; def_seg=SR_SS; end
            3'b100: begin use_index=1; index_reg=R_SI; def_seg=SR_DS; end
            3'b101: begin use_index=1; index_reg=R_DI; def_seg=SR_DS; end
            3'b110: begin
                if (direct) begin
                    // Displacement-only form: no register, and it is DS, not SS.
                    def_seg = SR_DS;
                end else begin
                    use_base = 1'b1; base_reg = R_BP; def_seg = SR_SS;
                end
            end
            default: begin use_base=1; base_reg=R_BX; def_seg=SR_DS; end
        endcase
    end

endmodule
