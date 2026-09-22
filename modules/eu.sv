// ---------------------------------------------------------------------------
// eu — Execution Unit: structural wiring only.
//
// Hierarchy: cpu_top -> eu -> {decode, microcode, execUnit, ALU, regfile}
// Reference: learnings/00-overview.md (BIU/EU decoupling)
//
// Deliberately contains no logic. execUnit holds all sequencing state, decode
// and microcode are combinational lookups, and ALU and regfile are datapath.
// Keeping this module as pure wiring means there is exactly one place to look
// for control flow, which matters in a design where the FSM is the hard part.
// ---------------------------------------------------------------------------

`timescale 1ns/1ns
module eu
    import cpu_pkg::*;
(
    input  logic        clk,
    input  logic        rst_n,

    // instruction bytes from the BIU
    input  logic [7:0]  fetch_data,
    input  logic        fetch_valid,
    output logic        fetch_pop,
    output logic        fetch_set,
    output logic [19:0] fetch_addr,

    // data bus requests to the BIU
    output logic        req,
    output logic        req_wr,
    output logic        req_io,
    output logic        req_word,
    output logic [19:0] req_addr,
    output logic [15:0] req_wdata,
    input  logic [15:0] req_rdata,
    input  logic        req_done,

    // interrupts
    input  logic        nmi,
    input  logic        intr_req,
    input  logic [7:0]  intr_type,
    output logic        intr_ack,

    output logic        halted,

    // observability for testbenches
    output logic [15:0] dbg_ip,
    output logic [15:0] dbg_cs,
    output logic [15:0] dbg_flags,
    output logic [7:0]  dbg_int_type,
    output logic        dbg_int_taken
);

    // decode <-> execUnit
    logic [7:0]  dec_opcode, dec_modrm;
    logic [5:0]  iclass;
    logic        has_modrm, word_op, dir_to_reg;
    logic [5:0]  alu_op;
    logic [2:0]  imm_bytes;
    logic        imm_sext;
    logic [2:0]  op_reg;
    logic [3:0]  cond;
    logic [1:0]  mod_field;
    logic [2:0]  reg_field, rm_field;
    logic [1:0]  disp_bytes;
    logic        rm_is_mem;

    // microcode <-> execUnit
    logic [2:0]  amode_rm;
    logic [1:0]  amode_mod;
    logic        use_base, use_index, direct_addr;
    logic [2:0]  base_reg, index_reg;
    logic [1:0]  def_seg;

    // regfile <-> execUnit
    logic [2:0]  rd0_sel, rd1_sel, rf_wr_sel;
    logic        rd0_word, rd1_word, rf_wr_word, rf_wr_en;
    logic [15:0] rd0_data, rd1_data, rf_wr_data;
    logic [1:0]  sreg_rd_sel;
    logic [15:0] sreg_rd_data, cs, ip;
    logic        ip_we;
    logic [15:0] ip_wdata, flags, flags_wdata, flags_wmask;

    // ALU <-> execUnit
    logic [5:0]  alu_sel;
    logic        alu_word, alu_start;
    logic [15:0] alu_a, alu_a_hi, alu_b;
    logic        alu_cf_in;
    logic [4:0]  alu_shift;
    logic [15:0] alu_result, alu_result_hi;
    logic        alu_cf, alu_pf, alu_af, alu_zf, alu_sf, alu_of, alu_busy;
    logic        alu_div_zero, alu_byte_ok;
    logic [1:0]  sreg_wr_sel;
    logic        sreg_wr_en;
    logic [15:0] sreg_wr_data;

    assign dbg_ip    = ip;
    assign dbg_cs    = cs;
    assign dbg_flags = flags;

    decode u_decode (
        .opcode     (dec_opcode),
        .modrm      (dec_modrm),
        .iclass     (iclass),
        .has_modrm  (has_modrm),
        .word_op    (word_op),
        .dir_to_reg (dir_to_reg),
        .alu_op     (alu_op),
        .imm_bytes  (imm_bytes),
        .imm_sext   (imm_sext),
        .op_reg     (op_reg),
        .cond       (cond),
        .mod_field  (mod_field),
        .reg_field  (reg_field),
        .rm_field   (rm_field),
        .disp_bytes (disp_bytes),
        .rm_is_mem  (rm_is_mem)
    );

    microcode u_microcode (
        .rm_field  (amode_rm),
        .mod_field (amode_mod),
        .use_base  (use_base),
        .base_reg  (base_reg),
        .use_index (use_index),
        .index_reg (index_reg),
        .def_seg   (def_seg),
        .direct    (direct_addr)
    );

    regfile u_regfile (
        .clk          (clk),
        .rst_n        (rst_n),
        .rd0_sel      (rd0_sel),
        .rd0_word     (rd0_word),
        .rd0_data     (rd0_data),
        .rd1_sel      (rd1_sel),
        .rd1_word     (rd1_word),
        .rd1_data     (rd1_data),
        .wr_sel       (rf_wr_sel),
        .wr_word      (rf_wr_word),
        .wr_en        (rf_wr_en),
        .wr_data      (rf_wr_data),
        .sreg_rd_sel  (sreg_rd_sel),
        .sreg_rd_data (sreg_rd_data),
        .sreg_wr_sel  (sreg_wr_sel),
        .sreg_wr_en   (sreg_wr_en),
        .sreg_wr_data (sreg_wr_data),
        .seg_written  (),
        .cs           (cs),
        .ip           (ip),
        .ip_we        (ip_we),
        .ip_wdata     (ip_wdata),
        .flags        (flags),
        .flags_wdata  (flags_wdata),
        .flags_wmask  (flags_wmask)
    );

    ALU u_alu (
        .clk       (clk),
        .rst_n     (rst_n),
        .alu_op    (alu_sel),
        .word      (alu_word),
        .start     (alu_start),
        .a         (alu_a),
        .a_hi      (alu_a_hi),
        .b         (alu_b),
        .cf_in     (alu_cf_in),
        .shift_cnt (alu_shift),
        .result    (alu_result),
        .result_hi (alu_result_hi),
        .div_zero  (alu_div_zero),
        .byte_ok   (alu_byte_ok),
        .cf        (alu_cf),
        .pf        (alu_pf),
        .af        (alu_af),
        .zf        (alu_zf),
        .sf        (alu_sf),
        .of        (alu_of),
        .busy      (alu_busy)
    );

    execUnit u_exec (
        .clk           (clk),
        .rst_n         (rst_n),
        .fetch_data    (fetch_data),
        .fetch_valid   (fetch_valid),
        .fetch_pop     (fetch_pop),
        .fetch_set     (fetch_set),
        .fetch_addr    (fetch_addr),
        .req           (req),
        .req_wr        (req_wr),
        .req_io        (req_io),
        .req_word      (req_word),
        .req_addr      (req_addr),
        .req_wdata     (req_wdata),
        .req_rdata     (req_rdata),
        .req_done      (req_done),
        .dec_opcode    (dec_opcode),
        .dec_modrm     (dec_modrm),
        .iclass        (iclass),
        .has_modrm     (has_modrm),
        .word_op       (word_op),
        .dir_to_reg    (dir_to_reg),
        .alu_op        (alu_op),
        .imm_bytes     (imm_bytes),
        .imm_sext      (imm_sext),
        .op_reg        (op_reg),
        .cond          (cond),
        .mod_field     (mod_field),
        .reg_field     (reg_field),
        .rm_field      (rm_field),
        .disp_bytes    (disp_bytes),
        .rm_is_mem     (rm_is_mem),
        .amode_rm      (amode_rm),
        .amode_mod     (amode_mod),
        .use_base      (use_base),
        .base_reg      (base_reg),
        .use_index     (use_index),
        .index_reg     (index_reg),
        .def_seg       (def_seg),
        .direct_addr   (direct_addr),
        .rd0_sel       (rd0_sel),
        .rd0_word      (rd0_word),
        .rd0_data      (rd0_data),
        .rd1_sel       (rd1_sel),
        .rd1_word      (rd1_word),
        .rd1_data      (rd1_data),
        .rf_wr_sel     (rf_wr_sel),
        .rf_wr_word    (rf_wr_word),
        .rf_wr_en      (rf_wr_en),
        .rf_wr_data    (rf_wr_data),
        .sreg_rd_sel   (sreg_rd_sel),
        .sreg_rd_data  (sreg_rd_data),
        .cs            (cs),
        .ip_we         (ip_we),
        .ip_wdata      (ip_wdata),
        .flags         (flags),
        .flags_wdata   (flags_wdata),
        .flags_wmask   (flags_wmask),
        .alu_sel       (alu_sel),
        .alu_word      (alu_word),
        .alu_start     (alu_start),
        .alu_a         (alu_a),
        .alu_a_hi      (alu_a_hi),
        .alu_b         (alu_b),
        .alu_cf_in     (alu_cf_in),
        .alu_shift     (alu_shift),
        .alu_result    (alu_result),
        .alu_result_hi (alu_result_hi),
        .alu_cf        (alu_cf),
        .alu_pf        (alu_pf),
        .alu_af        (alu_af),
        .alu_zf        (alu_zf),
        .alu_sf        (alu_sf),
        .alu_of        (alu_of),
        .alu_busy      (alu_busy),
        .alu_div_zero  (alu_div_zero),
        .alu_byte_ok   (alu_byte_ok),
        .sreg_wr_sel   (sreg_wr_sel),
        .sreg_wr_en    (sreg_wr_en),
        .sreg_wr_data  (sreg_wr_data),
        .nmi           (nmi),
        .intr_req      (intr_req),
        .intr_type     (intr_type),
        .intr_ack      (intr_ack),
        .halted        (halted),
        .dbg_int_type  (dbg_int_type),
        .dbg_int_taken (dbg_int_taken)
    );

endmodule
