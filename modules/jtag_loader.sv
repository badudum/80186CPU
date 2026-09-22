// ---------------------------------------------------------------------------
// jtag_loader — writes a disk image into SDRAM over the USB-Blaster cable.
//
// Hierarchy: FPGA80186 -> jtag_loader
// Testbench: sim/tb_jtag_loader.sv
// Host side: tools/jtag_load.tcl, run by quartus_stp
//
// WHY THIS EXISTS. With the disk backed by SDRAM it can be any size, which is
// what makes a 1.44 MB floppy possible at all -- but SDRAM is volatile, so
// something has to put an image there after every power-up. This is that
// something, and it needs no hardware beyond the cable already used to program
// the board.
//
// PROTOCOL. A virtual JTAG node with a 4-bit instruction register:
//
//   1 ADDR    DR = 24 bits   set the write pointer (a byte address)
//   2 DATA    DR = N*16 bits stream words; each completed word is written and
//                            the pointer advances by two
//   3 CTRL    DR = 8 bits    bit 0 holds the CPU in reset
//   4 STATUS  DR = 32 bits   {overflow, 7'b0, words_written[23:0]}, read back
//   5 PEEK    DR = 32 bits   {8'h0, seq, data}: requests the word at the
//                            pointer and advances it, returning the PREVIOUS
//                            request's result
//   6 PEEKD   DR = 32 bits   the same {seq, data}, with no side effect -- poll
//                            this until `seq` changes to know the result is
//                            the one just asked for
//   7 PROF    DR = 32 bits   one {CS, IP} sample from the profile ring; each
//                            scan returns the next one
//
// THE PROFILE RING samples the CPU's CS:IP in hardware, sixteen times, and is
// the only way to ask "where is the machine spending its time" that a guest
// operating system cannot lie to. Doing it in the BIOS's timer interrupt
// instead looks obvious and does not work: MS-DOS hooks INT 08h and chains,
// so the return address the BIOS sees is MS-DOS's chaining call, every single
// time, and the profile is a picture of the profiler. CTRL bit 1 freezes
// sampling so a ring can be read out without tearing.
//
// WORD FRAMING, and why it is not a bit counter. A virtual JTAG node does not
// see the host's data at the first shift clock of a scan: the SLD hub and the
// bypass registers of every other SLD node in the chain sit ahead of it, so
// some number of junk bits arrive first. For an ordinary node that latches at
// update-DR this is harmless -- the junk is pushed out the far end and the
// last N bits shifted in are the host's. A STREAMING node that consumes words
// as they arrive has no such luck: counting sixteen bits from the start of the
// scan puts every word boundary off by the width of that lead-in, and since
// the lead-in depends on what else is in the chain, the whole image comes back
// rotated. That is not a hypothetical -- enabling the in-system memory editor
// added five nodes and skewed the stream by seven bits.
//
// So the host prefixes each scan with sixteen zero bits and a SYNC word, and
// the framing here hunts for SYNC before it frames anything. The zeros flush
// whatever the lead-in contained, and because SYNC is odd no truncated prefix
// of it followed by zeros can equal it -- a false sync is impossible rather
// than merely unlikely.
//
// Streaming many words in ONE scan is the whole performance story. Per-word
// JTAG transactions cost milliseconds of host overhead each, and 1.44 MB is
// 737,280 words; at one transaction apiece it would take hours. Shifting
// thousands of words in a single DR scan and consuming them as they arrive
// turns that into seconds.
//
// THE CLOCK CROSSING, and why STATUS exists. TCK is asynchronous to the system
// clock and JTAG cannot be stalled -- there is no way to tell the host to wait
// mid-scan. So words are handed across with a toggle handshake, and if one
// arrives before the previous has been written the OVERFLOW bit latches. The
// host reads STATUS afterwards: a set overflow bit, or a word count that does
// not match what was sent, means the load is invalid and should be retried at
// a lower TCK. Silent corruption of a disk image would be very hard to
// diagnose from the far end.
//
// In practice the margin is wide. At 6 MHz TCK a 16-bit word takes 2.7 us to
// shift while an SDRAM write takes about 0.6 us, so the writer drains four
// times faster than the shifter fills.
//
// SIM_HOOKS lets everything except the Altera primitive be tested. The virtual
// JTAG node cannot be simulated without vendor libraries, which would break
// this project's dependency-free simulation flow -- so with SIM_HOOKS set the
// same logic is driven from ordinary ports instead.
// ---------------------------------------------------------------------------

`timescale 1ns/1ns
module jtag_loader #(
    parameter int  INSTANCE_ID = 0,
    parameter bit  SIM_HOOKS   = 1'b0
) (
    input  logic clk,              // system clock
    input  logic rst_n,

    // SDRAM requester port, held until `ready` as sdram_arbiter requires
    output logic [23:0] mem_addr,
    output logic [15:0] mem_wdata,
    output logic [1:0]  mem_be,
    output logic        mem_rd,
    output logic        mem_wr,
    input  logic [15:0] mem_rdata,
    input  logic        mem_ready,

    // {CS, IP}, sampled straight out of the CPU
    input  logic [31:0] cpu_pc,

    // Holds the CPU in reset while an image is being written, so the machine
    // cannot execute out of memory that is being rewritten underneath it.
    output logic        cpu_hold,

    // ---- simulation hooks (SIM_HOOKS = 1) ----
    input  logic        sim_tck,
    input  logic        sim_tdi,
    input  logic [3:0]  sim_ir,
    input  logic        sim_cdr,
    input  logic        sim_sdr,
    input  logic        sim_udr,
    output logic        sim_tdo
);

    localparam logic [3:0] IR_ADDR   = 4'd1;
    localparam logic [3:0] IR_DATA   = 4'd2;
    localparam logic [3:0] IR_CTRL   = 4'd3;
    localparam logic [3:0] IR_STATUS = 4'd4;
    localparam logic [3:0] IR_PEEK   = 4'd5;
    localparam logic [3:0] IR_PEEKD  = 4'd6;
    localparam logic [3:0] IR_PROF   = 4'd7;

    // One sample every 2^PROF_LOG clocks: at 25 MHz that is about 24 Hz, so
    // sixteen samples span two thirds of a second of real execution.
    localparam int PROF_LOG = 20;
    localparam int PROF_N   = 16;

    // Must be odd: see the framing note above.
    localparam logic [15:0] SYNC = 16'hB2C1;

    // ---- the JTAG side ----
    logic       tck, tdi, tdo;
    logic [3:0] ir_in;
    logic       v_cdr, v_sdr, v_udr;

    assign sim_tdo = tdo;

    generate
        if (SIM_HOOKS) begin : g_sim
            assign tck   = sim_tck;
            assign tdi   = sim_tdi;
            assign ir_in = sim_ir;
            assign v_cdr = sim_cdr;
            assign v_sdr = sim_sdr;
            assign v_udr = sim_udr;
        end else begin : g_jtag
            sld_virtual_jtag #(
                .sld_auto_instance_index ("NO"),
                .sld_instance_index      (INSTANCE_ID),
                .sld_ir_width            (4)
            ) u_vjtag (
                .tck               (tck),
                .tdi               (tdi),
                .tdo               (tdo),
                .ir_in             (ir_in),
                .ir_out            (4'b0000),
                .virtual_state_cdr (v_cdr),
                .virtual_state_sdr (v_sdr),
                .virtual_state_udr (v_udr),
                .virtual_state_e1dr (),
                .virtual_state_pdr  (),
                .virtual_state_e2dr (),
                .virtual_state_cir  (),
                .virtual_state_uir  ()
            );
        end
    endgenerate

    // =====================================================================
    // TCK domain
    // =====================================================================
    logic [23:0] addr_sr;
    logic [15:0] data_sr;
    logic [7:0]  ctrl_sr;
    logic [31:0] out_sr;
    logic [3:0]  bitcnt;

    logic [15:0] word_tck;
    logic        w_toggle;
    logic        a_toggle;
    logic        pending;
    logic        ovf_tck;
    logic        r_toggle;
    logic [15:0] peek_s1, peek_s2;
    logic [7:0]  pseq_s1, pseq_s2;
    logic        synced;
    logic [23:0] addr_tck;
    logic        hold_tck;
    logic        frz_tck;
    logic [3:0]  prof_rd;
    logic [31:0] prof [0:PROF_N-1];

    // The write side's acknowledgement, brought back into TCK.
    logic ack_sys;
    logic ack_s1, ack_s2, ack_s3;
    logic ack_edge;
    assign ack_edge = ack_s2 ^ ack_s3;

    // Status, brought into TCK. Only read when the host has stopped streaming,
    // at which point these are static and a plain synchroniser is enough.
    logic [23:0] words_sys;
    logic        ovf_sys;
    logic [15:0] peek_sys;      // driven by the system domain, read here
    logic [7:0]  peek_seq;      // bumped one cycle AFTER peek_sys settles
    logic [23:0] words_s1, words_s2;
    logic        ovfs_s1, ovfs_s2;

    logic word_done;
    assign word_done = v_sdr && (ir_in == IR_DATA) && synced && (bitcnt == 4'd15);

    assign tdo = out_sr[0];

    // rst_n is a system-domain signal used here as an ASYNCHRONOUS reset. Its
    // deassertion is not synchronised to TCK, which is acceptable because JTAG
    // is quiet during power-on reset -- but without a reset at all these
    // registers start as X, the toggle handshake never produces a recognisable
    // edge, and nothing is ever written. On hardware flops come up at zero and
    // it would appear to work, which is precisely why it has to be explicit.
    always_ff @(posedge tck or negedge rst_n) begin
        if (!rst_n) begin
            ack_s1   <= 1'b0; ack_s2 <= 1'b0; ack_s3 <= 1'b0;
            words_s1 <= 24'd0; words_s2 <= 24'd0;
            ovfs_s1  <= 1'b0; ovfs_s2 <= 1'b0;
            addr_sr  <= 24'd0;
            data_sr  <= 16'd0;
            ctrl_sr  <= 8'd0;
            out_sr   <= 32'd0;
            bitcnt   <= 4'd0;
            word_tck <= 16'd0;
            w_toggle <= 1'b0;
            a_toggle <= 1'b0;
            r_toggle <= 1'b0;
            peek_s1  <= 16'd0;
            peek_s2  <= 16'd0;
            pseq_s1  <= 8'd0;
            pseq_s2  <= 8'd0;
            synced   <= 1'b0;
            pending  <= 1'b0;
            ovf_tck  <= 1'b0;
            addr_tck <= 24'd0;
            hold_tck <= 1'b0;
            frz_tck  <= 1'b0;
            prof_rd  <= 4'd0;
        end else begin
        ack_s1 <= ack_sys;
        ack_s2 <= ack_s1;
        ack_s3 <= ack_s2;

        words_s1 <= words_sys;  words_s2 <= words_s1;
        peek_s1  <= peek_sys;   peek_s2  <= peek_s1;
        pseq_s1  <= peek_seq;   pseq_s2  <= pseq_s1;
        ovfs_s1  <= ovf_sys;    ovfs_s2  <= ovfs_s1;

        if (v_cdr) begin
            // Framing restarts with every scan: the lead-in junk is per-scan,
            // so the hunt has to be too.
            bitcnt  <= 4'd0;
            synced  <= 1'b0;
            data_sr <= 16'h0000;
            // PEEK carries a sequence number beside the data so the host can
            // tell a fresh result from the previous one; STATUS the counters.
            if (ir_in == IR_PROF)
                out_sr <= prof[prof_rd];
            else if (ir_in == IR_PEEK || ir_in == IR_PEEKD)
                out_sr <= {8'h00, pseq_s2, peek_s2};
            else
                out_sr <= {ovfs_s2 | ovf_tck, 7'b0, words_s2};
        end else if (v_sdr) begin
            out_sr <= {1'b0, out_sr[31:1]};
            case (ir_in)
                IR_ADDR: addr_sr <= {tdi, addr_sr[23:1]};
                IR_CTRL: ctrl_sr <= {tdi, ctrl_sr[7:1]};
                IR_DATA: begin
                    data_sr <= {tdi, data_sr[15:1]};
                    if (!synced) begin
                        // bitcnt is left at zero until SYNC lands, so the bit
                        // that follows it is bit 0 of word 0.
                        if ({tdi, data_sr[15:1]} == SYNC) synced <= 1'b1;
                    end else begin
                        bitcnt <= (bitcnt == 4'd15) ? 4'd0 : (bitcnt + 4'd1);
                    end
                end
                default: ;
            endcase
        end

        // A completed word is handed over with a toggle. If the previous one
        // has not been acknowledged yet, the writer has fallen behind and the
        // image would be corrupt -- latch that rather than lose it quietly.
        if (word_done) begin
            word_tck <= {tdi, data_sr[15:1]};
            w_toggle <= ~w_toggle;
            if (pending && !ack_edge) ovf_tck <= 1'b1;
            pending <= 1'b1;
        end else if (ack_edge) begin
            pending <= 1'b0;
        end

        if (v_udr) begin
            case (ir_in)
                IR_ADDR: begin
                    // Setting the pointer begins a load: it also clears the
                    // counters and the overflow flag on BOTH sides, so a retry
                    // after a failed load starts from a clean slate rather than
                    // reporting the previous attempt's error forever.
                    addr_tck <= addr_sr;
                    a_toggle <= ~a_toggle;
                    ovf_tck  <= 1'b0;
                    // An aborted load can leave a word handed over whose
                    // acknowledgement never came back, and a stale `pending`
                    // makes the very first word of the NEXT load look like an
                    // overrun -- reporting a failure for an image that is
                    // perfectly good. A new pointer is the clean-slate point,
                    // so clear it here too.
                    pending  <= 1'b0;
                end
                IR_CTRL: begin
                    hold_tck <= ctrl_sr[0];
                    frz_tck  <= ctrl_sr[1];   // stop sampling so a read is stable
                end
                IR_PROF: prof_rd <= prof_rd + 4'd1;
                // Ask the system side for the word at the pointer. The result
                // is picked up by the NEXT capture, which is why a host reads
                // one scan behind.
                IR_PEEK: r_toggle <= ~r_toggle;   // IR_PEEKD deliberately does not
                default: ;
            endcase
        end
        end
    end

    // =====================================================================
    // System domain
    // =====================================================================
    logic r_s1, r_s2, r_s3;
    logic new_read;
    assign new_read = r_s2 ^ r_s3;

    logic w_s1, w_s2, w_s3;
    logic a_s1, a_s2, a_s3;
    logic h_s1, h_s2;
    logic f_s1, f_s2;
    logic o_s1, o_s2;
    logic [PROF_LOG-1:0] prof_div;
    logic [3:0]          prof_wr;

    logic        new_word, new_addr;
    assign new_word = w_s2 ^ w_s3;
    assign new_addr = a_s2 ^ a_s3;

    logic [23:0] ptr;
    logic [15:0] wdat;

    localparam logic [2:0] W_IDLE = 3'd0;
    localparam logic [2:0] W_REQ  = 3'd1;
    localparam logic [2:0] W_GAP  = 3'd2;
    localparam logic [2:0] W_RD   = 3'd3;
    localparam logic [2:0] W_SEQ  = 3'd4;
    logic [2:0] wstate;

    assign mem_addr  = ptr;
    assign mem_wdata = wdat;
    assign mem_be    = 2'b11;
    assign mem_rd    = (wstate == W_RD);
    assign mem_wr    = (wstate == W_REQ);

    assign cpu_hold  = h_s2;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w_s1 <= 1'b0; w_s2 <= 1'b0; w_s3 <= 1'b0;
            r_s1 <= 1'b0; r_s2 <= 1'b0; r_s3 <= 1'b0;
            peek_sys <= 16'd0;
            a_s1 <= 1'b0; a_s2 <= 1'b0; a_s3 <= 1'b0;
            h_s1 <= 1'b0; h_s2 <= 1'b0;
            f_s1 <= 1'b0; f_s2 <= 1'b0;
            o_s1 <= 1'b0; o_s2 <= 1'b0;
            prof_div <= '0;
            prof_wr  <= 4'd0;
            for (int k = 0; k < PROF_N; k++) prof[k] <= 32'h0;
            ptr       <= 24'h000000;
            wdat      <= 16'h0000;
            peek_seq  <= 8'd0;
            wstate    <= W_IDLE;
            ack_sys   <= 1'b0;
            words_sys <= 24'd0;
            ovf_sys   <= 1'b0;
        end else begin
            w_s1 <= w_toggle; w_s2 <= w_s1; w_s3 <= w_s2;
            r_s1 <= r_toggle; r_s2 <= r_s1; r_s3 <= r_s2;
            a_s1 <= a_toggle; a_s2 <= a_s1; a_s3 <= a_s2;
            h_s1 <= hold_tck; h_s2 <= h_s1;
            f_s1 <= frz_tck;  f_s2 <= f_s1;

            prof_div <= prof_div + 1'b1;
            if (prof_div == '1 && !f_s2) begin
                prof[prof_wr] <= cpu_pc;
                prof_wr       <= prof_wr + 4'd1;
            end
            o_s1 <= ovf_tck;  o_s2 <= o_s1;

            // The TCK-side flag is level, not a pulse, so it is captured
            // continuously -- but a new address clears it below, and that
            // clear must win.
            if (o_s2) ovf_sys <= 1'b1;

            // A new pointer takes effect immediately; the host sets it while
            // not streaming.
            if (new_addr) begin
                ptr       <= addr_tck;
                words_sys <= 24'd0;
                ovf_sys   <= 1'b0;
            end

            case (wstate)
                W_IDLE: if (new_word) begin
                    wdat   <= word_tck;
                    wstate <= W_REQ;
                end else if (new_read) begin
                    wstate <= W_RD;
                end
                W_RD: if (mem_ready) begin
                    peek_sys <= mem_rdata;
                    ptr      <= ptr + 24'd2;
                    wstate   <= W_SEQ;
                end
                // The sequence number is bumped a cycle after the data lands,
                // so that by the time the host sees the new number the data it
                // labels has been stable for a whole system clock plus two TCK
                // synchroniser stages. Without that gap the two could cross
                // together and the host could read a number that promises a
                // word which has not arrived.
                W_SEQ: begin
                    peek_seq <= peek_seq + 8'd1;
                    wstate   <= W_GAP;
                end
                W_REQ: if (mem_ready) begin
                    ptr       <= ptr + 24'd2;
                    words_sys <= words_sys + 24'd1;
                    ack_sys   <= ~ack_sys;
                    wstate    <= W_GAP;
                end
                // One cycle with the request deasserted, which is what the
                // arbiter needs between accesses.
                W_GAP: wstate <= W_IDLE;
                default: wstate <= W_IDLE;
            endcase
        end
    end

endmodule
