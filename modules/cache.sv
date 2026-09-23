// ---------------------------------------------------------------------------
// cache — a direct-mapped instruction/data cache in front of the SDRAM.
//
// Hierarchy: memory_controller -> cache -> sdram_arbiter -> sdram_controller
// Testbench: sim/tb_cache.sv (the module), sim/tb_msdos.sv (the payoff)
//
// WHY THIS EXISTS, measured rather than assumed. Instrumenting the two big
// system testbenches with the same stall counters gave opposite answers:
//
//     workload                     cycles with a bus request   stalled on SDRAM
//     tb_bios   (BIOS, on-chip ROM)          10.4%                   2.6%
//     tb_msdos  (MS-DOS, from SDRAM)         57.2%                  44.4%
//
// The BIOS number is the misleading one: the BIOS runs out of on-chip ROM,
// which answers in a single cycle, so it never waits for anything. Everything
// that actually matters -- DOS, and a game on top of it -- runs from SDRAM and
// loses nearly half of every second to it. That is what this recovers.
//
// WRITE-THROUGH, NO WRITE-ALLOCATE. A write goes to memory every time and the
// CPU waits for it. That sounds wasteful and is not: fetches vastly outnumber
// writes here, and write-through means the copy in SDRAM is ALWAYS current.
// Nothing downstream can ever see a stale word, so there is no writeback, no
// dirty bit, and no flush-before-anything-else-looks. For a first cache in a
// system with three bus masters, that is worth more than the write bandwidth.
// A write that HITS still updates the cached copy, because the stack is
// written and re-read constantly and invalidating it on every push would cost
// far more than the update logic does.
//
// CRITICAL WORD FIRST. A miss fetches the word the CPU asked for BEFORE the
// rest of the line, and releases the CPU the moment it lands; the remaining
// words fill behind it. Without this a miss would cost the whole line -- about
// 28 cycles against the 10 an uncached read costs today -- and any code with
// poor locality would get SLOWER. This way a miss costs what it always did and
// the line fill is close to free, because sdram_controller keeps the row open
// and the rest of the line is a sequence of page hits.
//
// THE COHERENCE ARGUMENT, which is the part that could silently corrupt
// memory. Three masters share the SDRAM: this CPU port, the block device, and
// the JTAG loader. A cache is only safe if nothing else writes the memory it
// caches. As the system stands, nothing does:
//
//   * storage.sv is programmed I/O, not DMA. It only ever touches SDRAM at
//     BASE = 100000h and above, where the disk image lives; sector data
//     reaches CPU memory by the BIOS copying it through an I/O port, which is
//     a CPU write and goes through this cache like any other.
//   * the JTAG loader writes the same region, above 1 MB.
//
// So the snoop below never actually fires today. It is here anyway, because
// "no other master writes low memory" is a property of the current design
// rather than of the architecture, and the day someone adds a real DMA engine
// the failure mode is not a compile error -- it is DOS reading a sector into
// memory and executing whatever the cache still had. The snoop turns that
// silent corruption into a performance blip: any foreign write below 1 MB
// invalidates everything. Valid bits are flops precisely so that costs one
// cycle rather than a sweep.
//
// GEOMETRY. KB of data, LINE_WORDS 16-bit words per line, direct mapped.
// Default 8 KB in 8-byte lines: 1024 lines, a 7-bit tag, about 8 M10K blocks.
// ---------------------------------------------------------------------------

`timescale 1ns/1ns
module cache #(
    parameter int KB         = 8,
    parameter int LINE_WORDS = 4
) (
    input  logic        clk,
    input  logic        rst_n,

    // ---- CPU side ----
    // Same contract as the memory it replaces: hold addr/wdata/be and rd/wr
    // steady until c_ready pulses; c_rdata is valid in that cycle and held.
    input  logic [19:0] c_addr,
    input  logic [15:0] c_wdata,
    output logic [15:0] c_rdata,
    input  logic        c_rd,
    input  logic        c_wr,
    input  logic [1:0]  c_be,
    output logic        c_ready,

    // ---- memory side (one requester port on the arbiter) ----
    output logic [23:0] m_addr,
    output logic [15:0] m_wdata,
    input  logic [15:0] m_rdata,
    output logic        m_rd,
    output logic        m_wr,
    output logic [1:0]  m_be,
    input  logic        m_ready,

    // ---- snoop: another master is writing memory ----
    input  logic        snoop_wr,
    input  logic [23:0] snoop_addr,

    // ---- counters for the testbenches; no function in hardware ----
    output logic        stat_hit,
    output logic        stat_miss
);

    localparam int WORDS = (KB * 1024) / 2;
    localparam int LINES = WORDS / LINE_WORDS;
    localparam int OFF_W = $clog2(LINE_WORDS);
    localparam int IDX_W = $clog2(LINES);
    localparam int MEM_W = OFF_W + IDX_W;        // word address bits kept
    localparam int TAG_W = 19 - MEM_W;           // a CPU word address is 19 bits

    // ---- address break-up ----
    logic [18:0]      c_word;
    logic [IDX_W-1:0] c_idx;
    logic [TAG_W-1:0] c_tag;
    logic [MEM_W-1:0] c_ma;

    assign c_word = c_addr[19:1];
    assign c_ma   = c_word[MEM_W-1:0];
    assign c_idx  = c_word[MEM_W-1:OFF_W];
    assign c_tag  = c_word[18:MEM_W];

    // ---- storage ----
    // Two byte-wide banks so a byte write needs no read-modify-write, the same
    // shape vram and framebuffer use. Tags are a third block; valid bits are
    // flops so the whole cache can be dropped in one cycle.
    logic [7:0]       dat_lo  [0:WORDS-1];
    logic [7:0]       dat_hi  [0:WORDS-1];
    logic [TAG_W-1:0] tag_mem [0:LINES-1];
    logic [LINES-1:0] valid;

    // ---- lookup, launched every cycle ----
    // This is the one thing that makes a hit cost nothing. The BIU drives the
    // address in T1 and does not assert rd until T2, so by starting the tag
    // and data read unconditionally on every edge, the answer is already
    // waiting when the request arrives and c_ready can come back in T3 -- the
    // 4-T-state minimum, the fastest an 80186 bus cycle can be. Gating the
    // lookup on rd instead would add a cycle to every hit.
    logic [TAG_W-1:0] r_tag_q, r_tag;
    logic [15:0]      r_data;
    logic [IDX_W-1:0] r_idx;
    logic [MEM_W-1:0] r_ma;

    always_ff @(posedge clk) begin
        r_tag_q <= tag_mem[c_idx];
        r_data  <= {dat_hi[c_ma], dat_lo[c_ma]};
        r_tag   <= c_tag;
        r_idx   <= c_idx;
        r_ma    <= c_ma;
    end

    // valid is read COMBINATIONALLY, not registered alongside the rest. A
    // registered copy would be a cycle stale, and a snoop that invalidated a
    // line in that cycle would be ignored -- a hit on data that had just been
    // declared wrong. Reading the flops directly closes that window.
    logic hit;
    assign hit = valid[r_idx] && (r_tag_q == r_tag);

    // THE LOOKUP IS NOT ALWAYS TRUSTWORTHY, and this was a bug the testbench
    // caught rather than a precaution. The arrays are read-before-write, so
    // in the cycle immediately after a fill finishes, r_tag_q still holds the
    // tag of the line that was just EVICTED, while the valid bit has already
    // been set for its replacement. If the evicted tag is the one being asked
    // for -- exactly what happens when code alternates between two addresses
    // that share an index -- the comparison succeeds and the cache returns
    // the new line's data under the old line's address. The data array has
    // the same hazard for the last word of a fill.
    //
    // The fix is to WAIT one cycle, not to force a miss. Forcing a miss looks
    // equivalent and is not: it would start a redundant fill of the line that
    // had just been filled, and, worse, a write arriving in that cycle would
    // record w_hit = 0 and skip updating a line that really is resident --
    // leaving a stale word in the cache that write-through is supposed to
    // make impossible. Gating the DECISION rather than the comparison costs
    // one cycle in a rare case and keeps `hit` meaning what it says.
    logic arr_wr_d, lu_ok;
    assign lu_ok = !arr_wr_d;

    // ---- fill / write state ----
    localparam logic [1:0] S_IDLE = 2'd0, S_FILL = 2'd1, S_WRITE = 2'd2;
    logic [1:0] state;

    logic [IDX_W-1:0] f_idx;
    logic [TAG_W-1:0] f_tag;
    logic [OFF_W-1:0] f_off;          // word being fetched; wraps within the line
    logic [OFF_W:0]   f_cnt;          // how many of the line have landed
    logic             f_poison;       // a snoop hit us mid-fill; do not validate
    logic             gap;            // one idle cycle between memory requests

    logic             w_hit;
    logic [MEM_W-1:0] w_ma;

    logic hit_now, miss_now, wr_now, fill_ack, fill_last, write_ack;
    logic snoop_flush;

    assign hit_now   = (state == S_IDLE) && lu_ok && c_rd && !c_wr &&  hit;
    assign miss_now  = (state == S_IDLE) && lu_ok && c_rd && !c_wr && !hit;
    assign wr_now    = (state == S_IDLE) && lu_ok && c_wr;
    assign fill_ack  = (state == S_FILL)  && m_ready && (f_cnt == '0);
    assign fill_last = (state == S_FILL)  && m_ready &&
                       (f_cnt == LINE_WORDS[OFF_W:0] - 1);
    assign write_ack = (state == S_WRITE) && m_ready;

    // Only writes BELOW 1 MB can alias anything the CPU can address, so the
    // disk image traffic above that -- which is all of it today -- costs
    // nothing at all.
    assign snoop_flush = snoop_wr && (snoop_addr < 24'h100000);

    // ---- memory port ----
    // The arbiter's contract is one access per request, so rd/wr must drop
    // between words of a fill; `gap` is that cycle. Holding it up instead
    // would start a second unasked-for access.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) gap <= 1'b0;
        else        gap <= m_ready;
    end

    logic [18:0] f_word;
    assign f_word = {f_tag, f_idx, f_off};

    always_comb begin
        if (state == S_FILL) begin
            m_addr  = {4'h0, f_word, 1'b0};
            m_wdata = 16'h0000;
            m_be    = 2'b11;
            m_rd    = !gap;
            m_wr    = 1'b0;
        end else if (state == S_WRITE) begin
            m_addr  = {4'h0, c_addr};
            m_wdata = c_wdata;
            m_be    = c_be;
            m_rd    = 1'b0;
            m_wr    = !gap;
        end else begin
            m_addr  = {4'h0, c_addr};
            m_wdata = c_wdata;
            m_be    = c_be;
            m_rd    = 1'b0;
            m_wr    = 1'b0;
        end
    end

    // ---- data array write port ----
    // Two writers, and they cannot collide: a fill only runs in S_FILL and a
    // write-hit update only in S_WRITE.
    logic [MEM_W-1:0] wr_ma;
    logic [15:0]      wr_dat;
    logic [1:0]       wr_en;

    always_comb begin
        wr_ma  = {f_idx, f_off};
        wr_dat = m_rdata;
        wr_en  = 2'b00;
        if (state == S_FILL && m_ready) begin
            wr_en = 2'b11;
        end else if (write_ack && w_hit) begin
            wr_ma  = w_ma;
            wr_dat = c_wdata;
            wr_en  = c_be;
        end
    end

    always_ff @(posedge clk) begin
        if (wr_en[0]) dat_lo[wr_ma] <= wr_dat[7:0];
        if (wr_en[1]) dat_hi[wr_ma] <= wr_dat[15:8];
        if (fill_last && !f_poison && !snoop_flush) tag_mem[f_idx] <= f_tag;
    end

    // The tag is only ever written in a cycle that also writes data, so one
    // flag covers both arrays.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) arr_wr_d <= 1'b0;
        else        arr_wr_d <= |wr_en;
    end

    // ---- valid bits ----
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid <= '0;
        end else if (snoop_flush) begin
            // Conservative and deliberate: a foreign write to low memory drops
            // everything, including a line that is mid-fill.
            valid <= '0;
        end else begin
            // Clear on the way in, set on the way out, so a half-filled line
            // is never a candidate for a hit even for one cycle.
            if (miss_now)                    valid[r_idx] <= 1'b0;
            if (fill_last && !f_poison)      valid[f_idx] <= 1'b1;
        end
    end

    // ---- the state machine ----
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            f_idx    <= '0;
            f_tag    <= '0;
            f_off    <= '0;
            f_cnt    <= '0;
            f_poison <= 1'b0;
            w_hit    <= 1'b0;
            w_ma     <= '0;
        end else begin
            if (snoop_flush && state == S_FILL) f_poison <= 1'b1;

            case (state)
                S_IDLE: begin
                    if (miss_now) begin
                        f_idx    <= r_idx;
                        f_tag    <= r_tag;
                        f_off    <= r_ma[OFF_W-1:0];   // critical word first
                        f_cnt    <= '0;
                        f_poison <= snoop_flush;
                        state    <= S_FILL;
                    end else if (wr_now) begin
                        w_hit <= hit;
                        w_ma  <= r_ma;
                        state <= S_WRITE;
                    end
                end

                S_FILL: begin
                    if (m_ready) begin
                        f_off <= f_off + 1'b1;         // wraps inside the line
                        f_cnt <= f_cnt + 1'b1;
                        if (fill_last) begin
                            f_poison <= 1'b0;
                            state    <= S_IDLE;
                        end
                    end
                end

                S_WRITE: begin
                    if (m_ready) state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // ---- back to the CPU ----
    // A hit answers one cycle after rd rises, which is T3 -- exactly what the
    // on-chip regions do, and the minimum the bus allows.
    logic        hit_ack;
    logic [15:0] hold;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) hit_ack <= 1'b0;
        else        hit_ack <= hit_now;
    end

    always_ff @(posedge clk) begin
        if (hit_now)       hold <= r_data;
        else if (fill_ack) hold <= m_rdata;
    end

    // On the fill path the word is on m_rdata in the very cycle c_ready
    // pulses, so it has to be forwarded rather than waited for.
    assign c_rdata = fill_ack ? m_rdata : hold;
    assign c_ready = hit_ack || fill_ack || write_ack;

    // ---- statistics ----
    // One pulse per access, taken on the rising edge of the request so a
    // multi-cycle wait is not counted several times.
    logic acc_d;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) acc_d <= 1'b0;
        else        acc_d <= c_rd || c_wr;
    end

    assign stat_hit  = (c_rd || c_wr) && !acc_d &&  hit;
    assign stat_miss = (c_rd || c_wr) && !acc_d && !hit;

endmodule
