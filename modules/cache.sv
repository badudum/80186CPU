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
// GEOMETRY. KB of data in LINE_WORDS-word lines, WAYS-way set associative
// with tree pseudo-LRU replacement. Default 8 KB in 8-byte lines, 8 ways:
// 128 sets, a 10-bit tag, 7 PLRU bits per set.
//
// WHY SET ASSOCIATIVE. Direct mapped is one line per index, so two hot
// addresses 8 KB apart evict each other on every single access -- and the
// code that does that is ordinary: a loop reading one array and writing
// another. Eight ways means eight such addresses can coexist.
//
// WHY TREE PLRU RATHER THAN TRUE LRU. True LRU over 8 ways needs an ordering
// of 8 items per set, which is 3 bits per way plus the logic to reorder them
// on every hit. Tree PLRU is 7 bits per set and one XOR-free update: each bit
// is a node in a binary tree saying "the least recently used side is this
// way". It picks a victim that is never the most recently used and is almost
// always among the oldest, for a fraction of the state.
//
// AN INVALID WAY ALWAYS WINS over the PLRU choice. On a cold cache PLRU would
// otherwise keep replacing the same way while seven sit empty.
//
// READING ALL WAYS AT ONCE is what keeps a hit single-cycle. The way is not
// known until the tags compare, so the data array is WAYS words wide and the
// match selects one afterwards. A narrow array indexed by the winning way
// would need a second cycle for every hit, which costs more than the width.
// ---------------------------------------------------------------------------

`timescale 1ns/1ns
module cache #(
    parameter int KB         = 8,
    parameter int LINE_WORDS = 4,
    parameter int WAYS       = 8,
    // Next-line prefetch. OFF, because it was measured and it LOSES: see the
    // note above the prefetch logic for the numbers and the reason.
    parameter bit PREFETCH   = 1'b0,
    // Posted writes. 0 restores the old behaviour -- the CPU waits for every
    // write to reach SDRAM -- which is the baseline every measurement of this
    // is against, and the thing to fall back to if it is ever suspected.
    parameter int WBUF_DEPTH = 4
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
    output logic        stat_miss,
    output logic        stat_pf_start,   // a prefetch was begun
    output logic        stat_pf_done,    // ...and it completed
    output logic        stat_pf_abort
);

    localparam int WORDS = (KB * 1024) / 2;
    localparam int LINES = WORDS / LINE_WORDS;
    localparam int SETS  = LINES / WAYS;
    localparam int OFF_W = $clog2(LINE_WORDS);
    localparam int IDX_W = $clog2(SETS);
    localparam int WAY_W = $clog2(WAYS);
    localparam int TAG_W = 19 - OFF_W - IDX_W;   // a CPU word address is 19 bits
    localparam int SA_W  = IDX_W + OFF_W;        // data array: set and offset

    // ---- address break-up ----
    logic [18:0]      c_word;
    logic [IDX_W-1:0] c_set;
    logic [TAG_W-1:0] c_tag;
    logic [OFF_W-1:0] c_off;
    logic [SA_W-1:0]  c_sa;

    assign c_word = c_addr[19:1];
    assign c_off  = c_word[OFF_W-1:0];
    assign c_set  = c_word[SA_W-1:OFF_W];
    assign c_tag  = c_word[18:SA_W];
    assign c_sa   = c_word[SA_W-1:0];

    // ---- storage ----
    // Two byte-wide banks so a byte write needs no read-modify-write, each
    // WAYS bytes wide so every way is read at once. Tags are one wide word
    // per set for the same reason. Valid bits are flops so the whole cache
    // can be dropped in a cycle, which is what the snoop needs.
    // ONE NARROW ARRAY PER WAY, not one wide array indexed by way. The wide
    // form is the obvious way to write this and it does not synthesise: a
    // variable part-select write, dat[addr][way*8 +: 8], is not a shape
    // Quartus recognises as a byte-enabled RAM, so it built the whole 8 KB
    // out of logic -- 74,235 ALMs against the 32,070 the device has, with
    // ZERO RAM blocks used. Per-way arrays written from a generate loop give
    // each way a constant index, which is an ordinary simple dual-port RAM
    // and infers M10K.
    logic [7:0]            dat_lo  [0:WAYS-1][0:SETS*LINE_WORDS-1];
    logic [7:0]            dat_hi  [0:WAYS-1][0:SETS*LINE_WORDS-1];
    logic [TAG_W-1:0]      tag_mem [0:WAYS-1][0:SETS-1];
    logic [WAYS-1:0]       valid [0:SETS-1];
    logic [WAYS-2:0]       plru  [0:SETS-1];

    // ---- lookup, launched every cycle ----
    // The BIU drives the address in T1 and does not assert rd until T2, so
    // starting the read unconditionally on every edge means the answer is
    // already waiting when the request arrives and a hit can answer in T3 --
    // the 4-T-state minimum. Gating on rd would add a cycle to every hit.
    logic [TAG_W-1:0]      r_tags [0:WAYS-1];
    logic [7:0]            r_lo [0:WAYS-1];
    logic [7:0]            r_hi [0:WAYS-1];
    logic [TAG_W-1:0]      r_tag;
    logic [IDX_W-1:0]      r_set;
    logic [OFF_W-1:0]      r_off;
    logic [SA_W-1:0]       r_sa;

    always_ff @(posedge clk) begin
        r_tag  <= c_tag;
        r_set  <= c_set;
        r_off  <= c_off;
        r_sa   <= c_sa;
    end

    // valid is read COMBINATIONALLY, not registered alongside the rest. A
    // registered copy would be a cycle stale, and a snoop that invalidated a
    // line in that cycle would be ignored -- a hit on data just declared
    // wrong. Reading the flops directly closes that window.
    logic [WAYS-1:0] way_hit;
    logic            hit_any;
    logic [WAY_W-1:0] hit_way;

    always_comb begin
        for (int w = 0; w < WAYS; w++)
            way_hit[w] = valid[r_set][w] && (r_tags[w] == r_tag);
    end
    assign hit_any = |way_hit;

    // WAY_HIT IS ONE-HOT BY CONSTRUCTION. A line is only installed on a miss,
    // and a miss means no valid way already holds that tag, so a tag cannot
    // appear twice in a set. That is what makes the selects below legal.
    //
    // SELECT THE DATA ONE-HOT rather than encoding the way and then muxing
    // with it. Encode-then-mux puts a priority encoder in series with an 8:1
    // mux on the path that decides c_ready -- two extra levels of logic for
    // no benefit. Masking each way's bytes with its own hit bit and OR-ing
    // them is one AND and a balanced OR tree, and it is the same shape the
    // comparators already produce.
    logic [15:0] hit_data;
    always_comb begin
        hit_data = 16'h0000;
        for (int w = 0; w < WAYS; w++)
            hit_data = hit_data | ({16{way_hit[w]}} & {r_hi[w], r_lo[w]});
    end

    // The encoded way is still needed, but only for the write-hit path and
    // the PLRU update -- both of which are registered a cycle later and are
    // nowhere near the critical path. OR-ing rather than prioritising,
    // again because the input is one-hot.
    always_comb begin
        hit_way = '0;
        for (int w = 0; w < WAYS; w++)
            hit_way = hit_way | ({WAY_W{way_hit[w]}} & w[WAY_W-1:0]);
    end

    // THE LOOKUP IS NOT ALWAYS TRUSTWORTHY. The arrays are read-before-write,
    // so in the cycle after a fill finishes the registered tags still hold
    // the line that was just evicted while its valid bit has been set for the
    // replacement. Waiting one cycle is the fix; forcing a miss instead would
    // start a redundant fill and, worse, make a write record w_hit = 0 for a
    // line that really is resident, leaving a stale word that write-through
    // is supposed to make impossible.
    logic arr_wr_d, lu_ok;
    assign lu_ok = !arr_wr_d;

    // ---- victim selection ----
    // An invalid way first; otherwise walk the PLRU tree from the root. Each
    // bit points at the side to replace, so following them lands on a way
    // that is never the most recently used.
    logic [WAY_W-1:0] plru_way, victim;
    logic             have_invalid;
    logic [WAY_W-1:0] invalid_way;
    logic [WAYS-2:0]  p;

    assign p = plru[r_set];

    always_comb begin
        have_invalid = 1'b0;
        invalid_way  = '0;
        for (int w = WAYS-1; w >= 0; w--)
            if (!valid[r_set][w]) begin
                have_invalid = 1'b1;
                invalid_way  = w[WAY_W-1:0];
            end

        // Tree walk for 8 ways: node 0 is the root, 1 and 2 the halves,
        // 3..6 the leaf pairs.
        plru_way[2] = p[0];
        plru_way[1] = p[0] ? p[2] : p[1];
        case ({p[0], plru_way[1]})
            2'b00:   plru_way[0] = p[3];
            2'b01:   plru_way[0] = p[4];
            2'b10:   plru_way[0] = p[5];
            default: plru_way[0] = p[6];
        endcase

        victim = have_invalid ? invalid_way : plru_way;
    end

    // Point the tree away from the way just touched.
    task automatic plru_touch(input int st, input logic [WAY_W-1:0] w);
        begin
            plru[st][0] <= ~w[2];
            if (!w[2]) plru[st][1] <= ~w[1];
            else       plru[st][2] <= ~w[1];
            case (w[2:1])
                2'b00:   plru[st][3] <= ~w[0];
                2'b01:   plru[st][4] <= ~w[0];
                2'b10:   plru[st][5] <= ~w[0];
                default: plru[st][6] <= ~w[0];
            endcase
        end
    endtask

    // ---- fill / write state ----
    localparam logic [1:0] S_IDLE = 2'd0, S_FILL = 2'd1, S_WRITE = 2'd2;
    logic [1:0] state;

    logic [IDX_W-1:0] f_set;
    logic [TAG_W-1:0] f_tag;
    logic [WAY_W-1:0] f_way;
    logic [OFF_W-1:0] f_off;          // word being fetched; wraps in the line
    logic [OFF_W:0]   f_cnt;          // how many of the line have landed
    logic             f_poison;       // a snoop hit us mid-fill
    logic             gap;            // one idle cycle between requests

    logic             w_hit;
    logic [SA_W-1:0]  w_sa;
    logic [WAY_W-1:0] w_way;

    // ---- next-line prefetch ----
    // See the note at the end of this file: it is measured and it LOSES on
    // this memory system, so PREFETCH defaults off. The logic is kept because
    // the mechanism is sound and the memory path is what is not ready.
    logic             pf_pending;
    logic [TAG_W-1:0] pf_tag;
    logic [IDX_W-1:0] pf_set;
    logic             is_pf;
    logic             pf_abort, pf_start;

    // ---- write buffer ----
    // WRITE-THROUGH MADE THE CPU WAIT FOR SDRAM ON EVERY STORE. c_ready was
    // asserted by write_ack, which is m_ready coming back from the memory, so
    // a store cost a full SDRAM write before the sequencer could move on. On
    // the MS-DOS boot that is STR_WR spending 48% of its 2.8M cycles waiting,
    // plus every PUSH and every STORE.
    //
    // Nothing needs the CPU to wait. Write-through exists so that no OTHER
    // master ever sees a stale word, and the ordering that guarantees still
    // holds if the writes are merely queued: they reach memory in the order
    // they were issued, just later. So a store now lands in this FIFO and
    // c_ready goes up in the same cycle, and the FIFO drains whenever the
    // memory port is otherwise idle.
    //
    // THE HAZARD THIS CREATES is a read of an address whose write is still in
    // the queue. A write that HIT updated the cached copy, so a read that
    // hits is already correct. A write that MISSED did not -- there is no
    // write-allocate -- so its only copy is in the queue, and a later read of
    // that line would fill from SDRAM and get the word before the write.
    // wb_hazard catches exactly that: a fill whose line matches a queued
    // write drains first. Line-granular, not word-granular, because the fill
    // brings in the whole line.
    //
    // No foreign-read hazard exists. Per the coherence argument above, the
    // CPU port is the only master that touches memory below 1 MB, and the CPU
    // cannot address anything else.
    localparam int WB_AW = $clog2(WBUF_DEPTH < 2 ? 2 : WBUF_DEPTH);

    logic [19:0]        wb_addr [0:WBUF_DEPTH-1];
    logic [15:0]        wb_data [0:WBUF_DEPTH-1];
    logic [1:0]         wb_be   [0:WBUF_DEPTH-1];
    logic [WB_AW-1:0]   wb_head, wb_tail;
    logic [WB_AW:0]     wb_cnt;

    logic wb_full, wb_empty, wb_push, wb_pop;
    assign wb_full  = (wb_cnt == WBUF_DEPTH[WB_AW:0]);
    assign wb_empty = (wb_cnt == '0);

    // Does a queued write cover the line this fill would bring in?
    localparam int LINE_LSB = OFF_W + 1;           // byte address -> line
    logic wb_hazard;
    always_comb begin
        wb_hazard = 1'b0;
        for (int q = 0; q < WBUF_DEPTH; q++)
            if ((q < int'(wb_cnt)) &&
                (wb_addr[(wb_head + q[WB_AW-1:0]) % WBUF_DEPTH][19:LINE_LSB]
                 == c_addr[19:LINE_LSB]))
                wb_hazard = 1'b1;
    end

    logic hit_now, miss_now, wr_now, fill_ack, fill_last, write_ack;
    logic snoop_flush;

    assign hit_now   = (state == S_IDLE) && lu_ok && c_rd && !c_wr &&  hit_any;
    assign miss_now  = (state == S_IDLE) && lu_ok && c_rd && !c_wr && !hit_any
                       && !wb_hazard;
    assign wr_now    = (state == S_IDLE) && lu_ok && c_wr;
    // The store is complete as far as the CPU is concerned the moment it is
    // queued. Only a full queue makes it wait.
    assign wb_push   = wr_now && !wb_full;
    // !is_pf is load-bearing: a prefetch must never answer the CPU. fill_ack
    // forwards the word straight out and asserts c_ready, which is right for
    // a demand miss and catastrophic for a prefetch -- a waiting read would
    // take data from an address it never asked for.
    assign fill_ack  = (state == S_FILL) && m_ready && (f_cnt == '0) && !is_pf;
    assign fill_last = (state == S_FILL) && m_ready &&
                       (f_cnt == LINE_WORDS[OFF_W:0] - 1);
    assign write_ack = (state == S_WRITE) && m_ready;

    // Only writes BELOW 1 MB can alias anything the CPU can address, so the
    // disk image traffic above that -- which is all of it today -- costs
    // nothing at all.
    assign snoop_flush = snoop_wr && (snoop_addr < 24'h100000);

    // A demand access while a prefetch is filling. No latch is needed on the
    // request, which looks wrong but is not: an access that cannot be served
    // holds c_rd high until it IS served, so it is still asserted when the
    // next word lands.
    assign pf_abort = is_pf && (c_rd || c_wr) && m_ready;
    assign pf_start = PREFETCH && pf_pending && !c_rd && !c_wr && lu_ok
                      && (state == S_IDLE);

    assign stat_pf_start = pf_start;
    assign stat_pf_done  = fill_last && is_pf && !pf_abort;
    assign stat_pf_abort = pf_abort;

    // ---- memory port ----
    // The arbiter's contract is one access per request, so rd/wr must drop
    // between words of a fill; `gap` is that cycle.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) gap <= 1'b0;
        else        gap <= m_ready;
    end

    logic [18:0] f_word;
    assign f_word = {f_tag, f_set, f_off};

    always_comb begin
        if (state == S_FILL) begin
            m_addr  = {4'h0, f_word, 1'b0};
            m_wdata = 16'h0000;
            m_be    = 2'b11;
            m_rd    = !gap;
            m_wr    = 1'b0;
        end else if (state == S_WRITE) begin
            // Draining the queue, so the address comes from its head rather
            // than from the CPU -- which has long since moved on.
            m_addr  = {4'h0, wb_addr[wb_head]};
            m_wdata = wb_data[wb_head];
            m_be    = wb_be[wb_head];
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
    // Two writers that cannot collide: a fill only runs in S_FILL and a
    // write-hit update only in S_WRITE. The way selects which byte lane of
    // the wide word is written.
    logic [SA_W-1:0]  wr_sa;
    logic [WAY_W-1:0] wr_way;
    logic [15:0]      wr_dat;
    logic [1:0]       wr_en;

    always_comb begin
        wr_sa  = {f_set, f_off};
        wr_way = f_way;
        wr_dat = m_rdata;
        wr_en  = 2'b00;
        if (state == S_FILL && m_ready) begin
            wr_en = 2'b11;
        end else if (wb_push && hit_any) begin
            // Updated when the write is QUEUED, not when it reaches memory.
            // The CPU has been told the store is done, so the cached copy has
            // to agree from that cycle on -- otherwise a read hit between the
            // queueing and the draining returns the old word.
            wr_sa  = r_sa;
            wr_way = hit_way;
            wr_dat = c_wdata;
            wr_en  = c_be;
        end
    end

    genvar gw;
    generate
        for (gw = 0; gw < WAYS; gw++) begin : g_way
            always_ff @(posedge clk) begin
                // Read every way each cycle; the tag compare picks one after.
                r_tags[gw] <= tag_mem[gw][c_set];
                r_lo[gw]   <= dat_lo[gw][c_sa];
                r_hi[gw]   <= dat_hi[gw][c_sa];
                // Constant way index, so each of these is a plain RAM.
                if (wr_en[0] && wr_way == gw[WAY_W-1:0])
                    dat_lo[gw][wr_sa] <= wr_dat[7:0];
                if (wr_en[1] && wr_way == gw[WAY_W-1:0])
                    dat_hi[gw][wr_sa] <= wr_dat[15:8];
                if (fill_last && !f_poison && !snoop_flush
                    && f_way == gw[WAY_W-1:0])
                    tag_mem[gw][f_set] <= f_tag;
            end
        end
    endgenerate

    // The tag is only ever written in a cycle that also writes data, so one
    // flag covers both arrays.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) arr_wr_d <= 1'b0;
        else        arr_wr_d <= |wr_en;
    end

    // ---- valid bits and PLRU ----
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int st = 0; st < SETS; st++) begin
                valid[st] <= '0;
                plru[st]  <= '0;
            end
        end else if (snoop_flush) begin
            // Conservative and deliberate: a foreign write to low memory
            // drops everything, including a line that is mid-fill.
            for (int st = 0; st < SETS; st++) valid[st] <= '0;
        end else begin
            // Clear on the way in, set on the way out, so a half-filled line
            // is never a candidate for a hit even for one cycle.
            if (miss_now) valid[r_set][victim] <= 1'b0;
            // A prefetch must invalidate its target too: the fill writes the
            // data array immediately but only fixes the tag at the end, so
            // without this it overwrites another line's data while that
            // line's tag and valid bit still say it is good.
            if (pf_start) valid[pf_set][victim] <= 1'b0;
            if (fill_last && !f_poison) valid[f_set][f_way] <= 1'b1;

            // PLRU is updated on every access that USES a way: a hit, and the
            // fill that installs a line. A prefetch deliberately does not
            // touch it -- nobody asked for that line, so it should not count
            // as recently used and push out something that was.
            if (hit_now)                          plru_touch(r_set, hit_way);
            if (wb_push && hit_any)               plru_touch(r_set, hit_way);
            if (fill_last && !f_poison && !is_pf) plru_touch(f_set, f_way);
        end
    end

    // The drain pops as the memory acknowledges; the push happens whenever a
    // store is accepted, including in the same cycle as a pop.
    assign wb_pop = (state == S_WRITE) && m_ready;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wb_head <= '0;
            wb_tail <= '0;
            wb_cnt  <= '0;
        end else begin
            if (wb_push) begin
                wb_addr[wb_tail] <= c_addr;
                wb_data[wb_tail] <= c_wdata;
                wb_be[wb_tail]   <= c_be;
                wb_tail <= (wb_tail == WBUF_DEPTH[WB_AW-1:0] - 1'b1)
                           ? '0 : wb_tail + 1'b1;
            end
            if (wb_pop)
                wb_head <= (wb_head == WBUF_DEPTH[WB_AW-1:0] - 1'b1)
                           ? '0 : wb_head + 1'b1;
            case ({wb_push, wb_pop})
                2'b10:   wb_cnt <= wb_cnt + 1'b1;
                2'b01:   wb_cnt <= wb_cnt - 1'b1;
                default: ;
            endcase
        end
    end

    // ---- the state machine ----
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            f_set      <= '0;
            f_tag      <= '0;
            f_way      <= '0;
            f_off      <= '0;
            f_cnt      <= '0;
            f_poison   <= 1'b0;
            w_hit      <= 1'b0;
            w_sa       <= '0;
            w_way      <= '0;
            pf_pending <= 1'b0;
            pf_tag     <= '0;
            pf_set     <= '0;
            is_pf      <= 1'b0;
        end else begin
            if (snoop_flush && state == S_FILL) f_poison <= 1'b1;

            case (state)
                S_IDLE: begin
                    if (miss_now) begin
                        f_set    <= r_set;
                        f_tag    <= r_tag;
                        f_way    <= victim;
                        f_off    <= r_off;             // critical word first
                        f_cnt    <= '0;
                        f_poison <= snoop_flush;
                        is_pf    <= 1'b0;
                        state    <= S_FILL;
                    end else if (!wb_empty && (wb_hazard || !c_rd || !lu_ok)) begin
                        // Drain when the memory port would otherwise be idle,
                        // or when a queued write is standing in the way of a
                        // fill. A pending read with no hazard is served first:
                        // the CPU is waiting on it, and nothing is waiting on
                        // the queue.
                        state <= S_WRITE;
                    end else if (pf_start) begin
                        // Demand work always wins: this arm is only reached
                        // when the CPU is asking for nothing at all.
                        f_set    <= pf_set;
                        f_tag    <= pf_tag;
                        f_way    <= victim;
                        f_off    <= '0;
                        f_cnt    <= '0;
                        f_poison <= snoop_flush;
                        is_pf    <= 1'b1;
                        pf_pending <= 1'b0;
                        state    <= S_FILL;
                    end
                end

                S_FILL: begin
                    if (pf_abort) begin
                        // Give the port back without validating the line.
                        is_pf <= 1'b0;
                        state <= S_IDLE;
                    end else if (m_ready) begin
                        f_off <= f_off + 1'b1;         // wraps inside the line
                        f_cnt <= f_cnt + 1'b1;
                        if (fill_last) begin
                            f_poison <= 1'b0;
                            is_pf    <= 1'b0;
                            state    <= S_IDLE;
                            // Only after a DEMAND miss. Chaining prefetches
                            // off prefetches would run ahead of the program
                            // indefinitely.
                            if (!is_pf) begin
                                {pf_tag, pf_set} <= {f_tag, f_set} + 1'b1;
                                pf_pending       <= 1'b1;
                            end
                        end
                    end
                end

                S_WRITE: begin
                    // One entry per visit, then back to S_IDLE so a waiting
                    // CPU access is looked at again before the next drain.
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
        if (hit_now)       hold <= hit_data;
        else if (fill_ack) hold <= m_rdata;
    end

    // On the fill path the word is on m_rdata in the very cycle c_ready
    // pulses, so it has to be forwarded rather than waited for.
    assign c_rdata = fill_ack ? m_rdata : hold;
    // A queued store is acknowledged immediately; write_ack no longer has
    // anything to do with the CPU, since the drain it belongs to happens long
    // after the store retired.
    assign c_ready = hit_ack || fill_ack || wb_push;

    // ---- statistics ----
    logic acc_d;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) acc_d <= 1'b0;
        else        acc_d <= c_rd || c_wr;
    end

    assign stat_hit  = (c_rd || c_wr) && !acc_d &&  hit_any;
    assign stat_miss = (c_rd || c_wr) && !acc_d && !hit_any;

    // ---- the prefetch measurement ----
    // Measured on the MS-DOS boot, direct-mapped 8 KB:
    //
    //                      CPI     instructions   SDRAM stall
    //     PREFETCH = 0    18.36      1,078,332       26.1%
    //     PREFETCH = 1    19.81        999,479       29.7%
    //
    // It retired 7.3% FEWER instructions. Abandoning is too coarse -- a bus
    // cycle cannot be recalled, so a demand access waits up to a whole word
    // transaction behind a wrong guess -- and in a direct-mapped cache every
    // prefetch was also an eviction. Set associativity removes the second
    // reason, so this is worth re-measuring now.

endmodule
