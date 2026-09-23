// ---------------------------------------------------------------------------
// sdram_controller — DE1-SoC SDRAM interface (conventional RAM backing store).
//
// Hierarchy: FPGA80186 -> memory_controller -> sdram_controller
// Reference: DE1-SoC User Manual, IS42S16320 datasheet
// Testbench: sim/tb_sdram.sv (against a behavioural SDRAM model)
//
// WHY THIS EXISTS: MS-DOS wants 640 KB of conventional RAM, and the Cyclone V
// on this board has nothing like that much M10K once the BIOS ROM, text buffer
// and font ROM have taken their share. The board's 64 MB SDRAM is the only
// realistic home for main memory.
//
// ONE REQUEST PORT. This controller serves a single requester. Sharing it
// between the CPU, the block device and the JTAG loader is sdram_arbiter's
// job, which sits in front and presents this same interface to each of them.
//
// SINGLE CLOCK DOMAIN. The design has no PLL, so the SDRAM runs on the same
// 25 MHz system clock as the CPU. That removes the clock-domain crossing a
// faster memory clock would have forced, and 25 MHz is far below what the
// part can do, so every timing parameter below is comfortably satisfied.
//
// DRAM_CLK is driven from the inverted clock, giving the SDRAM half a cycle
// (20 ns) of setup on the command and address lines. With a PLL the usual
// approach is a phase-shifted clock instead; inverting is the standard
// substitute when no PLL is available.
//
// The 25 MHz system clock exists BECAUSE of this interface. Driving DRAM_CLK
// out through the fabric costs about 4.5 ns of clock delay before the memory
// even sees the edge, and tAC adds 5.4 ns more before read data comes back.
// At 50 MHz that overshot the capture edge by 2.4 ns at every timing corner;
// at 25 MHz the same edge sits deep inside the data valid window. See the
// header of clk_rst.sv for the full derivation.
//
// OPEN-ROW POLICY. A row is 1024 columns of 16 bits -- 2 KB -- and it stays
// ACTIVE after an access instead of being closed by auto-precharge. A second
// access to the same row then costs only the column command and the CAS
// latency, skipping ACTIVATE, tRCD and tRP entirely. Since instruction fetch,
// stack traffic and block copies all walk consecutive addresses, most accesses
// hit the open row:
//
//   read   11 clocks -> 6 on a hit      write  10 clocks -> 3 on a hit
//
// The row is closed only when it has to be: a different row, or a refresh,
// which requires every bank precharged. The cost is the bookkeeping below and
// one hazard worth naming -- interleaving two streams in different rows makes
// every access a miss, so the arbiter's rotating priority can thrash the row
// when the CPU and the block device run together. A miss is no slower than the
// old unconditional auto-precharge, so the floor is unchanged.
//
// REFRESH runs on its own counter and takes priority over requests. Missing
// refresh does not fail loudly -- it silently corrupts memory in a way that
// looks like random program misbehaviour -- so the refresh timer is
// unconditional and the FSM will not start a new access while one is due.
// ---------------------------------------------------------------------------

`timescale 1ns/1ns
module sdram_controller #(
    // Power-up delay before initialisation. 200 us at 25 MHz; the testbench
    // overrides this to something it can afford to simulate.
    parameter int INIT_CYCLES = 5000,
    // Refresh interval. The part needs 8192 refreshes per 64 ms, which is one
    // every 7.8 us, or 195 clocks at 25 MHz. 175 leaves margin.
    //
    // This one MUST track the clock rate. Every other timing value here is a
    // minimum, so a slower clock only makes them safer -- but refresh is a
    // DEADLINE, and overshooting it corrupts memory silently rather than
    // failing in any way a test would notice.
    parameter int REFRESH_CYCLES = 175,
    parameter int CAS_LATENCY = 2
) (
    input  logic        clk,
    input  logic        rst_n,

    // ---- request interface ----
    // 24 bits, not the CPU's 20. Conventional memory only needs 1 MB, but the
    // disk image lives ABOVE it -- out of reach of an 8086 address, visible
    // only to the block device -- so the controller has to address more than
    // the CPU can. 16 MB of the chip's 64 is plenty and still fits in bank 0.
    input  logic [23:0] addr,          // byte address
    input  logic [15:0] wdata,
    output logic [15:0] rdata,
    input  logic        rd,
    input  logic        wr,
    input  logic [1:0]  be,            // [0] low byte, [1] high byte
    output logic        ready,         // one-cycle pulse when the access completes

    // ---- SDRAM chip pins ----
    output logic [12:0] dram_addr,
    output logic [1:0]  dram_ba,
    inout  wire  [15:0] dram_dq,
    output logic        dram_cke,
    output logic        dram_cs_n,
    output logic        dram_ras_n,
    output logic        dram_cas_n,
    output logic        dram_we_n,
    output logic [1:0]  dram_dqm,
    output logic        dram_clk
);

    // Command encoding: {cs_n, ras_n, cas_n, we_n}
    localparam logic [3:0] CMD_NOP      = 4'b0111;
    localparam logic [3:0] CMD_ACTIVE   = 4'b0011;
    localparam logic [3:0] CMD_READ     = 4'b0101;
    localparam logic [3:0] CMD_WRITE    = 4'b0100;
    localparam logic [3:0] CMD_PRECHARGE= 4'b0010;
    localparam logic [3:0] CMD_REFRESH  = 4'b0001;
    localparam logic [3:0] CMD_MRS      = 4'b0000;

    // Mode register: burst length 1, sequential, the configured CAS latency.
    localparam logic [12:0] MODE_REG = {3'b000, 1'b0, 2'b00,
                                        CAS_LATENCY[2:0], 1'b0, 3'b000};

    localparam logic [3:0] S_WAIT     = 4'd0;
    localparam logic [3:0] S_INIT_PRE = 4'd1;
    localparam logic [3:0] S_INIT_R1  = 4'd2;
    localparam logic [3:0] S_INIT_R2  = 4'd3;
    localparam logic [3:0] S_INIT_MRS = 4'd4;
    localparam logic [3:0] S_IDLE     = 4'd5;
    localparam logic [3:0] S_ACTIVE   = 4'd6;
    localparam logic [3:0] S_RCD      = 4'd7;
    localparam logic [3:0] S_READ     = 4'd8;
    localparam logic [3:0] S_READ_W   = 4'd9;
    localparam logic [3:0] S_WRITE    = 4'd10;
    localparam logic [3:0] S_RECOVER  = 4'd11;
    localparam logic [3:0] S_REFRESH  = 4'd12;
    localparam logic [3:0] S_PRE      = 4'd13;

    logic [3:0]  state, next_after_wait;
    logic [15:0] delay;
    logic [3:0]  cmd;
    logic [15:0] dq_out;
    logic        dq_oe;
    logic        is_write;

    logic [15:0] refresh_cnt;
    logic        refresh_due;


    // ---- address split ----
    // Word address, then column / bank / row. Sequential CPU accesses walk the
    // column address, which keeps them inside one row.
    logic [1:0]  dqm_r;
    logic [1:0]  xfer_be;      // byte enables latched with the request

    logic [22:0] word_addr;
    logic [9:0]  col;
    logic [1:0]  bank;
    logic [12:0] row;

    assign word_addr = addr[23:1];
    assign col       = word_addr[9:0];
    // 10 column bits + 13 row bits = 8M words = 16 MB, all inside bank 0. The
    // part has four banks and 64 MB; the other three are simply unused, which
    // costs nothing here because every access auto-precharges anyway.
    assign bank      = 2'b00;
    assign row       = word_addr[22:10];

    assign {dram_cs_n, dram_ras_n, dram_cas_n, dram_we_n} = cmd;
    assign dram_ba   = bank;
    // ---- the open row ----
    // `bank` is fixed at zero, so one row is all there is to track.
    logic        row_open;
    logic [12:0] open_row;
    logic        page_hit;
    assign page_hit = row_open && (row == open_row);
    assign dram_cke  = 1'b1;
    assign dram_clk  = ~clk;
    assign dram_dq   = dq_oe ? dq_out : 16'hzzzz;

    // DQM masks bytes on a write and must be low during a read.
    //
    // REGISTERED, like every other output to the chip. It used to be driven
    // combinationally from `be`, which meant the path ran all the way from a
    // CPU register, through the arbiter's mux, through this logic and out to
    // the pin without a flop anywhere -- 8.1 ns of it. That was invisible
    // while the arbiter had one live port and broke DRAM_CLK setup the moment
    // the block device took a second one.
    assign dram_dqm  = dqm_r;

    assign refresh_due = (refresh_cnt >= REFRESH_CYCLES[15:0]);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= S_WAIT;
            next_after_wait <= S_INIT_PRE;
            delay           <= INIT_CYCLES[15:0];
            cmd             <= CMD_NOP;
            dram_addr       <= 13'h0000;
            dq_out          <= 16'h0000;
            dq_oe           <= 1'b0;
            rdata           <= 16'h0000;
            ready           <= 1'b0;
            dqm_r           <= 2'b00;
            xfer_be         <= 2'b11;
            is_write        <= 1'b0;
            refresh_cnt     <= 16'h0000;
            row_open        <= 1'b0;
            open_row        <= 13'h0000;
        end else begin
            cmd   <= CMD_NOP;
            ready <= 1'b0;
            // Masks are only ever asserted for the one cycle a WRITE command
            // is on the bus; a read requires them low.
            dqm_r <= 2'b00;
            dq_oe <= 1'b0;

            if (!refresh_due) refresh_cnt <= refresh_cnt + 16'd1;

            case (state)
                // Generic timed wait; next_after_wait says where to go.
                S_WAIT: begin
                    if (delay == 16'd0) state <= next_after_wait;
                    else                delay <= delay - 16'd1;
                end

                // ---- power-up initialisation ----
                S_INIT_PRE: begin
                    cmd             <= CMD_PRECHARGE;
                    dram_addr       <= 13'h0400;      // A10 high: precharge all banks
                    delay           <= 16'd4;
                    next_after_wait <= S_INIT_R1;
                    state           <= S_WAIT;
                end

                S_INIT_R1: begin
                    cmd             <= CMD_REFRESH;
                    delay           <= 16'd8;
                    next_after_wait <= S_INIT_R2;
                    state           <= S_WAIT;
                end

                S_INIT_R2: begin
                    cmd             <= CMD_REFRESH;
                    delay           <= 16'd8;
                    next_after_wait <= S_INIT_MRS;
                    state           <= S_WAIT;
                end

                S_INIT_MRS: begin
                    cmd             <= CMD_MRS;
                    dram_addr       <= MODE_REG;
                    delay           <= 16'd4;
                    next_after_wait <= S_IDLE;
                    state           <= S_WAIT;
                end

                // ---- normal operation ----
                S_IDLE: begin
                    // Refresh outranks a pending access. A request waits a few
                    // clocks; a missed refresh corrupts memory silently. Every
                    // bank must be precharged before a refresh, so an open row
                    // is closed first.
                    if (refresh_due) begin
                        if (row_open) begin
                            cmd             <= CMD_PRECHARGE;
                            dram_addr       <= 13'h0400;   // A10: all banks
                            row_open        <= 1'b0;
                            delay           <= 16'd1;      // tRP
                            next_after_wait <= S_REFRESH;
                            state           <= S_WAIT;
                        end else begin
                            state <= S_REFRESH;
                        end
                    end else if (rd || wr) begin
                        is_write <= wr;
                        xfer_be  <= be;
                        if (page_hit) begin
                            // The row is already active: straight to the column.
                            state <= wr ? S_WRITE : S_READ;
                        end else if (row_open) begin
                            cmd             <= CMD_PRECHARGE;
                            dram_addr       <= 13'h0400;
                            row_open        <= 1'b0;
                            delay           <= 16'd1;      // tRP
                            next_after_wait <= S_ACTIVE;
                            state           <= S_WAIT;
                        end else begin
                            state <= S_ACTIVE;
                        end
                    end
                end

                S_ACTIVE: begin
                    cmd       <= CMD_ACTIVE;
                    dram_addr <= row;
                    open_row  <= row;
                    row_open  <= 1'b1;
                    delay     <= 16'd1;                    // tRCD
                    state     <= S_RCD;
                end

                S_RCD: begin
                    if (delay == 16'd0) state <= is_write ? S_WRITE : S_READ;
                    else                delay <= delay - 16'd1;
                end

                S_READ: begin
                    // A10 LOW: no auto-precharge, so the row stays active for
                    // whatever comes next.
                    cmd       <= CMD_READ;
                    dram_addr <= {2'b00, 1'b0, col};
                    delay     <= CAS_LATENCY[15:0];
                    state     <= S_READ_W;
                end

                S_READ_W: begin
                    if (delay == 16'd0) begin
                        rdata <= dram_dq;
                        ready <= 1'b1;
                        // No precharge to wait for. One cycle of recovery so a
                        // requester that has not yet seen `ready` cannot be
                        // served twice.
                        delay <= 16'd0;
                        state <= S_RECOVER;
                    end else begin
                        delay <= delay - 16'd1;
                    end
                end

                S_WRITE: begin
                    cmd       <= CMD_WRITE;
                    dqm_r     <= ~xfer_be;
                    dram_addr <= {2'b00, 1'b0, col};   // A10 low: row stays open
                    dq_out    <= wdata;
                    dq_oe     <= 1'b1;
                    ready     <= 1'b1;
                    // tWR only. At 40 ns a clock one cycle covers it several
                    // times over, and there is no precharge to wait for.
                    delay     <= 16'd0;
                    state     <= S_RECOVER;
                end

                S_RECOVER: begin
                    if (delay == 16'd0) state <= S_IDLE;
                    else                delay <= delay - 16'd1;
                end

                S_REFRESH: begin
                    cmd             <= CMD_REFRESH;
                    refresh_cnt     <= 16'h0000;
                    delay           <= 16'd8;          // tRFC
                    next_after_wait <= S_IDLE;
                    state           <= S_WAIT;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
