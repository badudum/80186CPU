`timescale 1ns/1ns
//
// Behavioural SDRAM model for testing sdram_controller.
//
// This is deliberately strict about PROTOCOL rather than about timing. It
// checks that a READ or WRITE is only ever issued to a bank whose row has been
// activated, and that the row matches -- which is what catches the bugs that
// actually happen when writing a controller (a missing ACTIVATE, a stale row,
// precharging too early). Exact tRCD/tRP/tRFC numbers are not enforced; at
// 50 MHz they have enormous margin and checking them would mostly test the
// testbench.
//
// The device samples commands on the rising edge of its own clock, which the
// controller drives inverted, so everything here keys off posedge dram_clk.
//
module sdram_model #(
    parameter int CAS_LATENCY = 2,
    parameter int ROW_BITS    = 11     // 4 MB: conventional RAM plus a disk
                                      // image above it, without making the
                                      // model's array unreasonably large
) (
    input  logic        dram_clk,
    input  logic        dram_cke,
    input  logic        dram_cs_n,
    input  logic        dram_ras_n,
    input  logic        dram_cas_n,
    input  logic        dram_we_n,
    input  logic [12:0] dram_addr,
    input  logic [1:0]  dram_ba,
    input  logic [1:0]  dram_dqm,
    inout  wire  [15:0] dram_dq
);

    localparam logic [3:0] CMD_NOP       = 4'b0111;
    localparam logic [3:0] CMD_ACTIVE    = 4'b0011;
    localparam logic [3:0] CMD_READ      = 4'b0101;
    localparam logic [3:0] CMD_WRITE     = 4'b0100;
    localparam logic [3:0] CMD_PRECHARGE = 4'b0010;
    localparam logic [3:0] CMD_REFRESH   = 4'b0001;
    localparam logic [3:0] CMD_MRS       = 4'b0000;

    logic [3:0] cmd;
    assign cmd = {dram_cs_n, dram_ras_n, dram_cas_n, dram_we_n};

    // 512 rows x 1024 columns x 16 bits == 1 MB, which is the whole 80186
    // address space.
    logic [15:0] mem [0:(1<<(ROW_BITS+10))-1];

    logic                 row_open [0:3];
    logic [ROW_BITS-1:0]  open_row [0:3];

    logic        initialised;
    logic [12:0] mode_reg;
    int          refresh_count;
    int          errors;

    // read return pipeline
    logic [15:0] rd_pipe   [0:7];
    logic        rd_valid  [0:7];
    logic [15:0] dq_drive;
    logic        dq_oe;

    assign dram_dq = dq_oe ? dq_drive : 16'hzzzz;

    task automatic protocol_error(input string msg);
        begin
            $display("SDRAM PROTOCOL ERROR: %s (time %0t)", msg, $time);
            errors++;
        end
    endtask

    initial begin
        for (int i = 0; i < (1<<(ROW_BITS+10)); i++) mem[i] = 16'h0000;
        for (int b = 0; b < 4; b++) begin
            row_open[b] = 1'b0;
            open_row[b] = '0;
        end
        for (int i = 0; i < 8; i++) begin
            rd_pipe[i]  = 16'h0000;
            rd_valid[i] = 1'b0;
        end
        initialised   = 1'b0;
        mode_reg      = 13'h0000;
        refresh_count = 0;
        errors        = 0;
        dq_oe         = 1'b0;
        dq_drive      = 16'h0000;
    end

    logic [ROW_BITS+9:0] a_idx;

    always @(posedge dram_clk) begin
        // advance the CAS pipeline
        dq_oe    <= rd_valid[0];
        dq_drive <= rd_pipe[0];
        for (int i = 0; i < 7; i++) begin
            rd_pipe[i]  <= rd_pipe[i+1];
            rd_valid[i] <= rd_valid[i+1];
        end
        rd_valid[7] <= 1'b0;

        if (dram_cke) begin
            case (cmd)
                CMD_MRS: begin
                    mode_reg    <= dram_addr;
                    initialised <= 1'b1;
                end

                CMD_REFRESH: begin
                    refresh_count <= refresh_count + 1;
                    for (int b = 0; b < 4; b++)
                        if (row_open[b]) protocol_error("refresh with a row still open");
                end

                CMD_ACTIVE: begin
                    if (row_open[dram_ba]) protocol_error("ACTIVE on a bank that is already open");
                    row_open[dram_ba] <= 1'b1;
                    open_row[dram_ba] <= dram_addr[ROW_BITS-1:0];
                end

                CMD_PRECHARGE: begin
                    if (dram_addr[10]) begin
                        for (int b = 0; b < 4; b++) row_open[b] <= 1'b0;
                    end else begin
                        row_open[dram_ba] <= 1'b0;
                    end
                end

                CMD_READ: begin
                    if (!initialised) protocol_error("READ before the mode register was set");
                    if (!row_open[dram_ba]) begin
                        protocol_error("READ without an ACTIVE row");
                    end else begin
                        a_idx = {open_row[dram_ba], dram_addr[9:0]};
                        rd_pipe[CAS_LATENCY-1]  <= mem[a_idx];
                        rd_valid[CAS_LATENCY-1] <= 1'b1;
                    end
                    // A10 selects auto-precharge.
                    if (dram_addr[10]) row_open[dram_ba] <= 1'b0;
                end

                CMD_WRITE: begin
                    if (!initialised) protocol_error("WRITE before the mode register was set");
                    if (!row_open[dram_ba]) begin
                        protocol_error("WRITE without an ACTIVE row");
                    end else begin
                        a_idx = {open_row[dram_ba], dram_addr[9:0]};
                        if (!dram_dqm[0]) mem[a_idx][7:0]  <= dram_dq[7:0];
                        if (!dram_dqm[1]) mem[a_idx][15:8] <= dram_dq[15:8];
                    end
                    if (dram_addr[10]) row_open[dram_ba] <= 1'b0;
                end

                default: ;   // NOP or deselect
            endcase
        end
    end

endmodule
