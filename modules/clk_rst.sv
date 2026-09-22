// ---------------------------------------------------------------------------
// clk_rst — clock generation and reset synchronisation (DE1-SoC / Cyclone V).
//
// Hierarchy: FPGA80186 -> clk_rst
// Reference: learnings/04-integrated-peripherals.md §2 (reset timing)
// Testbench: sim/tb_clkrst.sv
//
// NO PLL. CLOCK_50 is divided by two in this module and that 25 MHz clock runs
// the entire design -- CPU, SDRAM and video alike.
//
// WHY 25 AND NOT 50. The CPU core itself closes timing at 50 MHz with room to
// spare; the binding path was the SDRAM READ CAPTURE. DRAM_CLK is the inverted
// system clock driven out through the fabric to a pin, so it arrives at the
// memory about 4.5 ns after the internal clock edge. The chip then takes a
// further tAC (5.4 ns) to drive read data back. At a 20 ns period that lands
// the data 2.4 ns past the capture edge -- a real violation at all four timing
// corners, not a pessimistic model. A PLL would fix it by phase-shifting
// DRAM_CLK to cancel the output delay, which is exactly what Terasic's own
// SDRAM reference design does, but a PLL is deliberately not used here. At a
// 40 ns period the same edge lands deep inside the data valid window instead,
// with no change to the memory controller at all.
//
// The cost is CPU speed, and it is a cheap one: 25 MHz is still two to four
// times the 6-12.5 MHz a real 80186 ran at.
//
// ONE CLOCK, TWO NAMES. clk_cpu and clk_vga are now the same net. They are kept
// as separate ports because the rest of the hierarchy is written in terms of
// two domains, and because the video side genuinely wants 25 MHz on its own
// terms: the nominal pixel clock for 640x480 @ 60 Hz is 25.175 MHz, so 25.000
// is 0.7% slow and refresh lands near 59.5 Hz, well inside what any monitor
// tolerates. A pleasant side effect is that the text buffer's dual-port
// crossing is now synchronous rather than a true CDC.
//
// clk_sys is produced by a flip-flop rather than a PLL output, so it is a
// derived clock. Quartus will normally promote it onto a global clock network
// automatically; FPGA80186.sdc declares it with create_generated_clock so
// timing analysis understands the relationship.
//
// RESET: KEY[0] is a raw mechanical button, so it is debounced before use, and
// its DEASSERTION is synchronised into each clock domain. Assertion may be
// asynchronous; deassertion must not be, or flops around the design leave
// reset on different cycles. The 80186 requires reset to be held for at least
// four clocks -- the debounce interval exceeds that by orders of magnitude, so
// that requirement is met for free.
// ---------------------------------------------------------------------------

`timescale 1ns/1ns
module clk_rst #(
    // ~10 ms at 50 MHz. Overridden low by the testbench, which has no patience.
    parameter int DEBOUNCE = 500000
) (
    input  logic clk_board,    // CLOCK_50
    input  logic rst_btn_n,    // KEY[0], active low

    output logic clk_cpu,      // 25 MHz
    output logic clk_vga,      // 25 MHz (the same net as clk_cpu)
    output logic rst_n,        // synchronised to clk_cpu
    output logic rst_vga_n     // synchronised to clk_vga
);

    // ---- system clock: divide by 2 ----
    // Plain `always`, not `always_ff`, throughout this module's reset
    // generator: these registers get their power-up values from the initial
    // block at the bottom, because they ARE the reset source and so cannot
    // themselves be reset. always_ff forbids that second driver. Quartus
    // honours initial values as the power-up state on Cyclone V.
    logic clk_sys_r;
    always @(posedge clk_board) clk_sys_r <= ~clk_sys_r;

    assign clk_cpu = clk_sys_r;
    assign clk_vga = clk_sys_r;

    // ---- button debounce ----
    // The button is only accepted as changed once it has held its new value
    // for the full interval, which rejects contact bounce in both directions.
    localparam int CW = $clog2(DEBOUNCE + 1);

    logic [CW-1:0] db_cnt;
    logic          btn_sync, btn_meta, btn_stable;

    always @(posedge clk_board) begin
        btn_meta <= rst_btn_n;
        btn_sync <= btn_meta;

        if (btn_sync == btn_stable) begin
            db_cnt <= '0;
        end else if (db_cnt == DEBOUNCE[CW-1:0]) begin
            btn_stable <= btn_sync;
            db_cnt     <= '0;
        end else begin
            db_cnt <= db_cnt + 1'b1;
        end
    end

    // Power-on reset: hold everything in reset until the debouncer has had
    // time to settle, so the design does not start against an unknown button
    // state. Counts the same interval, once.
    logic [CW-1:0] por_cnt;
    logic          por_done;
    always @(posedge clk_board) begin
        if (!por_done) begin
            if (por_cnt == DEBOUNCE[CW-1:0]) por_done <= 1'b1;
            else                             por_cnt  <= por_cnt + 1'b1;
        end
    end

    logic rst_src_n;
    assign rst_src_n = btn_stable && por_done;

    // ---- per-domain deassertion synchronisers ----
    logic [1:0] sync_cpu, sync_vga;

    always_ff @(posedge clk_cpu or negedge rst_src_n) begin
        if (!rst_src_n) sync_cpu <= 2'b00;
        else            sync_cpu <= {sync_cpu[0], 1'b1};
    end
    assign rst_n = sync_cpu[1];

    always_ff @(posedge clk_vga or negedge rst_src_n) begin
        if (!rst_src_n) sync_vga <= 2'b00;
        else            sync_vga <= {sync_vga[0], 1'b1};
    end
    assign rst_vga_n = sync_vga[1];

    initial begin
        clk_sys_r  = 1'b0;
        btn_meta   = 1'b1;
        btn_sync   = 1'b1;
        btn_stable = 1'b0;      // start held in reset
        db_cnt     = '0;
        por_cnt    = '0;
        por_done   = 1'b0;
    end

endmodule
