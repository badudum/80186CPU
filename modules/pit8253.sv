// ---------------------------------------------------------------------------
// pit8253 — the PC's programmable interval timer, far enough to be useful.
//
// Hierarchy: FPGA80186 -> pit8253
//            (registers from io_decode, channel 0's output to cpu_top)
// Testbench: sim/tb_pit.sv
//
// WHY THIS EXISTS. This machine's timer interrupt comes from the 80186's own
// timer, which the BIOS programs for the PC's 18.2 Hz. That is right for
// software which asks the BIOS what time it is, and wrong for software which
// does what PC games do: reprogram channel 0 for a much faster tick, hook
// IRQ0, and count its own interrupts. Such a program does not fail -- it runs,
// at the ratio between the rate it asked for and the rate it gets. Doom8088
// asks for about 140 Hz, gets 18.2, and advances roughly one frame every two
// seconds.
//
// So channel 0 is a real counter here, and when software programs it, IT
// becomes the source of the timer interrupt in place of the 80186's timer.
// Until then nothing changes, so software that never touches 40h-43h -- which
// is most of DOS -- keeps exactly the behaviour it had.
//
// THE INPUT CLOCK is 1.193182 MHz on a PC, from a 14.31818 MHz crystal
// divided by twelve. At 25 MHz the nearest integer divisor is 21, giving
// 1.190 MHz: 0.25% slow, which is a few seconds a day on a clock nobody is
// setting their watch by, and far below the error in anything this drives.
//
// WHAT IS NOT HERE, and why it is honest to say so rather than pretend:
//   * BCD counting. Nothing has used it since the 1980s.
//   * Modes are all treated as a rate generator -- count down, pulse at zero,
//     reload. That is modes 2 and 3, which is what every tick programmer uses.
//     A one-shot (mode 0) will therefore retrigger rather than stop.
//   * Channels 1 and 2 count and read back but drive nothing. Channel 1 was
//     DRAM refresh and channel 2 the speaker; neither exists here.
//   * The read-back command (8254) is not implemented, only the 8253 latch.
// ---------------------------------------------------------------------------

`timescale 1ns/1ns
module pit8253 #(
    // System clocks per PIT tick: 25 MHz / 1.193182 MHz.
    parameter int CLK_DIV = 21
) (
    input  logic       clk,
    input  logic       rst_n,

    // register interface: 40h-42h are the counters, 43h the control word
    input  logic       sel,
    input  logic [1:0] port,
    input  logic       rd,
    input  logic       wr,
    input  logic [7:0] wdata,
    output logic [7:0] rdata,

    // channel 0's terminal count, one system clock wide
    output logic       irq0,
    // set once software has programmed channel 0, which is what hands it the
    // timer interrupt
    output logic       ch0_programmed
);

    localparam logic [1:0] ACC_LATCH = 2'd0;
    localparam logic [1:0] ACC_LO    = 2'd1;
    localparam logic [1:0] ACC_HI    = 2'd2;
    localparam logic [1:0] ACC_BOTH  = 2'd3;

    // ---- the 1.193 MHz tick ----
    logic [$clog2(CLK_DIV)-1:0] presc;
    logic                       pit_tick;
    assign pit_tick = (presc == CLK_DIV[$clog2(CLK_DIV)-1:0] - 1);

    logic [15:0] count  [0:2];
    logic [15:0] reload [0:2];
    logic [1:0]  access [0:2];
    logic [15:0] latch  [0:2];
    logic [2:0]  latched;
    logic [2:0]  wr_hi;          // a lo/hi write is expecting the high byte
    logic [2:0]  rd_hi;          // ...and likewise for reads
    logic [2:0]  armed;          // a reload value has been written

    logic [1:0] ch;
    assign ch = port;

    // Reads never have a side effect on anything but the lo/hi sequence, so
    // the data itself can be combinational.
    logic [15:0] rd_src;
    assign rd_src = latched[ch] ? latch[ch] : count[ch];
    always_comb begin
        if (port == 2'd3)                rdata = 8'h00;   // control is write-only
        else if (access[ch] == ACC_HI)   rdata = rd_src[15:8];
        else if (access[ch] == ACC_LO)   rdata = rd_src[7:0];
        else                             rdata = rd_hi[ch] ? rd_src[15:8]
                                                           : rd_src[7:0];
    end

    assign ch0_programmed = armed[0];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            presc   <= '0;
            irq0    <= 1'b0;
            latched <= 3'b000;
            wr_hi   <= 3'b000;
            rd_hi   <= 3'b000;
            armed   <= 3'b000;
            for (int i = 0; i < 3; i++) begin
                count[i]  <= 16'hFFFF;
                reload[i] <= 16'h0000;
                access[i] <= ACC_BOTH;
                latch[i]  <= 16'h0000;
            end
        end else begin
            irq0 <= 1'b0;

            // ---- count down ----
            if (presc == CLK_DIV[$clog2(CLK_DIV)-1:0] - 1) presc <= '0;
            else                                           presc <= presc + 1'b1;

            if (pit_tick) begin
                for (int i = 0; i < 3; i++) begin
                    if (count[i] == 16'd0 || count[i] == 16'd1) begin
                        // A reload of zero means 65536, which is what the
                        // BIOS's 18.2 Hz actually programs.
                        count[i] <= (reload[i] == 16'd0) ? 16'hFFFF
                                                         : reload[i] - 16'd1;
                        if (i == 0 && armed[0]) irq0 <= 1'b1;
                    end else begin
                        count[i] <= count[i] - 16'd1;
                    end
                end
            end

            // ---- register access ----
            if (sel && wr) begin
                if (port == 2'd3) begin
                    // Control word: channel, access mode, mode, BCD.
                    if (wdata[5:4] == ACC_LATCH) begin
                        // Latch the live count for a stable read.
                        latch[wdata[7:6]]   <= count[wdata[7:6]];
                        latched[wdata[7:6]] <= 1'b1;
                        rd_hi[wdata[7:6]]   <= 1'b0;
                    end else begin
                        access[wdata[7:6]]  <= wdata[5:4];
                        latched[wdata[7:6]] <= 1'b0;
                        wr_hi[wdata[7:6]]   <= 1'b0;
                        rd_hi[wdata[7:6]]   <= 1'b0;
                    end
                end else begin
                    case (access[ch])
                        ACC_LO: begin
                            reload[ch] <= {8'h00, wdata};
                            count[ch]  <= {8'h00, wdata};
                            armed[ch]  <= 1'b1;
                        end
                        ACC_HI: begin
                            reload[ch] <= {wdata, 8'h00};
                            count[ch]  <= {wdata, 8'h00};
                            armed[ch]  <= 1'b1;
                        end
                        default: begin
                            // Low byte then high byte; the counter only
                            // restarts once both have arrived, which is why
                            // the halves cannot simply be written through.
                            if (!wr_hi[ch]) begin
                                reload[ch][7:0] <= wdata;
                                wr_hi[ch]       <= 1'b1;
                            end else begin
                                reload[ch][15:8] <= wdata;
                                count[ch]        <= {wdata, reload[ch][7:0]};
                                wr_hi[ch]        <= 1'b0;
                                armed[ch]        <= 1'b1;
                            end
                        end
                    endcase
                end
            end

            if (sel && rd && port != 2'd3) begin
                if (access[ch] == ACC_BOTH) begin
                    rd_hi[ch] <= ~rd_hi[ch];
                    if (rd_hi[ch]) latched[ch] <= 1'b0;   // both halves taken
                end else begin
                    latched[ch] <= 1'b0;
                end
            end
        end
    end

endmodule
