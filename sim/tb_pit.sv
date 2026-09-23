`timescale 1ns/1ns
//
// The PC interval timer at 40h-43h.
//
// The thing that matters is the RATE. Software that reprograms channel 0 is
// counting its own interrupts to keep time, so a tick at the wrong frequency
// is not a cosmetic fault -- it makes the program run at the ratio between
// what it asked for and what it got. Doom8088 asks for about 140 Hz; on the
// 80186's own 18.2 Hz timer it advanced roughly a frame every two seconds.
// So these tests measure the interval rather than merely checking that
// something pulsed.
//
module tb_pit;

    localparam int DIV = 21;          // 25 MHz / 1.193 MHz

    logic clk = 0, rst_n = 0;
    always #20 clk = ~clk;            // 25 MHz

    logic       sel = 0, rd = 0, wr = 0;
    logic [1:0] port = 0;
    logic [7:0] wdata = 0, rdata;
    logic       irq0, ch0_programmed;

    pit8253 #(.CLK_DIV(DIV)) dut (
        .clk (clk), .rst_n (rst_n),
        .sel (sel), .port (port), .rd (rd), .wr (wr),
        .wdata (wdata), .rdata (rdata),
        .irq0 (irq0), .ch0_programmed (ch0_programmed)
    );

    int errors = 0, checks = 0;
    task chk(input string nm, input int got, input int exp);
        checks++;
        if (got !== exp) begin
            $display("FAIL %-48s got=%0d exp=%0d", nm, got, exp);
            errors++;
        end
    endtask

    task automatic io_wr(input [1:0] p, input [7:0] v);
        begin
            @(negedge clk);
            sel = 1; port = p; wdata = v; wr = 1;
            @(negedge clk);
            wr = 0; sel = 0;
        end
    endtask

    task automatic io_rd(input [1:0] p, output [7:0] v);
        begin
            @(negedge clk);
            sel = 1; port = p; rd = 1;
            #1 v = rdata;                    // combinational; let it settle
            @(negedge clk);                  // the posedge between toggles the
            rd = 0; sel = 0;                 // lo/hi sequence
        end
    endtask

    int cyc, t1, t2;
    logic [7:0] lo, hi;

    initial begin
        repeat (4) @(negedge clk);
        rst_n = 1;
        repeat (2) @(negedge clk);

        chk("channel 0 is not the tick source until programmed",
            ch0_programmed, 1'b0);

        // ---- programme channel 0 for a divisor of 100 ----
        // Control word: channel 0, lo/hi access, mode 2, binary.
        io_wr(2'd3, 8'h34);
        io_wr(2'd0, 8'd100);             // low byte
        chk("one half of a lo/hi divisor does not arm it yet",
            ch0_programmed, 1'b0);
        io_wr(2'd0, 8'd0);               // high byte completes it
        chk("channel 0 now owns the tick", ch0_programmed, 1'b1);

        // ---- measure the interval between two pulses ----
        cyc = 0;
        while (!irq0 && cyc < 100000) begin @(posedge clk); cyc++; end
        cyc = 0;
        @(posedge clk);
        while (!irq0 && cyc < 100000) begin @(posedge clk); cyc++; end
        t1 = cyc + 1;
        $display("  divisor 100 -> %0d system clocks between pulses (expect %0d)",
                 t1, 100 * DIV);
        checks++;
        if (t1 < 100 * DIV - DIV || t1 > 100 * DIV + DIV) begin
            $display("FAIL interval %0d is not %0d +/- one PIT tick",
                     t1, 100 * DIV);
            errors++;
        end

        // ---- halving the divisor must halve the interval ----
        io_wr(2'd3, 8'h34);
        io_wr(2'd0, 8'd50);
        io_wr(2'd0, 8'd0);
        cyc = 0;
        while (!irq0 && cyc < 100000) begin @(posedge clk); cyc++; end
        cyc = 0;
        @(posedge clk);
        while (!irq0 && cyc < 100000) begin @(posedge clk); cyc++; end
        t2 = cyc + 1;
        $display("  divisor 50  -> %0d system clocks between pulses (expect %0d)",
                 t2, 50 * DIV);
        checks++;
        if (t2 < 50 * DIV - DIV || t2 > 50 * DIV + DIV) begin
            $display("FAIL interval %0d is not %0d +/- one PIT tick", t2, 50 * DIV);
            errors++;
        end
        // The ratio is the whole point: a timer that pulses at a fixed rate
        // regardless of the divisor passes every "did it pulse" check.
        checks++;
        if (!(t1 > t2 * 3 / 2)) begin
            $display("FAIL halving the divisor did not halve the rate (%0d vs %0d)",
                     t1, t2);
            errors++;
        end

        // ---- the counter must be readable, and must be moving ----
        io_wr(2'd3, 8'h34);
        io_wr(2'd0, 8'h00);
        io_wr(2'd0, 8'hFF);              // divisor FF00, a long one
        io_wr(2'd3, 8'h00);              // latch channel 0
        io_rd(2'd0, lo);
        io_rd(2'd0, hi);
        t1 = {hi, lo};
        repeat (DIV * 40) @(negedge clk);
        io_wr(2'd3, 8'h00);
        io_rd(2'd0, lo);
        io_rd(2'd0, hi);
        t2 = {hi, lo};
        $display("  latched count went %0d -> %0d", t1, t2);
        checks++;
        if (!(t2 < t1 && (t1 - t2) >= 20 && (t1 - t2) <= 60)) begin
            $display("FAIL the counter did not count down by about 40 (%0d -> %0d)",
                     t1, t2);
            errors++;
        end

        // A latched value must not move while it is being read, or a program
        // reading the halves separately can see a value that never existed.
        io_wr(2'd3, 8'h00);
        io_rd(2'd0, lo);
        repeat (DIV * 10) @(negedge clk);
        io_rd(2'd0, hi);
        checks++;
        if ({hi, lo} > t2 || t2 - {hi, lo} > 5) begin
            $display("FAIL latched value moved while being read (%0d, was %0d)",
                     {hi, lo}, t2);
            errors++;
        end

        $display("");
        $display("==================================");
        $display(" checks: %0d   failures: %0d", checks, errors);
        $display("==================================");
        if (errors == 0) $display("ALL TESTS PASSED");
        $finish;
    end

    initial begin
        #20000000;
        $display("FAIL global timeout");
        $display(" checks: %0d   failures: %0d", checks, errors + 1);
        $finish;
    end

endmodule
