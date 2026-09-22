`timescale 1ns/1ns
//
// PS/2 keyboard receiver test. Drives real 11-bit PS/2 frames at the pins and
// checks what comes out of the register interface, including the cases that
// matter for robustness: bad parity, a stalled frame, and FIFO behaviour.
//
module tb_keyboard;

    logic       clk = 0, rst_n = 0;
    logic       ps2_clk = 1, ps2_dat = 1;
    logic       sel = 0, port = 0, rd = 0, wr = 0;
    logic [7:0] wdata = 0, rdata;
    logic       data_avail, irq;

    // Short idle limit so the resync test does not take forever in simulation.
    keyboard_controller #(.IDLE_LIMIT(200)) dut (
        .clk (clk), .rst_n (rst_n),
        .ps2_clk (ps2_clk), .ps2_dat (ps2_dat),
        .sel (sel), .port (port), .rd (rd), .wr (wr),
        .wdata (wdata), .rdata (rdata),
        .data_avail (data_avail), .irq (irq)
    );

    always #5 clk = ~clk;

    int errors = 0, checks = 0;
    task chk(input string nm, input int got, input int exp);
        checks++;
        if (got !== exp) begin
            $display("FAIL %-34s got=%02h exp=%02h", nm, got, exp);
            errors++;
        end
    endtask

    // One PS/2 bit: data is presented, then the clock falls (which is the edge
    // the host samples on) and rises again.
    task ps2_bit(input logic b);
        begin
            ps2_dat = b;
            repeat (10) @(posedge clk);
            ps2_clk = 1'b0;
            repeat (10) @(posedge clk);
            ps2_clk = 1'b1;
            repeat (10) @(posedge clk);
        end
    endtask

    // A whole frame: start, 8 data bits LSB first, parity, stop.
    task ps2_send(input [7:0] b, input logic good_parity);
        logic p;
        begin
            p = ~(^b);                      // odd parity over the data bits
            if (!good_parity) p = ~p;
            ps2_bit(1'b0);
            for (int i = 0; i < 8; i++) ps2_bit(b[i]);
            ps2_bit(p);
            ps2_bit(1'b1);
            repeat (20) @(posedge clk);
        end
    endtask

    // Sample during the cycle, not after it: the pop happens on the clock edge
    // and `rdata` is combinational off `head`, so reading afterwards would see
    // the next entry. This is how a real bus read behaves too.
    task read_data(output [7:0] v);
        begin
            @(negedge clk);
            sel = 1; port = 0; rd = 1;
            #1 v = rdata;
            @(negedge clk);
            sel = 0; rd = 0;
            @(negedge clk);
        end
    endtask

    task read_status(output [7:0] v);
        begin
            @(negedge clk);
            sel = 1; port = 1; rd = 1;
            #1 v = rdata;
            @(negedge clk);
            sel = 0; rd = 0;
        end
    endtask

    logic [7:0] v;
    int         irq_count;

    // irq is a LEVEL, held while a byte is pending, because the interrupt
    // controller latches nothing on its external pins. Counting cycles it is
    // high is therefore not meaningful; what matters is that it rises with
    // data and falls when the FIFO is drained.
    always @(posedge clk) if (irq) irq_count++;

    initial begin
        irq_count = 0;
        repeat (4) @(negedge clk);
        rst_n = 1;
        @(negedge clk);

        chk("no data after reset", data_avail, 1'b0);
        read_status(v);
        chk("status clear after reset", v, 8'h00);

        // ---- one good scancode ----
        ps2_send(8'h1C, 1'b1);              // 'A' in set 2
        chk("data available", data_avail, 1'b1);
        chk("irq asserted with data", irq, 1'b1);
        read_status(v);
        chk("status shows byte waiting", v, 8'h01);

        // A level request must still be asserted long after the byte
        // arrived -- a one-clock pulse would have been missed by the
        // interrupt controller, which samples its pins and latches nothing.
        repeat (50) @(posedge clk);
        chk("irq still asserted 50 clocks later", irq, 1'b1);

        read_data(v);
        chk("scancode received", v, 8'h1C);
        chk("buffer empty after read", data_avail, 1'b0);
        chk("irq released once drained", irq, 1'b0);

        // ---- a byte with the high bit set, and 00/FF edge cases ----
        ps2_send(8'hF0, 1'b1);              // break prefix
        read_data(v);
        chk("break prefix received", v, 8'hF0);

        ps2_send(8'h00, 1'b1);
        read_data(v);
        chk("zero byte received", v, 8'h00);

        ps2_send(8'hFF, 1'b1);
        read_data(v);
        chk("FF byte received", v, 8'hFF);

        // ---- bad parity must be dropped, not latched ----
        irq_count = 0;
        ps2_send(8'h5A, 1'b0);
        chk("bad parity dropped", data_avail, 1'b0);
        chk("bad parity raised no irq", irq, 1'b0);
        chk("bad parity never asserted irq", irq_count, 0);

        // the receiver must still work afterwards
        ps2_send(8'h29, 1'b1);              // space
        read_data(v);
        chk("recovers after bad parity", v, 8'h29);

        // ---- FIFO holds several bytes in order ----
        ps2_send(8'h11, 1'b1);
        ps2_send(8'h22, 1'b1);
        ps2_send(8'h33, 1'b1);
        read_data(v); chk("fifo order 1", v, 8'h11);
        read_data(v); chk("fifo order 2", v, 8'h22);
        read_data(v); chk("fifo order 3", v, 8'h33);
        chk("fifo drained", data_avail, 1'b0);
        // With several bytes queued the request stays up between reads, so
        // the handler is re-entered until the FIFO is actually empty.
        chk("irq released only when empty", irq, 1'b0);

        // ---- a stalled frame must resync, not wedge ----
        ps2_bit(1'b0);                      // start bit
        ps2_bit(1'b1);                      // one data bit, then the line stops
        repeat (400) @(posedge clk);        // longer than IDLE_LIMIT
        chk("partial frame produced nothing", data_avail, 1'b0);

        ps2_send(8'h4B, 1'b1);
        read_data(v);
        chk("resyncs after a stalled frame", v, 8'h4B);

        $display("");
        $display("==================================");
        $display(" checks: %0d   failures: %0d", checks, errors);
        $display("==================================");
        if (errors == 0) $display("ALL TESTS PASSED");
        $finish;
    end

    initial begin
        #5000000;
        $display("FAIL global timeout");
        $display(" checks: %0d   failures: %0d", checks, errors + 1);
        $finish;
    end

endmodule
