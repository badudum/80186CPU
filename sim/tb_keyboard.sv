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
        ps2_send(8'h1C, 1'b1);              // 'A': set 2 1C -> set 1 1E
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
        chk("scancode translated to set 1", v, 8'h1E);
        chk("buffer empty after read", data_avail, 1'b0);
        chk("irq released once drained", irq, 1'b0);

        // ---- a byte with the high bit set, and 00/FF edge cases ----
        // The break prefix is consumed, not forwarded: set 1 has no F0.
        ps2_send(8'hF0, 1'b1);
        chk("break prefix produces no byte of its own", data_avail, 1'b0);
        ps2_send(8'h1C, 1'b1);              // release of 'A'
        read_data(v);
        chk("release arrives as set 1 code with bit 7", v, 8'h9E);

        ps2_send(8'h00, 1'b1);
        read_data(v);
        chk("a code with no set 1 equivalent is dropped", data_avail, 1'b0);

        ps2_send(8'hFF, 1'b1);
        read_data(v);
        chk("FF (keyboard error) is dropped too", data_avail, 1'b0);

        // ---- bad parity must be dropped, not latched ----
        irq_count = 0;
        ps2_send(8'h5A, 1'b0);
        chk("bad parity dropped", data_avail, 1'b0);
        chk("bad parity raised no irq", irq, 1'b0);
        chk("bad parity never asserted irq", irq_count, 0);

        // the receiver must still work afterwards
        ps2_send(8'h29, 1'b1);              // space
        read_data(v);
        chk("recovers after bad parity", v, 8'h39);   // space -> set 1 39

        // ---- FIFO holds several bytes in order ----
        ps2_send(8'h11, 1'b1);
        ps2_send(8'h22, 1'b1);
        ps2_send(8'h33, 1'b1);
        read_data(v); chk("fifo order 1", v, 8'h38);   // 11 Alt   -> 38
        read_data(v); chk("fifo order 2", v, 8'h2D);   // 22 X     -> 2D
        read_data(v); chk("fifo order 3", v, 8'h23);   // 33 H     -> 23
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
        chk("resyncs after a stalled frame", v, 8'h26);   // 4B L -> 26

        // ---- extended keys ----
        // The arrows are E0-prefixed in both sets, and the SAME table applies
        // to the byte after the prefix. A game reads the E0, discards it, and
        // acts on the code that follows, so the pair has to arrive intact and
        // in order.
        ps2_send(8'hE0, 1'b1);
        ps2_send(8'h75, 1'b1);                          // Up
        read_data(v); chk("extended prefix passes through", v, 8'hE0);
        read_data(v); chk("...followed by set 1 Up", v, 8'h48);
        chk("nothing else queued", data_avail, 1'b0);

        // Releasing an extended key is E0 F0 <code> -- the F0 arrives SECOND,
        // after the prefix. If E0 cleared the pending-break flag this would
        // come back as a key press.
        ps2_send(8'hE0, 1'b1);
        ps2_send(8'hF0, 1'b1);
        ps2_send(8'h75, 1'b1);
        read_data(v); chk("extended release keeps its prefix", v, 8'hE0);
        read_data(v); chk("...and sets bit 7", v, 8'hC8);
        chk("extended release queued nothing more", data_avail, 1'b0);

        // ---- the bug this was all for ----
        // Set 2 signals a release as F0 <code>. Software that tests bit 7 --
        // which is every DOS game, DOOM8088 included -- reads the F0 prefix
        // itself as a key-up and then the real code as a key-DOWN, so keys
        // latch on. Press and release must produce exactly two bytes, the
        // second being the first with bit 7 set, and no 0xF0 anywhere.
        ps2_send(8'h1D, 1'b1);              // press W (set 2 1D -> set 1 11)
        ps2_send(8'hF0, 1'b1);
        ps2_send(8'h1D, 1'b1);              // release W
        read_data(v); chk("press produces the make code", v, 8'h11);
        read_data(v); chk("release produces make|80, not a second press",
                          v, 8'h91);
        chk("a press and release are exactly two bytes", data_avail, 1'b0);

        // ---- a break prefix stranded by a dropped frame ----
        // F0 arms the release flag and the code that follows consumes it. If
        // that code is lost to bad parity the flag is left armed, and the
        // next key would come back as a release -- a keypress the game never
        // sees. E0 clearing the flag is what stops one corrupt frame turning
        // into a stuck control.
        ps2_send(8'hF0, 1'b1);              // release prefix...
        ps2_send(8'h1C, 1'b0);              // ...whose code is corrupt
        chk("the corrupt frame produced nothing", data_avail, 1'b0);
        ps2_send(8'hE0, 1'b1);
        ps2_send(8'h75, 1'b1);              // Up, pressed
        read_data(v); chk("stranded break: prefix still passes", v, 8'hE0);
        read_data(v); chk("stranded break: Up is a PRESS, not a release",
                          v, 8'h48);

        // ---- the keys DOOM8088 actually binds ----
        // Its constants, from its own source: arrows 48/50/4B/4D, Ctrl 1D,
        // Alt 38, Shift 2A/36. These are the mappings that decide whether the
        // game is playable, so they are asserted by name rather than left to
        // a general table check.
        ps2_send(8'hE0, 1'b1); ps2_send(8'h72, 1'b1);
        read_data(v); read_data(v); chk("Down  -> 50", v, 8'h50);
        ps2_send(8'hE0, 1'b1); ps2_send(8'h6B, 1'b1);
        read_data(v); read_data(v); chk("Left  -> 4B", v, 8'h4B);
        ps2_send(8'hE0, 1'b1); ps2_send(8'h74, 1'b1);
        read_data(v); read_data(v); chk("Right -> 4D", v, 8'h4D);
        ps2_send(8'h14, 1'b1); read_data(v); chk("Ctrl  -> 1D (fire)",  v, 8'h1D);
        ps2_send(8'h11, 1'b1); read_data(v); chk("Alt   -> 38 (strafe)", v, 8'h38);
        ps2_send(8'h12, 1'b1); read_data(v); chk("LShift-> 2A (run)",   v, 8'h2A);
        ps2_send(8'h59, 1'b1); read_data(v); chk("RShift-> 36",         v, 8'h36);
        ps2_send(8'h5A, 1'b1); read_data(v); chk("Enter -> 1C (use)",   v, 8'h1C);
        ps2_send(8'h29, 1'b1); read_data(v); chk("Space -> 39 (use)",   v, 8'h39);
        ps2_send(8'h76, 1'b1); read_data(v); chk("Esc   -> 01 (menu)",  v, 8'h01);
        ps2_send(8'h0D, 1'b1); read_data(v); chk("Tab   -> 0F (map)",   v, 8'h0F);
        ps2_send(8'h09, 1'b1); read_data(v); chk("F10   -> 44 (quit)",  v, 8'h44);
        ps2_send(8'h54, 1'b1); read_data(v); chk("[     -> 1A (weapon)",v, 8'h1A);
        ps2_send(8'h5B, 1'b1); read_data(v); chk("]     -> 1B (weapon)",v, 8'h1B);
        ps2_send(8'h41, 1'b1); read_data(v); chk("comma -> 33 (strafe L)", v, 8'h33);
        ps2_send(8'h49, 1'b1); read_data(v); chk("period-> 34 (strafe R)", v, 8'h34);

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
