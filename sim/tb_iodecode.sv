`timescale 1ns/1ns
//
// I/O port decode driven with REAL bus timing.
//
// tb_keyboard already checks the keyboard controller thoroughly, but it drives
// the register interface with `rd` held for a single clock. The BIU does not:
// it asserts RD for the whole of T2 and T3, and longer still if the cycle
// takes wait states. Anything with a side effect on access -- a FIFO that
// pops, a counter that advances -- therefore sees a multi-clock strobe, and a
// module test that pulses for one clock cannot see the difference.
//
// That gap hid a real defect. With a level-sensitive pop, one `IN AL,60h`
// retired TWO scancodes, and because the BIU latches read data at the END of
// T3 while the first pop happened at the end of T2, the byte the CPU got was
// the one AFTER the one it asked for. Typing would have produced garbage on
// hardware while every module test passed.
//
// So this testbench models the bus cycle rather than the register interface:
//
//   T1  address driven
//   T2  RD asserted            <- ready is registered from RD here
//   T3  ready high             <- the BIU latches data at the END of T3
//   T4  RD released
//
// T3 repeats as Tw for as long as ready is low, and the BIU samples in the
// first cycle where it IS high -- so bus_read below waits for ready rather
// than counting clocks, exactly as the hardware does.
//
// THE INVARIANT that makes this safe for any future slow device on this bus:
// the device strobe must fire in the same cycle the CPU latches data, i.e.
// only ever while `ready` is high. io_decode derives the strobe from ready's
// rising edge, so that holds by construction however long the cycle takes.
// It is checked continuously below rather than at one sampled instant.
//
module tb_iodecode;

    logic        clk = 0, rst_n = 0;
    logic [15:0] io_addr = 16'h0000;
    logic        io_rd = 0, io_wr = 0;
    logic [15:0] wdata = 16'h0000, rdata;
    logic        ready;

    logic        kbd_sel, kbd_port, kbd_rd, kbd_wr;
    logic [7:0]  kbd_wdata, kbd_rdata;

    logic        ps2_clk = 1, ps2_dat = 1;
    logic        kbd_avail, kbd_irq;

    logic        crtc_sel, crtc_port, crtc_rd, crtc_wr;
    logic [7:0]  crtc_wdata, crtc_rdata;
    logic        cursor_en;
    logic [10:0] cursor_addr;

    logic        stor_sel, stor_rd, stor_wr;
    logic [2:0]  stor_reg;
    logic [15:0] stor_wdata, stor_rdata;

    io_decode u_io (
        .clk (clk), .rst_n (rst_n),
        .io_addr (io_addr), .io_rd (io_rd), .io_wr (io_wr),
        .wdata (wdata), .rdata (rdata), .ready (ready),
        .kbd_sel (kbd_sel), .kbd_port (kbd_port),
        .kbd_rd (kbd_rd), .kbd_wr (kbd_wr),
        .kbd_wdata (kbd_wdata), .kbd_rdata (kbd_rdata),
        .crtc_sel (crtc_sel), .crtc_port (crtc_port),
        .crtc_rd (crtc_rd), .crtc_wr (crtc_wr),
        .crtc_wdata (crtc_wdata), .crtc_rdata (crtc_rdata),
        .stor_sel (stor_sel), .stor_reg (stor_reg),
        .stor_rd (stor_rd), .stor_wr (stor_wr),
        .stor_wdata (stor_wdata), .stor_rdata (stor_rdata)
    );

    crtc u_crtc (
        .clk (clk), .rst_n (rst_n),
        .sel (crtc_sel), .port (crtc_port),
        .rd (crtc_rd), .wr (crtc_wr),
        .wdata (crtc_wdata), .rdata (crtc_rdata),
        .cursor_en (cursor_en), .cursor_addr (cursor_addr)
    );

    logic [23:0] st_maddr;
    logic [15:0] st_mwdata;
    logic [1:0]  st_mbe;
    logic        st_mrd, st_mwr;

    storage #(.SECTORS(256), .USE_SDRAM(1'b0)) u_stor (
        .clk (clk), .rst_n (rst_n),
        .sel (stor_sel), .reg_sel (stor_reg),
        .rd (stor_rd), .wr (stor_wr),
        .wdata (stor_wdata), .rdata (stor_rdata),
        .mem_addr (st_maddr), .mem_wdata (st_mwdata), .mem_be (st_mbe),
        .mem_rd (st_mrd), .mem_wr (st_mwr),
        .mem_rdata (16'h0000), .mem_ready (1'b0)
    );

    keyboard_controller #(.IDLE_LIMIT(200)) u_kbd (
        .clk (clk), .rst_n (rst_n),
        .ps2_clk (ps2_clk), .ps2_dat (ps2_dat),
        .sel (kbd_sel), .port (kbd_port), .rd (kbd_rd), .wr (kbd_wr),
        .wdata (kbd_wdata), .rdata (kbd_rdata),
        .data_avail (kbd_avail), .irq (kbd_irq)
    );

    always #5 clk = ~clk;

    int errors = 0, checks = 0;
    task chk(input string nm, input int got, input int exp);
        checks++;
        if (got !== exp) begin
            $display("FAIL %-44s got=%04h exp=%04h", nm, got, exp);
            errors++;
        end
    endtask

    // Count how many times the device actually saw an access strobe, which is
    // what "exactly once per bus cycle" means concretely.
    int kbd_rd_pulses;
    always @(posedge clk) if (rst_n && kbd_rd) kbd_rd_pulses <= kbd_rd_pulses + 1;

    // The invariant: a strobe outside a ready cycle would mean the device
    // acted before (or after) the CPU took the data.
    int strobe_outside_ready;
    always @(posedge clk)
        if (rst_n && kbd_rd && !ready) strobe_outside_ready <= strobe_outside_ready + 1;

    // ---- PS/2 frame injection (same shape as tb_keyboard) ----
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

    task ps2_send(input [7:0] b);
        logic p;
        begin
            p = ~(^b);                      // odd parity
            ps2_bit(1'b0);
            for (int i = 0; i < 8; i++) ps2_bit(b[i]);
            ps2_bit(p);
            ps2_bit(1'b1);
            repeat (20) @(posedge clk);
        end
    endtask

    // ---- a bus cycle shaped like the BIU's ----
    task bus_read(input [15:0] a, output [15:0] d);
        begin
            @(negedge clk);
            io_addr = a;
            io_rd   = 1'b1;                 // T2
            @(negedge clk);                 // T3
            while (!ready) @(negedge clk);  // Tw, as long as the device needs
            d = rdata;                      // what the BIU latches this cycle
            @(negedge clk);
            io_rd = 1'b0;                   // T4
            @(negedge clk);
        end
    endtask

    task bus_write(input [15:0] a, input [15:0] v);
        begin
            @(negedge clk);
            io_addr = a; wdata = v;
            io_wr   = 1'b1;                 // T2
            @(negedge clk);                 // T3
            while (!ready) @(negedge clk);
            @(negedge clk);
            io_wr = 1'b0;                   // T4
            @(negedge clk);
        end
    endtask

    // A BYTE bus cycle. The BIU puts an odd-address byte on D15-D8 and an
    // even-address byte on D7-D0 (biu.sv, cur_dout), and takes read data from
    // the matching lane. Modelling that is the whole point: every byte port in
    // the design was at an even address until the CRTC landed at 3D5, so the
    // upper lane had never been exercised.
    task bus_write_byte(input [15:0] a, input [7:0] v);
        begin
            @(negedge clk);
            io_addr = a;
            wdata   = a[0] ? {v, 8'h00} : {8'h00, v};
            io_wr   = 1'b1;
            @(negedge clk);
            while (!ready) @(negedge clk);
            @(negedge clk);
            io_wr = 1'b0;
            @(negedge clk);
        end
    endtask

    task bus_read_byte(input [15:0] a, output [7:0] v);
        logic [15:0] w;
        begin
            bus_read(a, w);
            v = a[0] ? w[15:8] : w[7:0];
        end
    endtask

    logic [15:0] d;
    logic [7:0]  b;

    initial begin
        kbd_rd_pulses = 0;
        strobe_outside_ready = 0;
        repeat (4) @(negedge clk);
        rst_n = 1;
        @(negedge clk);

        // ---- three scancodes queued, read back over the real bus ----
        ps2_send(8'h11);
        ps2_send(8'h22);
        ps2_send(8'h33);

        chk("keyboard reports data waiting", kbd_avail, 1'b1);

        bus_read(16'h0064, d);
        chk("status port reads ready", d, 16'h0001);
        chk("reading status did not pop", kbd_avail, 1'b1);

        // A status read strobes the device too -- it just must not pop -- so
        // count only the data reads that follow.
        kbd_rd_pulses = 0;

        // Each of these must return the NEXT byte, not the one after it.
        bus_read(16'h0060, d);
        chk("bus read 1 returned the first scancode", d, 16'h0011);
        bus_read(16'h0060, d);
        chk("bus read 2 returned the second scancode", d, 16'h0022);
        bus_read(16'h0060, d);
        chk("bus read 3 returned the third scancode", d, 16'h0033);

        chk("FIFO drained by exactly three reads", kbd_avail, 1'b0);
        chk("one strobe per bus cycle", kbd_rd_pulses, 3);

        // ---- a longer run, to be sure it is not a one-off alignment ----
        kbd_rd_pulses = 0;
        ps2_send(8'h44);
        ps2_send(8'h55);

        bus_read(16'h0060, d);
        chk("later read returned the right byte", d, 16'h0044);
        bus_read(16'h0060, d);
        chk("later read 2 returned the right byte", d, 16'h0055);
        chk("FIFO drained again", kbd_avail, 1'b0);
        chk("still one strobe per cycle", kbd_rd_pulses, 2);

        // ---- reading an empty FIFO must not underflow ----
        bus_read(16'h0060, d);
        chk("empty read did not wrap the FIFO", kbd_avail, 1'b0);
        ps2_send(8'h66);
        bus_read(16'h0060, d);
        chk("FIFO still correct after an empty read", d, 16'h0066);

        // ---- port 61h: the refresh toggle must actually toggle ----
        // PC software calibrates delay loops by watching bit 4 change, in
        // loops with no timeout, because on real hardware it cannot fail to.
        // A port that reads a constant hangs them forever -- which is exactly
        // what an undecoded port does, and what MS-DOS hung on here.
        begin
            automatic int changes = 0;
            automatic logic [7:0] prev, cur;
            bus_read_byte(16'h0061, prev);
            for (int k = 0; k < 400; k++) begin
                bus_read_byte(16'h0061, cur);
                if (cur[4] !== prev[4]) changes++;
                prev = cur;
            end
            checks++;
            if (changes < 2) begin
                $display("FAIL port 61h bit 4 changed %0d times in 400 reads",
                         changes);
                errors++;
            end else begin
                $display("  port 61h bit 4 toggled %0d times in 400 reads",
                         changes);
            end
        end

        // Bits 1:0 are the speaker gate and data; code that writes them reads
        // first and puts them back, so they have to read back.
        bus_write_byte(16'h0061, 8'h03);
        bus_read_byte(16'h0061, b);
        chk("port 61h speaker bits read back", b[1:0], 2'b11);
        bus_write_byte(16'h0061, 8'h00);
        bus_read_byte(16'h0061, b);
        chk("port 61h speaker bits cleared", b[1:0], 2'b00);

        // ---- storage answers on its own ports, over the same bus ----
        // Proves the decode split works and that a second device on this bus
        // gets the same single-shot strobe the keyboard does.
        bus_read(16'h0328, d);
        chk("storage reports its size", d, 16'd256);

        // Sector 1 is the first FAT. Its first three bytes are the media
        // descriptor and the end-of-chain marker for the two reserved
        // entries, which is a recognisable fingerprint of a real FAT12
        // volume rather than of any convention invented here.
        bus_write(16'h0322, 16'd1);      // LBA_LO
        bus_write(16'h0324, 16'd0);      // LBA_HI
        bus_write(16'h0326, 16'd1);      // CMD = READ
        do bus_read(16'h0326, d); while (d[0]);   // poll BUSY
        chk("storage read completed without error", d[2], 1'b0);
        bus_read(16'h0320, d);
        chk("FAT[0] carries the media descriptor", d, 16'hFFF8);
        bus_read(16'h0320, d);
        chk("FAT[1] is the end-of-chain marker", d[7:0], 8'hFF);

        // A keyboard access must not have been disturbed by any of that.
        ps2_send(8'h77);
        bus_read(16'h0060, d);
        chk("keyboard still correct after storage traffic", d, 16'h0077);

        // ---- CRTC, and with it the odd-address byte lane ----
        // 3D4 is even and 3D5 is odd, so this pair exercises both lanes in
        // both directions. Getting the lane wrong here reads back as zero or
        // as the other half of the word.
        bus_write_byte(16'h03D4, 8'h0E);          // index := cursor high
        bus_read_byte (16'h03D4, b);
        chk("CRTC index reads back (even port)", b, 8'h0E);

        bus_write_byte(16'h03D5, 8'h03);          // data := 03  (ODD port)
        bus_read_byte (16'h03D5, b);
        chk("CRTC data reads back (odd port)", b, 8'h03);

        bus_write_byte(16'h03D4, 8'h0F);
        bus_write_byte(16'h03D5, 8'hC0);
        chk("cursor address set over the bus", cursor_addr, 11'h3C0);

        bus_write_byte(16'h03D4, 8'h0A);
        bus_write_byte(16'h03D5, 8'h20);          // bit 5 = cursor off
        chk("cursor disabled over the bus", cursor_en, 1'b0);
        bus_write_byte(16'h03D5, 8'h0E);
        chk("cursor re-enabled over the bus", cursor_en, 1'b1);

        // ---- an unclaimed port still terminates the cycle ----
        bus_read(16'h0378, d);
        chk("unclaimed port reads FFFF", d, 16'hFFFF);

        chk("no strobe ever fired outside a ready cycle", strobe_outside_ready, 0);

        // ---- ready must actually rise, or the CPU would hang ----
        @(negedge clk);
        io_addr = 16'h0378;
        io_rd   = 1'b1;
        @(negedge clk);
        chk("ready asserts for an unclaimed port", ready, 1'b1);
        @(negedge clk);
        io_rd = 1'b0;

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
