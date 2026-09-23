`timescale 1ns/1ns
//
// Memory subsystem and chip-select test. These are driven directly rather than
// through the CPU, because what matters here is region decode, byte lanes and
// ready timing -- all of which are easier to pin down at the module boundary
// than by inferring them from program behaviour.
//
module tb_memsys;

    logic        clk = 0, clk_vga = 0, rst_n = 0;
    logic [19:0] addr = 0;
    logic [15:0] wdata = 0, rdata;
    logic        rd = 0, wr = 0, bhe = 1, a0 = 0;
    logic        ready;
    logic [10:0] vram_read_addr = 0;
    logic [15:0] vram_read_data;

    // SDRAM pins are unused in this configuration but still have to be tied.
    logic [12:0] dram_addr;
    logic [1:0]  dram_ba, dram_dqm;
    wire  [15:0] dram_dq;
    logic        dram_cke, dram_cs_n, dram_ras_n, dram_cas_n, dram_we_n, dram_clk;

    // This test uses the on-chip fallback, so the extra SDRAM requesters are
    // unused -- but they are tied off explicitly rather than left unconnected,
    // so flipping USE_SDRAM here would not silently feed the arbiter X's.
    logic [1:0]  ext_rd = 2'b00, ext_wr = 2'b00, ext_ready;
    logic [23:0] ext_addr  [2];
    logic [15:0] ext_wdata [2];
    logic [1:0]  ext_be    [2];
    logic [15:0] ext_rdata;
    initial for (int k = 0; k < 2; k++) begin
        ext_addr[k] = 24'h000000; ext_wdata[k] = 16'h0000; ext_be[k] = 2'b11;
    end

    memory_controller #(.RAM_KB(32), .USE_SDRAM(1'b0)) u_mem (
        .clk (clk), .rst_n (rst_n),
        .addr (addr), .wdata (wdata), .rdata (rdata),
        .rd (rd), .wr (wr), .bhe (bhe), .a0 (a0), .ready (ready),
        .clk_vga (clk_vga),
        .vram_read_addr (vram_read_addr), .vram_read_data (vram_read_data),
        .ext_rd (ext_rd), .ext_wr (ext_wr), .ext_addr (ext_addr),
        .ext_wdata (ext_wdata), .ext_be (ext_be),
        .ext_ready (ext_ready), .ext_rdata (ext_rdata),
        .dram_addr (dram_addr), .dram_ba (dram_ba), .dram_dq (dram_dq),
        .dram_cke (dram_cke), .dram_cs_n (dram_cs_n), .dram_ras_n (dram_ras_n),
        .dram_cas_n (dram_cas_n), .dram_we_n (dram_we_n), .dram_dqm (dram_dqm),
        .dram_clk_in (~clk),
        .dram_clk (dram_clk)
    );

    // chip-select unit, exercised through its PCB bus
    logic        cs_sel = 0, cs_we = 0, cs_re = 0;
    logic [7:1]  cs_off = 0;
    logic [15:0] cs_wdata = 0, cs_rdata;
    logic        ucs_n, lcs_n;
    logic [3:0]  mcs_n;
    logic [6:0]  pcs_n;
    logic [1:0]  wait_cnt;
    logic        use_ext_ready;
    logic [19:0] cs_addr = 0;
    logic        cs_io = 0;

    chip_select u_cs (
        .clk (clk), .rst_n (rst_n),
        .sel (cs_sel), .pcb_off (cs_off), .pcb_wdata (cs_wdata),
        .pcb_we (cs_we), .pcb_re (cs_re), .pcb_rdata (cs_rdata),
        .addr (cs_addr), .io_space (cs_io),
        .ucs_n (ucs_n), .lcs_n (lcs_n), .mcs_n (mcs_n), .pcs_n (pcs_n),
        .wait_cnt (wait_cnt), .use_ext_ready (use_ext_ready)
    );

    always #5  clk     = ~clk;
    always #7  clk_vga = ~clk_vga;      // deliberately unrelated to clk

    int errors = 0, checks = 0;
    task chk(input string nm, input int got, input int exp);
        checks++;
        if (got !== exp) begin
            $display("FAIL %-34s got=%04h exp=%04h", nm, got, exp);
            errors++;
        end
    endtask

    // One bus write: lanes selected by bhe/a0 exactly as the BIU drives them.
    task bus_write(input [19:0] a, input [15:0] d, input w_bhe, input w_a0);
        begin
            @(negedge clk);
            addr = a; wdata = d; bhe = w_bhe; a0 = w_a0; wr = 1;
            @(negedge clk);
            wr = 0;
            @(negedge clk);
        end
    endtask

    task bus_read(input [19:0] a, output [15:0] d);
        begin
            @(negedge clk);
            addr = a; bhe = 0; a0 = 0; rd = 1;
            @(negedge clk);
            rd = 0;
            d = rdata;
            @(negedge clk);
        end
    endtask

    task cs_write(input [7:0] off, input [15:0] d);
        begin
            @(negedge clk);
            cs_off = off[7:1]; cs_wdata = d; cs_sel = 1; cs_we = 1;
            @(negedge clk);
            cs_sel = 0; cs_we = 0;
        end
    endtask

    logic [15:0] d;

    initial begin
        repeat (3) @(negedge clk);
        rst_n = 1;
        @(negedge clk);

        // ---------------- conventional RAM ----------------
        bus_write(20'h01000, 16'h1234, 1'b0, 1'b0);
        bus_read (20'h01000, d);
        chk("RAM word write/read", d, 16'h1234);

        // byte lanes: write only the low byte, upper must survive
        bus_write(20'h01000, 16'h00AA, 1'b1, 1'b0);
        bus_read (20'h01000, d);
        chk("RAM low-byte write", d, 16'h12AA);

        // write only the high byte
        bus_write(20'h01001, 16'hBB00, 1'b0, 1'b1);
        bus_read (20'h01000, d);
        chk("RAM high-byte write", d, 16'hBBAA);

        // a different address must be independent
        bus_write(20'h01002, 16'h5678, 1'b0, 1'b0);
        bus_read (20'h01000, d);
        chk("neighbouring word untouched", d, 16'hBBAA);
        bus_read (20'h01002, d);
        chk("neighbouring word readable", d, 16'h5678);

        // ---------------- above implemented RAM ----------------
        // 32 KB implemented, so 40000 is real address space but unbacked.
        bus_read (20'h40000, d);
        chk("unmapped reads as FFFF", d, 16'hFFFF);

        // ---------------- video RAM ----------------
        bus_write(20'hB8000, 16'h0741, 1'b0, 1'b0);   // 'A', grey on black
        bus_read (20'hB8000, d);
        chk("VRAM write/read", d, 16'h0741);

        bus_write(20'hB8002, 16'h0742, 1'b0, 1'b0);
        bus_read (20'hB8002, d);
        chk("VRAM second cell", d, 16'h0742);

        // VRAM must not alias onto conventional RAM
        bus_read (20'h01000, d);
        chk("RAM unaffected by VRAM write", d, 16'hBBAA);

        // the VGA-side port sees what the CPU wrote, on its own clock
        vram_read_addr = 11'd0;
        repeat (4) @(posedge clk_vga);
        chk("VGA port reads cell 0", vram_read_data, 16'h0741);
        vram_read_addr = 11'd1;
        repeat (4) @(posedge clk_vga);
        chk("VGA port reads cell 1", vram_read_data, 16'h0742);

        // ---------------- boot ROM ----------------
        // FFFF0 is the reset vector, and the image built by tools/gen_bios.py
        // starts there with a far jump: EA 00 01 00 F0 -> JMP F000:0100. Word
        // zero of that is 00EA.
        //
        // This deliberately checks the real contents rather than expecting
        // zeros. Expecting zeros would pass just as happily if the ROM were
        // never loaded or were mapped at the wrong address, which is exactly
        // the failure that would leave the board dead at power-on.
        bus_read (20'hFFFF0, d);
        chk("reset vector readable from ROM", d, 16'h00EA);

        // ROM is read-only: a write must be ignored, not corrupt it
        bus_write(20'hFFFF0, 16'hDEAD, 1'b0, 1'b0);
        bus_read (20'hFFFF0, d);
        chk("ROM ignores writes", d, 16'h00EA);

        // ---------------- ready timing ----------------
        @(negedge clk);
        chk("ready low while idle", ready, 1'b0);
        addr = 20'h01000; rd = 1;
        @(negedge clk);
        chk("ready asserted for access", ready, 1'b1);
        rd = 0;
        @(negedge clk);
        chk("ready drops after access", ready, 1'b0);

        // ---------------- chip-select unit ----------------
        // UMCS resets to FFFB: top 1 KB, 3 wait states, external ready used.
        cs_off = 8'hA0 >> 1; cs_sel = 1; cs_re = 1; #1;
        chk("UMCS reset value", cs_rdata, 16'hFFFB);
        cs_sel = 0; cs_re = 0;

        // the reset vector must fall inside UCS out of reset
        cs_addr = 20'hFFFF0; cs_io = 0; #1;
        chk("UCS covers the reset vector", ucs_n, 1'b0);
        chk("UCS wait states = 3",          wait_cnt, 2'd3);
        chk("UCS factors external ready",   use_ext_ready, 1'b1);

        // ...and low memory must not
        cs_addr = 20'h01000; #1;
        chk("UCS inactive in low memory", ucs_n, 1'b1);

        // reprogram UMCS for a 64 KB window and re-check
        cs_write(8'hA0, 16'hF038);          // start F0000, 0 wait states
        cs_addr = 20'hF0000; #1;
        chk("UCS follows reprogramming", ucs_n, 1'b0);
        chk("UCS new wait states",       wait_cnt, 2'd0);
        cs_addr = 20'hEFFFF; #1;
        chk("UCS excludes below the base", ucs_n, 1'b1);

        // PCS lines stay inactive until both PACS and MPCS have been touched
        chk("PCS inactive before arming", pcs_n, 7'h7F);

        $display("");
        $display("==================================");
        $display(" checks: %0d   failures: %0d", checks, errors);
        $display("==================================");
        if (errors == 0) $display("ALL TESTS PASSED");
        $finish;
    end

    initial begin
        #200000;
        $display("FAIL global timeout");
        $display(" checks: %0d   failures: %0d", checks, errors + 1);
        $finish;
    end

endmodule
