`timescale 1ns/1ns
//
// The whole machine booting with the disk in SDRAM instead of on-chip ROM.
//
// This is the test that says the backend swap is invisible. The BIOS, the
// FAT12 boot sector and KERNEL.BIN are byte-for-byte the same as in tb_bios;
// only DISK_IN_SDRAM changes. If the same seven lines appear on screen, then
// INT 13h, the boot sector's cluster walking and everything above them cannot
// tell which backend is underneath -- which is the whole point of putting the
// swap behind the device's register interface.
//
// The SDRAM model is preloaded with the disk image at BASE, which is exactly
// what the JTAG loader will do on hardware. SDRAM is volatile, so without that
// step the machine boots to a blank disk.
//
// Note the two things sharing the memory here: conventional RAM below 1 MB and
// the disk image above it, both going through sdram_arbiter to one controller
// port. That contention is not simulated anywhere else at this scale.
//
module tb_bios_sdramdisk;

    localparam int  DISK_SECTORS = 256;
    localparam int  DISK_BASE    = 24'h100000;

    logic       CLOCK_50 = 0;
    logic [0:0] KEY = 1'b1;
    logic [9:0] LEDR;
    logic [7:0] VGA_R, VGA_G, VGA_B;
    logic       VGA_HS, VGA_VS, VGA_CLK, VGA_BLANK_N, VGA_SYNC_N;
    logic       PS2_CLK = 1'b1, PS2_DAT = 1'b1;
    logic [12:0] DRAM_ADDR;
    logic [1:0]  DRAM_BA;
    wire  [15:0] DRAM_DQ;
    logic        DRAM_CKE, DRAM_CS_N, DRAM_RAS_N, DRAM_CAS_N, DRAM_WE_N;
    logic        DRAM_LDQM, DRAM_UDQM, DRAM_CLK;

    FPGA80186 #(
        .DISK_IN_SDRAM      (1'b1),
        .DISK_SECTORS       (DISK_SECTORS),
        .ENABLE_JTAG_LOADER (1'b0)
    ) dut (.*);
    defparam dut.u_clk_rst.DEBOUNCE = 20;
    defparam dut.u_mem.SDRAM_INIT_CYCLES = 40;

    sdram_model #(.CAS_LATENCY(2)) chip (
        .dram_clk (DRAM_CLK), .dram_cke (DRAM_CKE), .dram_cs_n (DRAM_CS_N),
        .dram_ras_n (DRAM_RAS_N), .dram_cas_n (DRAM_CAS_N), .dram_we_n (DRAM_WE_N),
        .dram_addr (DRAM_ADDR), .dram_ba (DRAM_BA),
        .dram_dqm ({DRAM_UDQM, DRAM_LDQM}), .dram_dq (DRAM_DQ)
    );

    always #10 CLOCK_50 = ~CLOCK_50;

    int errors = 0, checks = 0;
    task chk(input string nm, input int got, input int exp);
        checks++;
        if (got !== exp) begin
            $display("FAIL %-40s got=%04h exp=%04h", nm, got, exp);
            errors++;
        end
    endtask

    function automatic [7:0] cell_ch(input int n);
        cell_ch = dut.u_mem.u_vram.ram_lo[n];
    endfunction

    // Byte address -> index in the model's array; see tb_storage_sdram.
    function automatic int unsigned midx(input int unsigned byte_addr);
        midx = (byte_addr >> 1) & 21'h1FFFFF;
    endfunction

    string want [0:6] = '{
        "FPGA80186 BIOS -- 640K, VGA text, PS/2 keyboard, block storage.",
        "Booting from disk 0...",
        "Boot sector loaded, starting.",
        "Loading KERNEL.BIN",
        "Starting kernel.",
        "KERNEL.BIN loaded from FAT12 and running.",
        "hello"
    };

    logic [15:0] w604;
    string got;
    int i, r;

    logic [15:0] image [0:DISK_SECTORS*256-1];

    // Cycles during which the device was asking SDRAM for something. This is
    // a level held until the arbiter answers, so it counts clocks rather than
    // transactions -- around fifteen per word. What matters is that it is not
    // zero, which is what a build where DISK_IN_SDRAM failed to propagate
    // would show while every other check still passed.
    int disk_busy_cycles = 0;
    always @(posedge dut.clk_cpu) if (dut.disk_mrd) disk_busy_cycles++;

    // CYCLES TO REACH THE HLT. This testbench does a FIXED amount of disk
    // work -- the boot sector, then KERNEL.BIN through INT 13h -- and then
    // stops, so the cycle count is a work-normalised measure of the whole
    // read path in a way CPI is not. Changing how many instructions a sector
    // takes moves CPI's denominator, and a change that deletes a great many
    // cheap instructions raises CPI while making the machine faster. This
    // number cannot do that: the work is the same, so lower is faster.
    int boot_cycles = 0;
    logic was_halted = 1'b0;
    always @(posedge dut.clk_cpu) begin
        if (!dut.halted && !was_halted) boot_cycles++;
        if (dut.halted) was_halted <= 1'b1;
    end

    function automatic string read_line(input int row, input int len);
        string t = "";
        for (int c = 0; c < len; c++) t = {t, string'(cell_ch(row * 80 + c))};
        return t;
    endfunction

    task ps2_bit(input logic b);
        begin
            PS2_DAT = b;
            repeat (30) @(posedge CLOCK_50);
            PS2_CLK = 1'b0;
            repeat (30) @(posedge CLOCK_50);
            PS2_CLK = 1'b1;
            repeat (30) @(posedge CLOCK_50);
        end
    endtask

    task ps2_key(input [7:0] code);
        logic p;
        begin
            p = ~(^code);
            ps2_bit(1'b0);
            for (int k = 0; k < 8; k++) ps2_bit(code[k]);
            ps2_bit(p);
            ps2_bit(1'b1);
            repeat (60) @(posedge CLOCK_50);
        end
    endtask

    initial begin
        // ---- put the disk image in SDRAM, as the loader will ----
        $readmemh("rom/disk.hex", image);
        for (i = 0; i < DISK_SECTORS*256; i++)
            chip.mem[midx(DISK_BASE + i*2)] = image[i];

        repeat (10) @(negedge CLOCK_50);
        KEY[0] = 1'b0;
        repeat (100) @(negedge CLOCK_50);
        KEY[0] = 1'b1;

        i = 0;
        while (!dut.halted && i < 8000000) begin @(negedge CLOCK_50); i++; end
        chk("machine reached the keyboard wait", dut.halted, 1'b1);

        ps2_key(8'h33);   // h
        ps2_key(8'h24);   // e
        ps2_key(8'h4B);   // l
        ps2_key(8'h4B);   // l
        ps2_key(8'h44);   // o

        // Long enough for the kernel to finish -- echoing, the disk write and
        // read-back, and the graphics block after them. 400,000 was enough
        // when this only had to see the echoed text, and it silently was not
        // once there was anything to check afterwards: the kernel had not yet
        // reached the code whose results the checks below read, so they were
        // reading whatever was in memory beforehand. tb_bios waits the same
        // 5,000,000 for the same reason.
        i = 0;
        while (i < 5000000) begin @(negedge CLOCK_50); i++; end
        chk("machine reached the boot sector's HLT", dut.halted, 1'b1);
        $display("  boot reached HLT in %0d cycles (disk busy %0d)",
                 boot_cycles, disk_busy_cycles);

        $display("");
        // Row r holds what was printed as line r+1. The kernel ends by
        // forcing a scroll from the last row -- the check that the BIOS's
        // newline path actually scrolls rather than overwriting the bottom
        // line -- so the whole screen has moved up one and the banner has
        // gone off the top. want[6] ("hello") is echoed onto the last row
        // rather than line 6, for the same reason.
        for (r = 0; r <= 4; r++) begin
            got = read_line(r, want[r + 1].len());
            $display("  line %0d: \"%s\"", r, got);
            checks++;
            if (got != want[r + 1]) begin
                $display("FAIL line %0d mismatch", r);
                $display("    expected: \"%s\"", want[r + 1]);
                errors++;
            end
        end
        chk("the echo landed on the last row", read_line(24, 5) == want[6], 1'b1);
        $display("");

        chk("boot signature reached 0000:7DFE", chip.mem[midx('h7DFE)], 16'hAA55);

        // ---- INT 13h AH=03, the round trip ----
        // The disk was read-only until AH=03 existed, and the symptom was not
        // a missing feature -- DOS reported "General failure writing drive A"
        // for mkdir, copy, or saving anything. This is the case that has to
        // work: an SDRAM-backed disk, written through the BIOS, read back
        // through the BIOS, and checked in the memory the device actually
        // owns.
        // Read into a local first. Passing chip.mem[midx(...)][7:0] straight
        // into chk gave a different value from $display of the same
        // expression in the same statement block -- a part-select of an array
        // element indexed by a function call, handed to a task argument, is
        // apparently more than this simulator wants to evaluate. Not worth
        // chasing when a named local is clearer regardless.
        w604 = chip.mem[midx('h604)];
        chk("INT 13h AH=03 reported success", w604[15:8], 8'h00);

        // Read back at 0900:0000, a different segment from the 0800:0000 the
        // kernel wrote from, so a read that quietly handed back the caller's
        // own buffer cannot pass. Each word differs from the last, so a
        // rotated or duplicated sector cannot pass either.
        chk("written word 0 read back",   chip.mem[midx('h9000)],       16'h5AA5);
        chk("written word 1 read back",   chip.mem[midx('h9002)],       16'h5AA6);
        chk("written word 255 read back", chip.mem[midx('h9000 + 510)], 16'h5BA4);

        // ...and it is on the disk itself, not merely in a buffer.
        chk("word 0 reached the disk image",
            chip.mem[midx(DISK_BASE + 20*512)], 16'h5AA5);
        chk("word 255 reached the disk image",
            chip.mem[midx(DISK_BASE + 20*512 + 510)], 16'h5BA4);
        // The neighbouring sectors must be exactly as the image left them.
        chk("the sector before is untouched",
            chip.mem[midx(DISK_BASE + 19*512)], image[19*256]);
        chk("the sector after is untouched",
            chip.mem[midx(DISK_BASE + 21*512)], image[21*256]);

        $display("  the device spent %0d cycles reading SDRAM", disk_busy_cycles);
        checks++;
        if (disk_busy_cycles < 256) begin
            $display("FAIL the disk was not actually read out of SDRAM");
            errors++;
        end
        chk("no SDRAM protocol errors", chip.errors, 0);

        $display("");
        $display("==================================");
        $display(" checks: %0d   failures: %0d", checks, errors);
        $display("==================================");
        if (errors == 0) $display("ALL TESTS PASSED");
        $finish;
    end

    initial begin
        #200000000;
        $display("FAIL global timeout (IP=%04h halted=%b)", dut.dbg_ip, dut.halted);
        $display(" checks: %0d   failures: %0d", checks, errors + 1);
        $finish;
    end

endmodule
