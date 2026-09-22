`timescale 1ns/1ns
module tb_regfile;

    logic        clk = 0, rst_n = 0;
    logic [2:0]  rd0_sel, rd1_sel, wr_sel;
    logic        rd0_word, rd1_word, wr_word, wr_en;
    logic [15:0] rd0_data, rd1_data, wr_data;
    logic [1:0]  sreg_rd_sel, sreg_wr_sel;
    logic [15:0] sreg_rd_data, sreg_wr_data;
    logic        sreg_wr_en, seg_written;
    logic [15:0] cs, ip, ip_wdata;
    logic        ip_we;
    logic [15:0] flags, flags_wdata, flags_wmask;

    regfile dut (.*);

    always #5 clk = ~clk;

    int errors = 0, checks = 0;

    localparam [2:0] R_AX=0, R_CX=1, R_DX=2, R_BX=3, R_SP=4, R_BP=5, R_SI=6, R_DI=7;
    localparam [2:0] R_AL=0, R_CL=1, R_DL=2, R_BL=3, R_AH=4, R_CH=5, R_DH=6, R_BH=7;
    localparam [1:0] S_ES=0, S_CS=1, S_SS=2, S_DS=3;

    task chk(input string nm, input int got, input int exp);
        checks++;
        if (got !== exp) begin
            $display("FAIL %-36s got=%0h exp=%0h", nm, got, exp);
            errors++;
        end
    endtask

    task wr(input [2:0] sel, input w, input [15:0] d);
        begin
            @(negedge clk);
            wr_sel = sel; wr_word = w; wr_data = d; wr_en = 1;
            @(negedge clk);
            wr_en = 0;
        end
    endtask

    task rd0(input [2:0] sel, input w);
        begin rd0_sel = sel; rd0_word = w; #1; end
    endtask

    task rd1(input [2:0] sel, input w);
        begin rd1_sel = sel; rd1_word = w; #1; end
    endtask

    task wr_seg(input [1:0] sel, input [15:0] d);
        begin
            @(negedge clk);
            sreg_wr_sel = sel; sreg_wr_data = d; sreg_wr_en = 1;
            @(negedge clk);
            sreg_wr_en = 0;
        end
    endtask

    task wr_flags(input [15:0] data, input [15:0] mask);
        begin
            @(negedge clk);
            flags_wdata = data; flags_wmask = mask;
            @(negedge clk);
            flags_wmask = 16'h0000;
        end
    endtask

    initial begin
        rd0_sel=0; rd1_sel=0; wr_sel=0; rd0_word=1; rd1_word=1; wr_word=1;
        wr_en=0; wr_data=0; sreg_rd_sel=0; sreg_wr_sel=0; sreg_wr_data=0;
        sreg_wr_en=0; ip_we=0; ip_wdata=0; flags_wdata=0; flags_wmask=0;

        repeat (3) @(negedge clk);

        // ---------------- reset state ----------------
        chk("reset CS", cs, 16'hFFFF);
        chk("reset IP", ip, 16'h0000);
        sreg_rd_sel = S_CS; #1; chk("reset CS via port", sreg_rd_data, 16'hFFFF);
        sreg_rd_sel = S_DS; #1; chk("reset DS", sreg_rd_data, 16'h0000);
        sreg_rd_sel = S_ES; #1; chk("reset ES", sreg_rd_data, 16'h0000);
        sreg_rd_sel = S_SS; #1; chk("reset SS", sreg_rd_data, 16'h0000);
        rd0(R_AX, 1); chk("reset AX", rd0_data, 16'h0000);
        rd0(R_SP, 1); chk("reset SP", rd0_data, 16'h0000);
        chk("reset FLAGS (bit1 set)", flags, 16'h0002);

        rst_n = 1;
        @(negedge clk);

        // ---------------- word write/read, all 8 registers ----------------
        wr(R_AX, 1, 16'h1234);
        wr(R_CX, 1, 16'h5678);
        wr(R_DX, 1, 16'h9ABC);
        wr(R_BX, 1, 16'hDEF0);
        wr(R_SP, 1, 16'hFFFE);
        wr(R_BP, 1, 16'hB0B0);
        wr(R_SI, 1, 16'h5151);
        wr(R_DI, 1, 16'hD1D1);

        rd0(R_AX, 1); chk("AX word", rd0_data, 16'h1234);
        rd0(R_CX, 1); chk("CX word", rd0_data, 16'h5678);
        rd0(R_DX, 1); chk("DX word", rd0_data, 16'h9ABC);
        rd0(R_BX, 1); chk("BX word", rd0_data, 16'hDEF0);
        rd0(R_SP, 1); chk("SP word", rd0_data, 16'hFFFE);
        rd0(R_BP, 1); chk("BP word", rd0_data, 16'hB0B0);
        rd0(R_SI, 1); chk("SI word", rd0_data, 16'h5151);
        rd0(R_DI, 1); chk("DI word", rd0_data, 16'hD1D1);

        // ---------------- byte reads alias the word registers ----------------
        rd0(R_AL, 0); chk("AL = low(AX)",  rd0_data, 16'h0034);
        rd0(R_AH, 0); chk("AH = high(AX)", rd0_data, 16'h0012);
        rd0(R_CL, 0); chk("CL = low(CX)",  rd0_data, 16'h0078);
        rd0(R_CH, 0); chk("CH = high(CX)", rd0_data, 16'h0056);
        rd0(R_DL, 0); chk("DL = low(DX)",  rd0_data, 16'h00BC);
        rd0(R_DH, 0); chk("DH = high(DX)", rd0_data, 16'h009A);
        rd0(R_BL, 0); chk("BL = low(BX)",  rd0_data, 16'h00F0);
        rd0(R_BH, 0); chk("BH = high(BX)", rd0_data, 16'h00DE);

        // ---------------- byte write must not disturb the other half -------
        wr(R_AL, 0, 16'h00EE);
        rd0(R_AX, 1); chk("AX after AL write", rd0_data, 16'h12EE);
        wr(R_AH, 0, 16'h00AA);
        rd0(R_AX, 1); chk("AX after AH write", rd0_data, 16'hAAEE);
        rd0(R_AL, 0); chk("AL still intact",   rd0_data, 16'h00EE);

        wr(R_BH, 0, 16'h0011);
        rd0(R_BX, 1); chk("BX after BH write", rd0_data, 16'h11F0);

        // byte index 4..7 must hit AH/CH/DH/BH, NOT SP/BP/SI/DI
        rd0(R_SP, 1); chk("SP untouched by AH write", rd0_data, 16'hFFFE);
        rd0(R_BP, 1); chk("BP untouched by CH index", rd0_data, 16'hB0B0);

        // upper bits of a byte write are ignored
        wr(R_CL, 0, 16'hFF99);
        rd0(R_CX, 1); chk("CX after CL write (upper ignored)", rd0_data, 16'h5699);

        // ---------------- two read ports are independent ----------------
        rd0(R_AX, 1);
        rd1(R_DI, 1);
        chk("port0 AX", rd0_data, 16'hAAEE);
        chk("port1 DI", rd1_data, 16'hD1D1);
        rd0(R_AH, 0);
        rd1(R_AL, 0);
        chk("port0 AH byte", rd0_data, 16'h00AA);
        chk("port1 AL byte", rd1_data, 16'h00EE);

        // ---------------- segment registers ----------------
        wr_seg(S_DS, 16'h2000);
        sreg_rd_sel = S_DS; #1; chk("DS written", sreg_rd_data, 16'h2000);
        wr_seg(S_CS, 16'h8000);
        chk("CS dedicated output tracks write", cs, 16'h8000);
        sreg_rd_sel = S_ES; #1; chk("ES unaffected", sreg_rd_data, 16'h0000);

        // seg_written pulses during the write
        @(negedge clk);
        sreg_wr_sel = S_SS; sreg_wr_data = 16'h3000; sreg_wr_en = 1; #1;
        chk("seg_written asserted", seg_written, 1);
        @(negedge clk); sreg_wr_en = 0; #1;
        chk("seg_written deasserted", seg_written, 0);

        // ---------------- IP ----------------
        @(negedge clk);
        ip_wdata = 16'h7C00; ip_we = 1;
        @(negedge clk);
        ip_we = 0; #1;
        chk("IP written", ip, 16'h7C00);

        // ---------------- FLAGS ----------------
        // reserved bits are hardwired regardless of what is written
        wr_flags(16'hFFFF, 16'h0FD5);
        chk("FLAGS all set, reserved forced", flags, 16'h0FD7);
        //  0FD7 = OF DF IF TF SF ZF AF PF CF all 1, bit1 = 1, bits 3/5/12-15 = 0

        wr_flags(16'h0000, 16'h0FD5);
        chk("FLAGS all clear, bit1 still 1", flags, 16'h0002);

        // masked write: set only CF, leave the rest alone
        wr_flags(16'h0001, 16'h0001);
        chk("CF set via mask", flags, 16'h0003);

        // set ZF while CF stays set
        wr_flags(16'h0040, 16'h0040);
        chk("ZF set, CF preserved", flags, 16'h0043);

        // a write with data=0 but mask=0 must change nothing
        wr_flags(16'h0000, 16'h0000);
        chk("empty mask changes nothing", flags, 16'h0043);

        // clearing CF only
        wr_flags(16'h0000, 16'h0001);
        chk("CF cleared, ZF preserved", flags, 16'h0042);

        // IF/TF cleared on interrupt entry, other flags untouched
        wr_flags(16'h0FFF, 16'h0300);      // set IF and TF
        chk("IF+TF set", flags, 16'h0342);
        wr_flags(16'h0000, 16'h0300);      // interrupt entry clears them
        chk("IF+TF cleared, ZF kept", flags, 16'h0042);

        // DF only (CLD/STD)
        wr_flags(16'h0400, 16'h0400);
        chk("DF set", flags, 16'h0442);
        wr_flags(16'h0000, 16'h0400);
        chk("DF cleared", flags, 16'h0042);

        // the six ALU flags as a group
        wr_flags(16'h08D5, 16'h08D5);
        chk("all ALU flags set", flags, 16'h08D7);

        $display("");
        $display("==================================");
        $display(" checks: %0d   failures: %0d", checks, errors);
        $display("==================================");
        if (errors == 0) $display("ALL TESTS PASSED");
        $finish;
    end

endmodule
