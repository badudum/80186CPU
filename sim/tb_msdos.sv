`timescale 1ns/1ns
//
// Boot the real MS-DOS image in simulation, with a trace.
//
// This exists because the machine gets as far as "Starting MS-DOS..." on
// hardware and then executes an invalid opcode inside the operating system,
// where nothing can be asked of it afterwards. The BIOS's fault record says
// WHAT the state was; only this says how it got there.
//
// It is not a pass/fail test and is deliberately not part of the regression --
// it needs a proprietary disk image that cannot live in this repository, and
// it runs for minutes rather than seconds. Build the image it wants with:
//
//   python3 -c "d=open('ms-dos/disk01.img','rb').read();
//              open('ms-dos/msdos.hex','w').write(
//                  ''.join('%04x\n' % (d[i]|(d[i+1]<<8))
//                          for i in range(0,len(d),2)))"
//   ./sim/run.sh tb_msdos
//
// WHAT IT RECORDS.
//   * every character the machine prints, as it prints it, so the transcript
//     shows the boot progressing rather than only the final screen;
//   * a rolling window of the last TRACE_DEPTH retired instructions, dumped
//     when the fault hits -- the path that led there;
//   * every load of a segment register with a value of interest, because the
//     fault on hardware is a relocation copy reading from A000, above the top
//     of memory, and the question is which instruction put A000 there.
//
module tb_msdos;

    localparam int DISK_SECTORS = 2880;                 // a 1.44 MB floppy
    localparam int DISK_BASE    = 24'h100000;
    localparam int TRACE_DEPTH  = 512;
    localparam int MAX_CYCLES   = 25000000;
    // No character printed for this long means it is not going to.
    localparam int STUCK_CYCLES = 18000000;

    // Segment register encodings, from cpu_pkg.
    localparam logic [1:0] S_ES = 2'd0, S_CS = 2'd1, S_SS = 2'd2, S_DS = 2'd3;
    localparam logic [5:0] ST_RETIRE = 6'd18;

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
    // A BIOS whose timer ticks 64x too fast, built by
    //   python3 tools/gen_bios.py rom/fast --fast-tick
    // MS-DOS times its startup waits in ticks, and at the real 18.2 Hz the
    // F5/F8 prompt alone is fifty million clocks of doing nothing. This is the
    // only difference from the ROM that goes on the board.
    // Cache size is deliberately NOT overridden here, so this measures the
    // cache the board is actually built with. Measured on this workload,
    // against the same MS-DOS boot:
    //
    //     cache    stalled on SDRAM    memory accesses completed per cycle
    //     none          44.4%                     1.00x
    //     8 KB          26.3%                     1.28x
    //     32 KB         23.3%                     1.33x
    //
    // Four times the cache for another four percent: the misses that remain
    // are not capacity misses, so the next real gain is associativity, not
    // size. Put a `defparam dut.CACHE_KB = ...` here to re-measure.
    defparam dut.u_mem.u_rom.INIT_LO = "rom/fast/bios.lo.hex";
    defparam dut.u_mem.u_rom.INIT_HI = "rom/fast/bios.hi.hex";

    sdram_model #(.CAS_LATENCY(2)) chip (
        .dram_clk (DRAM_CLK), .dram_cke (DRAM_CKE), .dram_cs_n (DRAM_CS_N),
        .dram_ras_n (DRAM_RAS_N), .dram_cas_n (DRAM_CAS_N), .dram_we_n (DRAM_WE_N),
        .dram_addr (DRAM_ADDR), .dram_ba (DRAM_BA),
        .dram_dqm ({DRAM_UDQM, DRAM_LDQM}), .dram_dq (DRAM_DQ)
    );

    always #10 CLOCK_50 = ~CLOCK_50;

    function automatic int unsigned midx(input int unsigned byte_addr);
        midx = (byte_addr >> 1) & 21'h1FFFFF;
    endfunction

    // ---- shorthands into the CPU ----
    `define EXEC dut.u_cpu.u_eu.u_exec
    `define REGS dut.u_cpu.u_eu.u_regfile

    wire [15:0] cs_now  = `REGS.sreg[S_CS];
    wire [15:0] ds_now  = `REGS.sreg[S_DS];
    wire [15:0] es_now  = `REGS.sreg[S_ES];
    wire [15:0] ip_now  = `EXEC.instr_start_ip;

    // Same stall accounting as tb_bios, but against real MS-DOS code running
    // from SDRAM rather than a BIOS running from on-chip ROM -- which is the
    // workload the question "would a cache help" is really about.
    int stall_ram = 0, stall_other = 0, bus_cycles = 0;
    always @(posedge dut.clk_cpu) begin
        if (dut.rd || dut.wr) begin
            bus_cycles++;
            if (!dut.ready) begin
                if (dut.u_mem.in_ram) stall_ram++;
                else                  stall_other++;
            end
        end
    end

    int cycles = 0;
    always @(posedge dut.clk_cpu) cycles++;

    // ---- cycles per instruction ----
    // The number that decides whether fetch-side work (a branch predictor, a
    // prefetcher) can matter at all: if the microcode sequencer is spending
    // tens of cycles per instruction internally, shaving cycles off fetch
    // cannot move much. Halted cycles are excluded because parking in HLT is
    // not work, and tb_bios is NOT a fair place to measure this -- its
    // clear_gfx is a single REP STOSW covering 32,000 words, which retires
    // once and runs for hundreds of thousands of cycles.
    int retired = 0, busy_cycles = 0;
    logic [5:0] prev_ret = 6'd0;

    // ---- where the sequencer's cycles go ----
    // CPI says the execution unit is the bottleneck; this says which part of
    // it. Cycles are attributed to whatever state the sequencer is sitting
    // in, and separately to whether the bus was making it wait -- a state
    // that is slow because memory is slow needs a different fix from one
    // that is slow because it exists at all.
    int st_cyc  [0:63];
    int st_wait [0:63];
    initial for (int k = 0; k < 64; k++) begin st_cyc[k] = 0; st_wait[k] = 0; end

    always @(posedge dut.clk_cpu) begin
        if (!dut.halted) begin
            busy_cycles++;
            st_cyc[`EXEC.state]++;
            if ((dut.rd || dut.wr) && !dut.ready) st_wait[`EXEC.state]++;
        end
        if (`EXEC.state == ST_RETIRE && prev_ret != ST_RETIRE) retired++;
        prev_ret <= `EXEC.state;
    end

    // ---- shadow check: does decode_len agree with the sequencer? ----
    // Hand-worked encodings in tb_decode_len prove the rules; this proves
    // them against the real instruction stream, which is the only thing that
    // covers the encodings DOS actually uses. A wrong length does not corrupt
    // one instruction, it moves the queue head to the wrong byte and every
    // instruction after it decodes from garbage -- so this has to agree a
    // million times, not most of the time.
    //
    // The sequencer's own answer is how far IP moved: instr_start_ip marks
    // the first PREFIX byte of the instruction, and ip_next is where the next
    // one begins. Jumps are excluded because IP then comes from the target,
    // not from the length.
    logic [7:0] dl_peek [0:5];
    logic [3:0] dl_count;
    logic       dl_valid;
    logic [2:0] dl_len, dl_npfx;
    logic       dl_rep, dl_seg;
    logic [1:0] dl_segovr;

    always_comb begin
        for (int i = 0; i < 6; i++) dl_peek[i] = dut.u_cpu.u_biu.u_pq.peek[i];
        dl_count = dut.u_cpu.u_biu.u_pq.count;
    end

    decode_len u_dl (
        .peek (dl_peek), .count (dl_count), .valid (dl_valid),
        .len (dl_len), .n_prefix (dl_npfx),
        .has_rep (dl_rep), .has_seg_ovr (dl_seg), .seg_ovr (dl_segovr)
    );

    localparam logic [5:0] ST_FETCH_OP = 6'd1;
    int dl_checked = 0, dl_mismatch = 0, dl_unknown = 0;
    logic       dl_armed = 1'b0;
    logic [2:0] dl_pred;
    int         dl_bytes;
    logic [5:0] dl_prev = 6'd0;

    // SAMPLED ON THE NEGEDGE, and counting bytes rather than inferring them
    // from IP. Two reasons, both of which produced a false 13% mismatch rate
    // on the first attempt:
    //
    //   Reading `state` on the posedge races its own non-blocking update, so
    //   the state was a cycle stale while the combinational queue read was
    //   not. Everything has settled by the negedge.
    //
    //   IP movement is a proxy for length, not length itself. Counting what
    //   the queue actually retired measures exactly what decode_len claims.
    // The #1 is not cosmetic. dl_peek is driven by an always_comb and
    // decode_len's outputs are another delta behind it, so reading them from
    // a different always block at the same instant samples stale values --
    // which showed up as a one-byte instruction being predicted as three.
    always @(negedge dut.clk_cpu) begin
        #1;
        if (`EXEC.state == ST_FETCH_OP && dl_prev != ST_FETCH_OP
            && !`EXEC.prefix_seen) begin
            dl_bytes = 0;
            if (dl_valid) begin
                dl_armed = 1'b1;
                dl_pred  = dl_len;
            end else begin
                dl_armed = 1'b0;          // length not yet determinable
                dl_unknown++;
            end
        end

        // do_pop is what the queue will retire on the coming edge.
        if (dl_armed) dl_bytes = dl_bytes + dut.u_cpu.u_biu.u_pq.do_pop;

        if (`EXEC.state == ST_RETIRE && dl_prev != ST_RETIRE) begin
            if (dl_armed) begin
                dl_checked++;
                if (dl_bytes != {29'd0, dl_pred}) begin
                    if (dl_mismatch < 10)
                        $display("  DECODE_LEN MISMATCH: predicted %0d, queue retired %0d",
                                 dl_pred, dl_bytes);
                    dl_mismatch++;
                end
            end
            dl_armed = 1'b0;
        end
        dl_prev = `EXEC.state;
    end

    string st_name [0:63];
    initial begin
        for (int k = 0; k < 64; k++) st_name[k] = "?";
        st_name[0]="START";     st_name[1]="FETCH_OP";  st_name[2]="MODRM";
        st_name[3]="DISP";      st_name[4]="IMM";       st_name[5]="EA";
        st_name[6]="PREP";      st_name[7]="LOAD2";     st_name[8]="LOAD";
        st_name[9]="EXEC";      st_name[10]="LOOP_DEC"; st_name[11]="ALU_WAIT";
        st_name[12]="STORE";    st_name[13]="PUSH";     st_name[14]="POP";
        st_name[15]="SP_UPD";   st_name[16]="WB";       st_name[17]="WB_HI";
        st_name[18]="RETIRE";   st_name[19]="REDIRECT"; st_name[20]="HALT";
        st_name[21]="INT_PREP"; st_name[22]="INT_SETUP";st_name[23]="INT_WR";
        st_name[24]="INT_SP";   st_name[25]="INT_RDLO"; st_name[26]="INT_RDHI";
        st_name[27]="INT_APPLY";st_name[28]="IRET_PREP";st_name[29]="IRET_RD";
        st_name[30]="IRET_APPL";st_name[31]="INT_GAP";  st_name[32]="IRET_GAP";
        st_name[33]="IO_RD";    st_name[34]="IO_WR";    st_name[35]="FAR_APPLY";
        st_name[36]="SREG_WB";  st_name[37]="STR_PREP1";st_name[38]="STR_PREP2";
        st_name[39]="STR_CHECK";st_name[40]="STR_RD1";  st_name[41]="STR_G1";
        st_name[42]="STR_RD2";  st_name[43]="STR_G2";   st_name[44]="STR_WR";
    end

    task dump_states;
        int tot;
        begin
            tot = 0;
            for (int k = 0; k < 64; k++) tot += st_cyc[k];
            $display("");
            $display("  sequencer cycles by state (%0d total, CPI %0.2f):",
                     tot, real'(tot) / real'(retired));
            $display("    %-10s %10s %7s %10s", "state", "cycles", "share", "bus-wait");
            for (int k = 0; k < 64; k++)
                if (st_cyc[k] * 200 > tot)          // anything over 0.5%
                    $display("    %-10s %10d %6.1f%% %10d",
                             st_name[k], st_cyc[k],
                             100.0 * st_cyc[k] / tot, st_wait[k]);
        end
    endtask

    // ---- console echo ----
    // Every byte the machine writes into the text buffer, in order. Watching
    // it arrive is the difference between "it is still running" and "it hung
    // twenty million cycles ago".
    // cpu_we is held for the whole bus cycle, not pulsed, so only its rising
    // edge is one character -- otherwise every letter appears twice.
    string line = "";
    int    last_print_cycle = 0;
    logic  prev_vwe = 1'b0;
    always @(posedge dut.clk_cpu) begin
        if (dut.u_mem.u_vram.cpu_we && !prev_vwe &&
            dut.u_mem.u_vram.cpu_be[0]) begin
            automatic logic [7:0] ch = dut.u_mem.u_vram.cpu_wdata[7:0];
            // Printing spaces too would bury the log under the screen clear.
            last_print_cycle = cycles;
            if (ch > 8'h20 && ch < 8'h7F) line = {line, string'(ch)};
            else if (ch == 8'h20 && line.len() > 0 &&
                     line.substr(line.len()-1, line.len()-1) != " ")
                line = {line, " "};
        end
        prev_vwe <= dut.u_mem.u_vram.cpu_we;
    end

    // ---- instruction trace ----
    logic [15:0] tr_cs [0:TRACE_DEPTH-1];
    logic [15:0] tr_ip [0:TRACE_DEPTH-1];
    int          tr_cy [0:TRACE_DEPTH-1];
    int          tr_wr = 0, tr_n = 0;

    logic [5:0] prev_state = 6'd0;
    always @(posedge dut.clk_cpu) begin
        if (`EXEC.state == ST_RETIRE && prev_state != ST_RETIRE) begin
            tr_cs[tr_wr] = cs_now;
            tr_ip[tr_wr] = ip_now;
            tr_cy[tr_wr] = cycles;
            tr_wr = (tr_wr + 1) % TRACE_DEPTH;
            if (tr_n < TRACE_DEPTH) tr_n++;
        end
        prev_state <= `EXEC.state;
    end

    task automatic dump_trace(input int n);
        int idx;
        begin
            $display("");
            $display("  last %0d retired instructions (oldest first):", n);
            for (int k = n; k > 0; k--) begin
                idx = (tr_wr - k + TRACE_DEPTH) % TRACE_DEPTH;
                $display("    %8d  %04h:%04h", tr_cy[idx], tr_cs[idx], tr_ip[idx]);
            end
        end
    endtask

    // Dump a window of physical memory, so guest code that has relocated
    // itself can be pulled out and disassembled offline. Conventional RAM is
    // in SDRAM here, so the model's array is the machine's memory.
    task automatic dump_mem(input int unsigned base, input int nbytes);
        int unsigned a;
        string row;
        begin
            $display("  memory %06h..%06h:", base, base + nbytes - 1);
            for (a = base; a < base + nbytes; a += 16) begin
                row = "";
                for (int k = 0; k < 16; k += 2) begin
                    automatic logic [15:0] w = chip.mem[midx(a + k)];
                    row = {row, $sformatf("%02h %02h ", w[7:0], w[15:8])};
                end
                $display("    %06h  %s", a, row);
            end
        end
    endtask

    // Write a physical memory range out as hex, one byte per line, so it can
    // be diffed against the file it was loaded from. Every difference is a
    // fixup somebody applied -- which is a far quicker way to find the one
    // that is wrong than following a value through three relocations.
    task automatic save_mem(input string path, input int unsigned base,
                            input int nbytes);
        int fd;
        begin
            fd = $fopen(path, "w");
            if (fd == 0) begin
                $display("  could not open %s", path);
                return;
            end
            for (int k = 0; k < nbytes; k++) begin
                automatic logic [15:0] w = chip.mem[midx(base + k)];
                $fwrite(fd, "%02x\n", ((base + k) & 1) ? w[15:8] : w[7:0]);
            end
            $fclose(fd);
            $display("  saved %0d bytes from %06h to %s", nbytes, base, path);
        end
    endtask

    task automatic show_regs;
        begin
            $display("    AX %04h BX %04h CX %04h DX %04h  SI %04h DI %04h BP %04h SP %04h",
                     `REGS.gpr[0], `REGS.gpr[3], `REGS.gpr[1], `REGS.gpr[2],
                     `REGS.gpr[6], `REGS.gpr[7], `REGS.gpr[5], `REGS.gpr[4]);
            $display("    CS %04h DS %04h ES %04h SS %04h",
                     `REGS.sreg[S_CS], `REGS.sreg[S_DS],
                     `REGS.sreg[S_ES], `REGS.sreg[S_SS]);
        end
    endtask

    // ---- segment loads worth knowing about ----
    // A000 is the top of conventional memory. Nothing should ever read through
    // a data segment pointing there: it is the video aperture, not RAM.
    int seg_hits = 0;
    always @(posedge dut.clk_cpu) begin
        if (`EXEC.sreg_wr_en && `EXEC.sreg_wr_data == 16'hA000 &&
            `EXEC.sreg_wr_sel != S_CS) begin
            seg_hits++;
            if (seg_hits <= 40) begin
                $display("  [%8d] %s <= A000  by %04h:%04h",
                         cycles,
                         (`EXEC.sreg_wr_sel == S_DS) ? "DS" :
                         (`EXEC.sreg_wr_sel == S_ES) ? "ES" : "SS",
                         cs_now, ip_now);
                // Our own fault handler reads the saved DS back, which is not
                // interesting; guest code doing it is the whole question.
                if (cs_now != 16'hF000) begin
                    show_regs();
                    dump_trace(40);
                    dump_mem(int'(cs_now) * 16 + int'(ip_now) - 96, 192);
                    // The whole relocated block, for an offline diff against
                    // the copy of it that is still in IO.SYS on the disk.
                    save_mem("block.hex", int'(cs_now) * 16, 'h1A70);
                end
            end
        end
    end

    // ---- who writes A000 into memory ----
    // The relocation the machine dies in takes its SOURCE segment out of a
    // variable, and that variable holds A000 -- the video aperture, not RAM.
    // Nothing loads a segment register with A000 before then, so the value was
    // stored rather than computed in place: this catches the store.
    int a000_writes = 0;
    logic prev_wr = 1'b0;
    always @(posedge dut.clk_cpu) begin
        if (dut.wr && !prev_wr && !dut.io_cycle &&
            dut.cpu_dout == 16'hA000 && cs_now != 16'hF000) begin
            a000_writes++;
            if (a000_writes <= 20) begin
                $display("  [%8d] wrote A000 to %05h  by %04h:%04h",
                         cycles, dut.addr, cs_now, ip_now);
                show_regs();
                dump_trace(24);
            end
        end
        prev_wr <= dut.wr;
    end

    // ---- the relocation loop, step by step ----
    // The code that dies walks itself up through memory: copy a block, patch
    // the far pointer that points at the copy, call into it, repeat. It runs
    // at a different CS each pass -- that is the point of it -- so the trigger
    // is the OFFSET, which stays put.
    //
    //   4E5  rep movsw, the 40 KB block move
    //   502  xchg ax, cs:[289] -- swaps in the new call target, hands back the
    //        old one, which becomes the source segment
    //   515  rep movsw, the block that is then called into
    //   519  lcall cs:[287]
    int step_hits = 0;
    always @(posedge dut.clk_cpu) begin
        if (`EXEC.state == ST_RETIRE && prev_state != ST_RETIRE &&
            cs_now != 16'hF000 && step_hits < 200 &&
            (ip_now == 16'h04E5 || ip_now == 16'h0502 ||
             ip_now == 16'h0507 || ip_now == 16'h0515 ||
             ip_now == 16'h0519)) begin
            step_hits++;
            $display("  [%8d] %04h:%04h  AX %04h CX %04h  DS %04h ES %04h SI %04h DI %04h",
                     cycles, cs_now, ip_now, `REGS.gpr[0], `REGS.gpr[1],
                     ds_now, es_now, `REGS.gpr[6], `REGS.gpr[7]);
        end
    end

    // ---- the far pointer, at every generation of the block ----
    // The block relocates itself repeatedly and carries a far pointer at
    // +0287 that is meant to keep naming the current copy. By the time it is
    // used its segment half is A000, which is not memory. Printing it at each
    // relocation says which generation lost it.
    function automatic logic [15:0] peek16(input int unsigned a);
        peek16 = {chip.mem[midx(a+1)][15:8] & 8'hFF, 8'h00} |
                 {8'h00, (a & 1) ? chip.mem[midx(a)][15:8]
                                 : chip.mem[midx(a)][7:0]};
    endfunction

    int gen = 0;
    always @(posedge dut.clk_cpu) begin
        if (`EXEC.state == ST_RETIRE && prev_state != ST_RETIRE &&
            cs_now != 16'hF000 && gen < 40 &&
            (ip_now == 16'h04B0 || ip_now == 16'h04BF || ip_now == 16'h0502)) begin
            gen++;
            $display("  [%8d] gen at %04h:%04h  farptr %04h:%04h  (bytes %02h %02h %02h %02h)",
                     cycles, cs_now, ip_now,
                     peek16(int'(cs_now)*16 + 'h289),
                     peek16(int'(cs_now)*16 + 'h287),
                     chip.mem[midx(int'(cs_now)*16 + 'h287)][7:0],
                     chip.mem[midx(int'(cs_now)*16 + 'h287)][15:8],
                     chip.mem[midx(int'(cs_now)*16 + 'h289)][7:0],
                     chip.mem[midx(int'(cs_now)*16 + 'h289)][15:8]);
        end
    end

    // How often the timer interrupt has actually been taken.
    int tick_ints = 0;
    // ...and every other vector, because a machine that is busy but silent is
    // usually busy servicing one interrupt over and over. A keyboard IRQ that
    // never deasserts looks exactly like this from the outside.
    // dbg_int_taken is a LEVEL held for the whole vectoring sequence, not a
    // one-cycle pulse, so counting it directly multiplies every total by
    // however many clocks that takes -- about twelve. Counting the rising edge
    // is the difference between "MS-DOS issued 15,000 disk reads, something is
    // badly wrong" and "MS-DOS issued 1,200, which is what loading its kernel
    // costs".
    int int_count [0:255];
    logic prev_int_taken = 1'b0;
    initial for (int k = 0; k < 256; k++) int_count[k] = 0;
    always @(posedge dut.clk_cpu) begin
        if (dut.dbg_int_taken && !prev_int_taken) begin
            int_count[dut.dbg_int_type]++;
            if (dut.dbg_int_type == 8'd8) tick_ints++;
        end
        prev_int_taken <= dut.dbg_int_taken;
    end

    // Which FUNCTION of the two services that dominate. A count alone says
    // the machine is busy; the function says what it is busy doing, and
    // "reading the same sectors over and over" looks nothing like "loading a
    // program" once the AH values are separated out.
    int i13_ah [0:255];
    int i10_ah [0:255];
    int vram_writes = 0;
    int other_video_writes = 0;
    initial for (int k = 0; k < 256; k++) begin i13_ah[k] = 0; i10_ah[k] = 0; end

    always @(posedge dut.clk_cpu) begin
        if (dut.dbg_int_taken && !prev_int_taken) begin
            if (dut.dbg_int_type == 8'h13) i13_ah[`REGS.gpr[0][15:8]]++;
            if (dut.dbg_int_type == 8'h10) i10_ah[`REGS.gpr[0][15:8]]++;
        end
        if (dut.u_mem.u_vram.cpu_we && !prev_vwe) vram_writes++;
        // Anything writing into the video aperture that is NOT the text
        // buffer -- a guest that decided it had a monochrome adapter would
        // land at B0000 and vanish.
        if (dut.wr && !prev_wr && !dut.io_cycle &&
            dut.addr >= 20'hA0000 && dut.addr < 20'hB8000)
            other_video_writes++;
    end

    // How often our own INT 10h handler actually runs. The interrupt count
    // says the vector was taken; this says where it went. A guest whose
    // output disappears has either had the vector moved out from under it or
    // is writing somewhere nobody is looking.
    int int10_entries = 0;
    always @(posedge dut.clk_cpu)
        if (`EXEC.state == ST_RETIRE && prev_state != ST_RETIRE &&
            cs_now == 16'hF000 && ip_now == 16'h025D) int10_entries++;

    // Where an INT 10h actually LANDS. The vector says one thing and the
    // handler entry count says another, so the only way to settle it is to
    // watch the instructions that run immediately after the interrupt is
    // taken, and to read the vector out of memory at that same moment.
    int  land_shown = 0;
    int  land_left  = 0;
    always @(posedge dut.clk_cpu) begin
        if (dut.dbg_int_taken && !prev_int_taken &&
            dut.dbg_int_type == 8'h10 && land_shown < 6) begin
            land_shown++;
            land_left = 4;
            $display("  [%8d] INT 10h taken, AH=%02h, from %04h:%04h; vector in memory = %04h:%04h",
                     cycles, `REGS.gpr[0][15:8], cs_now, ip_now,
                     chip.mem[midx('h42)], chip.mem[midx('h40)]);
        end else if (land_left > 0 && `EXEC.state == ST_RETIRE &&
                     prev_state != ST_RETIRE) begin
            land_left--;
            $display("               landed at %04h:%04h", cs_now, ip_now);
        end
    end

    task automatic dump_ivt;
        begin
            $display("  BIOS int10_video entered %0d times", int10_entries);
            $display("  interrupt vectors now:");
            for (int k = 8; k < 16'h30; k++)
                $display("    INT %02h -> %04h:%04h", k,
                         chip.mem[midx(k*4 + 2)], chip.mem[midx(k*4)]);
        end
    endtask

    task automatic dump_ah;
        begin
            $display("  VRAM writes %0d, writes to A0000-B7FFF %0d",
                     vram_writes, other_video_writes);
            $display("  INT 13h by function:");
            for (int k = 0; k < 256; k++)
                if (i13_ah[k] != 0) $display("    AH=%02h  %0d", k, i13_ah[k]);
            $display("  INT 10h by function:");
            for (int k = 0; k < 256; k++)
                if (i10_ah[k] != 0) $display("    AH=%02h  %0d", k, i10_ah[k]);
        end
    endtask

    task automatic dump_ints;
        begin
            $display("  interrupts taken:");
            for (int k = 0; k < 256; k++)
                if (int_count[k] != 0)
                    $display("    INT %02h  %0d", k, int_count[k]);
        end
    endtask

    // ---- the fault ----
    int fault_cycle = 0;
    always @(posedge dut.clk_cpu) begin
        if (`EXEC.state != 6'd0 && `EXEC.int_type_r == 8'd6 &&
            `EXEC.state == 6'd19 && fault_cycle == 0) begin
            fault_cycle = cycles;
        end
    end

    logic [15:0] image [0:DISK_SECTORS*256-1];
    int i;

    initial begin
        $display("loading ms-dos/msdos.hex ...");
        $readmemh("ms-dos/msdos.hex", image);
        for (i = 0; i < DISK_SECTORS*256; i++)
            chip.mem[midx(DISK_BASE + i*2)] = image[i];
        $display("  %0d sectors placed at %06h", DISK_SECTORS, DISK_BASE);

        repeat (10) @(negedge CLOCK_50);
        KEY[0] = 1'b0;
        repeat (100) @(negedge CLOCK_50);
        KEY[0] = 1'b1;

        fork
            begin : progress
                forever begin
                    repeat (4000000) @(negedge dut.clk_cpu);
                    // The BIOS tick counter at 0040:006C matters more than it
                    // looks: MS-DOS times its F5/F8 startup wait off it, and a
                    // tick that never advances is a wait that never ends.
                    $display("  [%8d] %04h:%04h  ticks %04h  int08 %0d  screen so far: \"%s\"",
                             cycles, cs_now, ip_now,
                             chip.mem[midx('h46C)], tick_ints,
                             line.len() > 90 ? line.substr(line.len()-90,
                                                           line.len()-1) : line);
                end
            end
            begin : run
                forever begin
                    repeat (100000) @(negedge dut.clk_cpu);
                    if (dut.halted) break;
                    if (cycles > MAX_CYCLES) break;
                    if (cycles > 4000000 &&
                        (cycles - last_print_cycle) > STUCK_CYCLES) begin
                        $display("");
                        $display("  nothing printed for %0d cycles -- stuck",
                                 STUCK_CYCLES);
                        break;
                    end
                end
            end
        join_any
        disable fork;

        $display("");
        $display("==================================");
        $display(" stopped at cycle %0d, halted=%0b", cycles, dut.halted);
        $display(" CS:IP  %04h:%04h", cs_now, ip_now);
        $display(" DS %04h  SS %04h", ds_now, `REGS.sreg[S_SS]);
        $display(" screen: \"%s\"", line);
        $display(" segment loads of A000 seen: %0d", seg_hits);
        $display(" %0d cycles, %0d with a bus request (%0.1f%%)",
                 cycles, bus_cycles, 100.0 * bus_cycles / cycles);
        $display(" stalled on SDRAM:     %0d (%0.1f%%)",
                 stall_ram, 100.0 * stall_ram / cycles);
        $display(" stalled on ROM/other: %0d (%0.1f%%)",
                 stall_other, 100.0 * stall_other / cycles);
        $display(" %0d instructions in %0d running cycles -- CPI %0.2f",
                 retired, busy_cycles, real'(busy_cycles) / real'(retired));
        dump_states();
        $display("");
        $display(" decode_len shadow: %0d checked, %0d mismatched, %0d not yet knowable",
                 dl_checked, dl_mismatch, dl_unknown);
        show_regs();
        // The loop it is sitting in, for an offline disassembly.
        dump_mem(int'(cs_now) * 16 + int'(ip_now) - 128, 256);
        dump_ints();
        dump_ah();
        dump_ivt();
        // The loop it is actually sitting in. Everything above says what the
        // machine is doing in aggregate; this is the code, so it can be pulled
        // out and disassembled.
        show_regs();
        dump_mem(int'(cs_now) * 16 + int'(ip_now) - 128, 256);
        dump_trace(tr_n < 120 ? tr_n : 120);
        $display("==================================");
        $finish;
    end

endmodule
