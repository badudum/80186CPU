// ---------------------------------------------------------------------------
// storage — block storage device, 512-byte sectors, PIO data port.
//
// Hierarchy: FPGA80186 -> io_decode -> storage
// Testbench: sim/tb_storage.sv
//
// TWO BACKENDS, chosen by USE_SDRAM.
//
//   0  on-chip ROM built into the bitstream. Survives power-up for free and
//      needs no memory controller, but is capped by block RAM at roughly 700
//      sectors -- so a 1.44 MB floppy image cannot be held this way. Read-only,
//      because it is ROM.
//
//   1  a window of the board's SDRAM, starting at BASE. Any size, and
//      WRITABLE, which is what lets INT 13h have a write function at all. The
//      cost is that SDRAM is volatile: something has to put an image there
//      after every power-up.
//
// The register interface is identical either way, so the BIOS, the FAT12 boot
// sector and everything above them cannot tell which is in use.
//
// WHY THE DISK LIVES ABOVE 1 MB. With the SDRAM backend the image sits at
// BASE = 100000h and up. An 8086 address cannot reach there, so conventional
// memory and the disk cannot collide however wrong the software gets -- the
// CPU literally cannot address the disk, only this device can.
//
// A SECTOR BUFFER, NOT PER-ACCESS READS. A command copies the whole sector
// into a local buffer and the data port then serves from it. Reading SDRAM on
// each DATA access would mean ten-odd cycles of latency inside a bus cycle
// io_decode expects to answer immediately, so it would need wait states
// threaded all the way back to the BIU. Buffering also makes BUSY *real*: it
// now covers an actual transfer rather than a counter that existed to stop
// software being written against an unrealistically instant device.
//
// REGISTERS (I/O space, the PC/XT hard-disk range so nothing collides):
//   0320  r/w DATA      next 16-bit word of the sector; auto-increments
//   0322  w   LBA_LO    sector number, bits 15:0
//   0324  w   LBA_HI    sector number, bits 23:16
//   0326  w   CMD       1 = READ SECTOR, 2 = WRITE SECTOR
//         r   STATUS    bit0 BUSY, bit1 DRQ, bit2 ERR
//   0328  r   SECTORS   size of the device, in sectors
//
// USAGE: write LBA, write CMD, poll STATUS until BUSY clears, then read (or
// write) 256 words at DATA. For a write, fill the buffer through DATA first
// and then issue CMD=2.
// ---------------------------------------------------------------------------

`timescale 1ns/1ns
module storage #(
    parameter int  SECTORS   = 256,
    parameter bit  USE_SDRAM = 1'b0,
    // Only meaningful for the ROM backend.
    parameter      INIT_FILE = "rom/disk.hex",
    // Only meaningful for the SDRAM backend: byte address of sector 0.
    parameter logic [23:0] BASE = 24'h100000
) (
    input  logic        clk,
    input  logic        rst_n,

    // register interface (address decode done by io_decode).
    // `rd`/`wr` are SINGLE-CYCLE strobes marking a completed access -- see the
    // contract note in io_decode.sv. `sel` and `reg_sel` are levels.
    input  logic        sel,
    input  logic [2:0]  reg_sel,       // (port address - 0320h) >> 1
    input  logic        rd,
    input  logic        wr,
    input  logic [15:0] wdata,
    output logic [15:0] rdata,

    // SDRAM requester port (idle when USE_SDRAM = 0). Holds its request until
    // it sees `ready`, then drops it, as sdram_arbiter requires.
    output logic [23:0] mem_addr,
    output logic [15:0] mem_wdata,
    output logic [1:0]  mem_be,
    output logic        mem_rd,
    output logic        mem_wr,
    input  logic [15:0] mem_rdata,
    input  logic        mem_ready
);

    localparam logic [2:0] R_DATA    = 3'd0;   // 0320
    localparam logic [2:0] R_LBA_LO  = 3'd1;   // 0322
    localparam logic [2:0] R_LBA_HI  = 3'd2;   // 0324
    localparam logic [2:0] R_CMD     = 3'd3;   // 0326
    localparam logic [2:0] R_SECTORS = 3'd4;   // 0328

    localparam logic [15:0] CMD_READ  = 16'h0001;
    localparam logic [15:0] CMD_WRITE = 16'h0002;

    // Sized explicitly. Comparing a 24-bit unsigned against a bare `int`
    // parameter drags the comparison into signed 32-bit arithmetic, which is
    // the sort of thing that works until the sector count crosses a bit
    // boundary.
    localparam logic [23:0] SECTOR_LIMIT = SECTORS[23:0];
    localparam logic [15:0] SECTOR_COUNT = SECTORS[15:0];

    // ---- the sector buffer, 256 words ----
    //
    // BOTH READ PORTS ARE REGISTERED, which is what makes this a block RAM
    // instead of 4096 flip-flops. An asynchronous read cannot map onto an
    // M10K, so Quartus builds it from registers plus a 256-to-1 mux per read
    // port -- that cost 4,250 ALMs, more than doubling the whole design, for
    // 512 bytes of storage.
    //
    // Two ports, one per user: port A is the CPU side addressed by windex,
    // port B is the transfer engine addressed by tidx. Each reads and writes,
    // which is exactly the true-dual-port shape an M10K provides.
    //
    // `no_rw_check` says the read-during-write behaviour does not matter, and
    // here it genuinely does not: each port only ever reads the address it is
    // writing in the same cycle as a side effect, and that value is discarded
    // -- after a CPU write to DATA the index advances immediately, and the
    // transfer engine never reads the buffer during a fill or writes it during
    // a flush. Without this Quartus refuses to infer the RAM at all ("unsupported
    // read-during-write behavior") and silently spends 4,250 ALMs on flops.
    (* ramstyle = "no_rw_check" *) logic [15:0] sbuf [0:255];
    logic [15:0] q_cpu, q_xfer;

    // ---- registers ----
    logic [23:0] lba_set;      // what software has written
    logic [23:0] lba_cur;      // what the active transfer is using
    logic [7:0]  windex;       // word within the sector, 0..255
    logic        drq, err;

    // ---- transfer engine ----
    localparam logic [2:0] T_IDLE  = 3'd0;
    localparam logic [2:0] T_RD_RQ = 3'd1;
    localparam logic [2:0] T_RD_NX = 3'd2;
    localparam logic [2:0] T_WR_SU = 3'd6;
    localparam logic [2:0] T_WR_RQ = 3'd3;
    localparam logic [2:0] T_WR_NX = 3'd4;
    localparam logic [2:0] T_DONE  = 3'd5;

    logic [2:0] tstate;
    logic [7:0] tidx;
    logic       is_read;

    logic busy;
    assign busy = (tstate != T_IDLE);

    // ---- SDRAM request ----
    // The word being moved lives at BASE + lba*512 + index*2.
    logic [23:0] xfer_addr;
    assign xfer_addr = BASE + {lba_cur[14:0], 9'd0} + {15'd0, tidx, 1'b0};

    assign mem_addr  = xfer_addr;
    assign mem_wdata = q_xfer;
    assign mem_be    = 2'b11;
    assign mem_rd    = USE_SDRAM && (tstate == T_RD_RQ);
    assign mem_wr    = USE_SDRAM && (tstate == T_WR_RQ);

    // ---- the ROM backend ----
    // Read synchronously so it infers block RAM rather than an enormous mux.
    logic [15:0] rom_q;
    generate
        if (!USE_SDRAM) begin : g_rom
            localparam int WORDS   = SECTORS * 256;
            localparam int WORD_AW = $clog2(WORDS);

            logic [15:0] disk [0:WORDS-1];
            initial $readmemh(INIT_FILE, disk);

            logic [WORD_AW-1:0] rom_addr;
            assign rom_addr = {lba_cur[WORD_AW-9:0], tidx};

            always_ff @(posedge clk) rom_q <= disk[rom_addr];
        end else begin : g_no_rom
            assign rom_q = 16'h0000;
        end
    endgenerate

    // ---- read mux ----
    always_comb begin
        case (reg_sel)
            R_DATA:    rdata = q_cpu;
            R_CMD:     rdata = {13'd0, err, drq, busy};
            R_SECTORS: rdata = SECTOR_COUNT;
            default:   rdata = 16'h0000;
        endcase
    end

    // ---- port A: the CPU side ----
    logic bufa_we;
    assign bufa_we = sel && wr && !busy && (reg_sel == R_DATA) && drq;

    // Plain `always`, not `always_ff`: an M10K's two ports are written from
    // two separate blocks, and always_ff forbids a second driver.
    always @(posedge clk) begin
        if (bufa_we) sbuf[windex] <= wdata;
        q_cpu <= sbuf[windex];
    end

    // ---- port B: the transfer engine ----
    // A read fills the buffer from whichever backend is in use; a write reads
    // it back out through q_xfer.
    logic        bufb_we;
    logic [15:0] bufb_data;
    assign bufb_we   = USE_SDRAM ? ((tstate == T_RD_RQ) && mem_ready)
                                 : (tstate == T_RD_NX);
    assign bufb_data = USE_SDRAM ? mem_rdata : rom_q;

    always @(posedge clk) begin
        if (bufb_we) sbuf[tidx] <= bufb_data;
        q_xfer <= sbuf[tidx];
    end

    // A DATA access only advances the index while data is actually on offer.
    // Reading past the end of a sector must not walk into the next one.
    logic data_taken;
    assign data_taken = sel && (rd || wr) && (reg_sel == R_DATA) && drq && !busy;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lba_set <= 24'd0;
            lba_cur <= 24'd0;
            windex  <= 8'd0;
            drq     <= 1'b0;
            err     <= 1'b0;
            tstate  <= T_IDLE;
            tidx    <= 8'd0;
            is_read <= 1'b1;
        end else begin
            // ---------------- the transfer engine ----------------
            case (tstate)
                T_RD_RQ: begin
                    if (USE_SDRAM) begin
                        // SDRAM answers with a ready pulse and its data in the
                        // same cycle, so take it here.
                        if (mem_ready) tstate <= T_RD_NX;
                    end else begin
                        // The ROM's output is REGISTERED off an address driven
                        // by tidx, so it is a cycle behind: sampling it here
                        // would store the previous word and shift the whole
                        // sector by one. This state exists to let it catch up.
                        tstate <= T_RD_NX;
                    end
                end
                T_RD_NX: begin
                    // For the ROM the data is valid now, for tidx as it stood
                    // during T_RD_RQ. For SDRAM this is simply a cycle with the
                    // request deasserted, which is what the arbiter needs to
                    // see between accesses.
                    if (tidx == 8'd255) begin
                        tstate <= T_DONE;
                    end else begin
                        tidx   <= tidx + 8'd1;
                        tstate <= T_RD_RQ;
                    end
                end

                // One cycle for the buffer's registered output to catch up
                // with tidx before the data is put on the memory bus.
                T_WR_SU: tstate <= T_WR_RQ;
                T_WR_RQ: if (mem_ready) tstate <= T_WR_NX;
                T_WR_NX: begin
                    if (tidx == 8'd255) begin
                        tstate <= T_DONE;
                    end else begin
                        tidx   <= tidx + 8'd1;
                        tstate <= T_WR_SU;
                    end
                end

                T_DONE: begin
                    // A read leaves the sector on offer; a write does not.
                    drq    <= is_read;
                    windex <= 8'd0;
                    tstate <= T_IDLE;
                end
                default: ;   // T_IDLE
            endcase

            // ---------------- register writes ----------------
            if (sel && wr && !busy) begin
                case (reg_sel)
                    R_LBA_LO: lba_set[15:0]  <= wdata;
                    R_LBA_HI: lba_set[23:16] <= wdata[7:0];
                    R_CMD: begin
                        if ((wdata == CMD_READ) ||
                            (wdata == CMD_WRITE && USE_SDRAM)) begin
                            // Range check up front: an out-of-range access
                            // reports ERR and moves nothing, rather than
                            // wrapping around and quietly hitting the wrong
                            // sector -- or, with the SDRAM backend, writing
                            // over whatever lies past the end of the disk.
                            if (lba_set < SECTOR_LIMIT) begin
                                lba_cur <= lba_set;
                                tidx    <= 8'd0;
                                drq     <= 1'b0;
                                err     <= 1'b0;
                                is_read <= (wdata == CMD_READ);
                                tstate  <= (wdata == CMD_READ) ? T_RD_RQ
                                                               : T_WR_SU;
                            end else begin
                                err <= 1'b1;
                                drq <= 1'b0;
                            end
                        end else begin
                            // An unknown command, or a write aimed at the ROM
                            // backend, which has nowhere to put it.
                            err <= 1'b1;
                            drq <= 1'b0;
                        end
                    end
                    default: ;
                endcase
            end

            // ---------------- the data window ----------------
            if (data_taken) begin
                windex <= windex + 8'd1;
                // 255 -> 0 ends the sector.
                if (windex == 8'd255) drq <= 1'b0;
            end
        end
    end

endmodule
