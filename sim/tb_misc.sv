`timescale 1ns/1ns
//
// The long tail of the instruction set: sign extension, flag transfers, the
// decimal and ASCII adjusts, XCHG, segment push/pop, XLAT, LES, PUSHA/POPA,
// ENTER/LEAVE, BOUND, the 80186 three-operand IMUL, and the indirect
// transfer forms of group 5.
//
// Each test writes its result to a slot at 5000h so everything can be checked
// once at the end, rather than trying to catch register values mid-run.
//
module tb_misc;

    logic        clk = 0, rst_n = 0;
    logic [19:0] addr;
    logic [15:0] dout, din;
    logic        rd, wr, io_cycle, bhe, a0, ale;
    logic [2:0]  s;
    logic        ready;
    logic        nmi = 0;
    logic        int0 = 0, int1 = 0, int2 = 0, int3 = 0;
    logic        drq0 = 0, drq1 = 0;
    logic        intr_req = 0;
    logic [7:0]  intr_type = 8'h00;
    logic        intr_ack;
    logic        halted;
    logic [15:0] dbg_ip, dbg_cs, dbg_flags;
    logic        ext_eoi = 1'b0;   // the 8259 shim's EOI; unused here
    logic [7:0]  dbg_int_type;
    logic        dbg_int_taken;

    cpu_top dut (.*);

    always #5 clk = ~clk;

    logic [7:0] mem [0:'hFFFFF];
    logic [19:0] even_a, odd_a;
    assign even_a = {addr[19:1], 1'b0};
    assign odd_a  = {addr[19:1], 1'b1};
    assign din    = {mem[odd_a], mem[even_a]};
    assign ready  = 1'b1;

    always @(posedge clk) begin
        if (wr) begin
            if (!a0)  mem[even_a] <= dout[7:0];
            if (!bhe) mem[odd_a]  <= dout[15:8];
        end
    end

    int errors = 0, checks = 0;
    task chk(input string nm, input int got, input int exp);
        checks++;
        if (got !== exp) begin
            $display("FAIL %-36s got=%04h exp=%04h", nm, got, exp);
            errors++;
        end
    endtask
    // Result slots are little-endian words at 5000h.
    task chkw(input string nm, input int slot, input int exp);
        chk(nm, {mem['h5000 + slot + 1], mem['h5000 + slot]}, exp);
    endtask

    int i, p;
    task put(input int a, input byte b); begin mem[a] = b; end endtask
    task vec(input int typ, input int off, input int seg);
        begin
            mem[typ*4+0] = off[7:0];  mem[typ*4+1] = off[15:8];
            mem[typ*4+2] = seg[7:0];  mem[typ*4+3] = seg[15:8];
        end
    endtask

    initial begin
        for (i = 0; i < 'hFFFFF; i++) mem[i] = 8'h00;

        vec('h05, 'h0390, 'hF000);   // BOUND range exceeded -> HLT
        vec('h06, 'h0390, 'hF000);   // illegal opcode      -> HLT

        // data the program reads
        put('h2003, 8'h9E);                              // XLAT table entry 3
        put('h2100, 8'h34); put('h2101, 8'h12);          // LES offset 1234
        put('h2102, 8'h78); put('h2103, 8'h56);          // LES segment 5678
        put('h2200, 8'h00); put('h2201, 8'h00);          // BOUND lower = 0
        put('h2202, 8'h0A); put('h2203, 8'h00);          // BOUND upper = 10
        put('h2300, 8'h80); put('h2301, 8'h02);          // far pointer offset 0280
        put('h2302, 8'h00); put('h2303, 8'hF0);          // far pointer segment F000

        // reset vector: far jump to F000:0100
        put('hFFFF0, 8'hEA);
        put('hFFFF1, 8'h00); put('hFFFF2, 8'h01);
        put('hFFFF3, 8'h00); put('hFFFF4, 8'hF0);

        p = 'hF0100;
        put(p++, 8'hBC); put(p++, 8'h00); put(p++, 8'h80); // MOV SP,8000h
        put(p++, 8'hB8); put(p++, 8'h00); put(p++, 8'h00); // MOV AX,0
        put(p++, 8'h8E); put(p++, 8'hD8);                  // MOV DS,AX
        put(p++, 8'h8E); put(p++, 8'hC0);                  // MOV ES,AX
        // CBW: AL=80h sign-extends to FF80h
        put(p++, 8'hB8); put(p++, 8'h80); put(p++, 8'h00); // MOV AX,0080h
        put(p++, 8'h98);                                   // CBW
        put(p++, 8'hA3); put(p++, 8'h00); put(p++, 8'h50); // MOV [5000h],AX
        // CWD: AX=8000h sign-extends into DX
        put(p++, 8'hB8); put(p++, 8'h00); put(p++, 8'h80); // MOV AX,8000h
        put(p++, 8'h99);                                   // CWD
        put(p++, 8'h89); put(p++, 8'h16); put(p++, 8'h02); put(p++, 8'h50); // MOV [5002h],DX
        // LAHF with CF set
        put(p++, 8'hF9);                                   // STC
        put(p++, 8'h9F);                                   // LAHF
        put(p++, 8'hA3); put(p++, 8'h04); put(p++, 8'h50); // MOV [5004h],AX
        // PUSHF then POPF must restore the flags that were pushed
        put(p++, 8'hF8);                                   // CLC
        put(p++, 8'h9C);                                   // PUSHF
        put(p++, 8'hF9);                                   // STC
        put(p++, 8'h9D);                                   // POPF   (CF back to 0)
        put(p++, 8'hB8); put(p++, 8'h00); put(p++, 8'h00); // MOV AX,0
        put(p++, 8'h9F);                                   // LAHF
        put(p++, 8'hA3); put(p++, 8'h06); put(p++, 8'h50); // MOV [5006h],AX
        // PUSH/POP a segment register
        put(p++, 8'hB8); put(p++, 8'h34); put(p++, 8'h12); // MOV AX,1234h
        put(p++, 8'h8E); put(p++, 8'hC0);                  // MOV ES,AX
        put(p++, 8'h06);                                   // PUSH ES
        put(p++, 8'hB8); put(p++, 8'h00); put(p++, 8'h00); // MOV AX,0
        put(p++, 8'h8E); put(p++, 8'hC0);                  // MOV ES,AX
        put(p++, 8'h07);                                   // POP ES
        put(p++, 8'h8C); put(p++, 8'hC0);                  // MOV AX,ES
        put(p++, 8'hA3); put(p++, 8'h08); put(p++, 8'h50); // MOV [5008h],AX
        // XCHG AX,BX
        put(p++, 8'hB8); put(p++, 8'h11); put(p++, 8'h11); // MOV AX,1111h
        put(p++, 8'hBB); put(p++, 8'h22); put(p++, 8'h22); // MOV BX,2222h
        put(p++, 8'h93);                                   // XCHG AX,BX
        put(p++, 8'hA3); put(p++, 8'h0A); put(p++, 8'h50); // MOV [500Ah],AX
        put(p++, 8'h89); put(p++, 8'h1E); put(p++, 8'h0C); put(p++, 8'h50); // MOV [500Ch],BX
        // XCHG CX,DX through the ModR/M form
        put(p++, 8'hB9); put(p++, 8'h33); put(p++, 8'h33); // MOV CX,3333h
        put(p++, 8'hBA); put(p++, 8'h44); put(p++, 8'h44); // MOV DX,4444h
        put(p++, 8'h87); put(p++, 8'hCA);                  // XCHG CX,DX
        put(p++, 8'h89); put(p++, 8'h0E); put(p++, 8'h0E); put(p++, 8'h50); // MOV [500Eh],CX
        put(p++, 8'h89); put(p++, 8'h16); put(p++, 8'h10); put(p++, 8'h50); // MOV [5010h],DX
        // DAA after a binary add that produced a non-decimal digit
        put(p++, 8'hB8); put(p++, 8'h15); put(p++, 8'h00); // MOV AX,0015h
        put(p++, 8'h04); put(p++, 8'h06);                  // ADD AL,6   -> 1Bh
        put(p++, 8'h27);                                   // DAA        -> 21h
        put(p++, 8'hA3); put(p++, 8'h12); put(p++, 8'h50); // MOV [5012h],AX
        // AAA
        put(p++, 8'hB8); put(p++, 8'h0B); put(p++, 8'h00); // MOV AX,000Bh
        put(p++, 8'h37);                                   // AAA        -> 0101h
        put(p++, 8'hA3); put(p++, 8'h14); put(p++, 8'h50); // MOV [5014h],AX
        // AAM then AAD should round-trip 27
        put(p++, 8'hB8); put(p++, 8'h1B); put(p++, 8'h00); // MOV AX,001Bh (27)
        put(p++, 8'hD4); put(p++, 8'h0A);                  // AAM 10     -> 0207h
        put(p++, 8'hA3); put(p++, 8'h16); put(p++, 8'h50); // MOV [5016h],AX
        put(p++, 8'hB8); put(p++, 8'h07); put(p++, 8'h02); // MOV AX,0207h
        put(p++, 8'hD5); put(p++, 8'h0A);                  // AAD 10     -> 001Bh
        put(p++, 8'hA3); put(p++, 8'h18); put(p++, 8'h50); // MOV [5018h],AX
        // XLAT
        put(p++, 8'hBB); put(p++, 8'h00); put(p++, 8'h20); // MOV BX,2000h
        put(p++, 8'hB8); put(p++, 8'h03); put(p++, 8'h00); // MOV AX,0003h
        put(p++, 8'hD7);                                   // XLAT       -> AL = 9Eh
        put(p++, 8'hA3); put(p++, 8'h1A); put(p++, 8'h50); // MOV [501Ah],AX
        // LES loads both a register and a segment register
        put(p++, 8'hC4); put(p++, 8'h1E); put(p++, 8'h00); put(p++, 8'h21); // LES BX,[2100h]
        put(p++, 8'h89); put(p++, 8'h1E); put(p++, 8'h1C); put(p++, 8'h50); // MOV [501Ch],BX
        put(p++, 8'h8C); put(p++, 8'hC0);                  // MOV AX,ES
        put(p++, 8'hA3); put(p++, 8'h1E); put(p++, 8'h50); // MOV [501Eh],AX
        // PUSH imm then POP straight into memory
        put(p++, 8'h68); put(p++, 8'hCD); put(p++, 8'hAB); // PUSH ABCDh
        put(p++, 8'h8F); put(p++, 8'h06); put(p++, 8'h20); put(p++, 8'h50); // POP [5020h]
        // PUSHA / POPA
        put(p++, 8'hB8); put(p++, 8'h01); put(p++, 8'h00); // MOV AX,1
        put(p++, 8'hBB); put(p++, 8'h02); put(p++, 8'h00); // MOV BX,2
        put(p++, 8'h60);                                   // PUSHA
        put(p++, 8'hB8); put(p++, 8'hFF); put(p++, 8'hFF); // MOV AX,FFFFh
        put(p++, 8'hBB); put(p++, 8'hFF); put(p++, 8'hFF); // MOV BX,FFFFh
        put(p++, 8'h61);                                   // POPA
        put(p++, 8'hA3); put(p++, 8'h22); put(p++, 8'h50); // MOV [5022h],AX
        put(p++, 8'h89); put(p++, 8'h1E); put(p++, 8'h24); put(p++, 8'h50); // MOV [5024h],BX
        // ENTER / LEAVE at nesting level 0
        put(p++, 8'hBD); put(p++, 8'h00); put(p++, 8'h00); // MOV BP,0
        put(p++, 8'hC8); put(p++, 8'h04); put(p++, 8'h00); put(p++, 8'h00); // ENTER 4,0
        put(p++, 8'h89); put(p++, 8'h2E); put(p++, 8'h26); put(p++, 8'h50); // MOV [5026h],BP
        put(p++, 8'hC9);                                   // LEAVE
        put(p++, 8'h89); put(p++, 8'h2E); put(p++, 8'h28); put(p++, 8'h50); // MOV [5028h],BP
        put(p++, 8'h89); put(p++, 8'h26); put(p++, 8'h2A); put(p++, 8'h50); // MOV [502Ah],SP
        // BOUND with the index inside the range must NOT trap
        put(p++, 8'hBB); put(p++, 8'h05); put(p++, 8'h00); // MOV BX,5
        put(p++, 8'h62); put(p++, 8'h1E); put(p++, 8'h00); put(p++, 8'h22); // BOUND BX,[2200h]
        put(p++, 8'hB8); put(p++, 8'hAA); put(p++, 8'h00); // MOV AX,00AAh
        put(p++, 8'hA3); put(p++, 8'h2C); put(p++, 8'h50); // MOV [502Ch],AX
        // three-operand IMUL (80186)
        put(p++, 8'hBB); put(p++, 8'h03); put(p++, 8'h00); // MOV BX,3
        put(p++, 8'h6B); put(p++, 8'hDB); put(p++, 8'h05); // IMUL BX,BX,5
        put(p++, 8'h89); put(p++, 8'h1E); put(p++, 8'h2E); put(p++, 8'h50); // MOV [502Eh],BX
        // indirect JMP through a register
        put(p++, 8'hBB); put(p++, 8'h50); put(p++, 8'h02); // MOV BX,0250h
        put(p++, 8'hFF); put(p++, 8'hE3);                  // JMP BX

        // ---- landing at F000:0250 ----
        p = 'hF0250;
        put(p++, 8'hB8); put(p++, 8'h5A); put(p++, 8'h5A); // MOV AX,5A5Ah
        put(p++, 8'hA3); put(p++, 8'h30); put(p++, 8'h50); // MOV [5030h],AX
        put(p++, 8'hBB); put(p++, 8'h70); put(p++, 8'h02); // MOV BX,0270h
        put(p++, 8'hFF); put(p++, 8'hD3);                  // CALL BX
        put(p++, 8'h89); put(p++, 8'h3E); put(p++, 8'h32); put(p++, 8'h50); // MOV [5032h],DI
        put(p++, 8'hFF); put(p++, 8'hF3);                  // PUSH BX
        put(p++, 8'h8F); put(p++, 8'h06); put(p++, 8'h34); put(p++, 8'h50); // POP [5034h]
        put(p++, 8'hFF); put(p++, 8'h2E); put(p++, 8'h00); put(p++, 8'h23); // JMP FAR [2300h]

        // ---- near subroutine reached by CALL BX ----
        p = 'hF0270;
        put(p++, 8'hBF); put(p++, 8'h77); put(p++, 8'h77); // MOV DI,7777h
        put(p++, 8'hC3);                                   // RET

        // ---- far jump target ----
        p = 'hF0280;
        put(p++, 8'hB8); put(p++, 8'h8C); put(p++, 8'h8C); // MOV AX,8C8Ch
        put(p++, 8'hA3); put(p++, 8'h36); put(p++, 8'h50); // MOV [5036h],AX

        // ---- byte MUL and DIV write the WHOLE of AX ----
        // A byte MUL produces a 16-bit product in AX and a byte DIV leaves the
        // remainder in AH, so neither result is byte-wide even though the
        // operand is. AH is deliberately loaded with rubbish first: an
        // implementation that writes only AL passes every ALU-level test (the
        // arithmetic is right) and leaves the caller's AH sitting in the high
        // half. That is exactly what made the BIOS compute a screen address of
        // B9C00 instead of B8000.
        put(p++, 8'hB8); put(p++, 8'h00); put(p++, 8'hEE); // MOV AX,EE00h
        put(p++, 8'hB3); put(p++, 8'h50);                  // MOV BL,80
        put(p++, 8'hF6); put(p++, 8'hE3);                  // MUL BL
        put(p++, 8'hA3); put(p++, 8'h38); put(p++, 8'h50); // MOV [5038h],AX

        put(p++, 8'hB8); put(p++, 8'h07); put(p++, 8'hFF); // MOV AX,FF07h
        put(p++, 8'hB3); put(p++, 8'h32);                  // MOV BL,50
        put(p++, 8'hF6); put(p++, 8'hE3);                  // MUL BL
        put(p++, 8'hA3); put(p++, 8'h3A); put(p++, 8'h50); // MOV [503Ah],AX

        put(p++, 8'hB8); put(p++, 8'h65); put(p++, 8'h00); // MOV AX,0065h (101)
        put(p++, 8'hB3); put(p++, 8'h0A);                  // MOV BL,10
        put(p++, 8'hF6); put(p++, 8'hF3);                  // DIV BL
        put(p++, 8'hA3); put(p++, 8'h3C); put(p++, 8'h50); // MOV [503Ch],AX

        put(p++, 8'hB8); put(p++, 8'h00); put(p++, 8'h10); // MOV AX,1000h
        put(p++, 8'hBB); put(p++, 8'h10); put(p++, 8'h00); // MOV BX,0010h
        put(p++, 8'hF7); put(p++, 8'hE3);                  // MUL BX
        put(p++, 8'hA3); put(p++, 8'h3E); put(p++, 8'h50); // MOV [503Eh],AX
        put(p++, 8'h89); put(p++, 8'h16); put(p++, 8'h40); put(p++, 8'h50);
                                                           // MOV [5040h],DX

        // ---- XCHG with a MEMORY operand ----
        // The register-to-register form above needs no memory access at all,
        // so it cannot tell whether the r/m operand is actually read. This
        // form does both: the register must come back with what was in memory
        // AND memory must come back with what was in the register. A swap that
        // only goes one way leaves the register holding whatever happened to
        // be lying around, which is how MS-DOS ended up far-calling into
        // nothing.
        put(p++, 8'hC7); put(p++, 8'h06); put(p++, 8'h60); put(p++, 8'h50);
        put(p++, 8'hEF); put(p++, 8'hBE);                  // MOV word [5060h],BEEFh
        put(p++, 8'hB8); put(p++, 8'h0D); put(p++, 8'hF0); // MOV AX,F00Dh
        put(p++, 8'h87); put(p++, 8'h06); put(p++, 8'h60); put(p++, 8'h50);
                                                           // XCHG AX,[5060h]
        put(p++, 8'hA3); put(p++, 8'h50); put(p++, 8'h50); // MOV [5050h],AX
        put(p++, 8'hA1); put(p++, 8'h60); put(p++, 8'h50); // MOV AX,[5060h]
        put(p++, 8'hA3); put(p++, 8'h52); put(p++, 8'h50); // MOV [5052h],AX

        put(p++, 8'hC6); put(p++, 8'h06); put(p++, 8'h62); put(p++, 8'h50);
        put(p++, 8'h5A);                                   // MOV byte [5062h],5Ah
        put(p++, 8'hB0); put(p++, 8'hA5);                  // MOV AL,A5h
        put(p++, 8'h86); put(p++, 8'h06); put(p++, 8'h62); put(p++, 8'h50);
                                                           // XCHG AL,[5062h]
        put(p++, 8'hA2); put(p++, 8'h54); put(p++, 8'h50); // MOV [5054h],AL
        put(p++, 8'hA0); put(p++, 8'h62); put(p++, 8'h50); // MOV AL,[5062h]
        put(p++, 8'hA2); put(p++, 8'h55); put(p++, 8'h50); // MOV [5055h],AL

        // ...and through a segment override, which is the exact shape MS-DOS
        // uses to patch the far pointer it is about to call through.
        // ES = 0100h, so ES:4064h and DS:5064h are the same byte.
        put(p++, 8'hB8); put(p++, 8'h00); put(p++, 8'h01); // MOV AX,0100h
        put(p++, 8'h8E); put(p++, 8'hC0);                  // MOV ES,AX
        put(p++, 8'hC7); put(p++, 8'h06); put(p++, 8'h64); put(p++, 8'h50);
        put(p++, 8'h34); put(p++, 8'h12);                  // MOV word [5064h],1234h
        put(p++, 8'hB8); put(p++, 8'h78); put(p++, 8'h56); // MOV AX,5678h
        put(p++, 8'h26); put(p++, 8'h87); put(p++, 8'h06);
        put(p++, 8'h64); put(p++, 8'h40);                  // ES: XCHG AX,[4064h]
        put(p++, 8'hA3); put(p++, 8'h56); put(p++, 8'h50); // MOV [5056h],AX
        put(p++, 8'hA1); put(p++, 8'h64); put(p++, 8'h50); // MOV AX,[5064h]
        put(p++, 8'hA3); put(p++, 8'h58); put(p++, 8'h50); // MOV [5058h],AX

        put(p++, 8'hF4);                                   // HLT

        // Well clear of the code above: this used to sit at F0290 and was
        // quietly overwriting the tail of the far-jump target's block.
        put('hF0390, 8'hF4);                               // trap handler: stop

        repeat (4) @(negedge clk);
        rst_n = 1;

        i = 0;
        while (!halted && i < 200000) begin @(negedge clk); i++; end
        chk("cpu halted", halted, 1'b1);
        chk("no unexpected trap", dbg_int_taken, 1'b0);

        chkw("CBW sign-extended AL",      'h00, 16'hFF80);
        chkw("CWD sign-extended into DX", 'h02, 16'hFFFF);
        // LAHF puts the flag byte in AH. Bit 1 always reads 1, so CF set gives 03h.
        chk ("LAHF captured CF set",   mem['h5005], 8'h03);
        chk ("POPF restored CF clear", mem['h5007], 8'h02);
        chkw("PUSH/POP ES round-trip",    'h08, 16'h1234);
        chkw("XCHG AX,BX gave AX",        'h0A, 16'h2222);
        chkw("XCHG AX,BX gave BX",        'h0C, 16'h1111);
        chkw("XCHG CX,DX gave CX",        'h0E, 16'h4444);
        chkw("XCHG CX,DX gave DX",        'h10, 16'h3333);
        chkw("XCHG r16,m16 loaded memory into the register", 'h50, 16'hBEEF);
        chkw("XCHG r16,m16 stored the register into memory", 'h52, 16'hF00D);
        chk ("XCHG r8,m8 loaded memory into the register",
             mem['h5054], 8'h5A);
        chk ("XCHG r8,m8 stored the register into memory",
             mem['h5055], 8'hA5);
        chkw("XCHG read through a segment override",  'h56, 16'h1234);
        chkw("XCHG wrote through a segment override", 'h58, 16'h5678);
        chk ("DAA adjusted AL",        mem['h5012], 8'h21);
        chkw("AAA split into AH:AL",      'h14, 16'h0101);
        chkw("AAM split 27 into 2 and 7", 'h16, 16'h0207);
        chkw("AAD recombined to 27",      'h18, 16'h001B);
        chk ("XLAT indexed the table",  mem['h501A], 8'h9E);
        chkw("LES loaded the offset",     'h1C, 16'h1234);
        chkw("LES loaded the segment",    'h1E, 16'h5678);
        chkw("PUSH imm / POP mem",        'h20, 16'hABCD);
        chkw("POPA restored AX",          'h22, 16'h0001);
        chkw("POPA restored BX",          'h24, 16'h0002);
        chkw("ENTER set BP to the frame", 'h26, 16'h7FFE);
        chkw("LEAVE restored BP",         'h28, 16'h0000);
        chkw("ENTER/LEAVE balanced SP",   'h2A, 16'h8000);
        chkw("BOUND in range did not trap", 'h2C, 16'h00AA);
        chkw("IMUL r,rm,imm",             'h2E, 16'h000F);
        chkw("indirect JMP reached target", 'h30, 16'h5A5A);
        chkw("indirect CALL ran and returned", 'h32, 16'h7777);
        chkw("PUSH r/m then POP to memory",    'h34, 16'h0270);
        chkw("indirect far JMP reached target",'h36, 16'h8C8C);

        // byte MUL/DIV must overwrite AH, not preserve it
        chkw("byte MUL 0*80 clears a dirty AH", 'h38, 16'h0000);
        chkw("byte MUL 7*50 = 350 across AX",   'h3A, 16'h015E);
        chkw("byte DIV 101/10 -> AL=10 AH=1",   'h3C, 16'h010A);
        chkw("word MUL low half",               'h3E, 16'h0000);
        chkw("word MUL high half in DX",        'h40, 16'h0001);

        $display("");
        $display("==================================");
        $display(" checks: %0d   failures: %0d", checks, errors);
        $display("==================================");
        if (errors == 0) $display("ALL TESTS PASSED");
        $finish;
    end

    initial begin
        #6000000;
        $display("FAIL global timeout (IP=%04h int=%02h taken=%b)",
                 dbg_ip, dbg_int_type, dbg_int_taken);
        $display(" checks: %0d   failures: %0d", checks, errors + 1);
        $finish;
    end

endmodule
