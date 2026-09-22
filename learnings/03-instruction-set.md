# 03 — Instruction Set: Overview, Encoding, 80186 vs 8086 Differences

## Baseline

The 80186 executes the full 8086/8088 instruction set unmodified (object-code compatible),
using the same opcode encoding: `[prefixes] opcode [ModR/M] [SIB-not-present-on-8086]
[displacement] [immediate]`. `decode.sv` should implement the standard 8086 opcode map
first — this is well-documented (Intel MCS-86 manual / countless 8086 references) — then
layer the 80186-specific additions below on top. This file focuses on **what's different**,
plus a clock-cycle reference table pulled directly from the 80186 datasheet (values are for
the 80186; 80188 cycle counts differ only where a memory operand forces extra bus cycles
through the narrower 8-bit external bus).

## New 80186 instructions (10 types, require `$MOD186` on period assemblers)

These appear "shaded" in the datasheet's instruction-set summary. Full semantics from AP-186
Appendix H:

| Instruction | Encoding notes | Semantics |
|---|---|---|
| `PUSH imm` | opcode `011010s0` + data | Push an immediate byte (sign-extended to word) or word directly onto the stack. |
| `PUSHA` | `01100000` | Push AX,CX,DX,BX,**original SP**,BP,SI,DI in that order (36 clocks). SP pushed is its value *before* AX was pushed. |
| `POPA` | `01100001` | Inverse of PUSHA; the popped SP value is discarded (51 clocks). |
| `IMUL reg,r/m,imm` | `011010s1` + ModR/M + imm | 3-operand signed multiply by an immediate; result truncated to 16 bits, placed in any GP/pointer register. |
| Shift/rotate by immediate count | `1100000w` + ModR/M(TTT) + count byte | All of ROL/ROR/RCL/RCR/SHL/SAL/SHR/SAR gain a form taking an immediate shift count (previously only "by 1" or "by CL" existed). Count is ANDed with 1Fh (mod 32) before use — same masking applied to the existing "by CL" form on the 80186 (the 8086 does **not** mask; see differences below). |
| `INS` (input string) | `0110110w`, repeatable | Block input from I/O port (address in DX) to memory at `ES:DI`; DI += 1 or 2 per element depending on DF; ES cannot be overridden. |
| `OUTS` (output string) | `0110111w`, repeatable | Block output from memory at `DS:SI` (DS overridable) to I/O port in DX; SI adjusted like INS. DX itself is never modified by either instruction. |
| `BOUND reg,mem` | `01100010` + ModR/M | Signed-compares `reg` against a `[lower,upper]` pair stored in two consecutive words at `mem`; if out of bounds (strictly outside, inclusive endpoints OK), raises **interrupt type 5**. Used for array bounds checking. |
| `ENTER disp,level` | `11001000` + data-low + data-high + level byte | Builds a stack frame for nested block-structured-language procedures (see pseudocode below). `disp` = local variable space (0–65535), `level` = lexical nesting level (0–255). |
| `LEAVE` | `11001001` | Tears down an ENTER frame: `SP := BP; BP := pop()`. Does **not** itself RET. |

`ENTER` pseudocode (from AP-186 Appendix H, needed if the microcode sequencer implements it
directly rather than as a macro of simpler micro-ops):
```
PUSH BP
if level == 0:
    BP := SP
else:
    temp1 := SP
    temp2 := level - 1
    while temp2 > 0:
        BP := BP - 2
        PUSH [BP]
        temp2 -= 1
    BP := temp1
    PUSH BP
SP := SP - disp
```

## Execution-behavior differences vs the 8086 (same opcode, different result — AP-186 §9.3)

These matter for correctness even without adding new opcodes:

| Behavior | 8086 | 80186 |
|---|---|---|
| Opcodes `63H,64H,65H,66H,67H,F1H`, and `FEH`/`FFH` with reg field `xx111xxx` | Ignored (treated as NOP-ish/undefined but harmless) | **Illegal instruction exception, interrupt type 6** |
| Opcode `0FH` | Executes `POP CS` | **Illegal instruction exception, interrupt type 6** |
| Word write at segment offset FFFFH | Wraps: second byte written at offset 0000h (same segment) | Second byte written at offset **10000h** (i.e. one byte past segment end — genuine "segment overflow", not a wraparound) |
| Stack PUSH when SP = 1 | (not specifically called out) | One-byte segment underflow occurs (writes below offset 0) |
| Shift/rotate count > 31 (CL or immediate) | Uses the full count as given (e.g. shifts 33 bits, which for a 16-bit value is observably different from shifting 1 bit) | **ANDs count with 1Fh first** (mod-32) — shifting by 33 behaves like shifting by 1 |
| LOCK prefix activation timing | LOCK asserted immediately when the prefix is decoded (can activate well before the locked bus cycle actually runs; prefetching can be locked) | LOCK asserted only when the CPU is ready to run the *actual* locked data cycle; opcode prefetching is **never** locked |
| Interrupted repeated string instruction — return address pushed | Points at the last prefix before the string op; multiple prefixes (e.g. segment override + REP) are not correctly re-executed on return | Points at the **first** prefix, so the instruction resumes correctly even with multiple prefixes, as long as prefixes aren't themselves repeated |
| Integer divide overflow condition | Divide error if `|quotient| > 7FFFh` (word) or `> 7Fh` (byte) | Range extended by one to include `8000h`/`80h` (the most-negative representable 2's-complement values) before triggering divide error |
| ESC opcode (coprocessor) trap | No such option | Can be **programmed** (via the peripheral relocation register's ET bit) to trap to interrupt type 7 whenever an ESC opcode is seen — useful for software-emulating an absent 8087 |

The safest single test to distinguish 8086 vs 80186 at runtime is the shift-count masking
behavior (AP-186's own recommended detection trick) — worth keeping in mind if writing any
compatibility-detection or self-test code for bring-up.

## Instruction timing reference (80186, 8 MHz part; from 80186 datasheet Instruction Set Summary)

Clock counts are `register/memory` form where two numbers are shown (e.g. `3/10` = 3 clocks
register operand, 10 clocks memory operand); a trailing `*` means "shown for byte transfers,
add 4 clocks per memory transfer for word operations."

**Data transfer**
| Instruction | Clocks |
|---|---|
| MOV reg↔reg | 2 |
| MOV reg↔mem | 9/12 |
| MOV imm→reg | 3/4 |
| MOV imm→mem | 12/13 |
| MOV mem↔accumulator (direct addr) | 8/9 |
| PUSH reg / PUSH mem / PUSH segreg | 10 / 16 / 9 |
| PUSH imm | 10 |
| PUSHA | 36 |
| POP reg / POP mem / POP segreg | 10 / 20 / 8 |
| POPA | 51 |
| XCHG reg,reg | 3 |
| XCHG reg,mem | 4/17 |
| IN/OUT fixed port | 10 / 9 |
| IN/OUT variable (DX) port | 8 / 7 |
| XLAT | 11 |
| LEA | 6 |
| LDS / LES | 18 |
| LAHF / SAHF | 2 / 3 |
| PUSHF / POPF | 9 / 8 |

**Arithmetic/logic** (ADD/ADC/SUB/SBB/CMP/AND/OR/XOR/TEST all similar):
| Form | Clocks |
|---|---|
| reg/mem with reg → either | 3/10 |
| immediate → reg/mem | 4/16 |
| immediate → accumulator | 3/4 |
| INC/DEC reg | 3 |
| INC/DEC mem | 15 |
| NEG / NOT mem | 3/10 |
| MUL (unsigned) byte reg/mem | 26–28 / 32–34 |
| MUL (unsigned) word reg/mem | 35–37 / 41–43 |
| IMUL (signed) byte reg/mem | 25–28 / 31–34 |
| IMUL (signed) word reg/mem | 34–37 / 40–43 |
| IMUL reg,r/m,imm | 22–25 (byte imm) / 29–32 (word imm) |
| DIV (unsigned) byte reg/mem | 29 / 35 |
| DIV (unsigned) word reg/mem | 38 / 44 |
| IDIV (signed) byte reg/mem | 44–52 / 50–58 |
| IDIV (signed) word reg/mem | 53–61 / 59–67 (**slowest instruction on the chip**) |
| AAA/AAS | 8 / 7 |
| DAA/DAS | 4 / 4 |
| AAM / AAD | 19 / 15 |
| CBW / CWD | 2 / 4 |

This confirms `ALU.sv`'s stub comment that MUL/IMUL/DIV/IDIV are genuinely multicycle
(26–67 clocks) and must be sequenced by `execUnit.sv`/`microcode.sv`, not treated as
1-cycle combinational ops — the current stub's synchronous `count`-based iteration for MUL
is the right shape, but note it needs ~16 iterations for a word multiply via shift-add, not
a single always-block pass, and the existing draft has a bug (`product <= product + ...`
races with `count`, and IMUL doesn't yet use `$signed`) — see the ALU stub's own inline
`// FIX` comments.

**Shift/rotate**
| Form | Clocks |
|---|---|
| reg/mem by 1 | 2/15 |
| reg/mem by CL | `5+n` / `17+n` (n = shift count) |
| reg/mem by immediate count | `5+n` / `17+n` |

**String ops** (unrepeated / repeated-by-CX-count `n`):
| Instruction | Unrepeated | Repeated (`REP`) |
|---|---|---|
| MOVS | 14 | `8 + 8n` |
| CMPS | 22 | `5 + 22n` |
| SCAS | 15 | `5 + 15n` |
| LODS | 12 | `6 + 11n` |
| STOS | 10 | `6 + 9n` |
| INS | 14 | `8 + 8n` |
| OUTS | 14 | `8 + 8n` |

**Control transfer**
| Instruction | Clocks |
|---|---|
| CALL near direct | 15 |
| CALL near indirect reg/mem | 13/19 |
| CALL far direct | 23 |
| CALL far indirect mem | 38 |
| RET near / near+imm | 16 / 18 |
| RET far / far+imm | 22 / 25 |
| Jcc (not taken / taken) | 4 / 13 |
| JCXZ | 5 / 15 |
| LOOP / LOOPZ / LOOPNZ | 6 / 16 |
| JMP short/near direct | 14 |
| JMP near indirect reg/mem | 11/17 |
| JMP far direct | 14 |
| JMP far indirect mem | 26 |
| ENTER, level=0 | 15 |
| ENTER, level=1 | 25 |
| ENTER, level=n>1 | `22 + 16(n-1)` |
| LEAVE | 8 |
| INT (type specified) | 47 |
| INT 3 | 45 |
| INTO (taken/not taken) | 48 / 4 |
| IRET | 28 |
| BOUND | 33–35 |

**Processor control**: CLC/CMC/STC/CLD (and by symmetry STD/CLI/STI/HLT/NOP/WAIT/LOCK
prefix/ESC) are all in the 2-clock class for the flag-only ones; consult the datasheet
directly (page ~31–32 of the PDF) if exact HLT/WAIT/ESC timing matters for cycle-accurate
verification later — not reproduced here since they weren't in the extracted range but
follow the same table format immediately after CLD in the datasheet.

## Practical notes for `decode.sv` / `microcode.sv`

- ModR/M decoding, addressing-mode tables, and segment-override handling are unchanged from
  8086 — reuse any standard 8086 decode reference/table.
- The 80186-new instructions above are the only opcodes needing brand-new decode paths;
  everything else is "8086 plus a mod-32 mask on shift counts and a couple of new illegal
  opcode traps."
- Multicycle instructions (MUL/IMUL/DIV/IDIV, string REP forms, PUSHA/POPA, ENTER with
  level>1) all need a sequencer state, not single-shot combinational decode — this validates
  the project's existing `microcode.sv` placeholder as the right architectural slot for them.
