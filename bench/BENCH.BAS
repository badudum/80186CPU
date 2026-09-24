' FPGA80186 benchmark
'
' Nine timed loops covering the usual kinds of work.  Run it, write the
' numbers down, change something, run it again.  Only the difference between
' two runs means anything: the absolute figures are mostly QBASIC's
' interpreter, which sits underneath every line here.
'
' SIZING.  TIMER ticks at 18.2 Hz, so its resolution is 55 ms.  Each test is
' aimed at several seconds, which keeps the granularity near one percent.
' Raise SCALE for longer, steadier runs.
'
' TWO THINGS THAT WILL BITE ANY BASIC YOU WRITE HERE:
'   DEFINT A-Z makes every variable a 16-bit integer, so anything that can
'   reach 32768 overflows and stops the program.  Counters below are masked
'   with AND 255 and BIG * 2 is never computed for that reason.
'   Timing variables need the ! suffix or they are integers too, and every
'   result truncates to a whole number of seconds.

REM $DYNAMIC
DEFINT A-Z

CONST SMALL = 2000                ' 4 KB of integers
CONST BIG = 20000                 ' 40 KB of integers
CONST SCALE = 1

DIM A(SMALL)
DIM B(BIG)

SREP = 30 * SCALE
BREP = 3 * SCALE

CLS
PRINT "FPGA80186 benchmark"
PRINT "small array"; SMALL; "ints     big array"; BIG; "ints"
PRINT

' ---- 1. integer arithmetic --------------------------------------------
T! = TIMER
C = 0
FOR R = 1 TO SREP
  FOR I = 1 TO SMALL
    C = (C + I) AND 255
    C = (C * 3) AND 255
  NEXT I
NEXT R
T1! = TIMER - T!
PRINT "1 integer arithmetic "; T1!; "s"

' ---- 2. conditional branches ------------------------------------------
' A cheap integer generator drives the test, so the branch goes both ways.
' S stays under 4096 so S * 5 cannot overflow.
T! = TIMER
S = 1
C = 0
FOR R = 1 TO SREP
  FOR I = 1 TO SMALL
    S = (S * 5 + 3) AND 4095
    IF S > 2048 THEN C = (C + 1) AND 255 ELSE C = (C - 1) AND 255
  NEXT I
NEXT R
T2! = TIMER - T!
PRINT "2 conditionals       "; T2!; "s"

' ---- 3. small array, in order -----------------------------------------
T! = TIMER
FOR R = 1 TO SREP
  FOR I = 1 TO SMALL
    A(I) = (A(I) + 1) AND 255
  NEXT I
NEXT R
T3! = TIMER - T!
PRINT "3 small array        "; T3!; "s"

' ---- 4. big array, in order -------------------------------------------
T! = TIMER
FOR R = 1 TO BREP
  FOR I = 1 TO BIG
    B(I) = (B(I) + 1) AND 255
  NEXT I
NEXT R
T4! = TIMER - T!
PRINT "4 big array          "; T4!; "s"

' ---- 5. big array, scattered ------------------------------------------
' Same array and the same number of accesses as test 4, in a jumbled order.
T! = TIMER
FOR R = 1 TO BREP
  J = 0
  FOR I = 1 TO BIG
    J = J + 4099
    IF J >= BIG THEN J = J - BIG
    B(J) = (B(J) + 1) AND 255
  NEXT I
NEXT R
T5! = TIMER - T!
PRINT "5 scattered access   "; T5!; "s"

' ---- 6. subroutine calls ----------------------------------------------
T! = TIMER
FOR R = 1 TO SREP
  FOR I = 1 TO SMALL
    GOSUB Outer
  NEXT I
NEXT R
T6! = TIMER - T!
PRINT "6 subroutine calls   "; T6!; "s"

' ---- 7. multiply and divide -------------------------------------------
T! = TIMER
FOR R = 1 TO SREP
  FOR I = 1 TO SMALL
    X = (I * 3) \ 7
    Y = (I * 5) MOD 11
  NEXT I
NEXT R
T7! = TIMER - T!
PRINT "7 multiply divide    "; T7!; "s"

' ---- 8. block copy -----------------------------------------------------
T! = TIMER
FOR R = 1 TO BREP
  FOR I = 1 TO BIG - 1
    B(I) = B(I + 1)
  NEXT I
NEXT R
T8! = TIMER - T!
PRINT "8 block copy         "; T8!; "s"

' ---- 9. strings --------------------------------------------------------
' String work is the one test here that allocates, so it also exercises
' QBASIC's heap rather than only the core.
T! = TIMER
FOR R = 1 TO SREP
  FOR I = 1 TO 200
    Q$ = "FPGA80186"
    Q$ = Q$ + "0123456789"
    IF LEN(Q$) > 0 THEN Z = ASC(Q$)
  NEXT I
NEXT R
T9! = TIMER - T!
PRINT "9 strings            "; T9!; "s"

TOT! = T1! + T2! + T3! + T4! + T5! + T6! + T7! + T8! + T9!
PRINT
PRINT "TOTAL                "; TOT!; "s"
END

Outer:
  GOSUB Inner
  RETURN
Inner:
  W = (W + 1) AND 255
  RETURN
