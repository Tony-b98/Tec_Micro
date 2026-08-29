.include "m328Pdef.inc"

.def temp    = r16
.def A       = r17
.def B       = r18
.def S       = r19
.def F       = r20
.def C_flag  = r21
.def aux     = r22
.def aux2    = r23
.def salida  = r24

.equ S_CLEAR = 0
.equ S_SUB   = 1
.equ S_ADD   = 2
.equ S_XOR   = 3
.equ S_AND   = 4
.equ S_OR    = 5
.equ S_SHL   = 6
.equ S_INC   = 7

.org 0x0000
    rjmp RESET_ALU
