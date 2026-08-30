.include "m328Pdef.inc"

.def temp    = r16
.def A       = r17
.def B       = r18
.def S       = r19
.def F       = r20
.def C_flag  = r21
.def aux     = r22

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


RESET_ALU:
    ldi temp, low(RAMEND)
    out SPL, temp

    ldi temp, high(RAMEND)
    out SPH, temp

    ldi temp, 0b00110000
    out DDRC, temp

    ldi temp, 0b00001111
    out PORTC, temp

    ldi temp, 0b00000000
    out DDRD, temp

    ldi temp, 0b11111100
    out PORTD, temp

    ldi temp, 0b00111110
    out DDRB, temp

    ldi temp, 0b00000001
    out PORTB, temp

    clr A
    clr B
    clr S
    clr F
    clr C_flag

    rcall MOSTRAR_SALIDA


MAIN_LOOP:
    rcall LEER_ENTRADAS
    rcall CALCULAR_ALU
    rcall MOSTRAR_SALIDA
    rjmp MAIN_LOOP


LEER_ENTRADAS:
    in temp, PINC
    com temp
    andi temp, 0x0F
    mov A, temp

    in temp, PIND
    com temp

    mov B, temp
    lsr B
    lsr B
    andi B, 0x0F

    clr S

    sbrc temp, 6
    ori S, 0b00000001

    sbrc temp, 7
    ori S, 0b00000010

    in aux, PINB
    com aux

    sbrc aux, 0
    ori S, 0b00000100

    ret

CALCULAR_ALU:
    cpi S, S_CLEAR
    breq DO_CLEAR

    cpi S, S_SUB
    breq DO_SUB

    cpi S, S_ADD
    breq DO_ADD

    cpi S, S_XOR
    breq DO_XOR

    cpi S, S_AND
    breq DO_AND

    cpi S, S_OR
    breq DO_OR

    cpi S, S_SHL
    breq DO_SHL

    rjmp DO_INC


DO_CLEAR:
    clr F
    clr C_flag
    ret


DO_SUB:
    mov F, A
    sub F, B

    brcc SUB_NO_CARRY

    ldi C_flag, 1
    rjmp SUB_FIN

SUB_NO_CARRY:
    clr C_flag

SUB_FIN:
    andi F, 0x0F
    ret
