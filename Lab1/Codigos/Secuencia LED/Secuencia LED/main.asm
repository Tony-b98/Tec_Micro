.include "m328Pdef.inc"

.def temp      = r16
.def secuencia = r17
.def patron    = r18
.def aux       = r21
.def aux2      = r22
.def aux3      = r23

.equ BTN_SIG    = 2
.equ BTN_ANT    = 3
.equ BTN_INICIO = 4

.org 0x0000
    rjmp RESET

RESET:

    ldi temp, low(RAMEND)
    out SPL, temp

    ldi temp, high(RAMEND)
    out SPH, temp


    ldi temp, 0b11111100
    out DDRD, temp

    ldi temp, 0b00000011
    out DDRB, temp


    ldi temp, 0b00011100
    out PORTB, temp


    clr secuencia
    rcall REINICIAR_SECUENCIA
