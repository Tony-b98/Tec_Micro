.include "m328Pdef.inc"

; Definicion de registros
.def temp = r16
.def dato = r17

; RESET
.cseg
.org 0x0000
    rjmp RESET

RESET:
    ;Inicializo SP
    ldi temp, low(RAMEND)
    out SPL, temp

    ldi temp, high(RAMEND)
    out SPH, temp

    ; PB0..PB5 salidas (LED 0..5)
    ldi temp, 0b00111111
    out DDRB, temp

    ; PC0..PC1 salidas (LED 6..7)
    ldi temp, 0b00000011
    out DDRC, temp

    clr temp
    out PORTB, temp
    out PORTC, temp

    rcall USART_INIT

; MAIN
MAIN:
    ; Espera dato del micro 1
    rcall USART_RX

	andi dato, 0x07

	rcall APAGAR_LEDS