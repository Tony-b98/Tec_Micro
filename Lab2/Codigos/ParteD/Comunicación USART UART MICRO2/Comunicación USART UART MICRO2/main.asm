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

    ; Seleccion de LED correspondiente
    
    cpi dato, 0
    breq LED_0

    cpi dato, 1
    breq LED_1

    cpi dato, 2
    breq LED_2

    cpi dato, 3
    breq LED_3

    cpi dato, 4
    breq LED_4

    cpi dato, 5
    breq LED_5

    cpi dato, 6
    breq LED_6

    cpi dato, 7
    breq LED_7

    rjmp MAIN


; LED 0 -> D8 / PB0
LED_0:
    sbi PORTB, 0
    rjmp MAIN

; LED 1 -> D9 / PB1
LED_1:
    sbi PORTB, 1
    rjmp MAIN

; LED 2 -> D10 / PB2
LED_2:
    sbi PORTB, 2
    rjmp MAIN

; LED 3 -> D11 / PB3
LED_3:
    sbi PORTB, 3
    rjmp MAIN

; LED 4 -> D12 / PB4
LED_4:
    sbi PORTB, 4
    rjmp MAIN

; LED 5 -> D13 / PB5
LED_5:
    sbi PORTB, 5
    rjmp MAIN

; LED 6 -> A0 / PC0
LED_6:
    sbi PORTC, 0
    rjmp MAIN

; LED 7 -> A1 / PC1
LED_7:
    sbi PORTC, 1
    rjmp MAIN

; APAGAR TODOS LOS LEDs
APAGAR_LEDS:

    clr temp

    out PORTB, temp
    out PORTC, temp

    ret

; INICIALIZACION USART
USART_INIT:

    ldi temp, high(103)
    sts UBRR0H, temp

    ldi temp, low(103)
    sts UBRR0L, temp

    ; Velocidad normal
    clr temp
    sts UCSR0A, temp

    ; Habilitar solamente receptor USART
    ldi temp, (1<<RXEN0)
    sts UCSR0B, temp

    ; 8 bits
    ; sin paridad
    ; 1 bit de stop
    ldi temp, (1<<UCSZ01) | (1<<UCSZ00)
    sts UCSR0C, temp
    ret

; USART_RX
; Espera hasta recibir un byte.
; Salida: dato = byte recibido

USART_RX:
USART_RX_WAIT:
    ; Leer estado USART
    lds temp, UCSR0A

    ; Esperar RXC0 = 1
    sbrs temp, RXC0
    rjmp USART_RX_WAIT

    ; Leer byte recibido
    lds dato, UDR0
	ret

