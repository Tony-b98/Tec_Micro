; MICROCONTROLADOR 1 - TRANSMISOR USART
; Entradas:
; PC0 / A0 -> Bit 0
; PC1 / A1 -> Bit 1
; PC2 / A2 -> Bit 2
;
; Salida USART:
; PD1 / TX -> RX del segundo microcontrolador
; Los pulsadores utilizan resistencias pull-up internas.
; Pulsado = 1 logico luego de invertir la lectura.

.include "m328Pdef.inc"

; Registros
.def temp      = r16
.def dato      = r17
.def anterior  = r18
.def delay1    = r19
.def delay2    = r20

; VECTOR DE RESET
.cseg
.org 0x0000
    rjmp RESET

RESET:

    ; Inicializar Stack Pointer
    ldi temp, low(RAMEND)
    out SPL, temp

    ldi temp, high(RAMEND)
    out SPH, temp

    ; PC0, PC1 y PC2 como entradas
    cbi DDRC, 0
    cbi DDRC, 1
    cbi DDRC, 2

    ; Activar pull-up internas
    sbi PORTC, 0
    sbi PORTC, 1
    sbi PORTC, 2


    ; Inicializar USART
    rcall USART_INIT

    ; Forzar primera transmision
    ldi anterior, 0xFF

; PROGRAMA PRINCIPAL
MAIN:
    ; Leer PC0, PC1 y PC2
    in dato, PINC

    ; Como usamos pull-up:
    ; pulsador libre   = 1
    ; pulsador pulsado = 0
  
    ; Invertimos para obtener:
    
    ; pulsador libre   = 0
    ; pulsador pulsado = 1
    com dato

    ; Conservar solamente los 3 bits inferiores
    ; dato:
    ; 000 = 0
    ; 001 = 1
    ; 010 = 2
    ; 011 = 3
    ; 100 = 4
    ; 101 = 5
    ; 110 = 6
    ; 111 = 7
    andi dato, 0x07

    ; Comprobar si el valor cambio
    cp dato, anterior

    breq MAIN

    ; Guardar nuevo valor
    mov anterior, dato

    ; Pequeno antirrebote
    rcall DELAY_20MS

    ; Leer nuevamente despues del antirrebote
    in dato, PINC
    com dato
    andi dato, 0x07

    ; Actualizar valor definitivo
    mov anterior, dato
    rcall USART_TX
	rjmp MAIN

; Inicializacion USART
USART_INIT:

    ; Baud rate = 9600
    ldi temp, high(103)
    sts UBRR0H, temp

    ldi temp, low(103)
    sts UBRR0L, temp
    ; U2X0 = 0
    ldi temp, 0x00
    sts UCSR0A, temp

    ; Habilitar solamente transmisor
    ldi temp, (1<<TXEN0)
    sts UCSR0B, temp

    ; 8 bits, sin paridad, 1 stop
    ldi temp, (1<<UCSZ01) | (1<<UCSZ00)
    sts UCSR0C, temp
    ret

; USART_TX
; Entrada: dato = byte a transmitir
USART_TX:

USART_TX_WAIT:
    ; Leer estado USART
    lds temp, UCSR0A
    ; Esperar buffer disponible
    sbrs temp, UDRE0
    rjmp USART_TX_WAIT
    ; Transmitir
    sts UDR0, dato
    ret

;Delay para evitar rebote
DELAY_20MS:

    ldi delay1, 100

DELAY_20MS_1:

    ldi delay2, 255

DELAY_20MS_2:

    dec delay2
    brne DELAY_20MS_2

    dec delay1
    brne DELAY_20MS_1

    ret
