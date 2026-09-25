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

    ; Enviar valor 0...7 por USART
    ; Los bits transmitidos quedan:
    ; 00000000 -> 0
    ; 00000001 -> 1
    ; 00000010 -> 2
    ; ...
    ; 00000111 -> 7
   
    rcall USART_TX


    rjmp MAIN
