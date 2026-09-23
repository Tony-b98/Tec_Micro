.include "m328Pdef.inc"

; Definicion de registros de trabajo
.def temp        = r16
.def temp2       = r17
.def imagen      = r18     
.def fila        = r19
.def patron      = r20
.def delay1      = r24
.def delay2      = r25
.def zero        = r0       ; constante 0 
.def col_idx     = r2       ; indice de la columna actual del mensaje
.def tickcnt     = r3       ; cuenta regresiva para el paso de scroll
.def scrolldly   = r4       ; velocidad de scroll (mas alto = mas lento)
.def newcol      = r5       ; columna nueva leida de la fuente
.def uartc       = r6       ; ultimo caracter recibido por UART
.def rowcnt      = r7       ; contador auxiliar de filas

; BAUD RATE
.equ UBRR_VAL = 103
; Paso de velocidad de scroll por cada '+'/'-' recibido por UART
.equ SCROLL_MIN  = 5
.equ SCROLL_MAX  = 60
.equ SCROLL_STEP = 5

; MEMORIA DE DATOS (RAM)
.dseg
FRAME: .byte 8
.cseg

; VECTOR RESET
.org 0x0000
    rjmp inicio

inicio:
    ; Configurar SP
    ldi temp, low(RAMEND)
    out SPL, temp
    ldi temp, high(RAMEND)
    out SPH, temp
	
	; PORTD -> 6 FILAS DE LA MATRIZ
    ldi temp, 0b1111_1100
    out DDRD, temp

    ; Apagar filas PD2-PD7
    in temp, PORTD
    andi temp, 0b0000_0011
    out PORTD, temp

    ; PORTB -> PB0 a PB5
    ldi temp, 0b0011_1111
    out DDRB, temp
    ldi temp, 0b0011_1111
    out PORTB, temp

    ; PORTC
    ; PC0-PC1 = columnas 7 y 8 // PC4-PC5 = filas 7 y 8
    ldi temp, 0b0011_0011
    out DDRC, temp
