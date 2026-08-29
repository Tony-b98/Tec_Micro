.include "m328Pdef.inc"

.def temp     = r16 ; Tomo registro 16 para almacenar variables
.def contador = r18, ;r18 como contador
.def aux      = r21 ; r21 y r22 registros para generar retardos
.def aux2     = r22

; Asigno pines a los pulsadores
.equ P_INC = 0   ; PB0
.equ P_DEC = 1   ; PB1
.equ P_RST = 2   ; PB2

.org 0x0000
    rjmp RESET
.org 0x0034

; Inicializo la pila, parte Low y parte HIGH a lo ultimo de la memoria RAM
RESET: 
    ldi temp, low(RAMEND)
    out SPL, temp
    ldi temp, high(RAMEND) 
    out SPH, temp

