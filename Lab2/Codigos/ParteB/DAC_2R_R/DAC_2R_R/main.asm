
; Mapeo DAC R-2R:
;   bit0 -> PB0 (D8)
;   bit1 -> PB1 (D9)
;   bit2 -> PB2 (D10)
;   bit3 -> PB3 (D11)
;   bit4 -> PB4 (D12)
;   bit5 -> PB5 (D13)
;   bit6 -> PD6 (D6)
;   bit7 -> PD7 (D7)

.include "m328Pdef.inc"

; Constantes del sistema
.equ F_CPU      = 16000000 ; Frecuencia del Atmega328p
.equ BAUD       = 9600      ; Velocidad del puerto serial

; Valor necesario para configurar la velocidad de la UART
.equ UBRRVAL    = (F_CPU/16/BAUD)-1 

; Conf. Timmer1

.equ OCR1A_INIT = 200    ; Valor inicial
.equ OCR1A_MIN  = 20     ; Valor minimo
.equ OCR1A_MAX  = 2000   ; Valor maximo
.equ OCR1A_STEP = 20     ; Paso para aumentar o disminuir


; Registros
.def temp       = r16
.def dato       = r17
.def cero       = r18
.def rx_char    = r19
.def r_idx      = r20
.def r_baseL    = r22
.def r_baseH    = r23

; Interrupciones a utilizar
.cseg
.org 0x0000
        rjmp RESET

.org 0x0016
        rjmp TIMER1_COMPA_ISR

.org 0x0034

; PROGRAMA PRINCIPAL
RESET:
        ; Inicializar Stack Pointer
        ldi     temp, LOW(RAMEND)
        out     SPL, temp

        ldi     temp, HIGH(RAMEND)
        out     SPH, temp

        clr     cero
        clr     r_idx

        rcall   PORTS_INIT
        rcall   UART_INIT
        rcall   TIMER1_INIT

        ; Arranque por defecto con Señal 13
        rcall   SELECCIONAR_SIG13

        sei

        ; Mostrar menu
        ldi     ZL, LOW(MSG_MENU*2)
        ldi     ZH, HIGH(MSG_MENU*2)
        rcall   PRINT_STRING

		; BUCLE PRINCIPAL
MAIN_LOOP:

        ; Consultar si llego un byte por UART
        lds     temp, UCSR0A

        sbrs    temp, RXC0
        rjmp    MAIN_LOOP

        lds     rx_char, UDR0


        ; Seleccionar Senal 13
        cpi     rx_char, '1'
        breq    CMD_SIG13


        ; Seleccionar Senal 15
        cpi     rx_char, '2'
        breq    CMD_SIG15


        ; Aumentar frecuencia
        cpi     rx_char, '+'
        breq    CMD_MAS_RAPIDO


        ; Disminuir frecuencia
        cpi     rx_char, '-'
        breq    CMD_MAS_LENTO


        ; Cualquier otro caracter se ignora
        rjmp    MAIN_LOOP
