
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

; COMANDOS UART

CMD_SIG13:

        rcall   SELECCIONAR_SIG13

        ldi     ZL, LOW(MSG_SIG13*2)
        ldi     ZH, HIGH(MSG_SIG13*2)

        rcall   PRINT_STRING

        rjmp    MAIN_LOOP


CMD_SIG15:

        rcall   SELECCIONAR_SIG15

        ldi     ZL, LOW(MSG_SIG15*2)
        ldi     ZH, HIGH(MSG_SIG15*2)

        rcall   PRINT_STRING

        rjmp    MAIN_LOOP


CMD_MAS_RAPIDO:

        rcall   TIMER1_MAS_RAPIDO

        rjmp    MAIN_LOOP


CMD_MAS_LENTO:

        rcall   TIMER1_MAS_LENTO

        rjmp    MAIN_LOOP


SELECCIONAR_SIG13:

        ; Guardar estado previo de interrupciones
        in      temp, SREG

        cli

        ; Direccion inicial de la tabla
        ldi     r_baseL, LOW(SIG13*2)
        ldi     r_baseH, HIGH(SIG13*2)

        ; Empezar desde muestra 0
        clr     r_idx

        ; Restaurar estado previo
        out     SREG, temp

        ret


SELECCIONAR_SIG15:

        in      temp, SREG

        cli

        ; Direccion inicial de la tabla
        ldi     r_baseL, LOW(SIG15*2)
        ldi     r_baseH, HIGH(SIG15*2)

        ; Empezar desde muestra 0
        clr     r_idx

        out     SREG, temp

        ret

; INICIALIZACION DE PUERTOS

PORTS_INIT:
        ; PORTB: PB0 -> bit 0 DAC / PB1 -> bit 1 DAC
        ;        PB2 -> bit 2 DAC / PB3 -> bit 3 DAC
        ;        PB4 -> bit 4 DAC / PB5 -> bit 5 DAC
	    ldi     temp, 0x3F
        out     DDRB, temp
				
        ; PORTD
        ; PD6 -> bit 6 DAC / PD7 -> bit 7 DAC
        ldi     temp, 0xC0
        out     DDRD, temp

        ; DAC inicialmente en 0
        clr     temp
        out     PORTB, temp
        out     PORTD, temp
        ret
UART_INIT:

        ; Baud rate
        ldi     temp, HIGH(UBRRVAL)
        sts     UBRR0H, temp

        ldi     temp, LOW(UBRRVAL)
        sts     UBRR0L, temp


        ; Habilitar RX y TX
        ldi     temp, (1<<RXEN0)|(1<<TXEN0)
        sts     UCSR0B, temp


        ; 8 bits de datos
        ; sin paridad
        ; 1 bit de stop
        ldi     temp, (1<<UCSZ01)|(1<<UCSZ00)
        sts     UCSR0C, temp

        ret

; Transmición UART
UART_TX:
        ; Leer estado UART
        lds     r0, UCSR0A
		; Esperar hasta que el buffer este libre
        sbrs    r0, UDRE0
        rjmp    UART_TX
		; Transmitir
        sts     UDR0, temp
		ret
; Imprimir string
; Z apunta al comienzo del string en Flash.
PRINT_STRING:
        lpm     temp, Z+
        cpi     temp, 0
        breq    PRINT_STRING_FIN
        rcall   UART_TX
        rjmp    PRINT_STRING

PRINT_STRING_FIN:
        ret
		

TIMER1_INIT:
        ; Detener Timer1 y configurar modo CTC
        clr     temp
        sts     TCCR1A, temp

        ldi     temp, (1<<WGM12)
        sts     TCCR1B, temp

        ldi     temp, HIGH(OCR1A_INIT)
        sts     OCR1AH, temp

        ldi     temp, LOW(OCR1A_INIT)
        sts     OCR1AL, temp

        ; TCNT1 = 0
        clr     temp

        sts     TCNT1H, temp
        sts     TCNT1L, temp

        ; Limpiar posible bandera pendiente
        ldi     temp, (1<<OCF1A)
        sts     TIFR1, temp

        ; Habilitar interrupcion Compare Match A
        ldi     temp, (1<<OCIE1A)
        sts     TIMSK1, temp

        ; Arrancar Timer1
        ldi     temp, (1<<WGM12)|(1<<CS11)
        sts     TCCR1B, temp

        ret

		; AUMENTAR FRECUENCIA
; Menor OCR1A = mayor frecuencia.
TIMER1_MAS_RAPIDO:
        ; Leer OCR1A
        ; Para lectura:
        ; primero LOW
        ; despues HIGH
        
        lds     ZL, OCR1AL
        lds     ZH, OCR1AH

        ; OCR1A = OCR1A - 20
        subi    ZL, LOW(OCR1A_STEP)
        sbci    ZH, HIGH(OCR1A_STEP)

        ; Verificar limite minimo
        cpi     ZL, LOW(OCR1A_MIN)

        ldi     temp, HIGH(OCR1A_MIN)
        cpc     ZH, temp

        brge    TMR_SET_RAPIDO

        ; Si pasa el limite:
        ; OCR1A = OCR1A_MIN
        ldi     ZL, LOW(OCR1A_MIN)
        ldi     ZH, HIGH(OCR1A_MIN)


TMR_SET_RAPIDO:

        ; Escribir HIGH primero
        sts     OCR1AH, ZH

        ; Escribir LOW despues
        sts     OCR1AL, ZL

        ret

