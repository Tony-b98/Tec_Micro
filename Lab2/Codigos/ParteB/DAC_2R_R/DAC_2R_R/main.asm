
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

; DISMINUIR FRECUENCIA
; Mayor OCR1A = menor frecuencia.

TIMER1_MAS_LENTO:

        ; Leer OCR1A
        lds     ZL, OCR1AL
        lds     ZH, OCR1AH

        ; OCR1A = OCR1A + 20

        subi    ZL, LOW(-OCR1A_STEP)
        sbci    ZH, HIGH(-OCR1A_STEP)

        ; Verificar limite maximo

        cpi     ZL, LOW(OCR1A_MAX)

        ldi     temp, HIGH(OCR1A_MAX)
        cpc     ZH, temp

        brlo    TMR_SET_LENTO


        ; Si supera OCR1A_MAX
        ldi     ZL, LOW(OCR1A_MAX)
        ldi     ZH, HIGH(OCR1A_MAX)


TMR_SET_LENTO:

        ; Escribir HIGH primero
        sts     OCR1AH, ZH

        ; Escribir LOW despues
        sts     OCR1AL, ZL

        ret

; INTERRUPCION TIMER1 COMPARE MATCH A
TIMER1_COMPA_ISR:

        ; Guardar registros utilizados
        push    temp
        push    dato
        push    ZL
        push    ZH
        ; Guardar SREG
        in      temp, SREG
        push    temp

        ; Z = direccion base + indice

        mov     ZL, r_baseL
        mov     ZH, r_baseH

        add     ZL, r_idx
        adc     ZH, cero
    
	; Leer muestra desde Flash
        lpm     dato, Z


        ; Bits 0 a 5 -> PORTB
        mov     temp, dato

        andi    temp, 0x3F

        out     PORTB, temp

        ; Bits 6 y 7 -> PORTD
        mov     temp, dato

        andi    temp, 0xC0

        out     PORTD, temp

        inc     r_idx

        ; Restaurar contexto
        pop     temp
        out     SREG, temp

        pop     ZH
        pop     ZL
        pop     dato
        pop     temp

        reti

; MENSAJES UART
MSG_MENU:

        .db "\r\n--- DAC R-2R 8 bits - Grupo 1 ---\r\n1: Senal 13\r\n2: Senal 15\r\n+ : mas rapido   - : mas lento\r\n> ",0


MSG_SIG13:
    .db "\r\nMostrando Senal 13\r\n> ",0,0

MSG_SIG15:
    .db "\r\nMostrando Senal 15\r\n> ",0,0

SIG13:

        .db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01
        .db 0x01,0x01,0x01,0x01,0x02,0x02,0x02,0x02,0x03,0x03,0x03,0x03,0x04,0x04,0x05,0x05
        .db 0x05,0x06,0x06,0x07,0x08,0x08,0x09,0x0a,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,0x10,0x11
        .db 0x12,0x13,0x15,0x16,0x18,0x19,0x1b,0x1c,0x1e,0x20,0x22,0x24,0x26,0x28,0x2b,0x2d
        .db 0x2f,0x32,0x35,0x37,0x3a,0x3d,0x40,0x43,0x46,0x4a,0x4d,0x51,0x54,0x58,0x5b,0x5f
        .db 0x63,0x67,0x6b,0x6f,0x73,0x77,0x7c,0x80,0x84,0x89,0x8d,0x91,0x96,0x9a,0x9f,0xa3
        .db 0xa7,0xac,0xb0,0xb4,0xb9,0xbd,0xc1,0xc5,0xc9,0xcd,0xd1,0xd4,0xd8,0xdc,0xdf,0xe2
        .db 0xe5,0xe8,0xeb,0xee,0xf0,0xf2,0xf4,0xf6,0xf8,0xf9,0xfb,0xfc,0xfd,0xfe,0xfe,0xfe
        .db 0xff,0xfe,0xfe,0xfe,0xfd,0xfc,0xfb,0xf9,0xf8,0xf6,0xf4,0xf2,0xf0,0xee,0xeb,0xe8
        .db 0xe5,0xe2,0xdf,0xdc,0xd8,0xd4,0xd1,0xcd,0xc9,0xc5,0xc1,0xbd,0xb9,0xb4,0xb0,0xac
        .db 0xa7,0xa3,0x9f,0x9a,0x96,0x91,0x8d,0x89,0x84,0x80,0x7c,0x77,0x73,0x6f,0x6b,0x67
        .db 0x63,0x5f,0x5b,0x58,0x54,0x51,0x4d,0x4a,0x46,0x43,0x40,0x3d,0x3a,0x37,0x35,0x32
        .db 0x2f,0x2d,0x2b,0x28,0x26,0x24,0x22,0x20,0x1e,0x1c,0x1b,0x19,0x18,0x16,0x15,0x13
        .db 0x12,0x11,0x10,0x0f,0x0e,0x0d,0x0c,0x0b,0x0a,0x0a,0x09,0x08,0x08,0x07,0x06,0x06
        .db 0x05,0x05,0x05,0x04,0x04,0x03,0x03,0x03,0x03,0x02,0x02,0x02,0x02,0x01,0x01,0x01
        .db 0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00

SIG15:

        .db 0x00,0x06,0x0c,0x12,0x18,0x1f,0x25,0x2b,0x31,0x37,0x3d,0x44,0x4a,0x4f,0x55,0x5b
        .db 0x61,0x67,0x6d,0x72,0x78,0x7d,0x83,0x88,0x8d,0x92,0x97,0x9c,0xa1,0xa6,0xab,0xaf
        .db 0xb4,0xb8,0xbc,0xc1,0xc5,0xc9,0xcc,0xd0,0xd4,0xd7,0xda,0xdd,0xe0,0xe3,0xe6,0xe9
        .db 0xeb,0xed,0xf0,0xf2,0xf4,0xf5,0xf7,0xf8,0xfa,0xfb,0xfc,0xfd,0xfd,0xfe,0xfe,0xfe
        .db 0xff,0xfe,0xfe,0xfe,0xfd,0xfd,0xfc,0xfb,0xfa,0xf8,0xf7,0xf5,0xf4,0xf2,0xf0,0xed
        .db 0xeb,0xe9,0xe6,0xe3,0xe0,0xdd,0xda,0xd7,0xd4,0xd0,0xcc,0xc9,0xc5,0xc1,0xbc,0xb8
        .db 0xb4,0xaf,0xab,0xa6,0xa1,0x9c,0x97,0x92,0x8d,0x88,0x83,0x7d,0x78,0x72,0x6d,0x67
        .db 0x61,0x5b,0x55,0x4f,0x4a,0x44,0x3d,0x37,0x31,0x2b,0x25,0x1f,0x18,0x12,0x0c,0x06

        .db 0x00,0x06,0x0c,0x12,0x18,0x1f,0x25,0x2b,0x31,0x37,0x3d,0x44,0x4a,0x4f,0x55,0x5b
        .db 0x61,0x67,0x6d,0x72,0x78,0x7d,0x83,0x88,0x8d,0x92,0x97,0x9c,0xa1,0xa6,0xab,0xaf
        .db 0xb4,0xb8,0xbc,0xc1,0xc5,0xc9,0xcc,0xd0,0xd4,0xd7,0xda,0xdd,0xe0,0xe3,0xe6,0xe9
        .db 0xeb,0xed,0xf0,0xf2,0xf4,0xf5,0xf7,0xf8,0xfa,0xfb,0xfc,0xfd,0xfd,0xfe,0xfe,0xfe
        .db 0xff,0xfe,0xfe,0xfe,0xfd,0xfd,0xfc,0xfb,0xfa,0xf8,0xf7,0xf5,0xf4,0xf2,0xf0,0xed
        .db 0xeb,0xe9,0xe6,0xe3,0xe0,0xdd,0xda,0xd7,0xd4,0xd0,0xcc,0xc9,0xc5,0xc1,0xbc,0xb8
        .db 0xb4,0xaf,0xab,0xa6,0xa1,0x9c,0x97,0x92,0x8d,0x88,0x83,0x7d,0x78,0x72,0x6d,0x67
        .db 0x61,0x5b,0x55,0x4f,0x4a,0x44,0x3d,0x37,0x31,0x2b,0x25,0x1f,0x18,0x12,0x0c,0x06
