.include "m328Pdef.inc"

; Definición de sinonimos para registros de trabajo
.def temp      = r16
.def secuencia = r17
.def patron    = r18
.def aux       = r21
.def aux2      = r22
.def aux3      = r23

;Asignación de pines
.equ BTN_SIG    = 2
.equ BTN_ANT    = 3
.equ BTN_INICIO = 4

; Vector de reset
.org 0x0000
    rjmp RESET

RESET:
	;Inicio de SP en el ultimo lugar de la RAM
    ldi temp, low(RAMEND)
    out SPL, temp
	ldi temp, high(RAMEND)
    out SPH, temp


    ldi temp, 0xfc
    out DDRD, temp       ; Conf de los bit como salida en el registro DDRD para los LED (0 - 5)

    ldi temp, 0x03 ; Agrego los 2 led como salidas en el PB0 y PB1 el resto de los bits quedan como entradas
    out DDRB, temp

	ldi temp, 0x1c	; Se activa pull-up en los bit de entrada del puerto B
    out PORTB, temp


	clr secuencia
    rcall REINICIAR_SECUENCIA

MAIN_LOOP:
    rcall CHECK_BOTONES
    rcall MOSTRAR_PATRON
    rcall DELAY_PASO
    rcall EJECUTAR_PASO
    rjmp MAIN_LOOP


CHECK_BOTONES:

    ; Boton siguiente
    sbis PINB, BTN_SIG
    rcall MANEJAR_SIGUIENTE

    ; Boton anterior
    sbis PINB, BTN_ANT
    rcall MANEJAR_ANTERIOR

    ; Boton inicio
    sbis PINB, BTN_INICIO
    rcall MANEJAR_INICIO
	ret

MANEJAR_SIGUIENTE:
    rcall DELAY_DEBOUNCE
	; Confirmar que sigue presionado
    sbis PINB, BTN_SIG
    rjmp SIG_CONFIRMADO
	ret

SIG_CONFIRMADO:
    inc secuencia
	; Compara el registro hasta que alcance la ultima secuencia
    cpi secuencia, 8
    brne SIG_OK
	; Si llega a 8 vuelve a 0
    clr secuencia

SIG_OK:
    rcall REINICIAR_SECUENCIA

SIG_ESPERA:
    ; Esperar a que se suelte el boton
    sbis PINB, BTN_SIG
    rjmp SIG_ESPERA
	rcall DELAY_DEBOUNCE
    ret

MANEJAR_ANTERIOR:
    rcall DELAY_DEBOUNCE
	; Confirmar pulsacion
    sbis PINB, BTN_ANT
    rjmp ANT_CONFIRMADO
	ret

ANT_CONFIRMADO:
	; Si estamos en 0, pasar a 7
    cpi secuencia, 0
    brne ANT_DEC
	ldi secuencia, 8

ANT_DEC:
    dec secuencia
	rcall REINICIAR_SECUENCIA

ANT_ESPERA:
    ; Esperar a soltar el boton
    sbis PINB, BTN_ANT
    rjmp ANT_ESPERA
	rcall DELAY_DEBOUNCE
    ret

MANEJAR_INICIO:
    rcall DELAY_DEBOUNCE
	; Confirmar pulsacion
    sbis PINB, BTN_INICIO
    rjmp INICIO_CONFIRMADO
	ret

INICIO_CONFIRMADO:
	; Volver a la primera secuencia
    clr secuencia
	rcall REINICIAR_SECUENCIA

INICIO_ESPERA:
    sbis PINB, BTN_INICIO
    rjmp INICIO_ESPERA
	rcall DELAY_DEBOUNCE
    ret

REINICIAR_SECUENCIA:
	; Secuencia 0
	cpi secuencia, 0
	brne RS_1
	ldi patron, 0x01
	rjmp RS_OUT

RS_1:
	cpi secuencia, 1
	brne RS_2
	ldi patron, 0x80
    rjmp RS_OUT

RS_2:
	cpi secuencia, 2
    brne RS_3
	ldi patron, 0xAA
    rjmp RS_OUT

RS_3:
	cpi secuencia, 3
    brne RS_4
	clr patron
    rjmp RS_OUT

RS_4:
	cpi secuencia, 4
    brne RS_5
	ldi patron, 0x11
    rjmp RS_OUT

RS_5:
	cpi secuencia, 5
    brne RS_6
	ldi patron, 0x03
    rjmp RS_OUT

RS_6:
    cpi secuencia, 6
    brne RS_7
	clr patron
    rjmp RS_OUT

RS_7:
    ldi patron, 0x0F

RS_OUT:
    rcall MOSTRAR_PATRON
    ret

