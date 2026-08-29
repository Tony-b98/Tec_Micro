.include "m328Pdef.inc"

.def temp      = r16
.def secuencia = r17
.def patron    = r18
.def aux       = r21
.def aux2      = r22
.def aux3      = r23

.equ BTN_SIG    = 2
.equ BTN_ANT    = 3
.equ BTN_INICIO = 4

.org 0x0000
    rjmp RESET

RESET:

    ldi temp, low(RAMEND)
    out SPL, temp

    ldi temp, high(RAMEND)
    out SPH, temp


    ldi temp, 0b11111100
    out DDRD, temp

    ldi temp, 0b00000011
    out DDRB, temp


    ldi temp, 0b00011100
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

    ; Ahora hay 8 secuencias: 0,1,2,3,4,5,6,7
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
