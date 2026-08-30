.include "m328Pdef.inc"

.cseg

; DEFINICIÓN DE REGISTROS

.def temp     = r16 ; Tomo registro 16 para almacenar variables
.def contador = r18 ;r18 como contador
.def aux      = r21 ;registros para generar retardos
.def aux2     = r22
.def aux3     = r23

; DEFINICIÓN DE PINES

.equ BTN_INC = 2  ; Pulsador incrementar
.equ BTN_DEC = 3  ; Pulsador decrementar
.equ BTN_RST = 4  ; Pulsador reset
.equ SEG_G   = 0  ; Segmento G

; VECTOR DE RESET

.org 0x0000
    rjmp RESET

; CONFIGURACIÓN INICIAL


RESET:       
    cli                      ; Deshabilita interrupciones

    ; Inicialización del puntero de pila
	ldi temp, low(RAMEND)
    out SPL, temp
    ldi temp, high(RAMEND) 
    out SPH, temp

	; PC0-PC5 como salidas para los segmentos A-F
	ldi temp, 0x3F
    out DDRC, temp

    ; PB0 como salida para el segmento G
	ldi temp, (1<<SEG_G)
    out DDRB, temp

	; PORTD configurado como entrada
    clr temp
    out DDRD, temp

	 ; Activación de resistencias pull-up internas en los pulsadores
    ldi temp, (1<<BTN_INC) | (1<<BTN_DEC) | (1<<BTN_RST)
    out PORTD, temp

	 ; Inicializa las salidas del display en cero
    clr temp
    out PORTC, temp
    out PORTB, temp

	; El contador inicia en cero
    clr contador
    rcall ACTUALIZAR_DISPLAY

; BUCLE PRINCIPAL

; Comprueba continuamente el estado de los tres pulsadores
MAIN_LOOP:
    sbis PIND, BTN_INC
    rcall MANEJAR_INC

    sbis PIND, BTN_DEC
    rcall MANEJAR_DEC

    sbis PIND, BTN_RST
    rcall MANEJAR_RST

    rjmp MAIN_LOOP


