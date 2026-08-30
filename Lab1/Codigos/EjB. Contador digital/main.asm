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

; Incremento de contador
MANEJAR_INC:
    rcall DELAY_DEBOUNCE    ; Antirrebote inicial

    ; Confirma que el pulsador continúa presionado
    sbis PIND, BTN_INC
    rjmp INC_CONFIRMADO
    ret

INC_CONFIRMADO:
    ; Compara con el valor maximo para que no se supere.
    cpi contador, 9
    breq INC_ESPERA_SUELTA

    inc contador
    rcall ACTUALIZAR_DISPLAY

INC_ESPERA_SUELTA:
    ; Espera hasta que el usuario libere el pulsador
    sbis PIND, BTN_INC
    rjmp INC_ESPERA_SUELTA
	rcall DELAY_DEBOUNCE
    ret

; Decremento del contador
MANEJAR_DEC:
    rcall DELAY_DEBOUNCE    ; Antirrebote inicial

    ; Confirma que el pulsador continúa presionado
    sbis PIND, BTN_DEC
    rjmp DEC_CONFIRMADO
    ret

DEC_CONFIRMADO:
    ; Evita disminuir por debajo de cero
    cpi contador, 0
    breq DEC_ESPERA_SUELTA
	dec contador
    rcall ACTUALIZAR_DISPLAY

DEC_ESPERA_SUELTA:
    ; Espera hasta que el usuario libere el pulsador
    sbis PIND, BTN_DEC
    rjmp DEC_ESPERA_SUELTA
	rcall DELAY_DEBOUNCE
    ret



;definir etiqueta DELAY_DEBOUNCE,MANEJAR_RST Y ACTUALIZAR_DISPLAY



MANEJAR_RST:
    rcall DELAY_DEBOUNCE    

    ; Confirma que el pulsador continúa presionado
    sbis PIND, BTN_RST
    rjmp RST_CONFIRMADO
    ret

RST_CONFIRMADO:
    ; Reinicia el contador al valor inicial
    clr contador
    rcall ACTUALIZAR_DISPLAY

RST_ESPERA_SUELTA:
    ; Espera hasta que el usuario libere el pulsador
    sbis PIND, BTN_RST
    rjmp RST_ESPERA_SUELTA

    rcall DELAY_DEBOUNCE
    ret


ACTUALIZAR_DISPLAY:
    ; Guarda los registros utilizados por la rutina
    push temp
    push aux
    push ZL
    push ZH

    ; Z apunta al inicio de la tabla de patrones
    ldi ZL, low(TABLA_7SEG*2)
    ldi ZH, high(TABLA_7SEG*2)

    ; Desplaza el puntero según el valor actual del contador
    clr temp
    add ZL, contador
    adc ZH, temp

    ; Lee de memoria de programa el patrón correspondiente
    lpm temp, Z
    mov aux, temp

    ; Bits 0-5 controlan los segmentos A-F por PORTC
    andi temp, 0x3F
    out PORTC, temp

    ; Bit 6 controla el segmento G mediante PB0
    clr temp
    sbrc aux, 6
    ori temp, (1<<SEG_G)
    out PORTB, temp

    ; Recupera los registros
    pop ZH
    pop ZL
    pop aux
    pop temp
    ret
