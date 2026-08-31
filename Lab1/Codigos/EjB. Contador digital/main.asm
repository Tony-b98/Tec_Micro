.include "m328Pdef.inc"

.cseg

; DEFINICIÓN DE REGISTROS

.def temp     = r16 ;Registro temporal de trabajo
.def contador = r18 ;Almacena el valor actual del contador
.def aux      = r21 ;Registros auxiliares utilizados en retardos
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

;REINICIO DEL CONTADOR
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

;ACTUALIZACIÓN DEL DISPLAY DE 7 SEG.
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

;SUBRUTINA DE RETARDO PARA ANTIRREBOTE
DELAY_DEBOUNCE:
    push aux
    push aux2
    push aux3

    ; Retardo por software mediante bucles anidados
    ldi aux3, 2

DD_PASS:
;Carga el contador externo con su valor máximo
    ldi aux, 255

DD_OUTER:
;Carga el contador interno con su valor máximo
    ldi aux2, 255

DD_INNER:
;Decrementa el contador interno hasta llegar a 0
    dec aux2
    brne DD_INNER

;Decrementa el contador externo y repite el bucle
    dec aux
    brne DD_OUTER

;Repite el conjunto completo de retardos
    dec aux3
    brne DD_PASS

;Recupera los registros aux. guardados
    pop aux3
    pop aux2
    pop aux
    ret

;TABLA DE PATRONES DEL DISPLAY PARA LOS NÚMEROS DEL 0 AL 9
TABLA_7SEG:
   .db 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F