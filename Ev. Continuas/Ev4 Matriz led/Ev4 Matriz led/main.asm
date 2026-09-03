.include "m328Pdef.inc"

; REGISTROS

.def temp        = r16
.def temp2       = r17
.def imagen      = r18
.def fila        = r19
.def patron      = r20
.def botones_ant = r21
.def botones     = r22
.def cambios     = r23
.def delay1      = r24
.def delay2      = r25

; BOTONES

.equ BTN_SIG = 2       ; PC2 = A2
.equ BTN_ANT = 3       ; PC3 = A3

; VECTOR RESET

.org 0x0000
    rjmp inicio

; CONFIGURACION

inicio:

    ; Configurar Stack Pointer
   
	ldi temp, low(RAMEND)
    out SPL, temp
	ldi temp, high(RAMEND)
    out SPH, temp

    ; PORTD -> 6 FILAS DE LA MATRIZ
   
    ldi temp, 0b1111_1100
    out DDRD, temp
	
	;Apagar filas PD2-PD7
	in temp, PORTD
	andi temp, 0b0000_0011
	out POTD, temp

    ; PORTB -> PB0 a PB5
    
    ldi temp, 0b0011_1111
    out DDRB, temp

    ldi temp, 0b0011_1111
    out PORTB, temp

    ; PORTC
    ; PC0-PC1 = columnas 7 y 8
    ; PC2-PC3 = botones
	;PC4 -PC5= filas 7 y 8
 
    ldi temp, 0b0011_0011
    out DDRC, temp

    ; Pull-up PC2 y PC3
	; Columas y filas apagadas
    ldi temp, 0b00001100
    out PORTC, temp

	; Primera imagen = sonrisa
    clr imagen

    ; Estado inicial de botones
    ldi botones_ant, 0b00001100

; PROGRAMA PRINCIPAL

MAIN:

    rcall MOSTRAR_IMAGEN
    rcall LEER_BOTONES
    rjmp MAIN

; MOSTRAR IMAGEN

MOSTRAR_IMAGEN:
    ; Elegir dibujo
    cpi imagen, 0
    breq CARGAR_SONRISA

    cpi imagen, 1
    breq CARGAR_CORAZON

    rjmp CARGAR_ASTERISCO

; CARITA SONRIENDO

CARGAR_SONRISA:

    ldi ZL, low(SONRISA*2)
    ldi ZH, high(SONRISA*2)

    rjmp INICIAR_MATRIZ
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;,;
;;;;;;;;;;;MODIFICACIONES CON NUEVA ASIGNACION DE PINES;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; CORAZON

CARGAR_CORAZON:

    ldi ZL, low(CORAZON*2)
    ldi ZH, high(CORAZON*2)

    rjmp INICIAR_MATRIZ


; ASTERISCO

CARGAR_ASTERISCO:

    ldi ZL, low(ASTERISCO*2)
    ldi ZH, high(ASTERISCO*2)

; BARRIDO DE LA MATRIZ

INICIAR_MATRIZ:

    ldi fila, 0b00000001
    ldi temp2, 8


BARRIDO:

    ; Apagar filas mientras cambiamos las columnas
    clr temp
    out PORTD, temp


    ; Leer una fila del dibujo
    lpm patron, Z+


    ; Los LED se encienden con columna en 0
    com patron

    ; COLUMNAS 1-6 -> PORTB
    
    mov temp, patron
    andi temp, 0b00111111
    out PORTB, temp

    ; COLUMNAS 7-8 -> PC0 y PC1
    
    mov temp, patron

    lsr temp
    lsr temp
    lsr temp
    lsr temp
    lsr temp
    lsr temp

    andi temp, 0b00000011

    ; Mantener pull-up de los botones
    ori temp, 0b00001100

    out PORTC, temp

    ; Activar fila
    
    out PORTD, fila


    rcall DELAY_FILA


    ; Pasar a siguiente fila
    lsl fila

    dec temp2
    brne BARRIDO


    ; Apagar matriz al terminar
    clr temp
    out PORTD, temp

    ret

; LEER BOTONES

LEER_BOTONES:

    in botones, PINC


    ; Detectar flanco 1 -> 0
    mov cambios, botones_ant

    mov temp, botones
    com temp

    and cambios, temp

    ; BOTON SIGUIENTE
    
    sbrc cambios, BTN_SIG
    rcall SIGUIENTE

    ; BOTON ANTERIOR
 
    sbrc cambios, BTN_ANT
    rcall ANTERIOR


    ; Guardar estado actual
    mov botones_ant, botones

    ret



; SIGUIENTE IMAGEN

SIGUIENTE:

    inc imagen

    cpi imagen, 3
    brlo FIN_SIG

    clr imagen

FIN_SIG:

    ret



; IMAGEN ANTERIOR

ANTERIOR:

    tst imagen
    brne RESTAR_IMAGEN

    ldi imagen, 2
    ret


RESTAR_IMAGEN:

    dec imagen
    ret



; DELAY PARA MULTIPLEXADO


DELAY_FILA:

    ldi delay1, 20

DELAY_EXT:

    ldi delay2, 200

DELAY_INT:

    dec delay2
    brne DELAY_INT

    dec delay1
    brne DELAY_EXT

    ret

; DIBUJOS 8x8

; MATRIZ CARITA SONRIENDO


SONRISA:

.db 0b00111100
.db 0b01000010
.db 0b10100101
.db 0b10000001
.db 0b10100101
.db 0b10011001
.db 0b01000010
.db 0b00111100

;MATRIZ CORAZON

CORAZON:

.db 0b00000000
.db 0b01100110
.db 0b11111111
.db 0b11111111
.db 0b01111110
.db 0b00111100
.db 0b00011000
.db 0b00000000

;MATRIZ ASTERISCO

ASTERISCO:

.db 0b00011000
.db 0b01011010
.db 0b00111100
.db 0b11111111
.db 0b00111100
.db 0b01011010
.db 0b00011000
.db 0b00000000