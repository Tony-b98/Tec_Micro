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
	out PORTD, temp

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

; INICIAR MATRIZ

INICIAR_MATRIZ:

    ; fila = número de fila, 0 a 7
    clr fila

    ; 8 filas
    ldi temp2, 8

; BARRIDO

BARRIDO:
    ; Apagar todas las filas
    rcall APAGAR_FILAS

    ; Leer fila de dibujo
    lpm patron, Z+

   ; Los LEDs se encienden con columna en 0
    com patron

    ; COLUMNAS 1-6 -> PORTB
    mov temp, patron
    andi temp, 0b00111111
    out PORTB, temp

    ; COLUMNAS 7-8 -> PC0-PC1
    mov temp, patron
    lsr temp
    lsr temp
    lsr temp
    lsr temp
    lsr temp
    lsr temp
	andi temp, 0b00000011

    ; Mantener pull-up de PC2 y PC3

    ori temp, 0b00001100

    ; Las filas PC4-PC5 todavía están apagadas

    out PORTC, temp
		
    ; Activar fila correspondiente
    rcall ACTIVAR_FILA
	
    ; Tiempo de encendido
    rcall DELAY_FILA

	; Siguiente fila
    
	inc fila
	dec temp2
    brne BARRIDO

	; Apagar matriz
    rcall APAGAR_FILAS
	ret



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;,;
;SE AGREGAN RUTINAS DE APAGAR Y ACTIVAR FILAS
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;,;


;RUTINA APAGAR FILAS
; Desactiva todas las filas de la matriz antes
; de encender la siguiente durante el multiplexado.
APAGAR_FILAS:
    ; PORTD
    ; Apagar PD2-PD7
    ; Mantener PD0-PD1 intactos
    in temp, PORTD
    andi temp, 0b00000011
    out PORTD, temp

    ; PORTC
    ; Apagar PC4-PC5
    ; Mantener PC0-PC3
    in temp, PORTC
    andi temp, 0b00001111
    out PORTC, temp
    ret

;RUTINA ACTIVAR FILA
; Activa una única fila de la matriz según
; el valor almacenado en el registro "fila".
ACTIVAR_FILA:
 ; fila = 0 -> Fila 1 -> PD2
    cpi fila, 0
    breq FILA_1
; fila = 1 -> Fila 2 -> PD3
    cpi fila, 1
    breq FILA_2
; fila = 2 -> Fila 3 -> PD4
    cpi fila, 2
    breq FILA_3
; fila = 3 -> Fila 4 -> PD5
    cpi fila, 3
    breq FILA_4
 ; fila = 4 -> Fila 5 -> PD6
    cpi fila, 4
    breq FILA_5
 ; fila = 5 -> Fila 6 -> PD7
    cpi fila, 5
    breq FILA_6
; fila = 6 -> Fila 7 -> PC4
    cpi fila, 6
    breq FILA_7

; fila = 7 -> Fila 8 -> PC5    
	sbi PORTC, 5
    ret

; ACTIVACION INDIVIDUAL DE FILAS
; Cada rutina pone en nivel alto el pin
; correspondiente a una fila de la matriz.
FILA_1:
;Activar fila 1 
    sbi PORTD, 2
    ret

FILA_2:
;Activar fila 2 
    sbi PORTD, 3
    ret
 
FILA_3:
;Activar fila 3
    sbi PORTD, 4
    ret

FILA_4:
;Activar fila 4 
    sbi PORTD, 5
    ret

FILA_5:
;Activar fila 5
    sbi PORTD, 6
    ret

FILA_6:
;Activar fila 6 
    sbi PORTD, 7
    ret

FILA_7:
;Activar fila 7 
    sbi PORTC, 4
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