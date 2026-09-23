.include "m328Pdef.inc"

; Definicion de registros de trabajo
.def temp        = r16
.def temp2       = r17
.def imagen      = r18     
.def fila        = r19
.def patron      = r20
.def delay1      = r24
.def delay2      = r25
.def zero        = r0       ; constante 0 
.def col_idx     = r2       ; indice de la columna actual del mensaje
.def tickcnt     = r3       ; cuenta regresiva para el paso de scroll
.def scrolldly   = r4       ; velocidad de scroll (mas alto = mas lento)
.def newcol      = r5       ; columna nueva leida de la fuente
.def uartc       = r6       ; ultimo caracter recibido por UART
.def rowcnt      = r7       ; contador auxiliar de filas

; BAUD RATE
.equ UBRR_VAL = 103
; Paso de velocidad de scroll por cada '+'/'-' recibido por UART
.equ SCROLL_MIN  = 5
.equ SCROLL_MAX  = 60
.equ SCROLL_STEP = 5

; MEMORIA DE DATOS (RAM)
.dseg
FRAME: .byte 8
.cseg

; VECTOR RESET
.org 0x0000
    rjmp inicio

inicio:
    ; Configurar SP
    ldi temp, low(RAMEND)
    out SPL, temp
    ldi temp, high(RAMEND)
    out SPH, temp
	
	; PORTD -> 6 FILAS DE LA MATRIZ
    ldi temp, 0b1111_1100
    out DDRD, temp

    ; Apagar filas PD2-PD7
    in temp, PORTD
    andi temp, 0b0000_0011
    out PORTD, temp

    ; PORTB -> PB0 a PB5
    ldi temp, 0b0011_1111
    out DDRB, temp
    ldi temp, 0b0011_1111
    out PORTB, temp

    ; PORTC
    ; PC0-PC1 = columnas 7 y 8 // PC4-PC5 = filas 7 y 8
    ldi temp, 0b0011_0011
    out DDRC, temp

    ;  filas apagadas
    ldi temp, 0b00000000
    out PORTC, temp
	
	; Registro cero (usado para sumas de 16 bits con acarreo)
    clr zero

    ; Configurar UART (9600, 8N1)
    ldi temp, high(UBRR_VAL)
    sts UBRR0H, temp
    ldi temp, low(UBRR_VAL)
    sts UBRR0L, temp
    ldi temp, (1<<RXEN0)|(1<<TXEN0)
    sts UCSR0B, temp
    ldi temp, (1<<UCSZ01)|(1<<UCSZ00)   ; 8 bits, sin paridad, 1 stop bit
    sts UCSR0C, temp

    ; Inicializar frame buffer del mensaje 
    ldi YL, low(FRAME)
    ldi YH, high(FRAME)
    clr temp
    ldi temp2, 8

LIMPIAR_FRAME:
    st Y+, temp
    dec temp2
    brne LIMPIAR_FRAME

    clr col_idx
    ldi temp, 15              ; velocidad inicial (ajustable por UART +/-)
    mov scrolldly, temp
    mov tickcnt, scrolldly

    ; Modo inicial: mensaje con desplazamiento
    ldi imagen, 3

    ; Mensaje de bienvenida + menu por UART
    ldi ZL, low(MSG_BIENVENIDA*2)
    ldi ZH, high(MSG_BIENVENIDA*2)
    rcall UART_SEND_STRING

    ldi ZL, low(MSG_MENU*2)
    ldi ZH, high(MSG_MENU*2)
    rcall UART_SEND_STRING
	
; PROGRAMA PRINCIPAL
MAIN:
    rcall MOSTRAR_IMAGEN
    rcall LEER_UART
    rjmp MAIN


; MOSTRAR IMAGEN / MENSAJE (segun el registro "imagen")
MOSTRAR_IMAGEN:
    cpi imagen, 3
    breq SHOW_SCROLL

    cpi imagen, 0
    breq CARGAR_SONRISA
    cpi imagen, 1
    breq CARGAR_CORAZON
    rjmp CARGAR_ASTERISCO

;Figuras fijas 
CARGAR_SONRISA:
    ldi ZL, low(SONRISA*2)
    ldi ZH, high(SONRISA*2)
    rjmp INICIAR_MATRIZ

CARGAR_CORAZON:
    ldi ZL, low(CORAZON*2)
    ldi ZH, high(CORAZON*2)
    rjmp INICIAR_MATRIZ

CARGAR_ASTERISCO:
    ldi ZL, low(ASTERISCO*2)
    ldi ZH, high(ASTERISCO*2)

INICIAR_MATRIZ:
    clr fila
    ldi temp2, 8
    rjmp BARRIDO_FLASH

; BARRIDO_FLASH: multiplexado leyendo el patron desde FLASH (lpm)
; Se usa para las 3 figuras fijas (sin cambios respecto al original)
BARRIDO_FLASH:
    rcall APAGAR_FILAS

    lpm patron, Z+
    com patron

    mov temp, patron
    andi temp, 0b00111111
    out PORTB, temp

    mov temp, patron
    lsr temp
    lsr temp
    lsr temp
    lsr temp
    lsr temp
    lsr temp
    andi temp, 0b00000011
    ori temp, 0b00001100
    out PORTC, temp

    rcall ACTIVAR_FILA
    rcall DELAY_FILA

    inc fila
    dec temp2
    brne BARRIDO_FLASH

    rcall APAGAR_FILAS
    ret

; SHOW_SCROLL: muestra el frame buffer (RAM) del mensaje y controla
; el ritmo de avance del desplazamiento
SHOW_SCROLL:
    rcall BARRIDO_RAM

    dec tickcnt
    brne FIN_SHOW_SCROLL
    mov tickcnt, scrolldly
    rcall ADVANCE_SCROLL
FIN_SHOW_SCROLL:
    ret

; BARRIDO_RAM: igual que BARRIDO_FLASH pero lee el patron de fila
; desde el frame buffer en RAM (ld) en lugar de la flash (lpm)
BARRIDO_RAM:
    clr fila
    ldi temp2, 8
    ldi YL, low(FRAME)
    ldi YH, high(FRAME)

BARRIDO_RAM_LOOP:
    rcall APAGAR_FILAS

    ld patron, Y+
    com patron

    mov temp, patron
    andi temp, 0b00111111
    out PORTB, temp

    mov temp, patron
    lsr temp
    lsr temp
    lsr temp
    lsr temp
    lsr temp
    lsr temp
    andi temp, 0b00000011
    ori temp, 0b00001100
    out PORTC, temp

    rcall ACTIVAR_FILA
    rcall DELAY_FILA

    inc fila
    dec temp2
    brne BARRIDO_RAM_LOOP

    rcall APAGAR_FILAS
    ret

; ADVANCE_SCROLL: toma la siguiente columna del mensaje (flash) e
; inserta un bit por fila en el frame buffer, desplazando el resto
ADVANCE_SCROLL:
    ; Z = MENSAJE_COLS + col_idx (direccion de byte en flash)
    ldi ZL, low(MENSAJE_COLS*2)
    ldi ZH, high(MENSAJE_COLS*2)
    add ZL, col_idx
    adc ZH, zero
    lpm newcol, Z

    ; Avanzar y hacer wrap del indice del mensaje
    inc col_idx
    mov temp, col_idx
    cpi temp, MENSAJE_LEN
    brlo FIN_INDEX
    clr col_idx
FIN_INDEX:

    ; Insertar el bit r de "newcol" como bit7 de FRAME[r], desplazando
    ; el resto de la fila un bit a la derecha (entra por la derecha,
    ; sale por la izquierda -> texto se mueve de derecha a izquierda)
    ldi temp, 8
    mov rowcnt, temp
    ldi YL, low(FRAME)
    ldi YH, high(FRAME)

ADVANCE_SCROLL_ROWS:
    ld temp, Y
    lsr temp
    lsr newcol
    brcc ADVANCE_SCROLL_SKIP
    ori temp, 0b10000000
ADVANCE_SCROLL_SKIP:
    st Y+, temp
    dec rowcnt
    brne ADVANCE_SCROLL_ROWS
    ret

;Rutinas de filas

APAGAR_FILAS:
    in temp, PORTD
    andi temp, 0b00000011
    out PORTD, temp

    in temp, PORTC
    andi temp, 0b0000_1111
    out PORTC, temp
    ret

ACTIVAR_FILA:
    cpi fila, 0
    breq FILA_1
    cpi fila, 1
    breq FILA_2
    cpi fila, 2
    breq FILA_3
    cpi fila, 3
    breq FILA_4
    cpi fila, 4
    breq FILA_5
    cpi fila, 5
    breq FILA_6
    cpi fila, 6
    breq FILA_7
	sbi PORTC, 5
    ret

FILA_1:
    sbi PORTD, 2
    ret
FILA_2:
    sbi PORTD, 3
    ret
FILA_3:
    sbi PORTD, 4
    ret
FILA_4:
    sbi PORTD, 5
    ret
FILA_5:
    sbi PORTD, 6
    ret
FILA_6:
    sbi PORTD, 7
    ret
FILA_7:
    sbi PORTC, 4
    ret

