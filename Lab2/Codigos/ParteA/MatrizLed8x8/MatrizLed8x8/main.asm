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
; =====================================================================
; MENU POR UART (no bloqueante: si no llego nada, sigue de largo)
; =====================================================================

LEER_UART:
    lds temp, UCSR0A
    sbrs temp, RXC0
    ret
    lds uartc, UDR0
    mov temp, uartc

    cpi temp, '1'
    brne CHK_2
    clr imagen
    ldi ZL, low(MSG_OK_SONRISA*2)
    ldi ZH, high(MSG_OK_SONRISA*2)
    rjmp UART_ACK

CHK_2:
    cpi temp, '2'
    brne CHK_3
    ldi imagen, 1
    ldi ZL, low(MSG_OK_CORAZON*2)
    ldi ZH, high(MSG_OK_CORAZON*2)
    rjmp UART_ACK

CHK_3:
    cpi temp, '3'
    brne CHK_4
    ldi imagen, 2
    ldi ZL, low(MSG_OK_ASTERISCO*2)
    ldi ZH, high(MSG_OK_ASTERISCO*2)
    rjmp UART_ACK

CHK_4:
    cpi temp, '4'
    brne CHK_MAS
    ldi imagen, 3
    ldi ZL, low(MSG_OK_MENSAJE*2)
    ldi ZH, high(MSG_OK_MENSAJE*2)
    rjmp UART_ACK

CHK_MAS:
    cpi temp, '+'
    brne CHK_MENOS
    rcall SCROLL_MAS_RAPIDO
    ret

CHK_MENOS:
    cpi temp, '-'
    brne LEER_UART_FIN
    rcall SCROLL_MAS_LENTO
LEER_UART_FIN:
    ret

UART_ACK:
    rcall UART_SEND_STRING
    ret

SCROLL_MAS_RAPIDO:
    mov temp, scrolldly
    cpi temp, SCROLL_MIN + SCROLL_STEP
    brsh RESTAR_STEP
    ldi temp, SCROLL_MIN
    rjmp GUARDAR_RAPIDO
RESTAR_STEP:
    subi temp, SCROLL_STEP
GUARDAR_RAPIDO:
    mov scrolldly, temp
    ret

SCROLL_MAS_LENTO:
    mov temp, scrolldly
    cpi temp, (SCROLL_MAX + 1) - SCROLL_STEP
    brlo SUMAR_STEP
    ldi temp, SCROLL_MAX
    rjmp GUARDAR_LENTO
SUMAR_STEP:
    subi temp, -SCROLL_STEP        ; subi con negativo = sumar (no existe addi)
GUARDAR_LENTO:
    mov scrolldly, temp
    ret

; =====================================================================
; RUTINAS UART DE ENVIO
; =====================================================================

; Envia el caracter en "temp" por UART (bloqueante, espera buffer libre)
UART_SEND_CHAR:
    lds temp2, UCSR0A
    sbrs temp2, UDRE0
    rjmp UART_SEND_CHAR
    sts UDR0, temp
    ret

; Envia una cadena terminada en 0x00 apuntada por Z (direccion de flash)
UART_SEND_STRING:
    lpm temp, Z+
    cpi temp, 0
    breq FIN_SEND_STRING
    rcall UART_SEND_CHAR
    rjmp UART_SEND_STRING
FIN_SEND_STRING:
    ret

; =====================================================================
; DELAY (igual al original)
; =====================================================================
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

; =====================================================================
; DIBUJOS 8x8 (figuras fijas, sin cambios)
; =====================================================================
SONRISA:
.db 0b0011_1100, 0b0100_0010, 0b1010_0101, 0b1000_0001
.db 0b1010_0101, 0b1001_1001, 0b0100_0010, 0b0011_1100

CORAZON:
.db 0b0000_0000, 0b0110_0110, 0b1111_1111, 0b1111_1111
.db 0b0111_1110, 0b0011_1100, 0b0001_1000, 0b0000_0000

ASTERISCO:
.db 0b0001_1000, 0b0101_1010, 0b0011_1100, 0b1111_1111
.db 0b0011_1100, 0b0101_1010, 0b0001_1000, 0b0000_0000

; =====================================================================
; FUENTE DEL MENSAJE "HELLO WORLD" - formato COLUMNA (5 cols x 7 filas,
; bit0 = fila superior). Fuente clasica 5x7 de dominio publico usada en
; proyectos de matrices LED. Cada letra: 5 bytes de datos + 1 byte de
; separacion (0x00). Si alguna letra se ve distinta a lo esperado en la
; matriz fisica, se puede ajustar el byte correspondiente sin tocar el
; resto del programa.
; =====================================================================
MENSAJE_COLS:
; H
.db 0x7F,0x08,0x08,0x08,0x7F,0x00
; E
.db 0x7F,0x49,0x49,0x49,0x41,0x00
; L
.db 0x7F,0x40,0x40,0x40,0x40,0x00
; L
.db 0x7F,0x40,0x40,0x40,0x40,0x00
; O
.db 0x3E,0x41,0x41,0x41,0x3E,0x00,0x00,0x00   ; espacio extra (fin de palabra)
; W
.db 0x3F,0x40,0x38,0x40,0x3F,0x00
; O
.db 0x3E,0x41,0x41,0x41,0x3E,0x00
; R
.db 0x7F,0x09,0x19,0x29,0x46,0x00
; L
.db 0x7F,0x40,0x40,0x40,0x40,0x00
; D
.db 0x7F,0x41,0x41,0x22,0x1C,0x00
; relleno en blanco para que el mensaje termine de salir antes de repetirse
.db 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
MENSAJE_END:

.equ MENSAJE_LEN = (MENSAJE_END - MENSAJE_COLS) * 2

; =====================================================================
; CADENAS DE TEXTO PARA LA UART
; Cada .DB contiene una cantidad PAR de bytes.
; =====================================================================

MSG_BIENVENIDA:
.db "Bienvenido al Menu",13,10,0,0

; IMPORTANTE: todo el menu es UNA sola cadena que termina en un unico
; byte 0 (al final). Antes el primer .db terminaba en 0,0, y como
; UART_SEND_STRING corta al primer byte 0 que encuentra, solo se
; enviaba la primera linea. Ahora se manda el menu completo de un tiro.
MSG_MENU:
.db "Seleccione una opcion:",13,10
.db "1 = Sonrisa",13,10,"2 = Corazon",13,10
.db "3 = Asterisco",13,10,"4 = Mensaje (+ = mas rapido, - = mas lento)",13,10,0,0

MSG_OK_SONRISA:
.db "-> Sonrisa",13,10,0,0

MSG_OK_CORAZON:
.db "-> Corazon",13,10,0,0

MSG_OK_ASTERISCO:
.db "-> Asterisco",13,10,0,0

MSG_OK_MENSAJE:
.db "-> Mensaje HELLO WORLD",13,10,0,0

