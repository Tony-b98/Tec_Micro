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

    ; ---------------- Configurar UART (9600, 8N1) ----------------
    ldi temp, high(UBRR_VAL)
    sts UBRR0H, temp
    ldi temp, low(UBRR_VAL)
    sts UBRR0L, temp
    ldi temp, (1<<RXEN0)|(1<<TXEN0)
    sts UCSR0B, temp
    ldi temp, (1<<UCSZ01)|(1<<UCSZ00)   ; 8 bits, sin paridad, 1 stop bit
    sts UCSR0C, temp

    ; ---------------- Inicializar frame buffer del mensaje --------
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

    ; ---------------- Mensaje de bienvenida + menu por UART -------
    ldi ZL, low(MSG_BIENVENIDA*2)
    ldi ZH, high(MSG_BIENVENIDA*2)
    rcall UART_SEND_STRING

    ldi ZL, low(MSG_MENU*2)
    ldi ZH, high(MSG_MENU*2)
    rcall UART_SEND_STRING
