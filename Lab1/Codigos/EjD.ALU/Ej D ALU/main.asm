.include "m328Pdef.inc"

.cseg

; DEFINICION DE REGISTROS
.def temp    = r16
.def A       = r17
.def B       = r18
.def S       = r19
.def F       = r20
.def C_flag  = r21
.def aux     = r22

;CÓDIGOS DE SELECCIÓN DE LA ALU
.equ S_CLEAR = 0
.equ S_SUB   = 1
.equ S_ADD   = 2
.equ S_XOR   = 3
.equ S_AND   = 4
.equ S_OR    = 5
.equ S_SHL   = 6
.equ S_INC   = 7

; VECTOR DE RESET
.org 0x0000
    rjmp RESET_ALU


;CONFIGURACIÓN INICIAL
RESET_ALU:

    ldi temp, low(RAMEND)
    out SPL, temp
	ldi temp, high(RAMEND)
    out SPH, temp

    ldi temp, 0x30
    out DDRC, temp

    ldi temp, 0x0f
    out PORTC, temp

    ldi temp, 0x00
    out DDRD, temp

    ldi temp, 0xfc
    out PORTD, temp

    ldi temp, 0x3e
    out DDRB, temp

    ldi temp, 0x01
    out PORTB, temp

; Inicialización de los registros en cero	
    clr A
    clr B
    clr S
    clr F
    clr C_flag

    rcall MOSTRAR_SALIDA

; BUCLE PRINCIPAL
MAIN_LOOP:
    rcall LEER_ENTRADAS
    rcall CALCULAR_ALU
    rcall MOSTRAR_SALIDA
    rjmp MAIN_LOOP

; LECTURA DE ENTRADAS
LEER_ENTRADAS:

 ; Lectura del operando A desde PC0-PC3
    in temp, PINC
    com temp
    andi temp, 0x0F
    mov A, temp

; Lectura del operando B desde PD2-PD5
    in temp, PIND
    com temp

    mov B, temp
    lsr B
    lsr B
    andi B, 0x0F

; Formación del selector S = S2 S1 S0
    clr S

    sbrc temp, 6
    ori S, 0b00000001

    sbrc temp, 7
    ori S, 0b00000010

    in aux, PINB
    com aux

    sbrc aux, 0
    ori S, 0b00000100

    ret

; SELECCIÓN DE LA OPERACIÓN
CALCULAR_ALU:
    cpi S, S_CLEAR
    breq DO_CLEAR

    cpi S, S_SUB
    breq DO_SUB

    cpi S, S_ADD
    breq DO_ADD

    cpi S, S_XOR
    breq DO_XOR

    cpi S, S_AND
    breq DO_AND

    cpi S, S_OR
    breq DO_OR

    cpi S, S_SHL
    breq DO_SHL

    rjmp DO_INC


; CLEAR
; F = 0000
DO_CLEAR:
    clr F
    clr C_flag
    ret

; OPERACIÓN SUB
; F = A - B
DO_SUB:
    mov F, A
    sub F, B

    brcc SUB_NO_CARRY

    ldi C_flag, 1
    rjmp SUB_FIN

SUB_NO_CARRY:
    clr C_flag

SUB_FIN:
    andi F, 0x0F
    ret

; SUMA
; F = A + B
DO_ADD:
    mov F, A
    add F, B
	sbrc F, 4
    rjmp ADD_CARRY
	clr C_flag
    rjmp ADD_FIN

ADD_CARRY:
    ldi C_flag, 1

ADD_FIN:
    andi F, 0x0F
    ret

; XOR
; F = A XOR B
DO_XOR:
    mov F, A
    eor F, B
    clr C_flag
    andi F, 0x0F
    ret

; OPERACIÓN AND
; F = A AND B
DO_AND:
    mov F, A
    and F, B
	; Las operaciones lógicas no generan Carry
    clr C_flag
	; Limitar resultado a 4 bits
    andi F, 0x0F
    ret

; OPERACIÓN OR
;F= A OR B
DO_OR:
    mov F, A
    or F, B
    clr C_flag
    andi F, 0x0F
    ret

; OPERACIÓN SHL
; F = A << 1
DO_SHL:
    clr C_flag
; El bit A3 que sale del rango se guarda en Carry
    sbrc A, 3
    ldi C_flag, 1

    mov F, A
    lsl F
 ; Mantener solo los 4 bits del resultado
    andi F, 0x0F

    ret

; OPERACIÓN INC
; F = A + 1
DO_INC:
; Incrementar A en una unidad
    mov F, A

    ldi temp, 1
    add F, temp
; Verificar si hubo acarreo fuera de los 4 bits
    sbrc F, 4
    rjmp INC_CARRY

    clr C_flag
    rjmp INC_FIN

INC_CARRY:
  ; Activar bandera Carry
    ldi C_flag, 1

INC_FIN:
 ; Limitar resultado a 4 bits
    andi F, 0x0F
    ret

	
MOSTRAR_SALIDA:
; Bit0 en 1 para mantener PB0 como entrada con pull-up (S2)
    ldi temp, 0b00000001

; F0 -> PB1
    sbrc F, 0
    ori temp, 0b00000010

; F1 -> PB2
    sbrc F, 1
    ori temp, 0b00000100

; F2 -> PB3
    sbrc F, 2
    ori temp, 0b00001000

; F3 -> PB4
    sbrc F, 3
    ori temp, 0b00010000

; Carry -> PB5
    tst C_flag
    breq MS_SIN_C

    ori temp, 0b00100000

MS_SIN_C:
    out PORTB, temp

    ldi temp, 0b00001111

; Negativo (bit3 de F) -> PC4
    sbrc F, 3
    ori temp, 0b00010000

; Zero (F = 0000) -> PC5
    tst F
    brne MS_NO_ZERO

    ori temp, 0b00100000

MS_NO_ZERO:
    out PORTC, temp

    ret