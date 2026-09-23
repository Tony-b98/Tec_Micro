.include "m328Pdef.inc"

; Definicion de registros

.def temp         = r16
.def cmd          = r17
.def dur          = r18
.def dato         = r19
.def pen_state    = r20      ; 0 = arriba, 1 = abajo
.def auto_mode    = r21      ; 1 solamente mientras se ejecuta T
.def next_stage   = r22      ; etapa a guardar antes de PEN_UP
.def resume_stage = r23      ; etapa leida desde EEPROM
.def sreg_save    = r24      ; respaldo de SREG para escritura EEPROM

; Pines PORTD
.equ PIN_BAJAR  = 2
.equ PIN_SUBIR  = 3
.equ PIN_ABAJO  = 4
.equ PIN_ARRIBA = 5
.equ PIN_IZQ    = 6
.equ PIN_DER    = 7

; Movimientos
.equ MV_D  = 0x10
.equ MV_U  = 0x20
.equ MV_L  = 0x40
.equ MV_R  = 0x80
.equ MV_DL = 0x50
.equ MV_DR = 0x90
.equ MV_UL = 0x60
.equ MV_UR = 0xA0

.equ OP_PEN_DOWN = 0x01
.equ OP_PEN_UP   = 0x02
.equ OP_END      = 0xFF

.equ PEN_IS_UP   = 0
.equ PEN_IS_DOWN = 1

; Reanudacion persistente SOLO para el comando "Todas (T)"
.equ EE_STAGE_ADDR = 0
.equ STAGE_IDLE           = 0
.equ STAGE_AFTER_TRIANGLE = 1
.equ STAGE_AFTER_CIRCLE   = 2
.equ STAGE_AFTER_STAR     = 3
.equ STAGE_AFTER_CUBE     = 4
.equ STAGE_AFTER_PORYGON  = 5

; retorno automatico a HOME para comandos individuales.
.equ STAGE_SINGLE_TRIANGLE = 10
.equ STAGE_SINGLE_CIRCLE   = 11
.equ STAGE_SINGLE_STAR     = 12
.equ STAGE_SINGLE_CUBE     = 13
.equ STAGE_SINGLE_PORYGON  = 14

; Timer1 genera 1 tick cada 10 ms.
; Lapiz: pulso mas largo para asegurar el cambio de posicion
.equ SOL_PULSE_TICKS  = 10    ; 100 ms
.equ SOL_SETTLE_TICKS = 30    ; 300 ms

; Diseño en A4
; [1] TRIANGULO    [2] CIRCULO      [3] PENTAGRAMA
;
; [4] CUBO 3D      [5] PORYGON      [6] T = TODAS

.equ SLOT_X_PART = 165
.equ SLOT_Y_PART = 175

; RESET
.cseg
.org 0x0000
    rjmp RESET

RESET:
	;Inicializo SP
    ldi temp, low(RAMEND)
    out SPL, temp
    ldi temp, high(RAMEND)
    out SPH, temp

    ; PD2..PD7 salidas
    ldi temp, 0b11111100
    out DDRD, temp
	clr temp
    out PORTD, temp

	; No se acciona el solenoide durante RESET.
    ldi pen_state, PEN_IS_UP

    clr auto_mode
    clr next_stage
    clr resume_stage

    rcall TIMER1_INIT
    rcall UART_INIT
    rcall STOP_MOV
