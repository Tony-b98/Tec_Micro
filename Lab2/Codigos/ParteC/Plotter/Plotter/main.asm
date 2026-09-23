;=====================================================================
; LABORATORIO 2 - PROBLEMA C: PLOTTER
; ATmega328P - Arduino Uno - Equipo 1
;
; CORRECCIONES:
; 1 -> Triangulo aproximadamente al doble
; 2 -> Circulo de 16 lados para verse mas redondeado
; 3 -> Estrella regular de 5 puntas
; 4 -> Cubo 3D de un solo trazo
; P -> Porygon redisenado segun la imagen de referencia
; T -> Todas las figuras en grilla A4 de 6 cuadrantes
;
; Pines:
; D2 / PD2 -> Bajar solenoide
; D3 / PD3 -> Subir solenoide
; D4 / PD4 -> Abajo
; D5 / PD5 -> Arriba
; D6 / PD6 -> Izquierda
; D7 / PD7 -> Derecha
;
; IMPORTANTE:
; Antes de la primera prueba ubicar el cabezal en HOME,
; cerca de la esquina superior izquierda de la hoja A4.
; Todas las trayectorias vuelven matematicamente a su origen local.
;=====================================================================

.include "m328Pdef.inc"

;---------------------------------------------------------------------
; Registros
;---------------------------------------------------------------------
.def temp         = r16
.def cmd          = r17
.def dur          = r18
.def dato         = r19
.def pen_state    = r20      ; 0 = arriba, 1 = abajo
.def auto_mode    = r21      ; 1 solamente mientras se ejecuta T
.def next_stage   = r22      ; etapa a guardar antes de PEN_UP
.def resume_stage = r23      ; etapa leida desde EEPROM
.def sreg_save    = r24      ; respaldo de SREG para escritura EEPROM

;---------------------------------------------------------------------
; Pines PORTD
;---------------------------------------------------------------------
.equ PIN_BAJAR  = 2
.equ PIN_SUBIR  = 3
.equ PIN_ABAJO  = 4
.equ PIN_ARRIBA = 5
.equ PIN_IZQ    = 6
.equ PIN_DER    = 7

;---------------------------------------------------------------------
; Movimientos
;---------------------------------------------------------------------
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

;---------------------------------------------------------------------
; Reanudacion persistente SOLO para el comando T
;---------------------------------------------------------------------
.equ EE_STAGE_ADDR = 0

.equ STAGE_IDLE           = 0
.equ STAGE_AFTER_TRIANGLE = 1
.equ STAGE_AFTER_CIRCLE   = 2
.equ STAGE_AFTER_STAR     = 3
.equ STAGE_AFTER_CUBE     = 4
.equ STAGE_AFTER_PORYGON  = 5

; Etapas 10..14:
; retorno automatico a HOME para comandos individuales.
.equ STAGE_SINGLE_TRIANGLE = 10
.equ STAGE_SINGLE_CIRCLE   = 11
.equ STAGE_SINGLE_STAR     = 12
.equ STAGE_SINGLE_CUBE     = 13
.equ STAGE_SINGLE_PORYGON  = 14

;---------------------------------------------------------------------
; Tiempos
; Timer1 genera 1 tick cada 10 ms.
;---------------------------------------------------------------------
; Lapiz: pulso mas largo para asegurar el cambio de posicion
; Timer1 = 10 ms por tick
.equ SOL_PULSE_TICKS  = 10    ; 100 ms
.equ SOL_SETTLE_TICKS = 30    ; 300 ms

;---------------------------------------------------------------------
; Separacion de las posiciones A4
;
; Se hacen CUATRO movimientos por SLOT para duplicar la escala
; sin superar el limite de 255 ticks del registro dur.
;
; CUADRANTES LOGICOS - NO SE DIBUJAN LINEAS DIVISORAS
;
; Ancho logico de cada cuadrante = 660 ticks
; Alto logico de cada cuadrante  = 700 ticks
;
; [1] TRIANGULO    [2] CIRCULO      [3] PENTAGRAMA
;
; [4] CUBO 3D      [5] PORYGON      [6] T = TODAS
;
; Cada figura se desplaza dentro de su cuadrante para quedar
; aproximadamente centrada.
;---------------------------------------------------------------------
.equ SLOT_X_PART = 165
.equ SLOT_Y_PART = 175

;=====================================================================
; RESET
;=====================================================================
.cseg
.org 0x0000
    rjmp RESET

RESET:
    ldi temp, low(RAMEND)
    out SPL, temp
    ldi temp, high(RAMEND)
    out SPH, temp

    ; PD2..PD7 salidas. PD0/PD1 reservados para USART.
    ldi temp, 0b11111100
    out DDRD, temp

    clr temp
    out PORTD, temp

    ; Al arrancar se asume que el lapiz esta arriba.
    ; IMPORTANTE: no se acciona el solenoide durante RESET.
    ldi pen_state, PEN_IS_UP

    clr auto_mode
    clr next_stage
    clr resume_stage

    rcall TIMER1_INIT
    rcall UART_INIT
    rcall STOP_MOV
