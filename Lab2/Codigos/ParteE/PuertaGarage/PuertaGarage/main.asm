.include "m328Pdef.inc"
; Definición de pines de entrada 
.equ BTN_ABRIR    = 2
.equ BTN_CERRAR   = 3
.equ SENS_S1      = 4 ; sensor de posición S1
.equ SENS_S2      = 5 ;sensor de posición S2

; Definición de pines de salida 
.equ PIN_OBST       = 0 
.equ PIN_MOT_ABRIR  = 1
.equ PIN_MOT_CERRAR = 2
.equ PIN_ALARMA     = 3

; Leds indicadores 
.equ PIN_LED_ABRIENDO = 0
.equ PIN_LED_CERRANDO = 1
.equ PIN_LED_S1       = 2
.equ PIN_LED_S2       = 3

; Estados de la máquina de estados
.equ ST_CERRADA   = 0
.equ ST_ABRIENDO  = 1
.equ ST_ABIERTA   = 2
.equ ST_CERRANDO  = 3
.equ ST_DETENIDA  = 4
