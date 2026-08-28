.include "m328Pdef.inc"

.def temp      = r16
.def secuencia = r17     ; 0-3, secuencia actualmente activa
.def patron    = r18     ; patrón de 8 bits que se muestra en los LEDs
.def aux       = r21
.def aux2      = r22

.equ BTN_SIG    = 0   ; PB0
.equ BTN_ANT    = 1   ; PB1
.equ BTN_INICIO = 2   ; PB2
