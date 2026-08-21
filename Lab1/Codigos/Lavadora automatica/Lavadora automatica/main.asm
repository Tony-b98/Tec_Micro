.include "m328Pdef.inc"

; Defino sinonimos para los registros a utilizar

.def temp        = r16   ; registro para guardar 
.def carga       = r17   ; Almacena la variable de tipo de carga, (0=ligera, 1=media, 2=pesada)
.def ciclo_lav   = r18   ; contador ciclos de lavado (5 -> 0)
.def segundos    = r19   ; parámetro para delay_seg
.def data_portD   = r20   ; copia los datos actuales del PORTD (LEDs D2-D7)
.def data_portB   = r21   ; copia los datos actuales del PORTB (LEDs D8-D9 + actuadores D10-D12)

; Asigno las salidas de bits de LEDs en PORTD (D2..D7)
.equ LED_LISTO      = 2
.equ LED_LAVADO     = 3
.equ LED_CENTRIF    = 4
.equ LED_SECADO     = 5
.equ LED_FIN        = 6
.equ LED_C_LIGERA   = 7

; Bits de LEDs/actuadores en PORTB 
.equ LED_C_MEDIA    = 0   ; D8
.equ LED_C_PESADA   = 1   ; D9
.equ MOTOR_DER      = 2   ; D10
.equ MOTOR_IZQ      = 3   ; D11
.equ VALVULA        = 4   ; D12

; Asigno los bits de entradas (PORTC = A0..A3)
.equ PIN_SELECCION  = 0
.equ PIN_INICIO     = 1
.equ PIN_PUERTA     = 2
.equ PIN_LLENADO    = 3

; Reset de programa
.org 0x0000
    rjmp RESET

RESET:
    ; Inicializo el Stack pointer, en el ultimo lugar de la RAM
    ldi   temp, LOW(RAMEND)
    out   SPL, temp
    ldi   temp, HIGH(RAMEND)
    out   SPH, temp

    ; Defino PORTD: D2-D7 como salida, colocando un "1" en sus bits 
    ldi   temp, 0b11111100
    out   DDRD, temp

    ; Defino PORTB: PB0-PB4 (D8-D12) como salida. PB6/PB7 no se tocan (cristal).
    ldi   temp, 0b00011111
    out   DDRB, temp

    ; Asigno PORTC (A0..A3) como entrada, con pull-ups activados
    clr   temp
    out   DDRC, temp
    ldi   temp, 0x0F
    out   PORTC, temp        ; pull-ups en PC0..PC3

    clr   data_portD
    clr   data_portB
    out   PORTD, data_portD
    out   PORTB, data_portB
    clr   carga
