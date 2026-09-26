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

; Definicion de registros
.def temp      = r16
.def temp2     = r17
.def estado    = r18
.def flag_obst = r19      ; 1 = obstaculo detectado
.def entr_act  = r20      ; entradas en esta lectura
.def entr_ant  = r21      ; entradas en la lectura anterior
.def flancos   = r22      ; entradas recien activadas
.def obst_ant  = r23      ; 1 = sin obstaculo en la lectura anterior

; RESET
.org 0x0000
        rjmp RESET

.org 0x0034
RESET:
        ;Inicializo SP
        ldi   temp, LOW(RAMEND)
        out   SPL, temp
        ldi   temp, HIGH(RAMEND)
        out   SPH, temp

        ; PORTD entradas con pull-up
        ldi   temp, 0x00
        out   DDRD, temp
        ldi   temp, (1<<BTN_ABRIR)|(1<<BTN_CERRAR)|(1<<SENS_S1)|(1<<SENS_S2)
        out   PORTD, temp

        ; Motor y alarma salidas, pull-up en sensor de obstaculo
        ldi   temp, (1<<PIN_MOT_ABRIR)|(1<<PIN_MOT_CERRAR)|(1<<PIN_ALARMA)
        out   DDRB, temp
        ldi   temp, (1<<PIN_OBST)
        out   PORTB, temp

        ldi   temp, (1<<PIN_LED_ABRIENDO)|(1<<PIN_LED_CERRANDO)|(1<<PIN_LED_S1)|(1<<PIN_LED_S2)
        out   DDRC, temp
        clr   temp
        out   PORTC, temp

        ; USART 9600 bps, solo TX
        ldi   temp, 103
        sts   UBRR0L, temp
        clr   temp
        sts   UBRR0H, temp
        ldi   temp, (1<<TXEN0)
        sts   UCSR0B, temp
        ldi   temp, (1<<UCSZ01)|(1<<UCSZ00)
        sts   UCSR0C, temp

        ; Se arranca con la puerta cerrada y sin obstaculo.
        clr   flag_obst
        clr   entr_ant
        ldi   temp, 1
        mov   obst_ant, temp
        ldi   estado, ST_CERRADA

        rcall SALIDAS_CERRADA
        ldi   ZL, LOW(MSG_CERRADA*2)
        ldi   ZH, HIGH(MSG_CERRADA*2)
        rcall ENVIAR_STRING

; LOOP
LOOP:
        rcall LEER_ENTRADAS
        rcall LEER_OBSTACULO
        rcall MAQUINA_ESTADOS
        rjmp  LOOP

; ENTRADAS
LEER_ENTRADAS:
        ; Entradas activas en bajo, se invierten.
        in    temp, PIND
        com   temp
        andi  temp, (1<<BTN_ABRIR)|(1<<BTN_CERRAR)|(1<<SENS_S1)|(1<<SENS_S2)
        mov   entr_act, temp

        ; flancos = activas ahora y no en la lectura anterior
        mov   temp2, entr_ant
        com   temp2
        and   temp2, entr_act
        mov   flancos, temp2
        mov   entr_ant, entr_act
        ret

; Obstaculo activo en bajo, flag solo en el flanco.
LEER_OBSTACULO:
        sbic  PINB, PIN_OBST
        rjmp  LO_IDLE
        tst   obst_ant
        breq  LO_FIN
        clr   obst_ant
        ldi   flag_obst, 1
        rjmp  LO_FIN
LO_IDLE:
        ldi   temp, 1
        mov   obst_ant, temp
LO_FIN:
        ret