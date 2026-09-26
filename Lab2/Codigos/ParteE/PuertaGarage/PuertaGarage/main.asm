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

; MÁQUINA DE ESTADOS
; Controla las transiciones según entradas, sensores y obstáculo
MAQUINA_ESTADOS:

        ; Si hay obstáculo, detener el sistema
        tst   flag_obst
        breq  ME_SIN_OBSTACULO
        clr   flag_obst

        ; Evita repetir la detención si ya está detenido
        cpi   estado, ST_DETENIDA
        breq  ME_YA_DETENIDA

        ldi   estado, ST_DETENIDA
        rcall SALIDAS_DETENIDA

        ; Mensajes USART de seguridad
        ldi   ZL, LOW(MSG_OBSTACULO*2)
        ldi   ZH, HIGH(MSG_OBSTACULO*2)
        rcall ENVIAR_STRING

        ldi   ZL, LOW(MSG_DETENIDO*2)
        ldi   ZH, HIGH(MSG_DETENIDO*2)
        rcall ENVIAR_STRING

        rjmp  ME_FIN

ME_YA_DETENIDA:
        rjmp  ME_FIN


; Selección del estado actual
ME_SIN_OBSTACULO:

        cpi   estado, ST_CERRADA
        breq  ME_CERRADA

        cpi   estado, ST_ABRIENDO
        breq  ME_ABRIENDO

        cpi   estado, ST_ABIERTA
        breq  ME_ABIERTA

        cpi   estado, ST_CERRANDO
        breq  ME_CERRANDO

        cpi   estado, ST_DETENIDA
        breq  ME_DETENIDA

        rjmp  ME_FIN


; ESTADO CERRADA: espera orden de apertura
ME_CERRADA:

        sbrs  flancos, BTN_ABRIR
        rjmp  ME_FIN

        ldi   estado, ST_ABRIENDO
        rcall SALIDAS_ABRIENDO

        ldi   ZL, LOW(MSG_ABRIENDO*2)
        ldi   ZH, HIGH(MSG_ABRIENDO*2)
        rcall ENVIAR_STRING

        rjmp  ME_FIN


; ESTADO ABRIENDO: espera sensor S1 o una orden de parada
ME_ABRIENDO:

        sbrc  entr_act, SENS_S1
        rjmp  ME_AB_A_ABIERTA

        sbrc  flancos, BTN_CERRAR
        rjmp  ME_AB_A_DETENIDA

        rjmp  ME_FIN


; Transición ABRIENDO -> ABIERTA
ME_AB_A_ABIERTA:

        ldi   estado, ST_ABIERTA
        rcall SALIDAS_ABIERTA

        ldi   ZL, LOW(MSG_ABIERTA*2)
        ldi   ZH, HIGH(MSG_ABIERTA*2)
        rcall ENVIAR_STRING

        rjmp  ME_FIN


; Transición ABRIENDO -> DETENIDA
ME_AB_A_DETENIDA:

        ldi   estado, ST_DETENIDA
        rcall SALIDAS_DETENIDA

        ldi   ZL, LOW(MSG_DETENIDO*2)
        ldi   ZH, HIGH(MSG_DETENIDO*2)
        rcall ENVIAR_STRING

        rjmp  ME_FIN


; ESTADO ABIERTA: espera orden de cierre
ME_ABIERTA:

        sbrs  flancos, BTN_CERRAR
        rjmp  ME_FIN

        ldi   estado, ST_CERRANDO
        rcall SALIDAS_CERRANDO

        ldi   ZL, LOW(MSG_CERRANDO*2)
        ldi   ZH, HIGH(MSG_CERRANDO*2)
        rcall ENVIAR_STRING

        rjmp  ME_FIN


; ESTADO CERRANDO: espera sensor S2 o una orden de parada
ME_CERRANDO:

        sbrc  entr_act, SENS_S2
        rjmp  ME_CE_A_CERRADA

        sbrc  flancos, BTN_ABRIR
        rjmp  ME_CE_A_DETENIDA

        rjmp  ME_FIN


; Transición CERRANDO -> CERRADA
ME_CE_A_CERRADA:

        ldi   estado, ST_CERRADA
        rcall SALIDAS_CERRADA

        ldi   ZL, LOW(MSG_CERRADA*2)
        ldi   ZH, HIGH(MSG_CERRADA*2)
        rcall ENVIAR_STRING

        rjmp  ME_FIN


; Transición CERRANDO -> DETENIDA
ME_CE_A_DETENIDA:

        ldi   estado, ST_DETENIDA
        rcall SALIDAS_DETENIDA

        ldi   ZL, LOW(MSG_DETENIDO*2)
        ldi   ZH, HIGH(MSG_DETENIDO*2)
        rcall ENVIAR_STRING

        rjmp  ME_FIN


; ESTADO DETENIDA: espera nueva orden
ME_DETENIDA:

        sbrc  flancos, BTN_ABRIR
        rjmp  ME_DE_A_ABRIENDO

        sbrc  flancos, BTN_CERRAR
        rjmp  ME_DE_A_CERRANDO

        rjmp  ME_FIN


; Transición DETENIDA -> ABRIENDO
ME_DE_A_ABRIENDO:

        ldi   estado, ST_ABRIENDO
        rcall SALIDAS_ABRIENDO

        ldi   ZL, LOW(MSG_ABRIENDO*2)
        ldi   ZH, HIGH(MSG_ABRIENDO*2)
        rcall ENVIAR_STRING

        rjmp  ME_FIN


; Transición DETENIDA -> CERRANDO
ME_DE_A_CERRANDO:

        ldi   estado, ST_CERRANDO
        rcall SALIDAS_CERRANDO

        ldi   ZL, LOW(MSG_CERRANDO*2)
        ldi   ZH, HIGH(MSG_CERRANDO*2)
        rcall ENVIAR_STRING


; Fin de la máquina de estados
ME_FIN:
        ret

; Salidas
SALIDAS_CERRADA:
        ldi   temp, (1<<PIN_LED_S2)
        out   PORTC, temp
        ldi   temp, (1<<PIN_OBST)
        out   PORTB, temp
        ret

SALIDAS_ABRIENDO:
        ldi   temp, (1<<PIN_LED_ABRIENDO)
        out   PORTC, temp
        ldi   temp, (1<<PIN_OBST)|(1<<PIN_MOT_ABRIR)|(1<<PIN_ALARMA)
        out   PORTB, temp
        ret

SALIDAS_ABIERTA:
        ldi   temp, (1<<PIN_LED_S1)
        out   PORTC, temp
        ldi   temp, (1<<PIN_OBST)
        out   PORTB, temp
        ret

SALIDAS_CERRANDO:
        ldi   temp, (1<<PIN_LED_CERRANDO)
        out   PORTC, temp
        ldi   temp, (1<<PIN_OBST)|(1<<PIN_MOT_CERRAR)|(1<<PIN_ALARMA)
        out   PORTB, temp
        ret

SALIDAS_DETENIDA:
        clr   temp
        out   PORTC, temp
        ldi   temp, (1<<PIN_OBST)
        out   PORTB, temp
        ret

ENVIAR_STRING:
        lpm   temp, Z+
        tst   temp
        breq  ENVIAR_STRING_FIN
        rcall ENVIAR_BYTE
        rjmp  ENVIAR_STRING
ENVIAR_STRING_FIN:
        ret

ENVIAR_BYTE:
        push  temp2
ENVIAR_BYTE_ESPERA:
        lds   temp2, UCSR0A
        sbrs  temp2, UDRE0
        rjmp  ENVIAR_BYTE_ESPERA
        sts   UDR0, temp
        pop   temp2
        ret

; Mensajes de visualizacion de accion realizada
MSG_CERRADA:
        .db   "Puerta cerrada", 13, 10, 0
MSG_ABRIENDO:
        .db   "Puerta abriendo", 13, 10, 0
MSG_ABIERTA:
        .db   "Puerta abierta", 13, 10, 0
MSG_CERRANDO:
        .db   "Puerta cerrando", 13, 10, 0
MSG_OBSTACULO:
        .db   "Obstaculo detectado", 13, 10, 0
MSG_DETENIDO:
        .db   "Movimiento detenido", 13, 10, 0

