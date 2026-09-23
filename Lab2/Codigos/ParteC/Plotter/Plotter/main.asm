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
	
    ; Si T fue interrumpida por el RESET de PEN_UP,
    ; EEPROM contiene exactamente el punto desde el que continuar.
    rcall EEPROM_READ_STAGE
    mov resume_stage, dato

    ; EEPROM = 0 -> funcionamiento normal.
    tst resume_stage
    breq RESET_NORMAL

    ; Etapas validas:
    ;   1..5   = reanudacion del comando T
    ;   10..14 = terminar retorno a HOME de comandos individuales
    cpi resume_stage, 6
    brlo RESET_RESUME_T

    cpi resume_stage, STAGE_SINGLE_TRIANGLE
    brlo RESET_INVALID_STAGE

    cpi resume_stage, 15
    brlo RESET_RESUME_SINGLE

RESET_INVALID_STAGE:
    ; EEPROM virgen (0xFF) u otro valor invalido.
    clr dato
    rcall EEPROM_WRITE_STAGE
    clr resume_stage
    rjmp RESET_NORMAL


RESET_RESUME_T:
    ldi auto_mode, 1

    cpi resume_stage, STAGE_AFTER_TRIANGLE
    brne RESET_CHECK_STAGE_2
    jmp T_RESUME_AFTER_TRIANGLE

RESET_CHECK_STAGE_2:
    cpi resume_stage, STAGE_AFTER_CIRCLE
    brne RESET_CHECK_STAGE_3
    jmp T_RESUME_AFTER_CIRCLE

RESET_CHECK_STAGE_3:
    cpi resume_stage, STAGE_AFTER_STAR
    brne RESET_CHECK_STAGE_4
    jmp T_RESUME_AFTER_STAR

RESET_CHECK_STAGE_4:
    cpi resume_stage, STAGE_AFTER_CUBE
    brne RESET_CHECK_STAGE_5
    jmp T_RESUME_AFTER_CUBE

RESET_CHECK_STAGE_5:
    jmp T_RESUME_AFTER_PORYGON


RESET_RESUME_SINGLE:
    ; El lapiz ya quedo arriba luego del reset.
    ; No queremos guardar otra etapa durante el retorno XY.
    clr auto_mode

    cpi resume_stage, STAGE_SINGLE_TRIANGLE
    brne RESET_SINGLE_CHECK_CIRCLE
    jmp SINGLE_RESUME_TRIANGLE

RESET_SINGLE_CHECK_CIRCLE:
    cpi resume_stage, STAGE_SINGLE_CIRCLE
    brne RESET_SINGLE_CHECK_STAR
    jmp SINGLE_RESUME_CIRCLE

RESET_SINGLE_CHECK_STAR:
    cpi resume_stage, STAGE_SINGLE_STAR
    brne RESET_SINGLE_CHECK_CUBE
    jmp SINGLE_RESUME_STAR

RESET_SINGLE_CHECK_CUBE:
    cpi resume_stage, STAGE_SINGLE_CUBE
    brne RESET_SINGLE_CHECK_PORYGON
    jmp SINGLE_RESUME_CUBE

RESET_SINGLE_CHECK_PORYGON:
    jmp SINGLE_RESUME_PORYGON


RESET_NORMAL:
    ldi ZL, low(MSG_WELCOME<<1)
    ldi ZH, high(MSG_WELCOME<<1)
    rcall UART_PUTS

; MENU
SHOW_MENU:
    ldi ZL, low(MSG_MENU<<1)
    ldi ZH, high(MSG_MENU<<1)
    rcall UART_PUTS

WAIT_COMMAND:
    rcall UART_RX

    ; Ignorar Enter
    cpi cmd, 13
    breq WAIT_COMMAND
    cpi cmd, 10
    breq WAIT_COMMAND

    ; Eco
    mov dato, cmd
    rcall UART_TX
    rcall UART_NEWLINE

    ; Seleccion de comando
    ; Sin comandos de prueba U/B.
    cpi cmd, '1'
    brne CHECK_CMD_2
    rjmp CMD_TRIANGLE

CHECK_CMD_2:
    cpi cmd, '2'
    brne CHECK_CMD_3
    rjmp CMD_CIRCLE

CHECK_CMD_3:
    cpi cmd, '3'
    brne CHECK_CMD_4
    rjmp CMD_STAR

CHECK_CMD_4:
    cpi cmd, '4'
    brne CHECK_CMD_P
    rjmp CMD_CUBE

CHECK_CMD_P:
    cpi cmd, 'P'
    breq JUMP_CMD_PORYGON
    cpi cmd, 'p'
    brne CHECK_CMD_T

JUMP_CMD_PORYGON:
    rjmp CMD_PORYGON

CHECK_CMD_T:
    cpi cmd, 'T'
    breq JUMP_CMD_ALL
    cpi cmd, 't'
    brne COMMAND_INVALID

JUMP_CMD_ALL:
    rjmp CMD_ALL

COMMAND_INVALID:
    ldi ZL, low(MSG_ERROR<<1)
    ldi ZH, high(MSG_ERROR<<1)
    rcall UART_PUTS
    rjmp SHOW_MENU

CMD_TRIANGLE:
    ldi auto_mode, 1
    ldi next_stage, STAGE_SINGLE_TRIANGLE
    rcall PLOT_TRIANGLE_SLOT
    rcall CLEAR_PERSIST_STATE
    rjmp DRAW_FINISHED

CMD_CIRCLE:
    ldi auto_mode, 1
    ldi next_stage, STAGE_SINGLE_CIRCLE
    rcall PLOT_CIRCLE_SLOT
    rcall CLEAR_PERSIST_STATE
    rjmp DRAW_FINISHED

CMD_STAR:
    ldi auto_mode, 1
    ldi next_stage, STAGE_SINGLE_STAR
    rcall PLOT_STAR_SLOT
    rcall CLEAR_PERSIST_STATE
    rjmp DRAW_FINISHED

CMD_CUBE:
    ldi auto_mode, 1
    ldi next_stage, STAGE_SINGLE_CUBE
    rcall PLOT_CUBE_SLOT
    rcall CLEAR_PERSIST_STATE
    rjmp DRAW_FINISHED

CMD_PORYGON:
    ldi auto_mode, 1
    ldi next_stage, STAGE_SINGLE_PORYGON
    rcall PLOT_PORYGON_SLOT
    rcall CLEAR_PERSIST_STATE
    rjmp DRAW_FINISHED

CMD_ALL:
    ; T puede atravesar varios RESET; entra al motor persistente.
    jmp DRAW_ALL_A4

DRAW_FINISHED:
    ldi ZL, low(MSG_DONE<<1)
    ldi ZH, high(MSG_DONE<<1)
    rcall UART_PUTS
    rjmp SHOW_MENU

; EEPROM - ETAPA DE REANUDACION DEL COMANDO T
; dato = valor leido / valor a escribir
EEPROM_WAIT_READY:
    sbic EECR, EEPE
    rjmp EEPROM_WAIT_READY
    ret


EEPROM_READ_STAGE:
    rcall EEPROM_WAIT_READY

    clr temp
    out EEARH, temp
    ldi temp, EE_STAGE_ADDR
    out EEARL, temp

    sbi EECR, EERE
    in dato, EEDR
    ret


EEPROM_WRITE_STAGE:
    ; dato = etapa
    rcall EEPROM_WAIT_READY

    clr temp
    out EEARH, temp
    ldi temp, EE_STAGE_ADDR
    out EEARL, temp
    out EEDR, dato

    ; Secuencia atomica obligatoria EEMPE -> EEPE.
    in sreg_save, SREG
    cli
    sbi EECR, EEMPE
    sbi EECR, EEPE
    out SREG, sreg_save

    ; Esperar que termine antes de energizar PEN_UP.
    rcall EEPROM_WAIT_READY
    ret

; Limpiar estado persistente cuando el cabezal ya regreso a HOME.
CLEAR_PERSIST_STATE:
    clr dato
    rcall EEPROM_WRITE_STAGE

    clr auto_mode
    clr next_stage
    clr resume_stage
    ret

UART_INIT:
    clr temp
    sts UCSR0A, temp

    ldi temp, high(103)
    sts UBRR0H, temp
    ldi temp, low(103)
    sts UBRR0L, temp

    ldi temp, (1<<RXEN0) | (1<<TXEN0)
    sts UCSR0B, temp

    ldi temp, (1<<UCSZ01) | (1<<UCSZ00)
    sts UCSR0C, temp
    ret

UART_RX:
UART_RX_WAIT:
    lds temp, UCSR0A
    sbrs temp, RXC0
    rjmp UART_RX_WAIT
    lds cmd, UDR0
    ret

UART_TX:
UART_TX_WAIT:
    lds temp, UCSR0A
    sbrs temp, UDRE0
    rjmp UART_TX_WAIT
    sts UDR0, dato
    ret

UART_NEWLINE:
    ldi dato, 13
    rcall UART_TX
    ldi dato, 10
    rcall UART_TX
    ret

UART_PUTS:
UART_PUTS_LOOP:
    lpm dato, Z+
    tst dato
    breq UART_PUTS_END
    rcall UART_TX
    rjmp UART_PUTS_LOOP

UART_PUTS_END:
    ret
