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
	
	;Comienzo de estados
STAND_BY:
    clr   carga
    ldi   data_portD, (1<<LED_LISTO)
    out   PORTD, data_portD
    clr   data_portB
    out   PORTB, data_portB

SB_ESPERA_SELECCION:
    sbic  PINC, PIN_SELECCION      ; si el pulsador de selección NO está presionado, salta
    rjmp  SB_CHEQUEA_INICIO

    rcall debounce_20ms
    sbic  PINC, PIN_SELECCION      ; confirma que sigue presionado (evita rebote)
    rjmp  SB_CHEQUEA_INICIO

    inc   carga
    cpi   carga, 3
    brne  SB_ACTUALIZA_LED
    clr   carga

SB_ACTUALIZA_LED:
    rcall actualiza_leds_fase_carga

SB_ESPERA_SUELTA:                  ; espera a que se suelte el pulsador (anti-repique)
    sbis  PINC, PIN_SELECCION
    rjmp  SB_ESPERA_SUELTA

SB_CHEQUEA_INICIO:
    sbic  PINC, PIN_INICIO         ; si el pulsador de inicio NO está presionado, vuelve a esperar
    rjmp  SB_ESPERA_SELECCION

    sbic  PINC, PIN_PUERTA         ; exige puerta cerrada (Sensor puerta = 0 -> cerrada, activo bajo)
    rjmp  SB_ESPERA_SELECCION

    rcall debounce_20ms
    rjmp  LLENADO
 
LLENADO:
    sbr   data_portB, (1<<VALVULA)
    out   PORTB, data_portB

LL_ESPERA_SENSOR:
    sbic  PINC, PIN_LLENADO         ; espera hasta que el sensor de llenado esté activo (pin = 0)
    rjmp  LL_ESPERA_SENSOR

    cbr   data_portB, (1<<VALVULA)   ; cierra la válvula: ya no hace falta seguir llenando
    out   PORTB, data_portB
    rjmp  LAVADO

LAVADO:
    ldi   temp, (1<<LED_LAVADO)
    rcall fija_led_fase             ; prende LED de fase (Lavado) + LED de carga
    ldi   ciclo_lav, 5

LAV_CICLO:
    
    sbr   data_portB, (1<<MOTOR_DER)
    out   PORTB, data_portB
    ldi   segundos, 2
    add   segundos, carga            ; segundos = 2 + Carga
    rcall delay_segundos

   
    cbr   data_portB, (1<<MOTOR_DER)
    out   PORTB, data_portB
    ldi   segundos, 1
    add   segundos, carga            ; segundos = 1 + Carga
    rcall delay_segundos

    dec   ciclo_lav
    brne  LAV_CICLO                  ; repite hasta completar 5 ciclos

	rjmp CENTRIFUGADO

; Ciclo de centrifugado
CENTRIFUGADO:
	ldi temp, (1<<LED_CENTRIF)
	rcall fija_led_fase

	sbr data_portB, (1<<MOTOR_DER)
	out PORTB, data_portB

	ldi segundos, 15 ; Cargo los 15 seg iniciales 
	add segundos, carga 
	add segundos, carga
	add segundos, carga ; Ecuacion para el tiempo en funcion de la carga seleccionada "segundos= 15+3 * carga"
	rcall delay_segundos

	cbr data_portB, (1<<MOTOR_DER)
	out PORTB, data_portB
	rjmp SECADO_DERECHA

SECADO_DERECHA:
	ldi temp, (1<<LED_SECADO)
	rcall fija_led_fase
	sbr   data_portB, (1<<MOTOR_DER)
    out   PORTB, data_portB
    ldi   segundos, 5
    add   segundos, carga
    add   segundos, carga             ; segundos = 5 + 2*Carga
    rcall delay_segundos
    cbr   data_portB, (1<<MOTOR_DER)
    out   PORTB, data_portB

SECADO_ESPERA:
    ; motor detenido (pausa entre giro derecha e izquierda)
    ldi   segundos, 3
    add   segundos, carga
    add   segundos, carga             ; segundos = 3 + 2*Carga
    rcall delay_segundos
	
SECADO_IZQUIERDA:
    sbr   data_portB, (1<<MOTOR_IZQ)
    out   PORTB, data_portB
    ldi   segundos, 5
    add   segundos, carga
    add   segundos, carga             ; segundos = 5 + 2*Carga
    rcall delay_segundos
    cbr   data_portB, (1<<MOTOR_IZQ)
    out   PORTB, data_portB
    rjmp  FIN_PROCESO

FIN_PROCESO:
    ldi   data_portD, (1<<LED_FIN)     ; el resto de LEDs de PORTD en 0
    out   PORTD, data_portD
    clr   data_portB                ; LEDs de carga + actuadores en 0
    out   PORTB, data_portB

    ldi   segundos, 2
    rcall delay_segundos

    rjmp  STAND_BY

; SUBRUTINA: fija led de fase y carga
fija_led_fase:
    mov   data_portD, temp             ; solo el LED de fase queda prendido en PORTD

    cpi   carga, 0
    breq  FLF_LIGERA
    cpi   carga, 1
    breq  FLF_MEDIA

    ; --- pesada: LED en PORTB (D9) ---
    out   PORTD, data_portD
    sbr   data_portB, (1<<LED_C_PESADA)
    cbr   data_portB, (1<<LED_C_MEDIA)
    out   PORTB, data_portB
    ret
FLF_LIGERA:
    ; --- ligera: LED en PORTD (D7) ---
    ori   data_portD, (1<<LED_C_LIGERA)
    out   PORTD, data_portD
    ret
FLF_MEDIA:
    ; --- media: LED en PORTB (D8) ---
    out   PORTD, data_portD
    sbr   data_portB, (1<<LED_C_MEDIA)
    cbr   data_portB, (1<<LED_C_PESADA)
    out   PORTB, data_portB
    ret

; Actualiza LEDs de fase y carga
actualiza_leds_fase_carga:
    ldi   data_portD, (1<<LED_LISTO)
    clr   data_portB

    cpi   carga, 0
    breq  ALF_LIGERA
    cpi   carga, 1
    breq  ALF_MEDIA
    sbr   data_portB, (1<<LED_C_PESADA)
    rjmp  ALF_SALIDA
ALF_LIGERA:
    ori   data_portD, (1<<LED_C_LIGERA)
    rjmp  ALF_SALIDA
ALF_MEDIA:
    sbr   data_portB, (1<<LED_C_MEDIA)
ALF_SALIDA:
    out   PORTD, data_portD
    out   PORTB, data_portB
    ret

; SUBRUTINA: debounce_20ms  (retardo corto anti-rebote)
debounce_20ms: 
   push  r22
    push  r23
    ldi   r22, 40
DB1:
    ldi   r23, 200
DB2:
    dec   r23
    brne  DB2
    dec   r22
    brne  DB1
    pop   r23
    pop   r22
    ret

delay_segundos:
    push  segundos	
DS_LOOP:
    cpi   segundos, 0
    breq  DS_FIN
    rcall delay_1s
    dec   segundos
    rjmp  DS_LOOP
DS_FIN:
    pop   segundos
    ret

delay_1s:
    push  r24
    push  r25
    push  r26
    ldi   r26, 82       
DL1:
    ldi   r25, 250      
DL2:
    ldi   r24, 250        
DL3:
    dec   r24
    brne  DL3
    dec   r25
    brne  DL2
    dec   r26
    brne  DL1
    pop   r26
    pop   r25
    pop   r24
    ret
